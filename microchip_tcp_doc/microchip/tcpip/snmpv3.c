/*******************************************************************************

  Summary:
    
  Description:
*******************************************************************************/

/*******************************************************************************
FileName:  SNMPv3.c 
Copyright © 2012 released Microchip Technology Inc.  All rights
reserved.

Microchip licenses to you the right to use, modify, copy and distribute
Software only when embedded on a Microchip microcontroller or digital signal
controller that is integrated into your product or third party product
(pursuant to the sublicense terms in the accompanying license agreement).

You should refer to the license agreement accompanying this Software for
additional information regarding your rights and obligations.

SOFTWARE AND DOCUMENTATION ARE PROVIDED “AS IS” WITHOUT WARRANTY OF ANY KIND,
EITHER EXPRESS OR IMPLIED, INCLUDING WITHOUT LIMITATION, ANY WARRANTY OF
MERCHANTABILITY, TITLE, NON-INFRINGEMENT AND FITNESS FOR A PARTICULAR PURPOSE.
IN NO EVENT SHALL MICROCHIP OR ITS LICENSORS BE LIABLE OR OBLIGATED UNDER
CONTRACT, NEGLIGENCE, STRICT LIABILITY, CONTRIBUTION, BREACH OF WARRANTY, OR
OTHER LEGAL EQUITABLE THEORY ANY DIRECT OR INDIRECT DAMAGES OR EXPENSES
INCLUDING BUT NOT LIMITED TO ANY INCIDENTAL, SPECIAL, INDIRECT, PUNITIVE OR
CONSEQUENTIAL DAMAGES, LOST PROFITS OR LOST DATA, COST OF PROCUREMENT OF
SUBSTITUTE GOODS, TECHNOLOGY, SERVICES, OR ANY CLAIMS BY THIRD PARTIES
(INCLUDING BUT NOT LIMITED TO ANY DEFENSE THEREOF), OR OTHER SIMILAR COSTS.
*******************************************************************************/

#include "tcpip_private.h"
#include "tcpip_config_private.h"

#if defined(TCPIP_STACK_USE_SNMPV3_SERVER)
#include "tcpip/snmpv3.h"


typedef struct 
{
	uint8_t privAndAuthFlag:2;
    uint8_t reportableFlag :1;
}snmpV3MsgFlagsBitWise;


/*   
SNMP_ENGINE_MAX_MSG_SIZE is determined as the minimum of the max msg size
values supported among all of the transports available to and supported by the engine. 
*/
#define SNMP_ENGINE_MAX_MSG_SIZE	1024 
#define SET_SNMPV3_GET_PDU_INDEX  (gSNMPv3ScopedPduRequestBuf.length++)


/*header+MSGID +msgMAXSIZE +msg flag +security model */
#define MSGGLOBAL_HEADER_LEN(x)	( x= (2 \
							  +1+1+4 \
							  +1+1+4 \
							  +1+1+1  \
							  +1+1+1)\
						    )
/*header+ engineID+ engine boot+ engine time+security name+authentication parameters
+privacy parameters*/
#define MSG_AUTHORITATIVE_HEADER_LEN(x)   ( x=(2+2 \
								     +1+1+snmpEngnIDLength \
								     +1+1+4 \
								     +1+1+4 \
								     +1+1+securityPrimitivesOfIncomingPdu.securityNameLength \
								     +1+1+snmpOutMsgAuthParamLen \
								     +1+1+snmpOutMsgPrivParamLen) \
								)


/* Snmp Engine Group */
uint8_t snmpEngineID[32]; //Reserving 32 bytes for the snmpEngineID as the octet string length can vary form 5 to 32 
uint32_t snmpEngineBoots;//The number of times that the SNMP engine has (re-)initialized itself since snmpEngineID was last configured.
TCPIP_UINT32_VAL snmpEngineTime;//The number of seconds since the value of the snmpEngineBoots object last changed
TCPIP_UINT32_VAL snmpEngineMaxMessageSize;


uint8_t snmpEngnIDLength=0;
uint32_t snmpEngineTimeOffset=0;
TCPIP_UINT32_VAL authoritativeSnmpEngineBoots;
TCPIP_UINT32_VAL authoritativeSnmpEngineTime;

uint8_t snmpInMsgAuthParamStrng[12];
uint8_t snmpInMsgAuthParamLen=12;
uint8_t snmpInMsgPrvParamStrng[8];
uint8_t snmpInMsgPrivParamLen=8;

uint8_t snmpOutMsgAuthParamStrng[12];
uint8_t snmpOutMsgAuthParamLen=12;
uint8_t snmpOutMsgPrvParamStrng[8];
uint8_t snmpOutMsgPrivParamLen=8;

uint16_t snmpMsgBufSeekPos=0;
uint16_t gSNMPv3ScopedPduDataPos=0;
uint16_t gSnmpv3UserDBIndex=0; 
TCPIP_UINT16_VAL gUsmStatsEngineID={0}; 	

uint32_t snmpEngineSecurityModel=0;//Maximum range (2^31-1), RFC3411    ///done
uint32_t snmpEngineMsgProcessModel=0;//Maximum range (2^31-1), RFC3411 ///done
uint8_t  snmpSecurityLevel=0; 
uint8_t  snmpResponseSecurityFlag=0;

TCPIP_UINT32_VAL incomingSnmpPDUmsgID;
struct dispatcherProcessPdu incomingPdu;
SecuritySysProcessIncomingMsg securityPrimitivesOfIncomingPdu;

extern APP_CONFIG AppConfig;
extern void SaveAppConfig(void);
extern uint16_t SNMPTxOffset;
extern uint16_t msgSecrtyParamLenOffset;

SNMPV3MSGDATA gSNMPv3ScopedPduRequestBuf ={NULL,0,0,0};
SNMPV3MSGDATA gSNMPv3ScopedPduResponseBuf = {NULL,0,0,0};
SNMPV3MSGDATA gSNMPv3PduHeaderBuf = {NULL,0,0,0};
SNMPV3MSGDATA gSNMPv3TrapMsgHeaderBuf = {NULL,0,0,0};
SNMPV3MSGDATA gSNMPv3TrapScopedPduResponseBuf = {NULL,0,0,0};

SNMPV3_REQUEST_WHOLEMSG gSnmpV3InPduWholeMsgBuf={NULL,NULL,{0},{0},0,0,0,0};
SNMPV3_RESPONSE_WHOLEMSG gSnmpV3OUTPduWholeMsgBuf={NULL,NULL,{0},{0},0,0,0,0};
SNMPV3_RESPONSE_WHOLEMSG gSnmpV3TrapOUTPduWholeMsgBuf={NULL,NULL,{0},{0},0,0,0,0};


#if defined(SNMP_V1_V2_TRAP_WITH_SNMPV3)
//snmv3 global database for trap table
snmpV3TrapConfigDataBase gSnmpv3TrapConfigData[SNMPV3_USM_MAX_USER]=
	{ \
	{"microchip",SNMPV3_MSG_PROCESSING_MODEL,SNMPV3_USM_SECURITY_MODEL,AUTH_PRIV}, \
	{"SnmpAdmin",SNMPV3_MSG_PROCESSING_MODEL,SNMPV3_USM_SECURITY_MODEL,AUTH_NO_PRIV}, \
	{"root",SNMPV3_MSG_PROCESSING_MODEL,SNMPV3_USM_SECURITY_MODEL,NO_AUTH_NO_PRIV} \
	};
#endif

bool Snmpv3IsValidInt(uint32_t* val);
bool IsSNMPv3ValidStructure(uint16_t* dataLen);
static uint8_t FindOIDsFromSnmpV3Request(uint16_t pdulen);

void Snmpv3ReportPdu(SNMPV3MSGDATA *dynScopedBufPtr);
void Snmpv3FreeDynAllocMem(void);
static bool IsSnmpv3ValidOID(uint8_t* oid, uint8_t* len);
static bool IsSnmpV3ASNNull(void);
void Snmpv3SetErrorStatus(uint16_t errorStatusOffset,
                           uint16_t errorIndexOffset,
                           SNMP_ERR_STATUS errorStatus,
                           uint8_t errorIndex,SNMPV3MSGDATA *dynScopedPduPutBuf);
uint8_t Snmpv3IsValidAuthStructure(uint16_t* dataLen);
bool _Snmpv3IsValidInt(uint8_t* wholeMsgPtr,uint16_t* pos, uint32_t* val );
bool _IsSNMPv3ValidStructure(uint8_t* wholeMsgPtr,uint16_t* pos, uint16_t* dataLen );
uint8_t _Snmpv3IsValidAuthStructure(uint16_t* dataLen);



/****************************************************************************
  Function:
	uint8_t snmpConfigureEngineSecurityModel(uint32_t securityModelInRequest)
	
  Summary:
  	Updates the Snmp agent security model value.

  Description:
	Checks for the security model value the in the request pdu against the security 
	model value specified in the RFC3411 max range.
  	Updates the snmp agent global variable 'snmpEngineSecurityModel' for storing 
  	security model value reuqested in the incoming request from the SNMP managers.
	  	 		 		  	
  Precondition:
   	SNMPInit(); is called. The agent is recognised in the network as the SNMPv3 node.
   	Snmp request is received.
		
  Parameters:
  	securityModelInRequest : this value is populated from the incoming request PDUs.
  	
  Return Values:
	true:	If the incoming PDU has valid security model value.
	false:    If the incoming PDU has invalid or un supported security model.

  Remarks:
  	'snmpEngineSecurityModel' is value to uniquely identiy a security model of the 
  	security sub system whithin the SNMP management architecture. This value is 
  	used by the SNMP engine to send respond or inform PDUs to the other SNMP nodes.
  	'snmpEngineSecurityModel' value is used for interoperability. 
***************************************************************************/
uint8_t snmpConfigureEngineSecurityModel(uint32_t securityModelInRequest)
{
	if(securityModelInRequest < 0x80000000  )
	{
		
		if(securityModelInRequest > ANY_SECUTIRY_MODEL && 
		   securityModelInRequest <= SNMPV3_USM_SECURITY_MODEL
		   /* && User can add the Enterprise specific Security Model if reuired and is implemented */ )
		{
			snmpEngineSecurityModel=securityModelInRequest;
			return true;
		}
		else
			return false;
	}
	else
		return false;
}


/****************************************************************************
  Function:
	uint8_t snmpEngineDecodeMessageFlags(uint8_t msgFlasgInRequest)
	
  Summary:
  	Returns the message flag value in the received incoming snmp request pdu. 

  Description:
	Compare the message flag value the in the request pdu against the Octet's 
	least significant three bits: Reportable, PrivFlag, AuthFlag. Return the message flag
	value to message processing model to take corresponding decision for the 
	message processing flow.  
		  	 		 		  	
  Precondition:
   	SNMPInit(); is called. The agent is recognised in the network as the SNMPv3 node.
	Snmp request is received.	

  Parameters:
  	'msgFlasgInRequest' is value populated from the incoming request PDUs.
  	
  Return Values:
	INVALID_MSG: If the incoming PDU has undefined message flag value  or 
	REPORT_FLAG_AND_SECURITY_LEVEL_FLAGS: if not INVALID_MSG, then 
	any other matching value in the enum except INVALID_MSG.

  Remarks:
  	'msgFlasgInRequest' is compared to possible combinations of snmp message flags.
  	The return value from this routine is utilised by the message processing unit to 
  	decide on the course of action to be taken while processing the incoming request. 
***************************************************************************/

uint8_t snmpEngineDecodeMessageFlags(uint8_t msgFlasgInRequest)
{
	if(msgFlasgInRequest > REPORT2REQ_PRIVACY_AND_AUTH_PROVIDED) /*if all msgFlags are SET*/
	{
		return INVALID_MSG;
	}
	else if( (msgFlasgInRequest & NO_REPORT_NO_PRIVACY_NO_AUTH) == NO_REPORT_NO_PRIVACY_NO_AUTH)
	{
		return NO_REPORT_NO_PRIVACY_NO_AUTH;
	}
	else if( (msgFlasgInRequest & NO_REPORT_NO_PRIVACY_BUT_AUTH_PROVIDED) == NO_REPORT_NO_PRIVACY_BUT_AUTH_PROVIDED)
	{
		return NO_REPORT_NO_PRIVACY_BUT_AUTH_PROVIDED;
	}
	else if( (msgFlasgInRequest & NO_REPORT_PRIVACY_PROVIDED_BUT_NO_AUTH) == NO_REPORT_PRIVACY_PROVIDED_BUT_NO_AUTH)
	{
		return INVALID_MSG;
	}
	else if( (msgFlasgInRequest & NO_REPORT_PRIVACY_AND_AUTH_PROVIDED) == NO_REPORT_PRIVACY_AND_AUTH_PROVIDED)
	{
		return NO_REPORT_PRIVACY_AND_AUTH_PROVIDED;
	}

	else if( (msgFlasgInRequest & REPORT2REQ_NO_PRIVACY_NO_AUTH) == REPORT2REQ_NO_PRIVACY_NO_AUTH)
	{
		return REPORT2REQ_NO_PRIVACY_NO_AUTH;
	}
	else if( (msgFlasgInRequest & REPORT2REQ_NO_PRIVACY_BUT_AUTH_PROVIDED) == REPORT2REQ_NO_PRIVACY_BUT_AUTH_PROVIDED)
	{
		return REPORT2REQ_NO_PRIVACY_BUT_AUTH_PROVIDED;
	}
	else if( (msgFlasgInRequest & REPORT2REQ_PRIVACY_PROVIDED_BUT_NO_AUTH) == REPORT2REQ_PRIVACY_PROVIDED_BUT_NO_AUTH)
	{
		return INVALID_MSG;
	}
	else if( (msgFlasgInRequest & REPORT2REQ_PRIVACY_AND_AUTH_PROVIDED) == REPORT2REQ_PRIVACY_AND_AUTH_PROVIDED)
	{
		return REPORT2REQ_PRIVACY_AND_AUTH_PROVIDED;
	}
	else
		return INVALID_MSG;
}



/****************************************************************************
  Function:
	uint32_t Snmpv3TrackAuthEngineTimeTick(void)
	
  Summary:
  	Returns the internal tick timer value to be used by 'snmpEngineTime'.
	
  Description:
  	
    This routine reads the internal system tick and converts it to tenths of milliseconds.
  	
  Precondition:
   	None

  Parameters:
  	None
  	
  Return Values:
	timeStamp : uint32_t value of the timer ticks 

  Remarks:
	None
***************************************************************************/
uint32_t Snmpv3TrackAuthEngineTimeTick(void)
{

   
    SYS_TICK    timeStamp;

    // convert the current system tick to 10 ms units
    timeStamp = (SYS_TICK_Get() * 100ull)/SYS_TICK_TicksPerSecondGet();

    return timeStamp;

}


/****************************************************************************
  Function:
	void Snmpv3FormulateEngineID(uint8_t fifthOctectIdentifier )
	
  Summary:
  	Formulates the snmpEngineID for the SNMPV3 engine.

  Description:
  	Formulates the snmpEngineID depending on value of  'fifthOctectIdentifier'.
	as MAC_ADDR_ENGN_ID using the application MAC address. 
  	'fifthOctectIdentifier' defualt set to MAC_ADDR_ENGN_ID as the following octets 
  	used for the snmpEngineID are of mac address.

	User can set this octet of their choice to fomulate new snmpEngineID.
	fifthOctectIdentifier=IPV4_ADDR_ENGN_ID; 
	
	If 
	fifthOctectIdentifier=ADMIN_ASSIGNED_TEXT;  or
	fifthOctectIdentifier=ADMIN_ASSIGNED_OCTETS;
	then the following octets should be provided by the administrator through some 
	custom application interface mechanism.
	API parameter 'fifthOctectIdentifier' has to be upated in the intefrace API before 
	passing through Snmpv3FormulateEngineID().  
  	 		 		  	
  Precondition:
   	InitAppConfig(); is called. 
		
  Parameters:
  	fifthOctectIdentifier : Value of the 5th octet in the snmpEngineID which indicates 
  	how the rest (6th and following octets) are formatted.
  	
  Return Values:
	None

  Remarks:
	Authentication and encryption keys are generated using corresponding passwords and 
	snmpEngineID. If the snmpEngineID is newly configured, then the auth and privacy keys 
	would also change. Hence while using this API to change the snmpEngineID dynamically,
	care should be taken to update the new localized keys at the agent as well as at the manager.
***************************************************************************/
void Snmpv3FormulateEngineID(uint8_t fifthOctectIdentifier )
{

uint8_t* dummyptr;
unsigned int i;

//Modify this private enterprise assigned number to your organization number asssigned by IANA 
TCPIP_UINT32_VAL mchpPvtEntpriseAssignedNumber; 

mchpPvtEntpriseAssignedNumber.Val = 0x42C7; //microchip = 17095.


	//Set the first bit as '1b' .  Refer to RFC3411 section5 Page# 41	
	mchpPvtEntpriseAssignedNumber.Val = ((0x80000000) | mchpPvtEntpriseAssignedNumber.Val);

	snmpEngineID[0]=mchpPvtEntpriseAssignedNumber.v[3];
	snmpEngineID[1]=mchpPvtEntpriseAssignedNumber.v[2];
	snmpEngineID[2]=mchpPvtEntpriseAssignedNumber.v[1];
	snmpEngineID[3]=mchpPvtEntpriseAssignedNumber.v[0];

	//Refer to RFC3411 section5 Page# 41	
	fifthOctectIdentifier=MAC_ADDR_ENGN_ID;
	
	snmpEngineID[4]= fifthOctectIdentifier;

	if(fifthOctectIdentifier == MAC_ADDR_ENGN_ID)
	{
		for(i=0;i<6/*sizeof(MAC_ADDR)*/;i++)
		{
			snmpEngineID[5+i]=AppConfig.MyMACAddr.v[i];
		}
		
		snmpEngineID[5+6/*sizeof(MAC_ADDR)*/]='\0';
		snmpEngnIDLength=4/* 4 Bytes of IANA Pvt Enterprise Assigned Number*/
						+1/* 1 Byte for fifthOctectIdentifier*/
						+6/*sizeof(MAC_ADDR)*/;
	}
	else if(fifthOctectIdentifier == IPV4_ADDR_ENGN_ID)
	{
		dummyptr= (uint8_t*)strncpy((char *)&snmpEngineID[5], (const char *) &AppConfig.MyMACAddr, sizeof(IP_ADDR));
	
		snmpEngineID[5+4/*sizeof(IP_ADDR)*/]='\0';
		snmpEngnIDLength=4/* 4 Bytes of IANA Pvt Enterprise Assigned Number*/
						+1/* 1 Byte for fifthOctectIdentifier*/
						+4 /*sizeof(IP_ADDR)*/;
	}
	else if((fifthOctectIdentifier == ADMIN_ASSIGNED_TEXT )||(fifthOctectIdentifier == ADMIN_ASSIGNED_OCTETS))
	{

		//Interface API updates the  snmpEngineID[4] = fifthOctectIdentifier 
		//and snmpEngineID[5] onwords with the corresponding octet string or value.
		;
		//snmpEngnIDLength=strlen((const char*) snmpEngineID);
	}
		
	
	//Increment the snmpEngineBoots record as snmpEngineID reconfigured
	snmpEngineBoots+=1;

	//Increment the snmpEngineBootRcrd to be stored in the non volatile memory 
	AppConfig.SnmpEngineBootRcrd=snmpEngineBoots;

	//Store the new incremented boot record to the non volatile memory
//	SaveAppConfig();

	//Reset the snmEngineTime as snmpEngineBoots incremented
	snmpEngineTime.Val=0;

	snmpEngineTimeOffset=Snmpv3TrackAuthEngineTimeTick();
}




/****************************************************************************
  Function:
	void Snmpv3MaintainEngineBootsRcrd(void)
	
  Summary:
  	Updates the snmp engine boots counter for the SNMPV3 engine.

  Description:
  	Updates the global snmpEngineBoots counter variable with the boots counter stored in the 
  	non volatile memory. Also increments and stores the new incremented value  to the non 
  	volatile memoryafter the SNMP engine initialization at the power cycle.
  	 		 		  	
  Precondition:
   	InitAppConfig(); is called.	

  Parameters:
  	None
  	
  Return Values:
	None

  Remarks:
	Should be called only during tcp/ip stack initialization and only after the InitAppConfig();
	is called.
***************************************************************************/
void Snmpv3MaintainEngineBootsRcrd(void)
{

	//Increment the snmpEngineBootRcrd as due to power cycle snmp egine reinitialization happened 
	//to be stored in the non volatile memory 
	AppConfig.SnmpEngineBootRcrd+=1;

	//Assgined this counter value to Global cntr which will track the snmp engine boot in real time.
	//'snmpEnigineBoots' can incremement in case of 'snmpEngineTime' overflow or adminstrator configuring  
	//'snmpEngineID' with new 'fifthOctectIdentifier' through administrative interface.
	snmpEngineBoots=AppConfig.SnmpEngineBootRcrd;

	//Store the new incremented boot record to the non volatile memory
	//SaveAppConfig();


}



/****************************************************************************
  Function:
	void Snmpv3GetAuthEngineTime(void)
	
  Summary:
  	Updates the snmp engine time variable 'snmpEngineTime' for the SNMPV3 engine. 
	
  Description:
  	'snmpEngineTime' is used for Timeliness checking for Message level security. Snmp 
  	engine keep updating the ''snmpEngineTime' variable for checking the time window 
  	for the requrest and responses/inform etc. This routine also updates snmpEngineBoots
  	in scenarios of internal timer reset or 'snmpEngineTime' cntr ovrflowed 
  	the (2^31 -1) value specified in RFC3411. 
  	 		 		  	
  Precondition:
   	SNMPInit(); is called.	

  Parameters:
  	None
  	
  Return Values:
	None

  Remarks:
	This routine is called every time the rx/tx PDU processing is handled  by the SNMP agent.
	Updates the 'snmpEngineTime' and requires frequnet access to internal timer registers. 
***************************************************************************/
void Snmpv3GetAuthEngineTime(void)
{
	snmpEngineTime.Val=Snmpv3TrackAuthEngineTimeTick()-snmpEngineTimeOffset;

	if((Snmpv3TrackAuthEngineTimeTick() < snmpEngineTimeOffset)/* Internal Timer Reset occured*/
	   ||(snmpEngineTime.Val > 2147483647 /* (2^31 -1) Refer RFC 3411 Section 5 */))
	{
		/*This means the snmpEngineTime cntr ovrflowed the (2^31 -1) value 
		    or the internal Tick timer Reset occured*/
		snmpEngineTime.Val=0;
		snmpEngineTimeOffset=0;

		//Increment the snmpEngineBoots counter
		Snmpv3MaintainEngineBootsRcrd();
		
	}


}




/****************************************************************************
  Function:
	uint8_t Snmpv3NegotiateEngineMaxMsgSize(uint32_t maxMsgSizeInRequest)
	
  Summary:
  	Snmp engine max pdu message size is updated for the pdu recipient from this
  	Snmp agent.
	
  Description:
  	This routine defines the maximum size PDU that could be generated or received 
  	from this SNMP agent. The maximum size is limited to the maximum pdu size the
  	recipient can receive and process or originator can sent. For this SNMP engine, 
  	maximum message size is limited between 484 and 'SNMP_ENGINE_MAX_MSG_SIZE'. 
   	SNMP_ENGINE_MAX_MSG_SIZE is determined as the minimum of the maximum 
   	message size values supported among all of the transports available to and supported 
   	by the engine. 
   	
   Precondition:
   	The SNMP engine should have received a message from the other Snmp node 
   	notifying the maximum message size they can receive and process or send. 
 

  Parameters:
  	maxMsgSizeInRequest: incoming SNMPv3 request max msg size value
  	
  Return Values:
	true:  If the incoming message size is in the predefined range
	false: If the incoming maximum messgae size is less than 484 bytes 

  Remarks:
	SNMP_ENGINE_MAX_MSG_SIZE should not be more than 0x80000000 (2^31 -1).
	
***************************************************************************/
uint8_t Snmpv3NegotiateEngineMaxMsgSize(uint32_t maxMsgSizeInRequest)
{

	snmpEngineMaxMessageSize.Val=SNMP_ENGINE_MAX_MSG_SIZE;//"The maximum length in octets of an SNMP message ranges 484 to (2^31-1), send or receive and process.RFC3411


	if(maxMsgSizeInRequest > 0x80000000 || maxMsgSizeInRequest< 484)

		return false;

	else if(maxMsgSizeInRequest < SNMP_ENGINE_MAX_MSG_SIZE ) 
	{
		snmpEngineMaxMessageSize.Val=maxMsgSizeInRequest;
		return true;
	}
	else
		snmpEngineMaxMessageSize.Val=SNMP_ENGINE_MAX_MSG_SIZE;
		return true;
}


/****************************************************************************
  Function:
	bool Snmpv3BufferPut(uint8_t val ,SNMPV3MSGDATA *putbuf)
	
  Summary:
  	Copies uint8_t data to dynamically allocated memory buffer.

  Description:
	The SNMPv3 stack implementation uses dynamically allocated memory buffer for
	processing of request and response packets. This routine copies the uint8_t data to the 
	allocated buffer and updates the offset length couter. 
		  	 		 		  	
  Precondition:
	The SNMPv3 stack has sucessfully allocated dynamic memory buffer from the Heap
   	
  Parameters:
  	val: uint8_t value to be written to the buffer
  	putbuf: pointer to the dynamically allocated buffer to which the 'val' to be written 
  	
  Return Values:
	true: if successfully write to the buffer
	false: failure in writing to the buffer
	
  Remarks:
  	This routine is used by the SNMPv3 stack. If required to be used by the application
  	code, valid pointers should be passed to this routine. 
  	
***************************************************************************/
bool Snmpv3BufferPut(uint8_t val ,SNMPV3MSGDATA *putbuf)
{
	if(putbuf->maxlength > putbuf->length)
	{
		putbuf->head[putbuf->length] = (uint8_t)val;
		putbuf->length++;
		return true;
	}
	else
	{
		return false;
	}

}

/****************************************************************************
  Function:
	bool Snmpv3GetBufferData(SNMPV3MSGDATA getbuf,uint16_t pos)
	
  Summary:
  	Reads uint8_t data from dynamically allocated memory buffer.

  Description:
	The SNMPv3 stack implementation uses dynamically allocated memory buffer for
	processing of request and response packets. This routine reads the uint8_t data from 
	the allocated buffer at the positions (offset) provided.
		  	 		 		  	
  Precondition:
	The SNMPv3 stack has sucessfully allocated dynamic memory buffer from the Heap
   	
  Parameters:
  	getbuf: Structure from where to read the data byte.
  	pos: position in the buffer from which the data to be read 
  	
  Return Values:
	uint8_t: 1 byte value read
	
  Remarks:
  	The read position offset is required to be provided every time the routine is called.
  	This API do not increment the buffer read offset automatically, everytime it is called. 
  	
***************************************************************************/	
uint8_t Snmpv3GetBufferData(SNMPV3MSGDATA getbuf,uint16_t pos)
{
	return (uint8_t)(getbuf.head[pos]);
}


/****************************************************************************
  Function:
	uint8_t Snmpv3GetWholeMsgBufferData(uint8_t* getbuf, uint16_t* pos)
	
  Summary:
  	Reads uint8_t data from dynamically allocated memory buffer.

  Description:
	The SNMPv3 stack implementation uses dynamically allocated memory buffer for
	processing of request and response packets. This routine reads the uint8_t data from 
	the allocated buffer at the positions (offset) provided.
		  	 		 		  	
  Precondition:
	The SNMPv3 stack has sucessfully allocated dynamic memory buffer from the Heap
   	
  Parameters:
  	getbuf: pointer to the buffer where the request or repsonse data is stored.
  	pos: pointer to the position in the buffer from which the data to be read 
  	
  Return Values:
	uint8_t: 1 byte value read
	
  Remarks:
  	This API increments the read offset every time it is called. Hence useful in the 
  	consecutive reads.  
  	
***************************************************************************/	
uint8_t Snmpv3GetWholeMsgBufferData(uint8_t* getbuf, uint16_t* pos)
{
uint8_t* testPtr;
uint16_t* posPtr;

	posPtr=pos; 
	testPtr=getbuf+*posPtr;
	
	*pos=(*posPtr+1);
	return (uint8_t)*(testPtr);
}


/****************************************************************************
  Function: 
  	SNMP_ACTION Snmpv3MsgProcessingModelProcessPDU(uint8_t inOutPdu)

  Summary:
  	This routine collects or populates the message processing model infomation 
  	from the received SNMPv3 request PDU or to the response PDU respectively.
  
  Description:
	The recievd SNMPv3 PDU or the transmit PDU header has message processing
	data bytes infomration. This routine retrievs the messgae processing model 
	infomration from the stored pdu or write the appropriate msg proc info to the 
	repsonse msg buffer.
		  	 		 		  	
  Precondition:
	Valid SNMPv3 request msg is received.
   	
  Parameters:
  	inOutPdu: indicates whether the incomig PDU is to be read for msg proc values 
  	to be retrieved or the response PDU is to be populated with these values 
  	
  Return Values:
	SNMP_NO_CREATION: Failure due to improper msg processing information format in
					      the received PDU or failure in constructing the response PDU.
	SNMP_NO_ERR: The message processing infomration retrieval or response PDU 
				   fomration is successful
				   
  Remarks:
  	The messgae processing model parameters like 'msgID', 'msgMaxSize', 'msgFlags' and 
  	'msgSecurityModel' decides the SNMPv3 engine processing modalities regarding 
  	request or response PDU  	
***************************************************************************/
SNMP_ACTION Snmpv3MsgProcessingModelProcessPDU(uint8_t inOutPdu)
{
	TCPIP_UINT32_VAL tempLen={0};
	uint8_t tempData=0;	
	uint8_t *ptr=NULL;
	uint16_t tempPos=0,snmpv3Headerlength=0;
	uint8_t snmpv3MsgGlobalHeaderlength=0;
	uint8_t snmpv3MsgAuthHedaerLength = 0;	
	  
	if(inOutPdu == SNMP_REQUEST_PDU)
	{
		tempPos=snmpMsgBufSeekPos=0x00;
		if (!_IsSNMPv3ValidStructure(gSnmpV3InPduWholeMsgBuf.snmpMsgHead,&tempPos,(uint16_t*)&tempLen.w[0]))
			return SNMP_NO_CREATION;
		
		//Check and collect "msgID"
		if(! _Snmpv3IsValidInt(gSnmpV3InPduWholeMsgBuf.snmpMsgHead,&tempPos,&tempLen.Val))
			return SNMP_NO_CREATION;	

		incomingSnmpPDUmsgID.Val= tempLen.Val;

		//Check and collect "msgMaxSize" 
		if(! _Snmpv3IsValidInt(gSnmpV3InPduWholeMsgBuf.snmpMsgHead,&tempPos,&tempLen.Val))
			return SNMP_NO_CREATION;	
		
		incomingPdu.maxSizeResponseScopedPDU.Val = tempLen.Val;

		if(Snmpv3NegotiateEngineMaxMsgSize(tempLen.Val)==false)
			return SNMP_NO_CREATION; 	

		//Check and collect "msgFlags" 
		tempData = Snmpv3GetWholeMsgBufferData(gSnmpV3InPduWholeMsgBuf.snmpMsgHead,&tempPos);
		
		if ( !IS_OCTET_STRING(tempData) )
			return false;

		tempData=Snmpv3GetWholeMsgBufferData(gSnmpV3InPduWholeMsgBuf.snmpMsgHead,&tempPos);//Length byte of "msgFlags"
		tempData=Snmpv3GetWholeMsgBufferData(gSnmpV3InPduWholeMsgBuf.snmpMsgHead,&tempPos);//"msgFlag"

		snmpSecurityLevel=tempData;
		incomingPdu.securityLevel=snmpSecurityLevel;
		securityPrimitivesOfIncomingPdu.securityLevel=snmpSecurityLevel;

		//Check and collect "msgSecurityModel"	
		if(! _Snmpv3IsValidInt(gSnmpV3InPduWholeMsgBuf.snmpMsgHead,&tempPos,(uint32_t*)&tempLen.Val))
			return SNMP_NO_CREATION;	

		if(snmpConfigureEngineSecurityModel(tempLen.Val))
		{
			incomingPdu.securityModel= tempLen.Val;
			securityPrimitivesOfIncomingPdu.securityModel= (uint8_t)tempLen.Val;
		}
		else
			return SNMP_NO_CREATION;

		snmpMsgBufSeekPos=tempPos;	
	}
	else if(inOutPdu == SNMP_RESPONSE_PDU)
	{
		bool retBuf=true;

		snmpv3Headerlength = MSGGLOBAL_HEADER_LEN(snmpv3MsgGlobalHeaderlength)+
							  MSG_AUTHORITATIVE_HEADER_LEN(snmpv3MsgAuthHedaerLength);

		ptr = gSNMPv3PduHeaderBuf.head = (uint8_t *)(malloc((size_t)snmpv3Headerlength+5));
		if(ptr == NULL)
			return SNMP_NO_CREATION;
		
		gSNMPv3PduHeaderBuf.length = 0;
		gSNMPv3PduHeaderBuf.maxlength = snmpv3Headerlength+1;

		//message header
		Snmpv3BufferPut(STRUCTURE,&gSNMPv3PduHeaderBuf);
		Snmpv3BufferPut(MSGGLOBAL_HEADER_LEN(snmpv3MsgGlobalHeaderlength)-2,&gSNMPv3PduHeaderBuf);

		//Put "msgID" type ASN_INT of length 4 bytes	
		Snmpv3BufferPut(ASN_INT,&gSNMPv3PduHeaderBuf);		
		Snmpv3BufferPut(0x04,&gSNMPv3PduHeaderBuf);
		Snmpv3BufferPut(incomingSnmpPDUmsgID.v[3],&gSNMPv3PduHeaderBuf);
		Snmpv3BufferPut(incomingSnmpPDUmsgID.v[2],&gSNMPv3PduHeaderBuf);
		Snmpv3BufferPut(incomingSnmpPDUmsgID.v[1],&gSNMPv3PduHeaderBuf);
		Snmpv3BufferPut(incomingSnmpPDUmsgID.v[0],&gSNMPv3PduHeaderBuf);

		//Put "msgMaxSize"  type ASN_INT of length 4 bytes
		Snmpv3BufferPut(ASN_INT,&gSNMPv3PduHeaderBuf);
		Snmpv3BufferPut(0x04,&gSNMPv3PduHeaderBuf);
		Snmpv3BufferPut(snmpEngineMaxMessageSize.v[3],&gSNMPv3PduHeaderBuf);
		Snmpv3BufferPut(snmpEngineMaxMessageSize.v[2],&gSNMPv3PduHeaderBuf);
		Snmpv3BufferPut(snmpEngineMaxMessageSize.v[1],&gSNMPv3PduHeaderBuf);
		Snmpv3BufferPut(snmpEngineMaxMessageSize.v[0],&gSNMPv3PduHeaderBuf);

		//Put "msgFlags"  type octet_string 
		Snmpv3BufferPut(OCTET_STRING,&gSNMPv3PduHeaderBuf);
		Snmpv3BufferPut(0x01,&gSNMPv3PduHeaderBuf);

		if((snmpSecurityLevel & 0x03)==0x03) // Rxed pkt is authenticated and encrypted
		{
			snmpResponseSecurityFlag=0x03;
			Snmpv3BufferPut(snmpResponseSecurityFlag,&gSNMPv3PduHeaderBuf);//Rsponse "msgFlags" value as Reportable, Encrypted and Authenticated Bits are not set. 
		}
		else if ((snmpSecurityLevel & 0x01)==0x01) // Rxed pkt is  authenticated and no priv
		{
			snmpResponseSecurityFlag=0x01;
			Snmpv3BufferPut(snmpResponseSecurityFlag,&gSNMPv3PduHeaderBuf);//Rsponse "msgFlags" value as Reportable, Encrypted and Authenticated Bits are not set. 
		}
		else
		{			
			snmpResponseSecurityFlag=0x00;
			Snmpv3BufferPut(snmpResponseSecurityFlag,&gSNMPv3PduHeaderBuf);
		}
		//Put "msgSecurityModel"	
		Snmpv3BufferPut(ASN_INT,&gSNMPv3PduHeaderBuf);
		Snmpv3BufferPut(0x01,&gSNMPv3PduHeaderBuf);
		retBuf = Snmpv3BufferPut(snmpEngineSecurityModel,&gSNMPv3PduHeaderBuf);
		if(retBuf != true)
			return SNMP_NO_CREATION;
	}
	return SNMP_NO_ERR;
}






void Snmpv3UsmAuthoritativeEngnKeyLocalization(void)
{
static HASH_SUM md5;
uint8_t *cp, password_buf[64];
uint32_t password_index = 0;
uint32_t count = 0, i;
uint8_t localizedAuthKey[16];
	
	MD5Initialize(&md5);

	while (count < 1048576)
	{
		cp = password_buf;
		for (i = 0; i < 64; i++) 
		{
			*cp++ = snmpInMsgAuthParamStrng[password_index++ % 8];
		}


		MD5AddData(&md5, password_buf, 64);
		count+=64;
	}
	MD5Calculate(&md5, localizedAuthKey);
	memcpy(password_buf, localizedAuthKey, 16);
	memcpy(password_buf+16, snmpEngineID, snmpEngnIDLength);
	memcpy(password_buf+16+snmpEngnIDLength, localizedAuthKey, 16);

	MD5Initialize(&md5);
	MD5AddData(&md5,password_buf,32+snmpEngnIDLength);
		
	MD5Calculate(&md5, localizedAuthKey);
	count+=64;
	return;
}



/****************************************************************************
  Function: 
  	SNMP_ACTION Snmpv3UserSecurityModelProcessPDU(uint8_t inOutPdu)

  Summary:
  	This routine collects or populates the security model parametrs infomation 
  	from the received SNMPv3 request PDU or to the response PDU respectively.
  
  Description:
	The recievd SNMPv3 PDU or the transmit PDU header has message security
	data bytes infomration. This routine retrievs the messgae security parameters
	infomration from the stored incoming pdu or write the appropriate security
	model info to the repsonse msg buffer.
		  	 		 		  	
  Precondition:
	Valid SNMPv3 request msg is received.
   	
  Parameters:
  	inOutPdu: indicates whether the incomig PDU is to be read for user security
  	model to be retrieved or the response PDU to be populated with these values 
  	
  Return Values:
	SNMP_NO_CREATION: Failure due to improper security model processing information 
					     format in the received PDU or failure in constructing the response PDU.
	SNMP_NO_ERR: The user security model retrieval or response PDU fomration is successful
				   
  Remarks:
  	The user security parameter constitute the vital information for the message  
	authentication and privacy of the message.
  	The user security model parameters header structure
	MsgAuthEngnID+MsgAuthEngnBoots+MsgAuthEngnTime
	+MsgUserName+MsgAuthParam+MsgPrivParam
***************************************************************************/
SNMP_ACTION Snmpv3UserSecurityModelProcessPDU(uint8_t inOutPdu)
{
	TCPIP_UINT32_VAL tempLen={0};
	uint8_t* ptr=NULL;	
	uint8_t engnIdCntr=0;	
	uint8_t tempData=0,putCntr=0;
	uint16_t tempPos;

	tempPos=snmpMsgBufSeekPos;

	if(inOutPdu == SNMP_REQUEST_PDU)
	{
	
		tempData = Snmpv3GetWholeMsgBufferData(gSnmpV3InPduWholeMsgBuf.snmpMsgHead,&tempPos);
		 if ( !IS_OCTET_STRING(tempData) )
		    return SNMP_NO_CREATION;
		 
		//Msg security Parameter length
		tempLen.Val=Snmpv3GetWholeMsgBufferData(gSnmpV3InPduWholeMsgBuf.snmpMsgHead,&tempPos);	
			
		 //Start collecting the security parameters from the incoming PDU.
		 //Check if the security parametrs are binded in ASN structure format 
		if (!_IsSNMPv3ValidStructure(gSnmpV3InPduWholeMsgBuf.snmpMsgHead,&tempPos,(uint16_t*)&tempLen.w[0]))
			return SNMP_NO_CREATION;

		 //Collect "msgAuthoritiveEngineID" 
		tempData=Snmpv3GetWholeMsgBufferData(gSnmpV3InPduWholeMsgBuf.snmpMsgHead,&tempPos);	
		 if ( !IS_OCTET_STRING(tempData) )
		    return false;
		 
		tempData=Snmpv3GetWholeMsgBufferData(gSnmpV3InPduWholeMsgBuf.snmpMsgHead,&tempPos);	

		securityPrimitivesOfIncomingPdu.securityEngineIDLen=tempData;
		if(tempData == 0x00)
		{
			securityPrimitivesOfIncomingPdu.securityEngineID = NULL;
		}
		else
		{
			securityPrimitivesOfIncomingPdu.securityEngineID=ptr=(uint8_t *)malloc((size_t)tempData+5);
			if(securityPrimitivesOfIncomingPdu.securityEngineID == NULL)
				return SNMP_NO_CREATION;
			while( tempData--)
			{
		       *ptr++ = Snmpv3GetWholeMsgBufferData(gSnmpV3InPduWholeMsgBuf.snmpMsgHead,&tempPos);	
			}	
		}

		//Check and collect "msgAuthoritiveEngineBoots"	
		if(! _Snmpv3IsValidInt(gSnmpV3InPduWholeMsgBuf.snmpMsgHead,&tempPos,&tempLen.Val))
		return SNMP_NO_CREATION; 

		authoritativeSnmpEngineBoots.Val= tempLen.Val;

		//Check and collect "msgAuthoritiveEngineTime"	
			 if(! _Snmpv3IsValidInt(gSnmpV3InPduWholeMsgBuf.snmpMsgHead,&tempPos,&tempLen.Val))
			return SNMP_NO_CREATION;	
		 
		authoritativeSnmpEngineTime.Val= tempLen.Val;


		//Collect "msgUserName"	
		tempData = Snmpv3GetWholeMsgBufferData(gSnmpV3InPduWholeMsgBuf.snmpMsgHead,&tempPos);	
		 if ( !IS_OCTET_STRING(tempData) )
		    return false;
		 
		tempData = Snmpv3GetWholeMsgBufferData(gSnmpV3InPduWholeMsgBuf.snmpMsgHead,&tempPos);	
		securityPrimitivesOfIncomingPdu.securityNameLength=tempData;

		if(tempData == 0x00)
		{
			securityPrimitivesOfIncomingPdu.securityName = NULL;
		}
		else
		{
			securityPrimitivesOfIncomingPdu.securityName=ptr=(uint8_t *)malloc((size_t)tempData+5);
			if(securityPrimitivesOfIncomingPdu.securityName == NULL)
				return SNMP_NO_CREATION;
			while( tempData--)
			{
			   *ptr++ = Snmpv3GetWholeMsgBufferData(gSnmpV3InPduWholeMsgBuf.snmpMsgHead,&tempPos);	
			}	
		
		}


		//validate user security name with security level
		if(!Snmpv3ValidateSecNameAndSecLvl())
			return SNMP_NO_CREATION;
		
		//Validate if the "msgAuthoritiveEngineID" matches to this agent's SNMP Engine ID
		if(!Snmpv3ValidateEngineId())
			return SNMP_NO_CREATION; 

		//Check and collect "msgAuthenticationParameters"	
		tempData = Snmpv3GetWholeMsgBufferData(gSnmpV3InPduWholeMsgBuf.snmpMsgHead,&tempPos);	
		 if ( !IS_OCTET_STRING(tempData) )
		    return false;

		 //(snmpInMsgAuthParamLen should be 12 bytes if using HAMC-MD5-96 or HMAC-SHA-96)
		snmpInMsgAuthParamLen=tempData= Snmpv3GetWholeMsgBufferData(gSnmpV3InPduWholeMsgBuf.snmpMsgHead,&tempPos);	

		if((snmpSecurityLevel&0x01)==0x01)//If message is authenticated
		if(snmpInMsgAuthParamLen !=12 /* if using HAMC-MD5-96 or HMAC-SHA-96 */)
			return SNMP_NO_CREATION;
		
		if(tempData != 0x00)
		{
			ptr=snmpInMsgAuthParamStrng;
			
			gSnmpV3InPduWholeMsgBuf.msgAuthParamOffsetInWholeMsg=tempPos;//From snmpMsgHead;
			while( tempData--)
			{
			   *ptr++= Snmpv3GetWholeMsgBufferData(gSnmpV3InPduWholeMsgBuf.snmpMsgHead,&tempPos);	
			   
			}	
		}

		
		//Check and collect "msgPrivacyParameters"	
		tempData = Snmpv3GetWholeMsgBufferData(gSnmpV3InPduWholeMsgBuf.snmpMsgHead,&tempPos);	
		 if ( !IS_OCTET_STRING(tempData) )
		    return false;

		//(snmpInMsgPrivParamLen should be 8 bytes) 
		snmpInMsgPrivParamLen=tempData =Snmpv3GetWholeMsgBufferData(gSnmpV3InPduWholeMsgBuf.snmpMsgHead,&tempPos);	
		if((snmpSecurityLevel&0x02)==0x02)//If message is encrypted
		if(snmpInMsgPrivParamLen !=8)
			return SNMP_NO_CREATION;
		
		if(tempData != 0x00)
		{
			ptr=snmpInMsgPrvParamStrng;
			
			while( tempData--)
			{
			   *ptr++ = Snmpv3GetWholeMsgBufferData(gSnmpV3InPduWholeMsgBuf.snmpMsgHead,&tempPos);	
			}	


			//This is a secured request. Compute the AES decryption IV 
			Snmpv3UsmAesEncryptDecryptInitVector(SNMP_REQUEST_PDU);
		}

		/* global variable to find out how many times SNMPv3 engine id has been validated*/
		gUsmStatsEngineID.Val++;
		snmpMsgBufSeekPos=tempPos;
		
	}
	else if(inOutPdu == SNMP_RESPONSE_PDU)
	{
		uint16_t snmpv3MsgAuthHeaderLength=0;
		bool   retBuf=true;
		uint16_t msgHeaderOffset1=0;
		uint16_t msgHeaderOffset2=0;
		uint16_t tempMsgHeaderOffset=0;
		
		Snmpv3BufferPut(OCTET_STRING,&gSNMPv3PduHeaderBuf);  //Security Parameter string 	
		msgHeaderOffset1 = gSNMPv3PduHeaderBuf.length;
		Snmpv3BufferPut(MSG_AUTHORITATIVE_HEADER_LEN(snmpv3MsgAuthHeaderLength)-2,&gSNMPv3PduHeaderBuf);
		Snmpv3BufferPut(STRUCTURE,&gSNMPv3PduHeaderBuf);
		msgHeaderOffset2 = gSNMPv3PduHeaderBuf.length;
		Snmpv3BufferPut(MSG_AUTHORITATIVE_HEADER_LEN(snmpv3MsgAuthHeaderLength)-4,&gSNMPv3PduHeaderBuf);
		
		//Put "msgAuthoritiveEngineID"	
		Snmpv3BufferPut(OCTET_STRING,&gSNMPv3PduHeaderBuf);	
		Snmpv3BufferPut(snmpEngnIDLength,&gSNMPv3PduHeaderBuf); //Integer Length
		tempData=snmpEngnIDLength;
		for(;engnIdCntr<tempData;engnIdCntr++)
		{
			Snmpv3BufferPut(snmpEngineID[engnIdCntr],&gSNMPv3PduHeaderBuf);
		}
		
		//Put "msgAuthoritiveEngineBoots" 
		Snmpv3BufferPut(ASN_INT,&gSNMPv3PduHeaderBuf);	
		Snmpv3BufferPut(0x04,&gSNMPv3PduHeaderBuf);
		Snmpv3BufferPut(snmpEngineBoots>>24,&gSNMPv3PduHeaderBuf);
		Snmpv3BufferPut(snmpEngineBoots>>16,&gSNMPv3PduHeaderBuf);
		Snmpv3BufferPut(snmpEngineBoots>>8,&gSNMPv3PduHeaderBuf);
		Snmpv3BufferPut(snmpEngineBoots,&gSNMPv3PduHeaderBuf);

		//Put "msgAuthoritiveEngineTime" 
		Snmpv3GetAuthEngineTime();
		Snmpv3BufferPut(ASN_INT,&gSNMPv3PduHeaderBuf);	
		Snmpv3BufferPut(0x04,&gSNMPv3PduHeaderBuf); 
		Snmpv3BufferPut(snmpEngineTime.v[3],&gSNMPv3PduHeaderBuf);
		Snmpv3BufferPut(snmpEngineTime.v[2],&gSNMPv3PduHeaderBuf);
		Snmpv3BufferPut(snmpEngineTime.v[1],&gSNMPv3PduHeaderBuf);
		Snmpv3BufferPut(snmpEngineTime.v[0],&gSNMPv3PduHeaderBuf);


		//Put "msgUserName"	
		Snmpv3BufferPut(OCTET_STRING,&gSNMPv3PduHeaderBuf);	
		Snmpv3BufferPut(securityPrimitivesOfIncomingPdu.securityNameLength,&gSNMPv3PduHeaderBuf);
		tempData=securityPrimitivesOfIncomingPdu.securityNameLength ;
		if(securityPrimitivesOfIncomingPdu.securityNameLength != 0)
		{
			ptr= securityPrimitivesOfIncomingPdu.securityName;
			for(putCntr=0;putCntr<tempData;putCntr++)
			{
				//Snmpv3BufferPut(ptr[putCntr],&gSNMPv3PduHeaderBuf);
				Snmpv3BufferPut(*(ptr+putCntr),&gSNMPv3PduHeaderBuf);
			}
		}

		putCntr = 0;
		
		//Snmpv3UsmOutMsgAuthenticationParam(SNMPV3_HAMC_MD5/*Hash Accoding to user*/);
		//Put "msgAuthenticationParameters"	
		Snmpv3BufferPut(OCTET_STRING,&gSNMPv3PduHeaderBuf);
		if((snmpSecurityLevel & 0x01) == 0x01)
		{
			Snmpv3BufferPut(snmpOutMsgAuthParamLen,&gSNMPv3PduHeaderBuf); //Not supported with the Alpha Release.

			gSNMPv3PduHeaderBuf.msgAuthParamOffset=gSNMPv3PduHeaderBuf.length;

			//Put 0x00 to msgAuthenticationParameters, Once the response WholeMsg is 
			//populated, this offset can be updated with the new msgAuthenticationParam
			for(;putCntr<snmpOutMsgAuthParamLen;putCntr++)
			Snmpv3BufferPut(0x00,&gSNMPv3PduHeaderBuf);//RFC3414 Section 6.3.2 Page#56 Step3
		}
		else
		{
			Snmpv3BufferPut(0x00,&gSNMPv3PduHeaderBuf); //Not supported with the Alpha Release.
		}
		putCntr = 0;
		Snmpv3USMOutMsgPrivParam();
		
		//Put "msgPrivacyParameters" 
		Snmpv3BufferPut(OCTET_STRING,&gSNMPv3PduHeaderBuf); 
		if((snmpSecurityLevel & 0x02) == 0x02)
		{
			Snmpv3BufferPut(snmpOutMsgPrivParamLen,&gSNMPv3PduHeaderBuf); //Not supported with the Alpha Release
			for(;putCntr<snmpOutMsgPrivParamLen;putCntr++)
				retBuf = Snmpv3BufferPut(snmpOutMsgPrvParamStrng[putCntr],&gSNMPv3PduHeaderBuf);
		}
		else
		{
			Snmpv3BufferPut(0x00,&gSNMPv3PduHeaderBuf); //Not supported with the Alpha Release.
		}
		if(retBuf != true)
			return SNMP_NO_CREATION;
		tempMsgHeaderOffset = gSNMPv3PduHeaderBuf.length;
		gSNMPv3PduHeaderBuf.length = msgHeaderOffset2;
		Snmpv3BufferPut((tempMsgHeaderOffset-msgHeaderOffset2)-1,&gSNMPv3PduHeaderBuf); 
		gSNMPv3PduHeaderBuf.length = tempMsgHeaderOffset;

		tempMsgHeaderOffset = gSNMPv3PduHeaderBuf.length;
		gSNMPv3PduHeaderBuf.length = msgHeaderOffset1;
		Snmpv3BufferPut((tempMsgHeaderOffset-msgHeaderOffset1)-1,&gSNMPv3PduHeaderBuf);
		gSNMPv3PduHeaderBuf.length = tempMsgHeaderOffset;
	}
	return SNMP_NO_ERR;
}


/****************************************************************************
  Function: 
  	SNMP_ACTION Snmpv3ScopedPduProcessing(uint8_t inOutPdu)

  Summary:
  	This routine collects  the scoped pdu header information from the 
  	received SNMPv3 request PDU or populates to the response PDU respectively.
  
  Description:
	The recievd SNMPv3 PDU or the transmit PDU header has scoped pdu parameters
	like 'contextEngineID' 'context name' etc. This routine retrievs these parameters
	infomration from the stored incoming pdu or write the appropriate dynamically 
	allocated memory for the transmit response PDU.
		  	 		 		  	
  Precondition:
	Valid SNMPv3 request msg is received.
   	
  Parameters:
  	inOutPdu: indicates whether the incomig PDU is to be read for scoped pdu
  	paraemters to be retrieved or the response PDU to be populated with these values 
  	
  Return Values:
	SNMP_NO_CREATION: Failure due to improper scoped pdu information format in the  
					     PDU or failure in constructing the response PDU.
	SNMP_NO_ERR: The scoped parameters retrieval or response PDU fomration 
				   is successful
				   
  Remarks:
  	The scoped pDu parameters 
	msg data : - <contextEngineID><context name>[<data> == <pdutype><request id>
	<error status><error index><varbinds>
***************************************************************************/
SNMP_ACTION Snmpv3ScopedPduProcessing(uint8_t inOutPdu)
{
	TCPIP_UINT32_VAL tempLen={0};
	uint8_t* ptr=NULL;
	uint16_t msgDataLen=0;
	SNMPV3MSGDATA scopedPtr={NULL,0,0,0};

	uint16_t 	contextIDlen=0;
	uint16_t	contextNameLength=0;
	uint16_t	snmpv3Headerlength=0,snmpv3MsgGlobalHeaderlength=0;
	uint16_t	snmpv3MsgAuthHedaerLength=0;

	if(inOutPdu == SNMP_REQUEST_PDU)
	{
		if ( !_Snmpv3IsValidAuthStructure((uint16_t*)&tempLen) )
		{
			return SNMP_NO_CREATION;
		}

		gSnmpV3InPduWholeMsgBuf.scopedPduOffset=snmpMsgBufSeekPos;
		

		gSNMPv3ScopedPduRequestBuf.head=gSnmpV3InPduWholeMsgBuf.snmpMsgHead+snmpMsgBufSeekPos;
		gSNMPv3ScopedPduRequestBuf.length = 0;
		gSNMPv3ScopedPduRequestBuf.maxlength = tempLen.Val+1;
	}
	else if(inOutPdu == SNMP_RESPONSE_PDU)
	{		
			
		snmpv3Headerlength = MSGGLOBAL_HEADER_LEN(snmpv3MsgGlobalHeaderlength)+
									  MSG_AUTHORITATIVE_HEADER_LEN(snmpv3MsgAuthHedaerLength);
		
		msgDataLen = SNMP_MAX_MSG_SIZE - snmpv3Headerlength;
		ptr = gSNMPv3ScopedPduResponseBuf.head =(uint8_t*)(malloc((size_t)msgDataLen+5));
		if(ptr == NULL)
		{
			return SNMP_NO_CREATION;
		}
		gSNMPv3ScopedPduResponseBuf.length = 0;
		gSNMPv3ScopedPduResponseBuf.maxlength = msgDataLen;
		gSNMPv3ScopedPduDataPos = 0;


    	//Start collecting the plaint text Scoped PDU data byte from the WholeMsg buffer
		//Check if the plain text scoped pdu data bytes are binded in ASN structure format 
		scopedPtr = gSNMPv3ScopedPduResponseBuf;
		Snmpv3BufferPut(STRUCTURE,&scopedPtr); // First item to Response buffer is packet structure
		Snmpv3BufferPut(0x82,&scopedPtr); 
		Snmpv3BufferPut(0,&scopedPtr);
		Snmpv3BufferPut(0,&scopedPtr);

		if((snmpSecurityLevel & 0x02)==0x02)
		{
			contextIDlen=Snmpv3GetBufferData(gSNMPv3ScopedPduRequestBuf,gSNMPv3ScopedPduDataPos++) ;
			contextIDlen=Snmpv3GetBufferData(gSNMPv3ScopedPduRequestBuf,gSNMPv3ScopedPduDataPos++) ;
			if(contextIDlen == 0x81)
			{
				Snmpv3GetBufferData(gSNMPv3ScopedPduRequestBuf,gSNMPv3ScopedPduDataPos++) ;
			}
			else if(contextIDlen == 0x82)
			{
				Snmpv3GetBufferData(gSNMPv3ScopedPduRequestBuf,gSNMPv3ScopedPduDataPos++) ;
				Snmpv3GetBufferData(gSNMPv3ScopedPduRequestBuf,gSNMPv3ScopedPduDataPos++) ;
			}
		}
		
		//Collect context engine id
		if (!IS_OCTET_STRING(Snmpv3GetBufferData(gSNMPv3ScopedPduRequestBuf,gSNMPv3ScopedPduDataPos) ))
		{
			return SNMP_NO_CREATION;
		}
		Snmpv3BufferPut(OCTET_STRING,&scopedPtr);

		contextIDlen = Snmpv3GetBufferData(gSNMPv3ScopedPduRequestBuf,++gSNMPv3ScopedPduDataPos);
		if(contextIDlen == 0)
		{			
			Snmpv3BufferPut(0,&scopedPtr);
		}
		else
		{
			//copy context engine id from a local buffer			
			Snmpv3BufferPut(contextIDlen,&scopedPtr);
			while(contextIDlen!=0)	
			{
				Snmpv3BufferPut(Snmpv3GetBufferData(gSNMPv3ScopedPduRequestBuf,++gSNMPv3ScopedPduDataPos),&scopedPtr);
				contextIDlen-=1;
			}
		}

		//Check and collect "contextName" 
		 if (!IS_OCTET_STRING(Snmpv3GetBufferData(gSNMPv3ScopedPduRequestBuf,++gSNMPv3ScopedPduDataPos) ))
		{
		   return SNMP_NO_CREATION;
		}
		Snmpv3BufferPut(OCTET_STRING,&scopedPtr);
		contextNameLength = (uint16_t)Snmpv3GetBufferData(gSNMPv3ScopedPduRequestBuf,++gSNMPv3ScopedPduDataPos);
		if(contextNameLength == 0x00)
		{
			Snmpv3BufferPut(0x00,&scopedPtr);
		}
		else
		{
			Snmpv3BufferPut(contextNameLength,&scopedPtr);
			while(contextNameLength!=0x00)
			{
				Snmpv3BufferPut(Snmpv3GetBufferData(gSNMPv3ScopedPduRequestBuf,++gSNMPv3ScopedPduDataPos),&scopedPtr);
				contextNameLength-=1;
			}	
		}

		gSNMPv3ScopedPduResponseBuf.length = scopedPtr.length;

	}
	
	return SNMP_NO_ERR;
}



/****************************************************************************
  Function:
	void Snmpv3FreeDynAllocMem(void)
	
  Summary:
  	Allocated dynamic memory freeing is done by this routine.
	
  Description:
  	On the successful completion of the processing of the SNMPv3 request, or the 
  	failure in the processing due to improper PDU formats, the allocated dynamic
  	memory is required to be freed. This routine calls the free(), to deallocate memory.
	
  Precondition:
	The dyanmic memory buffer is allocated.
	
  Parameters:
  	None
  	
  Return Values:
	None
	
  Remarks:
	The SNMPv3 stack does uses the dynamic memory extensively for different 
       processing needs, hence incoming and outgoing pdu memory buffers are created.
	This routine checks for the memory is being allocated before it attempts for the deallocation. 
 ***************************************************************************/
void Snmpv3FreeDynAllocMem(void)
{

	if(gSNMPv3PduHeaderBuf.head != NULL)
	{
		free(gSNMPv3PduHeaderBuf.head);
		gSNMPv3PduHeaderBuf.head=NULL;
		gSNMPv3PduHeaderBuf.length=0x00;
	}

	if(gSNMPv3ScopedPduResponseBuf.head != NULL)
	{
		free(gSNMPv3ScopedPduResponseBuf.head);
		gSNMPv3ScopedPduResponseBuf.head=NULL;
		gSNMPv3ScopedPduResponseBuf.length = 0;
	}

	if(gSnmpV3InPduWholeMsgBuf.wholeMsgHead != NULL)
	{
		free(gSnmpV3InPduWholeMsgBuf.wholeMsgHead);
		gSnmpV3InPduWholeMsgBuf.wholeMsgLen.Val = 0;
		gSnmpV3InPduWholeMsgBuf.wholeMsgHead=NULL;
		gSnmpV3InPduWholeMsgBuf.snmpMsgHead=NULL;
	}

	if(securityPrimitivesOfIncomingPdu.securityEngineID!=NULL)
	{
		free(securityPrimitivesOfIncomingPdu.securityEngineID);
		securityPrimitivesOfIncomingPdu.securityEngineID=0x00;
		securityPrimitivesOfIncomingPdu.securityEngineIDLen=0x00;
	}

	if(securityPrimitivesOfIncomingPdu.securityName != NULL)
	{ 
		free(securityPrimitivesOfIncomingPdu.securityName);
		securityPrimitivesOfIncomingPdu.securityName=NULL;
	}

	if(gSNMPv3TrapMsgHeaderBuf.head != NULL)
	{
		free(gSNMPv3TrapMsgHeaderBuf.head);
		gSNMPv3TrapMsgHeaderBuf.length=0;
        gSNMPv3TrapMsgHeaderBuf.head = NULL;
	}

	if(gSNMPv3TrapScopedPduResponseBuf.head != NULL)
	{
		free(gSNMPv3TrapScopedPduResponseBuf.head);
		gSNMPv3TrapScopedPduResponseBuf.length = 0;
		gSNMPv3TrapScopedPduResponseBuf.head = NULL;
	}
}

/****************************************************************************
  Function:
	bool ProcessSnmpv3MsgData(PDU_INFO* pduDbPtr)
	
  Summary:
  	This routine processes the snmpv3 request and parallely creates the response pdu.
	
  Description:
  	Once the received pdu is validated as Snmpv3 pdu, it is forwarded for 
  	processing to this routine. This rotuine handles Get, Get_Next, Get_Bulk,
  	Set request and creates appropriate response as Get_Response. 
  	This routine will decide on whether the request pdu should be processed
  	or be discarded. 
  	
  Precondition:
	The received udp packet is varified as valid SNMPv3 request.
		
  Parameters:
  	pduDbPtr  - Pointer to received pdu information database
  	
  Return Values:
	true 	- If the snmp request processing is successful.
	false	- If the processing failed else the processing is not completed.
	
  Remarks:
	None
 ***************************************************************************/
bool ProcessSnmpv3MsgData(PDU_INFO* pduDbPtr)
{
	uint8_t 			Getbulk_N=0,Getbulk_M=0,Getbulk_R=0;/*Refer RFC 3416 Section "4.2.3" GetBulkRequest-PDU*/
	uint8_t			OIDValue[OID_MAX_LEN];
	uint8_t			OIDlen=0;
	uint8_t			oidLookUpRet=0;
	uint8_t			noOfVarbindreq=0xFF;
	uint8_t tempBuf[4];
	uint8_t tempCntr=0;
	uint8_t* tempPtr=NULL;
	uint8_t* outBufPtr=NULL;
	uint8_t			varIndex=0;
	uint8_t 			repeatCntr=0,varBindCntr=0;
	uint8_t			succesor=0,tempRet=0xFF;
	uint16_t	 		pduLenOffset=0;
	uint16_t 	 		pduLength=0;
	uint16_t 			errorStatusOffset=0;
	uint16_t 			errorIndexOffset=0;
	uint16_t 			varBindHeaderLen=0;
	uint16_t			previousGetBufpos=0;
	uint16_t			varBindHeaderOffset=0;
	uint16_t			varBindHeaderOffset_2=0;
	uint16_t			varBindHeaderOffset_3=0;
	uint16_t			contextNameOffset=0;	
	uint16_t		 	varStructLenOffset=0;
	uint16_t			maxRepeatationOffset=0;
	uint16_t			tempOffset=0;						
	TCPIP_UINT16_VAL		varPairLen = {0x0};
	TCPIP_UINT32_VAL 		tempVal={0};
	static SNMPV3MSGDATA	*dynScopedBufPtr=NULL;
	OID_INFO		OIDInfo;  
	SNMP_ERR_STATUS errorStatus;

		
	enum 
	{
		SM_PKT_STRUCT_LEN_OFFSET=0u,
		SM_RESPONSE_PDU_LEN_OFFSET,
		SM_ERROR_STATUS_OFFSET,
		SM_ERROR_INDEX_OFFSET,
		SM_FIND_NO_OF_REQUESTED_VARBINDS,
		SM_FIND_NO_OF_RESPONSE_VARBINDS,
		SM_VARBIND_STRUCT_OFFSET,
		SM_VARSTRUCT_LEN_OFFSET,
		SM_POPULATE_REQ_OID,
		SM_FIND_OID_IN_MIB,
		SM_NON_REPETITIONS,
		SM_MAX_REPETITIONS
	}smSnmp=SM_PKT_STRUCT_LEN_OFFSET;

	dynScopedBufPtr = &gSNMPv3ScopedPduResponseBuf;
	
	while(1)
	{
		switch(smSnmp)
		{
	
			// Before each variables are processed, prepare necessary header.
				case SM_PKT_STRUCT_LEN_OFFSET:
	
				varPairLen.Val=0x0000;
				
				if(Snmpv3MsgProcessingModelProcessPDU(SNMP_RESPONSE_PDU)!= SNMP_NO_ERR)
				{
					Snmpv3FreeDynAllocMem();
					return false;
				}
				if(Snmpv3UserSecurityModelProcessPDU(SNMP_RESPONSE_PDU)!= SNMP_NO_ERR)
				{
					Snmpv3FreeDynAllocMem();
					return false;
				}
				contextNameOffset = dynScopedBufPtr->length;
				if(Snmpv3ScopedPduProcessing(SNMP_RESPONSE_PDU)!= SNMP_NO_ERR)
				{
					Snmpv3FreeDynAllocMem();
					return false;
				}
				
				dynScopedBufPtr = &gSNMPv3ScopedPduResponseBuf;					
				dynScopedBufPtr->head = gSNMPv3ScopedPduResponseBuf.head;
				dynScopedBufPtr->length = gSNMPv3ScopedPduResponseBuf.length;
				
				pduDbPtr->pduType = Snmpv3GetBufferData(gSNMPv3ScopedPduRequestBuf,++gSNMPv3ScopedPduDataPos);
				pduLength = Snmpv3GetBufferData(gSNMPv3ScopedPduRequestBuf,++gSNMPv3ScopedPduDataPos);
				if(pduLength == 0x81)
				{
					pduLength = Snmpv3GetBufferData(gSNMPv3ScopedPduRequestBuf,++gSNMPv3ScopedPduDataPos);
				}
				else if(pduLength == 0x82)
				{
					pduLength = Snmpv3GetBufferData(gSNMPv3ScopedPduRequestBuf,++gSNMPv3ScopedPduDataPos);
					pduLength = Snmpv3GetBufferData(gSNMPv3ScopedPduRequestBuf,++gSNMPv3ScopedPduDataPos);
				}					
			
				varBindHeaderOffset = dynScopedBufPtr->length;
				previousGetBufpos = gSNMPv3ScopedPduDataPos;
				if(Snmpv3GetBufferData(gSNMPv3ScopedPduRequestBuf,previousGetBufpos+0x0E)==0x0)
				{
					noOfVarbindreq = 0;
				}
				smSnmp++;
			case SM_RESPONSE_PDU_LEN_OFFSET:

				if(!noOfVarbindreq)
					Snmpv3BufferPut(REPORT_RESPONSE,dynScopedBufPtr);
				else
					Snmpv3BufferPut(GET_RESPONSE,dynScopedBufPtr);			
				// Since we don't know length of this response, use placeholders until
				pduLenOffset = dynScopedBufPtr->length;
				Snmpv3BufferPut(0x82,dynScopedBufPtr);				
				Snmpv3BufferPut(0,dynScopedBufPtr);
				Snmpv3BufferPut(0,dynScopedBufPtr);
				
				if(Snmpv3IsValidInt(&tempVal.Val) != true)
				{					
					Snmpv3FreeDynAllocMem();
					return false;
				}
				else
				{
					// Put original request back.
					Snmpv3BufferPut(ASN_INT,dynScopedBufPtr);	// Int type.
					Snmpv3BufferPut(4,dynScopedBufPtr);		// To simplify logic, always use 4 byte long requestID
					Snmpv3BufferPut(tempVal.v[3],dynScopedBufPtr); // Start MSB
					Snmpv3BufferPut(tempVal.v[2],dynScopedBufPtr);
					Snmpv3BufferPut(tempVal.v[1],dynScopedBufPtr);
					Snmpv3BufferPut(tempVal.v[0],dynScopedBufPtr);
				}

				smSnmp++;
			case SM_ERROR_STATUS_OFFSET :
				/*update pduDbPtr structure for error index and eroor status 
				and non repeators and max repeators */
				if(pduDbPtr->pduType != GET_BULK_REQUEST)
				{
					// ignore error index and error status but update pduDBptr
					tempVal.Val = 0;
					Snmpv3IsValidInt(&tempVal.Val);
					pduDbPtr->errorStatus = tempVal.Val;
					tempVal.Val = 0;
					Snmpv3IsValidInt(&tempVal.Val);
					pduDbPtr->erroIndex = tempVal.Val;
				}
				else
				{
					// update max repeators and non repeators
					tempVal.Val = 0;
					if(Snmpv3IsValidInt(&tempVal.Val) == true)
						pduDbPtr->nonRepeators = tempVal.Val;
					else
					{						
						Snmpv3FreeDynAllocMem();
						return false;
					}
					tempVal.Val = 0;
					if(Snmpv3IsValidInt(&tempVal.Val) == true)
						pduDbPtr->maxRepetitions = tempVal.Val;
					else
					{						
						Snmpv3FreeDynAllocMem();
						return false;
					}
					
				}
				
				// Put error status.
				// Since we do not know error status, put place holder until we know it...
				Snmpv3BufferPut(ASN_INT,dynScopedBufPtr);				// Int type
				Snmpv3BufferPut(1,dynScopedBufPtr);					// One byte long.
				errorStatusOffset = dynScopedBufPtr->length;
				Snmpv3BufferPut(0,dynScopedBufPtr);					// Placeholder.
				smSnmp++;
	
			case SM_ERROR_INDEX_OFFSET :
	
				// Similarly put error index.
				Snmpv3BufferPut(ASN_INT,dynScopedBufPtr);				// Int type
				Snmpv3BufferPut(1,dynScopedBufPtr);					// One byte long
				errorIndexOffset = dynScopedBufPtr->length;
				Snmpv3BufferPut(0,dynScopedBufPtr);					// Placeholder.
	
				smSnmp++;
	
			case SM_FIND_NO_OF_REQUESTED_VARBINDS:

				varBindHeaderOffset_2 = dynScopedBufPtr->length;
				// Decode variable binding structure
				if ( IsSNMPv3ValidStructure(&varBindHeaderLen) == false)
				{
					noOfVarbindreq = 0;						
				}
				else	//Find number of OIDs/varbinds's data requested in received PDU.
					noOfVarbindreq = FindOIDsFromSnmpV3Request(varBindHeaderLen);
				
				Snmpv3BufferPut(STRUCTURE,dynScopedBufPtr);
				varBindHeaderOffset_3 = dynScopedBufPtr->length;
				Snmpv3BufferPut(0x82,dynScopedBufPtr);
				Snmpv3BufferPut(0,dynScopedBufPtr);				
				Snmpv3BufferPut(0,dynScopedBufPtr);
				if(noOfVarbindreq == 0)
				{
					Snmpv3ReportPdu(dynScopedBufPtr);
					break;
				}
				smSnmp++;	
	
			case SM_FIND_NO_OF_RESPONSE_VARBINDS:
				//Calculate number of variables to be responded for the received request
				Getbulk_N = noOfVarbindreq; Getbulk_M=0; Getbulk_R=0;
				if((pduDbPtr->snmpVersion == (uint8_t)SNMP_V3) && 
					(pduDbPtr->pduType == GET_BULK_REQUEST))
				{
					if((pduDbPtr->nonRepeators) <= noOfVarbindreq)
					{
						Getbulk_N = pduDbPtr->nonRepeators;
					}	
					Getbulk_M = pduDbPtr->maxRepetitions;
	
					if((noOfVarbindreq - Getbulk_N)>=0u)
						Getbulk_R = noOfVarbindreq-Getbulk_N;
					
					noOfVarbindreq = Getbulk_N + (Getbulk_M * Getbulk_R);//Refer RFC 3416 
					
					if(Getbulk_N == 0)
					{
						smSnmp=SM_MAX_REPETITIONS;
						break;
					}
				}	
				//noOfVarbindreq = Getbulk_N + (Getbulk_M * Getbulk_R);//Refer RFC 3416 
	
				smSnmp++;
			case SM_VARSTRUCT_LEN_OFFSET:
				if(noOfVarbindreq == 0)
					break;
				
				if(Getbulk_N!= 0u) // decreament non repeators.
					Getbulk_N--;
				else if(Getbulk_M > 0) // jump to max repeatations
				{
					smSnmp = SM_MAX_REPETITIONS;
					break;
				}
				
				varIndex++;
				if ( IsSNMPv3ValidStructure(&varBindHeaderLen) == false)
				{
					Snmpv3SetErrorStatus(errorStatusOffset,errorIndexOffset,SNMP_GEN_ERR,varIndex,dynScopedBufPtr);
					goto SNMP_PROCESS_END;
				}
				smSnmp++;
			case SM_POPULATE_REQ_OID:
				for(OIDlen=0;OIDlen<sizeof(OIDValue);OIDlen++)
					OIDValue[OIDlen]=0;
				OIDlen=0;
				if(IsSnmpv3ValidOID(OIDValue,&OIDlen) == false)
				{
					Snmpv3SetErrorStatus(errorStatusOffset,errorIndexOffset,SNMP_GEN_ERR,varIndex,dynScopedBufPtr);
					goto SNMP_PROCESS_END;
				}
				
				// For Get & Get-Next, value must be NULL.
				if ( pduDbPtr->pduType != (uint8_t)SET_REQUEST )
				{
					if ( !IsSnmpV3ASNNull() )
					{						
						Snmpv3SetErrorStatus(errorStatusOffset,errorIndexOffset,SNMP_GEN_ERR,varIndex,dynScopedBufPtr);
						goto SNMP_PROCESS_END;
					}
				}
				noOfVarbindreq--;
				smSnmp++;
			
			case SM_FIND_OID_IN_MIB:
			
				/* Search for the requested OID in the MIB database with the agent.*/
				
				
				//Searching the requested OID in the MIB database 
				oidLookUpRet = OIDLookup(pduDbPtr,OIDValue, OIDlen, &OIDInfo);	
				if(Snmpv3BufferPut(STRUCTURE,dynScopedBufPtr) != true)
					goto GET_BULK_OVERFLOW_ERROR;
				varStructLenOffset= dynScopedBufPtr->length;
				if(Snmpv3BufferPut(0,dynScopedBufPtr)!= true)
					goto GET_BULK_OVERFLOW_ERROR;
		
				// ASN OID data type
				if(Snmpv3BufferPut(ASN_OID,dynScopedBufPtr)!= true)
					goto GET_BULK_OVERFLOW_ERROR;
		
				/* send the error code for SNMPv3 version for GET request and SET - request.
				As the follwing code is only for the get and set response, so Snmpv3BufferPut is not
				under the buffer over flow check.
				*/
				if(oidLookUpRet != (uint8_t)true && (pduDbPtr->pduType != GET_NEXT_REQUEST) &&
					(pduDbPtr->pduType != GET_BULK_REQUEST))
				{
					if(appendZeroToOID)
						Snmpv3BufferPut(OIDlen+1,dynScopedBufPtr);//for appending "0"
					else 
						Snmpv3BufferPut(OIDlen,dynScopedBufPtr);//do not append "0"		
					pduLength = 0;							
					//Put OID
					while( OIDlen-- )
						Snmpv3BufferPut(OIDValue[pduLength++],dynScopedBufPtr);//do not append "0"		
					
					if(appendZeroToOID)
					{
						Snmpv3BufferPut(0x00,dynScopedBufPtr);//Appending '0' to OID in response
					}
					if(( pduDbPtr->snmpVersion == (uint8_t)SNMP_V3)
							&& (pduDbPtr->pduType == SNMP_GET))
					{
						Snmpv3BufferPut(oidLookUpRet,dynScopedBufPtr);//Appending '0' to OID in response
						Snmpv3BufferPut(0x0,dynScopedBufPtr);//Appending '0' to OID in response
					}
					tempOffset = dynScopedBufPtr->length;
					pduLength = dynScopedBufPtr->length-(varStructLenOffset+1);
					dynScopedBufPtr->length = varStructLenOffset;
					Snmpv3BufferPut(pduLength,dynScopedBufPtr);
					dynScopedBufPtr->length = tempOffset;
					//Reset to state machine to access the next oid in request
					smSnmp=SM_VARSTRUCT_LEN_OFFSET;
					break;	
				}
				smSnmp++;
			
				//return false;
				
				case SM_NON_REPETITIONS:
				
					/*	Variables in get,get_next,set and get_bulk ( non repetition variables)
						of snmp request are processed in this part of the state machine.*/

					if(pduDbPtr->pduType == SNMP_SET)
					{
						uint8_t templen=OIDlen;
						uint8_t *ptroid=OIDValue;	
						//to validate the REC ID is present or not
	
						if(SNMPIdRecrdValidation(pduDbPtr,&OIDInfo,OIDValue,OIDlen) != true)
						{
							/*if(pduDbPtr->snmpVersion == (uint8_t)SNMP_V1)
					       		*errorStatus = SNMP_NO_SUCH_NAME;
							else if ((pduDbPtr->snmpVersion == (uint8_t)SNMP_V2C) ||
											(pduDbPtr->snmpVersion == (uint8_t)SNMP_V3))*/

							 /*if the variable binding's name specifies a
						     * variable which does not exist and could not ever be
						     * created, then the value of the Response-PDU's error-
						     * status field is set to `noCreation', and the value of its
						     * error-index field is set to the index of the failed
						     * variable binding.
						     */
							errorStatus = SNMP_NO_CREATION;
							
							return false;
						}
						
						if(appendZeroToOID)
							Snmpv3BufferPut(OIDlen+1,dynScopedBufPtr);//for appending "0"
						else 
							Snmpv3BufferPut(OIDlen,dynScopedBufPtr);//do not append "0"
						
						//Put OID
						while( templen-- )
							Snmpv3BufferPut(*ptroid++,dynScopedBufPtr);//do not append "0" 	
						
						if(appendZeroToOID)
						{
							Snmpv3BufferPut(0x00,dynScopedBufPtr);//Appending '0' to OID in response
						}
						//Now process the SET command
						tempRet = ProcessSetVar(pduDbPtr,&OIDInfo, &errorStatus);
				
						if ( errorStatus != SNMP_NO_ERR )
						{
							//SET var command failed. Update the error status.
							Snmpv3SetErrorStatus(errorStatusOffset, \
										   errorIndexOffset, \
										   errorStatus, \
										   varIndex,dynScopedBufPtr); \
				
						}	
						
					}/*Get-Next-rquest also calls the ProcessGetVar for 0the instance.  */
					else if((pduDbPtr->pduType == SNMP_GET) || (getZeroInstance) ||
						((pduDbPtr->pduType == SNMP_V2C_GET_BULK) && (oidLookUpRet == true)))
					{	
						uint8_t templen=OIDlen;
						uint8_t *ptroid=OIDValue;	

						//to validate the REC ID is present or not
						if(SNMPIdRecrdValidation(pduDbPtr,&OIDInfo,OIDValue,OIDlen) != true)
						{
							return false;
						}
						if(appendZeroToOID)
						{
							if(Snmpv3BufferPut(OIDlen+1,dynScopedBufPtr)!= true) //for appending "0"
								goto GET_BULK_OVERFLOW_ERROR;
						}
						else 
						{
							if(Snmpv3BufferPut(OIDlen,dynScopedBufPtr) != true)//do not append "0"
								goto GET_BULK_OVERFLOW_ERROR;
						}
						
						//Put OID
						while( templen-- )
							if(Snmpv3BufferPut(*ptroid++,dynScopedBufPtr)!= true)//do not append "0" 	
								goto GET_BULK_OVERFLOW_ERROR;
						
						if(appendZeroToOID)
						{
							if(Snmpv3BufferPut(0x00,dynScopedBufPtr)!= true)//Appending '0' to OID in response
								goto GET_BULK_OVERFLOW_ERROR;
						}

						tempRet = ProcessGetVar(&OIDInfo,false,pduDbPtr);				
					}	
					else if((pduDbPtr->pduType == SNMP_GET_NEXT)||
						((pduDbPtr->pduType == SNMP_V2C_GET_BULK) && (oidLookUpRet != true)))
					{			
						tempRet = ProcessGetNextVar(&OIDInfo,pduDbPtr);
						if(tempRet ==0)
						{
							uint8_t templen=OIDlen;
							uint8_t *ptroid=OIDValue;	
							
							if(appendZeroToOID)
							{
								if(Snmpv3BufferPut(OIDlen+1,dynScopedBufPtr) != true)//for appending "0"
									goto GET_BULK_OVERFLOW_ERROR;
							}
							else 
							{
								if(Snmpv3BufferPut(OIDlen,dynScopedBufPtr)!= true)//do not append "0"
									goto GET_BULK_OVERFLOW_ERROR;
							}
							
							//Put OID
							while( templen-- )
								if(Snmpv3BufferPut(*ptroid++,dynScopedBufPtr)!= true)//do not append "0" 
									goto GET_BULK_OVERFLOW_ERROR;
							
							if(appendZeroToOID)
							{
								if(Snmpv3BufferPut(0x00,dynScopedBufPtr)!= true)//Appending '0' to OID in response
									goto GET_BULK_OVERFLOW_ERROR;
							}
						}
					}
				
				
					/*	If the request command processing is failed, update
						the error status, index accordingly and response pdu.*/ 
					if(tempRet == 0u &&(pduDbPtr->pduType != SNMP_SET))
					{
						if(dynScopedBufPtr->length >= dynScopedBufPtr->maxlength)
							goto GET_BULK_OVERFLOW_ERROR;
						if(((pduDbPtr->pduType == SNMP_GET_NEXT)|| (pduDbPtr->pduType == SNMP_V2C_GET_BULK))&&pduDbPtr->snmpVersion == (uint8_t)SNMP_V3)
						{
							if(Snmpv3BufferPut(SNMP_END_OF_MIB_VIEW,dynScopedBufPtr)!= true)
								goto GET_BULK_OVERFLOW_ERROR;
							if(Snmpv3BufferPut(0,dynScopedBufPtr)!= true)
								goto GET_BULK_OVERFLOW_ERROR;
							//if get bulk response reaches END of mIB view break from the loop.
							noOfVarbindreq = 0;
							Getbulk_N = 0u; 					
						}
				
					}
					dynScopedBufPtr->length = gSNMPv3ScopedPduResponseBuf.length;
					tempOffset = dynScopedBufPtr->length;
					pduLength = dynScopedBufPtr->length-(varStructLenOffset+1);
					dynScopedBufPtr->length = varStructLenOffset;
					Snmpv3BufferPut(pduLength,dynScopedBufPtr);
					dynScopedBufPtr->length = tempOffset;

					/* to avoid Dynamic out buffer crash   we need to calculate buffer
					availability .Approximatly the next variable bind length should be less than 30.*/
					
					if(dynScopedBufPtr->length>= dynScopedBufPtr->maxlength)
					{
						noOfVarbindreq = 0;
						Getbulk_N = 0u;						
						break;
					}
					/*	Decide on the number of Non repetition variables remained to 
						be processed, decide the course of state machine.*/
					if((pduDbPtr->pduType==GET_BULK_REQUEST) && (pduDbPtr->snmpVersion == (uint8_t)SNMP_V3)
														&&( Getbulk_N == 0u))
					{					
						smSnmp=SM_MAX_REPETITIONS;
					}
					else
						smSnmp=SM_VARSTRUCT_LEN_OFFSET;
				
					getZeroInstance = false;
					break;
				case SM_MAX_REPETITIONS:
					maxRepeatationOffset = gSNMPv3ScopedPduDataPos;
					/*Process each variable in request as Get_Next for 
					  Getbulk_M (Max_repetition) times */
					for(repeatCntr=0;repeatCntr<Getbulk_M;repeatCntr++)
					{
						gSNMPv3ScopedPduDataPos = maxRepeatationOffset;
						
						//Process every veriable in the request.
						for(varBindCntr=0;varBindCntr<Getbulk_R;varBindCntr++)
						{
							if(noOfVarbindreq != 0)
							{
								noOfVarbindreq--;
							}
							else
								break;
							
							if(varBindCntr==0u)
								varIndex=(noOfVarbindreq - Getbulk_R);
				
							varIndex++;

							if(Snmpv3BufferPut(STRUCTURE,dynScopedBufPtr)!= true)
								goto GET_BULK_OVERFLOW_ERROR;
							varStructLenOffset= dynScopedBufPtr->length;
							if(Snmpv3BufferPut(0,dynScopedBufPtr)!= true)
								goto GET_BULK_OVERFLOW_ERROR;
							succesor=repeatCntr;
				
							// Decode variable length structure
							if(IsSNMPv3ValidStructure(&varBindHeaderLen) == false)
							{
								Snmpv3SetErrorStatus(errorStatusOffset,errorIndexOffset,SNMP_GEN_ERR,varIndex,dynScopedBufPtr);
								goto SNMP_PROCESS_END;
								//return false;
							}
				
							// Decode next object
							if ( !IsSnmpv3ValidOID(OIDValue, &OIDlen) )
							{
								Snmpv3SetErrorStatus(errorStatusOffset,errorIndexOffset,SNMP_GEN_ERR,varIndex,dynScopedBufPtr);
								goto SNMP_PROCESS_END;
								//return false;
							}
							
							// For Get & Get-Next, value must be NULL.
							if ( pduDbPtr->pduType != (uint8_t)SET_REQUEST )
								if ( !IsSnmpV3ASNNull() )
									break;
				
							oidLookUpRet = OIDLookup(pduDbPtr,OIDValue, OIDlen, &OIDInfo);
							if(oidLookUpRet == SNMP_END_OF_MIB_VIEW)
							{
								tempRet = GetNextLeaf(&OIDInfo);
							}
							if(oidLookUpRet == false)
							{
								uint8_t templen=OIDlen;
								uint8_t *ptroid=OIDValue;	
								
								if(Snmpv3BufferPut(ASN_OID,dynScopedBufPtr)!= true)
									goto GET_BULK_OVERFLOW_ERROR;
								if(appendZeroToOID)
								{
									if(Snmpv3BufferPut(OIDlen+1,dynScopedBufPtr)!= true)
										goto GET_BULK_OVERFLOW_ERROR;
									//_SNMPPut(OIDLen+1);//for appending "0"
									OIDlen += 1;
								}
								else 
									if(Snmpv3BufferPut(OIDlen,dynScopedBufPtr) != true)
										goto GET_BULK_OVERFLOW_ERROR;
				
								//Put OID
								while( templen-- )
									if(Snmpv3BufferPut(*ptroid++,dynScopedBufPtr) != true)
										goto GET_BULK_OVERFLOW_ERROR;
								if(appendZeroToOID)
									if(Snmpv3BufferPut(0x0,dynScopedBufPtr) != true)
										goto GET_BULK_OVERFLOW_ERROR;
								if(Snmpv3BufferPut(SNMP_END_OF_MIB_VIEW,dynScopedBufPtr) != true)
									goto GET_BULK_OVERFLOW_ERROR;
								if(Snmpv3BufferPut(0x0,dynScopedBufPtr)!= true)
									goto GET_BULK_OVERFLOW_ERROR;

								noOfVarbindreq = 0;

							}
							else if(tempRet != 0)//if(oidLookUpRet != SNMP_END_OF_MIB_VIEW)
							{
								tempRet = ProcessGetBulkVar(&OIDInfo, &OIDValue[0],&OIDlen,&succesor,pduDbPtr);
							}
							if ( tempRet == 0u )
							{
								uint8_t templen=OIDlen;
								uint8_t *ptroid=OIDValue;	
								if(Snmpv3BufferPut(ASN_OID,dynScopedBufPtr) != true)
									goto GET_BULK_OVERFLOW_ERROR;
								if(appendZeroToOID)
								{
									if(Snmpv3BufferPut(OIDlen+1,dynScopedBufPtr)!= true) //for appending "0"
										goto GET_BULK_OVERFLOW_ERROR;
									OIDlen += 1;
								}
								else 
									if(Snmpv3BufferPut(OIDlen,dynScopedBufPtr)!=  true)
										goto GET_BULK_OVERFLOW_ERROR;
				
								//Put OID
								while( templen-- )
									if(Snmpv3BufferPut(*ptroid++,dynScopedBufPtr) != true)
										goto GET_BULK_OVERFLOW_ERROR;
				
								/*Do send back the Same OID if get_next is EndOfMibView. Do not
								  append zero to this OID*/
								
								if(Snmpv3BufferPut(SNMP_END_OF_MIB_VIEW,dynScopedBufPtr) != true)
									goto GET_BULK_OVERFLOW_ERROR;
								if(Snmpv3BufferPut(0x0,dynScopedBufPtr) != true)
									goto GET_BULK_OVERFLOW_ERROR;
				
								//snmpReqVarErrStatus.endOfMibViewErr  |=(0x0001 << varIndex);
								if(appendZeroToOID)
									if(Snmpv3BufferPut(0x0,dynScopedBufPtr) != true)
										goto GET_BULK_OVERFLOW_ERROR;

								noOfVarbindreq = 0;
								
							}
							
						
							dynScopedBufPtr = &gSNMPv3ScopedPduResponseBuf;
							tempOffset = dynScopedBufPtr->length;
							pduLength = dynScopedBufPtr->length -(varStructLenOffset+1);
							dynScopedBufPtr->length = varStructLenOffset;
							Snmpv3BufferPut(pduLength,dynScopedBufPtr);
							dynScopedBufPtr->length = tempOffset;

							/* if length dynamic buffer length increases more than the allocated memory 
							then break from the loop.*/
							if(dynScopedBufPtr->length>= dynScopedBufPtr->maxlength)
							{
								noOfVarbindreq = 0;
								Getbulk_N = 0u; 					
							    goto	GET_BULK_OVERFLOW_ERROR;
							}
							tempRet = 0xFF;
						}//for(varBindCntr=0;varBindCntr<Getbulk_R;varBindCntr++)						
					}//for(repeatCntr=0;repeatCntr<Getbulk_M;repeatCntr++)
					/* check length*/
				break;	

			default:				
				Snmpv3FreeDynAllocMem();
				return false;
		}
		if(noOfVarbindreq == 0)
		{
			break;
		}
	}

GET_BULK_OVERFLOW_ERROR:
	if(dynScopedBufPtr->length >= dynScopedBufPtr->maxlength)
	{
		if(pduDbPtr->pduType == SNMP_V2C_GET_BULK)
		{
			pduLength = dynScopedBufPtr->length - (varStructLenOffset-1);
			dynScopedBufPtr->length = dynScopedBufPtr->length - pduLength;			
		}
		else
		{
			dynScopedBufPtr->length = varBindHeaderOffset_3;
			Snmpv3BufferPut(0,dynScopedBufPtr);
			Snmpv3SetErrorStatus(errorStatusOffset,errorIndexOffset,SNMP_TOO_BIG,varIndex,dynScopedBufPtr); 
		}
	}


SNMP_PROCESS_END:
	/* pass the data to wire*/
	{
		uint16_t	 	scopedpduHeaderOffset = 0;
		TCPIP_UINT16_VAL 	totalPdulength = {0};
		TCPIP_UINT16_VAL	scoped_pdu_len_1 = {0}; // context data length
		TCPIP_UINT16_VAL	scoped_pdu_len_2 = {0}; // pdu response length
		TCPIP_UINT16_VAL	scoped_pdu_len_3 = {0}; //variable binding header with varbinds
		uint16_t 		i=0;
		//uint32_t       scopedPduHeadPtr=0;
		SNMPV3MSGDATA tempScopedData;
		
		scopedpduHeaderOffset = dynScopedBufPtr->length;
		tempScopedData = gSNMPv3ScopedPduResponseBuf;
		
		// update length for variable binds
		scoped_pdu_len_3.Val = (dynScopedBufPtr->length-3)-varBindHeaderOffset_3;
		dynScopedBufPtr->length = varBindHeaderOffset_3+1;
		Snmpv3BufferPut(scoped_pdu_len_3.v[1],dynScopedBufPtr);
		Snmpv3BufferPut(scoped_pdu_len_3.v[0],dynScopedBufPtr);
		dynScopedBufPtr->length = scopedpduHeaderOffset;
		
		//update the length for get response pduLenOffset 
		scoped_pdu_len_2.Val = (dynScopedBufPtr->length-3)-pduLenOffset;
		dynScopedBufPtr->length = pduLenOffset+1;
		Snmpv3BufferPut(scoped_pdu_len_2.v[1],dynScopedBufPtr);
		Snmpv3BufferPut(scoped_pdu_len_2.v[0],dynScopedBufPtr);
		dynScopedBufPtr->length = scopedpduHeaderOffset;

		scoped_pdu_len_1.Val = dynScopedBufPtr->length-4;
		if((scoped_pdu_len_1.Val >= 0x80) && (scoped_pdu_len_1.Val <= 0xFF))
		{// total scoped pdu length decreamented by 1
			dynScopedBufPtr->length = contextNameOffset+1;
			//if(snmpSecurityLevel>>1 & 0x01)
			//	Snmpv3BufferPut(0x04,dynScopedBufPtr);
			Snmpv3BufferPut(0x30,dynScopedBufPtr);
			Snmpv3BufferPut(0x81,dynScopedBufPtr);
			Snmpv3BufferPut(scoped_pdu_len_1.Val,dynScopedBufPtr);
			dynScopedBufPtr->length = scopedpduHeaderOffset;
			tempScopedData.head++;
			tempScopedData.length--;
		}
		else if((scoped_pdu_len_1.Val > 0xFF) && (scoped_pdu_len_1.Val < 0xFFFF))
		{			
			dynScopedBufPtr->length = contextNameOffset;
			Snmpv3BufferPut(0x30,dynScopedBufPtr);
			Snmpv3BufferPut(0x82,dynScopedBufPtr);
			Snmpv3BufferPut(scoped_pdu_len_1.v[1],dynScopedBufPtr);
			Snmpv3BufferPut(scoped_pdu_len_1.v[0],dynScopedBufPtr);
			dynScopedBufPtr->length = scopedpduHeaderOffset;
			//gSNMPv3ScopedPduResponseBuf.head++;
			//gSNMPv3ScopedPduResponseBuf.length--;
		}
		else
		{// total scoped pdu length decreamented by 2
			dynScopedBufPtr->length = contextNameOffset+2;
			Snmpv3BufferPut(0x30,dynScopedBufPtr);
			//Snmpv3BufferPut(0x81,dynScopedBufPtr);
			Snmpv3BufferPut(scoped_pdu_len_1.Val,dynScopedBufPtr);
			dynScopedBufPtr->length = scopedpduHeaderOffset;
			tempScopedData.head=tempScopedData.head+2;
			tempScopedData.length=tempScopedData.length-2;
		}


		//dynScopedBufPtr = &gSNMPv3ScopedPduResponseBuf;
		totalPdulength.Val = tempScopedData.length + \
							 gSNMPv3PduHeaderBuf.length + \
							 3; // asn_int+len+version

		if((snmpSecurityLevel & 0x02)==0x02)
		{
			tempPtr=tempBuf;
			*tempPtr++=0X04;
			if((tempScopedData.length >= 0x80) && (tempScopedData.length <= 0xFF))
			{
				*tempPtr++=0x81;
				*tempPtr=tempScopedData.length;
				tempCntr=3; //0x04(encrypted pkt),0x81,len
			}
			else if((tempScopedData.length > 0xFF) && (tempScopedData.length < 0xFFFF))
			{			
				*tempPtr++=0x82;
				*tempPtr++=tempScopedData.length>>8;
				*tempPtr=tempScopedData.length;
				tempCntr=4; //0x04(encrypted pkt),0x81,len_1,len_0
			}
			else
			{
				*tempPtr=tempScopedData.length;
				tempCntr=2; //0x04(encrypted pkt),len
			}
		}

		gSnmpV3OUTPduWholeMsgBuf.wholeMsgLen.Val=(totalPdulength.Val+tempCntr/*0x04,0x82,len_1,len_0*/);
		gSnmpV3OUTPduWholeMsgBuf.wholeMsgHead=(uint8_t*)(malloc((size_t)gSnmpV3OUTPduWholeMsgBuf.wholeMsgLen.Val+4+16));
		if(gSnmpV3OUTPduWholeMsgBuf.wholeMsgHead == NULL)
			return false;
		outBufPtr=gSnmpV3OUTPduWholeMsgBuf.wholeMsgHead;	


		//Start Writing to the outPut Buffer

		*outBufPtr++=STRUCTURE;

		totalPdulength.Val+=tempCntr;
							 
		if((totalPdulength.Val >= 0x80) && (totalPdulength.Val <= 0xFF))
		{
			*outBufPtr++=0x81;
			*outBufPtr++=totalPdulength.Val;
		}
		else if((totalPdulength.Val > 0xFF) && (totalPdulength.Val < 0xFFFF))
		{			
			*outBufPtr++=0x82;
			*outBufPtr++=totalPdulength.v[1];
			*outBufPtr++=totalPdulength.v[0];
		}
		else
			*outBufPtr++=totalPdulength.Val;

		*outBufPtr++=ASN_INT;
		*outBufPtr++=0x1;		
		*outBufPtr++=SNMP_V3;

		gSnmpV3OUTPduWholeMsgBuf.msgAuthParamOffsetOutWholeMsg=(uint8_t*)(outBufPtr+gSNMPv3PduHeaderBuf.msgAuthParamOffset);
		//put global snmpv3 msg header 
		for(i=0;i<gSNMPv3PduHeaderBuf.length;i++)
		{
			*outBufPtr++=gSNMPv3PduHeaderBuf.head[i];
		}

		if (gSNMPv3PduHeaderBuf.head!=NULL)
		{	
			free(gSNMPv3PduHeaderBuf.head);
			gSNMPv3PduHeaderBuf.length=0x00;
			gSNMPv3PduHeaderBuf.head=NULL;
		}


		//Copy Scoped PDU to the Out Buffer
		if((snmpSecurityLevel & 0x02)==0x02) //Encrypted message	
		{
			//Copy Packet Auth indicator, length
			for(i=0;i<tempCntr;i++) 
			{
				*outBufPtr++=tempBuf[i];

			}
		}
		gSnmpV3OUTPduWholeMsgBuf.scopedPduOffset=outBufPtr;
		gSnmpV3OUTPduWholeMsgBuf.scopedPduStructLen=tempScopedData.length;

		i=0;
		*outBufPtr++=tempScopedData.head[i++];//0x30

		if(tempScopedData.head[1] == 0x81)
		{
			*outBufPtr++=tempScopedData.head[i++];//0x81
			*outBufPtr++=tempScopedData.head[i++];//len_0
		}
		else if(tempScopedData.head[1] == 0x82)
		{
			*outBufPtr++=tempScopedData.head[i++]; //0x82
			*outBufPtr++=tempScopedData.head[i++]; //len_1
			*outBufPtr++=tempScopedData.head[i++]; //len_0
		}
		else
			*outBufPtr++=tempScopedData.head[i++];//len_o
		
		// send context id and context name and the get response 
		// Authentication and privacy data packet will be sent from here onwards
		for(;i<(tempScopedData.length);i++)
		{
			*outBufPtr++=tempScopedData.head[i];
		}

		if(gSNMPv3ScopedPduResponseBuf.head != NULL)
		{
			free(gSNMPv3ScopedPduResponseBuf.head);
			gSNMPv3ScopedPduResponseBuf.length=0x00;
			gSNMPv3ScopedPduResponseBuf.head=NULL;
		}


		/*Encrypt the Response to the messgae originator*/
		if((snmpSecurityLevel & 0x02)==0x02) //Encrypted message
		{
			 /*Rxed SNMPv3 message is encrypted. Hence Response should be encrypted*/
			 gSnmpV3OUTPduWholeMsgBuf.wholeMsgLen.Val = outBufPtr-gSnmpV3OUTPduWholeMsgBuf.wholeMsgHead;
			 /*If user privacy protocol is AES*/
			if(Snmpv3AESEncryptResponseScopedPdu(&gSnmpV3OUTPduWholeMsgBuf) != SNMPV3_MSG_PRIV_PASS)
				return SNMPV3_MSG_PRIV_FAIL;
		
			/*If user privacy Protocol is DES*/
			//snmpV3DESDecryptRxedScopedPdu();
		}

		/* Authenticate the whole message to be transmitted*/
		if((snmpSecurityLevel & 0x01)==0x01) //Authenticatd message
		{
			 /*Rxed SNMPv3 message is Authenticated.Send authenticatin parameters for the Response*/
			gSnmpV3OUTPduWholeMsgBuf.wholeMsgLen.Val = outBufPtr-gSnmpV3OUTPduWholeMsgBuf.wholeMsgHead;
			 /*If user authentication is HAMC-MD5-96*/
			if(Snmpv3AuthenticateTxPduForDataIntegrity(&gSnmpV3OUTPduWholeMsgBuf)!=SNMPV3_MSG_AUTH_PASS)
				return SNMPV3_MSG_AUTH_FAIL;

			tempPtr = outBufPtr;
			outBufPtr=gSnmpV3OUTPduWholeMsgBuf.msgAuthParamOffsetOutWholeMsg;
			for(i=0;i<snmpOutMsgAuthParamLen;i++)
				*outBufPtr++=snmpOutMsgAuthParamStrng[i];
			outBufPtr = tempPtr;
		}
			
		gSnmpV3OUTPduWholeMsgBuf.wholeMsgLen.Val = outBufPtr-gSnmpV3OUTPduWholeMsgBuf.wholeMsgHead;
		outBufPtr=gSnmpV3OUTPduWholeMsgBuf.wholeMsgHead;
		for(i=0;i < gSnmpV3OUTPduWholeMsgBuf.wholeMsgLen.Val;i++)
		{	
			_SNMPPut(*(outBufPtr+i));
		}

		//if(gSNMPv3ScopedPduResponseBuf.head != NULL)
		if(gSnmpV3OUTPduWholeMsgBuf.wholeMsgHead != NULL)
		{
			free(gSnmpV3OUTPduWholeMsgBuf.wholeMsgHead);
			gSnmpV3OUTPduWholeMsgBuf.wholeMsgHead=0x00;
			gSnmpV3OUTPduWholeMsgBuf.wholeMsgLen.Val=0x00;
			gSnmpV3OUTPduWholeMsgBuf.snmpMsgHead = NULL;
			gSnmpV3OUTPduWholeMsgBuf.msgAuthParamOffsetOutWholeMsg = NULL;
			gSnmpV3OUTPduWholeMsgBuf.scopedPduOffset =  NULL;			
		}

	}
	return true;
}


/****************************************************************************
  Function:
	bool Snmpv3IsValidInt(uint32_t* val)
	
  Summary:
  	Verifies variable datatype as int and retrieves its value.

  Description:
  	This routine populates and validates the received variable for the
  	data type as "ASN_INT" and the data length for max 4 bytes.
  	This rotuine only refers to the incoming snmpv3 request dynamically 
	allocated 	memory buffer 'gSNMPv3ScopedPduRequestBuf' .
  	
  	 		 		  	
  Precondition:
	None
	
  Parameters:
  	val - Pointer to memory where int var value will be stored.
 
  ReturnValues:
	true	- If valid integer type and value is received.
	false	- Other than integer data type and value received .
	
  Remarks:
	None.
***************************************************************************/
bool Snmpv3IsValidInt(uint32_t* val)
{
	TCPIP_UINT32_VAL tempData;
    uint8_t	tempLen=0;

    
    // Get variable type
    if (!IS_ASN_INT(Snmpv3GetBufferData(gSNMPv3ScopedPduRequestBuf,++gSNMPv3ScopedPduDataPos)))
        return false;

	// Integer length of more than 32-bit is not supported.
	tempLen = Snmpv3GetBufferData(gSNMPv3ScopedPduRequestBuf,++gSNMPv3ScopedPduDataPos);
    if ( tempLen > 4u)
        return false;
    

    tempData.Val = 0;
    while( tempLen-- )
        tempData.v[tempLen] = Snmpv3GetBufferData(gSNMPv3ScopedPduRequestBuf,
        													++gSNMPv3ScopedPduDataPos);

    *val = tempData.Val;

    return true;
}


/****************************************************************************
  Function:
	bool _Snmpv3IsValidInt(uint8_t * wholeMsgPtr,uint16_t* pos, uint32_t* val )
	
  Summary:
  	Verifies variable datatype as int and retrieves its value.

  Description:
  	This routine populates and validates the received variable for the
  	data type as "ASN_INT" and the data length for max 4 bytes.
  	 		 		  	
  Precondition:
	None
	
  Parameters:
  	wholeMsgPtr - Pointer to memory where int var value is be stored.
 	pos - position in the memory buffer where data taype to be varified is stored
 	val - Pointer to memory where int var value will be stored.
 	
  ReturnValues:
	true	- If valid integer type and value is received.
	false	- Other than integer data type and value received .
	
  Remarks:
	None.
***************************************************************************/

bool _Snmpv3IsValidInt(uint8_t * wholeMsgPtr,uint16_t* pos, uint32_t* val )
{
	TCPIP_UINT32_VAL tempData;
    uint8_t	tempLen=0;

	uint8_t* lclWholeMsgPtr;
	
	lclWholeMsgPtr=wholeMsgPtr;

    
    // Get variable type
    if (!IS_ASN_INT(Snmpv3GetWholeMsgBufferData(lclWholeMsgPtr,pos)))
        return false;

	// Integer length of more than 32-bit is not supported.
	tempLen = Snmpv3GetWholeMsgBufferData(lclWholeMsgPtr,pos);
    if ( tempLen > 4u)
        return false;
    

    tempData.Val = 0;
    while( tempLen-- )
        tempData.v[tempLen] = Snmpv3GetWholeMsgBufferData(lclWholeMsgPtr,pos);

    *val = tempData.Val;

    return true;
}

/****************************************************************************
  Function:
	bool IsSNMPv3ValidStructure(uint16_t* dataLen)
	
  Summary:
  	Decode variable length structure.

  Description:
	This routine is used  to verify whether the received varbind is of type
	STRUCTURE and to find out the variable binding structure length.
	This rotuine only refers to the incoming snmpv3 request dynamically 
	allocated 	memory buffer 'gSNMPv3ScopedPduRequestBuf' .
  	
  Precondition:
	ProcessHeader() is called.	
	
  Parameters:
  	datalen	- Pointer to memory to store OID structure length.
 
  Return Values:
	true	- If valid Structure data type and value is received.
	false	- If variable data structure is not type STRUCTURE. 	

  Remarks:
	None.
***************************************************************************/
bool IsSNMPv3ValidStructure(uint16_t* dataLen)
{
    TCPIP_UINT32_VAL tempLen;

    if ( !IS_STRUCTURE(Snmpv3GetBufferData(gSNMPv3ScopedPduRequestBuf,++gSNMPv3ScopedPduDataPos)) )
        return false;

	tempLen.Val = Snmpv3GetBufferData(gSNMPv3ScopedPduRequestBuf,++gSNMPv3ScopedPduDataPos);
    if ( tempLen.Val == 0x81) 
    {
        tempLen.Val = Snmpv3GetBufferData(gSNMPv3ScopedPduRequestBuf,++gSNMPv3ScopedPduDataPos);
    }
	else if(tempLen.Val == 0x82)
	{
        tempLen.v[1] = Snmpv3GetBufferData(gSNMPv3ScopedPduRequestBuf,++gSNMPv3ScopedPduDataPos);		
        tempLen.v[0] = Snmpv3GetBufferData(gSNMPv3ScopedPduRequestBuf,++gSNMPv3ScopedPduDataPos);
	}
	else if(tempLen.Val == 0)
		return false;
    // Since we are using UDP as our transport and UDP are not fragmented,
    // this structure length cannot be more than 1500 bytes.
    // As a result, we will only use lower uint16_t of length value.
    *dataLen = tempLen.Val;

    return true;
}


/****************************************************************************
  Function:
	bool _IsSNMPv3ValidStructure(uint8_t* wholeMsgPtr,uint16_t* pos, uint16_t* dataLen )
	
  Summary:
  	Decode variable length structure.

  Description:
  	This routine populates and validates the received variable for the
  	data type as "STRUCTURE" and the data length for max 4 bytes.
  	 		 		  	
  Precondition:
	None
	
  Parameters:
  	wholeMsgPtr - Pointer to memory where int var value is be stored.
 	pos - position in the memory buffer where data taype to be varified is stored
 	val - Pointer to memory where int var value will be stored.
 	
  ReturnValues:
	true	- If valid integer type and value is received.
	false	- Other than integer data type and value received .
	
  Remarks:
	None.
***************************************************************************/
bool _IsSNMPv3ValidStructure(uint8_t* wholeMsgPtr,uint16_t* pos, uint16_t* dataLen )
{
    TCPIP_UINT32_VAL tempLen;

    if ( !IS_STRUCTURE(Snmpv3GetWholeMsgBufferData(wholeMsgPtr,pos)) )
        return false;


	tempLen.Val = Snmpv3GetWholeMsgBufferData(wholeMsgPtr,pos);
    if ( tempLen.Val == 0x81) 
    {
        tempLen.Val = Snmpv3GetWholeMsgBufferData(wholeMsgPtr,pos);
    }
	else if(tempLen.Val == 0x82)
	{
        tempLen.v[1] = Snmpv3GetWholeMsgBufferData(wholeMsgPtr,pos);		
        tempLen.v[0] = Snmpv3GetWholeMsgBufferData(wholeMsgPtr,pos);
	}
	else if(tempLen.Val == 0)
		return false;
    // Since we are using UDP as our transport and UDP are not fragmented,
    // this structure length cannot be more than 1500 bytes.
    // As a result, we will only use lower uint16_t of length value.
    *dataLen = tempLen.Val;

    return true;
}


/****************************************************************************
  Function:
	static uint8_t FindOIDsFromSnmpV3Request(uint16_t pdulen)
	
  Summary:
  	Finds number of varbinds in the varbind list received in a SNMPv3 pdu.
  	
  Description:
  	This routine is used to find the number of OIDs requested in the received
  	snmp pdu.   	
  	
  Precondition	:
	ProcessVariables() is called.	
		
  Parameters:
	pdulen		-	Length of snmp pdu request received. 
    
  Return Values:
	varCount	-	Number of OIDs found in a pdu request. 
	
  Remarks:
  	None.
  	
***************************************************************************/
static uint8_t FindOIDsFromSnmpV3Request(uint16_t pdulen)
{
uint8_t varCount=0;
uint16_t prevUDPRxOffset;
uint16_t varBindLen;
uint16_t snmpPduLen;
 	
	snmpPduLen=pdulen;

	prevUDPRxOffset=gSNMPv3ScopedPduDataPos;

	while(snmpPduLen)
	{
		
		if(!IsSNMPv3ValidStructure(&varBindLen))
			return false;

		gSNMPv3ScopedPduDataPos = gSNMPv3ScopedPduDataPos+varBindLen;
		varCount++;
		snmpPduLen=snmpPduLen
					-1      //  1   byte for STRUCTURE identifier
					-1  //  1  byte for varbind length 
					-varBindLen;
		
	}

	gSNMPv3ScopedPduDataPos=prevUDPRxOffset;

	return varCount;
}


/****************************************************************************
  Function:
	void Snmpv3ReportPdu(SNMPV3MSGDATA *dynScopedBufPtr)
	
  Summary:
  	Constructs the report pdu infomration for the Report Pdu.
  	
  Description:
  	The SNMPv3 PDU exchange starts with the agent sending a report pdu on 
  	reception of any Get_Request PDU for SNMPv3 request. 
  	This routine froms the report pdu for response to the requesting entity.
  	
  Precondition	:
	ProcessVariables() is called and a valid SNMPv3 request is received. 
		
  Parameters:
	dynScopedBufPtr	- pointer to the response buffer memory where the 'report' response 
					   to be savced for transmission. 
    
  Return Values:
	None
	
  Remarks:
  	None.
  	
***************************************************************************/
void Snmpv3ReportPdu(SNMPV3MSGDATA *dynScopedBufPtr)
{
	uint8_t	usmStatEngineIds[]={43,6,1,6,3,15,1,1,4,0};
	uint8_t	reportPduLenOffset=0;	
	uint8_t	usmLen=0,i=0;
	uint16_t	varbindPairOffset1=0;
	

	Snmpv3BufferPut(STRUCTURE,dynScopedBufPtr);
	varbindPairOffset1 = dynScopedBufPtr->length;
	Snmpv3BufferPut(0,dynScopedBufPtr);

/* put  usm OID */
	Snmpv3BufferPut(ASN_OID,dynScopedBufPtr);
	usmLen = sizeof(usmStatEngineIds);
	Snmpv3BufferPut(usmLen,dynScopedBufPtr);
	while(usmLen--)
		Snmpv3BufferPut(usmStatEngineIds[i++],dynScopedBufPtr);

/* put engine ID stat value */	
	Snmpv3BufferPut(SNMP_COUNTER32,dynScopedBufPtr);
	Snmpv3BufferPut(4,dynScopedBufPtr);
	Snmpv3BufferPut(gUsmStatsEngineID.v[3],dynScopedBufPtr);
	Snmpv3BufferPut(gUsmStatsEngineID.v[2],dynScopedBufPtr);
	Snmpv3BufferPut(gUsmStatsEngineID.v[1],dynScopedBufPtr);
	Snmpv3BufferPut(gUsmStatsEngineID.v[0],dynScopedBufPtr);
	
	reportPduLenOffset = dynScopedBufPtr->length;
	
	usmLen = dynScopedBufPtr->length - (varbindPairOffset1+1) ;
	dynScopedBufPtr->length = varbindPairOffset1;
	Snmpv3BufferPut(usmLen,dynScopedBufPtr);
	
	dynScopedBufPtr->length = reportPduLenOffset;
	
	
}


/****************************************************************************
  Function:
	bool IsSnmpv3ValidOID(uint8_t* oid, uint8_t* len)
	
  Summary:
  	Populates OID type, length and oid string from the received pdu.

  Description:
	In this routine, OID data type "ASN_OID" is verified in the received pdu.
	If the data type is matched, then only var bind is processed. OID length
	and OID is populated. The max OID length can be 15. 
  	
  Precondition:
	ProcessVariabels() is called.
	
  Parameters:
  	oid - Pointer to memory to store the received OID string
  	len	- Pointer to memory to store OID length
 
  Return Values:
	true	- If value type is ASN_OID and oid length not more than 15. 
	false	- Otherwise.
	
  Remarks:
	None.
***************************************************************************/
static bool IsSnmpv3ValidOID(uint8_t* oid, uint8_t* len)
{
    uint8_t tempLen=0;

    // Fetch and verify that this is OID.
    if ( !IS_OID(Snmpv3GetBufferData(gSNMPv3ScopedPduRequestBuf,++gSNMPv3ScopedPduDataPos)) )
        return false;

   tempLen = Snmpv3GetBufferData(gSNMPv3ScopedPduRequestBuf,++gSNMPv3ScopedPduDataPos);

    // Make sure that OID length is within our capability.
    if (tempLen > (uint8_t)OID_MAX_LEN )
        return false;

    *len = tempLen;

	while( tempLen-- )
	{
       *oid++ = Snmpv3GetBufferData(gSNMPv3ScopedPduRequestBuf,++gSNMPv3ScopedPduDataPos);
	}
	*oid=0xff;
    return true;
}

/****************************************************************************
  Function:
	static bool IsSnmpV3ASNNull(void)
	
  Summary:
  	Verifies the value type as ASN_NULL.

  Description:
  	For Get,Get_Next,Get_Bulk snmp reuest, the var bind the value data type 
  	should be ASN_NULL and value field must be NULL and . This routine
  	verifies the data type and value fields in the received requests.
  	The SET request, the value data type can not be ASN_NULL,
  	otherwise the snmp request is not processed.
  	This rotuine only refers to the incoming snmpv3 request dynamically 
	allocated 	memory buffer 'gSNMPv3ScopedPduRequestBuf' .

  Precondition:
	None
	
  Parameters:
  	None
 
  Returns Values
	true	- If value type is ASN_NULL and value is NULL. 
	false	- If data type and value is other than ASN_NULL and NULL resp.
	
  Remarks:
	None.
***************************************************************************/
static bool IsSnmpV3ASNNull(void)
{
	uint8_t a;

	a = Snmpv3GetBufferData(gSNMPv3ScopedPduRequestBuf,++gSNMPv3ScopedPduDataPos);

	if (!IS_ASN_NULL(a))
			return false;

    // Fetch and verify that length value is zero.
    return (Snmpv3GetBufferData(gSNMPv3ScopedPduRequestBuf,++gSNMPv3ScopedPduDataPos) == 0u );
}


/****************************************************************************
  Function:
	void Snmpv3SetErrorStatus(uint16_t errorStatusOffset,
                           uint16_t errorIndexOffset,
                           SNMP_ERR_STATUS errorStatus,
                           uint8_t errorIndex,SNMPV3MSGDATA *dynScopedPduPutBuf)
  Summary:
  	Set snmpv3 error status in the response pdu. 
  	
  Description:
  	This routine processes the received snmp Get request pdu for the 
  	variable binding in the request and also creates the response pdu.
  	
  Precondition:
	ProcessVariables() is called.	
	
  Parameters:
    errorStatusOffset - Offset to update error status in Response Tx pdu 	
    errorIndexOffset  - Offset to update error index
    errorStatus		  - Snmp process error to be updated in response.	
    errorIndex		  - Index of the request varbind in the var bind list 
    					for which error status is to be updated. 					
    dynScopedPduPutBuf -  dynamic snmpv3 scoped pdu buffer
  Returns:
	None.
	
  Remarks:
  	None.
***************************************************************************/
void Snmpv3SetErrorStatus(uint16_t errorStatusOffset,
                           uint16_t errorIndexOffset,
                           SNMP_ERR_STATUS errorStatus,
                           uint8_t errorIndex,SNMPV3MSGDATA *dynScopedPduPutBuf)
{
    uint16_t prevOffset;

    prevOffset = dynScopedPduPutBuf->length;
	dynScopedPduPutBuf->length = errorStatusOffset;
    Snmpv3BufferPut(errorStatus,dynScopedPduPutBuf);

    
	dynScopedPduPutBuf->length = errorIndexOffset;
    Snmpv3BufferPut(errorIndex,dynScopedPduPutBuf);

	dynScopedPduPutBuf->length = prevOffset;
}

/****************************************************************************
  Function:
	uint8_t Snmpv3IsValidAuthStructure(uint16_t* dataLen)
	
  Summary:
  	Decode variable length structure.

  Description:
	This routine is used  to verify whether the received varbind is of type
	STRUCTURE and to find out the variable binding structure length.
  	
  Precondition:
	ProcessHeader() is called.	
	
  Parameters:
  	datalen	- Pointer to memory to store OID structure length.
 
  Return Values:
	headrbytes	- Variable binding length.
	false		- If variable data structure is not type STRUCTURE. 	

  Remarks:
	None.
***************************************************************************/
uint8_t Snmpv3IsValidAuthStructure(uint16_t* dataLen)
{
    TCPIP_UINT32_VAL tempLen;
    uint8_t headerBytes;
	uint8_t authStructure=0;

	authStructure = _SNMPGet();

    if ( !IS_STRUCTURE(authStructure) && !IS_SNMPV3_AUTH_STRUCTURE(authStructure) )
        return false;

    // Retrieve structure length
    headerBytes = IsValidLength(&tempLen.w[0]);
    if ( !headerBytes )
        return false;

    headerBytes++;

    // Since we are using UDP as our transport and UDP are not fragmented,
    // this structure length cannot be more than 1500 bytes.
    // As a result, we will only use lower uint16_t of length value.
    *dataLen = tempLen.w[0];

    return headerBytes;
}

/****************************************************************************
  Function:
	uint8_t _Snmpv3IsValidAuthStructure(uint16_t* dataLen)
	
  Summary:
  	Decode variable length structure.

  Description:
	This routine is used  to verify whether the received varbind is of type
	STRUCTURE and to find out the variable binding structure length.
  	This rotuine only refers to the incoming snmpv3 request dynamically 
	allocated   memory buffer 'gSnmpV3InPduWholeMsgBuf' .

  Precondition:
	ProcessHeader() is called.	
	
  Parameters:
  	datalen	- Pointer to memory to store OID structure length.
 
  Return Values:
	headrbytes	- Variable binding length.
	false		- If variable data structure is not type STRUCTURE. 	

  Remarks:
	None.
***************************************************************************/

uint8_t _Snmpv3IsValidAuthStructure(uint16_t* dataLen)
{
    TCPIP_UINT16_VAL tempLen;
    uint8_t headerBytes;
	uint8_t authStructure=0;
	uint16_t tempPos;
	
	uint8_t tempData;
	
	tempPos=snmpMsgBufSeekPos;
	authStructure = Snmpv3GetWholeMsgBufferData(gSnmpV3InPduWholeMsgBuf.snmpMsgHead,&tempPos);
	

    if ( !IS_STRUCTURE(authStructure) && !IS_SNMPV3_AUTH_STRUCTURE(authStructure) )
        return false;

	
    // Initialize length value.
    tempLen.Val = 0;
    headerBytes = 0;

    tempData = Snmpv3GetWholeMsgBufferData(gSnmpV3InPduWholeMsgBuf.snmpMsgHead,&tempPos);
    tempLen.v[0] = tempData;
    if ( tempData & 0x80 )
    {
        tempData &= 0x7F;

        // We do not support any length byte count of more than 2
        // i.e. total length value must not be more than 16-bit.
        if ( tempData > 2u )
            return false;

        // Total length bytes are 0x80 itself plus tempData.
        headerBytes = tempData + 1;

        // Get upto 2 bytes of length value.
        while( tempData-- )
            tempLen.v[tempData] = Snmpv3GetWholeMsgBufferData(gSnmpV3InPduWholeMsgBuf.snmpMsgHead,&tempPos);
    }
    else
        headerBytes = 1;

	 if ( !headerBytes )
      return false;

	gSnmpV3InPduWholeMsgBuf.scopedPduAuthStructVal=authStructure;
	gSnmpV3InPduWholeMsgBuf.scopedPduStructLen=tempLen.Val;
	
    headerBytes++;

    // Since we are using UDP as our transport and UDP are not fragmented,
    // this structure length cannot be more than 1500 bytes.
    // As a result, we will only use lower uint16_t of length value.
    *dataLen = tempLen.Val;

	snmpMsgBufSeekPos=tempPos;
    return headerBytes;
}


#if defined(SNMP_V1_V2_TRAP_WITH_SNMPV3)
uint8_t gSNMPV3TrapSecurityLevel = NO_REPORT_NO_PRIVACY_NO_AUTH;
#define INVALID_INDEX 0xFF

/****************************************************************************
  Function:
	uint8_t Snmpv3GetUserIndxFromUsmUserDB(uint8_t targetIndex)
	
  Summary:
  	Routine to find the index of the user name in the user data base table. 
  	
  Description:
  	There are two different data base tables defined with SNMPv3 stack,
  	like 'snmpV3UserDataBase' and 'gSnmpv3TrapConfigData'.
	It returns the index of the user name which matches to the trap target 
	user name within the user data base. 

  Precondition:
	Trap notification event is triggred and the trap send flag is enabled.
	
  Parameters:
  	targetIndex -index of the 'gSnmpv3TrapConfigData' table to match the 
  			     'userSecurityName' with the user data base  
 
  Return Values:
	INVALID_INDEX - if the trap target user name does not match.
	uint8_t - Byte value fo the index matched

  Remarks:
	None.
***************************************************************************/
uint8_t Snmpv3GetUserIndxFromUsmUserDB(uint8_t targetIndex)
{
	uint8_t *userSecurityName=NULL;
	uint8_t userDBsecurityLevel=0;
	uint8_t trapSecurityLevel=0; 
	uint8_t userTrapSecLen=0;
	uint8_t *userTrapSecurityName=NULL;
	uint8_t i=0;

	trapSecurityLevel = Snmpv3GetTrapSecurityLevel((STD_BASED_SNMPV3_SECURITY_LEVEL)gSnmpv3TrapConfigData[targetIndex].securityLevelType);
	if(trapSecurityLevel == INVALID_MSG)
		return INVALID_INDEX;

	for(i=0;i<SNMPV3_USM_MAX_USER;i++)
	{
		userSecurityName = snmpV3UserDataBase[i].userName;
		userDBsecurityLevel = Snmpv3GetSecurityLevel(i);
		userTrapSecLen = strlen(gSnmpv3TrapConfigData[targetIndex].userSecurityName);
		userTrapSecurityName = gSnmpv3TrapConfigData[targetIndex].userSecurityName;
		
		if(userTrapSecLen != snmpV3UserDataBase[i].userNameLength)
			continue;
		if(strncmp(userTrapSecurityName,userSecurityName,userTrapSecLen) == 0)
		{
			if(trapSecurityLevel == userDBsecurityLevel)
				return i;
			else
				continue;
		}
	}

	return INVALID_INDEX;
}


/****************************************************************************
  Function:
	bool Snmpv3CmprTrapSecNameAndSecLvlWithUSMDb(
				uint8_t tragetIndex,
				uint8_t userTrapSecLen,
				uint8_t *userTrapSecurityName,
				STD_BASED_SNMPV3_SECURITY_LEVEL securityLevel)
	
  Summary:
  	Routine to find the index of the user name in the user data base table. 
  	
  Description:
  	There are two different data base tables defined with SNMPv3 stack,
  	like 'snmpV3UserDataBase' and 'gSnmpv3TrapConfigData'.
	This routine is used to validte the trap user security level setting with
	SET request.
	

  Precondition:
	SET operation would be allowed if the USM security conditions and
	user security name in the request is matched to one of the user security 
	name stored in the usm user database.
	
  Parameters:
  	targetIndex -index of the 'gSnmpv3TrapConfigData' table to match the 
  			     'userSecurityName' with the user data base  
	userTrapSecLen - user sec name length in the SET request
	userTrapSecurityName - pointer to user sec name in the SET request
	securityLevel - trap security level to be SET on the agent
 
  Return Values:
	true - if the trap target user sec level setting is successful
	FLASE - If the SET failed due to non matching of the security parameters

  Remarks:
	None.
***************************************************************************/
bool Snmpv3CmprTrapSecNameAndSecLvlWithUSMDb(uint8_t tragetIndex,uint8_t userTrapSecLen,
								uint8_t *userTrapSecurityName,STD_BASED_SNMPV3_SECURITY_LEVEL securityLevel)
{

	uint8_t *userSecurityName=NULL;
	uint8_t userDBsecurityLevel=0;
	uint8_t trapSecurityLevel=0; 
	uint8_t i=0;

	trapSecurityLevel = Snmpv3GetTrapSecurityLevel(securityLevel);
	if(trapSecurityLevel == INVALID_MSG)
		return false;

	for(i=0;i<SNMPV3_USM_MAX_USER;i++)
	{
		userSecurityName = snmpV3UserDataBase[i].userName;
		userDBsecurityLevel = Snmpv3GetSecurityLevel(i);
		if(userTrapSecLen != snmpV3UserDataBase[i].userNameLength)
			continue;
		if(strncmp(userTrapSecurityName,userSecurityName,userTrapSecLen) == 0)
		{
			if(trapSecurityLevel == userDBsecurityLevel)
				return true;
			else
				continue;
		}
	}

	return false;
}


/****************************************************************************
  Function:
	uint8_t Snmpv3GetTrapSecurityLevel(STD_BASED_SNMPV3_SECURITY_LEVEL securityLevel)
	
  Summary:
  	Routine to find the report, auth and privacy flags settings in the TRAP. 
  	
  Description:
  	This routine to find the report, auth and privacy flags setting for the trap to be 
  	generated. The message flags octet's least significant three bits: 
  	Reportable, PrivFlag, AuthFlag forms different secuity level combinations.
	

  Precondition:
	None
	
  Parameters:
  	securityLevel -trap security level to be compared for getting the agent's security 
  				level settings 
  Return Values:
	NO_REPORT_NO_PRIVACY_NO_AUTH - No authentication, no encryption
	NO_REPORT_NO_PRIVACY_BUT_AUTH_PROVIDED - authentication but no encryption
  	NO_REPORT_PRIVACY_AND_AUTH_PROVIDED - authentication and encryption 
  	INVALID_MSG - if security level doesn't match any of the above

  Remarks:
	None.
***************************************************************************/
uint8_t Snmpv3GetTrapSecurityLevel(STD_BASED_SNMPV3_SECURITY_LEVEL securityLevel)
{
	uint8_t tempSecurityLevel=0xFF;
	
	switch(securityLevel)
	{
		case NO_AUTH_NO_PRIV:
			tempSecurityLevel =  NO_REPORT_NO_PRIVACY_NO_AUTH;
			break;
		case AUTH_NO_PRIV:
			tempSecurityLevel = NO_REPORT_NO_PRIVACY_BUT_AUTH_PROVIDED;
			break;
		case AUTH_PRIV:
			tempSecurityLevel = NO_REPORT_PRIVACY_AND_AUTH_PROVIDED;
			break;
		default:
			return INVALID_MSG;
	}
	return tempSecurityLevel;
}

/****************************************************************************
  Function:
	bool Snmpv3TrapMsgHeaderPDU(unsigned int targetIndex)
	
  Summary:
  	TRAP PDU message header construction. 
  	
  Description:
  	This routine forms the message header for the SNMPv3 trap PDU 
  	to be originated from this agent. 
  	
  Precondition:
	TRAP event is triggered.
	
  Parameters:
  	targetIndex -index of the 'gSnmpv3TrapConfigData' table's security user name 
  			     for which the TRAP PDU message header to constructed.
  			     
  Return Values:
	INVALID_INDEX - if the 'targetIndex' does not match to any of the users configured 
					with the agent in 'gSnmpv3TrapConfigData'.
	true - The trap message header generation is successful.
	false -The trap message header generation failed.

  Remarks:
	None.
***************************************************************************/
bool Snmpv3TrapMsgHeaderPDU(unsigned int targetIndex)
{
	uint8_t putCntr=0;	
	uint8_t *ptr=NULL;
	bool retBuf=true;
	uint8_t snmpv3MsgGlobalHeaderlength=0;
	uint16_t snmpv3Headerlength=0;
	uint16_t snmpv3MsgAuthHeaderLength=0;
	uint8_t tempData=0;
	uint16_t msgHeaderOffset1=0;
	uint16_t msgHeaderOffset2=0;
	uint16_t tempMsgHeaderOffset=0;
	uint8_t  USM_Index=0,temp_index=0;
	
	temp_index = securityPrimitivesOfIncomingPdu.securityNameLength ;
	USM_Index = Snmpv3GetUserIndxFromUsmUserDB(targetIndex);
	if(USM_Index != INVALID_INDEX)
	{
		securityPrimitivesOfIncomingPdu.securityNameLength = strlen(gSnmpv3TrapConfigData[targetIndex].userSecurityName);
	}
	snmpv3Headerlength = MSGGLOBAL_HEADER_LEN(snmpv3MsgGlobalHeaderlength)+
						  MSG_AUTHORITATIVE_HEADER_LEN(snmpv3MsgAuthHeaderLength);

	// update the IN pdu trap security name size
	securityPrimitivesOfIncomingPdu.securityNameLength = temp_index;	
	
	ptr = gSNMPv3TrapMsgHeaderBuf.head = (uint8_t *)(malloc((size_t)snmpv3Headerlength+1));
	//ptr = gSNMPv3PduHeaderBuf.head = (uint8_t *)(malloc(0x1));
	if(ptr == NULL)
		return false;
	
	gSNMPv3TrapMsgHeaderBuf.length = 0;
	gSNMPv3TrapMsgHeaderBuf.maxlength = snmpv3Headerlength+1;

	/*Msg Processing Model PDU header */
	/*ID + Msg Size + Msg Flag + Security Model */
	//message header
	Snmpv3BufferPut(STRUCTURE,&gSNMPv3TrapMsgHeaderBuf);
	Snmpv3BufferPut(MSGGLOBAL_HEADER_LEN(snmpv3MsgGlobalHeaderlength)-2,&gSNMPv3TrapMsgHeaderBuf);

	//Put "msgID" type ASN_INT of length 4 bytes	
	Snmpv3BufferPut(ASN_INT,&gSNMPv3TrapMsgHeaderBuf);		
	Snmpv3BufferPut(0x04,&gSNMPv3TrapMsgHeaderBuf);
	Snmpv3BufferPut(incomingSnmpPDUmsgID.v[3],&gSNMPv3TrapMsgHeaderBuf);
	Snmpv3BufferPut(incomingSnmpPDUmsgID.v[2],&gSNMPv3TrapMsgHeaderBuf);
	Snmpv3BufferPut(incomingSnmpPDUmsgID.v[1],&gSNMPv3TrapMsgHeaderBuf);
	Snmpv3BufferPut(incomingSnmpPDUmsgID.v[0],&gSNMPv3TrapMsgHeaderBuf);

	//Put "msgMaxSize"  type ASN_INT of length 4 bytes
	Snmpv3BufferPut(ASN_INT,&gSNMPv3TrapMsgHeaderBuf);
	Snmpv3BufferPut(0x04,&gSNMPv3TrapMsgHeaderBuf);
	Snmpv3BufferPut(snmpEngineMaxMessageSize.v[3],&gSNMPv3TrapMsgHeaderBuf);
	Snmpv3BufferPut(snmpEngineMaxMessageSize.v[2],&gSNMPv3TrapMsgHeaderBuf);
	Snmpv3BufferPut(snmpEngineMaxMessageSize.v[1],&gSNMPv3TrapMsgHeaderBuf);
	Snmpv3BufferPut(snmpEngineMaxMessageSize.v[0],&gSNMPv3TrapMsgHeaderBuf);

	//Put "msgFlags"  type octet_string 
	Snmpv3BufferPut(OCTET_STRING,&gSNMPv3TrapMsgHeaderBuf);
	Snmpv3BufferPut(0x01,&gSNMPv3TrapMsgHeaderBuf);
	Snmpv3BufferPut(gSNMPV3TrapSecurityLevel&0x03,&gSNMPv3TrapMsgHeaderBuf);//Rsponse "msgFlags" value as Reportable, Encrypted and Authenticated Bits are not set. 

	//Put "msgSecurityModel"	
	Snmpv3BufferPut(ASN_INT,&gSNMPv3TrapMsgHeaderBuf);
	Snmpv3BufferPut(0x01,&gSNMPv3TrapMsgHeaderBuf);
	Snmpv3BufferPut(gSnmpv3TrapConfigData[targetIndex].securityModelType,&gSNMPv3TrapMsgHeaderBuf);



	/*User Security Module pdu header 
	Authoritative Engin ID + Authoritative Boots + Authoritative Engine Time+ 
	User name + Authentication parameters + Privacy Parameter 
	*/
	Snmpv3BufferPut(OCTET_STRING,&gSNMPv3TrapMsgHeaderBuf);  //Security Parameter string		
	msgHeaderOffset1 = gSNMPv3TrapMsgHeaderBuf.length;
	Snmpv3BufferPut(MSG_AUTHORITATIVE_HEADER_LEN(snmpv3MsgAuthHeaderLength)-2,&gSNMPv3TrapMsgHeaderBuf);
	Snmpv3BufferPut(STRUCTURE,&gSNMPv3TrapMsgHeaderBuf);
	msgHeaderOffset2 = gSNMPv3TrapMsgHeaderBuf.length;
	Snmpv3BufferPut(MSG_AUTHORITATIVE_HEADER_LEN(snmpv3MsgAuthHeaderLength)-4,&gSNMPv3TrapMsgHeaderBuf);
	
	//Put "msgAuthoritiveEngineID"	
	Snmpv3BufferPut(OCTET_STRING,&gSNMPv3TrapMsgHeaderBuf);	
	Snmpv3BufferPut(snmpEngnIDLength,&gSNMPv3TrapMsgHeaderBuf); //Integer Length
	putCntr = 0;
	for(;putCntr<snmpEngnIDLength;putCntr++)
	{
		Snmpv3BufferPut(snmpEngineID[putCntr],&gSNMPv3TrapMsgHeaderBuf);
	}

	//Put "msgAuthoritiveEngineBoots" 
	Snmpv3BufferPut(ASN_INT,&gSNMPv3TrapMsgHeaderBuf);	
	Snmpv3BufferPut(0x04,&gSNMPv3TrapMsgHeaderBuf);
	Snmpv3BufferPut(snmpEngineBoots>>24,&gSNMPv3TrapMsgHeaderBuf);
	Snmpv3BufferPut(snmpEngineBoots>>16,&gSNMPv3TrapMsgHeaderBuf);
	Snmpv3BufferPut(snmpEngineBoots>>8,&gSNMPv3TrapMsgHeaderBuf);
	Snmpv3BufferPut(snmpEngineBoots,&gSNMPv3TrapMsgHeaderBuf);
	
	//Put "msgAuthoritiveEngineTime" 
	Snmpv3GetAuthEngineTime();
	Snmpv3BufferPut(ASN_INT,&gSNMPv3TrapMsgHeaderBuf);	
	Snmpv3BufferPut(0x04,&gSNMPv3TrapMsgHeaderBuf); 
	Snmpv3BufferPut(snmpEngineTime.v[3],&gSNMPv3TrapMsgHeaderBuf);
	Snmpv3BufferPut(snmpEngineTime.v[2],&gSNMPv3TrapMsgHeaderBuf);
	Snmpv3BufferPut(snmpEngineTime.v[1],&gSNMPv3TrapMsgHeaderBuf);
	Snmpv3BufferPut(snmpEngineTime.v[0],&gSNMPv3TrapMsgHeaderBuf);
	
	putCntr = 0;

	//Put "msgUserName" 
	Snmpv3BufferPut(OCTET_STRING,&gSNMPv3TrapMsgHeaderBuf);	
	tempData = strlen(gSnmpv3TrapConfigData[targetIndex].userSecurityName);
	Snmpv3BufferPut(tempData,&gSNMPv3TrapMsgHeaderBuf);	
	if(tempData != 0)
	{
		ptr= gSnmpv3TrapConfigData[targetIndex].userSecurityName;

		for(;putCntr<tempData;putCntr++)
		{
			Snmpv3BufferPut(ptr[putCntr],&gSNMPv3TrapMsgHeaderBuf);
		}
	}
	

	putCntr = 0;
	
	Snmpv3UsmOutMsgAuthenticationParam(snmpV3UserDataBase[USM_Index].userHashType);
	//Snmpv3UsmOutMsgAuthenticationParam(SNMPV3_HAMC_MD5);

	//Put "msgAuthenticationParameters" 
	Snmpv3BufferPut(OCTET_STRING,&gSNMPv3TrapMsgHeaderBuf);
	if((gSNMPV3TrapSecurityLevel &0x01) == 0x01)
	{
		Snmpv3BufferPut(snmpOutMsgAuthParamLen,&gSNMPv3TrapMsgHeaderBuf); //Not supported with the Alpha Release.

		gSNMPv3TrapMsgHeaderBuf.msgAuthParamOffset=gSNMPv3TrapMsgHeaderBuf.length;

		for(;putCntr<snmpOutMsgAuthParamLen;putCntr++)
			Snmpv3BufferPut(0x0,&gSNMPv3TrapMsgHeaderBuf);
	}
	else
	{
		Snmpv3BufferPut(0x0,&gSNMPv3TrapMsgHeaderBuf); //Not supported with the Alpha Release.
	}
	
	putCntr = 0;
	Snmpv3USMOutMsgPrivParam();
	
	//Put "msgPrivacyParameters" 
	Snmpv3BufferPut(OCTET_STRING,&gSNMPv3TrapMsgHeaderBuf); 
	if((gSNMPV3TrapSecurityLevel&0x02) == 0x02)
	{
		Snmpv3USMOutMsgPrivParam();
		Snmpv3BufferPut(snmpOutMsgPrivParamLen,&gSNMPv3TrapMsgHeaderBuf); //Not supported with the Alpha Release
		for(;putCntr<snmpOutMsgPrivParamLen;putCntr++)
			retBuf = Snmpv3BufferPut(snmpOutMsgPrvParamStrng[putCntr],&gSNMPv3TrapMsgHeaderBuf);
	}
	else
	{
		Snmpv3BufferPut(0x0,&gSNMPv3TrapMsgHeaderBuf); //Not supported with the Alpha Release.
	}


	tempMsgHeaderOffset = gSNMPv3TrapMsgHeaderBuf.length;
	gSNMPv3TrapMsgHeaderBuf.length = msgHeaderOffset2;
	Snmpv3BufferPut((tempMsgHeaderOffset-msgHeaderOffset2)-1,&gSNMPv3TrapMsgHeaderBuf); 
	gSNMPv3TrapMsgHeaderBuf.length = tempMsgHeaderOffset;
	
	tempMsgHeaderOffset = gSNMPv3TrapMsgHeaderBuf.length;
	gSNMPv3TrapMsgHeaderBuf.length = msgHeaderOffset1;
	Snmpv3BufferPut((tempMsgHeaderOffset-msgHeaderOffset1)-1,&gSNMPv3TrapMsgHeaderBuf);
	gSNMPv3TrapMsgHeaderBuf.length = tempMsgHeaderOffset;
	
	return true;
}


/****************************************************************************
  Function:
	uint8_t Snmpv3TrapScopedpdu(SNMP_ID var, SNMP_VAL val, SNMP_INDEX index,
							   uint8_t targetIndex)
	
  Summary:
  	TRAP PDU scoped pdu header construction. 
  	
  Description:
  	This routine forms the trap scoped pdu header for the SNMPv3 trap PDU 
  	to be originated from this agent. Scoped pdu comprises of 
  	msg data : - <contextEngineID><context name>[<data> == <pdutype><request id>
	<error status><error index><varbinds>
  	
  Precondition:
	TRAP event is triggered.
	
  Parameters:
  	var - var id of the variable whose value to be sent in the trap pdu
  	val - value of the variable
  	index - index of the variable in the multiple variable bind scenario
  	targetIndex -index of the 'gSnmpv3TrapConfigData' table's security user name 
  			     for which the TRAP PDU message header to constructed.
 
   Return Values:
	true - The trap scoped pdu header generation is successful.
	false -The trap scoped pdu header generation failed.

  Remarks:
	None.
***************************************************************************/

uint8_t Snmpv3TrapScopedpdu(SNMP_ID var, SNMP_VAL val, SNMP_INDEX index,uint8_t targetIndex)
{	
	uint8_t 			*ptr=NULL;
	uint8_t			contextName[]="";
	uint8_t			contextEngId[]="";
	uint8_t			count=0;
	uint8_t			OIDLen=0; 
    uint8_t 			len=0;
    uint8_t 			OIDValue[OID_MAX_LEN];
#ifdef SNMP_STACK_USE_V2_TRAP
	uint8_t 			snmptrap_oids[]  = {0x2b,6,1,6,3,1,1,4,1 }; /* len=10 */
	uint8_t			sysUpTime_oids[] = {0x2b,6,1,2,1,1,3}; /* len = 8 */
#endif
	uint16_t			contextIDlen=0;
	uint16_t			contextNameLength=0;
	uint16_t			snmpv3Headerlength=0,snmpv3MsgGlobalHeaderlength=0;
	uint16_t			snmpv3MsgAuthHeaderLength=0;
	SNMPV3MSGDATA	*dynTrapScopedPduBuf;
	static uint16_t		pduStructLenOffset=0;
	uint16_t			varPairStructLenOffset=0;
	static uint16_t		varBindStructLenOffset=0;
	uint16_t			tempOffset=0;
    OID_INFO		rec;
	DATA_TYPE_INFO	dataTypeInfo;
	TCPIP_UINT16_VAL		varBindLen = {0};
	uint8_t  			USM_Index=0,temp_index=0;
	int 			i=0;
	
	if(gSNMPv3TrapScopedPduResponseBuf.head == NULL)
	{
		
		temp_index = securityPrimitivesOfIncomingPdu.securityNameLength ;
		USM_Index = Snmpv3GetUserIndxFromUsmUserDB(targetIndex);
		if(USM_Index != INVALID_INDEX)
		{
			securityPrimitivesOfIncomingPdu.securityNameLength = strlen(gSnmpv3TrapConfigData[targetIndex].userSecurityName);
		}
		snmpv3Headerlength = MSGGLOBAL_HEADER_LEN(snmpv3MsgGlobalHeaderlength)+
									  MSG_AUTHORITATIVE_HEADER_LEN(snmpv3MsgAuthHeaderLength);
		// update the IN pdu trap security name size
		securityPrimitivesOfIncomingPdu.securityNameLength = temp_index;
		
		ptr = gSNMPv3TrapScopedPduResponseBuf.head = 
			(uint8_t*)(malloc((size_t)(SNMP_MAX_MSG_SIZE - snmpv3Headerlength)+1));
		if(ptr == NULL)
		{
			return SNMP_ACTION_UNKNOWN;
		}
		gSNMPv3TrapScopedPduResponseBuf.length = 0;
		gSNMPv3TrapScopedPduResponseBuf.maxlength = (SNMP_MAX_MSG_SIZE - snmpv3Headerlength)+1;

		 //Start collecting the plaint text Scoped PDU data byte from the incoming PDU.
		 //Check if the plain text scoped pdu data bytes are binded in ASN structure format 

		dynTrapScopedPduBuf = &gSNMPv3TrapScopedPduResponseBuf;
		Snmpv3BufferPut(STRUCTURE,dynTrapScopedPduBuf); // First item to Response buffer is packet structure
		Snmpv3BufferPut(0x82,dynTrapScopedPduBuf); 
		Snmpv3BufferPut(0,dynTrapScopedPduBuf);
		Snmpv3BufferPut(0,dynTrapScopedPduBuf);

		//Collect context engine id
		Snmpv3BufferPut(OCTET_STRING,dynTrapScopedPduBuf);
		// populate context Engine id to contextEngId
		contextIDlen = strlen((char*)contextEngId);
		if(contextIDlen == 0)
		{			
			Snmpv3BufferPut(0,dynTrapScopedPduBuf);
		}
		else
		{
			//copy context engine id from a local buffer			
			Snmpv3BufferPut(contextIDlen,dynTrapScopedPduBuf);
			while(contextIDlen--)
				Snmpv3BufferPut(contextEngId[count++],dynTrapScopedPduBuf);
		}

		//Check and collect "contextName" 
		Snmpv3BufferPut(OCTET_STRING,dynTrapScopedPduBuf);
		contextNameLength = strlen((char*)contextName);
		count = 0;
		if(contextNameLength == 0)
		{
			Snmpv3BufferPut(0,dynTrapScopedPduBuf);
		}
		else
		{
			Snmpv3BufferPut(contextNameLength,dynTrapScopedPduBuf);
			while(contextNameLength--)
				Snmpv3BufferPut(contextName[count++],dynTrapScopedPduBuf);
		}	


#ifdef SNMP_STACK_USE_V2_TRAP
		//TRAP Version type.  
		Snmpv3BufferPut(SNMP_V2_TRAP,dynTrapScopedPduBuf);
		pduStructLenOffset = dynTrapScopedPduBuf->length;
		Snmpv3BufferPut(0x82,dynTrapScopedPduBuf);
		Snmpv3BufferPut(0,dynTrapScopedPduBuf);
		Snmpv3BufferPut(0,dynTrapScopedPduBuf);

		//put Request ID for the trapv2 as 1 
		Snmpv3BufferPut(ASN_INT,dynTrapScopedPduBuf);// To simplify logic, always use 4 byte long requestID
		Snmpv3BufferPut(4,dynTrapScopedPduBuf);
		Snmpv3BufferPut(0,dynTrapScopedPduBuf);
		Snmpv3BufferPut(0,dynTrapScopedPduBuf);
		Snmpv3BufferPut(0,dynTrapScopedPduBuf);
		Snmpv3BufferPut(1,dynTrapScopedPduBuf);

		// Put error status.
		Snmpv3BufferPut(ASN_INT,dynTrapScopedPduBuf);// Int type
		Snmpv3BufferPut(1,dynTrapScopedPduBuf); // One byte long.
		Snmpv3BufferPut(0,dynTrapScopedPduBuf); // Placeholder.

		// Similarly put error index.
		Snmpv3BufferPut(ASN_INT,dynTrapScopedPduBuf);// Int type
		Snmpv3BufferPut(1,dynTrapScopedPduBuf); // One byte long.
		Snmpv3BufferPut(0,dynTrapScopedPduBuf); // Placeholder.

		// Variable binding structure header
		Snmpv3BufferPut(STRUCTURE,dynTrapScopedPduBuf);
		varBindStructLenOffset = dynTrapScopedPduBuf->length;
		Snmpv3BufferPut(0x82,dynTrapScopedPduBuf);
		Snmpv3BufferPut(0,dynTrapScopedPduBuf);
		Snmpv3BufferPut(0,dynTrapScopedPduBuf);

		// Create variable name-pair structure
		Snmpv3BufferPut(STRUCTURE,dynTrapScopedPduBuf);
		varPairStructLenOffset = dynTrapScopedPduBuf->length;
		Snmpv3BufferPut(0,dynTrapScopedPduBuf);

		// Set 1st varbind object i,e sysUpTime.0 time stamp for the snmpv2 trap
		// Get complete notification variable OID string.

		Snmpv3BufferPut(ASN_OID,dynTrapScopedPduBuf);
		OIDLen = (uint8_t)sizeof(sysUpTime_oids);
		Snmpv3BufferPut((uint8_t)(OIDLen)+1,dynTrapScopedPduBuf);
		ptr = sysUpTime_oids;
		while( OIDLen-- )
			Snmpv3BufferPut(*ptr++,dynTrapScopedPduBuf);

		//1st varbind	 and this is a scalar object so index = 0
		Snmpv3BufferPut(0,dynTrapScopedPduBuf);

		// Time stamp
		Snmpv3BufferPut(SNMP_TIME_TICKS,dynTrapScopedPduBuf);
		Snmpv3BufferPut(4,dynTrapScopedPduBuf);
		Snmpv3BufferPut(SNMPNotifyInfo.timestamp.v[3],dynTrapScopedPduBuf);
		Snmpv3BufferPut(SNMPNotifyInfo.timestamp.v[2],dynTrapScopedPduBuf);
		Snmpv3BufferPut(SNMPNotifyInfo.timestamp.v[1],dynTrapScopedPduBuf);
		Snmpv3BufferPut(SNMPNotifyInfo.timestamp.v[0],dynTrapScopedPduBuf);

		tempOffset = dynTrapScopedPduBuf->length;
		//set the snmp time varbind trap offset 
		dynTrapScopedPduBuf->length = varPairStructLenOffset;

		/*// SNMP time stamp varbind length
		OIDLen = 2							// 1st varbind header 
		   + (uint8_t)sizeof(sysUpTime_oids)
		   + 1						   // index byte
		   + 6 ;						// time stamp */
		   
		Snmpv3BufferPut((tempOffset-varPairStructLenOffset)-1,dynTrapScopedPduBuf);
		//set the previous TX offset
		dynTrapScopedPduBuf->length = tempOffset;
					
		// Set 2nd varbind object i,e snmpTrapOID.0 for the snmpv2 trap
		// Get complete notification variable OID string.

		// Create variable name-pair structure
		Snmpv3BufferPut(STRUCTURE,dynTrapScopedPduBuf);
		varPairStructLenOffset = dynTrapScopedPduBuf->length;
		Snmpv3BufferPut(0,dynTrapScopedPduBuf);

		// Copy OID string into PDU.
		Snmpv3BufferPut(ASN_OID,dynTrapScopedPduBuf);
		OIDLen = (uint8_t)sizeof(snmptrap_oids);
		Snmpv3BufferPut((uint8_t)(OIDLen)+1,dynTrapScopedPduBuf);

		ptr = snmptrap_oids;
		while( OIDLen-- )
		Snmpv3BufferPut(*ptr++,dynTrapScopedPduBuf);

		//2nd varbind  and this is a scalar object so index = 0
		Snmpv3BufferPut(0,dynTrapScopedPduBuf);

		if (!GetOIDStringByID(SNMPNotifyInfo.trapIDVar, &rec, OIDValue, &OIDLen) )
		{
			MPFSClose(hMPFS);
			UDPClose(SNMPNotifyInfo.socket);
			return false;
		}
		Snmpv3BufferPut(ASN_OID,dynTrapScopedPduBuf);
		len = OIDLen;
		Snmpv3BufferPut(OIDLen,dynTrapScopedPduBuf);
		for(i=0;i<len;i++)
		{
			Snmpv3BufferPut(OIDValue[i],dynTrapScopedPduBuf);
		}
		tempOffset = dynTrapScopedPduBuf->length;
		//set the snmp varbind trap offset
		dynTrapScopedPduBuf->length = varPairStructLenOffset;
		// Snmp trap varbind length 
		/*OIDLen = 2					 // Agent ID header bytes
			+ (uint8_t)sizeof(snmptrap_oids)
			+ 1 					   // index byte
			+ 2 					 // header
			+ agentIDLen;				 // Agent ID bytes				  */
		Snmpv3BufferPut((tempOffset-varPairStructLenOffset)-1,dynTrapScopedPduBuf);
		//set the previous TX offset
		dynTrapScopedPduBuf->length = tempOffset;
#else
		// Put PDU type.  SNMP agent's response is always GET RESPONSE
		Snmpv3BufferPut(TRAP,dynTrapScopedPduBuf);
		pduStructLenOffset = dynTrapScopedPduBuf->length;
		Snmpv3BufferPut(0,dynTrapScopedPduBuf);
	
		// Get complete OID string from MPFS.
		if(!GetOIDStringByID(SNMPNotifyInfo.agentIDVar,&rec, OIDValue, &OIDLen))
		{
			return false;
		}
	
		if(!rec.nodeInfo.Flags.bIsAgentID )
		{
			return false;
		}
	
		MPFSSeek(hMPFS, rec.hData, MPFS_SEEK_START);
		Snmpv3BufferPut(ASN_OID,dynTrapScopedPduBuf);
		MPFSGet(hMPFS, &len);
		OIDLen = len;
		Snmpv3BufferPut(len,dynTrapScopedPduBuf);
		while( len-- )
		{
			uint8_t c;
			MPFSGet(hMPFS, &c);
			Snmpv3BufferPut(c,dynTrapScopedPduBuf);
		}
	
		// This agent's IP address.
		Snmpv3BufferPut(SNMP_IP_ADDR,dynTrapScopedPduBuf);
		Snmpv3BufferPut(4,dynTrapScopedPduBuf);
		Snmpv3BufferPut(AppConfig.MyIPAddr.v[0],dynTrapScopedPduBuf);
		Snmpv3BufferPut(AppConfig.MyIPAddr.v[1],dynTrapScopedPduBuf);
		Snmpv3BufferPut(AppConfig.MyIPAddr.v[2],dynTrapScopedPduBuf);
		Snmpv3BufferPut(AppConfig.MyIPAddr.v[3],dynTrapScopedPduBuf);
	
		// Geberic/Enterprise Trap code
		 Snmpv3BufferPut(ASN_INT,dynTrapScopedPduBuf);
		 Snmpv3BufferPut(1,dynTrapScopedPduBuf);
		 Snmpv3BufferPut(gGenericTrapNotification,dynTrapScopedPduBuf); 
	
		// Specific Trap code
		Snmpv3BufferPut(ASN_INT,dynTrapScopedPduBuf);
		Snmpv3BufferPut(1,dynTrapScopedPduBuf);
		Snmpv3BufferPut(SNMPNotifyInfo.notificationCode,dynTrapScopedPduBuf);
	
		// Time stamp
		Snmpv3BufferPut(SNMP_TIME_TICKS,dynTrapScopedPduBuf);
		Snmpv3BufferPut(4,dynTrapScopedPduBuf);
		Snmpv3BufferPut(SNMPNotifyInfo.timestamp.v[3],dynTrapScopedPduBuf);
		Snmpv3BufferPut(SNMPNotifyInfo.timestamp.v[2],dynTrapScopedPduBuf);
		Snmpv3BufferPut(SNMPNotifyInfo.timestamp.v[1],dynTrapScopedPduBuf);
		Snmpv3BufferPut(SNMPNotifyInfo.timestamp.v[0],dynTrapScopedPduBuf);
	
		// Variable binding structure header
		Snmpv3BufferPut(0x30,dynTrapScopedPduBuf);
		varBindStructLenOffset = dynTrapScopedPduBuf->length;
		Snmpv3BufferPut(0,dynTrapScopedPduBuf);
	
		// Create variable name-pair structure
		Snmpv3BufferPut(0x30,dynTrapScopedPduBuf);
		varPairStructLenOffset = dynTrapScopedPduBuf->length;
		Snmpv3BufferPut(0,dynTrapScopedPduBuf);
		 
		// Get complete notification variable OID string.
		if ( !GetOIDStringByID(var, &rec, OIDValue, &OIDLen) )
		{
			return false;
		}
	
		// Copy OID string into packet.
		Snmpv3BufferPut(ASN_OID,dynTrapScopedPduBuf);
		Snmpv3BufferPut((uint8_t)(OIDLen+1),dynTrapScopedPduBuf);
		len = OIDLen;
		ptr = OIDValue;
		while( len-- )
			Snmpv3BufferPut(*ptr++,dynTrapScopedPduBuf);
		Snmpv3BufferPut(index,dynTrapScopedPduBuf);
	
		// Encode and Copy actual data bytes
		if ( !GetDataTypeInfo(rec.dataType, &dataTypeInfo) )
		{
			return false;
		}
	
		Snmpv3BufferPut(dataTypeInfo.asnType,dynTrapScopedPduBuf);
	
	
		//Modified to Send trap even for  dataTypeInfo.asnType= ASCII_STRING, 
		//where dataTypeInfo.asnLen=0xff
		if ( dataTypeInfo.asnLen == 0xff )
		{
			dataTypeInfo.asnLen=0x4;
			val.dword=0;
		}	
		len = dataTypeInfo.asnLen;
		Snmpv3BufferPut(len,dynTrapScopedPduBuf);
		while( len-- )
			Snmpv3BufferPut(val.v[len],dynTrapScopedPduBuf);
	
		tempOffset = dynTrapScopedPduBuf->length;
		dynTrapScopedPduBuf->length = varPairStructLenOffset;
		varBindLen.Val = (tempOffset - varPairStructLenOffset)-1;
		Snmpv3BufferPut(varBindLen.v[0],dynTrapScopedPduBuf);
		
		dynTrapScopedPduBuf->length = varBindStructLenOffset;
		varBindLen.Val = (tempOffset - varBindStructLenOffset)-1;
		Snmpv3BufferPut(varBindLen.v[0],dynTrapScopedPduBuf);
	
		dynTrapScopedPduBuf->length = pduStructLenOffset;
		varBindLen.Val = (tempOffset - pduStructLenOffset)-1;
		Snmpv3BufferPut(varBindLen.v[0],dynTrapScopedPduBuf);
	
		dynTrapScopedPduBuf->length = 1;
		Snmpv3BufferPut(0x82,dynTrapScopedPduBuf);
		varBindLen.Val = tempOffset - 4; // equal to tempOffset - dynTrapScopedPduBuf->length
		Snmpv3BufferPut(varBindLen.v[1],dynTrapScopedPduBuf);
		Snmpv3BufferPut(varBindLen.v[0],dynTrapScopedPduBuf);
	
		dynTrapScopedPduBuf->length = tempOffset;
		
		pduStructLenOffset = 0;
		varBindStructLenOffset = 0;
		return true;
	
#endif
	}
	else
	{
		dynTrapScopedPduBuf = &gSNMPv3TrapScopedPduResponseBuf;
	}
	
	// Create variable name-pair structure
	Snmpv3BufferPut(STRUCTURE,dynTrapScopedPduBuf);
	varPairStructLenOffset = dynTrapScopedPduBuf->length;
	Snmpv3BufferPut(0,dynTrapScopedPduBuf);
	/* to send generic trap trap */
	if(gGenericTrapNotification != ENTERPRISE_SPECIFIC)
	{
		ptr = (uint8_t*)getSnmpV2GenTrapOid(gGenericTrapNotification,&OIDLen);			
		if(ptr == NULL)
		{
			//MPFSClose(hMPFS);
			//UDPClose(SNMPNotifyInfo.socket);
			return false;
		}
		// Copy OID string into PDU.
		Snmpv3BufferPut(ASN_OID,dynTrapScopedPduBuf);
		Snmpv3BufferPut((uint8_t)(OIDLen)+1,dynTrapScopedPduBuf);
		while( OIDLen-- )
		Snmpv3BufferPut(*ptr++,dynTrapScopedPduBuf);

		//2nd varbind  and this is a scalar object so index = 0
		Snmpv3BufferPut(0,dynTrapScopedPduBuf);
		// for microchip , SNMPNotifyInfo.agentIDVar == MICROCHIP
		if ( !GetOIDStringByID(SNMPNotifyInfo.agentIDVar, &rec, OIDValue, &OIDLen) )
		{
			MPFSClose(hMPFS);
			//UDPClose(SNMPNotifyInfo.socket);
			return false;
		}
		if ( !rec.nodeInfo.Flags.bIsAgentID )
		{
			//MPFSClose(hMPFS);
			//UDPClose(SNMPNotifyInfo.socket);
			return false;
		}

		MPFSSeek(hMPFS, rec.hData, MPFS_SEEK_START);

		Snmpv3BufferPut(ASN_OID,dynTrapScopedPduBuf);
		MPFSGet(hMPFS, &len);
		OIDLen = len;
		Snmpv3BufferPut(OIDLen,dynTrapScopedPduBuf);
		while( OIDLen-- )
		{
			uint8_t c;
			MPFSGet(hMPFS, &c);
			Snmpv3BufferPut(c,dynTrapScopedPduBuf);
		}
		tempOffset = dynTrapScopedPduBuf->length;
		//set the snmp varbind trap offset
		dynTrapScopedPduBuf->length = varPairStructLenOffset;
		/*OIDLen = 2 					 // Agent ID header bytes
			+ (uint8_t)sizeof(snmptrap_oids)
			+ 1 					   // index byte
			+ 2 					 // header
			+ agentIDLen;				 // Agent ID bytes				  */
		
		Snmpv3BufferPut((tempOffset-varPairStructLenOffset)-1,dynTrapScopedPduBuf);
		//set the previous TX offset
		dynTrapScopedPduBuf->length = tempOffset;
		// len = OIDLen;
	}
	else
	{
		// Get complete notification variable OID string.
		if ( !GetOIDStringByID(var, &rec, OIDValue, &OIDLen) )
		{
			//MPFSClose(hMPFS);
			//UDPClose(SNMPNotifyInfo.socket);
			return false;
		}		
		ptr = OIDValue;
	
		// Copy OID string into packet.
		Snmpv3BufferPut(ASN_OID,dynTrapScopedPduBuf);
		Snmpv3BufferPut((uint8_t)(OIDLen+1),dynTrapScopedPduBuf);
		len = OIDLen;
		while( len-- )
			Snmpv3BufferPut(*ptr++,dynTrapScopedPduBuf);
		Snmpv3BufferPut(index,dynTrapScopedPduBuf);

		// Encode and Copy actual data bytes
		if ( !GetDataTypeInfo(rec.dataType, &dataTypeInfo) )
		{
			//MPFSClose(hMPFS);
			//UDPClose(SNMPNotifyInfo.socket);
			return false;
		}
		Snmpv3BufferPut(dataTypeInfo.asnType,dynTrapScopedPduBuf);
	     //Modified to Send trap even for  dataTypeInfo.asnType= ASCII_STRING, 
		//where dataTypeInfo.asnLen=0xff
		if ( dataTypeInfo.asnLen == 0xff )
		{
			dataTypeInfo.asnLen=0x4;
			val.dword=0;
		}
		len = dataTypeInfo.asnLen;

		Snmpv3BufferPut(len,dynTrapScopedPduBuf);
		while( len-- )
			Snmpv3BufferPut(val.v[len],dynTrapScopedPduBuf);
	  
		/*len	 = dataTypeInfo.asnLen	// data bytes count
			 + 1                    // Length byte
			 + 1                    // Data type byte
			 + OIDLen               // OID bytes
			 + 2                    // OID header bytes
			 + 1;		            // index byte */
		tempOffset = dynTrapScopedPduBuf->length;
		dynTrapScopedPduBuf->length = varPairStructLenOffset;
		Snmpv3BufferPut((tempOffset-varPairStructLenOffset)-1,dynTrapScopedPduBuf);
		dynTrapScopedPduBuf->length = tempOffset;
	} 
	//set the previous TX offset

	if(gSetTrapSendFlag == true)
	{
		//MPFSClose(hMPFS);
		return true;
	}
	tempOffset = dynTrapScopedPduBuf->length;
	dynTrapScopedPduBuf->length = varBindStructLenOffset;
	Snmpv3BufferPut(0x82,dynTrapScopedPduBuf);
	varBindLen.Val = (tempOffset - varBindStructLenOffset)-3;
	Snmpv3BufferPut(varBindLen.v[1],dynTrapScopedPduBuf);
	Snmpv3BufferPut(varBindLen.v[0],dynTrapScopedPduBuf);

	dynTrapScopedPduBuf->length = pduStructLenOffset;
	Snmpv3BufferPut(0x82,dynTrapScopedPduBuf);
	varBindLen.Val = (tempOffset - pduStructLenOffset)-3;
	Snmpv3BufferPut(varBindLen.v[1],dynTrapScopedPduBuf);
	Snmpv3BufferPut(varBindLen.v[0],dynTrapScopedPduBuf);

	dynTrapScopedPduBuf->length = 1;
	Snmpv3BufferPut(0x82,dynTrapScopedPduBuf);
	varBindLen.Val = tempOffset - 4; // equal to tempOffset - dynTrapScopedPduBuf->length
	Snmpv3BufferPut(varBindLen.v[1],dynTrapScopedPduBuf);
	Snmpv3BufferPut(varBindLen.v[0],dynTrapScopedPduBuf);

	dynTrapScopedPduBuf->length = tempOffset;

	
    pduStructLenOffset = 0;
    varBindStructLenOffset = 0;
	return true;

}

/****************************************************************************
  Function:
	bool Snmpv3Notify(SNMP_ID var, SNMP_VAL val, SNMP_INDEX index,uint8_t targetIndex)

  Summary:
  	Creates and Sends SNMPv3 TRAP pdu.
	
  Description:
	This function creates SNMPv3 trap PDU and sends it to previously specified
	remoteHost.
	       
  Precondition:
	TRAP event is triggered.

  Parameters:
	var     - SNMP var ID that is to be used in notification
	val     - Value of var. Only value of uint8_t, uint16_t or uint32_t can be sent.
	index   - Index of var. If this var is a single,index would be 0, or else 
			  if this var Is a sequence, index could be any value 
			  from 0 to 127
	targetIndex -index of the 'gSnmpv3TrapConfigData' table's security user name 
  			     for which the TRAP PDU message header to constructed. 

  Return Values:
	true	-	if SNMP notification was successful sent.
	   			This does not guarantee that remoteHost recieved it.
	false	-	Notification sent failed.
  			    This would fail under following contions:
  			    1) Given SNMP_BIB_FILE does not exist in MPFS
  			    2) Given var does not exist.
  			    3) Previously given agentID does not exist
  			    4) Data type of given var is unknown - only
  			       possible if MPFS itself was corrupted.
 	SNMPV3_MSG_PRIV_FAIL -encryption of the trap msg failed
	SNMPV3_MSG_AUTH_FAIL - HAMC of the trap msg failed

  Remarks:
	None
 ***************************************************************************/
 
bool Snmpv3Notify(SNMP_ID var, SNMP_VAL val, SNMP_INDEX index,uint8_t targetIndex)
{
	TCPIP_UINT16_VAL	totaltrapLen={0};
	uint16_t		i=0;
	uint8_t		USM_Index=0;

	//SNMPV3MSGDATA tempScopeData = {NULL,0,0};

	//validate the trap user security name , message processing model 
	//, security model and the security level
	//if(validateTrapSecurityNameAndSecuityLevel(tragetIndex) != true)
	//	return false;
	gSNMPV3TrapSecurityLevel = Snmpv3GetTrapSecurityLevel(gSnmpv3TrapConfigData[targetIndex].securityLevelType);
	if( gSNMPV3TrapSecurityLevel == INVALID_MSG)
	{
		Snmpv3FreeDynAllocMem();
		UDPClose(SNMPNotifyInfo.socket);
		return false;
	}

	hMPFS = MPFSOpen((const uint8_t*)SNMP_BIB_FILE_NAME);
	if ( hMPFS == MPFS_INVALID_HANDLE )
	{
		UDPClose(SNMPNotifyInfo.socket);
		Snmpv3FreeDynAllocMem();
		return false;
	}

	if(gSNMPv3TrapMsgHeaderBuf.head == NULL)
	{
		if(Snmpv3TrapMsgHeaderPDU(targetIndex)!= true)
		{
			MPFSClose(hMPFS);
			UDPClose(SNMPNotifyInfo.socket);
			Snmpv3FreeDynAllocMem();
			return false;
		}		
	}
	if(Snmpv3TrapScopedpdu(var,val,index,targetIndex) != true)
	{
		MPFSClose(hMPFS);
		UDPClose(SNMPNotifyInfo.socket);
		Snmpv3FreeDynAllocMem();
		return false;
	}
	if(gSetTrapSendFlag == true)
	{
		MPFSClose(hMPFS);
		return true;
	}

	USM_Index = Snmpv3GetUserIndxFromUsmUserDB(targetIndex);
	if(USM_Index == INVALID_INDEX)
	{
		Snmpv3FreeDynAllocMem();
		MPFSClose(hMPFS);
		UDPClose(SNMPNotifyInfo.socket);
		return false;
	}
#if 1
	{
		uint8_t tempBuf[4];
		uint8_t tempCntr=0;
		uint8_t* tempPtr=NULL;
		uint8_t* outBufPtr=NULL;
		//uint16_t  tempScopedPduLen=0;
		

		totaltrapLen.Val = gSNMPv3TrapMsgHeaderBuf.length + gSNMPv3TrapScopedPduResponseBuf.length+3;
		//tempScopedPduLen = gSNMPv3TrapScopedPduResponseBuf.length-4; // 4 == STRUCTURE+0x82+len1+len2
		if((gSNMPV3TrapSecurityLevel & 0x02)==0x02) //Encrypted message	
		{
			tempPtr=tempBuf;
			*tempPtr++=0X04;
			if((gSNMPv3TrapScopedPduResponseBuf.length >= 0x80) && (gSNMPv3TrapScopedPduResponseBuf.length <= 0xFF))
			{
				*tempPtr++=0x81;
				*tempPtr=gSNMPv3TrapScopedPduResponseBuf.length;
				tempCntr=3; //0x04(encrypted pkt),0x81,len
			}
			else if((gSNMPv3TrapScopedPduResponseBuf.length > 0xFF) && (gSNMPv3TrapScopedPduResponseBuf.length < 0xFFFF))
			{			
				*tempPtr++=0x82;
				*tempPtr++=gSNMPv3TrapScopedPduResponseBuf.length>>8;
				*tempPtr=gSNMPv3TrapScopedPduResponseBuf.length;
				tempCntr=4; //0x04(encrypted pkt),0x81,len_1,len_0
			}
			else
			{
				*tempPtr=gSNMPv3TrapScopedPduResponseBuf.length;
				tempCntr=2; //0x04(encrypted pkt),len
			}
		}

		gSnmpV3TrapOUTPduWholeMsgBuf.wholeMsgLen.Val=(totaltrapLen.Val+tempCntr/*0x04,0x82,len_1,len_0*/);
		gSnmpV3TrapOUTPduWholeMsgBuf.wholeMsgHead=(uint8_t*)(malloc((size_t)gSnmpV3TrapOUTPduWholeMsgBuf.wholeMsgLen.Val+4+16));
		if(gSnmpV3TrapOUTPduWholeMsgBuf.wholeMsgHead == NULL)
		{
			MPFSClose(hMPFS);
			UDPClose(SNMPNotifyInfo.socket);
			Snmpv3FreeDynAllocMem();
			return false;
		}

		outBufPtr=gSnmpV3TrapOUTPduWholeMsgBuf.wholeMsgHead;	


		//Start Writing to the outPut Buffer

		*outBufPtr++=STRUCTURE;

		totaltrapLen.Val+=tempCntr;
					 
		if((totaltrapLen.Val >= 0x80) && (totaltrapLen.Val <= 0xFF))
		{
			*outBufPtr++=0x81;
			*outBufPtr++=totaltrapLen.Val;
		}
		else if((totaltrapLen.Val > 0xFF) && (totaltrapLen.Val < 0xFFFF))
		{			
			*outBufPtr++=0x82;
			*outBufPtr++=totaltrapLen.v[1];
			*outBufPtr++=totaltrapLen.v[0];
		}
		else
			*outBufPtr++=totaltrapLen.Val;

		*outBufPtr++=ASN_INT;
		*outBufPtr++=0x1;		
		*outBufPtr++=SNMP_V3;

		gSnmpV3TrapOUTPduWholeMsgBuf.msgAuthParamOffsetOutWholeMsg=(uint8_t*)(outBufPtr+gSNMPv3TrapMsgHeaderBuf.msgAuthParamOffset);
		//put global snmpv3 msg header 
		for(i=0;i<gSNMPv3TrapMsgHeaderBuf.length;i++)
		{
			*outBufPtr++=gSNMPv3TrapMsgHeaderBuf.head[i];
		}

		if (gSNMPv3TrapMsgHeaderBuf.head!=NULL)
		{	
			free(gSNMPv3TrapMsgHeaderBuf.head);
			gSNMPv3TrapMsgHeaderBuf.length=0x00;
			gSNMPv3TrapMsgHeaderBuf.head=NULL;
		}


		//Copy Scoped PDU to the Out Buffer
		if((gSNMPV3TrapSecurityLevel & 0x02)==0x02) //Encrypted message	
		{	//Copy Packet Auth indicator, length
			for(i=0;i<tempCntr;i++) 
			{
				*outBufPtr++=tempBuf[i];
			}
		}
		gSnmpV3TrapOUTPduWholeMsgBuf.scopedPduOffset=outBufPtr;
		gSnmpV3TrapOUTPduWholeMsgBuf.scopedPduStructLen=gSNMPv3TrapScopedPduResponseBuf.length;

		i=0;
		*outBufPtr++=gSNMPv3TrapScopedPduResponseBuf.head[i++];//0x30

		if(gSNMPv3TrapScopedPduResponseBuf.head[1] == 0x81)
		{
			*outBufPtr++=gSNMPv3TrapScopedPduResponseBuf.head[i++];//0x81
			*outBufPtr++=gSNMPv3TrapScopedPduResponseBuf.head[i++];//len_0
		}
		else if(gSNMPv3TrapScopedPduResponseBuf.head[1] == 0x82)
		{
			*outBufPtr++=gSNMPv3TrapScopedPduResponseBuf.head[i++]; //0x82
			*outBufPtr++=gSNMPv3TrapScopedPduResponseBuf.head[i++]; //len_1
			*outBufPtr++=gSNMPv3TrapScopedPduResponseBuf.head[i++]; //len_0
		}
		else
			*outBufPtr++=gSNMPv3TrapScopedPduResponseBuf.head[i++];//len_o

		// send context id and context name and the get response 
		// Authentication and privacy data packet will be sent from here onwards
		for(;i<(gSNMPv3TrapScopedPduResponseBuf.length);i++)
		{
			*outBufPtr++=gSNMPv3TrapScopedPduResponseBuf.head[i];
		}

		if(gSNMPv3TrapScopedPduResponseBuf.head != NULL)
		{
			free(gSNMPv3TrapScopedPduResponseBuf.head);
			gSNMPv3TrapScopedPduResponseBuf.length=0x00;
			gSNMPv3TrapScopedPduResponseBuf.head=NULL;
		}

		/*Encrypt the Response to the messgae originator*/
		if((gSNMPV3TrapSecurityLevel & 0x02)==0x02) //Encrypted message
		{
			uint8_t temp_usm_index = 0;
			/*Rxed SNMPv3 message is encrypted. Hence Response should be encrypted*/

			/*If user privacy protocol is AES*/
			temp_usm_index = gSnmpv3UserDBIndex; 
			gSnmpv3UserDBIndex = USM_Index;
			if(Snmpv3AESEncryptResponseScopedPdu(&gSnmpV3TrapOUTPduWholeMsgBuf) != SNMPV3_MSG_PRIV_PASS)
			{
				gSnmpv3UserDBIndex = temp_usm_index;
				MPFSClose(hMPFS);
				UDPClose(SNMPNotifyInfo.socket);
				Snmpv3FreeDynAllocMem();
				return SNMPV3_MSG_PRIV_FAIL;
			}
			gSnmpv3UserDBIndex = temp_usm_index;
			/*If user privacy Protocol is DES*/
			//snmpV3DESDecryptRxedScopedPdu();
		}

		/* Authenticate the whole message to be transmitted*/
		if((gSNMPV3TrapSecurityLevel & 0x01)==0x01) //Authenticatd message
		{
			uint8_t temp_usm_index = 0;
			/*Rxed SNMPv3 message is Authenticated.Send authenticatin parameters for the Response*/
			gSnmpV3TrapOUTPduWholeMsgBuf.wholeMsgLen.Val = outBufPtr - gSnmpV3TrapOUTPduWholeMsgBuf.wholeMsgHead;
			/*If user authentication is HAMC-MD5-96*/
			temp_usm_index = gSnmpv3UserDBIndex; 
			gSnmpv3UserDBIndex = USM_Index;
			if(Snmpv3AuthenticateTxPduForDataIntegrity(&gSnmpV3TrapOUTPduWholeMsgBuf)!=SNMPV3_MSG_AUTH_PASS)
			{
				gSnmpv3UserDBIndex = temp_usm_index;
				MPFSClose(hMPFS);
				UDPClose(SNMPNotifyInfo.socket);
				Snmpv3FreeDynAllocMem();
				return SNMPV3_MSG_AUTH_FAIL;
			}
			gSnmpv3UserDBIndex = temp_usm_index;
			tempPtr = outBufPtr;
			outBufPtr=gSnmpV3TrapOUTPduWholeMsgBuf.msgAuthParamOffsetOutWholeMsg;
			for(i=0;i<snmpOutMsgAuthParamLen;i++)
				*outBufPtr++=snmpOutMsgAuthParamStrng[i];
			outBufPtr = tempPtr;
		}

		
		_SNMPDuplexInit(SNMPNotifyInfo.socket);
		//total length number of bytes need to be passed to the wire
		gSnmpV3TrapOUTPduWholeMsgBuf.wholeMsgLen.Val = outBufPtr - gSnmpV3TrapOUTPduWholeMsgBuf.wholeMsgHead;
		//outBufPtr=gSnmpV3TrapOUTPduWholeMsgBuf.wholeMsgHead;
		i=0;
		_SNMPPut(gSnmpV3TrapOUTPduWholeMsgBuf.wholeMsgHead[i++]);		//0x30

		if(gSnmpV3TrapOUTPduWholeMsgBuf.wholeMsgHead[i] == 0x81)
		{
			_SNMPPut(gSnmpV3TrapOUTPduWholeMsgBuf.wholeMsgHead[i++]);//0x81
			_SNMPPut(gSnmpV3TrapOUTPduWholeMsgBuf.wholeMsgHead[i++]);//len_0
		}
		else if(gSnmpV3TrapOUTPduWholeMsgBuf.wholeMsgHead[i] == 0x82)
		{
			_SNMPPut(gSnmpV3TrapOUTPduWholeMsgBuf.wholeMsgHead[i++]); //0x82
			_SNMPPut(gSnmpV3TrapOUTPduWholeMsgBuf.wholeMsgHead[i++]); //len_1
			_SNMPPut(gSnmpV3TrapOUTPduWholeMsgBuf.wholeMsgHead[i++]); //len_0
		}
		else
			_SNMPPut(gSnmpV3TrapOUTPduWholeMsgBuf.wholeMsgHead[i++]);//len_o
			
		for(;i<gSnmpV3TrapOUTPduWholeMsgBuf.wholeMsgLen.Val;i++)
		{	
			_SNMPPut(gSnmpV3TrapOUTPduWholeMsgBuf.wholeMsgHead[i]);
		}

		//if(gSNMPv3ScopedPduResponseBuf.head != NULL)
		if(gSnmpV3TrapOUTPduWholeMsgBuf.wholeMsgHead != NULL)
		{
			free(gSnmpV3TrapOUTPduWholeMsgBuf.wholeMsgHead);
			gSnmpV3TrapOUTPduWholeMsgBuf.wholeMsgHead=0x00;
			gSnmpV3TrapOUTPduWholeMsgBuf.wholeMsgLen.Val=0x00;
			gSnmpV3TrapOUTPduWholeMsgBuf.snmpMsgHead = NULL;
			gSnmpV3TrapOUTPduWholeMsgBuf.msgAuthParamOffsetOutWholeMsg = NULL;
			gSnmpV3TrapOUTPduWholeMsgBuf.scopedPduOffset =	NULL;			
		}

		MPFSClose(hMPFS);
		UDPFlush(SNMPNotifyInfo.socket);
		UDPClose(SNMPNotifyInfo.socket);
		SNMPNotifyInfo.socket =  INVALID_UDP_SOCKET;

		Snmpv3FreeDynAllocMem();
	}
#endif
	return true;
}
#endif


#endif // #if defined(TCPIP_STACK_USE_SNMPV3_SERVER)
