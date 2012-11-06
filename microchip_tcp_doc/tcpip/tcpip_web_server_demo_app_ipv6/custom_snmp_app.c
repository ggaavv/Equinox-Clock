/*******************************************************************************
  Application to Demo SNMP Server

  Summary:
    Support for SNMP module in Microchip TCP/IP Stack
    
  Description:
    - Implements the SNMP application
*******************************************************************************/

/*******************************************************************************
FileName:  CustomSNMPApp.c 
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

#define __CUSTOMSNMPAPP_C

#include "tcpip_config.h"

#if defined(TCPIP_STACK_USE_SNMP_SERVER)

#include "tcpip/tcpip.h"
#include "main_demo.h"



/****************************************************************************
  Section:
	Global Variables
  ***************************************************************************/
/*This Macro is used to provide maximum try for a failure Trap server address  */
#define MAX_TRY_TO_SEND_TRAP (10)

uint8_t gSendTrapFlag=false;//global flag to send Trap
uint8_t gOIDCorrespondingSnmpMibID=MICROCHIP;
uint8_t gGenericTrapNotification=ENTERPRISE_SPECIFIC;
uint8_t gSpecificTrapNotification=VENDOR_TRAP_DEFAULT; // Vendor specific trap code

#if defined(SNMP_STACK_USE_V2_TRAP)
//if gSetTrapSendFlag == false then the last varbind variable for 
//multiple varbind variable pdu structure or if there is only varbind variable send.
// if gSetTrapSendFlag == true, then v2 trap pdu is expecting more varbind variable.
uint8_t	gSetTrapSendFlag = false;
#endif /* SNMP_STACK_USE_V2_TRAP */

NET_CONFIG*     gpSnmpIf;   // interface we use for the SNMP

/*Initialize trap table with no entries.*/
TRAP_INFO trapInfo = { TRAP_TABLE_SIZE };

static SYS_TICK SNMPGetTimeStamp(void);
static bool SendNotification(uint8_t receiverIndex, SNMP_ID var, SNMP_VAL val);

/****************************************************************************
  ===========================================================================
  Section:
	SNMP Routines
  ===========================================================================
  ***************************************************************************/



#if !defined(SNMP_TRAP_DISABLED)
/****************************************************************************
  Function:
 	 bool SendNotification(uint8_t receiverIndex, SNMP_ID var, SNMP_VAL val)
 
  Summary:			
	Prepare, validate remote node which will receive trap and send trap pdu.
 	  	  
  Description:		
    This routine prepares the trap notification pdu, sends ARP and get
	remote device MAC address to which notification to sent, sends
	the notification. 
	Notofication state machine is getting updated if there is any ARP resolution failure for 
	a perticular trap destination address.
	
  PreCondition:
	SNMPTrapDemo() is called.
 	
  parameters:
     receiverIndex - The index to array where remote ip address is stored.  
     var		   - SNMP var ID that is to be used in notification
	 val           - Value of var. Only value of uint8_t, uint16_t or uint32_t can be sent.
	 
  Return Values:          
 	 true	-	If notification send is successful.
 	 false	-	If send notification failed.
 	 
  Remarks:
     None.
 *************************************************************************/
static bool SendNotification(uint8_t receiverIndex, SNMP_ID var, SNMP_VAL val)
{
    static enum { SM_PREPARE, SM_NOTIFY_WAIT } smState = SM_PREPARE;
    IP_ADDR IPAddress;
	static uint8_t tempRxIndex;
	static IP_ADDR tempIpAddress;
	
    // Convert local to network order.
    IPAddress.v[0] = trapInfo.table[receiverIndex].IPAddress.v[3];
    IPAddress.v[1] = trapInfo.table[receiverIndex].IPAddress.v[2];
    IPAddress.v[2] = trapInfo.table[receiverIndex].IPAddress.v[1];
    IPAddress.v[3] = trapInfo.table[receiverIndex].IPAddress.v[0];

    switch(smState)
    {
	    case SM_PREPARE:

			tempRxIndex=receiverIndex;			
			// Convert local to network order.
			tempIpAddress.v[0] = trapInfo.table[receiverIndex].IPAddress.v[3];
			tempIpAddress.v[1] = trapInfo.table[receiverIndex].IPAddress.v[2];
			tempIpAddress.v[2] = trapInfo.table[receiverIndex].IPAddress.v[1];
			tempIpAddress.v[3] = trapInfo.table[receiverIndex].IPAddress.v[0];
	        SNMPNotifyPrepare(&IPAddress,
	                          trapInfo.table[receiverIndex].community,
	                          trapInfo.table[receiverIndex].communityLen,
	                          MICROCHIP,            	   // Agent ID Var
	                          gSpecificTrapNotification,   // Specifc Trap notification code
	                          SNMPGetTimeStamp());
	        smState = SM_NOTIFY_WAIT;
	
	        break;
	
	    case SM_NOTIFY_WAIT:
				
	        if ( SNMPIsNotifyReady(&IPAddress) )
	        {
	            smState = SM_PREPARE;
	            SNMPNotify(var, val, 0);
	            return true;
	        }
			/* if trapInfo table address for a perticular index is different comparing to the SM_PREPARE IP address
				then change the state to SM_PREPARE*/
			if((tempIpAddress.Val != IPAddress.Val) && 
					(tempRxIndex == receiverIndex))
			{
				smState = SM_PREPARE;
			}
			/* Change state machine from SM_NOTIFY_WAIT to SM_PREPARE if incoming trap destination 
			index is different from the SM_PREPARE  trap destination index*/
			if(tempRxIndex != receiverIndex)
				smState=SM_PREPARE;
			
    }
    return false;
}

#if defined(SNMP_STACK_USE_V2_TRAP)
/**************************************************************************
  Function:
 	void SNMPV2TrapDemo(void)
 
  Summary:	
  	Send SNMP V2 notification with multiple varbinds.
 	  	  
  Description:		
	This routine sends a trap v2 pdu with multiple varbind variables
	for the predefined ip addresses with the agent. And as per RFC1905 
	the first two variable bindings in the varbind pdu list of an
   	SNMPv2-Trap-PDU are sysUpTime.0 and snmpTrapOID.0 respectively.
   	To support multiple varbind, user need to call SendNotification()
    for the first varbind variable and SendNotification() will do the 
    arp resolve and adds sysUpTime.0 and snmpTrapOID.0 variable to 
    the pdu. For the second varbind variable onwards user need to 
   	call only SNMPNotify().
	In this demo , snmpv2 trap includes ANALOG_POT0,PUSH_BUTTON and LED_D5
	variable bindains and this trap can be generated by using portmeter value.
	and SNMPv2-Trap-PDU will be generated only when pot meter reading exceeds 501.
	
	gSetTrapSendFlag Should be set to true when user is trying to send first 
	variable binding and gSetTrapSendFlag should be set to false before 
    sending the last variable binding.

	* if user is sending only one variable binding then 
	* gSetTrapSendFlag should be set to False.	
    * user can add more variable bindings.
  PreCondition:
 	Application defined event occurs to send the trap.
 	
  parameters:
     None.
 
  Returns:          
 	 None.
 
  Remarks:
    This routine guides how to build a event generated trap notification.
 *************************************************************************/
void SNMPV2TrapDemo(void)
{
	static SYS_TICK tempTimerRead = 0;
	static uint8_t	trapIndex=0;
	static SNMP_VAL		analogPotVal;
	static uint8_t potReadLock = false;
	static uint8_t timeLock = false;
	static uint8_t maxTryToSendTrap=0;
	
	if(timeLock==(uint8_t)false)
	{
		tempTimerRead=SYS_TICK_Get();
		timeLock=true;
	}

    gpSnmpIf = TCPIP_STACK_GetDefaultNet();

	for(;trapIndex<TRAP_TABLE_SIZE;trapIndex++)
	{	
		if(!trapInfo.table[trapIndex].Flags.bEnabled)
			continue;
		
		//Read POT reading once and send trap to all configured recipient
		if(potReadLock ==(uint8_t)false)
		{
			analogPotVal.word= (uint16_t)ADC1BUF0;
			potReadLock    = true;
		}
	
		if(analogPotVal.word >512u)
		{
			/*
			 * prepare  and send multivarbind pdu using pot meter value. 
			 * for SNMP v2 trap sysUpTime.0 and SNMPv2TrapOID.0 are mandatory
			 * apart from these varbinds, push button and potmeter OID are included
			 * to this pdu.
			*/
			gSpecificTrapNotification = 1; //expecting 1 should be the specific trap.
			gGenericTrapNotification = ENTERPRISE_SPECIFIC;
			gSetTrapSendFlag = true;
			// insert ANALOG_POT0 OID value and OID to the varbind pdu
			//set global flag gSetTrapSendFlag to true , it signifies that there are more than one 
			// variable need to be the part of SNMP v2 TRAP. 
			// if there is  only varbind variable to be the part of SNMP v2 trap, 
			// then user should set gSetTrapSendFlag to false.
			//gSetTrapSendFlag = false;

			if(SendNotification(trapIndex,ANALOG_POT0,analogPotVal) == false)
			{
				if(maxTryToSendTrap >= MAX_TRY_TO_SEND_TRAP)
				{
					trapIndex++;
					maxTryToSendTrap = 0;
					return;
				}
				maxTryToSendTrap++;
				return ;
			}
			//prepare PUSH_BUTTON trap .for the next trap varbind we need to use snmp_notify instead of 
			// SendNotification(), because we have already prepared SNMP v2 trap header 
			//and arp has been resolved already.
			
			analogPotVal.byte = BUTTON0_IO;
			SNMPNotify(PUSH_BUTTON,analogPotVal,0);
		
			// if this is the last trap variable need to be the part of SNMP v2 Trap,
			// then we should disable gSetTrapSendFlag to false
			gSetTrapSendFlag = false;
			analogPotVal.byte = LED0_IO;
			SNMPNotify(LED_D5,analogPotVal,0);			
		}
	}

	//Try for max 5 seconds to send TRAP, do not get block in while()
	if((SYS_TICK_Get()-tempTimerRead)>(5*SYS_TICK_TicksPerSecondGet()))
	{
//		UDPDiscard(s, gpSnmpIf);
		potReadLock = false;
		timeLock = false;
		trapIndex = 0;
		analogPotVal.word = 0;
		return;
	}
}
#endif /* SNMP_STACK_USE_V2_TRAP */

/**************************************************************************
  Function:
 	void SNMPTrapDemo(void)
 
  Summary:	
  	Send trap pdu demo application.
 	  	  
  Description:		
	This routine sends a trap pdu for the predefined ip addresses with the
	agent. The "event" to generate this trap pdu is "BUTTON_PUSH_EVENT". Whenever
	there occurs a specific button push, this routine is called and sends
	a trap containing PUSH_BUTTON mib var OID and notification type 
	as authentication failure. 
       
  PreCondition:
 	Application defined event occurs to send the trap.
 	
  parameters:
     None.
 
  Returns:          
 	 None.
 
  Remarks:
    This routine guides how to build a event generated trap notification.
    The application should make use of SNMPSendTrap() routine to generate 
    and send trap.
 *************************************************************************/
void SNMPTrapDemo(void)
{
	static SYS_TICK TimerRead=0;
	static bool analogPotNotify = false,buttonPushNotify=false;
	static uint8_t anaPotNotfyCntr=0,buttonPushNotfyCntr=0;
	static SNMP_VAL buttonPushval,analogPotVal;
	static uint8_t potReadLock=false,buttonLock=false;
	static uint8_t timeLock=false;
	static uint8_t maxTryToSendTrap=0;
	
	if(timeLock==(uint8_t)false)
	{
		TimerRead=SYS_TICK_Get();
		timeLock=true;
	}

	#if 1
	if(anaPotNotfyCntr >= trapInfo.Size)
	{
		anaPotNotfyCntr = 0;
		potReadLock=false;
		//analogPotNotify = false;
		analogPotNotify = true;
	}
	#endif
	
	if(!analogPotNotify)
	{
		//Read POT reading once and send trap to all configured recipient
		if(potReadLock ==(uint8_t)false)
		{
            analogPotVal.word= (uint16_t)ADC1BUF0;
			
			//Avoids Reading POT for every iteration unless trap sent to each configured recipients 
			potReadLock=true; 
		}
		if(trapInfo.table[anaPotNotfyCntr].Flags.bEnabled)
		{
			if(analogPotVal.word >512u)
			{
				gSpecificTrapNotification=POT_READING_MORE_512;
				gGenericTrapNotification=ENTERPRISE_SPECIFIC;
				if(SendNotification(anaPotNotfyCntr, ANALOG_POT0, analogPotVal))
					anaPotNotfyCntr++;
				else
				{					
					if(maxTryToSendTrap>=MAX_TRY_TO_SEND_TRAP)
					{
						anaPotNotfyCntr++;
						maxTryToSendTrap = 0;
						return;
					}
					maxTryToSendTrap++;
					return ;
				}
			}
		}
		else
			anaPotNotfyCntr++;
			
	}


	if(buttonPushNotfyCntr==trapInfo.Size)
	{
		buttonPushNotfyCntr = 0;
		buttonLock=false;
		buttonPushNotify = false;
	}


	if(buttonLock == (uint8_t)false)
	{
		if(BUTTON3_IO == 0u)
		{
			buttonPushNotify = true;
			buttonLock =true;
		}
	}

	if(buttonPushNotify)
	{			  
		buttonPushval.byte = 0;
		if ( trapInfo.table[buttonPushNotfyCntr].Flags.bEnabled )
		{
			gSpecificTrapNotification=BUTTON_PUSH_EVENT;
			gGenericTrapNotification=ENTERPRISE_SPECIFIC;
			if(SendNotification(buttonPushNotfyCntr, PUSH_BUTTON, buttonPushval))
				buttonPushNotfyCntr++;
		}
		else
			buttonPushNotfyCntr++;
	}

	//Try for max 5 seconds to send TRAP, do not get block in while()
	if((SYS_TICK_Get()-TimerRead)>(5*SYS_TICK_TicksPerSecondGet()))
	{
		UDPDiscard(s, gpSnmpIf);    
		buttonPushNotfyCntr = 0;
		buttonLock=false;
		buttonPushNotify = false;
		anaPotNotfyCntr = 0;
		potReadLock=false;
		analogPotNotify = false;
		timeLock=false;
		gSpecificTrapNotification=VENDOR_TRAP_DEFAULT;
		gGenericTrapNotification=ENTERPRISE_SPECIFIC;
		return;
	}

}


/**************************************************************************
  Function:
 	void SNMPSendTrap(void)
 
  Summary:	
  	 Prepare, validate remote node which will receive trap and send trap pdu.
 	 	  
  Description:		
     This function is used to send trap notification to previously 
     configured ip address if trap notification is enabled. There are
     different trap notification code. The current implementation
     sends trap for authentication failure (4).
  
  PreCondition:
 	 If application defined event occurs to send the trap.
 
  parameters:
     None.
 
  Returns:          
 	 None.
 
  Remarks:
     This is a callback function called by the application on certain 
     predefined events. This routine only implemented to send a 
     authentication failure Notification-type macro with PUSH_BUTTON
     oid stored in MPFS. If the ARP is no resolved i.e. if 
     SNMPIsNotifyReady() returns false, this routine times 
     out in 5 seconds. This routine should be modified according to 
     event occured and should update corrsponding OID and notification
     type to the trap pdu.
 *************************************************************************/
void SNMPSendTrap(void)
{
	static uint8_t timeLock=false;
	static uint8_t receiverIndex=0; ///is application specific
	IP_ADDR remHostIPAddress,* remHostIpAddrPtr;
	SNMP_VAL val;
	static SYS_TICK TimerRead;
    
	static enum 
	{
		SM_PREPARE,
		SM_NOTIFY_WAIT 
	} smState = SM_PREPARE;

    gpSnmpIf = TCPIP_STACK_GetDefaultNet();

	if(trapInfo.table[receiverIndex].Flags.bEnabled)
	{
		remHostIPAddress.v[0] = trapInfo.table[receiverIndex].IPAddress.v[3];
		remHostIPAddress.v[1] = trapInfo.table[receiverIndex].IPAddress.v[2];
		remHostIPAddress.v[2] = trapInfo.table[receiverIndex].IPAddress.v[1];
		remHostIPAddress.v[3] = trapInfo.table[receiverIndex].IPAddress.v[0];
		remHostIpAddrPtr = &remHostIPAddress;
		if(timeLock==(uint8_t)false)
		{
			TimerRead=SYS_TICK_Get();
			timeLock=true;
		}
	}	
	else
	{
		receiverIndex++;
		if((receiverIndex == (uint8_t)TRAP_TABLE_SIZE))
		{
			receiverIndex=0;
			timeLock=false;
			gSendTrapFlag=false;	
			UDPDiscard(s, gpSnmpIf);
		}
		return;
		
	}
		
	switch(smState)
	{
	
		case SM_PREPARE:

			SNMPNotifyPrepare(remHostIpAddrPtr,trapInfo.table[receiverIndex].community,
						trapInfo.table[receiverIndex].communityLen,
						MICROCHIP,			  // Agent ID Var
						gSpecificTrapNotification,					  // Notification code.
						SNMPGetTimeStamp());
			smState++;
			break;
			
		case SM_NOTIFY_WAIT:
			if(SNMPIsNotifyReady(remHostIpAddrPtr))
			{
				smState = SM_PREPARE;
		 		val.byte = 0;
				receiverIndex++;

				//application has to decide on which SNMP var OID to send. Ex. PUSH_BUTTON	
				SNMPNotify(gOIDCorrespondingSnmpMibID, val, 0);
            	smState = SM_PREPARE;
				UDPDiscard(s, gpSnmpIf);
				break;
			}
	}	
		
	//Try for max 5 seconds to send TRAP, do not get block in while()
	if((SYS_TICK_Get()-TimerRead)>(5*SYS_TICK_TicksPerSecondGet())|| (receiverIndex == (uint8_t)TRAP_TABLE_SIZE))
	{
		UDPDiscard(s, gpSnmpIf);
		smState = SM_PREPARE;
		receiverIndex=0;
		timeLock=false;
		gSendTrapFlag=false;
		return;
	}

}

#endif

/*********************************************************************
  Function:
 	 uint8_t SNMPValidateCommunity(uint8_t* community)
 
  Summary:			
 	 Validates community name for access control. 
 
  Description:		
     This function validates the community name for the mib access to NMS.
 	 The snmp community name received in the request pdu is validated for
 	 read and write community names. The agent gives an access to the mib
 	 variables only if the community matches with the predefined values.
  	 This routine also sets a gloabal flag to send trap if authentication
 	 failure occurs.
  
  PreCondition:
 	 SNMPInit is already called.
 
  parameters:
     community - Pointer to community string as sent by NMS.
 
  Returns:          
 	 This routine returns the community validation result as 
  	 READ_COMMUNITY or WRITE_COMMUNITY or INVALID_COMMUNITY	
 
  Remarks:
     This is a callback function called by module. User application must 
  	 implement this function and verify that community matches with 
 	 predefined value. This validation occurs for each NMS request.
 ********************************************************************/
uint8_t SNMPValidateCommunity(uint8_t * community)
{
	uint8_t i;
	uint8_t *ptr;
    NET_CONFIG* pConfig;
    
	/*
	If the community name is encrypted in the request from the Manager,
	agent required to decrypt it to match with the community it is
	configured for. The response from the agent should contain encrypted community 
	name using the same encryption algorithm which Manager used while
	making the request.
	*/ 		

	// Validate that community string is a legal size
	if(strlen((char*)community) <= SNMP_COMMUNITY_MAX_LEN)
	{
        pConfig = TCPIP_STACK_GetDefaultNet();
		// Search to see if this is a write community.  This is done before 
		// searching read communities so that full read/write access is 
		// granted if a read and write community name happen to be the same.
		for(i = 0; i < SNMP_MAX_COMMUNITY_SUPPORT; i++)
		{
			ptr = pConfig->writeCommunity[i];
			if(ptr == NULL)
				continue;
			if(*ptr == 0x00u)
				continue;
			if(strncmp((char*)community, (char*)ptr, SNMP_COMMUNITY_MAX_LEN) == 0)
				return WRITE_COMMUNITY;
		}
		
		// Did not find in write communities, search read communities
		for(i = 0; i < SNMP_MAX_COMMUNITY_SUPPORT; i++)
		{
			ptr = pConfig->readCommunity[i];
			if(ptr == NULL)
				continue;
			if(*ptr == 0x00u)
				continue;
			if(strncmp((char*)community, (char*)ptr, SNMP_COMMUNITY_MAX_LEN) == 0)
				return READ_COMMUNITY;
		}
	}
	
	// Could not find any matching community, set up to send a trap
	gSpecificTrapNotification=VENDOR_TRAP_DEFAULT;
	gGenericTrapNotification=AUTH_FAILURE;
	gSendTrapFlag=true;
	return INVALID_COMMUNITY;
	
}

/*********************************************************************
  Function:
  	bool SNMPIsValidSetLen(SNMP_ID var, uint8_t len)

  Summary: 	
	Validates the set variable data length to data type.
	
  Description:
  	This routine is used to validate the dyanmic variable data length
  	to the variable data type. It is used when SET request is processed.
  	This is a callback function called by module. User application
  	must implement this function.
  	
  PreCondition:
  	ProcessSetVar() is called.
 
  Parameters:  
  	var	-	Variable id whose value is to be set
  	len	-	Length value that is to be validated.
 
  Return Values:  
  	true  - if given var can be set to given len
    false - if otherwise.
 
  Remarks:
  	This function will be called for only dynamic variables that are
  	defined as ASCII_STRING and OCTET_STRING (i.e. data length greater
  	than 4 bytes)
 ********************************************************************/
bool SNMPIsValidSetLen(SNMP_ID var, uint8_t len)
{
    switch(var)
    {
    case TRAP_COMMUNITY:
        if ( len < (uint8_t)TRAP_COMMUNITY_MAX_LEN+1 )
            return true;
        break;

#if defined(SYS_OUT_ENABLE)
    case LCD_DISPLAY:
        if ( len < SYS_OUT_MESSAGE_LINE_COUNT() * SYS_OUT_MESSAGE_LINE_LENGTH() + 1 )
            return true;
        break;
#endif
    }
    return false;
}

/*********************************************************************
  Function:  
 	bool SNMPSetVar(SNMP_ID var, SNMP_INDEX index,
                                   uint8_t ref, SNMP_VAL val)
 
  Summary:
  	This routine Set the mib variable with the requested value.
 
  Description:
  	This is a callback function called by module for the snmp
  	SET request.User application must modify this function 
 	for the new variables address.

  Precondition:
 	ProcessVariables() is called.
 	
  Parameters:        
    var	-	Variable id whose value is to be set

    ref -   Variable reference used to transfer multi-byte data
            0 if first byte is set otherwise nonzero value to indicate
            corresponding byte being set.
            
    val -   Up to 4 byte data value.
            If var data type is uint8_t, variable
               value is in val->byte
            If var data type is uint16_t, variable
               value is in val->word
            If var data type is uint32_t, variable
               value is in val->dword.
            If var data type is IP_ADDRESS, COUNTER32,
               or GAUGE32, value is in val->dword
            If var data type is OCTET_STRING, ASCII_STRING
               value is in val->byte; multi-byte transfer
               will be performed to transfer remaining
               bytes of data.
 
  Return Values:  
  	true	-	if it is OK to set more byte(s).
    false	-	if otherwise.
 
  Remarks: 
  	This function may get called more than once depending on number 
	of bytes in a specific set request for given variable.
	only dynamic read-write variables needs to be handled.
********************************************************************/
bool SNMPSetVar(SNMP_ID var, SNMP_INDEX index, uint8_t ref, SNMP_VAL val)
{
    int lcdSize = 16*2+1;
    char message[lcdSize];

    switch(var)
    {
    case LED_D5:
        LED2_IO = val.byte;
        return true;

    case LED_D6:
        LED1_IO = val.byte;
        return true;

    case TRAP_RECEIVER_IP:
        // Make sure that index is within our range.
        if ( index < trapInfo.Size )
        {
            // This is just an update to an existing entry.
            trapInfo.table[index].IPAddress.Val = val.dword;
            return true;
        }
        else if ( index < (uint8_t)TRAP_TABLE_SIZE )
        {
            // This is an addition to table.
            trapInfo.table[index].IPAddress.Val = val.dword;
            trapInfo.table[index].communityLen = 0;
            trapInfo.Size++;
            return true;
        }
        break;

    case TRAP_RECEIVER_ENABLED:
        // Make sure that index is within our range.
        if ( index < trapInfo.Size )
        {
            // Value of '1' means Enabled".
            if ( val.byte == 1u )
                trapInfo.table[index].Flags.bEnabled = 1;
            // Value of '0' means "Disabled.
            else if ( val.byte == 0u )
                trapInfo.table[index].Flags.bEnabled = 0;
            else
                // This is unknown value.
                return false;
            return true;
        }
        // Given index is more than our current table size.
        // If it is within our range, treat it as an addition to table.
        else if ( index < (uint8_t)TRAP_TABLE_SIZE )
        {
            // Treat this as an addition to table.
            trapInfo.Size++;
            trapInfo.table[index].communityLen = 0;
        }

        break;

    case TRAP_COMMUNITY:
        // Since this is a ASCII_STRING data type, SNMP will call with
        // SNMP_END_OF_VAR to indicate no more bytes.
        // Use this information to determine if we just added new row
        // or updated an existing one.
        if ( ref ==  SNMP_END_OF_VAR )
        {
            // Index equal to table size means that we have new row.
            if ( index == trapInfo.Size )
                trapInfo.Size++;

            // Length of string is one more than index.
            trapInfo.table[index].communityLen++;

            return true;
        }

        // Make sure that index is within our range.
        if ( index < trapInfo.Size )
        {
            // Copy given value into local buffer.
            trapInfo.table[index].community[ref] = val.byte;
            // Keep track of length too.
            // This may not be NULL terminate string.
            trapInfo.table[index].communityLen = (uint8_t)ref;
            return true;
        }
        break;

    case LCD_DISPLAY:
        // Copy all bytes until all bytes are transferred
        if ( ref != SNMP_END_OF_VAR && ref+1 < lcdSize)
        {
            message[ref] = val.byte;
            message[ref+1] = 0;
        }
        else
        {
            SYS_OUT_MESSAGE(message);
        }

        return true;

    }

    return false;
}


/*********************************************************************
  Function:        
  	bool SNMPGetExactIndex(SNMP_ID var,SNMP_INDEX index)

  Summary:
  	To search for exact index node in case of a Sequence variable.
	
  Description:    
  	This is a callback function called by SNMP module.
    SNMP user must implement this function in user application and 
    provide appropriate data when called.  This function will only
    be called for OID variable of type sequence.
    
  PreCondition: 
  	None
 
  Parameters:
  	var		-	Variable id as per mib.h (input)
  	index      -	 Index of variable (input)
 
  Return Values:
  	true	-	 If the exact index value exists for given variable at given
                 index.
    false	-	 Otherwise.
 
  Remarks:
	  Only sequence index needs to be handled in this function.
 ********************************************************************/
bool SNMPGetExactIndex(SNMP_ID var, SNMP_INDEX index)
{
    
    switch(var)
    {
    case TRAP_RECEIVER_ID:
    case TRAP_RECEIVER_ENABLED:
	case TRAP_RECEIVER_IP:
	case TRAP_COMMUNITY:
        // There is no next possible index if table itself is empty.
        if ( trapInfo.Size == 0u )
            return false;

        if ( index < trapInfo.Size)
        {
            return true;
        }
        break;
    }
    return false;
}


/*********************************************************************
  Function:        
  	bool SNMPGetNextIndex(SNMP_ID var,SNMP_INDEX* index)

  Summary:
  	To search for next index node in case of a Sequence variable.
	
  Description:    
  	This is a callback function called by SNMP module.
    SNMP user must implement this function in user application and 
    provide appropriate data when called.  This function will only
    be called for OID variable of type sequence.
    
  PreCondition: 
  	None
 
  Parameters:
  	var		-	Variable id whose value is to be returned
  	index   -	Next Index of variable that should be transferred
 
  Return Values:
  	true	-	 If a next index value exists for given variable at given
                 index and index parameter contains next valid index.
    false	-	 Otherwise.
 
  Remarks:
	  Only sequence index needs to be handled in this function.
 ********************************************************************/
bool SNMPGetNextIndex(SNMP_ID var, SNMP_INDEX* index)
{
    SNMP_INDEX tempIndex;

    tempIndex = *index;

    switch(var)
    {
    case TRAP_RECEIVER_ID:
	case TRAP_RECEIVER_ENABLED:
	case TRAP_RECEIVER_IP:
	case TRAP_COMMUNITY:
        // There is no next possible index if table itself is empty.
        if ( trapInfo.Size == 0u )
            return false;

        // INDEX_INVALID means start with first index.
        if ( tempIndex == (uint8_t)SNMP_INDEX_INVALID )
        {
            *index = 0;
            return true;
        }
        else if ( tempIndex < (trapInfo.Size-1) )
        {
            *index = tempIndex+1;
            return true;
        }
        break;
    }
    return false;
}


/*********************************************************************
  Function:
  	bool SNMPGetVar(SNMP_ID var, SNMP_INDEX index,uint8_t* ref, SNMP_VAL* val)
                                   
  Summary:
  	Used to Get/collect OID variable information.

  Description:
 	This is a callback function called by SNMP module. SNMP user must 
 	implement this function in user application and provide appropriate
 	data when called.
   	
  PreCondition:
  	None
 
  parameters:
  	var		-	Variable id whose value is to be returned
    index   -	Index of variable that should be transferred
    ref     -   Variable reference used to transfer
              	multi-byte data
                It is always SNMP_START_OF_VAR when very
                first byte is requested.
                Otherwise, use this as a reference to
                keep track of multi-byte transfers.
    val     -	Pointer to up to 4 byte buffer.
                If var data type is uint8_t, transfer data
                  in val->byte
                If var data type is uint16_t, transfer data in
                  val->word
                If var data type is uint32_t, transfer data in
                  val->dword
                If var data type is IP_ADDRESS, transfer data
                  in val->v[] or val->dword
                If var data type is COUNTER32, TIME_TICKS or
                  GAUGE32, transfer data in val->dword
                If var data type is ASCII_STRING or OCTET_STRING
                  transfer data in val->byte using multi-byte
                  transfer mechanism.
 
  Return Values:
  	true	-	If a value exists for given variable at given index.
    false 	-	Otherwise.
 
  Remarks:
 	None.
 ********************************************************************/
bool SNMPGetVar(SNMP_ID var, SNMP_INDEX index, uint8_t* ref, SNMP_VAL* val)
{
    uint8_t myRef;
    uint32_t dw;

    int lcdSize = 16*2+1;
    char message[lcdSize];

    myRef = *ref;

    switch(var)
    {
    case SYS_UP_TIME:
    {
	 
        SYS_TICK dw10msTicks;
        dw10msTicks = (SYS_TICK_Get() * 100ull)/SYS_TICK_TicksPerSecondGet();

        val->dword = dw10msTicks;
        return true;
    }    

    case LED_D5:
        val->byte = LED2_IO;
        return true;

    case LED_D6:
        val->byte = LED1_IO;
        return true;

    case PUSH_BUTTON:
        // There is only one button - meaning only index of 0 is allowed.
        val->byte = BUTTON0_IO;
        return true;

    case ANALOG_POT0:
        val->word = atoi((char*)AN0String);
        return true;

    case TRAP_RECEIVER_ID:
        if ( index < trapInfo.Size )
        {
            val->byte = index;
            return true;
        }
        break;

    case TRAP_RECEIVER_ENABLED:
        if ( index < trapInfo.Size )
        {
            val->byte = trapInfo.table[index].Flags.bEnabled;
            return true;
        }
        break;

    case TRAP_RECEIVER_IP:
        if ( index < trapInfo.Size )
        {
            val->dword = trapInfo.table[index].IPAddress.Val;
            return true;
        }
        break;

    case TRAP_COMMUNITY:
        if ( index < trapInfo.Size )
        {
            if ( trapInfo.table[index].communityLen == 0u )
                *ref = SNMP_END_OF_VAR;
            else
            {
                val->byte = trapInfo.table[index].community[myRef];

                myRef++;

                if ( myRef == trapInfo.table[index].communityLen )
                    *ref = SNMP_END_OF_VAR;
                else
                    *ref = myRef;
            }
            return true;
        }
        break;

    case LCD_DISPLAY:
        break;
    }

    return false;
}


/*********************************************************************
  Function:
  	static SYS_TICK SNMPGetTimeStamp(void)
                                   
  Summary:
	Obtains the current Tick value for the SNMP time stamp.

  Description:
	This function retrieves the absolute time measurements for 
	SNMP time stamp in tens of milliseconds.

  PreCondition:
  	None
 
  parameters:
  	None
 
  Return Values:
  	timeStamp - SYS_TICK timevalue
 
  Remarks:
 	None.
 ********************************************************************/
static SYS_TICK SNMPGetTimeStamp(void)
{
    SYS_TICK    timeStamp;
    timeStamp = (SYS_TICK_Get() * 100ull)/SYS_TICK_TicksPerSecondGet();

    return timeStamp;

}

#endif	//#if defined(TCPIP_STACK_USE_SNMP_SERVER)
