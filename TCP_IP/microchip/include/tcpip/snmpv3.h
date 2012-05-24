/*******************************************************************************

  Summary:
    
  Description:
*******************************************************************************/

/*******************************************************************************
FileName:  SNMPv3.h 
Copyright � 2012 released Microchip Technology Inc.  All rights
reserved.

Microchip licenses to you the right to use, modify, copy and distribute
Software only when embedded on a Microchip microcontroller or digital signal
controller that is integrated into your product or third party product
(pursuant to the sublicense terms in the accompanying license agreement).

You should refer to the license agreement accompanying this Software for
additional information regarding your rights and obligations.

SOFTWARE AND DOCUMENTATION ARE PROVIDED �AS IS� WITHOUT WARRANTY OF ANY KIND,
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

#ifndef SNMPV3_H
#define SNMPV3_H



#include "tcpip/tcpip.h"
/*=======================================================================================*/

/* Abstarct Service Interfaces and Set of primitives for various sub systems of the SNMP engine */

/*=======================================================================================*/

/* ============== */
/* Dispatcher Primitives  */
/* ============== */

extern uint8_t snmpEngineID[]; //Reserving 32 bytes for the snmpEngineID as the octet string length can vary form 5 to 32 
extern	uint32_t snmpEngineBoots;//The number of times that the SNMP engine has (re-)initialized itself since snmpEngineID was last configured.
extern TCPIP_UINT32_VAL snmpEngineTime;//The number of seconds since the value of the snmpEngineBoots object last changed
extern	TCPIP_UINT32_VAL snmpEngineMaxMessageSize;
extern uint8_t snmpEngnIDLength;

/* 
Registering Responsibility for Handling SNMP PDUs 

Applications can register/unregister responsibility for a specific
contextEngineID, for specific pduTypes, with the PDU Dispatcher
according to the following primitives. The list of particular
pduTypes that an application can register for is determined by the
Message Processing Model(s) supported by the SNMP entity that
contains the PDU Dispatcher.

Note that realizations of the registerContextEngineID and
unregisterContextEngineID abstract service interfaces may provide
implementation-specific ways for applications to register/deregister
responsibility for all possible values of the contextEngineID or
pduType parameters.
*/

typedef struct  registerContextEngineID
{
	uint8_t* contextEngineID; //take responsibility for this one
	uint8_t pduType;			//the pduType(s) to be registered
}statusInformation; //success or errorIndication

struct unregisterContextEngineID 
{
	uint8_t* contextEngineID; //give up responsibility for this one
	uint8_t pduType;			//the pduType(s) to be unregistered
};

#define SNMPV3MSG_AUTHENTICATION_FAIL	0
#define SNMPV3MSG_AUTHENTICATION_SUCCESS 1

typedef enum{

	SNMPV3_MSG_AUTH_FAIL=0x00,
	SNMPV3_MSG_AUTH_PASS=0x01
}SNMPV3_MSG_AUTH_SEC_PARAM_RESULT;

typedef enum{

SNMPV3_MSG_PRIV_FAIL=0x00,
SNMPV3_MSG_PRIV_PASS=0x01

}SNMPV3_MSG_PRIV_SEC_PARAM_RESULT;




extern uint8_t  snmpSecurityLevel;

typedef enum
{
	SNMPV3_DES_PRIV=0x0,
	SNMPV3_AES_PRIV,
	SNMPV3_NO_PRIV
}SNMPV3_PRIV_PROT_TYPE;


// Type of hash being calculated
typedef enum
{
	SNMPV3_HAMC_MD5	= 0u,		// MD5 is being calculated
	SNMPV3_HMAC_SHA1,				// SHA-1 is being calculated
	SNMPV3_NO_HMAC_AUTH
} SNMPV3_HMAC_HASH_TYPE;

/*
Generate Outgoing Request or Notification 

statusInformation =  -- sendPduHandle if success
				   -- errorIndication if failure
*/

struct dispatcherStatusInfo
{
	uint8_t transportDomain;		//transport domain to be used
	uint32_t transportAddress;	//transport address to be used
	uint8_t messageProcessingModel;//typically, SNMP version
	uint8_t securityModel; 		//Security Model to use
	uint8_t* securityName; 		//on behalf of this principal
	uint8_t securityLevel;			//Level of Security requested
	uint8_t* contextEngineID; 	//data from/at this entity
	uint8_t* contextName;			//data from/in this context
	uint8_t pduVersion; 			//the version of the PDU
	uint8_t* PDU; 					//SNMP Protocol Data Unit
	bool expectResponse; 		//true or false

};



/* 
Process Incoming Request or Notification PDU 

Dispatcher provides the following primitive to pass an incoming snmp pdu to an application. 
*/

struct dispatcherProcessPdu  //process Request/Notification PDU
{
	uint8_t messageProcessingModel;	//typically, SNMP version
	uint8_t securityModel;				//Security Model in use
	uint8_t* securityName;			//on behalf of this principal
	uint8_t securityLevel;				//Level of Security
	uint8_t* contextEngineID;			//data from/at this SNMP entity
	uint8_t* contextName;				//data from/in this context
	uint8_t pduVersion;				//the version of the PDU
	uint8_t* PDU;						//SNMP Protocol Data Unit
	TCPIP_UINT32_VAL maxSizeResponseScopedPDU;// maximum size of the Response PDU
	uint32_t stateReference;			//reference to state information needed when sending a response

};

typedef struct
{
	
	uint8_t userName[USER_SECURITY_NAME_LEN];
	uint8_t userAuthPswd[AUTH_LOCALIZED_PASSWORD_KEY_LEN]; //RFC specifies not to save password with the managed nodes instead store pswd ipad and opad values.
	uint8_t userPrivPswd[PRIV_LOCALIZED_PASSWORD_KEY_LEN];
	uint8_t userAuthPswdLoclizdKey[AUTH_LOCALIZED_PASSWORD_KEY_LEN];
	uint8_t userPrivPswdLoclizdKey[PRIV_LOCALIZED_PASSWORD_KEY_LEN];
	//uint8_t userPrivPswdLoclizdKey[20];
	uint8_t userAuthLocalKeyHmacIpad[64];
	uint8_t userAuthLocalKeyHmacOpad[64];
	uint8_t userDBIndex;
	uint8_t userHashType;
	uint8_t userNameLength;
	uint8_t userAuthPswdLen;
	uint8_t userPrivPswdLen;
	uint8_t userPrivType;
}snmpV3EngnUserDataBase; 

extern snmpV3EngnUserDataBase snmpV3UserDataBase[];

typedef struct  
{
	
	uint32_t maxMessageSize; 			//of the sending SNMP entity
//	IN securityParameters; 			//for the received message
	uint32_t wholeMsgLength; 			//length as received on the wire
	uint8_t* wholeMsg; 				//as received on the wire
	uint8_t* securityEngineID;	 		//authoritative SNMP entity
	uint8_t* securityName; 			//identification of the principal
	uint8_t* scopedPDU; 				//message (plaintext) payload
//	OUT securityStateReference; 	//reference to security state
	uint32_t maxSizeResponseScopedPDU;//maximum size sender can handle
	uint8_t messageProcessingModel; 	//typically, SNMP version
	uint8_t securityModel; 			//for the received message
	uint8_t securityLevel; 			//Level of Security
	uint8_t securityEngineIDLen;	 		//authoritative SNMP entity
	uint8_t securityNameLength;
}SecuritySysProcessIncomingMsg; 

extern SecuritySysProcessIncomingMsg securityPrimitivesOfIncomingPdu;


/*
Generate Outgoing Response

The PDU Dispatcher provides the following primitive for an
application to return an SNMP Response PDU to the PDU Dispatcher:

result = SUCCESS or FAILURE
*/

struct dispathcerReturnResponsePdu
{
	uint8_t messageProcessingModel;	//typically, SNMP version
	uint8_t securityModel;				//Security Model in use
	uint8_t* securityName;			//on behalf of this principal
	uint8_t securityLevel;				//same as on incoming request
	uint8_t* contextEngineID;			//data from/at this SNMP entity
	uint8_t* contextName;				//data from/in this context
	uint8_t pduVersion;				//the version of the PDU
	uint8_t* PDU;						//SNMP Protocol Data Unit
	uint32_t maxSizeResponseScopedPDU;//maximum size sender can accept
	uint32_t stateReference;			//reference to state information as presented with the request
	statusInformation statInfo;		//success or errorIndication, error counter OID/value if error

};


/*
Process Incoming Response PDU

The PDU Dispatcher provides the following primitive to pass an
incoming SNMP Response PDU to an application:

*/

struct processResponsePdu //process Response PDU
{ 
	uint8_t messageProcessingModel;	//typically, SNMP version
	uint8_t securityModel;				//Security Model in use
	uint8_t* securityName;			//on behalf of this principal
	uint8_t securityLevel;				//Level of Security
	uint8_t* contextEngineID;			//data from/at this SNMP entity
	uint8_t* contextName;				//data from/in this context
	uint8_t pduVersion;				//the version of the PDU
	uint8_t* PDU;						//SNMP Protocol Data Unit
	statusInformation statInfo;		//success or errorIndication
//	IN sendPduHandle;				//handle from sendPdu
};			






/*=======================================================================================*/

/* =========================== */
/* Message Processing Subsystem Primitives  */
/* =========================== */

/*
The Dispatcher interacts with a Message Processing Model to process a
specific version of an SNMP Message. Below are the
primitives provided by the Message Processing Subsystem.
*/

/* 
Prepare Outgoing SNMP Request or Notification Message

The Message Processing Subsystem provides this service primitive for
preparing an outgoing SNMP Request or Notification Message
*/

struct MsgProcModPrepareOutgoingMessage
{
	uint8_t transportDomain;		//transport domain to be used
	uint32_t transportAddress;	//transport address to be used
	uint8_t messageProcessingModel;//typically, SNMP version
	uint8_t securityModel;			//Security Model to use
	uint8_t* securityName;		//on behalf of this principal
	uint8_t securityLevel;			//Level of Security requested
	uint8_t* contextEngineID;		//data from/at this entity
	uint8_t* contextName;			//data from/in this context
	uint8_t pduVersion;			//the version of the PDU
	uint8_t* PDU;					//SNMP Protocol Data Unit
	bool expectResponse;		//true or false
	//IN sendPduHandle;			//the handle for matching incoming responses
	uint8_t destTransportDomain;	//destination transport domain
	uint32_t destTransportAddress;//destination transport address
	uint8_t* outgoingMessage;		//the message to send
	uint32_t outgoingMessageLength; //its length
};



/*
Prepare an Outgoing SNMP Response Message

The Message Processing Subsystem provides this service primitive for
preparing an outgoing SNMP Response Message:
result = -- SUCCESS or FAILURE

*/
struct MsgProcModPrepareResponseMessage
{
	uint8_t messageProcessingModel;	//typically, SNMP version
	uint8_t securityModel;  			//same as on incoming request
	uint8_t* securityName;  			//same as on incoming request
	uint8_t securityLevel;  			//same as on incoming request
	uint8_t* contextEngineID;  		//data from/at this SNMP entity
	uint8_t* contextName;  			//data from/in this context
	uint8_t pduVersion;  				//the version of the PDU
	uint8_t* PDU;  					//SNMP Protocol Data Unit
	uint32_t maxSizeResponseScopedPDU;//maximum size able to accept
	uint32_t stateReference;  		//reference to state information as presented with the request
	statusInformation statInfo;//success or errorIndication, error counter OID/value if error
	uint8_t destTransportDomain;  		//destination transport domain
	uint32_t destTransportAddress;  	//destination transport address
	uint8_t* outgoingMessage;  		//the message to send
	uint32_t outgoingMessageLength;  	//its length
};



/*
Prepare Data Elements from an Incoming SNMP Message

The Message Processing Subsystem provides this service primitive for
preparing the abstract data elements from an incoming SNMP message:
result = -- SUCCESS or errorIndication

*/
struct MsgProcModPrepareDataElements
{
	uint8_t transportDomain;		//origin transport domain
	uint32_t transportAddress; 	//origin transport address
	uint8_t* wholeMsg; 			//as received from the network
	uint32_t wholeMsgLength; 		//as received from the network
	uint8_t messageProcessingModel;//typically, SNMP version
	uint8_t securityModel; 		//Security Model to use
	uint8_t* securityName; 		//on behalf of this principal
	uint8_t securityLevel; 		//Level of Security requested
	uint8_t* contextEngineID; 	//data from/at this entity
	uint8_t* contextName; 		//data from/in this context
	uint8_t pduVersion; 			//the version of the PDU
	uint8_t* PDU ; 				//SNMP Protocol Data Unit
	uint8_t pduType ; 				//SNMP PDU type
//	OUT sendPduHandle; 			// handle for matched request
	uint32_t maxSizeResponseScopedPDU; //maximum size sender can accept
	statusInformation statInfo; 	//success or errorIndication error counter OID/value if error
	uint32_t stateReference; 		//reference to state information to be used for possible Response
};



/*=======================================================================================*/

/* ======================== */
/* Access Control Subsystem Primitives  */
/* ======================== */

/*
Applications are the typical clients of the service(s) of the Access
Control Subsystem. The following primitive is provided by the Access 
Control Subsystem to check if access is allowed:

statusInformation = -- success or errorIndication
*/

struct AccessCtrlSubSysIsAccessAllowed
{
	uint8_t securityModel; 	//Security Model in use
	uint8_t* securityName;	//principal who wants to access
	uint8_t securityLevel;	 	//Level of Security
	uint8_t viewType; 			//read, write, or notify view
	uint8_t* contextName; 	//context containing variableName
	uint8_t* variableName; 	//OID for the managed object
};


/*=======================================================================================*/

/* ==================== */
/* Security Subsystem Primitives  */
/* ==================== */

/*
The Message Processing Subsystem is the typical client of the services of the Security Subsystem.
*/


/* 
Generate a Request or Notification Message

The Security Subsystem provides the following primitive to generate a
Request or Notification message:

*/

typedef enum
{

	RESERVED=0x0,
	IPV4_ADDR_ENGN_ID=0x01,//4octets 
	IPV6_ADDR_ENGN_ID=0x02,//16 octets
	MAC_ADDR_ENGN_ID=0x03,//6 octets
	ADMIN_ASSIGNED_TEXT=0x04,
	ADMIN_ASSIGNED_OCTETS=0x05,
	RESERVED_UNUSED=0x06, //6 to 127 are reserved and unused
	ENTERPRISE_DEFINED=128 //128 to 255 as defiend by the enterprise maximum remaining length

}SNMP_ENGNID_OCTET_IDENTIFIER_VAL;	//The fifth octet indicates how the rest (6th and following octets) are formatted. Refer to RFC3411 section5 Page# 41	


typedef enum
{
	/*Octet's Least significant three bits: Reportable, PrivFlag, AuthFlag */	
	NO_REPORT_NO_PRIVACY_NO_AUTH 				=0x00, /* 00000000b */
	NO_REPORT_NO_PRIVACY_BUT_AUTH_PROVIDED		=0x01, /* 00000001b */
	NO_REPORT_PRIVACY_PROVIDED_BUT_NO_AUTH		=0x02, /* 00000010b Priv without Auth is not allowed*/
	NO_REPORT_PRIVACY_AND_AUTH_PROVIDED			=0x03, /* 00000011b */
	
	REPORT2REQ_NO_PRIVACY_NO_AUTH				=0x04, /* 00000100b */
	REPORT2REQ_NO_PRIVACY_BUT_AUTH_PROVIDED		=0x05, /* 00000101b */
	REPORT2REQ_PRIVACY_PROVIDED_BUT_NO_AUTH		=0x06, /* 00000110b Priv without Auth is not allowed*/
	REPORT2REQ_PRIVACY_AND_AUTH_PROVIDED		=0x07, /* 00000111b */
	INVALID_MSG									=0xFF
	
}REPORT_FLAG_AND_SECURITY_LEVEL_FLAGS;

typedef enum
{
	noAuthProtocol = 0x1,
	hmacMD5Auth,
	hmacSHAAuth,
	noPrivProtocol,
	desPrivProtocol = 0x5,
	aesPrivProtocol =0x6
}USM_SECURITY_LEVEL;

struct SecuritySysGenerateRequestMsg
{
	uint8_t messageProcessingModel; 	//typically, SNMP version
	uint8_t* globalData; 				//message header, admin data
	uint32_t maxMessageSize; 			//of the sending SNMP entity
	uint8_t securityModel; 			//for the outgoing message
	uint8_t* securityEngineID ; 		//authoritative SNMP entity
	uint8_t* securityName;			//on behalf of this principal
	uint8_t securityLevel; 			//Level of Security requested
	uint8_t* scopedPDU; 				//message (plaintext) payload
	//OUT securityParameters; 		//filled in by Security Module
	uint8_t* wholeMsg ; 				//complete generated message
	uint32_t wholeMsgLength; 			//length of the generated message
};



/*
Process Incoming Message

The Security Subsystem provides the following primitive to process an
incoming message:
statusInformation = -- errorIndication or success error counter OID/value if error

*/ 




/*
Generate a Response Message

The Security Subsystem provides the following primitive to generate a
Response message:

*/ 

struct SecuritySysGenerateResponseMsg
{
	uint8_t messageProcessingModel; 	//typically, SNMP version
	uint8_t* globalData; 				//message header, admin data
	uint32_t maxMessageSize; 			//of the sending SNMP entity
	uint8_t securityModel; 			//for the outgoing message
	uint8_t* securityEngineID; 		//authoritative SNMP entity
	uint8_t* securityName;			//on behalf of this principal
	uint8_t securityLevel; 			//for the outgoing message
	uint8_t* scopedPDU; 				//message (plaintext) payload
//	IN securityStateReference; 	//reference to security state information from original request
//	OUT securityParameters; 	//filled in by Security Module
	uint8_t* wholeMsg;	 			//complete generated message
	uint32_t wholeMsgLength; 		//length of the generated message
};

/* Snmp Message Processing Model */

typedef enum
{
	ANY_SECUTIRY_MODEL=0x00,
	SNMPV1_SECURITY_MODEL=0X01,
	SNMPV2C_SECURITY_MODEL=0X02,
	SNMPV3_USM_SECURITY_MODEL=0X03
	/* Values between 1 to 255, inclusive, are resereved for standards-track
	     Security Models  and are managed by IANA.*/
}STD_BASED_SNMP_SECURITY_MODEL;

typedef enum
{
	SNMPV1_MSG_PROCESSING_MODEL=0X00,
	SNMPV2C_MSG_PROCESSING_MODEL=0X01,
	SNMPV2U_V2_MSG_PROCESSING_MODEL=0X02,
	SNMPV3_MSG_PROCESSING_MODEL=0X03
	/* Values between 0 to 255, inclusive, are resereved for standards-track
	     Message processing Models and are managed by IANA.*/

}STD_BASED_SNMP_MESSAGE_PROCESSING_MODEL;

typedef enum
{
	NO_AUTH_NO_PRIV=1,
	AUTH_NO_PRIV,
	AUTH_PRIV
}STD_BASED_SNMPV3_SECURITY_LEVEL;


/* 
 snmpv3 target configuration with respect to trap.
*/
typedef struct
{	
	uint8_t userSecurityName[USER_SECURITY_NAME_LEN];
	STD_BASED_SNMP_MESSAGE_PROCESSING_MODEL messageProcessingModelType;
	STD_BASED_SNMP_SECURITY_MODEL securityModelType;
	STD_BASED_SNMPV3_SECURITY_LEVEL securityLevelType;
}snmpV3TrapConfigDataBase; 




typedef struct 
{	
	uint8_t* wholeMsgHead;
	uint8_t* snmpMsgHead;
	TCPIP_UINT16_VAL wholeMsgLen;
	TCPIP_UINT16_VAL snmpMsgLen;
	uint16_t msgAuthParamOffsetInWholeMsg;
	uint16_t scopedPduOffset;
	uint8_t scopedPduAuthStructVal;
	uint16_t scopedPduStructLen;
}SNMPV3_REQUEST_WHOLEMSG;


typedef struct 
{	
	uint8_t* wholeMsgHead;
	uint8_t* snmpMsgHead;
	TCPIP_UINT16_VAL wholeMsgLen;
	TCPIP_UINT16_VAL snmpMsgLen;
	uint8_t* msgAuthParamOffsetOutWholeMsg;
	uint8_t* scopedPduOffset;
	uint16_t scopedPduStructLen;
	uint8_t scopedPduAuthStructVal;
}SNMPV3_RESPONSE_WHOLEMSG;


extern snmpV3TrapConfigDataBase gSnmpv3TrapConfigData[];
extern SNMPV3_REQUEST_WHOLEMSG gSnmpV3InPduWholeMsgBuf;

/*=======================================================================================*/

/* ============== */
/*  Common Primitives   */
/* ============== */

/*
These primitive(s) are provided by multiple Subsystems.
*/

/*
Release State Reference Information

All Subsystems which pass stateReference information also provide a
primitive to release the memory that holds the referenced state
information

*/

struct StateRelease
{
	uint32_t stateReference; 	//handle of reference to be released
};


void Snmpv3USMOutMsgPrivParam(void);

extern void Snmpv3Init(void);
extern void Snmpv3FreeDynAllocMem(void);
extern void Snmpv3ComputeHMACIpadOpadForAuthLoclzedKey(uint8_t userDBIndex);
extern uint8_t* Snmpv3ComputeHmacMD5Digest(uint8_t * inData, uint32_t dataLen,uint8_t* userExtendedLclzdKeyIpad,uint8_t* userExtendedLclzdKeyOpad);

extern bool Snmpv3CmprTrapSecNameAndSecLvlWithUSMDb(uint8_t tragetIndex,uint8_t userTrapSecLen,
			uint8_t *userTrapSecurityName,STD_BASED_SNMPV3_SECURITY_LEVEL securityLevel);

extern uint8_t Snmpv3GetTrapSecurityLevel(STD_BASED_SNMPV3_SECURITY_LEVEL securityLevel);
extern uint8_t Snmpv3GetSecurityLevel(uint8_t userIndex);

extern uint8_t Snmpv3AESEncryptResponseScopedPdu(SNMPV3_RESPONSE_WHOLEMSG* plain_text/*uint8_t userDBIndex*/);
extern uint8_t Snmpv3AuthenticateTxPduForDataIntegrity(SNMPV3_RESPONSE_WHOLEMSG* txDataPtr);
extern uint8_t Snmpv3AuthenticateRxedPduForDataIntegrity(SNMPV3_REQUEST_WHOLEMSG* rxDataPtr);
extern void Snmpv3UsmSnmpEngnAuthPrivPswdLocalization(uint8_t userDBIndex);


#endif
