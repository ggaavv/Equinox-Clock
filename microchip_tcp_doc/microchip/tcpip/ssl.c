/*******************************************************************************
  SSLv3 Protocol Client and Server Implementation

  Summary:
    Module for Microchip TCP/IP Stack
    
  Description:
    - Implements an SSL layer supporting both client and server
      operation for any given TCP socket.
*******************************************************************************/

/*******************************************************************************
FileName:   SSL.c
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

#define __SSL_C

#include "tcpip_private.h"
#include "tcpip_config_private.h"

#if defined(TCPIP_STACK_USE_SSL_SERVER) || defined(TCPIP_STACK_USE_SSL_CLIENT)

#include "ssl_private.h"

// define the number of interfaces we want to run SSL on
#if SSL_MULTIPLE_INTERFACES
    #define SSL_INTERFACES      (TCPIP_NETWORK_INTERFACES)
#else
    #define SSL_INTERFACES      1 
#endif


/****************************************************************************
  Section:
	SSL Connection State Global Variables
  ***************************************************************************/

typedef struct
{
     SSL_STUB   sslStub;		    // The current SSL stub
	 uint8_t       sslStubID;			// Which SSL_STUB is loaded
	 SSL_KEYS   sslKeys;		    // The current SSL session
	 uint8_t       sslKeysID;			// Which SSL_KEYS are loaded
     uint8_t       sslBufferID;		// Which buffer is loaded
     uint8_t       sslHashID;			// Which hash is loaded
     uint8_t       sslSessionID;		// Which session is loaded
     bool       sslSessionUpdated;	// Whether or not it has been updated
     uint8_t       sslRSAStubID;		// Which stub is using RSA, if any
}SSL_DCPT;      // SSL layer descriptor

    static SSL_DCPT   sslIfDcpt[SSL_INTERFACES];              // The SSL descriptor per interface
	
	SSL_BUFFER sslIfBuffer[SSL_INTERFACES];			         // SBox and RSA storage
                                                             // global, accessed by RSA

	static HASH_SUM sslIfHash[SSL_INTERFACES];				// Hash storage

	static SSL_SESSION sslIfSession[SSL_INTERFACES];			// Current session data
	
	// 8 byte session stubs
	static SSL_SESSION_STUB sslIfSessionStubs[SSL_INTERFACES][SSL_MAX_SESSIONS];
	
	static uint8_t             *ptrIfHS[SSL_INTERFACES]; 	    // Used in buffering handshake results

    static NET_CONFIG*      sslSyncIf = 0;                           // last SSL interface index that was synced
    
	extern const uint16_t SSL_CERT_LEN;	// RSA public certificate length		?
	extern const uint8_t SSL_CERT[];		// RSA public certificate data			?


/****************************************************************************
  Section:
	Resource Management Variables
  ***************************************************************************/
static uint16_t isIfStubUsed[SSL_INTERFACES];			// Indicates which stubs are in use
static uint16_t isIfHashUsed[SSL_INTERFACES];			// Indicates which hashes are in use
static uint16_t isIfBufferUsed[SSL_INTERFACES];		// Indicates which buffers are in use

// Masks for each bit in the is*Used variables
static const uint16_t masks[16] = { 0x0001, 0x0002, 0x0004, 0x0008,
							  0x0010, 0x0020, 0x0040, 0x0080, 
							  0x0100, 0x0200, 0x0400, 0x0800,
							  0x1000, 0x2000, 0x4000, 0x8000 };
	
/****************************************************************************
  Section:
	Function Prototypes
  ***************************************************************************/
	// Section: Cryptographic Calculation Functions
	static RSA_STATUS SSLRSAOperation(void);
	static void GenerateHashRounds(uint8_t num, uint8_t* rand1, uint8_t* rand2);
	static void CalculateFinishedHash(uint8_t hashID, bool fromClient, uint8_t *result);
	static void GenerateSessionKeys(void);

	// Section: Ethernet Buffer RAM Management
	static void SSLStubSync(NET_CONFIG* pIf, uint8_t id);
	static bool SSLStubAlloc(NET_CONFIG* currIf);
	static void SSLStubFree(uint8_t id);
	static void SSLKeysSync(uint8_t id);
	static void SSLHashSync(uint8_t id);
	static void SSLHashAlloc(uint8_t *id);
	static void SSLHashFree(uint8_t *id);
	static void SSLBufferSync(uint8_t id);
	static void SSLBufferAlloc(uint8_t *id);
	static void SSLBufferFree(uint8_t *id);
	static uint8_t SSLSessionNew(void);
	static void SSLSessionSync(uint8_t id);
	static void SaveOffChip(TCPIP_MAC_HANDLE hMac, uint8_t *ramAddr, PTR_BASE ethAddr, uint16_t len);
	static void LoadOffChip(TCPIP_MAC_HANDLE hMac, uint8_t *ramAddr, PTR_BASE ethAddr, uint16_t len);
	
	// Section: Handshake Hash and I/O Functions
	static void HSStart(void);
	static void HSEnd(void);
	static uint16_t HSGet(TCP_SOCKET skt, uint8_t *b);
	static uint16_t HSGetWord(TCP_SOCKET skt, uint16_t *w);
	static uint16_t HSGetArray(TCP_SOCKET skt, uint8_t *data, uint16_t len);
	static uint16_t HSPut(TCP_SOCKET skt, uint8_t b);
	static uint16_t HSPutWord(TCP_SOCKET skt, uint16_t w);
	static uint16_t HSPutArray(TCP_SOCKET skt, uint8_t *data, uint16_t len);
	#if defined(TCPIP_STACK_USE_SSL_SERVER)
	static uint16_t HSPutROMArray(TCP_SOCKET skt, const uint8_t *data, uint16_t len);
	#endif
	
	// Section: Client messages
	#if defined(TCPIP_STACK_USE_SSL_CLIENT)
		static uint8_t SSLSessionMatchIP(IP_ADDR ip);
		static void SSLTxClientHello(TCP_SOCKET hTCP);
		static void SSLRxServerHello(TCP_SOCKET hTCP);
		static void SSLRxServerCertificate(TCP_SOCKET hTCP);
		static void SSLTxClientKeyExchange(TCP_SOCKET hTCP);
	#endif
	
	// Section: Server messages
	#if defined(TCPIP_STACK_USE_SSL_SERVER)
		static uint8_t SSLSessionMatchID(uint8_t* SessionID);
		static void SSLRxAntiqueClientHello(TCP_SOCKET hTCP);
		static void SSLRxClientHello(TCP_SOCKET hTCP);
		static void SSLTxServerHello(TCP_SOCKET hTCP);
		static void SSLTxServerCertificate(TCP_SOCKET hTCP);
		static void SSLTxServerHelloDone(TCP_SOCKET hTCP);
		static void SSLRxClientKeyExchange(TCP_SOCKET hTCP);
	#endif
	
	// Section: Client and server messages
	static void SSLTxCCSFin(TCP_SOCKET hTCP);
	static void SSLRxCCS(TCP_SOCKET hTCP);
	static void SSLRxFinished(TCP_SOCKET hTCP);
	static void SSLRxAlert(TCP_SOCKET hTCP);

/****************************************************************************
  Section:
	Macros and Definitions
  ***************************************************************************/
	#define mMIN(a, b)	((a<b)?a:b)

	#define SSL_RSA_EXPORT_WITH_ARCFOUR_40_MD5	0x0003u
	#define SSL_RSA_WITH_ARCFOUR_128_MD5		0x0004u

#if SSL_MULTIPLE_INTERFACES
    #define sslStub(netIx)              (sslIfDcpt[netIx].sslStub)
    #define sslStubID(netIx)            (sslIfDcpt[netIx].sslStubID )
    #define sslKeys(netIx)		        (sslIfDcpt[netIx].sslKeys)
    #define sslKeysID(netIx)	        (sslIfDcpt[netIx].sslKeysID)
    #define sslBufferID(netIx)	        (sslIfDcpt[netIx].sslBufferID)
    #define sslHashID(netIx)	        (sslIfDcpt[netIx].sslHashID)
    #define sslSessionID(netIx)	        (sslIfDcpt[netIx].sslSessionID)
    #define sslSessionUpdated(netIx)    (sslIfDcpt[netIx].sslSessionUpdated)
    #define sslRSAStubID(netIx)         (sslIfDcpt[netIx].sslRSAStubID)

    #define sslBuffer(netIx)            (sslIfBuffer[netIx])
    #define sslHash(netIx)              (sslIfHash[netIx])
    #define sslSession(netIx)           (sslIfSession[netIx])
    #define sslSessionStubs(netIx)      (sslIfSessionStubs[netIx])
    #define ptrHS(netIx)                (ptrIfHS[netIx])
    #define isStubUsed(netIx)           (isIfStubUsed[netIx])
    #define isHashUsed(netIx)           (isIfHashUsed[netIx])
    #define isBufferUsed(netIx)         (isIfBufferUsed[netIx])
    
#else
    #define sslStub(netIx)              (sslIfDcpt[0].sslStub)
    #define sslStubID(netIx)            (sslIfDcpt[0].sslStubID )
    #define sslKeys(netIx)		        (sslIfDcpt[0].sslKeys)
    #define sslKeysID(netIx)	        (sslIfDcpt[0].sslKeysID)
    #define sslBufferID(netIx)	        (sslIfDcpt[0].sslBufferID)
    #define sslHashID(netIx)	        (sslIfDcpt[0].sslHashID)
    #define sslSessionID(netIx)	        (sslIfDcpt[0].sslSessionID)
    #define sslSessionUpdated(netIx)	(sslIfDcpt[0].sslSessionUpdated)
    #define sslRSAStubID(netIx)         (sslIfDcpt[0].sslRSAStubID)

    #define sslBuffer(netIx)            (sslIfBuffer[0])
    #define sslHash(netIx)              (sslIfHash[0])
    #define sslSession(netIx)           (sslIfSession[0])
    #define sslSessionStubs(netIx)      (sslIfSessionStubs[0])
    #define ptrHS(netIx)                (ptrIfHS[0])
    #define isStubUsed(netIx)           (isIfStubUsed[0])
    #define isHashUsed(netIx)           (isIfHashUsed[0])
    #define isBufferUsed(netIx)         (isIfBufferUsed[0])
    
#endif  // SSL_MULTIPLE_INTERFACES
    
#define SSLSessionUpdated(netIx)	    (sslSessionUpdated(netIx) = true)

// Base address for SSL stubs
#define SSL_BASE_STUB_ADDR(hMac)    (MACGetSslBaseAddr(hMac))

// Base address for SSL keys
#define SSL_BASE_KEYS_ADDR(hMac)	(MACGetSslBaseAddr(hMac) + SSL_STUB_SPACE)

// Base address for SSL hashes
#define SSL_BASE_HASH_ADDR(hMac)	(MACGetSslBaseAddr(hMac) + SSL_STUB_SPACE + SSL_KEYS_SPACE)

// Base address for SSL buffers
#define SSL_BASE_BUFFER_ADDR(hMac)	(MACGetSslBaseAddr(hMac) + SSL_STUB_SPACE + SSL_KEYS_SPACE + SSL_HASH_SPACE)

// Base address for SSL sessions
#define SSL_BASE_SESSION_ADDR(hMac)	(MACGetSslBaseAddr(hMac) + SSL_STUB_SPACE + SSL_KEYS_SPACE + SSL_HASH_SPACE + SSL_BUFFER_SPACE)


/****************************************************************************
  ===========================================================================
  Section:
	SSL Management Functions
  ===========================================================================
  ***************************************************************************/

/*****************************************************************************
  Function:
	void SSLInit(void)

  Description:
	Initializes the SSL engine.

  Precondition:
	None

  Parameters:
	None

  Returns:
  	None
  	
  Remarks:
	This function is called only one during lifetime of the application.
  ***************************************************************************/
bool SSLInit(const TCPIP_STACK_MODULE_CTRL* const stackData,
             const SSL_MODULE_CONFIG* const sslData)
{
    // if(stackInit->stackAction == TCPIP_STACK_ACTION_IF_UP)  // interface restart
    // if(stackInit->stackAction == TCPIP_STACK_ACTION_INIT)   // stack init

	// Set all resources to unused
    int netIx = stackData->netIx;

	isStubUsed(netIx) = 0;
	isHashUsed(netIx) = 0;
	isBufferUsed(netIx) = 0;
	for(sslSessionID(netIx) = 0; sslSessionID(netIx) < SSL_MAX_SESSIONS; sslSessionID(netIx)++)
	{
		sslSessionStubs(netIx)[sslSessionID(netIx)].tag.Val = 0;
	}

	// Indicate that nothing is loaded
	sslHashID(netIx) = SSL_INVALID_ID;
	sslStubID(netIx) = SSL_INVALID_ID;
	sslSessionID(netIx) = SSL_INVALID_ID;
	sslKeysID(netIx) = SSL_INVALID_ID;
	sslBufferID(netIx) = SSL_INVALID_ID;
	sslSessionUpdated(netIx) = false;
	sslRSAStubID(netIx) = SSL_INVALID_ID;

	sslSyncIf  = 0;
	
	return true;
}	

/*****************************************************************************
  Function:
	void SSLDeInit(void)

  Description:
	DeInitializes the SSL engine.

  Precondition:
	None

  Parameters:
	None

  Returns:
  	None
  	
  Remarks:
	This function may be called several 
  ***************************************************************************/
void SSLDeInit(const TCPIP_STACK_MODULE_CTRL* const stackData)
{
    // if(stackInit->stackAction == TCPIP_STACK_ACTION_DEINIT) // stack shut down
    // if(stackInit->stackAction == TCPIP_STACK_ACTION_IF_DOWN) // interface down

}

/*****************************************************************************
  Function:
	void SSLPeriodic(TCP_SOCKET hTCP, uint8_t id)

  Summary:
	Performs any periodic tasks for the SSL module.

  Description:
	This function performs periodic tasks for the SSL module.  This includes
	processing for RSA operations.

  Precondition:
	SSL has already been initialized.

  Parameters:
	hTCP - the socket for which to perform periodic functions
	id - the SSL stub to use
	
  Returns:
  	None
  	
  ***************************************************************************/
void SSLPeriodic(TCP_SOCKET hTCP, uint8_t id)
{
	// Sync the SSL Stub
    NET_CONFIG* currIf = (NET_CONFIG*)TCPSocketGetNet(hTCP);
#if SSL_MULTIPLE_INTERFACES
    int netIx = _TCPIPStackNetIx(currIf);
#endif // SSL_MULTIPLE_INTERFACES

    SSLStubSync(currIf, id);
	
	// For new sessions, try to claim a session
	if(sslStub(netIx).Flags.bNewSession && sslStub(netIx).idSession == SSL_INVALID_ID)
	{
		sslStub(netIx).idSession = SSLSessionNew();
	}
	
	// If RSA is in progress, do some RSA work
	if(sslStub(netIx).Flags.bRSAInProgress)
	{			
		if(SSLRSAOperation() == RSA_DONE)
		{// Move on with the connection
			sslStub(netIx).Flags.bRSAInProgress = 0;

			// For clients, request the CKE message
			#if defined(TCPIP_STACK_USE_SSL_CLIENT)
			if(!sslStub(netIx).Flags.bIsServer)
				TCPRequestSSLMessage(hTCP, SSL_CLIENT_KEY_EXCHANGE);
			#endif
			
			// For servers, copy the decoded message to the session data
			#if defined(TCPIP_STACK_USE_SSL_SERVER)
			if(sslStub(netIx).Flags.bIsServer)
			{
				// Copy over the pre-master secret
				SSLSessionSync(sslStub(netIx).idSession);
				memcpy((void*)sslSession(netIx).masterSecret, (void*)&sslBuffer(netIx).full[(SSL_RSA_KEY_SIZE/8)-48], 48);
												
				// Generate the Master Secret
				SSLKeysSync(sslStubID(netIx));
				SSLBufferSync(SSL_INVALID_ID);
				GenerateHashRounds(3, sslKeys(netIx).Remote.random, sslKeys(netIx).Local.random);
				memcpy(sslSession(netIx).masterSecret, (void*)sslBuffer(netIx).hashRounds.temp, 48);
				
				// Note the new session data and release RSA engine
				SSLSessionUpdated(netIx);
				RSAEndUsage(currIf);
				sslRSAStubID(netIx) = SSL_INVALID_ID;
			}
			
			// Continue receiving the CCS and Finished messages
			TCPSSLHandleIncoming(hTCP);
			#endif
		}
	}
}

/*****************************************************************************
  Function:
	uint8_t SSLStartSession(TCP_SOCKET hTCP, uint8_t * buffer, uint8_t supDataType)

  Description:
	Begins a new SSL session for the given TCP connection.

  Precondition:
	SSL has been initialized and hTCP is connected.

  Parameters:
	hTCP - the socket to begin the SSL connection on
	buffer - pointer to a supplementary data buffer
	supDataType - type of supplementary data to store
	
  Return Values:
  	SSL_INVALID_ID - insufficient SSL resources to start a new connection
  	others - the allocated SSL stub ID
  ***************************************************************************/
uint8_t SSLStartSession(TCP_SOCKET hTCP, void * buffer, uint8_t supDataType)
{
	uint8_t    i;
    NET_CONFIG* currIf = (NET_CONFIG*)TCPSocketGetNet(hTCP);
#if SSL_MULTIPLE_INTERFACES
    int     netIx = _TCPIPStackNetIx(currIf);
#endif  // SSL_MULTIPLE_INTERFACES
	
	// Allocate a stub for use, or fail
	if(!SSLStubAlloc(currIf))
    {
		return SSL_INVALID_ID;
    }

    
	// Clear stub state
	sslStub(netIx).wRxBytesRem = 0;
	sslStub(netIx).wRxHsBytesRem = 0;
	sslStub(netIx).Flags.w = 0x0000;
	
	// Clear any allocations
	sslStub(netIx).idSession = SSL_INVALID_ID;
	sslStub(netIx).idRxHash = SSL_INVALID_ID;
	sslStub(netIx).idMD5 = SSL_INVALID_ID;
	sslStub(netIx).idSHA1 = SSL_INVALID_ID;
	sslStub(netIx).idRxBuffer = SSL_INVALID_ID;
	sslStub(netIx).idTxBuffer = SSL_INVALID_ID;
	sslStub(netIx).requestedMessage = SSL_NO_MESSAGE;
	sslStub(netIx).dwTemp.Val = 0;
	sslStub(netIx).supplementaryBuffer = buffer;
    sslStub(netIx).supplementaryDataType = supDataType;

	// Allocate handshake hashes for use, or fail
	SSLHashAlloc(&sslStub(netIx).idMD5);
	SSLHashAlloc(&sslStub(netIx).idSHA1);
	if(sslStub(netIx).idMD5 == SSL_INVALID_ID || sslStub(netIx).idSHA1 == SSL_INVALID_ID)
	{
		SSLHashFree(&sslStub(netIx).idMD5);
		SSLHashFree(&sslStub(netIx).idSHA1);
		SSLStubFree(sslStubID(netIx));
		return SSL_INVALID_ID;
	}
	
	// Initialize the handshake hashes
	SSLHashSync(sslStub(netIx).idSHA1);
	SHA1Initialize(&sslHash(netIx));
	SSLHashSync(sslStub(netIx).idMD5);
	MD5Initialize(&sslHash(netIx));
	
	// Set up Local.random (4 byte UTC time, 28 bytes random)
	SSLKeysSync(sslStubID(netIx));
	#if defined(TCPIP_STACK_USE_SNTP_CLIENT)
	{
		TCPIP_UINT32_VAL temp;

		temp.Val = SNTPGetUTCSeconds();
		sslKeys(netIx).Local.random[0] = temp.v[3];
		sslKeys(netIx).Local.random[1] = temp.v[2];
		sslKeys(netIx).Local.random[2] = temp.v[1];
		sslKeys(netIx).Local.random[3] = temp.v[0];
		i = 4;
	}
	#else
		i = 0;
	#endif
	while(i < 32u)
		sslKeys(netIx).Local.random[i++] = SYS_RANDOM_GET();
		
	// Return the ID
	return sslStubID(netIx);
}

/*****************************************************************************
  Function:
	void SSLTerminate(TCP_SOCKET hTCP, uint8_t id)

  Description:
	Terminates an SSL connection and releases allocated resources.

  Precondition:
	None

  Parameters:
	hTCP - the socket to terminate the SSL connection on
	id - the SSL stub ID to terminate
	
  Returns:
  	None
  ***************************************************************************/
void SSLTerminate(TCP_SOCKET hTCP, uint8_t id)
{
	// Sync in the right stub
    NET_CONFIG* currIf = (NET_CONFIG*)TCPSocketGetNet(hTCP);
#if SSL_MULTIPLE_INTERFACES
    int netIx = _TCPIPStackNetIx(currIf);
#endif  // SSL_MULTIPLE_INTERFACES
    SSLStubSync(currIf, id);
	
	// If no CloseNotify, then invalidate the session so it cannot resume
	// ( This restriction is not presently enforced.  IE incorrectly
	//   completes the handshake, then disconnects without a CloseNotify
	//   when it decides to prompt the user whether or not to accept a 
	//   unverifiable certificate. )
	//if(!sslStub(netIx).Flags.bCloseNotify)
	//{
	//	sslSessionStubs(netIx)[sslStub(netIx).idSession].tag.Val = 0;
	//}	
	
	// Free up resources
	SSLBufferFree(&sslStub(netIx).idRxBuffer);
	SSLBufferFree(&sslStub(netIx).idTxBuffer);
	SSLHashFree(&sslStub(netIx).idMD5);
	SSLHashFree(&sslStub(netIx).idSHA1);
	SSLHashFree(&sslStub(netIx).idRxHash);
	SSLStubFree(id);
	if(sslRSAStubID(netIx) == id)
	{
		sslRSAStubID(netIx) = SSL_INVALID_ID;
		RSAEndUsage(currIf);
	}
	
}

/****************************************************************************
  ===========================================================================
  Section:
	SSL Record Processing Functions
  ===========================================================================
  ***************************************************************************/

/*****************************************************************************
  Function:
	uint16_t SSLRxRecord(TCP_SOCKET hTCP, uint8_t id)

  Summary:
	Receives an SSL record.

  Description:
	Reads at most one SSL Record header from the TCP stream and determines what
	to do with the rest of the data.  If not all of the data is available for 
	the record, then the function returns and future call(s) to SSLRxRecord() 
	will process the remaining data until the end of the record is reached.  
	If this call process data from a past record, the next record will not be 
	started until the next call.

  Precondition:
	The specified SSL stub is initialized and the TCP socket is connected.

  Parameters:
	hTCP - The TCP socket from which to read
	id - The active SSL stub ID
	
  Returns:
  	uint16_t indicating the number of data bytes there were decrypted but left in 
  	the stream.

  Remarks:
  	SSL record headers, MAC footers, and symetric cipher block padding (if any) 
  	will be extracted from the TCP stream by this function.  Data will be 
  	decrypted but left in the stream.
  ***************************************************************************/
uint16_t SSLRxRecord(TCP_SOCKET hTCP, uint8_t id)
{	
	uint8_t temp[32];
	uint16_t wLen;
    
    NET_CONFIG* currIf = (NET_CONFIG*)TCPSocketGetNet(hTCP);
#if SSL_MULTIPLE_INTERFACES
    int netIx = _TCPIPStackNetIx(currIf);
#endif // SSL_MULTIPLE_INTERFACES
	
	SSLStubSync(currIf, id);
	
	// Don't do anything for terminated connections
	if(sslStub(netIx).Flags.bDone)
		return 0;

	// If this is a new record, then read the header
	// When bytes remain, a message is not yet fully read, so
	// the switch statement will continue handling the data
	if(sslStub(netIx).wRxBytesRem == 0u)
	{
		// See if we expect a MAC
		if(sslStub(netIx).Flags.bExpectingMAC)
		{// Receive and verify the MAC
			if(TCPIsGetReady(hTCP) < 16u)
				return 0;
				
			// Read the MAC
			TCPGetArray(hTCP, temp, 16);
			
			// Calculate the expected MAC
			SSLBufferSync(sslStub(netIx).idRxBuffer);
			SSLKeysSync(id);
			SSLHashSync(sslStub(netIx).idRxHash);
			
			ARCFOURCrypt(&sslKeys(netIx).Remote.app.cryptCtx, temp, 16);
			SSLMACCalc(sslKeys(netIx).Remote.app.MACSecret, &temp[16]);
			
			// MAC no longer expected
			sslStub(netIx).Flags.bExpectingMAC = 0;
			
			// Verify the MAC
			if(memcmp((void*)temp, (void*)&temp[16], 16) != 0)
			{// MAC fails
				TCPRequestSSLMessage(hTCP, SSL_ALERT_BAD_RECORD_MAC);
				return 0;
			}
		}	
		
		// Check if a new header is available
		// Also ignore data if SSL is terminated
		if(TCPIsGetReady(hTCP) < 5u)
			return 0;
		
		// Read the record type (uint8_t)
		TCPGet(hTCP, &sslStub(netIx).rxProtocol);
		
		#if defined(TCPIP_STACK_USE_SSL_SERVER)
		// Check if we've received an SSLv2 ClientHello message
		// Client-only implementations don't need to deal with this
		if((sslStub(netIx).rxProtocol & 0x80) == 0x80)
		{
			// After MSB, next 15 bits are the length
			((uint8_t*)&sslStub(netIx).wRxBytesRem)[1] = sslStub(netIx).rxProtocol & 0x7F;
			TCPGet(hTCP, ((uint8_t*)&sslStub(netIx).wRxBytesRem));
			
			// Tell the handshaker what message to expect
			sslStub(netIx).wRxHsBytesRem = sslStub(netIx).wRxBytesRem;
			sslStub(netIx).rxProtocol = SSL_HANDSHAKE;
			sslStub(netIx).rxHSType = SSL_ANTIQUE_CLIENT_HELLO;
		}
		
		// Otherwise, this is a normal SSLv3 message
		// Read the rest of the record header and proceed normally
		else
		#endif
		{
			// Read version (uint16_t, currently ignored)
			TCPGet(hTCP, NULL);
			TCPGet(hTCP, NULL);
	
			// Read length (uint16_t)
			TCPGet(hTCP, ((uint8_t*)&sslStub(netIx).wRxBytesRem)+1);
			TCPGet(hTCP, ((uint8_t*)&sslStub(netIx).wRxBytesRem));
			
			// Determine if a MAC is expected
			if(sslStub(netIx).Flags.bRemoteChangeCipherSpec)
			{
				sslStub(netIx).Flags.bExpectingMAC = 1;
				sslStub(netIx).wRxBytesRem -= 16;
							
				// Set up the MAC
				SSLKeysSync(sslStubID(netIx));
				SSLHashSync(sslStub(netIx).idRxHash);
				SSLMACBegin(sslKeys(netIx).Remote.app.MACSecret, 
					sslKeys(netIx).Remote.app.sequence++, 
					sslStub(netIx).rxProtocol, sslStub(netIx).wRxBytesRem);
			}
		}
		
	}
	
	// See if data is ready that needs decryption
	wLen = TCPIsGetReady(hTCP);

	// Decrypt and MAC if necessary
	if(sslStub(netIx).Flags.bRemoteChangeCipherSpec && wLen)
	{// Need to decrypt the data
		
		// Only decrypt up to end of record
		if(wLen > sslStub(netIx).wRxBytesRem)
			wLen = sslStub(netIx).wRxBytesRem;
						
		// Prepare for decryption
		SSLKeysSync(id);
		SSLBufferSync(sslStub(netIx).idRxBuffer);
		SSLHashSync(sslStub(netIx).idRxHash);

		// Decrypt application data to proper location, non-app in place
		TCPSSLDecryptMAC(hTCP, &sslKeys(netIx).Remote.app.cryptCtx, wLen);
	}
	
	// Determine what to do with the rest of the data
	switch(sslStub(netIx).rxProtocol)
	{
		case SSL_HANDSHAKE:
			SSLRxHandshake(hTCP, sslStubID(netIx));
			break;
			
		case SSL_CHANGE_CIPHER_SPEC:
			SSLRxCCS(hTCP);
			break;
			
		case SSL_APPLICATION:
			// Data was handled above
			// Just note that it's all been read
			sslStub(netIx).wRxBytesRem -= wLen;
			return wLen;
		
		case SSL_ALERT:
			SSLRxAlert(hTCP);
			break;
	}
	
	return 0;
}

/*****************************************************************************
  Function:
	void SSLTxRecord(TCP_SOCKET hTCP, uint8_t id, uint8_t txProtocol)

  Summary:
	Transmits an SSL record.

  Description:
	Transmits all pending data in the TCP TX buffer as an SSL record using
	the specified protocol.  This function transparently encrypts and MACs
	the data if there is an active cipher spec.

  Precondition:
	The specified SSL stub is initialized and the TCP socket is connected.

  Parameters:
	hTCP - The TCP socket with data waiting to be transmitted
	id - The active SSL stub ID
	txPortocol - The SSL protocol number to attach to this record
	
  Returns:
  	None
  ***************************************************************************/
void SSLTxRecord(TCP_SOCKET hTCP, uint8_t id, uint8_t txProtocol)
{
	TCPIP_UINT16_VAL wLen;
	uint8_t hdr[5];
    NET_CONFIG* currIf = (NET_CONFIG*)TCPSocketGetNet(hTCP);
#if SSL_MULTIPLE_INTERFACES
    int netIx = _TCPIPStackNetIx(currIf);
#endif  // SSL_MULTIPLE_INTERFACES
	
	SSLStubSync(currIf, id);
	
	// If stub is done, prevent writing data
	if(sslStub(netIx).Flags.bDone)
		return;
	
	// Determine how many bytes are ready to write
	wLen.Val = TCPSSLGetPendingTxSize(hTCP);
	if(wLen.Val == 0u)
		return;
	
	// Determine if a MAC is required
	if(sslStub(netIx).Flags.bLocalChangeCipherSpec)
	{// Perform the encryption and MAC
		// Sync needed data
		SSLKeysSync(sslStubID(netIx));
		SSLHashSync(SSL_INVALID_ID);
		SSLBufferSync(sslStub(netIx).idTxBuffer);
		
		// Start the MAC calculation
		SSLMACBegin(sslKeys(netIx).Local.app.MACSecret, 
			sslKeys(netIx).Local.app.sequence, txProtocol, wLen.Val);
		sslKeys(netIx).Local.app.sequence++;
		
		// Get ready to send
		TCPSSLInPlaceMACEncrypt(hTCP, &sslKeys(netIx).Local.app.cryptCtx,
				sslKeys(netIx).Local.app.MACSecret, wLen.Val);
		
		// Add MAC length to the data length
		wLen.Val += 16;
	}
	
	// Prepare the header
	hdr[0] = txProtocol;
	hdr[1] = SSL_VERSION_HI;
	hdr[2] = SSL_VERSION_LO;
	hdr[3] = wLen.v[1];
	hdr[4] = wLen.v[0];
	
	// Put the record header and send the data
	TCPSSLPutRecordHeader(hTCP, hdr, true);
	
}

/*****************************************************************************
  Function:
	void SSLStartPartialRecord(TCP_SOCKET hTCP, uint8_t id, uint8_t txProtocol,
								 uint16_t wLen)

  Summary:
	Begins a long SSL record.

  Description:
	This function allows messages longer than the TCP buffer to be sent,
	which is frequently the case for the Certificate handshake message.  The
	final message length is required to be known in order to transmit the
	header.  Once called, SSLFlushPartialRecord and SSLFinishPartialRecord
	must be called to write remaining data, finalize, and prepare for a new
	record.

  Precondition:
	The specified SSL stub is initialized and the TCP socket is connected.

  Parameters:
	hTCP - The TCP socket with data waiting to be transmitted
	id - The active SSL stub ID
	txPortocol - The SSL protocol number to attach to this record
	wLen - The length of all the data to be sent
	
  Returns:
  	None
  
  Remarks:
	Partial messages do not support the current cipher spec, so this can
	only be used during the handshake procedure.
  ***************************************************************************/
#if defined(TCPIP_STACK_USE_SSL_SERVER)
void SSLStartPartialRecord(TCP_SOCKET hTCP, uint8_t id, uint8_t txProtocol, uint16_t wLen)
{
	uint8_t hdr[5];
    NET_CONFIG* currIf = (NET_CONFIG*)TCPSocketGetNet(hTCP);
	
	SSLStubSync(currIf, id);
	
	// Prepare the header
	hdr[0] = txProtocol;
	hdr[1] = SSL_VERSION_HI;
	hdr[2] = SSL_VERSION_LO;
	hdr[3] = wLen >> 8;
	hdr[4] = wLen;
	
	// Put the record header and send the data
	TCPSSLPutRecordHeader(hTCP, hdr, false);
	
}
#endif

/*****************************************************************************
  Function:
	void SSLTxMessage(TCP_SOCKET hTCP, uint8_t id, uint8_t msg)

  Summary:
	Transmits an SSL message.

  Description:
	This function transmits a specific SSL message for handshakes and alert
	messages.  Supported messages are listed in SSL_MESSAGES.

  Precondition:
	The specified SSL stub is initialized and the TCP socket is connected.

  Parameters:
	hTCP - The TCP socket with data waiting to be transmitted
	id - The active SSL stub ID
	msg - One of the SSL_MESSAGES types to send
	
  Returns:
  	None
  ***************************************************************************/
void SSLTxMessage(TCP_SOCKET hTCP, uint8_t id, uint8_t msg)
{
    NET_CONFIG* currIf = (NET_CONFIG*)TCPSocketGetNet(hTCP);
#if SSL_MULTIPLE_INTERFACES
    int netIx = _TCPIPStackNetIx(currIf);
#endif  // SSL_MULTIPLE_INTERFACES
	
    SSLStubSync(currIf, id);
	
	// Don't do anything for terminated connections
	if(sslStub(netIx).Flags.bDone)
		return;
	
	// Transmit the requested message
	switch(msg)
	{
		#if defined(TCPIP_STACK_USE_SSL_CLIENT)
		case SSL_CLIENT_HELLO:
			SSLTxClientHello(hTCP);
			break;
		case SSL_CLIENT_KEY_EXCHANGE:
			SSLTxClientKeyExchange(hTCP);
			break;
		#endif
		
		#if defined(TCPIP_STACK_USE_SSL_SERVER)
		case SSL_SERVER_HELLO:
			SSLTxServerHello(hTCP);
			break;
		case SSL_CERTIFICATE:
			SSLTxServerCertificate(hTCP);
			break;
		case SSL_SERVER_HELLO_DONE:
			SSLTxServerHelloDone(hTCP);
			break;
		#endif
		
		case SSL_CHANGE_CIPHER_SPEC:
			SSLTxCCSFin(hTCP);
			break;
			
		// Handle all alert messages
		default:
			if((msg & 0x80) != 0x80)
				break;
			
			// Make sure we can write the message
			if(TCPIsPutReady(hTCP) < 2u)
				break;
			
			// Select FATAL or WARNING
			if(msg == SSL_ALERT_CLOSE_NOTIFY)
			{
				TCPPut(hTCP, SSL_ALERT_WARNING);
				sslStub(netIx).Flags.bCloseNotify = 1;
			}
			else
				TCPPut(hTCP, SSL_ALERT_FATAL);
			
			// Put the message byte
			TCPPut(hTCP, msg - 0x80);
			
			// Flush the message
			SSLTxRecord(hTCP, sslStubID(netIx), SSL_ALERT);
			TCPRequestSSLMessage(hTCP, SSL_NO_MESSAGE);
			
			// Mark session as terminated
			sslStub(netIx).Flags.bDone = 1;
	}
	
}

/*****************************************************************************
  Function:
	void SSLRxHandshake(TCP_SOCKET hTCP, uint8_t id)

  Summary:
	Receives a handshake message.

  Description:
	This function receives handshake messages, reads the handshake header,
	and passes the data off to the appropriate handshake parser. 

  Precondition:
	The specified SSL stub is initialized and the TCP socket is connected.
	Also requires that rxBytesRem has been populated and the current SSL stub
	has been synced into memory.

  Parameters:
	hTCP - The TCP socket to read a handshake message from
	id - The active SSL stub ID
	
  Returns:
  	None
  ***************************************************************************/
void SSLRxHandshake(TCP_SOCKET hTCP, uint8_t id)
{
	uint16_t wLen;
    NET_CONFIG* currIf = (NET_CONFIG*)TCPSocketGetNet(hTCP);
#if SSL_MULTIPLE_INTERFACES
    int netIx = _TCPIPStackNetIx(currIf);
#endif  // SSL_MULTIPLE_INTERFACES
    
	SSLStubSync(currIf, id);   
	
	
	// Start reading handshake data
	HSStart();
	
	// If this is a new handshake message, read the header
	// If the message has already been started, there will
	// still be bytes remaining and the switch statement will 
	// handle the rest.
	if(sslStub(netIx).wRxHsBytesRem == 0u)
	{
		// Make sure entire header is in the buffer
		if(TCPIsGetReady(hTCP) < 4u)
			return;
		
		// Read the message type (uint8_t)
		HSGet(hTCP, &sslStub(netIx).rxHSType);
		
		// Read the length (3 BYTES)
		HSGet(hTCP, NULL);
		HSGetWord(hTCP, &wLen);
		sslStub(netIx).wRxHsBytesRem = wLen;
	}
	
	// Determine what to do with the rest of the data
	switch(sslStub(netIx).rxHSType)
	{
		#if defined(TCPIP_STACK_USE_SSL_CLIENT)
		case SSL_SERVER_HELLO:
			SSLRxServerHello(hTCP);
			break;

		case SSL_CERTIFICATE:
			SSLRxServerCertificate(hTCP);
			break;
			
		case SSL_SERVER_HELLO_DONE:
			// This message contains no data
			// Record that message was received
			sslStub(netIx).Flags.bServerHelloDone = 1;
			break;
		#endif
		
		#if defined(TCPIP_STACK_USE_SSL_SERVER)
		case SSL_ANTIQUE_CLIENT_HELLO:
			SSLRxAntiqueClientHello(hTCP);
			break;

		case SSL_CLIENT_HELLO:
			SSLRxClientHello(hTCP);
			break;
			
		case SSL_CLIENT_KEY_EXCHANGE:
			SSLRxClientKeyExchange(hTCP);
			break;
		#endif
		
		case SSL_FINISHED:
			SSLRxFinished(hTCP);
			break;
	}
	
	// End reading handshake data
	HSEnd();
	
}	


/****************************************************************************
  ===========================================================================
  Section:
	SSL Message Processing Functions
  ===========================================================================
  ***************************************************************************/

/*********************************************************************
 * Function:        uint8_t SSLTxClientHello(TCP_SOCKET hTCP)
 *
 * PreCondition:    Enough space is available in hTCP to write the
 *					entire message. 
 *
 * Input:           hTCP - the TCP Socket to send the message to
 *
 * Output:          None
 *
 * Side Effects:    None
 *
 * Overview:        Transmits the ClientHello message to initiate a
 *					new SSL session with the server.
 *
 * Note:            None
 ********************************************************************/
#if defined(TCPIP_STACK_USE_SSL_CLIENT)
static void SSLTxClientHello(TCP_SOCKET hTCP)
{	
    TCP_SOCKET_INFO sktInfo;
#if SSL_MULTIPLE_INTERFACES
    int netIx = _TCPIPStackNetIx(sslSyncIf);
#endif  // SSL_MULTIPLE_INTERFACES

	// Restart the handshake hasher
	HSStart();
	
	// Indicate that we're the client
	//sslStub(netIx).Flags.bIsServer = 0;  // This is the default already
	
	// Make sure enough space is available to transmit
	if(TCPIsPutReady(hTCP) < 100u)
		return;
	
	// Look for a valid session to reuse
    TCPGetSocketInfo(hTCP, &sktInfo);
	sslStub(netIx).idSession = SSLSessionMatchIP(sktInfo.remoteIPaddress.v4Add);
	sslStub(netIx).Flags.bNewSession = (sslStub(netIx).idSession == SSL_INVALID_ID);
	
	// If none is found, generate a new one
	if(sslStub(netIx).Flags.bNewSession)
	{
		sslStub(netIx).idSession = SSLSessionNew();
		if(sslStub(netIx).idSession == SSL_INVALID_ID)
		{// No free sessions, so abort
			return;
		}

		// Mark session as using this IP
		memcpy((void*)&sslSessionStubs(netIx)[sslStub(netIx).idSession].tag.v[0],
				(void*)&sktInfo.remoteIPaddress.v4Add, 4);
	}

	// Send handshake message header (hashed)
	HSPut(hTCP, SSL_CLIENT_HELLO);
	HSPut(hTCP, 0x00);				
	HSPut(hTCP, 0x00);				// Message length is 40 bytes,
	if(sslStub(netIx).Flags.bNewSession)	// plus 32 more if a session
		HSPut(hTCP, 43);			// ID is being included.
	else
		HSPut(hTCP, 43+32);
	
	// Send 
	HSPut(hTCP, SSL_VERSION_HI);
	HSPut(hTCP, SSL_VERSION_LO);
	
	// Put Client.Random
	HSPutArray(hTCP, sslKeys(netIx).Local.random, 32);
	
	// Put Session ID
	if(sslStub(netIx).Flags.bNewSession)
	{// Send no session ID
		HSPut(hTCP, 0x00);
	}
	else
	{// Send the requested Session ID
		SSLSessionSync(sslStub(netIx).idSession);
		HSPut(hTCP, 0x20);
		HSPutArray(hTCP, sslSession(netIx).sessionID, 32);
	}
	
	// Put Cipher Suites List
	HSPutWord(hTCP, 0x0004);
	HSPutWord(hTCP, SSL_RSA_WITH_ARCFOUR_128_MD5);
	HSPutWord(hTCP, SSL_RSA_EXPORT_WITH_ARCFOUR_40_MD5);
	
	// Put Compression Methods List (just null)
	HSPut(hTCP, 0x01);
	HSPut(hTCP, 0x00);
	
	// End the handshake and save the hash
	HSEnd();
	
	// Send record
	SSLTxRecord(hTCP, sslStubID(netIx), SSL_HANDSHAKE);
	
	// Record that message was sent
	TCPRequestSSLMessage(hTCP, SSL_NO_MESSAGE);
	sslStub(netIx).Flags.bClientHello = 1;

}
#endif

/*********************************************************************
 * Function:        uint8_t SSLRxClientHello(TCP_SOCKET hTCP)
 *
 * PreCondition:    Handshake hasher is started, and SSL has a stub
 *					assigned. 
 *
 * Input:           hTCP - the TCP Socket to send the message to
 *
 * Output:          None
 *
 * Side Effects:    None
 *
 * Overview:        Receives the ClientHello message, initiating a
 *					new SSL session with a client
 *
 * Note:            None
 ********************************************************************/
#if defined(TCPIP_STACK_USE_SSL_SERVER)
static void SSLRxClientHello(TCP_SOCKET hTCP)
{
	uint16_t w;
	uint8_t c, *ptrID;
#if SSL_MULTIPLE_INTERFACES
    int netIx = _TCPIPStackNetIx(sslSyncIf);
#endif  // SSL_MULTIPLE_INTERFACES
	
	// Make sure entire message is ready
	if(TCPIsGetReady(hTCP) < sslStub(netIx).wRxHsBytesRem)
		return;
		
	// Verify handshake message sequence
	if(sslStub(netIx).Flags.bClientHello)
		TCPRequestSSLMessage(hTCP, SSL_ALERT_HANDSHAKE_FAILURE);
	
	// Indicate that we're the server
	sslStub(netIx).Flags.bIsServer = 1;
	
	// Read the version again
	HSGetWord(hTCP, &w);
	// Ignore the version here.  It must be at least 3.0 to receive this type
	// of message, and Safari 3.1 sends 0x0301 (TLS 1.0) even when the last 
	// connection was only 0x0300 (SSL 3.0)
		
	// Make sure the session keys are synced
	SSLKeysSync(sslStubID(netIx));
	
	// Read the Client.Random array
	HSGetArray(hTCP, sslKeys(netIx).Remote.random, 32);
	
	// Read the Session ID length
	HSGet(hTCP, &c);
	
	// Read the Session ID if it exists
	sslStub(netIx).Flags.bNewSession = true;
	if(c > 0u)
	{
		// Note where it will be stored in RAM
		ptrID = ptrHS(netIx);
		HSGetArray(hTCP, NULL, c);
		
		// Try to match it with a known session
		sslStub(netIx).idSession = SSLSessionMatchID(ptrID);
		if(sslStub(netIx).idSession != SSL_INVALID_ID)
			sslStub(netIx).Flags.bNewSession = false;
	}
	
	// If we we're starting a new session, try to obtain a free one
	if(sslStub(netIx).Flags.bNewSession)
		sslStub(netIx).idSession = SSLSessionNew();
	
	// Read CipherSuites length
	HSGetWord(hTCP, &w);
	
	// Check for an acceptable CipherSuite
	// Right now we just ignore this and assume support for 
	// SSL_RSA_WITH_ARCFOUR_128_MD5.  If we request this suite later 
	// and it isn't supported, the client will kill the connection.
	HSGetArray(hTCP, NULL, w);
	
	// Read the Compression Methods length
	HSGet(hTCP, &c);
	
	// Check for an acceptable Compression Method
	// Right now we just ignore this and assume support for
	// NULL_COMPRESSION.  If we request this later and the client
	// doesn't really support it, they'll kill the connection.
	HSGetArray(hTCP, NULL, c);
	
	// For TLS compatibility, we must ignore further bytes in ClientHello.
	// FF2+ may send "extensions" ad other things we don't support
	HSGetArray(hTCP, NULL, sslStub(netIx).wRxBytesRem);
	
	// Mark message as received and request a ServerHello
	sslStub(netIx).Flags.bClientHello = 1;
	TCPRequestSSLMessage(hTCP, SSL_SERVER_HELLO);

}
#endif

/*********************************************************************
 * Function:        uint8_t SSLRxAntiqueClientHello(TCP_SOCKET hTCP)
 *
 * PreCondition:    Handshake hasher is started, and SSL has a stub
 *					assigned.
 *
 * Input:           hTCP - the TCP Socket to send the message to
 *
 * Output:          None
 *
 * Side Effects:    None
 *
 * Overview:        Receives the SSLv2 ClientHello message, initiating
 *					a new SSL session with a client
 *
 * Note:            This is the only SSLv2 message we support, and
 *					is provided for browsers seeking backwards
 *					compatibility.  Connections must be upgraded to
 *					SSLv3.0 immediately following, otherwise the 
 *					connection will fail.
 ********************************************************************/
#if defined(TCPIP_STACK_USE_SSL_SERVER)
static void SSLRxAntiqueClientHello(TCP_SOCKET hTCP)
{
	uint16_t suiteLen, idLen, randLen;
	uint8_t c;
#if SSL_MULTIPLE_INTERFACES
    int netIx = _TCPIPStackNetIx(sslSyncIf);
#endif  // SSL_MULTIPLE_INTERFACES
	
	// Make sure entire message is ready
	if(TCPIsGetReady(hTCP) < sslStub(netIx).wRxHsBytesRem)
		return;
		
	// Verify handshake message sequence
	if(sslStub(netIx).Flags.bClientHello)
		TCPRequestSSLMessage(hTCP, SSL_ALERT_HANDSHAKE_FAILURE);
		
	// Indicate that we're the server
	sslStub(netIx).Flags.bIsServer = 1;
	
	// Make sure the session keys are synced
	SSLKeysSync(sslStubID(netIx));
	
	// Read and verify the handshake message type
	HSGet(hTCP, &c);
	if(c != 0x01u)
	{// This message is not supported, so handshake fails
		TCPRequestSSLMessage(hTCP, SSL_ALERT_HANDSHAKE_FAILURE);
		return;
	}
	
	// Read and verify the version
	HSGet(hTCP, &c);
	if(c != SSL_VERSION_HI)
	{// Version is too low, so handshake fails
		TCPRequestSSLMessage(hTCP, SSL_ALERT_HANDSHAKE_FAILURE);
		return;
	}
	HSGet(hTCP, &c);	// Ignore low byte of version number
	
	// Read the CipherSuite length
	HSGetWord(hTCP, &suiteLen);
	
	// Read Session ID Length
	HSGetWord(hTCP, &idLen);
	
	// Read Challenge (Client.Random) length
	HSGetWord(hTCP, &randLen);
		
	// Check for an acceptable CipherSuite
	// Right now we just ignore this and assume support for 
	// SSL_RSA_WITH_ARCFOUR_128_MD5.  If we request this suite later 
	// and it isn't supported, the client will kill the connection.
	HSGetArray(hTCP, NULL, suiteLen);
	
	// Read the SessionID
	// SSLv3 clients will send a v3 ClientHello when resuming, so
	// this is always a new session.  Therefore, ignore the ID
	HSGetArray(hTCP, NULL, idLen);
	
	// Obtain a new session
	sslStub(netIx).idSession = SSLSessionNew();
	sslStub(netIx).Flags.bNewSession = 1;
	
	// Read Client.Random
	// This needs to be 32 bytes, so zero-pad the left side
	for(c = 0; c < 32 - randLen; c++)
		sslKeys(netIx).Remote.random[c] = 0;
	HSGetArray(hTCP, &sslKeys(netIx).Remote.random[c], randLen);
	
	// Mark message as received and request a ServerHello
	TCPRequestSSLMessage(hTCP, SSL_SERVER_HELLO);
	sslStub(netIx).Flags.bClientHello = 1;
}
#endif

/*********************************************************************
 * Function:        void SSLRxServerHello(TCP_SOCKET hTCP)
 *
 * PreCondition:    sslStub(netIx) is synchronized and HSStart() has been
 *					called.
 *
 * Input:           hTCP - the TCP Socket to read from
 *
 * Output:          None
 *
 * Side Effects:    None
 *
 * Overview:        Receives the ServerHello from the remote server
 *
 * Note:            None
 ********************************************************************/
#if defined(TCPIP_STACK_USE_SSL_CLIENT)
static void SSLRxServerHello(TCP_SOCKET hTCP)
{
	uint8_t b, sessionID[32];
	uint16_t w;
#if SSL_MULTIPLE_INTERFACES
    int netIx = _TCPIPStackNetIx(sslSyncIf);
#endif  // SSL_MULTIPLE_INTERFACES
		
	// Make sure entire message is ready
	if(TCPIsGetReady(hTCP) < sslStub(netIx).wRxHsBytesRem)
		return;
		
	// Verify handshake message sequence
	if(!sslStub(netIx).Flags.bClientHello || sslStub(netIx).Flags.bServerHello)
		TCPRequestSSLMessage(hTCP, SSL_ALERT_HANDSHAKE_FAILURE);
	
	// Make sure correct session and key set are loaded
	SSLKeysSync(sslStubID(netIx));
	
	// Read Version (2)
	HSGetWord(hTCP, NULL);
	
	// Read Server.Random (32)
	HSGetArray(hTCP, sslKeys(netIx).Remote.random, 32);
	
	// Read Session ID Length (byte)
	HSGet(hTCP, &b);
	
	// Read Session ID (if any)
	SSLSessionSync(sslStub(netIx).idSession);
	if(b != 0u)
	{
		HSGetArray(hTCP, sessionID, b);

		// If reusing a session, check if our session ID was accepted
		if(!sslStub(netIx).Flags.bNewSession &&
			memcmp((void*)sslSession(netIx).sessionID, (void*)sessionID, 32) == 0)
		{// Session restart was accepted
			// Nothing to do here...move along
		}
		else
		{// This is a new session
			memcpy((void*)sslSession(netIx).sessionID, (void*)sessionID, 32);

			// Reset the RxServerCertificate state machine
			sslStub(netIx).dwTemp.v[0] = RX_SERVER_CERT_START;
		}
	}
	else
	{
		// Session is non-resumable, so invalidate its tag
		sslSessionStubs(netIx)[sslStub(netIx).idSession].tag.Val = 0;
	}
	
	// Read and verify Cipher Suite (uint16_t)
	HSGetWord(hTCP, &w);
	if(w != SSL_RSA_WITH_ARCFOUR_128_MD5)
		TCPRequestSSLMessage(hTCP, SSL_ALERT_HANDSHAKE_FAILURE);
	
	// Read and verify Compression Method (uint8_t)
	HSGet(hTCP, &b);
	if(b != 0x00u)
		TCPRequestSSLMessage(hTCP, SSL_ALERT_HANDSHAKE_FAILURE);
	
	// Note that message was received
	sslStub(netIx).Flags.bServerHello = 1;
	
	// Note that we updated session data
	SSLSessionUpdated(netIx);
}
#endif

/*********************************************************************
 * Function:        uint8_t SSLTxServerHello(TCP_SOCKET hTCP)
 *
 * PreCondition:    None
 *
 * Input:           hTCP - the TCP Socket to send the message to
 *
 * Output:          None
 *
 * Side Effects:    None
 *
 * Overview:        Transmits the ServerHello message.
 *
 * Note:            None
 ********************************************************************/
#if defined(TCPIP_STACK_USE_SSL_SERVER)
static void SSLTxServerHello(TCP_SOCKET hTCP)
{
	uint8_t i;
#if SSL_MULTIPLE_INTERFACES
    int netIx = _TCPIPStackNetIx(sslSyncIf);
#endif  // SSL_MULTIPLE_INTERFACES
	
	// Only continue if the session has been obtained
	if(sslStub(netIx).idSession == SSL_INVALID_ID)
		return;
	
	// Make sure enough space is available to transmit
	if(TCPIsPutReady(hTCP) < 78u)
		return;
	
	// Restart the handshake hasher
	HSStart();
	
	// Sync the session and keys
	SSLKeysSync(sslStubID(netIx));
	SSLSessionSync(sslStub(netIx).idSession);
	
	// If this session is new, generate an ID
	if(sslStub(netIx).Flags.bNewSession)
	{
		for(i = 0; i < 32u; i++)
			sslSession(netIx).sessionID[i] = SYS_RANDOM_GET();
		SSLSessionUpdated(netIx);
		
		// Tag this session identifier
		memcpy((void*)&sslSessionStubs(netIx)[sslStub(netIx).idSession].tag.v[1],
			(void*)(sslSession(netIx).sessionID), 3);
	}

	// Send handshake message header (hashed)
	HSPut(hTCP, SSL_SERVER_HELLO);
	HSPut(hTCP, 0x00);				
	HSPut(hTCP, 0x00);				
	HSPut(hTCP, 70);
	
	// Send the version number
	HSPut(hTCP, SSL_VERSION_HI);
	HSPut(hTCP, SSL_VERSION_LO);
	
	// Put Server.Random
	#if defined(TCPIP_STACK_USE_SNTP_CLIENT)
	{
		TCPIP_UINT32_VAL temp;
		
		temp.Val = SNTPGetUTCSeconds();
		sslKeys(netIx).Local.random[0] = temp.v[3];
		sslKeys(netIx).Local.random[1] = temp.v[2];
		sslKeys(netIx).Local.random[2] = temp.v[1];
		sslKeys(netIx).Local.random[3] = temp.v[0];
		i = 4;
	}
	#else
		i = 0;
	#endif
	while(i < 32u)
		sslKeys(netIx).Local.random[i++] = SYS_RANDOM_GET();
	HSPutArray(hTCP, sslKeys(netIx).Local.random, 32);
	
	// Put Session ID
	HSPut(hTCP, 0x20);
	HSPutArray(hTCP, sslSession(netIx).sessionID, 32);
	
	// Put Cipher Suites
	HSPutWord(hTCP, SSL_RSA_WITH_ARCFOUR_128_MD5);
	
	// Put Compression Method (just null)
	HSPut(hTCP, 0x00);
	
	// End the handshake and save the hash
	HSEnd();
	
	// Send record
	SSLTxRecord(hTCP, sslStubID(netIx), SSL_HANDSHAKE);
	
	// Record that message was sent and request the next message
	sslStub(netIx).Flags.bServerHello = 1;
	TCPRequestSSLMessage(hTCP, SSL_NO_MESSAGE);
	if(sslStub(netIx).Flags.bNewSession)
		TCPRequestSSLMessage(hTCP, SSL_CERTIFICATE);
	else
		TCPRequestSSLMessage(hTCP, SSL_CHANGE_CIPHER_SPEC);
	
	// Set up to transmit certificate
	sslStub(netIx).dwTemp.Val = SSL_CERT_LEN;
}
#endif

/*********************************************************************
 * Function:        void SSLRxServerCertificate(TCP_SOCKET hTCP)
 *
 * PreCondition:    sslStub(netIx) is synchronized and HSStart() has been
 *					called.
 *
 * Input:           hTCP - the TCP Socket to read from
 *
 * Output:          None
 *
 * Side Effects:    None
 *
 * Overview:        Receives ServerCertificate from the remote server,
 *					locates the public key information, and executes
 *					RSA operation.
 *
 * Note:            This shortcuts full parsing of the certificate by
 *					just finding the Public Key Algorithm identifier
 *					for RSA.  From there, the following ASN.1 struct
 *					is the public key.  That struct consists of the
 *					value for N, followed by the value for E.
 ********************************************************************/
#if defined(TCPIP_STACK_USE_SSL_CLIENT)
static void SSLRxServerCertificate(TCP_SOCKET hTCP)
{
	uint16_t len;
	uint8_t i, e[3];
	uint16_t data_length;   // number of key bytes read from certificate
	uint8_t length_bytes;  // decoded length value
	uint8_t index;         // temp index


#if SSL_MULTIPLE_INTERFACES
    int netIx = _TCPIPStackNetIx(sslSyncIf);
#endif  // SSL_MULTIPLE_INTERFACES
	
	// Verify handshake message sequence
	if(!sslStub(netIx).Flags.bServerHello || sslStub(netIx).Flags.bServerCertificate)
		TCPRequestSSLMessage(hTCP, SSL_ALERT_HANDSHAKE_FAILURE);
	
	// Check state machine variable
	switch(sslStub(netIx).dwTemp.v[0]) {

		case RX_SERVER_CERT_START:
			// Find RSA Public Key Algorithm identifier
			len = TCPFindArray(hTCP, (const uint8_t*)"\x2A\x86\x48\x86\xF7\x0D\x01\x01\x01", 9, 0, 0, false);
			
			if(len == 0xFFFFu)
			{// If not found, clear all but 10 bytes and return to wait

                data_length = TCPIsGetReady(hTCP);
                if(data_length > 10)
                {
				    HSGetArray(hTCP, NULL, data_length-10);
                }
				return;
			}
			
			// Otherwise, read it and move on
			HSGetArray(hTCP, NULL, len + 9);
			sslStub(netIx).dwTemp.v[0]++;
					
		case RX_SERVER_CERT_FIND_KEY:
			// Search for beginning of struct
            data_length = TCPIsGetReady(hTCP);

			len = TCPFind(hTCP, 0x30, 0, 0, false);
			
			if(len == 0xFFFFu)
			{// Not found, so clear and return
				HSGetArray(hTCP, NULL, data_length);
                return;
			}
			
			// Clear up through the 0x30
			HSGetArray(hTCP, NULL, len + 1);
			
			// Increment and continue
			sslStub(netIx).dwTemp.v[0]++;
		
		case RX_SERVER_CERT_FIND_N:
            // Make sure tag and length bytes are ready, plus one more
            len = TCPIsGetReady(hTCP);
			if(len < 11u)
				return;
			
			// Read 1 or 2 length bytes (ignore)
			HSGet(hTCP, &i);
            data_length = 0;
			if(i & 0x80)
            {
                length_bytes = i & 0x7F;
                for(index=0;index<length_bytes;index++)
                {
				HSGet(hTCP, &i);
    				data_length = (data_length<<8)+i;
                }
            }
            else
                data_length = i;

		
            // Read until 0x02  (should be the next byte)
            i = 0;
            while((i != 2) && (len > 0))
            {
                len--;
                HSGet(hTCP, &i);
            }
            if(len == 0)    // abort if 0x02 is not found
            {
        		TCPRequestSSLMessage(hTCP, SSL_ALERT_HANDSHAKE_FAILURE);
				return;
            }

			// Read 1 or 2 length bytes to sslStub.dwTemp.v[1]
			// The next byte tells us how many bytes are in the length structure if it's MSB is set
			HSGet(hTCP, &i);
			data_length = 0;
			if(i & 0x80)
			{
    			length_bytes = i & 0x7F;
    			for(index=0;index<length_bytes;index++)
    			{
				HSGet(hTCP, &i);
    				data_length = (data_length<<8)+i;
    			}
        	}
            else
                data_length = i; // fix for single byte length < 0x80

			
            sslStub(netIx).dwTemp.w[1] = data_length;
			// If there's one odd byte, it's a leading zero that we don't need
			if(sslStub(netIx).dwTemp.w[1] & 0x01)
			{
				HSGet(hTCP, NULL);
				sslStub(netIx).dwTemp.w[1]--;
			}
			
			// The max modulus we support is 2048 bits
			if(sslStub(netIx).dwTemp.w[1] > SSL_RSA_CLIENT_SIZE/8)
			{
				TCPRequestSSLMessage(hTCP, SSL_ALERT_HANDSHAKE_FAILURE);
				sslStub(netIx).dwTemp.w[1] = SSL_RSA_CLIENT_SIZE/8;
			}

			// Increment and continue
			sslStub(netIx).dwTemp.v[0]++;
            // fall through to RX_SERVER_CERT_READ_N 
		
		case RX_SERVER_CERT_READ_N:
			// Make sure sslStub(netIx).dwTemp.w[1] bytes are ready
            data_length = TCPIsGetReady(hTCP);
			if(data_length < sslStub(netIx).dwTemp.w[1])
				return;
			
			// N will be stored in sslBuffer(netIx), which is currently in use
			// for handshaking.  We can stop the handshake hashing, read 
			// and hash this data, then resume more handshake hashing
			HSEnd();
			
			// Claim an SSL Buffer for RSA operations
			SSLBufferAlloc(&sslStub(netIx).idRxBuffer);
			if(sslStub(netIx).idRxBuffer == SSL_INVALID_ID)
				return;
				
			// Make sure we can claim RSA Engine
			if(!RSABeginEncrypt(sslSyncIf, sslStub(netIx).dwTemp.w[1]))
				return;
			sslRSAStubID(netIx) = sslStubID(netIx);
			
			// Read N to proper location
			HSGetArray(hTCP, sslBuffer(netIx).full, sslStub(netIx).dwTemp.w[1]);
			
            if (sslStub(netIx).supplementaryDataType == SSL_SUPPLEMENTARY_DATA_CERT_PUBLIC_KEY)
            {
                SSL_PKEY_INFO * tmpPKeyPtr = ((SSL_PKEY_INFO *)sslStub(netIx).supplementaryBuffer);
                tmpPKeyPtr->pub_size_bytes = sslStub(netIx).dwTemp.w[1];
                if (tmpPKeyPtr->pub_size_bytes <= sizeof (tmpPKeyPtr->pub_key))
                    memcpy (&tmpPKeyPtr->pub_key[0], sslBuffer(netIx).full, tmpPKeyPtr->pub_size_bytes);
            }


			// Hash what we just read
			SSLHashSync(sslStub(netIx).idSHA1);
			HashAddData(&sslHash(netIx), sslBuffer(netIx).full, sslStub(netIx).dwTemp.w[1]);
			SSLHashSync(sslStub(netIx).idMD5);
			HashAddData(&sslHash(netIx), sslBuffer(netIx).full, sslStub(netIx).dwTemp.w[1]);
			
			// Generate { SSL_VERSION rand[46] } as pre-master secret & save
			SSLSessionSync(sslStub(netIx).idSession);
			sslSession(netIx).masterSecret[0] = SSL_VERSION_HI;
			sslSession(netIx).masterSecret[1] = SSL_VERSION_LO;
			for(i = 2; i < 48u; i++)
				sslSession(netIx).masterSecret[i] = SYS_RANDOM_GET();
			SSLSessionUpdated(netIx);
			
			// Set RSA engine to use this data and key
			RSASetData(sslSyncIf, sslSession(netIx).masterSecret, 48, RSA_BIG_ENDIAN);
			RSASetN(sslBuffer(netIx).full, RSA_BIG_ENDIAN);
			RSASetResult(sslBuffer(netIx).full+sslStub(netIx).dwTemp.w[1], RSA_BIG_ENDIAN);
			
			// Start a new hash
			HSStart();
			
			// Increment and continue
			sslStub(netIx).dwTemp.v[0]++;
			
		case RX_SERVER_CERT_READ_E:
			// Make sure 5 bytes are ready
			if(TCPIsGetReady(hTCP) < 5u)
				return;

			// Read 0x02
			HSGet(hTCP, NULL);
			
			// Read 1 length byte to temp
			HSGet(hTCP, &i);
			if(i > 3u)
				i = 3;
			
			// Read E to temp
			HSGetArray(hTCP, e, i);

            if (sslStub(netIx).supplementaryDataType == SSL_SUPPLEMENTARY_DATA_CERT_PUBLIC_KEY)
            {
                SSL_PKEY_INFO * tmpPKeyPtr = ((SSL_PKEY_INFO *)sslStub(netIx).supplementaryBuffer);
                if (i <= sizeof (tmpPKeyPtr->pub_e))
                    memcpy (&tmpPKeyPtr->pub_e[0], e, i);
            }

			// Set RSA engine to encrypt with E
			RSASetE(e, i, RSA_BIG_ENDIAN);
			
			// Increment and continue
			sslStub(netIx).dwTemp.v[0]++;
		
		case RX_SERVER_CERT_CLEAR:
			// Clear up to sslStub(netIx).wRxHsBytesRem from hTCP
			len = TCPIsGetReady(hTCP);
			if(len > sslStub(netIx).wRxHsBytesRem)
				len = sslStub(netIx).wRxHsBytesRem;
			HSGetArray(hTCP, NULL, len);
			
			// If we're done, kick off the RSA encryption next
			if(sslStub(netIx).wRxHsBytesRem == 0u)
			{
				// Set periodic function to do RSA operation
				sslStub(netIx).Flags.bRSAInProgress = 1;
				sslSessionStubs(netIx)[sslStub(netIx).idSession].lastUsed += SSL_RSA_LIFETIME_EXTENSION;
				
				// Note that we've received this message
				sslStub(netIx).Flags.bServerCertificate = 1;	
			}
			
			break;
	}
}
#endif

/*********************************************************************
 * Function:        uint8_t SSLTxServerCertificate(TCP_SOCKET hTCP)
 *
 * PreCondition:    None
 *
 * Input:           hTCP - the TCP Socket to send the message to
 *
 * Output:          None
 *
 * Side Effects:    None
 *
 * Overview:        Transmits the Certificate message with the 
 *					server's specified public key certificate.
 *
 * Note:            Certificate is defined in CustomSSLCert.c.
 *					This function requires special handling for
 *					partial records because the certificate will 
 *					likely be larger than the TCP buffer, and SSL
 *					handshake messages are constrained to fit in a
 *					single SSL handshake record
 ********************************************************************/
#if defined(TCPIP_STACK_USE_SSL_SERVER)
static void SSLTxServerCertificate(TCP_SOCKET hTCP)
{
	uint16_t len;
	const uint8_t* loc;
#if SSL_MULTIPLE_INTERFACES
    int netIx = _TCPIPStackNetIx(sslSyncIf);
#endif  // SSL_MULTIPLE_INTERFACES
	
	// Restart the handshake hasher
	HSStart();
	
	// See how much we can write
	len = TCPIsPutReady(hTCP);
	
	// If full certificate remains, write the headers
	if(sslStub(netIx).dwTemp.Val == SSL_CERT_LEN)
	{
		// Make sure we can send all headers plus one byte
		if(len < 11u)
			return;
		
		// Transmit the handshake headers
		HSPut(hTCP, SSL_CERTIFICATE);
		HSPut(hTCP, 0x00);
		HSPutWord(hTCP, SSL_CERT_LEN + 3 + 3);
		
		// Send length of all certificates
		HSPut(hTCP, 0x00);
		HSPutWord(hTCP, SSL_CERT_LEN + 3);
		
		// Send length of this (only) certificate
		HSPut(hTCP, 0x00);
		HSPutWord(hTCP, SSL_CERT_LEN);
		
		// Put in the record header and begin the partial record sending
		SSLStartPartialRecord(hTCP, sslStubID(netIx), SSL_HANDSHAKE, SSL_CERT_LEN + 3 + 3 + 4);
		
		// Update free space
		len -= 10;
	}
	
	// Figure out where to start, and how much to send
	loc = SSL_CERT + (SSL_CERT_LEN - sslStub(netIx).dwTemp.Val);
	if(sslStub(netIx).dwTemp.Val < len)
		len = sslStub(netIx).dwTemp.Val;
		
	// Write the bytes
	HSPutROMArray(hTCP, loc, len);
	sslStub(netIx).dwTemp.Val -= len;
	
	// End the handshake and save the hash
	HSEnd();
	
	// Send record
	SSLFlushPartialRecord(hTCP);
		
	// Check if entire certificate was sent
	if(sslStub(netIx).dwTemp.Val == 0u)
	{
		// Finish the partial record
		SSLFinishPartialRecord(hTCP);
		
		// Record that message was sent and request a Certificate
		TCPRequestSSLMessage(hTCP, SSL_NO_MESSAGE);
		TCPRequestSSLMessage(hTCP, SSL_SERVER_HELLO_DONE);
		sslStub(netIx).Flags.bServerCertificate = 1;
	}
}
#endif

/*********************************************************************
 * Function:        uint8_t SSLTxServerHelloDone(TCP_SOCKET hTCP)
 *
 * PreCondition:    None
 *
 * Input:           hTCP - the TCP Socket to send the message to
 *
 * Output:          None
 *
 * Side Effects:    None
 *
 * Overview:        Transmits the ServerHelloDone message.
 *
 * Note:            None
 ********************************************************************/
#if defined(TCPIP_STACK_USE_SSL_SERVER)
static void SSLTxServerHelloDone(TCP_SOCKET hTCP)
{
#if SSL_MULTIPLE_INTERFACES
    int netIx = _TCPIPStackNetIx(sslSyncIf);
#endif  // SSL_MULTIPLE_INTERFACES

	// Make sure enough space is available to transmit
	if(TCPIsPutReady(hTCP) < 4u)
		return;
	
	// Restart the handshake hasher
	HSStart();
	
	// Send handshake message header (hashed)
	HSPut(hTCP, SSL_SERVER_HELLO_DONE);
	HSPut(hTCP, 0x00);				
	HSPut(hTCP, 0x00);				
	HSPut(hTCP, 0x00);

	// Message has no content, so we're done
	
	// End the handshake and save the hash
	HSEnd();
	
	// Send record
	SSLTxRecord(hTCP, sslStubID(netIx), SSL_HANDSHAKE);
	
	// Record that message was sent
	TCPRequestSSLMessage(hTCP, SSL_NO_MESSAGE);
	sslStub(netIx).Flags.bServerHelloDone = 1;
}
#endif

/*********************************************************************
 * Function:        void SSLTxClientKeyExchange(TCP_SOCKET hTCP)
 *
 * PreCondition:    sslStub(netIx) is synchronized, sslStub(netIx).dwTemp.v[1]
 *					contains the length of the public key, and 
 *					the RxBuffer contains the encrypted pre-master
 *					secret at address 0x80.
 *
 * Input:           hTCP - the TCP Socket to write the message to
 *
 * Output:          None
 *
 * Side Effects:    None
 *
 * Overview:        Transmits the encrypted pre-master secret to the
 *					server and requests the Change Cipher Spec.  Also
 *					generates the Master Secret from the pre-master
 *					secret that was used.
 *
 * Note:            None
 ********************************************************************/
#if defined(TCPIP_STACK_USE_SSL_CLIENT)
static void SSLTxClientKeyExchange(TCP_SOCKET hTCP)
{
	uint16_t len;
#if SSL_MULTIPLE_INTERFACES
    int netIx = _TCPIPStackNetIx(sslSyncIf);
#endif  // SSL_MULTIPLE_INTERFACES
	
	// Load length of modulus from RxServerCertificate
	len = sslStub(netIx).dwTemp.w[1];
	
	// Make sure there's len+9 bytes free
	if(TCPIsPutReady(hTCP) < len + 9)
		return;
	
	// Start the handshake processor
	HSStart();

	// Send handshake message header (hashed)
	HSPut(hTCP, SSL_CLIENT_KEY_EXCHANGE);
	HSPut(hTCP, 0x00);				
	HSPut(hTCP, (len>>8)&0xFF);				// Message length is (length of key) bytes
	HSPut(hTCP, len&0xFF);
	
	// Suspend the handshake hasher and load the buffer
	HSEnd();
	SSLBufferSync(sslStub(netIx).idRxBuffer);	

	// Send encrypted pre-master secret
	TCPPutArray(hTCP, (uint8_t*) sslBuffer(netIx).full + len, len);
	
	// Free the RSA Engine
	RSAEndUsage(sslSyncIf);
	sslRSAStubID(netIx) = SSL_INVALID_ID;

	// Hash what we just sent
	SSLHashSync(sslStub(netIx).idSHA1);
	HashAddData(&sslHash(netIx), sslBuffer(netIx).full + len, len);
	SSLHashSync(sslStub(netIx).idMD5);
	HashAddData(&sslHash(netIx), sslBuffer(netIx).full + len, len);
	
	// Generate the Master Secret
	SSLKeysSync(sslStubID(netIx));
	SSLSessionSync(sslStub(netIx).idSession);
	GenerateHashRounds(3, sslKeys(netIx).Local.random, sslKeys(netIx).Remote.random);
	memcpy(sslSession(netIx).masterSecret, (void*)sslBuffer(netIx).hashRounds.temp, 48);
	SSLSessionUpdated(netIx);
	
	// Free the buffer with the encrypted pre-master secret
	SSLBufferFree(&sslStub(netIx).idRxBuffer);
	
	// Restart the handshaker
	HSStart();
	
	// Send the record
	SSLTxRecord(hTCP, sslStubID(netIx), SSL_HANDSHAKE);
	
	// Request a Change Cipher Spec and Finished message
	TCPRequestSSLMessage(hTCP, SSL_NO_MESSAGE);
	TCPRequestSSLMessage(hTCP, SSL_CHANGE_CIPHER_SPEC);
	
	// Note that this message was sent
	sslStub(netIx).Flags.bClientKeyExchange = 1;
}
#endif

/*********************************************************************
 * Function:        void SSLRxClientKeyExchange(TCP_SOCKET hTCP)
 *
 * PreCondition:    sslStub(netIx) is synchronized and HSStart() has been
 *					called.
 *
 * Input:           hTCP - the TCP Socket to read from
 *
 * Output:          None
 *
 * Side Effects:    None
 *
 * Overview:        Receives the ClientKeyExchange message and begins
 *					the decryption process.
 *
 * Note:            None
 ********************************************************************/
#if defined(TCPIP_STACK_USE_SSL_SERVER)
static void SSLRxClientKeyExchange(TCP_SOCKET hTCP)
{
	uint16_t wKeyLength;
#if SSL_MULTIPLE_INTERFACES
    int netIx = _TCPIPStackNetIx(sslSyncIf);
#endif  // SSL_MULTIPLE_INTERFACES
	
	
	// Make sure entire message is ready
	if(TCPIsGetReady(hTCP) < sslStub(netIx).wRxHsBytesRem)
		return;
	
	// Verify handshake message sequence
	if(!sslStub(netIx).Flags.bServerHello || sslStub(netIx).Flags.bClientKeyExchange)
		TCPRequestSSLMessage(hTCP, SSL_ALERT_HANDSHAKE_FAILURE);
	
	// Obtain a buffer to use
	SSLBufferAlloc(&sslStub(netIx).idRxBuffer);
	if(sslStub(netIx).idRxBuffer == SSL_INVALID_ID)
		return;
	
	// Claim the RSA engine
	if(!RSABeginDecrypt(sslSyncIf))
		return;
	sslRSAStubID(netIx) = sslStubID(netIx);
	
	// Read the data
	wKeyLength = sslStub(netIx).wRxHsBytesRem;
	HSEnd();
	HSStart();
	HSGetArray(hTCP, NULL, wKeyLength);
	HSEnd();
	RSASetData(sslSyncIf, sslBuffer(netIx).full, wKeyLength, RSA_BIG_ENDIAN);
	sslBufferID(netIx) = sslStub(netIx).idRxBuffer;
	
	// Note that message was received
	sslStub(netIx).Flags.bClientKeyExchange = 1;
	
	// Kick off the RSA decryptor
	sslStub(netIx).Flags.bRSAInProgress = 1;
	sslSessionStubs(netIx)[sslStub(netIx).idSession].lastUsed += SSL_RSA_LIFETIME_EXTENSION;
	
}
#endif

/*********************************************************************
 * Function:        void SSLTxCCSFin(TCP_SOCKET hTCP)
 *
 * PreCondition:    sslStub(netIx) is synchronized, and the current session
 *					has a valid pre-master secret to use.
 *
 * Input:           hTCP - the TCP Socket to write the message to
 *
 * Output:          None
 *
 * Side Effects:    None
 *
 * Overview:        Generates the session keys from the master secret,
 *					then allocates and generates the encryption 
 *					context.  Once processing is complete, transmits
 *					the Change Cipher Spec message and the Finished
 *					handshake message to the server.
 *
 * Note:            None
 ********************************************************************/
static void SSLTxCCSFin(TCP_SOCKET hTCP)
{
	uint8_t data[20];
#if SSL_MULTIPLE_INTERFACES
    int netIx = _TCPIPStackNetIx(sslSyncIf);
#endif  // SSL_MULTIPLE_INTERFACES
	
	// Make sure enough space is available for both
	if(TCPIsPutReady(hTCP) < 68u)
		return;

	// Sync up the session
	SSLSessionSync(sslStub(netIx).idSession);
	SSLKeysSync(sslStubID(netIx));
	SSLBufferSync(SSL_INVALID_ID);
			
	// Send the CCS (not a handshake message)
	TCPPut(hTCP, 1);
	SSLTxRecord(hTCP, sslStubID(netIx), SSL_CHANGE_CIPHER_SPEC);
	sslStub(netIx).Flags.bLocalChangeCipherSpec = 1;
	
	// If keys are not ready, generate them
	if(!sslStub(netIx).Flags.bKeysValid)
	{
		// Obtain two full buffers for the Sboxes
		SSLBufferAlloc(&sslStub(netIx).idTxBuffer);
		SSLBufferAlloc(&sslStub(netIx).idRxBuffer);
		if(sslStub(netIx).idTxBuffer == SSL_INVALID_ID || sslStub(netIx).idRxBuffer == SSL_INVALID_ID)
			return;

		// Generate the keys
		SSLHashSync(SSL_INVALID_ID);
		GenerateSessionKeys();
		sslStub(netIx).Flags.bKeysValid = 1;
	}
	
	// Reset the sequence counters
	sslKeys(netIx).Local.app.sequence = 0;	
	
	// Start the handshake data processor
	HSStart();

	// First, write the handshake header
	HSPut(hTCP, SSL_FINISHED);
	HSPut(hTCP, 0x00);
	HSPut(hTCP, 0x00);
	HSPut(hTCP, 0x24);

	// Calculate the Finished hashes
	CalculateFinishedHash(sslStub(netIx).idMD5, !sslStub(netIx).Flags.bIsServer, data);
	HSPutArray(hTCP, data, 16);
	CalculateFinishedHash(sslStub(netIx).idSHA1, !sslStub(netIx).Flags.bIsServer, data);
	HSPutArray(hTCP, data, 20);	

	// Hash this message to the handshake hash
	HSEnd();

	// Send the record
	SSLTxRecord(hTCP, sslStubID(netIx), SSL_HANDSHAKE);
	
	// Update the connection state
	TCPRequestSSLMessage(hTCP, SSL_NO_MESSAGE);
	sslStub(netIx).Flags.bLocalFinished = 1;
	
	// If complete, note that
	if(sslStub(netIx).Flags.bRemoteFinished)
	{
		TCPSSLHandshakeComplete(hTCP);
		SSLHashFree(&sslStub(netIx).idMD5);
		SSLHashFree(&sslStub(netIx).idSHA1);
	}

}

/*********************************************************************
 * Function:        void SSLRxCCS(TCP_SOCKET hTCP)
 *
 * PreCondition:    sslStub(netIx) is synchronized.
 *
 * Input:           hTCP - the TCP Socket to read from
 *
 * Output:          None
 *
 * Side Effects:    None
 *
 * Overview:        Receives a ChangeCipherSpec from the remote server
 *
 * Note:            None
 ********************************************************************/
static void SSLRxCCS(TCP_SOCKET hTCP)
{
#if SSL_MULTIPLE_INTERFACES
    int netIx = _TCPIPStackNetIx(sslSyncIf);
#endif  // SSL_MULTIPLE_INTERFACES
	// Only proceed if RSA is done
	if(sslStub(netIx).Flags.bRSAInProgress)
		return;
	
	// Verify handshake message sequence
	if(!sslStub(netIx).Flags.bClientHello || !sslStub(netIx).Flags.bServerHello)
		TCPRequestSSLMessage(hTCP, SSL_ALERT_HANDSHAKE_FAILURE);
	
	// Allocate a hash for MACing data
	SSLHashAlloc(&sslStub(netIx).idRxHash);

	// Make sure entire message is ready and an RX hash is allocated
	if(TCPIsGetReady(hTCP) < sslStub(netIx).wRxBytesRem 
		|| sslStub(netIx).idRxHash == SSL_INVALID_ID)
		return;
	
	// If keys are not ready, generate them
	if(!sslStub(netIx).Flags.bKeysValid)
	{
		// Sync up the session
		SSLSessionSync(sslStub(netIx).idSession);
		SSLKeysSync(sslStubID(netIx));
		SSLBufferSync(SSL_INVALID_ID);

		// Obtain two full buffers for the Sboxes
		SSLBufferAlloc(&sslStub(netIx).idTxBuffer);
		SSLBufferAlloc(&sslStub(netIx).idRxBuffer);
		if(sslStub(netIx).idTxBuffer == SSL_INVALID_ID || sslStub(netIx).idRxBuffer == SSL_INVALID_ID)
			return;

		// Generate the keys
		SSLHashSync(SSL_INVALID_ID);
		GenerateSessionKeys();
		sslStub(netIx).Flags.bKeysValid = 1;
	}
	
	// Read the CCS message (ignoring its contents)
	sslStub(netIx).wRxBytesRem -= TCPGetArray(hTCP, NULL, sslStub(netIx).wRxBytesRem);
	
	// Note that message was received
	SSLKeysSync(sslStubID(netIx));
	sslKeys(netIx).Remote.app.sequence = 0;
	sslStub(netIx).Flags.bRemoteChangeCipherSpec = 1;
}

/*********************************************************************
 * Function:        void SSLRxFinished(TCP_SOCKET hTCP)
 *
 * PreCondition:    sslStub(netIx) is synchronized and HSStart() has been
 *					called.
 *
 * Input:           hTCP - the TCP Socket to read from
 *
 * Output:          None
 *
 * Side Effects:    None
 *
 * Overview:        Receives the Finished message from remote node
 *
 * Note:            None
 ********************************************************************/
static void SSLRxFinished(TCP_SOCKET hTCP)
{
	uint8_t rxHash[20], expectedHash[20];
#if SSL_MULTIPLE_INTERFACES
    int netIx = _TCPIPStackNetIx(sslSyncIf);
#endif  // SSL_MULTIPLE_INTERFACES
	
	// Make sure entire message is ready
	if(TCPIsGetReady(hTCP) < sslStub(netIx).wRxHsBytesRem)
		return;
	
	// Verify handshake message sequence
	if(!sslStub(netIx).Flags.bRemoteChangeCipherSpec)
		TCPRequestSSLMessage(hTCP, SSL_ALERT_HANDSHAKE_FAILURE);
	
	// Make sure correct session and key set are loaded
	SSLSessionSync(sslStub(netIx).idSession);
	SSLKeysSync(sslStubID(netIx));
	
	// Read md5_sum to temporary location
	HSGetArray(hTCP, rxHash, 16);
	
	// Calculate expected MD5 hash
	CalculateFinishedHash(sslStub(netIx).idMD5, sslStub(netIx).Flags.bIsServer, expectedHash);	
	if(memcmp((void*)rxHash, (void*)expectedHash, 16) != 0)
	{// Handshake hash fails
		TCPRequestSSLMessage(hTCP, SSL_ALERT_HANDSHAKE_FAILURE);
	}
	
	// Read sha_sum to temporary location
	HSGetArray(hTCP, rxHash, 20);
	
	// Calculate expected SHA-1 hash	
	CalculateFinishedHash(sslStub(netIx).idSHA1, sslStub(netIx).Flags.bIsServer, expectedHash);
	if(memcmp((void*)rxHash, (void*)expectedHash, 20) != 0)
	{// Handshake hash fails
		TCPRequestSSLMessage(hTCP, SSL_ALERT_HANDSHAKE_FAILURE);
	}
	
	// Note that message was received
	sslStub(netIx).Flags.bRemoteFinished = 1;
	
	// If complete, note that, otherwise, request our own CCS message
	if(sslStub(netIx).Flags.bLocalFinished)
	{
		TCPSSLHandshakeComplete(hTCP);
		SSLHashFree(&sslStub(netIx).idMD5);
		SSLHashFree(&sslStub(netIx).idSHA1);
	}
	else
		TCPRequestSSLMessage(hTCP, SSL_CHANGE_CIPHER_SPEC);

}

/*********************************************************************
 * Function:        void SSLRxAlert(TCP_SOCKET hTCP)
 *
 * PreCondition:    sslStub(netIx) is synchronized
 *
 * Input:           hTCP - the TCP Socket to read from
 *
 * Output:          None
 *
 * Side Effects:    None
 *
 * Overview:        Receives an alert message and decides what to do
 *
 * Note:            None
 ********************************************************************/
static void SSLRxAlert(TCP_SOCKET hTCP)
{
	uint8_t bLevel, bDesc;
#if SSL_MULTIPLE_INTERFACES
    int netIx = _TCPIPStackNetIx(sslSyncIf);
#endif  // SSL_MULTIPLE_INTERFACES
	
	// Make sure entire message is ready
	if(TCPIsGetReady(hTCP) < sslStub(netIx).wRxBytesRem)
		return;
	
	// Read the alert message
	TCPGet(hTCP, &bLevel);
	TCPGet(hTCP, &bDesc);
	sslStub(netIx).wRxBytesRem -= 2;
	
	// Determine what to do
	switch(bLevel)
	{
		case SSL_ALERT_WARNING:
			// Check if a close notify was received
			if(bDesc + 0x80 == SSL_ALERT_CLOSE_NOTIFY)
				sslStub(netIx).Flags.bCloseNotify = 1;
			
			// We don't support non-fatal warnings besides CloseNotify,
			// so the connection is always done now.  When the TCP 
			// session closes, the resources will be cleaned up.
			
			// No break here:
			// Session is terminated, so still mark Done below
			
		case SSL_ALERT_FATAL:
			// Mark session as terminated
			sslStub(netIx).Flags.bDone = 1;			
	}
	
}

/****************************************************************************
  ===========================================================================
  Section:
	SSL Key Processing Functions
  ===========================================================================
  ***************************************************************************/

/*********************************************************************
 * Function:        RSA_STATUS SSLRSAOperation(void)
 *
 * PreCondition:    The RSA Module has been secured, an RSA operation
 *					is pending, sslStub(netIx).wRxHsBytesRem is the value of
 *					sslStub(netIx).wRxBytesRem after completion, and 
 *					sslStub(netIx).wRxBytesRem is the value of 
 *					sslStub(netIx).rxProtocol after completion.  Also requires
 *					sslStub(netIx) to be synchronized.
 *
 * Input:           None
 *
 * Output:          None
 *
 * Side Effects:    None
 *
 * Overview:        Pauses connection processing until RSA calculation
 *					is complete.
 *
 * Note:            This function exists outside of the handshaking 
 *					functions so that the system does not incur the
 *					expense of resuming and suspending handshake 
 *					hashes.
 ********************************************************************/
RSA_STATUS SSLRSAOperation(void)
{
#if SSL_MULTIPLE_INTERFACES
    int netIx = _TCPIPStackNetIx(sslSyncIf);
#endif  // SSL_MULTIPLE_INTERFACES

	SSLBufferSync(sslStub(netIx).idRxBuffer);
		
	// Call RSAStep to perform some RSA processing
	return RSAStep(sslSyncIf);
}

/*********************************************************************
 * Function:        void GenerateHashRounds(uint8_t num, uint8_t* rand1,
 *												uint8_t* rand2)
 *
 * PreCondition:    The SSL buffer is allocated for temporary usage
 *					and the data to run rounds on is in
 *					sslSession(netIx).masterSecret
 *
 * Input:           num   - how many rounds to compute
 *					rand1 - the first random data block to use
 *					rand2 - the second random data block to use
 *
 * Output:          None
 *
 * Side Effects:    Destroys the SSL Buffer space
 *
 * Overview:        Generates hash rounds to find either the
 *					Master Secret or the Key Block.
 *
 * Note:            This function will overflow the buffer after 7
 *					rounds, but in practice num = 3 or num = 4.
 ********************************************************************/
void GenerateHashRounds(uint8_t num, uint8_t* rand1, uint8_t* rand2)
{
	uint8_t i, j, c, *res;
    int netIx;

    netIx = _TCPIPStackNetIx(sslSyncIf);
	
	c = 'A';
	res = sslBuffer(netIx).hashRounds.temp;
	
	for(i = 1; i <= num; i++, c++, res += 16)
	{
		SHA1Initialize(&sslBuffer(netIx).hashRounds.hash);
		for(j = 0; j < i; j++)
			HashAddData(&sslBuffer(netIx).hashRounds.hash, &c, 1);
		HashAddData(&sslBuffer(netIx).hashRounds.hash, sslSession(netIx).masterSecret, 48);
		HashAddData(&sslBuffer(netIx).hashRounds.hash, rand1, 32);
		HashAddData(&sslBuffer(netIx).hashRounds.hash, rand2, 32);
		SHA1Calculate(&sslBuffer(netIx).hashRounds.hash, sslBuffer(netIx).hashRounds.sha_hash);
		MD5Initialize(&sslBuffer(netIx).hashRounds.hash);
		HashAddData(&sslBuffer(netIx).hashRounds.hash, sslSession(netIx).masterSecret, 48);
		HashAddData(&sslBuffer(netIx).hashRounds.hash, sslBuffer(netIx).hashRounds.sha_hash, 20);
		MD5Calculate(&sslBuffer(netIx).hashRounds.hash, res);
	}	
}

/*********************************************************************
 * Function:        void GenerateSessionKeys(void)
 *
 * PreCondition:    The SSL buffer is allocated for temporary usage,
 *					session keys are synced, and the TX and RX buffers
 *					are allocated for S-boxes.
 *
 * Input:           None
 *
 * Output:          None
 *
 * Side Effects:    Destroys the SSL Buffer Space
 *
 * Overview:        Generates the session write keys and MAC secrets
 *
 * Note:            None
 ********************************************************************/
void GenerateSessionKeys(void)
{
	// This functionality differs slightly for client and server operations
#if SSL_MULTIPLE_INTERFACES
    int netIx = _TCPIPStackNetIx(sslSyncIf);
#endif  // SSL_MULTIPLE_INTERFACES

	#if defined(TCPIP_STACK_USE_SSL_SERVER)
	if(sslStub(netIx).Flags.bIsServer)
	{
		// Generate the key expansion block
		GenerateHashRounds(4, sslKeys(netIx).Local.random, sslKeys(netIx).Remote.random);
		memcpy(sslKeys(netIx).Remote.app.MACSecret, (void*)sslBuffer(netIx).hashRounds.temp, 16);
		memcpy(sslKeys(netIx).Local.app.MACSecret, (void*)sslBuffer(netIx).hashRounds.temp+16, 16);
	
		// Save write keys elsewhere temporarily
		SSLHashSync(SSL_INVALID_ID);
		memcpy(&sslHash(netIx), (void*)sslBuffer(netIx).hashRounds.temp+32, 32);
	
		// Generate ARCFOUR Sboxes
		SSLBufferSync(sslStub(netIx).idRxBuffer);
		sslKeys(netIx).Remote.app.cryptCtx.Sbox = sslBuffer(netIx).full;
		ARCFOURInitialize(&(sslKeys(netIx).Remote.app.cryptCtx), (uint8_t*)(&sslHash(netIx)), 16);
		SSLBufferSync(sslStub(netIx).idTxBuffer);
		sslKeys(netIx).Local.app.cryptCtx.Sbox = sslBuffer(netIx).full;
		ARCFOURInitialize(&(sslKeys(netIx).Local.app.cryptCtx), (uint8_t*)(&sslHash(netIx))+16, 16);
		
		return;
	}
	#endif
	
	#if defined(TCPIP_STACK_USE_SSL_CLIENT)
	// Generate the key expansion block
	GenerateHashRounds(4, sslKeys(netIx).Remote.random, sslKeys(netIx).Local.random);
	memcpy(sslKeys(netIx).Local.app.MACSecret, (void*)sslBuffer(netIx).hashRounds.temp, 16);
	memcpy(sslKeys(netIx).Remote.app.MACSecret, (void*)sslBuffer(netIx).hashRounds.temp+16, 16);

	// Save write keys elsewhere temporarily
	SSLHashSync(SSL_INVALID_ID);
	memcpy(&sslHash(netIx), (void*)sslBuffer(netIx).hashRounds.temp+32, 32);

	// Generate ARCFOUR Sboxes
	SSLBufferSync(sslStub(netIx).idTxBuffer);
	sslKeys(netIx).Local.app.cryptCtx.Sbox = sslBuffer(netIx).full;
	ARCFOURInitialize(&(sslKeys(netIx).Local.app.cryptCtx), (uint8_t*)(&sslHash(netIx)), 16);
	SSLBufferSync(sslStub(netIx).idRxBuffer);
	sslKeys(netIx).Remote.app.cryptCtx.Sbox = sslBuffer(netIx).full;
	ARCFOURInitialize(&(sslKeys(netIx).Remote.app.cryptCtx), (uint8_t*)(&sslHash(netIx))+16, 16);
	#endif
	
}

/*********************************************************************
 * Function:        static void CalculateFinishedHash(uint8_t hashID,
 *									bool fromClient, uint8_t *result)
 *
 * PreCondition:    hashID has all handshake data hashed so far and
 *					the current session is synced in.
 *
 * Input:           hashID     - the hash sum to use
 *					fromClient - true if client is sender
 *					result     - where to store results
 *
 * Output:          None
 *
 * Side Effects:    None
 *
 * Overview:		Calculates the handshake hash over the data.
 *					hashID can be either MD5 or SHA-1, and this
 *					function will calculate accordingly.
 *
 * Note:            None
 ********************************************************************/
static void CalculateFinishedHash(uint8_t hashID, bool fromClient, uint8_t *result)
{
	uint8_t i;
#if SSL_MULTIPLE_INTERFACES
    int netIx = _TCPIPStackNetIx(sslSyncIf);
#endif  // SSL_MULTIPLE_INTERFACES

	// Load the hash, but make sure updates aren't saved
	SSLHashSync(hashID);
	sslHashID(netIx) = SSL_INVALID_ID;
	
	// Sync the session data so masterSecret is available
	SSLSessionSync(sslStub(netIx).idSession);
	
	// Hash in the sender phrase & master secret
	if(fromClient)
		HashAddData(&sslHash(netIx), (const uint8_t*)"CLNT", 4);
	else
		HashAddData(&sslHash(netIx), (const uint8_t*)"SRVR", 4);
	HashAddData(&sslHash(netIx), sslSession(netIx).masterSecret, 48);
	
	// Hash in the pad1
	i = 6;
	if(sslHash(netIx).hashType == HASH_SHA1)
		i--;
	for(; i > 0u; i--)
		HashAddData(&sslHash(netIx), (const uint8_t*)"\x36\x36\x36\x36\x36\x36\x36\x36", 8);
	
	// Calculate the inner hash result and store, start new hash
	if(sslHash(netIx).hashType == HASH_MD5)
	{
		MD5Calculate(&sslHash(netIx), result);
		MD5Initialize(&sslHash(netIx));
	}
	else
	{
		SHA1Calculate(&sslHash(netIx), result);
		SHA1Initialize(&sslHash(netIx));
	}
	
	// Hash in master secret
	HashAddData(&sslHash(netIx), sslSession(netIx).masterSecret, 48);
	
	// Hash in pad2
	i = 6;
	if(sslHash(netIx).hashType == HASH_SHA1)
		i--;
	for(; i > 0u; i--)
		HashAddData(&sslHash(netIx), (const uint8_t*)"\x5c\x5c\x5c\x5c\x5c\x5c\x5c\x5c", 8);
	
	// Hash in the inner hash result and calculate
	if(sslHash(netIx).hashType == HASH_MD5)
	{
		HashAddData(&sslHash(netIx), result, 16);
		MD5Calculate(&sslHash(netIx), result);
	}
	else
	{
		HashAddData(&sslHash(netIx), result, 20);
		SHA1Calculate(&sslHash(netIx), result);
	}
	
}


/****************************************************************************
  ===========================================================================
  Section:
	SSL Memory Management Functions
  ===========================================================================
  ***************************************************************************/

#define Debug(a,b)	//do{SYS_MESSAGE("\r\n" a); while(BusyUART()); WriteUART(b+'0');} while(0)


/*********************************************************************
 * Function:        static void SSLStubAlloc(NET_CONFIG* currIf)
 *
 * PreCondition:    None
 *
 * Inputs:          currIf    - interface for this SSL connection
 *
 * Outputs:         None
 *
 * Returns:			true if stub was allocated, false otherwise
 *
 * Side Effects:    None
 *
 * Overview:        Allocates a stub for use.
 *
 * Note:            None
 ********************************************************************/
static bool SSLStubAlloc(NET_CONFIG* currIf)
{
    TCPIP_MAC_HANDLE    oldMac;   // corresponding MAC
	uint8_t i;
#if SSL_MULTIPLE_INTERFACES
    int                 currIx, oldIx;  // corresponding index
    currIx = _TCPIPStackNetIx(currIf);
#endif  // SSL_MULTIPLE_INTERFACES

    
	// Search for a free stub
	for(i = 0; i != SSL_MAX_CONNECTIONS; i++)
	{
		if(!(isStubUsed(currIx) & masks[i]))
		{// Stub is free, so claim it
			isStubUsed(currIx) |= masks[i];
			
			// Save stub currently in RAM
            if(sslSyncIf != 0)
            {   // old interface valid
            #if SSL_MULTIPLE_INTERFACES
                oldIx = _TCPIPStackNetIx(sslSyncIf);
            #endif  // SSL_MULTIPLE_INTERFACES
                if(sslStubID(oldIx) != SSL_INVALID_ID)
                {
                    oldMac = _TCPIPStackNetToMac(sslSyncIf);
                    SaveOffChip(oldMac, (uint8_t*)&sslStub(oldIx),
					    SSL_BASE_STUB_ADDR(oldMac)+SSL_STUB_SIZE*sslStubID(oldIx),
					    SSL_STUB_SIZE);
                }
            }
            
			// Switch to new stub and return
			Debug("S",i);
			sslStubID(currIx) = i;
            sslSyncIf = currIf;
			return true;
		}
	}
	
	// No stub was found to be free
	return false;
	
}

/*********************************************************************
 * Function:        static void SSLStubFree(uint8_t id)
 *
 * PreCondition:    None
 *
 * Inputs:          id      - the stub ID to free
 *
 * Outputs:         None
 *
 * Returns:			None
 *
 * Side Effects:    None
 *
 * Overview:        Specified stub is released
 *
 * Note:            Interface should be sync-ed!
 ********************************************************************/
static void SSLStubFree(uint8_t id)
{
	// If ID is not valid
	if(id >= SSL_MAX_CONNECTIONS)
		return;
	
#if SSL_MULTIPLE_INTERFACES
    int netIx = _TCPIPStackNetIx(sslSyncIf);
#endif  // SSL_MULTIPLE_INTERFACES
	// If currently in RAM, mark as unused
	if(sslStubID(netIx) == id)
		sslStubID(netIx) = SSL_INVALID_ID;

	// Release the stub
	Debug("s",id);
	isStubUsed(netIx) &= ~masks[id];
}

/*********************************************************************
 * Function:        static void SSLStubSync(NET_CONFIG* currIf, uint8_t id)
 *
 * PreCondition:    None
 *
 * Input:           currIf - interface for this SSL connection 
 *                  id     - the stub ID to sync to RAM
 *
 * Output:          None
 *
 * Side Effects:    None
 *
 * Overview:        Specified stub is loaded to RAM.  Only loads if
 *					necessary, and saves any current stub before
 *					switching.
 *
 * Note:            None
 ********************************************************************/
static void SSLStubSync(NET_CONFIG* currIf, uint8_t id)
{
    TCPIP_MAC_HANDLE    hMac;   // corresponding MAC
#if SSL_MULTIPLE_INTERFACES
    int                 currIx, oldIx;  // corresponding index
    currIx = _TCPIPStackNetIx(currIf);
#endif  // SSL_MULTIPLE_INTERFACES
    
	// Check if already loaded
	if(sslSyncIf == currIf && sslStubID(currIx) == id)
    {
		return;     // everything already in sync
    }
    
	// Save old stub
	if(sslSyncIf != 0)
    {   // old interface valid
    #if SSL_MULTIPLE_INTERFACES
        oldIx = _TCPIPStackNetIx(sslSyncIf);
    #endif  // SSL_MULTIPLE_INTERFACES
        if(sslStubID(oldIx) != SSL_INVALID_ID)
        {   // old id valid
            hMac = _TCPIPStackNetToMac(sslSyncIf);   // old MAC
		    SaveOffChip(hMac, (uint8_t*)&sslStub(oldIx),
			    SSL_BASE_STUB_ADDR(hMac)+SSL_STUB_SIZE*sslStubID(oldIx),
			    SSL_STUB_SIZE);
        }
    }
	
	// Load new stub
    hMac = _TCPIPStackNetToMac(currIf);   // new MAC
	LoadOffChip(hMac, (uint8_t*)&sslStub(currIx),
		SSL_BASE_STUB_ADDR(hMac)+SSL_STUB_SIZE*id,
		SSL_STUB_SIZE);
	sslStubID(currIx) = id;
    sslSyncIf = currIf;
}

/*********************************************************************
 * Function:        static void SSLKeysSync(uint8_t id)
 *
 * PreCondition:    None
 *
 * Input:           id - the key set ID to sync to RAM
 *
 * Output:          None
 *
 * Side Effects:    None
 *
 * Overview:        Specified key set is loaded to RAM.  Only loads if
 *					necessary, and saves any current key set before
 *					switching.
 *
 * Note:            Interface should be sync-ed!
 ********************************************************************/
static void SSLKeysSync(uint8_t id)
{
    TCPIP_MAC_HANDLE    hMac;   // interface MAC
    
#if SSL_MULTIPLE_INTERFACES
    int netIx = _TCPIPStackNetIx(sslSyncIf);
#endif  // SSL_MULTIPLE_INTERFACES


	// Check if already loaded
	if(sslKeysID(netIx) == id)
		return;
	
    hMac = _TCPIPStackNetToMac(sslSyncIf);
    
	// Save old stub
	if(sslKeysID(netIx) != SSL_INVALID_ID)
		SaveOffChip(hMac, (uint8_t*)&sslKeys(netIx),
			SSL_BASE_KEYS_ADDR(hMac)+SSL_KEYS_SIZE*sslKeysID(netIx),
			SSL_KEYS_SIZE);
	
	// Load new stub
	LoadOffChip(hMac, (uint8_t*)&sslKeys(netIx),
		SSL_BASE_KEYS_ADDR(hMac)+SSL_KEYS_SIZE*id,
		SSL_KEYS_SIZE);
	sslKeysID(netIx) = id;
}

/*********************************************************************
 * Function:        static void SSLHashAlloc(uint8_t *id)
 *
 * PreCondition:    None
 *
 * Inputs:          id - Where to store the allocated ID
 *
 * Outputs:         id - Allocated hash ID, or SSL_INVALID_ID if 
 *							none available
 *
 * Side Effects:    None
 *
 * Overview:        Allocates a hash for use.
 *
 * Note:            Interface should be sync-ed!
 ********************************************************************/
static void SSLHashAlloc(uint8_t *id)
{
    TCPIP_MAC_HANDLE    hMac;   // interface MAC
	uint8_t i;
	
	// If already allocated, just load it up
	if(*id != SSL_INVALID_ID)
	{
		SSLHashSync(*id);
		return;
	}
		
#if SSL_MULTIPLE_INTERFACES
    int netIx = _TCPIPStackNetIx(sslSyncIf);
#endif  // SSL_MULTIPLE_INTERFACES
    hMac = _TCPIPStackNetToMac(sslSyncIf);

	// Search for a free hash
	for(i = 0; i != SSL_MAX_HASHES; i++)
	{
		if(!(isHashUsed(netIx) & masks[i]))
		{// Hash is free, so claim it
			isHashUsed(netIx) |= masks[i];
			
			// Save hash currently in RAM
			if(sslHashID(netIx) != SSL_INVALID_ID)
				SaveOffChip(hMac, (uint8_t*)&sslHash(netIx),
					SSL_BASE_HASH_ADDR(hMac)+SSL_HASH_SIZE*sslHashID(netIx),
					SSL_HASH_SIZE);

			// Switch to new hash and return
			Debug("H",i);
			sslHashID(netIx) = i;
			*id = i;
			return;
		}
	}
}

/*********************************************************************
 * Function:        static void SSLHashFree(uint8_t *id)
 *
 * PreCondition:    None
 *
 * Inputs:           id - the hash ID to free
 *
 * Outputs:          id - SSL_INVALID_ID
 *
 * Side Effects:    None
 *
 * Overview:        Specified hash is released
 *
 * Note:            None
 ********************************************************************/
static void SSLHashFree(uint8_t *id)
{
  	// Nothing to do for invalid hashes
	if(*id > SSL_MAX_HASHES)
		return;
	
#if SSL_MULTIPLE_INTERFACES
    int netIx = _TCPIPStackNetIx(sslSyncIf);
#endif  // SSL_MULTIPLE_INTERFACES
	// Purge from RAM if not used
	if(sslHashID(netIx) == *id)
		sslHashID(netIx) = SSL_INVALID_ID;

	// Release the hash
	Debug("h",*id);
	isHashUsed(netIx) &= ~masks[*id];
	*id = SSL_INVALID_ID;
}

/*********************************************************************
 * Function:        static void SSLHashSync(uint8_t id)
 *
 * PreCondition:    None
 *
 * Input:           id - the hash ID to sync to RAM
 *
 * Output:          None
 *
 * Side Effects:    None
 *
 * Overview:        Specified hash is loaded to RAM.  Only loads if
 *					necessary, and saves any current hash before
 *					switching.
 *
 * Note:            Interface should be sync-ed!
 ********************************************************************/
static void SSLHashSync(uint8_t id)
{
    TCPIP_MAC_HANDLE    hMac;   // interface MAC
#if SSL_MULTIPLE_INTERFACES
    int                 netIx = _TCPIPStackNetIx(sslSyncIf);
#endif  // SSL_MULTIPLE_INTERFACES
    
    // Check if already loaded
	if(sslHashID(netIx) == id)
		return;
	
    hMac = _TCPIPStackNetToMac(sslSyncIf);
    
	// Save old hash
	if(sslHashID(netIx) != SSL_INVALID_ID)
		SaveOffChip(hMac, (uint8_t*)&sslHash(netIx),
			SSL_BASE_HASH_ADDR(hMac)+SSL_HASH_SIZE*sslHashID(netIx),
			SSL_HASH_SIZE);
	
	// Load new hash if not requesting a temporary hash
	if(id != SSL_INVALID_ID)
		LoadOffChip(hMac, (uint8_t*)&sslHash(netIx),
			SSL_BASE_HASH_ADDR(hMac)+SSL_HASH_SIZE*id,
			SSL_HASH_SIZE);

	sslHashID(netIx) = id;
}

/*********************************************************************
 * Function:        static void SSLBufferAlloc(uint8_t *id)
 *
 * PreCondition:    None
 *
 * Input:           id - Where to store the allocated ID
 *
 * Output:          id - Allocated buffer ID, or SSL_INVALID_ID if 
 *							none available
 *
 * Side Effects:    None
 *
 * Overview:        Allocates a buffer for use.
 *
 * Note:            Interface should be sync-ed!
 ********************************************************************/
static void SSLBufferAlloc(uint8_t *id)
{
    TCPIP_MAC_HANDLE    hMac;   // interface MAC
	uint8_t i;

	// If already allocated, just load it up
	if(*id != SSL_INVALID_ID)
	{
		SSLBufferSync(*id);
		return;
	}
	
#if SSL_MULTIPLE_INTERFACES
    int netIx = _TCPIPStackNetIx(sslSyncIf);
#endif  // SSL_MULTIPLE_INTERFACES
	// Search for a free buffer
	for(i = 0; i != SSL_MAX_BUFFERS; i++)
	{
		if(!(isBufferUsed(netIx) & masks[i]))
		{// Buffer is free, so claim it
			isBufferUsed(netIx) |= masks[i];
			
			// Save buffer currently in RAM
			if(sslBufferID(netIx) != SSL_INVALID_ID)
            {
                hMac = _TCPIPStackNetToMac(sslSyncIf);
                SaveOffChip(hMac, (uint8_t*)&sslBuffer(netIx), 
                        SSL_BASE_BUFFER_ADDR(hMac)+SSL_BUFFER_SIZE*sslBufferID(netIx),
                        SSL_BUFFER_SIZE);
            }

			// Switch to new buffer and return
			Debug("B",i);
			sslBufferID(netIx) = i;
			*id = i;
			return;
		}
	}
}

/*********************************************************************
 * Function:        static void SSLBufferFree(uint8_t *id)
 *
 * PreCondition:    None
 *
 * Inputs:           id - the buffer ID to free
 *
 * Outputs:          id - SSL_INVALID_ID
 *
 * Side Effects:    None
 *
 * Overview:        Specified buffer is released
 *
 * Note:            None
 ********************************************************************/
static void SSLBufferFree(uint8_t *id)
{
	// Nothing to do for invalid hashes
	if(*id > SSL_MAX_BUFFERS)
		return;

#if SSL_MULTIPLE_INTERFACES
    int netIx = _TCPIPStackNetIx(sslSyncIf);
#endif  // SSL_MULTIPLE_INTERFACES
	// Purge from RAM if not used
	if(sslBufferID(netIx) == *id)
		sslBufferID(netIx) = SSL_INVALID_ID;\
		
	// Release the buffer
	Debug("b",*id);
	isBufferUsed(netIx) &= ~masks[*id];
	*id = SSL_INVALID_ID;
}

/*********************************************************************
 * Function:        static void SSLBufferSync(uint8_t id)
 *
 * PreCondition:    None
 *
 * Input:           id - the buffer ID to sync to RAM
 *
 * Output:          None
 *
 * Side Effects:    None
 *
 * Overview:        Specified buffer is loaded to RAM.  Only loads if
 *					necessary, and saves any current buffer before
 *					switching.
 *
 * Note:            Interface should be sync-ed!
 ********************************************************************/
static void SSLBufferSync(uint8_t id)
{
    TCPIP_MAC_HANDLE    hMac;   // interface MAC
    
#if SSL_MULTIPLE_INTERFACES
    int netIx = _TCPIPStackNetIx(sslSyncIf);
#endif  // SSL_MULTIPLE_INTERFACES
	// Check if already loaded
	if(sslBufferID(netIx) == id)
		return;
	
    hMac = _TCPIPStackNetToMac(sslSyncIf);
	// Save old buffer
	if(sslBufferID(netIx) != SSL_INVALID_ID)
        SaveOffChip(hMac, (uint8_t*)&sslBuffer(netIx),
                SSL_BASE_BUFFER_ADDR(hMac)+SSL_BUFFER_SIZE*sslBufferID(netIx),
                SSL_BUFFER_SIZE);

	// Load new buffer if not requesting temporary space
	if(id != SSL_INVALID_ID)
		LoadOffChip(hMac, (uint8_t*)&sslBuffer(netIx),
			SSL_BASE_BUFFER_ADDR(hMac)+SSL_BUFFER_SIZE*id,
			SSL_BUFFER_SIZE);

	sslBufferID(netIx) = id;
}

/*********************************************************************
 * Function:        static uint8_t SSLSessionNew(void)
 *
 * PreCondition:    None
 *
 * Input:           None
 *
 * Output:          Allocated Session ID, or SSL_INVALID_ID if none available
 *
 * Side Effects:    None
 *
 * Overview:        Finds space for a new SSL session
 *
 * Note:            Interface should be sync-ed!
 ********************************************************************/
static uint8_t SSLSessionNew(void)
{
	uint8_t id, oldestID;
	SYS_TICK now, age, oldest;
    TCPIP_MAC_HANDLE    hMac;   // interface MAC
#if SSL_MULTIPLE_INTERFACES
    int netIx = _TCPIPStackNetIx(sslSyncIf);
#endif  // SSL_MULTIPLE_INTERFACES
	
	// Set up the search
	oldestID = SSL_INVALID_ID;
	oldest = SSL_MIN_SESSION_LIFETIME * SYS_TICK_TicksPerSecondGet(); // convert to ticks
	now = SYS_TICK_Get();
		
	// Search for a free session
	for(id = 0; id != SSL_MAX_SESSIONS; id++)
	{
		if(sslSessionStubs(netIx)[id].tag.Val == 0u)
		{// Unused session, so claim immediately
			break;
		}
		
		// Check how old this session is
		age = now - sslSessionStubs(netIx)[id].lastUsed;
		if(age > oldest)
		{// This is now the oldest one
			oldest = age;
			oldestID = id;
		}
	}
	
	// Check if we can claim a session
	if(id == SSL_MAX_SESSIONS && oldestID != SSL_INVALID_ID)
		id = oldestID;
	
	// If a valid ID was found, claim it
	if(id < SSL_MAX_SESSIONS)
	{
		// Save old one if needed
		if(sslSessionUpdated(netIx))
        {
            hMac = _TCPIPStackNetToMac(sslSyncIf);
            SaveOffChip(hMac, (uint8_t*)&sslSession(netIx),
                    SSL_BASE_SESSION_ADDR(hMac)+SSL_SESSION_SIZE*sslSessionID(netIx),
                    SSL_SESSION_SIZE);
        }
		// Set up the new session
		sslSessionID(netIx) = id;
		sslSessionStubs(netIx)[id].lastUsed = now;
		SSLSessionUpdated(netIx);
		return id;
	}
	
	return SSL_INVALID_ID;
}

/*********************************************************************
 * Function:        static uint8_t SSLSessionMatchID(uint8_t* SessionID)
 *
 * PreCondition:    None
 *
 * Input:           SessionID - the session identifier to match
 *
 * Output:          The matched session ID, or SSL_INVALID_ID if not found
 *
 * Side Effects:    None
 *
 * Overview:        Locates a cached SSL session for reuse.  Syncs 
 *                  found session into RAM.
 *
 * Note:            None
 ********************************************************************/
#if defined(TCPIP_STACK_USE_SSL_SERVER)
static uint8_t SSLSessionMatchID(uint8_t* SessionID)
{
	uint8_t i;
#if SSL_MULTIPLE_INTERFACES
    int netIx = _TCPIPStackNetIx(sslSyncIf);
#endif  // SSL_MULTIPLE_INTERFACES
	
	for(i = 0; i < SSL_MAX_SESSIONS; i++)
	{
		// Check if tag matches the ID
		if(sslSessionStubs(netIx)[i].tag.v[0] == 0u &&
			!memcmp((void*)&sslSessionStubs(netIx)[i].tag.v[1], (void*)SessionID, 3) )
		{
			// Found a partial match, so load it to memory
			SSLSessionSync(i);
			
			// Verify complete match
			if(memcmp((void*)sslSession(netIx).sessionID, (void*)SessionID, 32) != 0)
				continue;
			
			// Mark it as being used now
			sslSessionStubs(netIx)[i].lastUsed = SYS_TICK_Get();
			
			// Return this session for use
			return i;
		}
	}
	
	return SSL_INVALID_ID;

}
#endif

/*********************************************************************
 * Function:        static uint8_t SSLSessionMatchHost(IP_ADDR ip)
 *
 * PreCondition:    None
 *
 * Input:           ip - the host session to match
 *
 * Output:          The matched session ID, or SSL_INVALID_ID if not found
 *
 * Side Effects:    None
 *
 * Overview:        Locates a cached SSL session for reuse
 *
 * Note:            None
 ********************************************************************/
#if defined(TCPIP_STACK_USE_SSL_CLIENT)
static uint8_t SSLSessionMatchIP(IP_ADDR ip)
{
	uint8_t i;
#if SSL_MULTIPLE_INTERFACES
    int netIx = _TCPIPStackNetIx(sslSyncIf);
#endif  // SSL_MULTIPLE_INTERFACES
               	
	for(i = 0; i < SSL_MAX_SESSIONS; i++)
	{
		// Check if tag matches the IP
		if(!memcmp((void*)&sslSessionStubs(netIx)[i].tag.v[0], (void*)&ip, 4))
		{
			// Found a match, so load it to memory
			SSLSessionSync(i);
			
			// Mark it as being used now
			sslSessionStubs(netIx)[i].lastUsed = SYS_TICK_Get();
			
			// Return this session for use
			return i;
		}
	}
	
	// No match so return invalid
	return SSL_INVALID_ID;
}
#endif

/*********************************************************************
 * Function:        static void SSLSessionSync(uint8_t id)
 *
 * PreCondition:    None
 *
 * Input:           id - the session ID to sync to RAM
 *
 * Output:          None
 *
 * Side Effects:    None
 *
 * Overview:        Specified session is loaded to RAM.  Only loads if
 *					necessary, and saves any current session before
 *					switching if it has been updated.
 *
 * Note:            Interface should be sync-ed!
 ********************************************************************/
static void SSLSessionSync(uint8_t id)
{
    TCPIP_MAC_HANDLE    hMac;   // interface MAC
#if SSL_MULTIPLE_INTERFACES
    int netIx = _TCPIPStackNetIx(sslSyncIf);
#endif  // SSL_MULTIPLE_INTERFACES
    
	// Check if already loaded
	if(sslSessionID(netIx) == id)
		return;
	
    hMac = _TCPIPStackNetToMac(sslSyncIf);
	// Save old buffer
	if(sslSessionUpdated(netIx) && sslSessionID(netIx) != SSL_INVALID_ID)
		SaveOffChip(hMac, (uint8_t*)&sslSession(netIx),
			SSL_BASE_SESSION_ADDR(hMac)+SSL_SESSION_SIZE*sslSessionID(netIx),
			SSL_SESSION_SIZE);
	
	// Load new buffer
	LoadOffChip(hMac, (uint8_t*)&sslSession(netIx),
		SSL_BASE_SESSION_ADDR(hMac)+SSL_SESSION_SIZE*id,
		SSL_SESSION_SIZE);

	sslSessionID(netIx) = id;
	sslSessionUpdated(netIx) = false;
}

/*********************************************************************
 * Function:        static void SaveOffChip(TCPIP_MAC_HANDLE hMac, uint8_t *ramAddr,
 *											uint16_t ethAddr, uint16_t len)
 *
 * PreCondition:    None
 *
 * Input:           hMac    - handle of the MAC interface
 *                  ramAddr - source address in RAM
 *					ethAddr - destination address in Ethernet RAM
 *					len		- number of bytes to copy
 *
 * Output:          None
 *
 * Side Effects:    None
 *
 * Overview:        Copies data in PIC RAM to the Ethernet RAM
 *
 * Note:            None
 ********************************************************************/
static void SaveOffChip(TCPIP_MAC_HANDLE hMac, uint8_t *ramAddr, PTR_BASE ethAddr, uint16_t len)
{
#if SSL_SAVE_CONTEXT_IN_PIC_RAM
    memcpy((uint8_t*)ethAddr, ramAddr, len);
#else
	PTR_BASE oldPtr;
	
	oldPtr = MACSetWritePtr(hMac, ethAddr);
	MACPutArray(hMac, ramAddr, len);
	MACSetWritePtr(hMac, oldPtr);
#endif  // SSL_SAVE_CONTEXT_IN_PIC_RAM
}

/*********************************************************************
 * Function:        static void LoadOffChip(TCPIP_MAC_HANDLE hMac, uint8_t *ramAddr,
 *											uint16_t ethAddr, uint16_t len)
 *
 * PreCondition:    None
 *
 * Input:           hMac    - handle of the MAC interface
 *                  ramAddr - destination address in RAM
 *					ethAddr - source address in Ethernet RAM
 *					len		- number of bytes to copy
 *
 * Output:          None
 *
 * Side Effects:    None
 *
 * Overview:        Copies data from Ethernet RAM to local RAM
 *
 * Note:            None
 ********************************************************************/
static void LoadOffChip(TCPIP_MAC_HANDLE hMac, uint8_t *ramAddr, PTR_BASE ethAddr, uint16_t len)
{
#if SSL_SAVE_CONTEXT_IN_PIC_RAM
    memcpy(ramAddr, (uint8_t*)ethAddr, len);
#else
    PTR_BASE oldPtr;
	
	oldPtr = MACSetReadPtr(hMac, ethAddr);
	MACGetArray(hMac, ramAddr, len);
	MACSetReadPtr(hMac, oldPtr);
#endif  // SSL_SAVE_CONTEXT_IN_PIC_RAM
}


/****************************************************************************
  ===========================================================================
  Section:
	SSL Handshake Hashing Functions
  ===========================================================================
  ***************************************************************************/

/*********************************************************************
 * Function:        static void HSStart()
 *
 * PreCondition:    None
 *
 * Input:           None
 *
 * Output:          None
 *
 * Side Effects:    None
 *
 * Overview:		Sets up the buffer to store data for handshake
 *					hash tracking
 *
 * Note:            Interface should be sync-ed!
 ********************************************************************/
static void HSStart()
{
#if SSL_MULTIPLE_INTERFACES
    int netIx = _TCPIPStackNetIx(sslSyncIf);
#endif  // SSL_MULTIPLE_INTERFACES
	// Allocate temporary storage and set the pointer to it
	SSLBufferSync(SSL_INVALID_ID);
	ptrHS(netIx) = sslBuffer(netIx).full;
}

/*********************************************************************
 * Function:        static void HSEnd()
 *
 * PreCondition:    None
 *
 * Input:           None
 *
 * Output:          None
 *
 * Side Effects:    None
 *
 * Overview:		Hashes the message contents into the Handshake
 *					hash structures and begins a new handshake hash.
 *
 * Note:            None
 ********************************************************************/
static void HSEnd()
{
	// Hash in the received data and reset the pointer
#if SSL_MULTIPLE_INTERFACES
    int netIx = _TCPIPStackNetIx(sslSyncIf);
#endif  // SSL_MULTIPLE_INTERFACES

	if(sslHashID(netIx) == sslStub(netIx).idMD5)
	{
		HashAddData(&sslHash(netIx), sslBuffer(netIx).full, ptrHS(netIx) - sslBuffer(netIx).full);
		SSLHashSync(sslStub(netIx).idSHA1);
		HashAddData(&sslHash(netIx), sslBuffer(netIx).full, ptrHS(netIx) - sslBuffer(netIx).full);
	}
	else
	{
		SSLHashSync(sslStub(netIx).idSHA1);
		HashAddData(&sslHash(netIx), sslBuffer(netIx).full, ptrHS(netIx) - sslBuffer(netIx).full);
		SSLHashSync(sslStub(netIx).idMD5);
		HashAddData(&sslHash(netIx), sslBuffer(netIx).full, ptrHS(netIx) - sslBuffer(netIx).full);
	}
	ptrHS(netIx) = sslBuffer(netIx).full;
}

/*********************************************************************
 * Function:        static uint16_t HSGet(TCP_SOCKET skt, uint8_t *b)
 *
 * PreCondition:    None
 *
 * Input:           skt - socket to read data from
 *					b	- byte to read into
 *
 * Output:          Number of bytes read
 *
 * Side Effects:    None
 *
 * Overview:        Reads data from socket, transparently hashing it
 *					into the handshake hashes.
 *
 * Note:            None
 ********************************************************************/
static uint16_t HSGet(TCP_SOCKET skt, uint8_t *b)
{
	uint8_t c;

	// Must read to &c in case b is NULL (we still need to hash it)
	if(!TCPGet(skt, &c))
		return 0;
	
#if SSL_MULTIPLE_INTERFACES
    int netIx = _TCPIPStackNetIx(sslSyncIf);
#endif  // SSL_MULTIPLE_INTERFACES

	*ptrHS(netIx) = c;
	if(b)
		*b = c;
	ptrHS(netIx)++;
	
	sslStub(netIx).wRxBytesRem--;
	sslStub(netIx).wRxHsBytesRem--;
	
	if(ptrHS(netIx) > sslBuffer(netIx).full + 128)
	{
		HSEnd();
		HSStart();
	}

	return 1;
}

/*********************************************************************
 * Function:        static uint16_t HSGetWord(TCP_SOCKET skt, uint16_t *w)
 *
 * PreCondition:    None
 *
 * Input:           skt - socket to read data from
 *					w	- word to read into
 *
 * Output:          Number of bytes read
 *
 * Side Effects:    None
 *
 * Overview:        Reads data from socket, transparently hashing it
 *					into the handshake hashes.
 *
 * Note:            None
 ********************************************************************/
static uint16_t HSGetWord(TCP_SOCKET skt, uint16_t *w)
{
	if(w == NULL)
		return HSGet(skt, (uint8_t*)w) + HSGet(skt, (uint8_t*)w);
	else
		return HSGet(skt, (uint8_t*)w+1) + HSGet(skt, (uint8_t*)w);
}

/*********************************************************************
 * Function:        static uint16_t HSGetArray(TCP_SOCKET skt, 
 											 uint8_t *data, uint16_t len)
 *
 * PreCondition:    None
 *
 * Input:           skt  - socket to read data from
 *					data - array to read into, or NULL
 *					len  - number of bytes to read
 *
 * Output:          Number of bytes read
 *
 * Side Effects:    None
 *
 * Overview:        Reads data from socket, transparently hashing it
 *					into the handshake hashes.
 *
 * Note:            None
 ********************************************************************/
static uint16_t HSGetArray(TCP_SOCKET skt, uint8_t *data, uint16_t len)
{	
	uint16_t w;
#if SSL_MULTIPLE_INTERFACES
    int netIx = _TCPIPStackNetIx(sslSyncIf);
#endif  // SSL_MULTIPLE_INTERFACES

/*	
	uint16_t wLenActual;
	uint16_t wChunkLen;

	wLenActual = 0;
	// Add all of this data to the running hash
	while(len)
	{
		wChunkLen = sizeof(sslBuffer(netIx).full) - (ptrHS(netIx) - sslBuffer(netIx).full);
		if(wChunkLen == 0u)
		{
			HSEnd();
			wChunkLen = sizeof(sslBuffer(netIx).full);
		}	
		if(len < wChunkLen)
			wChunkLen = len;
		
		// Obtain data from TCP
		w = TCPGetArray(skt, ptrHS(netIx), wChunkLen);
		if(w == 0u)
			return wLenActual;
		
		// Copy data from hash area to *data area if a non-NULL pointer was 
		// provided
		if(data)
		{
			memcpy((void*)data, ptrHS(netIx), w);
			data += w;
		}
		
		// Update all tracking variables
		ptrHS(netIx) += w;
		len -= w;
		wLenActual += w;
		sslStub(netIx).wRxBytesRem -= w;
		sslStub(netIx).wRxHsBytesRem -= w;
	}

	return wLenActual;
*/

	//if reading to NULL, we still have to hash
	if(!data)
	{
		uint16_t i, rem;
		for(i = 0; i < len; )
		{
			// Determine how much we can read
			rem = (sslBuffer(netIx).full + sizeof(sslBuffer(netIx).full) - 1) - ptrHS(netIx);
			if(rem > len - i)
				rem = len - i;

			// Read that much
			rem = TCPGetArray(skt, ptrHS(netIx), rem);
			sslStub(netIx).wRxBytesRem -= rem;
			sslStub(netIx).wRxHsBytesRem -= rem;
						
			// Hash what we've got
			ptrHS(netIx) += rem;
			HSEnd();
			HSStart();
			
			i += rem;
			
			// Make sure we aren't in an infinite loop
			if(rem == 0u)
				break;
		}
		
		return i;
	}
	
	len = TCPGetArray(skt, data, len);
	w = len;
	
	memcpy(ptrHS(netIx), (void*)data, len);
	ptrHS(netIx) += len;

	
//	// Add all of this data to the running hash
//	while(w)
//	{
//		uint16_t wChunkLen = sizeof(sslBuffer(netIx).full) - (ptrHS(netIx) - sslBuffer(netIx).full);
//		if(wChunkLen == 0u)
//		{
//			HSEnd();
//			HSStart();
//			wChunkLen = sizeof(sslBuffer(netIx).full);
//		}	
//		if(w < wChunkLen)
//			wChunkLen = w;
//		memcpy(ptrHS(netIx), (void*)data, wChunkLen);
//		ptrHS(netIx) += wChunkLen;
//		w -= wChunkLen;
//	}
	
	sslStub(netIx).wRxBytesRem -= len;
	sslStub(netIx).wRxHsBytesRem -= len;
	
	if(ptrHS(netIx) > sslBuffer(netIx).full + (sizeof(sslBuffer(netIx).full)/2))
	{
		HSEnd();
		HSStart();
	}
	
	return len;
}

/*********************************************************************
 * Function:        static uint16_t HSPut(TCP_SOCKET skt, uint8_t b)
 *
 * PreCondition:    None
 *
 * Input:           skt  - socket to write data to
 *					b    - byte to write
 *
 * Output:          Number of bytes written
 *
 * Side Effects:    None
 *
 * Overview:        Writes data to socket, transparently hashing it
 *					into the handshake hashes.
 *
 * Note:            None
 ********************************************************************/
static uint16_t HSPut(TCP_SOCKET skt, uint8_t b)
{

	if(!TCPPut(skt, b))
		return 0;

#if SSL_MULTIPLE_INTERFACES
    int netIx = _TCPIPStackNetIx(sslSyncIf);
#endif  // SSL_MULTIPLE_INTERFACES
    
	// Ensure we don't overflow the Hash buffer	
	if(sizeof(sslBuffer(netIx).full) - (ptrHS(netIx) - sslBuffer(netIx).full) == 0u)
		HSEnd();
	
	// Add this byte of data to the running hash
	*ptrHS(netIx) = b;
	ptrHS(netIx)++;

	return 1;
}

/*********************************************************************
 * Function:        static uint16_t HSPutWord(TCP_SOCKET skt, uint16_t w)
 *
 * PreCondition:    None
 *
 * Input:           skt  - socket to write data to
 *					w    - word to write
 *
 * Output:          Number of bytes written
 *
 * Side Effects:    None
 *
 * Overview:        Writes data to socket, transparently hashing it
 *					into the handshake hashes.
 *
 * Note:            None
 ********************************************************************/
static uint16_t HSPutWord(TCP_SOCKET skt, uint16_t w)
{
	return HSPut(skt, (uint8_t)(w>>8)) + HSPut(skt, (uint8_t)w);
}

/*********************************************************************
 * Function:        static uint16_t HSPutArray(TCP_SOCKET skt,
 											uint8_t *data, uint8_t len)
 *
 * PreCondition:    None
 *
 * Input:           skt  - socket to write data to
 *					data - data to write
 *					len  - number of bytes to write
 *
 * Output:          Number of bytes written
 *
 * Side Effects:    None
 *
 * Overview:        Writes data to socket, transparently hashing it
 *					into the handshake hashes.
 *
 * Note:            None
 ********************************************************************/
static uint16_t HSPutArray(TCP_SOCKET skt, uint8_t *data, uint16_t len)
{	
	uint16_t w;
#if SSL_MULTIPLE_INTERFACES
    int netIx = _TCPIPStackNetIx(sslSyncIf);
#endif  // SSL_MULTIPLE_INTERFACES
	
	len = TCPPutArray(skt, data, len);
	w = len;

	// Add all of this data to the running hash
	while(w)
	{
		uint16_t wChunkLen = sizeof(sslBuffer(netIx).full) - (ptrHS(netIx) - sslBuffer(netIx).full);
		if(wChunkLen == 0u)
		{
			HSEnd();
			wChunkLen = sizeof(sslBuffer(netIx).full);
		}	
		if(w < wChunkLen)
			wChunkLen = w;
		memcpy(ptrHS(netIx), (void*)data, wChunkLen);
		data += wChunkLen;
		ptrHS(netIx) += wChunkLen;
		w -= wChunkLen;
	}
	
	return len;
}

#if defined(TCPIP_STACK_USE_SSL_SERVER)
static uint16_t HSPutROMArray(TCP_SOCKET skt, const uint8_t *data, uint16_t len)
{	
	uint16_t w;
#if SSL_MULTIPLE_INTERFACES
    int netIx = _TCPIPStackNetIx(sslSyncIf);
#endif  // SSL_MULTIPLE_INTERFACES
	
	len = TCPPutArray(skt, data, len);
	w = len;

	// Add all of this data to the running hash
	while(w)
	{
		uint16_t wChunkLen = sizeof(sslBuffer(netIx).full) - (ptrHS(netIx) - sslBuffer(netIx).full);
		if(wChunkLen == 0u)
		{
			HSEnd();
			wChunkLen = sizeof(sslBuffer(netIx).full);
		}	
		if(w < wChunkLen)
			wChunkLen = w;
		memcpy(ptrHS(netIx), (const void*)data, wChunkLen);
		data += wChunkLen;
		ptrHS(netIx) += wChunkLen;
		w -= wChunkLen;
	}

	return len;
}
#endif

/****************************************************************************
  ===========================================================================
  Section:
	SSL MAC Hashing Functions
  ===========================================================================
  ***************************************************************************/

/*********************************************************************
 * Function:        static void SSLMACBegin(uint8_t *MACSecret, uint32_t seq, 
 *											uint8_t protocol, uint16_t len)
 *
 * PreCondition:    sslHash(netIx) is ready to be written
 *					(any pending data saved, nothing useful stored)
 *
 * Input:           MACSecret - the MAC write secret
 *					seq       - the sequence number for this message
 *					protocol  - the SSL_PROTOCOL for this message
 *					len       - the length of the message being MAC'd
 *
 * Output:          None
 *
 * Side Effects:    None
 *
 * Overview:		Begins a MAC calculation in sslHash(netIx)
 *
 * Note:            None
 ********************************************************************/
void SSLMACBegin(uint8_t *MACSecret, uint32_t seq, uint8_t protocol, uint16_t len)
{
	uint8_t i, temp[7];
#if SSL_MULTIPLE_INTERFACES
    int netIx = _TCPIPStackNetIx(sslSyncIf);
#endif  // SSL_MULTIPLE_INTERFACES
	MD5Initialize(&sslHash(netIx));

	// Form the temp array
	temp[0] = *((uint8_t*)&seq+3);
	temp[1] = *((uint8_t*)&seq+2);
	temp[2] = *((uint8_t*)&seq+1);
	temp[3] = *((uint8_t*)&seq+0);
	temp[4] = protocol;
	temp[5] = *((uint8_t*)&len+1);
	temp[6] = *((uint8_t*)&len+0);
		
	// Hash the initial data (secret, padding, seq, protcol, len)
	HashAddData(&sslHash(netIx), MACSecret, 16);
	
	// Add in the padding
	for(i = 0; i < 6u; i++)
	{
		HashAddData(&sslHash(netIx), (const uint8_t*)"\x36\x36\x36\x36\x36\x36\x36\x36", 8);
	}

	// Hash in the data length
	HashAddData(&sslHash(netIx), (const uint8_t*)"\0\0\0\0", 4);
	HashAddData(&sslHash(netIx), temp, 7);	
}

/*********************************************************************
 * Function:        void SSLMACAdd(uint8_t *data, uint16_t len)
 *
 * PreCondition:    sslHash(netIx) is ready to be written
 *					(any pending data saved, SSLMACBegin called)
 *
 * Input:           data - the data to add to the MAC
 *					len  - the length of data
 *
 * Output:          None
 *
 * Side Effects:    None
 *
 * Overview:		Adds more data to a MAC in progress
 *
 * Note:            None
 ********************************************************************/
void SSLMACAdd(uint8_t *data, uint16_t len)
{
#if SSL_MULTIPLE_INTERFACES
    int netIx = _TCPIPStackNetIx(sslSyncIf);
#endif  // SSL_MULTIPLE_INTERFACES
	HashAddData(&sslHash(netIx), data, len);
}

/*********************************************************************
 * Function:        static void SSLMACCalc(uint8_t *result)
 *
 * PreCondition:    sslHash(netIx) is ready to be written
 *					(any pending data saved, SSLMACBegin called)
 *
 * Input:           MACSecret - the MAC write secret
 *					result    - a 16 byte buffer to store result
 *
 * Output:          None
 *
 * Side Effects:    None
 *
 * Overview:		Finishes the MAC calculation, and places the
 *					result in the *result array
 *
 * Note:            None
 ********************************************************************/
void SSLMACCalc(uint8_t *MACSecret, uint8_t *result)
{
	uint8_t i;
#if SSL_MULTIPLE_INTERFACES
    int netIx = _TCPIPStackNetIx(sslSyncIf);
#endif  // SSL_MULTIPLE_INTERFACES
	
	// Get inner hash result
	MD5Calculate(&sslHash(netIx), result);
	
	// Begin outer hash
	MD5Initialize(&sslHash(netIx));
	HashAddData(&sslHash(netIx), MACSecret, 16);
	
	// Add in padding
	for(i = 0; i < 6u; i++)
	{
		HashAddData(&sslHash(netIx), (const uint8_t*)"\x5c\x5c\x5c\x5c\x5c\x5c\x5c\x5c", 8);
	}
	
	// Hash in the previous result and calculate
	HashAddData(&sslHash(netIx), result, 16);
	MD5Calculate(&sslHash(netIx), result);	
}

#endif
