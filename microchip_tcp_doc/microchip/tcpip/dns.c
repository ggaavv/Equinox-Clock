/*******************************************************************************
  Domain Name System (DNS) Client 
  Module for Microchip TCP/IP Stack

  Summary:
    DNS client implementation file
    
  Description:
    This source file contains the functions of the 
    DNS client routines
    
    Provides  hostname to IP address translation
    Reference: RFC 1035
*******************************************************************************/

/*******************************************************************************
FileName:   DNS.c
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

#if defined(TCPIP_STACK_USE_DNS)

/****************************************************************************
  Section:
	Constants and Global Variables
  ***************************************************************************/


static UDP_SOCKET       DNSSocket = INVALID_UDP_SOCKET;	// UDP socket to use for DNS queries
static const char*      DNSHostName;					// Host name in to look up
static uint8_t             RecordType;						// Record type being queried
static  union
{
    IP_ADDR ipv4Address;
    IPV6_ADDR ipv6Address;
} ResolvedAddress; 
static SYS_TICK		    stateStartTime;
static TCPIP_UINT16_VAL		    SentTransactionID;
static NET_CONFIG*      pDNSNet;                        // current interface the DNS is working on
static IP_ADDR          DNSServers[2];                  // servers associated with the current interface
static int              vDNSServerIx;                   // current server used

#if DNS_CLIENT_VERSION_NO >= 2
static int              dnsTickPending = 0;             // DNS processing tick
static SystemTickHandle dnsTimerHandle = 0;
#endif  // DNS_CLIENT_VERSION_NO >= 2

static int              dnsInitCount = 0;               // module initialization count

// Semaphore flags for the DNS module
static union
{
	uint8_t Val;
	struct
	{
		unsigned char DNSInUse 		: 1;	// Indicates the DNS module is in use
		unsigned char AddressValid	: 1;	// Indicates that the address resolution is valid and complete
		unsigned char AddressType 	: 2;    // IP_ADDRESS_TYPE_IPV6 or IP_ADDRESS_TYPE_IPV4
        unsigned char filler        : 4;
	} bits;
} Flags = {0x00};

// State machine for a DNS query
typedef enum
{
	DNS_IDLE = 0, 				// Initial state to reset client state variables
    // running, transient state
	DNS_START, 				    // resolution process started
	DNS_OPEN_SOCKET,			// Open UDP socket
	DNS_QUERY,					// Send DNS query to DNS server
	DNS_GET_RESULT,				// Wait for response from DNS server
	DNS_FAIL_ARP,				// ARP server not responding
	DNS_FAIL_SERVER,			// DNS server not responding
    // success state
	DNS_DONE,					// DNS query is finished OK
    // some error state
	DNS_FAIL_ARP_TMO,			// ARP resolution TMO
	DNS_FAIL_OPEN_TMO,			// Open Socket TMO
	DNS_FAIL_SERVER_TMO,		// DNS server TMO
}DNS_STATE;

static DNS_STATE smDNS = DNS_IDLE;

// Structure for the DNS header
typedef struct
{
	TCPIP_UINT16_VAL TransactionID;
	TCPIP_UINT16_VAL Flags;
	TCPIP_UINT16_VAL Questions;
	TCPIP_UINT16_VAL Answers;
	TCPIP_UINT16_VAL AuthoritativeRecords;
	TCPIP_UINT16_VAL AdditionalRecords;
} DNS_HEADER;

typedef struct
{
	// Response name is first, but it is variable length and must be retrieved using the DNSDiscardName() function
	TCPIP_UINT16_VAL	ResponseType;
	TCPIP_UINT16_VAL	ResponseClass;
	TCPIP_UINT32_VAL	ResponseTTL;
	TCPIP_UINT16_VAL	ResponseLen;
} DNS_ANSWER_HEADER;


/****************************************************************************
  Section:
	Function Prototypes
  ***************************************************************************/

static void DNSPutString(UDP_SOCKET s, const char* String);
static void DNSDiscardName(UDP_SOCKET s);
static DNS_STATE DNSRetry(DNS_STATE currState);
#if DNS_CLIENT_VERSION_NO >= 2
static void DNSTmoHandler(SYS_TICK currSysTick);
#endif  // DNS_CLIENT_VERSION_NO >= 2

/*static __inline__*/static  void /*__attribute__((always_inline))*/ _DNSReleaseSocket(void)
{
	if(DNSSocket != INVALID_UDP_SOCKET)
	{
		UDPClose(DNSSocket);
		DNSSocket = INVALID_UDP_SOCKET;
	}
}

/****************************************************************************
  Section:
	Implementation
  ***************************************************************************/


/*****************************************************************************
  Function:
	bool DNSClientInit(const TCPIP_STACK_MODULE_CTRL* const stackData,
                       const DNS_CLIENT_MODULE_GONFIG* const dnsData);

  Summary:
	Initializes the DNS module.
	
  Description:
	This function perform the initialization of the DNS client module.
    It has to be called before any other operation with the DNS client
    is possible.

  Precondition:
	Stack is initialized.

  Parameters:
    stackData - stack initialization data

    dnsData   - DNS client module specific initialization data    

  Return Values:
  	true      - the initialization was performed OK and the module is ready to be used
  	false     - The DNS module initialization failed.
  	
  Remarks:
	None
  ***************************************************************************/
bool DNSClientInit(const TCPIP_STACK_MODULE_CTRL* const stackData,
                       const DNS_CLIENT_MODULE_GONFIG* const dnsData)
{
    if(stackData->stackAction == TCPIP_STACK_ACTION_IF_UP)
    {   // interface restart
        return true;
    }

    // stack start up
    if(dnsInitCount != 0)
    {   // initialize just once
        dnsInitCount++;
        return true;
    }

    // 1st time init    
    smDNS = DNS_IDLE;
    Flags.Val = 0;
    DNSSocket = INVALID_UDP_SOCKET;

#if DNS_CLIENT_VERSION_NO >= 2
    if(dnsTimerHandle == 0)
    {   // once per service
        dnsTimerHandle = SYS_TICK_TimerCreate(DNSTmoHandler);
        if(dnsTimerHandle)
        {
            dnsTickPending = 0;
            SYS_TICK_TimerSetRate(dnsTimerHandle, (SYS_TICK_ResolutionGet() * DNS_CLIENT_TASK_PROCESS_RATE)/1000);
        }
        else
        {
            return false;
        }
    }
#endif  // DNS_CLIENT_VERSION_NO >= 2

    dnsInitCount++;
    return true;
}

/*****************************************************************************
  Function:
	void DNSClientDeInit(const TCPIP_STACK_MODULE_CTRL* const stackData);

  Summary:
	De-Initializes the DNS module.
	
  Description:
	This function perform the de-initialization of the DNS client module.
    It is used to release all the resources that are in use by the DNS client.
    
  Precondition:
	Stack is initialized.

  Parameters:
	stackData   - interface to use
                Normally should be a default DNS interface
                
  Return Values:
  	None
  	
  Remarks:
	None
  ***************************************************************************/
void DNSClientDeInit(const TCPIP_STACK_MODULE_CTRL* const stackData)
{
    // interface going down
    if(pDNSNet == stackData->pNetIf)
    {   // my interface is shut down
        DNSEndUsage(0);
    }
        
    if(stackData->stackAction == TCPIP_STACK_ACTION_DEINIT)
    {   // stack shut down
        if(dnsInitCount > 0)
        {   // we're up and running

            if(--dnsInitCount == 0)
            {   // all closed
                // release resources
#if DNS_CLIENT_VERSION_NO >= 2
                if(dnsTimerHandle)
                {
                    SYS_TICK_TimerDelete(dnsTimerHandle);
                    dnsTimerHandle = 0;
                    dnsTickPending = 0;
                }
#endif  // DNS_CLIENT_VERSION_NO >= 2
            }
        }
    }


}

/*****************************************************************************
  Function:
	DNS_RESULT DNSBeginUsage(NET_CONFIG* pConfig)

  Summary:
	Claims access to the DNS module.
	
  Description:
	This function acts as a semaphore to obtain usage of the DNS module.
	Call this function and ensure that it returns DNS_RES_OK before calling any
	other DNS APIs.  Call DNSEndUsage when this application no longer 
	needs the DNS module so that other applications may make use of it.

  Precondition:
	Stack is initialized.

  Parameters:
	pConfig   - interface to use
                If 0, a default interface will be selected

  Return Values:
  	DNS_RES_OK      - the calling application has sucessfully taken ownership of the DNS module
  	DNS_RES_BUSY    - The DNS module is currently in use.
                      Yield to the stack and attempt this call again later.
  	
  Remarks:
	Ensure that DNSEndUsage is always called once your application has
	obtained control of the DNS module.  If this is not done, the stack
	will hang for all future applications requiring DNS access.
  ***************************************************************************/
DNS_RESULT DNSBeginUsage(TCPIP_NET_HANDLE netH)
{
    NET_CONFIG* pNewIf;
    
	if(Flags.bits.DNSInUse)
		return DNS_RES_BUSY;

    pNewIf = _TCPIPStackHandleToNet(netH);
    
    if(pNewIf == 0 || !_TCPIPStackIsNetUp(pNewIf))
    {   // try a default interface
        if(_TCPIPStackIsNetUp(pDNSNet))
        {
            pNewIf = pDNSNet;
        }
        else
        {
            pNewIf = (NET_CONFIG*)TCPIP_STACK_GetDefaultNet();
            if(!_TCPIPStackIsNetUp(pNewIf))
            {
                pNewIf = 0;
            }
        }
    }
    // else pNewIf should do just fine

    if(pNewIf == 0)
    {
        return DNS_RES_NO_INTERFACE;
    }
    
    pDNSNet = pNewIf;
    
	Flags.bits.DNSInUse = true;
    DNSServers[0] = pDNSNet->PrimaryDNSServer;
    DNSServers[1] = pDNSNet->SecondaryDNSServer;
    
    smDNS = DNS_IDLE;

	return DNS_RES_OK;
}


/*****************************************************************************
  Function:
	DNS_RESULT DNSEndUsage(NET_CONFIG* pConfig)

  Summary:
	Releases control of the DNS module.
	
  Description:
	This function acts as a semaphore to release control of the DNS module.
	Call this function when this application no longer needs the DNS 
	module so that other applications may make use of it.

  Precondition:
	DNSBeginUsage returned DNS_RES_OK on a previous call.

  Parameters:
	pConfig   - interface to release
                Not used.

  Return Values:
  	DNS_RES_OK - The DNS module successfully released.
  	
  Remarks:
	Ensure that DNSEndUsage is always called once your application has
	obtained control of the DNS module.  If this is not done, the stack
	will hang for all future applications requiring DNS access.
  ***************************************************************************/
DNS_RESULT DNSEndUsage(TCPIP_NET_HANDLE netH)
{
    _DNSReleaseSocket();
    smDNS = DNS_IDLE;
	Flags.bits.DNSInUse = false;

    return DNS_RES_OK;
}


/*****************************************************************************
  Function:
	void DNSResolve(uint8_t* Hostname, DNS_RESOLVE_TYPE Type)

  Summary:
	Begins resolution of an address.
	
  Description:
	This function attempts to resolve a host name to an IP address.  When 
	called, it starts the DNS state machine.  Call DNSIsResolved repeatedly
	to determine if the resolution is complete.
	
	Only one DNS resoultion may be executed at a time.  The Hostname must 
	not be modified in memory until the resolution is complete.

  Precondition:
	DNSBeginUsage returned DNS_RES_OK on a previous call.

  Parameters:
	Hostname - A pointer to the null terminated string specifiying the
		host for which to resolve an IP.
	RecordType - DNS_TYPE_A or DNS_TYPE_MX depending on what type of
		record resolution is desired.

  Returns:
  	DNS_RES_OK
  	
  Remarks:
	This function requires access to one UDP socket.  If none are available,
	UDP_MAX_SOCKETS may need to be increased.

  ***************************************************************************/
DNS_RESULT DNSResolve(const char* Hostname, DNS_RESOLVE_TYPE Type)
{
	if(TCPIP_HELPER_StringToIPAddress(Hostname, &ResolvedAddress.ipv4Address))
	{
		Flags.bits.AddressValid = true;
		smDNS = DNS_DONE;
        Flags.bits.AddressType = IP_ADDRESS_TYPE_IPV4;
	}
#if defined (TCPIP_STACK_USE_IPV6)
    else if (TCPIP_HELPER_StringToIPv6Address ((uint8_t *)Hostname, &ResolvedAddress.ipv6Address))
    {
        Flags.bits.AddressValid = true;
        Flags.bits.AddressType = IP_ADDRESS_TYPE_IPV6;
        smDNS = DNS_DONE;
    }
#endif
	else
	{	
		smDNS = DNS_START;
		RecordType = (uint8_t)Type;
		Flags.bits.AddressValid = false;
    }

    DNSHostName = Hostname;
    return DNS_RES_OK;
}


/*****************************************************************************
  Function:
	DNS_RESULT DNSIsResolved(const char* HostName, IP_ADDR* HostIP)

  Summary:
	Determines if the DNS resolution is complete and provides the IP.
	
  Description:
	Call this function to determine if the DNS resolution of an address has
	been completed.  If so, the resolved address will be provided in HostIP.

  Precondition:
	DNSResolve has been called.

  Parameters:
	Hostname - A pointer to the null terminated string specifiying the
		host for which to resolve an IP.
	HostIP - A pointer to an IP_ADDR structure in which to store the 
		resolved IP address once resolution is complete.

  Return Values:
  	DNS_RES_OK       - The DNS client has obtained an IP
  		               HostIP will contain the resolved address.
  	DNS_RES_PENDING  - The resolution process is still in progress.
    DNS_RES_SERVER_TMO  - DNS server timed out
    DNS_RES_NO_ENTRY - no such entry to be resolved exists
    
    
  Remarks:
    
  ***************************************************************************/
DNS_RESULT DNSIsResolved(const char* HostName, void* HostIP)
{
    
    if(smDNS == DNS_IDLE || strcmp(DNSHostName, HostName)!= 0)
    {
        return DNS_RES_NO_ENTRY;
    }
    
    if (smDNS == DNS_DONE)
    {
#if defined (TCPIP_STACK_USE_IPV6)
        if (Flags.bits.AddressType == IP_ADDRESS_TYPE_IPV6)
        {
            memcpy (HostIP, &ResolvedAddress.ipv6Address, sizeof (IPV6_ADDR));
        }
        else
#endif
        {
			((IP_ADDR *)HostIP)->Val = ResolvedAddress.ipv4Address.Val;
        }

        return DNS_RES_OK;   
    }
    
    if(smDNS < DNS_DONE)
    {
        return DNS_RES_PENDING;
    }
    
    // some kind of error
    switch (smDNS)
    {
        case DNS_FAIL_ARP_TMO:
            return DNS_RES_ARP_TMO; 

        case DNS_FAIL_OPEN_TMO:
            return DNS_RES_OPEN_TMO; 

        default:    // DNS_FAIL_SERVER_TMO:
            return DNS_RES_SERVER_TMO; 
    }

}


/*****************************************************************************
  Function:
	void DNSClientTask(void)

  Summary:
	DNS client state machine
	
  Description:
    Process the DNS client state machine
  
  Precondition:
	DNSClientInit has been called.

  Parameters:
    None
    
  Return Values:
    None

  ***************************************************************************/
void DNSClientTask(void)
{
	uint8_t 				i;
	TCPIP_UINT16_VAL			w;
	DNS_HEADER			DNSHeader;
	DNS_ANSWER_HEADER	DNSAnswerHeader;
    

    switch(smDNS)
	{
		case DNS_IDLE:
            break;  // nothing to do

		case DNS_START:
            smDNS = DNSRetry(DNS_START);
            stateStartTime = 0;  // flag the first Open try
			break;

		case DNS_OPEN_SOCKET:
            DNSSocket = UDPOpenClient(IP_ADDRESS_TYPE_IPV4, DNS_CLIENT_PORT, (IP_MULTI_ADDRESS*)(DNSServers + vDNSServerIx));
			if(DNSSocket == INVALID_UDP_SOCKET)
            {
                if(stateStartTime == 0)
                {
                    stateStartTime = SYS_TICK_Get();
                }
                else if(SYS_TICK_Get() - stateStartTime > (DNS_CLIENT_OPEN_TMO * SYS_TICK_TicksPerSecondGet()))
                {
					smDNS = DNS_FAIL_OPEN_TMO;
                }
                
                break;
            }
            
            // got a valid UDP socket
            UDPSocketSetNet(DNSSocket, pDNSNet);
            stateStartTime = SYS_TICK_Get();
            smDNS = DNS_QUERY;
            // no break, start sending the query;

		case DNS_QUERY:
            if(!UDPIsOpened(DNSSocket) || (UDPIsTxPutReady(DNSSocket, 18 + strlen (DNSHostName)) < (18 + strlen (DNSHostName))))
            {
                if(SYS_TICK_Get() - stateStartTime > (DNS_CLIENT_OPEN_TMO * SYS_TICK_TicksPerSecondGet()))
                {
					smDNS = DNS_FAIL_OPEN_TMO;
                }
                
				break;  // wait some more
            }
			
			// Put DNS query here
			SentTransactionID.Val = (uint16_t)rand();
			UDPPut(DNSSocket, SentTransactionID.v[1]);// User chosen transaction ID
			UDPPut(DNSSocket, SentTransactionID.v[0]);
			UDPPut(DNSSocket, 0x01);		// Standard query with recursion
			UDPPut(DNSSocket, 0x00);	
			UDPPut(DNSSocket, 0x00);		// 0x0001 questions
			UDPPut(DNSSocket, 0x01);
			UDPPut(DNSSocket, 0x00);		// 0x0000 answers
			UDPPut(DNSSocket, 0x00);
			UDPPut(DNSSocket, 0x00);		// 0x0000 name server resource records
			UDPPut(DNSSocket, 0x00);
			UDPPut(DNSSocket, 0x00);		// 0x0000 additional records
			UDPPut(DNSSocket, 0x00);

			// Put hostname string to resolve
            DNSPutString(DNSSocket, DNSHostName);

			UDPPut(DNSSocket, 0x00);		// Type: DNS_TYPE_A A (host address) or DNS_TYPE_MX for mail exchange
			UDPPut(DNSSocket, RecordType);
			UDPPut(DNSSocket, 0x00);		// Class: IN (Internet)
			UDPPut(DNSSocket, 0x01);

			UDPFlush(DNSSocket);
			stateStartTime = SYS_TICK_Get();
			smDNS = DNS_GET_RESULT;
			break;

		case DNS_GET_RESULT:
			if(!UDPIsGetReady(DNSSocket))
			{
				if(SYS_TICK_Get() - stateStartTime > (DNS_CLIENT_SERVER_TMO * SYS_TICK_TicksPerSecondGet()))
                {
					smDNS = DNS_FAIL_SERVER;
                }
				break;
			}


			// Retrieve the DNS header and de-big-endian it
			UDPGet(DNSSocket, &DNSHeader.TransactionID.v[1]);
			UDPGet(DNSSocket, &DNSHeader.TransactionID.v[0]);

			// Throw this packet away if it isn't in response to our last query
			if(DNSHeader.TransactionID.Val != SentTransactionID.Val)
			{
				UDPDiscard(DNSSocket);
				break;
			}

			UDPGet(DNSSocket, &DNSHeader.Flags.v[1]);
			UDPGet(DNSSocket, &DNSHeader.Flags.v[0]);
			UDPGet(DNSSocket, &DNSHeader.Questions.v[1]);
			UDPGet(DNSSocket, &DNSHeader.Questions.v[0]);
			UDPGet(DNSSocket, &DNSHeader.Answers.v[1]);
			UDPGet(DNSSocket, &DNSHeader.Answers.v[0]);
			UDPGet(DNSSocket, &DNSHeader.AuthoritativeRecords.v[1]);
			UDPGet(DNSSocket, &DNSHeader.AuthoritativeRecords.v[0]);
			UDPGet(DNSSocket, &DNSHeader.AdditionalRecords.v[1]);
			UDPGet(DNSSocket, &DNSHeader.AdditionalRecords.v[0]);

			// Remove all questions (queries)
			while(DNSHeader.Questions.Val--)
			{
				DNSDiscardName(DNSSocket);
				UDPGet(DNSSocket, &w.v[1]);		// Question type
				UDPGet(DNSSocket, &w.v[0]);
				UDPGet(DNSSocket, &w.v[1]);		// Question class
				UDPGet(DNSSocket, &w.v[0]);
			}
			
			// Scan through answers
			while(DNSHeader.Answers.Val--)
			{				
				DNSDiscardName(DNSSocket);					// Throw away response name
				UDPGet(DNSSocket, &DNSAnswerHeader.ResponseType.v[1]);		// Response type
				UDPGet(DNSSocket, &DNSAnswerHeader.ResponseType.v[0]);
				UDPGet(DNSSocket, &DNSAnswerHeader.ResponseClass.v[1]);	// Response class
				UDPGet(DNSSocket, &DNSAnswerHeader.ResponseClass.v[0]);
				UDPGet(DNSSocket, &DNSAnswerHeader.ResponseTTL.v[3]);		// Time to live
				UDPGet(DNSSocket, &DNSAnswerHeader.ResponseTTL.v[2]);
				UDPGet(DNSSocket, &DNSAnswerHeader.ResponseTTL.v[1]);
				UDPGet(DNSSocket, &DNSAnswerHeader.ResponseTTL.v[0]);
				UDPGet(DNSSocket, &DNSAnswerHeader.ResponseLen.v[1]);		// Response length
				UDPGet(DNSSocket, &DNSAnswerHeader.ResponseLen.v[0]);

				// Make sure that this is a 4 byte IP address, response type A or MX, class 1
				// Check if this is Type A, MX, or AAAA
				if( DNSAnswerHeader.ResponseClass.Val	== 0x0001u) // Internet class
				{
                    if (DNSAnswerHeader.ResponseType.Val	== 0x0001u &&
    					DNSAnswerHeader.ResponseLen.Val		== 0x0004u)
                    {
    					Flags.bits.AddressValid = true;
                        Flags.bits.AddressType = IP_ADDRESS_TYPE_IPV4;
    					UDPGet(DNSSocket, &ResolvedAddress.ipv4Address.v[0]);
    					UDPGet(DNSSocket, &ResolvedAddress.ipv4Address.v[1]);
    					UDPGet(DNSSocket, &ResolvedAddress.ipv4Address.v[2]);
    					UDPGet(DNSSocket, &ResolvedAddress.ipv4Address.v[3]);
    					goto DoneSearchingRecords;
                    }
                    else if (DNSAnswerHeader.ResponseType.Val == 0x001Cu &&
    					        DNSAnswerHeader.ResponseLen.Val	== 0x0010u)
                    {
                        if (RecordType != DNS_TYPE_AAAA)
                        {
        					while(DNSAnswerHeader.ResponseLen.Val--)
        					{
        						UDPGet(DNSSocket, &i);
        					}
                            break;
                        }
    					Flags.bits.AddressValid = true;
                        Flags.bits.AddressType = IP_ADDRESS_TYPE_IPV6;
                        UDPGetArray (DNSSocket, (void *)&ResolvedAddress.ipv6Address, sizeof (IPV6_ADDR));
    					goto DoneSearchingRecords;                        
                    }
                    else
                    {
    					while(DNSAnswerHeader.ResponseLen.Val--)
    					{
    						UDPGet(DNSSocket, &i);
    					}
                    }
				}
				else
				{
					while(DNSAnswerHeader.ResponseLen.Val--)
					{
						UDPGet(DNSSocket, &i);
					}
				}
			}

			// Remove all Authoritative Records
			while(DNSHeader.AuthoritativeRecords.Val--)
			{
				DNSDiscardName(DNSSocket);					// Throw away response name
				UDPGet(DNSSocket, &DNSAnswerHeader.ResponseType.v[1]);		// Response type
				UDPGet(DNSSocket, &DNSAnswerHeader.ResponseType.v[0]);
				UDPGet(DNSSocket, &DNSAnswerHeader.ResponseClass.v[1]);	// Response class
				UDPGet(DNSSocket, &DNSAnswerHeader.ResponseClass.v[0]);
				UDPGet(DNSSocket, &DNSAnswerHeader.ResponseTTL.v[3]);		// Time to live
				UDPGet(DNSSocket, &DNSAnswerHeader.ResponseTTL.v[2]);
				UDPGet(DNSSocket, &DNSAnswerHeader.ResponseTTL.v[1]);
				UDPGet(DNSSocket, &DNSAnswerHeader.ResponseTTL.v[0]);
				UDPGet(DNSSocket, &DNSAnswerHeader.ResponseLen.v[1]);		// Response length
				UDPGet(DNSSocket, &DNSAnswerHeader.ResponseLen.v[0]);

				// Make sure that this is a 4 byte IP address, response type A or MX, class 1
				// Check if this is Type A
				if( DNSAnswerHeader.ResponseClass.Val	== 0x0001u) // Internet class
				{
                    if (DNSAnswerHeader.ResponseType.Val	== 0x0001u &&
    					DNSAnswerHeader.ResponseLen.Val		== 0x0004u)
                    {
    					Flags.bits.AddressValid = true;
                        Flags.bits.AddressType = IP_ADDRESS_TYPE_IPV4;
    					UDPGet(DNSSocket, &ResolvedAddress.ipv4Address.v[0]);
    					UDPGet(DNSSocket, &ResolvedAddress.ipv4Address.v[1]);
    					UDPGet(DNSSocket, &ResolvedAddress.ipv4Address.v[2]);
    					UDPGet(DNSSocket, &ResolvedAddress.ipv4Address.v[3]);
    					goto DoneSearchingRecords;
                    }
                    else if (DNSAnswerHeader.ResponseType.Val == 0x001Cu &&
    					        DNSAnswerHeader.ResponseLen.Val	== 0x0010u)
                    {
                        if (RecordType != DNS_TYPE_AAAA)
                        {
        					while(DNSAnswerHeader.ResponseLen.Val--)
        					{
        						UDPGet(DNSSocket, &i);
        					}
                            break;
                        }
    					Flags.bits.AddressValid = true;
                        Flags.bits.AddressType = IP_ADDRESS_TYPE_IPV6;
                        UDPGetArray (DNSSocket, (void *)&ResolvedAddress.ipv6Address, sizeof (IPV6_ADDR));
    					goto DoneSearchingRecords;                        
                    }
                    else
                    {
    					while(DNSAnswerHeader.ResponseLen.Val--)
    					{
    						UDPGet(DNSSocket, &i);
    					}
                    }
				}
				else
				{
					while(DNSAnswerHeader.ResponseLen.Val--)
					{
						UDPGet(DNSSocket, &i);
					}
				}
			}

			// Remove all Additional Records
			while(DNSHeader.AdditionalRecords.Val--)
			{
				DNSDiscardName(DNSSocket);					// Throw away response name
				UDPGet(DNSSocket, &DNSAnswerHeader.ResponseType.v[1]);		// Response type
				UDPGet(DNSSocket, &DNSAnswerHeader.ResponseType.v[0]);
				UDPGet(DNSSocket, &DNSAnswerHeader.ResponseClass.v[1]);	// Response class
				UDPGet(DNSSocket, &DNSAnswerHeader.ResponseClass.v[0]);
				UDPGet(DNSSocket, &DNSAnswerHeader.ResponseTTL.v[3]);		// Time to live
				UDPGet(DNSSocket, &DNSAnswerHeader.ResponseTTL.v[2]);
				UDPGet(DNSSocket, &DNSAnswerHeader.ResponseTTL.v[1]);
				UDPGet(DNSSocket, &DNSAnswerHeader.ResponseTTL.v[0]);
				UDPGet(DNSSocket, &DNSAnswerHeader.ResponseLen.v[1]);		// Response length
				UDPGet(DNSSocket, &DNSAnswerHeader.ResponseLen.v[0]);

				// Make sure that this is a 4 byte IP address, response type A or MX, class 1
				// Check if this is Type A
				if( DNSAnswerHeader.ResponseClass.Val	== 0x0001u) // Internet class
				{
                    if (DNSAnswerHeader.ResponseType.Val	== 0x0001u &&
    					DNSAnswerHeader.ResponseLen.Val		== 0x0004u)
                    {
    					Flags.bits.AddressValid = true;
                        Flags.bits.AddressType = IP_ADDRESS_TYPE_IPV4;
    					UDPGet(DNSSocket, &ResolvedAddress.ipv4Address.v[0]);
    					UDPGet(DNSSocket, &ResolvedAddress.ipv4Address.v[1]);
    					UDPGet(DNSSocket, &ResolvedAddress.ipv4Address.v[2]);
    					UDPGet(DNSSocket, &ResolvedAddress.ipv4Address.v[3]);
    					goto DoneSearchingRecords;
                    }
                    else if (DNSAnswerHeader.ResponseType.Val == 0x001Cu &&
    					        DNSAnswerHeader.ResponseLen.Val	== 0x0010u)
                    {
                        if (RecordType != DNS_TYPE_AAAA)
                        {
        					while(DNSAnswerHeader.ResponseLen.Val--)
        					{
        						UDPGet(DNSSocket, &i);
        					}
                            break;
                        }
    					Flags.bits.AddressValid = true;
                        Flags.bits.AddressType = IP_ADDRESS_TYPE_IPV6;
                        UDPGetArray (DNSSocket, (void *)&ResolvedAddress.ipv6Address, sizeof (IPV6_ADDR));
    					goto DoneSearchingRecords;                        
                    }
                    else
                    {
    					while(DNSAnswerHeader.ResponseLen.Val--)
    					{
    						UDPGet(DNSSocket, &i);
    					}
                    }
				}
				else
				{
					while(DNSAnswerHeader.ResponseLen.Val--)
					{
						UDPGet(DNSSocket, &i);
					}
				}
			}

DoneSearchingRecords:

			UDPDiscard(DNSSocket);
            _DNSReleaseSocket();
			if(Flags.bits.AddressValid)
            {
                smDNS = DNS_DONE;
            }
            else
            {
                smDNS = DNSRetry(DNS_FAIL_SERVER);
            }
            break;  // done
            
		case DNS_FAIL_ARP:
            // see if there is other server we may try
            smDNS = DNSRetry(DNS_FAIL_ARP);
            break;

		case DNS_FAIL_SERVER:
            smDNS = DNSRetry(DNS_FAIL_SERVER);
			break;

        default:    // DNS_DONE, DNS_FAIL_ARP_TMO, DNS_FAIL_OPEN_TMO, DNS_FAIL_SERVER_TMO  
            // either done or some error state
            break;
	}
    
#if DNS_CLIENT_VERSION_NO >= 2
    dnsTickPending = 0;
#endif  // DNS_CLIENT_VERSION_NO >= 2
}

#if DNS_CLIENT_VERSION_NO >= 2
// returns true if service needed
// called by the stack manager
bool DNSClientTaskPending(void)
{
    return dnsTickPending != 0;
}
#endif  // DNS_CLIENT_VERSION_NO >= 2


// see if we can perform a retry
static DNS_STATE DNSRetry(DNS_STATE currState)
{
    _DNSReleaseSocket();
    
    if(currState == DNS_START)
    {
        vDNSServerIx = 0;
    }
    else
    {
        vDNSServerIx++;
    }
    
    for( ; vDNSServerIx < sizeof(DNSServers)/sizeof(*DNSServers); vDNSServerIx++) 
    {   // can try another server if valid address
        if(DNSServers[vDNSServerIx].Val != 0)
        {
            return DNS_OPEN_SOCKET;   // new state
        }
    }

    // nothing else to try
    if(currState == DNS_FAIL_ARP)
    {
        return DNS_FAIL_ARP_TMO;
    }

    // default: DNS_FAIL_SERVER
    return DNS_FAIL_SERVER_TMO;
}

/*****************************************************************************
  Function:
	static void DNSPutString(UDP_SOCKET s, const char* String)

  Summary:
	Writes a string to the DNS socket.
	
  Description:
	This function writes a string to the DNS socket, ensuring that it is
	properly formatted.

  Precondition:
	UDP socket is obtained and ready for writing.

  Parameters:
	String - the string to write to the UDP socket.

  Returns:
  	None
  ***************************************************************************/
static void DNSPutString(UDP_SOCKET s, const char* String)
{
	const char *RightPtr;
	uint8_t i;
	uint8_t Len;

	RightPtr = String;

	while(1)
	{
		do
		{
			i = *RightPtr++;
		} while((i != 0x00u) && (i != '.') && (i != '/') && (i != ',') && (i != '>'));
	
		// Put the length and data
		// Also, skip over the '.' in the input string
		Len = (uint8_t)(RightPtr-String-1);
		UDPPut(s, Len);
		String += UDPPutArray(s, (uint8_t*)String, Len) + 1;

		if(i == 0x00u || i == '/' || i == ',' || i == '>')
			break;
	}
	
	// Put the string null terminator character (zero length label)
	UDPPut(s, 0x00);
}


/*****************************************************************************
  Function:
	static void DNSDiscardName(UDP_SOCKET s)

  Summary:
	Reads a name string or string pointer from the DNS socket and discards it.
	
  Description:
	This function reads a name string from the DNS socket.  Each string 
	consists of a series of labels.  Each label consists of a length prefix 
	byte, followed by the label bytes.  At the end of the string, a zero length 
	label is found as termination.  If name compression is used, this function 
	will automatically detect the pointer and discard it.

  Precondition:
	UDP socket is obtained and ready for reading a DNS name

  Parameters:
	None

  Returns:
  	None
  ***************************************************************************/
static void DNSDiscardName(UDP_SOCKET s)
{
	uint8_t i;

	while(1)
	{
		// Get first byte which will tell us if this is a 16-bit pointer or the 
		// length of the first of a series of labels
		if(!UDPGet(s, &i))
			return;
		
		// Check if this is a pointer, if so, get the reminaing 8 bits and return
		if((i & 0xC0u) == 0xC0u)
		{
			UDPGet(s, &i);
			return;
		}

		// Exit once we reach a zero length label
		if(i == 0u)					
			return;

		// Discard complete label
		UDPGetArray(DNSSocket, NULL, i);		
	}
}

#if DNS_CLIENT_VERSION_NO >= 2
static void DNSTmoHandler(SYS_TICK currSysTick)
{
    dnsTickPending++;
}
#endif  // DNS_CLIENT_VERSION_NO >= 2
    



#endif	//#if defined(TCPIP_STACK_USE_DNS)
