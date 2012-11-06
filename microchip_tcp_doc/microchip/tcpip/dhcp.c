/*******************************************************************************
  Dynamic Host Configuration Protocol (DHCP) Client

  Summary:
    Module for Microchip TCP/IP Stack
    
  Description:
    - Provides automatic IP address, subnet mask, gateway address, 
      DNS server address, and other configuration parameters on DHCP 
      enabled networks.
    - Reference: RFC 2131, 2132
*******************************************************************************/

/*******************************************************************************
FileName:   DHCP.c
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

#define __DHCP_C

#include "tcpip_private.h"
#include "tcpip_config_private.h"
#include "dhcp_private.h"

#if defined(TCPIP_STACK_USE_DHCP_CLIENT)


// Unique variables per interface
typedef struct
{
	UDP_SOCKET			hDHCPSocket;	// Handle to DHCP client socket
	SM_DHCP				smState;		// DHCP client state machine variable
	SYS_TICK			dwTimer;		// Tick timer value used for triggering future events after a certain wait period.
	uint32_t				dwLeaseTime;	// DHCP lease time remaining, in seconds
	uint32_t				dwServerID;		// DHCP Server ID cache
	IP_ADDR				tempIPAddress;	// Temporary IP address to use when no DHCP lease
	IP_ADDR				tempGateway;	// Temporary gateway to use when no DHCP lease
	IP_ADDR				tempMask;		// Temporary mask to use when no DHCP lease
	#if defined(TCPIP_STACK_USE_DNS)
	IP_ADDR				tempDNS;		// Temporary primary DNS server
	IP_ADDR				tempDNS2;		// Temporary secondary DNS server
	#endif	
    TCPIP_UINT32_VAL           transactionID;  // current transaction ID
	union
	{
	    struct
	    {
	        unsigned char bDHCPEnabled        : 1;		// Whether or not DHCP is currently enabled
	        unsigned char bIsBound            : 1;		// Whether or not DHCP is currently bound
	        unsigned char bOfferReceived      : 1;		// Whether or not an offer has been received
			unsigned char bDHCPServerDetected : 1;	    // Indicates if a DCHP server has been detected
			unsigned char bUseUnicastMode     : 1;		// Indicates if the 
	        unsigned char bDHCPDefaultEnabled : 1;		// Whether or not DHCP is by default enabled

	    };
	    uint8_t val;
	} flags;
	// Indicates which DHCP values are currently valid
	union
	{
		struct
		{
			char IPAddress:1;	// Leased IP address is valid
			char Gateway:1;		// Gateway address is valid
			char Mask:1;		// Subnet mask is valid
			char DNS:1;			// Primary DNS is valid
			char DNS2:1;		// Secondary DNS is valid
			char HostName:1;	// Host name is valid (not implemented)
		};
		uint8_t val;
	} validValues;
} DHCP_CLIENT_VARS;

static uint8_t  _DHCPReceive(DHCP_CLIENT_VARS* pClient, NET_CONFIG* pNetIf);
static void     _DHCPSend(DHCP_CLIENT_VARS* pClient, NET_CONFIG* pNetIf, uint8_t messageType, bool bRenewing);
static void     _DHCPNotifyClients(NET_CONFIG* pNetIf, DHCP_EVENT_TYPE evType);


static DHCP_CLIENT_VARS	DHCPClients[TCPIP_NETWORK_INTERFACES];

static int      dhcpInitCount = 0;      // DHCP module initialization count


// DHCP event registration

static const void*          dhcpMemH = 0;        // memory handle

static SINGLE_LIST      dhcpRegisteredUsers = { 0 };

static void _DHCPClose(DHCP_CLIENT_VARS* pClient, bool disable)
{
    if(pClient->flags.bDHCPEnabled != 0)
    {
        if(pClient->hDHCPSocket != INVALID_UDP_SOCKET)
        {
            UDPClose(pClient->hDHCPSocket);
            pClient->hDHCPSocket = INVALID_UDP_SOCKET;
        }
        
		pClient->flags.bIsBound = false;
        pClient->flags.bDHCPServerDetected = false;
        
        if(disable)
        {
            pClient->flags.bDHCPEnabled = false;
        }
        else
        {   // let it active
            pClient->smState = SM_DHCP_IDLE;
        }
    }
}

static void _DHCPEnable(NET_CONFIG* pNetIf, bool connUp)
{
    DHCP_CLIENT_VARS* pClient = DHCPClients + _TCPIPStackNetIx(pNetIf);
    
    if(connUp && MACIsLinked(_TCPIPStackNetToMac(pNetIf)))
    {
        pClient->smState = SM_DHCP_WAIT_LINK;
    }
    else
    {
        pClient->smState = SM_DHCP_IDLE;
    }
    
    pClient->flags.bDHCPEnabled = true;
}

static void _DHCPCleanup(void)
{
    SGL_LIST_NODE* dNode;

    while( (dNode = SingleListRemoveHead(&dhcpRegisteredUsers)) != 0 )
    {
        TCPIP_HEAP_Free(dhcpMemH, dNode);
    }
}

/*****************************************************************************
  Function:
    bool DHCPInit(const TCPIP_STACK_MODULE_CTRL* const stackCtrl, const DHCP_MODULE_CONFIG* pDhcpConfig);

  Summary:
	Resets the DHCP client module for the specified interface.

  Description:
	Resets the DHCP client module, giving up any current lease, knowledge of 
	DHCP servers, etc. for the specified interface.

  Precondition:
	None

  Parameters:
	stackCtrl - pointer to stack structure specifying the interface to initialize

  Returns:
	None

  Remarks:
	This function should be called internally just once per interface 
    by the stack manager.
***************************************************************************/
bool DHCPInit(const TCPIP_STACK_MODULE_CTRL* const stackCtrl, const DHCP_MODULE_CONFIG* pDhcpConfig)
{
    DHCP_CLIENT_VARS*   pClient;
	
    if(stackCtrl->stackAction == TCPIP_STACK_ACTION_IF_UP)
    {   // interface restart
        pClient = DHCPClients + stackCtrl->netIx;
        if(pClient->flags.bDHCPDefaultEnabled)
        {
            _DHCPEnable(stackCtrl->pNetIf, false);
        }
        return true;
    }

    // stack init
    
    if(dhcpInitCount == 0)
    {   // first time we're run
        int ix;
        for(ix = 0, pClient = DHCPClients; ix < stackCtrl->nIfs; ix++, pClient++)
        {
            memset(pClient, 0x0, sizeof(*pClient));
            pClient->hDHCPSocket = INVALID_UDP_SOCKET;
        }

        SingleListInit(&dhcpRegisteredUsers);
        // store the memory allocation handle
        dhcpMemH = stackCtrl->memH;
    }
            
    pClient = DHCPClients + stackCtrl->netIx;

    // Reset state machine and flags to default values
    pClient->flags.val = 0;
    // This flag toggles before use, so this statement actually means to start out using broadcast mode.
    pClient->flags.bUseUnicastMode = true;	

    if(pDhcpConfig->dhcpEnable)
    {
        pClient->flags.bDHCPDefaultEnabled =  true;
        _DHCPEnable(stackCtrl->pNetIf, false);
    }

    dhcpInitCount++;


    return true;
}

/*****************************************************************************
  Function:
    bool DHCPDeinit(const TCPIP_STACK_MODULE_CTRL* const stackCtrl);

  Summary:
	Turns off the DHCP client module for the specified interface.

  Description:
	Closes out UDP socket.

  Precondition:
	None

  Parameters:
	stackData - pointer to stack structure specifying the interface to deinitialize

  Returns:
	None

  Remarks:
	This function should be called internally just once per interface 
    by the stack manager.
***************************************************************************/
void DHCPDeInit(const TCPIP_STACK_MODULE_CTRL* const stackCtrl)
{
    DHCP_CLIENT_VARS*   pClient = DHCPClients + stackCtrl->netIx;

    // if(stackCtrl->stackAction == TCPIP_STACK_ACTION_DEINIT) // stack shut down
    // if(stackCtrl->stackAction == TCPIP_STACK_ACTION_IF_DOWN) // interface down

    // one way or another this interface is going down
    _DHCPClose(pClient, true);

    //  the registered users for this interface are not removed
    //  since this interface is closed there won't be any event generated on it anyway
    //  deallocation will wait for the whole stack to deinit 
    
    if(stackCtrl->stackAction == TCPIP_STACK_ACTION_DEINIT)
    {   // whole stack is going down
        if(dhcpInitCount > 0)
        {   // we're up and running
            if(--dhcpInitCount == 0)
            {   // all closed
                // release resources
                _DHCPCleanup();
                dhcpMemH = 0;
            }
        }
    }

}

/*****************************************************************************
  Function:
	void DHCPDisable(TCPIP_NET_HANDLE hNet, bool keepLease)

  Summary:
	Disables the DHCP Client for the specified interface.

  Description:
	Disables the DHCP client for the specified interface.
	If the interface was previously configured by DHCP and keepLease
    is requested, the configuration will continue to be used
    but the module will no longer preform any renewals.

  Precondition:
	None

  Parameters:
	pNetIf - Interface to disable the DHCP client on.
    keepLease - keep the old lease if any; else discard

  Returns:
	true if success
    false otherwise

  Remarks:
	When the interface continues using its old configuration, it is possible 
	that the lease may expire and the DHCP server provide the IP to another
	client.
    The application should not request the keeping of the old lease
    unless there is no danger of conflict.
***************************************************************************/
bool DHCPDisable(TCPIP_NET_HANDLE hNet, bool keepLease)
{
    NET_CONFIG* pNetIf = _TCPIPStackHandleToNet(hNet);
    if(pNetIf)
    {
        _DHCPClose(DHCPClients + _TCPIPStackNetIx(pNetIf), true);
        if(keepLease == false)
        {
            _TCPIPStackSetDefaultAddress(pNetIf);
        }
        return true;
    }

    return false;
}


/*****************************************************************************
  Function:
	void DHCPEnable(TCPIP_NET_HANDLE hNet)

  Summary:
	Enables the DHCP client for the specified interface.

  Description:
	Enables the DHCP client for the specified interface, if it is disabled.  
	If it is already enabled, nothing is done.

  Precondition:
	None

  Parameters:
	 hNet - Interface to enable the DHCP client on. 

  Returns:
	true if success
    false otherwise
***************************************************************************/
bool DHCPEnable(TCPIP_NET_HANDLE hNet)
{
    NET_CONFIG* pNetIf = _TCPIPStackHandleToNet(hNet);
    if(pNetIf)
    {
        DHCP_CLIENT_VARS* pClient = DHCPClients + _TCPIPStackNetIx(pNetIf);
        if(pClient->flags.bDHCPEnabled == 0)
        {
            _DHCPEnable(pNetIf, false);
        }
        return true;
    }

    return false;
}

/*****************************************************************************
  Function:
	bool DHCPIsEnabled(CPIP_NET_HANDLE hNet)

  Summary:
	Determins if the DHCP client is enabled on the specified interface.

  Description:
	Determins if the DHCP client is enabled on the specified interface.

  Precondition:
	None

  Parameters:
	 hNet- Interface to query.

  Returns:
	None
***************************************************************************/
bool DHCPIsEnabled(TCPIP_NET_HANDLE hNet)
{
    NET_CONFIG* pNetIf = _TCPIPStackHandleToNet(hNet);
    if(pNetIf)
    {
        return (DHCPClients + _TCPIPStackNetIx(pNetIf))->flags.bDHCPEnabled != 0;
    }
    return false;
}


/*****************************************************************************
  Function:
	bool DHCPIsBound(TCPIP_NET_HANDLE hNet)

  Summary:
	Determins if the DHCP client has an IP address lease on the specified 
	interface.

  Description:
	Determins if the DHCP client has an IP address lease on the specified 
	interface.

  Precondition:
	None

  Parameters:
	hNet - Interface to query

  Returns:
	true - DHCP client has obtained an IP address lease (and likely other 
		parameters) and these values are currently being used.
	false - No IP address is currently leased
***************************************************************************/
bool DHCPIsBound(TCPIP_NET_HANDLE hNet)
{
    NET_CONFIG* pNetIf = _TCPIPStackHandleToNet(hNet);
    if(pNetIf)
    {
        return (DHCPClients + _TCPIPStackNetIx(pNetIf))->flags.bIsBound;
    }

    return false;
}


/*****************************************************************************
  Function:
	bool DHCPIsServerDetected(TCPIP_NET_HANDLE hNet)

  Summary:
	Determins if the DHCP client on the specified interface has seen a DHCP 
	server.

  Description:
	Determins if the DHCP client on the specified interface has seen a DHCP 
	server.
	
  Precondition:
	None

  Parameters:
	hNet- Interface to query.

  Returns:
	true - At least one DHCP server is attached to the specified network 
		interface.
	false - No DHCP servers are currently detected on the specified network 
		interface.
***************************************************************************/
bool DHCPIsServerDetected(TCPIP_NET_HANDLE hNet)
{
    NET_CONFIG* pNetIf = _TCPIPStackHandleToNet(hNet);
    if(pNetIf)
    {
        return (DHCPClients + _TCPIPStackNetIx(pNetIf))->flags.bDHCPServerDetected;
    }
    return false;
}


/*****************************************************************************
  Function:
	void DHCPTask(NET_CONFIG* pNetIf)

  Summary:
	Performs periodic DHCP tasks for all interfaces.

  Description:
	This function performs any periodic tasks requied by the DHCP module, 
	such as sending and receiving messages involved with obtaining and
	maintaining a lease.

  Precondition:
	None

  Parameters:
	pNetIf   - pointer to the interface the DHCP is running on

  Returns:
	None
***************************************************************************/
void DHCPTask(NET_CONFIG* pNetIf)
{
    DHCP_CLIENT_VARS*   pClient;
    bool                udpSuccess;
    IPV4_ADDR           bcastAdd = {0xffffffff};

    pClient = DHCPClients + _TCPIPStackNetIx(pNetIf); 

    if(pClient->flags.bDHCPEnabled == false)
    {   // not enabled on this interface
        return;
    }

    switch(pClient->smState)
    {
        case SM_DHCP_IDLE:
            break;

        case SM_DHCP_WAIT_LINK:
            if(!MACIsLinked(_TCPIPStackNetToMac(pNetIf)))
            {   // no connection yet
                break;
            }
            
            pClient->smState = SM_DHCP_GET_SOCKET;
            // and fall through

        case SM_DHCP_GET_SOCKET:
            // Open a socket to send and receive broadcast messages on
            pClient->hDHCPSocket = UDPOpenClient(IP_ADDRESS_TYPE_IPV4, DHCP_SERVER_PORT, 0);
            if(pClient->hDHCPSocket == INVALID_UDP_SOCKET)
            {
                break;
            }
            // and bind to the DHCP local port
            udpSuccess = UDPBind(pClient->hDHCPSocket, IP_ADDRESS_TYPE_IPV4, DHCP_CLIENT_PORT,  0);
            if(udpSuccess)
            {
                // bind to this interface
                UDPSocketSetNet(pClient->hDHCPSocket, pNetIf);
                // allow sending broadcast messages
                udpSuccess = UDPSetBcastIPAddress(pClient->hDHCPSocket, IP_ADDRESS_TYPE_IPV4, (IP_MULTI_ADDRESS*)&bcastAdd);
            }
            if(!udpSuccess)
            {
                UDPClose(pClient->hDHCPSocket);
                pClient->hDHCPSocket = INVALID_UDP_SOCKET;
                {
                    break;
                }
            }
            pClient->smState = SM_DHCP_SEND_DISCOVERY;
            // No break

        case SM_DHCP_SEND_DISCOVERY:
            // since we don't do ARP here anymore we have to wait for the UDP to do it!
            if(!UDPIsOpened(pClient->hDHCPSocket))
            {
                break;
            }
            // Assume default IP Lease time of 60 seconds.
            // This should be minimum possible to make sure that if the
            // server did not specify lease time, we try again after this 
            // minimum time.
            pClient->dwLeaseTime = 60;
            pClient->validValues.val = 0x00;
            pClient->flags.bIsBound = false;	
            pClient->flags.bOfferReceived = false;
            pClient->transactionID.Val = rand();


            // Ensure transmitter is ready to accept data
            if(UDPIsTxPutReady(pClient->hDHCPSocket, 300) < 300u)
                break;

            // Toggle the BOOTP Broadcast flag to ensure compatibility with 
            // bad DHCP servers that don't know how to handle broadcast 
            // responses.  This results in the next discovery attempt to be 
            // made using the opposite mode.
            pClient->flags.bUseUnicastMode ^= 1;

            // Ensure that we transmit to the broadcast IP and MAC addresses
            // The UDP Socket remembers who it was last talking to
                TCPIP_IPV4_SetDestAddress(UDPSocketDcpt[pClient->hDHCPSocket].pTxPkt,0xFFFFFFFF);
				memset((void*)&UDPSocketDcpt[pClient->hDHCPSocket].pTxPkt->remoteMACAddr, 0xFF, sizeof(MAC_ADDR));

            // Send the DHCP Discover broadcast
            _DHCPSend(pClient, pNetIf, DHCP_DISCOVER_MESSAGE, false);

            _DHCPNotifyClients(pNetIf, DHCP_EVENT_DISCOVER);
            // Start a timer and begin looking for a response
            pClient->dwTimer = SYS_TICK_Get();
            pClient->smState = SM_DHCP_GET_OFFER;
            break;

        case SM_DHCP_GET_OFFER:
            // Check to see if a packet has arrived
            if(UDPIsGetReady(pClient->hDHCPSocket) < 250u)
            {
                // Go back and transmit a new discovery if we didn't get an offer after 2 seconds
                if(SYS_TICK_Get() - pClient->dwTimer >= (DHCP_TIMEOUT * SYS_TICK_TicksPerSecondGet()))
                    pClient->smState = SM_DHCP_SEND_DISCOVERY;
                break;
            }

            // Let the DHCP server module know that there is a DHCP server 
            // on this network
            pClient->flags.bDHCPServerDetected = true;

            // Check to see if we received an offer
            if(_DHCPReceive(pClient, pNetIf) != DHCP_OFFER_MESSAGE)
                break;

            pClient->smState = SM_DHCP_SEND_REQUEST;
            // No break

        case SM_DHCP_SEND_REQUEST:
            if(UDPIsTxPutReady(pClient->hDHCPSocket, 300) < 300u)
                break;

            // Ensure that we transmit to the broadcast IP and MAC addresses
            // The UDP Socket remembers who it was last talking to, so 
            // we must set this back to the broadcast address since the 
            // current socket values are the unicast addresses of the DHCP 
            // server.
            TCPIP_IPV4_SetDestAddress(UDPSocketDcpt[pClient->hDHCPSocket].pTxPkt,0xFFFFFFFF);
			memset((void*)&UDPSocketDcpt[pClient->hDHCPSocket].pTxPkt->remoteMACAddr, 0xFF, sizeof(MAC_ADDR));

            // Send the DHCP request message
            _DHCPSend(pClient, pNetIf, DHCP_REQUEST_MESSAGE, false);

            // Start a timer and begin looking for a response
            pClient->dwTimer = SYS_TICK_Get();
            pClient->smState = SM_DHCP_GET_REQUEST_ACK;
            break;

        case SM_DHCP_GET_REQUEST_ACK:
            // Check to see if a packet has arrived
            if(UDPIsGetReady(pClient->hDHCPSocket) < 250u)
            {
                // Go back and transmit a new discovery if we didn't get an ACK after 2 seconds
                if(SYS_TICK_Get() - pClient->dwTimer >= (DHCP_TIMEOUT * SYS_TICK_TicksPerSecondGet()))
                    pClient->smState = SM_DHCP_SEND_DISCOVERY;
                break;
            }

            // Check to see if we received an offer
            switch(_DHCPReceive(pClient, pNetIf))
            {
                case DHCP_ACK_MESSAGE:
                    UDPClose(pClient->hDHCPSocket);
                    pClient->hDHCPSocket = INVALID_UDP_SOCKET;
                    pClient->dwTimer = SYS_TICK_Get();
                    pClient->smState = SM_DHCP_BOUND;
                    pClient->flags.bIsBound = true;	


                    if(pClient->validValues.IPAddress)
                        _TCPIPStackSetNetAddress(pNetIf, &pClient->tempIPAddress);
                    if(pClient->validValues.Mask)
                        _TCPIPStackSetNetMask(pNetIf, &pClient->tempMask);
                    if(pClient->validValues.Gateway)
                        _TCPIPStackSetGatewayAddress(pNetIf, &pClient->tempGateway);
#if defined(TCPIP_STACK_USE_DNS)
                    if(pClient->validValues.DNS)
                        _TCPIPStackSetPriDNSAddress(pNetIf, &pClient->tempDNS);
                    if(pClient->validValues.DNS2)
                    {
                        _TCPIPStackSetSecondDNSAddress(pNetIf, &pClient->tempDNS2);
                    }
                    else
                    {
                        IP_ADDR zeroAdd = {0};
                        _TCPIPStackSetSecondDNSAddress(pNetIf, &zeroAdd);
                    }
                    
#endif
                    //if(pClient->validValues.HostName)
                    //	memcpy(pNetIf->NetBIOSName, (void*)pClient->tempHostName, sizeof(pNetIf->NetBIOSName));
                    _DHCPNotifyClients(pNetIf, DHCP_EVENT_BOUND);
                    
                    break;

                case DHCP_NAK_MESSAGE:
                    pClient->smState = SM_DHCP_SEND_DISCOVERY;
                    break;
            }
            break;

        case SM_DHCP_BOUND:
            if(SYS_TICK_Get() - pClient->dwTimer < SYS_TICK_TicksPerSecondGet())
                break;

            // Check to see if our lease is still valid, if so, decrement lease 
            // time
            if(pClient->dwLeaseTime >= 2ul)
            {
                pClient->dwTimer += SYS_TICK_TicksPerSecondGet();
                pClient->dwLeaseTime--;
                break;
            }

            _DHCPNotifyClients(pNetIf, DHCP_EVENT_LEASE_EXPIRED);
            // Open a socket to send and receive DHCP messages on
            pClient->hDHCPSocket = UDPOpenClient(IP_ADDRESS_TYPE_IPV4, DHCP_SERVER_PORT, 0);
            if(pClient->hDHCPSocket == INVALID_UDP_SOCKET)
            {
                break;
            }
            // allow sending broadcast messages
            // and bind to the DHCP local port
            if(!UDPBind(pClient->hDHCPSocket, IP_ADDRESS_TYPE_IPV4, DHCP_CLIENT_PORT,  (IP_MULTI_ADDRESS*)&bcastAdd))
            {
                UDPClose(pClient->hDHCPSocket);
                pClient->hDHCPSocket = INVALID_UDP_SOCKET;
                {
                    break;
                }
            }
            UDPSocketSetNet(pClient->hDHCPSocket, pNetIf);            
            pClient->smState = SM_DHCP_SEND_RENEW;
            // No break

        case SM_DHCP_SEND_RENEW:
        case SM_DHCP_SEND_RENEW2:
        case SM_DHCP_SEND_RENEW3:
            if(UDPIsTxPutReady(pClient->hDHCPSocket, 300) < 300u)
                break;

            // Send the DHCP request message
            _DHCPSend(pClient, pNetIf, DHCP_REQUEST_MESSAGE, true);
            pClient->flags.bOfferReceived = false;

            // Start a timer and begin looking for a response
            pClient->dwTimer = SYS_TICK_Get();
            pClient->smState++;
            break;

        case SM_DHCP_GET_RENEW_ACK:
        case SM_DHCP_GET_RENEW_ACK2:
        case SM_DHCP_GET_RENEW_ACK3:
            // Check to see if a packet has arrived
            if(UDPIsGetReady(pClient->hDHCPSocket) < 250u)
            {
                // Go back and transmit a new discovery if we didn't get an ACK after 2 seconds
                if(SYS_TICK_Get() - pClient->dwTimer >=  (DHCP_TIMEOUT * SYS_TICK_TicksPerSecondGet()))
                {
                    if(++pClient->smState > SM_DHCP_GET_RENEW_ACK3)
                        pClient->smState = SM_DHCP_SEND_DISCOVERY;
                }
                break;
            }

            // Check to see if we received an offer
            switch(_DHCPReceive(pClient, pNetIf))
            {
                case DHCP_ACK_MESSAGE:
                    UDPClose(pClient->hDHCPSocket);
                    pClient->hDHCPSocket = INVALID_UDP_SOCKET;
                    pClient->dwTimer = SYS_TICK_Get();
                    pClient->smState = SM_DHCP_BOUND;
                    _DHCPNotifyClients(pNetIf, DHCP_EVENT_BOUND);
                    break;

                case DHCP_NAK_MESSAGE:
                    pClient->smState = SM_DHCP_SEND_DISCOVERY;
                    break;
            }
            break;
    }
}



/*****************************************************************************
Function:
  void _DHCPReceive(DHCP_CLIENT_VARS* pClient, NET_CONFIG* pNetIf)

Description:
  Receives and parses a DHCP message.

Precondition:
  A DHCP message is waiting in the UDP buffer.

Parameters:
  pClient - client descriptor
  pNetIf - interface to use

Returns:
  One of the DCHP_TYPE* contants.
***************************************************************************/
static uint8_t _DHCPReceive(DHCP_CLIENT_VARS* pClient, NET_CONFIG* pNetIf)
{
	/*********************************************************************
	DHCP PACKET FORMAT AS PER RFC 1541

	0                   1                   2                   3
	0 1 2 3 4 5 6 7 8 9 0 1 2 3 4 5 6 7 8 9 0 1 2 3 4 5 6 7 8 9 0 1
	+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+
	|     op (1)    |   htype (1)   |   hlen (1)    |   hops (1)    |
	+---------------+---------------+---------------+---------------+
	|                            xid (4)                            |
	+-------------------------------+-------------------------------+
	|           secs (2)            |           flags (2)           |
	+-------------------------------+-------------------------------+
	|                          ciaddr  (4)                          |
	+---------------------------------------------------------------+
	|                          yiaddr  (4)                          |
	+---------------------------------------------------------------+
	|                          siaddr  (4)                          |
	+---------------------------------------------------------------+
	|                          giaddr  (4)                          |
	+---------------------------------------------------------------+
	|                                                               |
	|                          chaddr  (16)                         |
	|                                                               |
	|                                                               |
	+---------------------------------------------------------------+
	|                                                               |
	|                          sname   (64)                         |
	+---------------------------------------------------------------+
	|                                                               |
	|                          file    (128)                        |
	+---------------------------------------------------------------+
	|                                                               |
	|                          options (312)                        |
	+---------------------------------------------------------------+

	********************************************************************/
	uint8_t v;
	uint8_t i, j;
	uint8_t type;
	bool lbDone;
	uint32_t tempServerID;
    TCPIP_UINT32_VAL transactionID;
    UDP_SOCKET      s;

	// Assume unknown message until proven otherwise.
	type = DHCP_UNKNOWN_MESSAGE;

    s = pClient->hDHCPSocket;
	UDPGet(s, &v);                             // op

	// Make sure this is BOOT_REPLY.
	if ( v == BOOT_REPLY )
	{
        // check the transaction ID
		UDPSetRxOffset(s, 4);
        UDPGetArray(s, transactionID.v, sizeof(transactionID.v));
        if(transactionID.Val != pClient->transactionID.Val)
        {   // not what we're expecting
            goto UDPInvalid;
        }

		// Jump to chaddr field (Client Hardware Address -- our MAC address for 
		// Ethernet and WiFi networks) and verify that this message is directed 
		// to us before doing any other processing.
		UDPSetRxOffset(s, 28);		// chaddr field is at offset 28 in the UDP packet payload -- see DHCP packet format above
		for ( i = 0; i < 6u; i++ )
		{
			UDPGet(s, &v);
			if ( v != pNetIf->MyMACAddr.v[i])
				goto UDPInvalid;
		}

		// Check to see if this is the first offer.  If it is, record its 
		// yiaddr value ("Your (client) IP address") so that we can REQUEST to 
		// use it later.
		if(!pClient->flags.bOfferReceived)
		{
			UDPSetRxOffset(s, 16);
			UDPGetArray(s, (uint8_t*)&pClient->tempIPAddress, sizeof(pClient->tempIPAddress));
			pClient->validValues.IPAddress = 1;
		}

		// Jump to DHCP options (ignore htype, hlen, hops, xid, secs, flags, 
		// ciaddr, siaddr, giaddr, padding part of chaddr, sname, file, magic 
		// cookie fields)
		UDPSetRxOffset(s, 240);

		lbDone = false;
		do
		{
			// Get the Option number
			// Break out eventually in case if this is a malformed 
			// DHCP message, ie: missing DHCP_END_OPTION marker
			if(!UDPGet(s, &v))
			{
				lbDone = true;
				break;
			}

			switch(v)
			{
				case DHCP_MESSAGE_TYPE:
					UDPGet(s, &v);                         // Skip len
					// Len must be 1.
					if ( v == 1u )
					{
						UDPGet(s, &type);                  // Get type

						// Throw away the packet if we know we don't need it (ie: another offer when we already have one)
						if(pClient->flags.bOfferReceived && (type == DHCP_OFFER_MESSAGE))
						{
							goto UDPInvalid;
						}
					}
					else
						goto UDPInvalid;
					break;

				case DHCP_SUBNET_MASK:
					UDPGet(s, &v);                     // Skip len
					// Len must be 4.
					if ( v == 4u )
					{
						// Check to see if this is the first offer
						if(pClient->flags.bOfferReceived)
						{
							// Discard offered IP mask, we already have an offer
							for ( i = 0; i < 4u; i++ )
								UDPGet(s, &v);
						}
						else
						{
							UDPGetArray(s, (uint8_t*)&pClient->tempMask, sizeof(pClient->tempMask));
							pClient->validValues.Mask = 1;
						}
					}
					else
						goto UDPInvalid;
					break;

				case DHCP_ROUTER:
					UDPGet(s, &j);
					// Len must be >= 4.
					if ( j >= 4u )
					{
						// Check to see if this is the first offer
						if(pClient->flags.bOfferReceived)
						{
							// Discard offered Gateway address, we already have an offer
							for ( i = 0; i < 4u; i++ )
								UDPGet(s, &v);
						}
						else
						{
							UDPGetArray(s, (uint8_t*)&pClient->tempGateway, sizeof(pClient->tempGateway));
							pClient->validValues.Gateway = 1;
						}
					}
					else
						goto UDPInvalid;

					// Discard any other router addresses.
					j -= 4;
					while(j--)
						UDPGet(s, &v);
					break;

				#if defined(TCPIP_STACK_USE_DNS)
				case DHCP_DNS:
					UDPGet(s, &j);
					// Len must be >= 4.
					if(j < 4u)
						goto UDPInvalid;

					// Check to see if this is the first offer
					if(!pClient->flags.bOfferReceived)
					{
						UDPGetArray(s, (uint8_t*)&pClient->tempDNS, sizeof(pClient->tempDNS));
						pClient->validValues.DNS = 1;
						j -= 4;
					}

					// Len must be >= 4 for a secondary DNS server address
					if(j >= 4u)
					{
						// Check to see if this is the first offer
						if(!pClient->flags.bOfferReceived)
						{
							UDPGetArray(s, (uint8_t*)&pClient->tempDNS2, sizeof(pClient->tempDNS2));
							pClient->validValues.DNS2 = 1;
							j -= 4;
						}
					}

					// Discard any other DNS server addresses
					while(j--)
						UDPGet(s, &v);
					break;
				#endif

					//            case DHCP_HOST_NAME:
					//                UDPGet(s, &j);
					//                // Len must be >= 4.
					//                if(j < 1u)
					//					goto UDPInvalid;
					//
					//				// Check to see if this is the first offer
					//				if(DHCPFlags.bOfferReceived)
					//				{
					//			        // Discard offered host name, we already have an offer
					//	                while(j--)
					//	                    UDPGet(s, &v);
					//				}
					//				else
					//				{
					//					for(i = 0; j, i < sizeof(tempHostName); i++, j--)
					//					{
					//						UDPGet(s, &tempHostName[i]);
					//					}
					//					while(j--)
					//					{
					//						UDPGet(s, &v);
					//					}
					//					ValidValues.HostName = 1;
					//				}
					//
					//                break;

				case DHCP_SERVER_IDENTIFIER:
					UDPGet(s, &v);                         // Get len
					// Len must be 4.
					if ( v == 4u )
					{
						UDPGet(s, &(((uint8_t*)&tempServerID)[3]));   // Get the id
						UDPGet(s, &(((uint8_t*)&tempServerID)[2]));
						UDPGet(s, &(((uint8_t*)&tempServerID)[1]));
						UDPGet(s, &(((uint8_t*)&tempServerID)[0]));
					}
					else
						goto UDPInvalid;
					break;

				case DHCP_END_OPTION:
					lbDone = true;
					break;

				case DHCP_IP_LEASE_TIME:
					UDPGet(s, &v);                         // Get len
					// Len must be 4.
					if ( v == 4u )
					{
						// Check to see if this is the first offer
						if(pClient->flags.bOfferReceived)
						{
							// Discard offered lease time, we already have an offer
							for ( i = 0; i < 4u; i++ )
								UDPGet(s, &v);
						}
						else
						{
							UDPGet(s, &(((uint8_t*)(&pClient->dwLeaseTime))[3]));
							UDPGet(s, &(((uint8_t*)(&pClient->dwLeaseTime))[2]));
							UDPGet(s, &(((uint8_t*)(&pClient->dwLeaseTime))[1]));
							UDPGet(s, &(((uint8_t*)(&pClient->dwLeaseTime))[0]));

							// In case if our clock is not as accurate as the remote 
							// DHCP server's clock, let's treat the lease time as only 
							// 96.875% of the value given
							pClient->dwLeaseTime -= pClient->dwLeaseTime>>5;
						}
					}
					else
						goto UDPInvalid;
					break;

				default:
					// Ignore all unsupport tags.
					UDPGet(s, &j);                     // Get option len
					while( j-- )                    // Ignore option values
						UDPGet(s, &v);
			}
		} while( !lbDone );
	}

	// If this is an OFFER message, remember current server id.
	if ( type == DHCP_OFFER_MESSAGE )
	{
		pClient->dwServerID = tempServerID;
		pClient->flags.bOfferReceived = true;
	}
	else if(type != DHCP_UNKNOWN_MESSAGE)
	{
		// For other types of messages, make sure that received
		// server id matches with our previous one.
		if ( pClient->dwServerID != tempServerID )
        {
			type = DHCP_UNKNOWN_MESSAGE;
        }
	}

	UDPDiscard(s);                             // We are done with this packet
	return type;

UDPInvalid:
	UDPDiscard(s);
	return DHCP_UNKNOWN_MESSAGE;
}




/*****************************************************************************
  Function:
	static void _DHCPSend(DHCP_CLIENT_VARS* pClient, NET_CONFIG* pNetIf, uint8_t messageType, bool bRenewing)

  Description:
	Sends a DHCP message.

  Precondition:
	UDP is ready to write a DHCP packet.

  Parameters:
    pClient  - client descriptor
    pNetIf   - interface to use
	messageType - One of the DHCP_TYPE constants
	bRenewing - Whether or not this is a renewal request

  Returns:
	None
***************************************************************************/
static void _DHCPSend(DHCP_CLIENT_VARS* pClient, NET_CONFIG* pNetIf, uint8_t messageType, bool bRenewing)
{
	uint8_t        i;
    UDP_SOCKET     s;
	IP_ADDR     zeroIP = { 0 };

    s = pClient->hDHCPSocket;

	UDPPut(s, BOOT_REQUEST);                       // op
	UDPPut(s, BOOT_HW_TYPE);                       // htype
	UDPPut(s, BOOT_LEN_OF_HW_TYPE);                // hlen
	UDPPut(s, 0);                                  // hops
	UDPPut(s, pClient->transactionID.v[0]);      // xid[0]
	UDPPut(s, pClient->transactionID.v[1]);      // xid[1]
	UDPPut(s, pClient->transactionID.v[2]);      // xid[2]
	UDPPut(s, pClient->transactionID.v[3]);      // xid[3]
	UDPPut(s, 0);                                  // secs[0]
	UDPPut(s, 0);                                  // secs[1]
	UDPPut(s, pClient->flags.bUseUnicastMode ? 0x00: 0x80);// flags[0] with Broadcast flag clear/set to correspond to bUseUnicastMode
	UDPPut(s, 0);                                  // flags[1]

	// If this is DHCP REQUEST message, use previously allocated IP address.
	if((messageType == DHCP_REQUEST_MESSAGE) && bRenewing)
	{
		UDPPutArray(s, (uint8_t*)&pClient->tempIPAddress, sizeof(pClient->tempIPAddress));
	}
	else
	{
		UDPPut(s, 0x00);
		UDPPut(s, 0x00);
		UDPPut(s, 0x00);
		UDPPut(s, 0x00);
	}

	// Set yiaddr, siaddr, giaddr as zeros,
	for ( i = 0; i < 12u; i++ )
		UDPPut(s, 0x00);

	// Load chaddr - Client hardware address.
	UDPPutArray(s, pNetIf->MyMACAddr.v, sizeof(pNetIf->MyMACAddr));

	// Set chaddr[6..15], sname and file as zeros.
	for ( i = 0; i < 202u; i++ )
		UDPPut(s, 0);

	// Load magic cookie as per RFC 1533.
	UDPPut(s, 99);
	UDPPut(s, 130);
	UDPPut(s, 83);
	UDPPut(s, 99);

	// Load message type.
	UDPPut(s, DHCP_MESSAGE_TYPE);
	UDPPut(s, DHCP_MESSAGE_TYPE_LEN);
	UDPPut(s, messageType);

	if(messageType == DHCP_DISCOVER_MESSAGE)
	{
		// Reset offered flag so we know to act upon the next valid offer
		pClient->flags.bOfferReceived = false;
	}


	if((messageType == DHCP_REQUEST_MESSAGE) && !bRenewing)
	{
		// DHCP REQUEST message must include server identifier the first time
		// to identify the server we are talking to.
		// _DHCPReceive() would populate "serverID" when it
		// receives DHCP OFFER message. We will simply use that
		// when we are replying to server.
		// If this is a renwal request, we must not include server id.
		UDPPut(s, DHCP_SERVER_IDENTIFIER);
		UDPPut(s, DHCP_SERVER_IDENTIFIER_LEN);
		UDPPut(s, ((uint8_t*)(&pClient->dwServerID))[3]);
		UDPPut(s, ((uint8_t*)(&pClient->dwServerID))[2]);
		UDPPut(s, ((uint8_t*)(&pClient->dwServerID))[1]);
		UDPPut(s, ((uint8_t*)(&pClient->dwServerID))[0]);
	}

	// Load our interested parameters
	// This is hardcoded list.  If any new parameters are desired,
	// new lines must be added here.
	UDPPut(s, DHCP_PARAM_REQUEST_LIST);
	UDPPut(s, DHCP_PARAM_REQUEST_LIST_LEN);
	UDPPut(s, DHCP_SUBNET_MASK);
	UDPPut(s, DHCP_ROUTER);
	UDPPut(s, DHCP_DNS);
	UDPPut(s, DHCP_HOST_NAME);

	// Add requested IP address to DHCP Request Message
	if( ((messageType == DHCP_REQUEST_MESSAGE) && !bRenewing) || 
		((messageType == DHCP_DISCOVER_MESSAGE) && pClient->tempIPAddress.Val))
	{
		UDPPut(s, DHCP_PARAM_REQUEST_IP_ADDRESS);
		UDPPut(s, DHCP_PARAM_REQUEST_IP_ADDRESS_LEN);
		UDPPutArray(s, (uint8_t*)&pClient->tempIPAddress, DHCP_PARAM_REQUEST_IP_ADDRESS_LEN);
	}

	// Add any new paramter request here.

	// End of Options.
	UDPPut(s, DHCP_END_OPTION);

	// Add zero padding to ensure compatibility with old BOOTP relays that discard small packets (<300 UDP octets)
	while(UDPGetTxCount(s) < 300u)
		UDPPut(s, 0); 

	// Make sure we advertise a 0.0.0.0 IP address so all DHCP servers will respond.  If we have a static IP outside the DHCP server's scope, it may simply ignore discover messages.
	if(!bRenewing)
    {
        UDPSetSourceIPAddress(s, IP_ADDRESS_TYPE_IPV4, (IP_MULTI_ADDRESS*)&zeroIP);
    }
	UDPFlush(s);
}

void DHCPConnectionHandler(NET_CONFIG* pNetIf, TCPIP_EVENT connEvent)
{
    int netIx = pNetIf->netIfIx;
    DHCP_CLIENT_VARS* pClient = DHCPClients + netIx;
    
    if(connEvent & TCPIP_EV_CONN_LOST)
    {
        if(pClient->flags.bDHCPEnabled != 0)
        {   // let it wait for the connection
            _DHCPClose(pClient, false);
            _TCPIPStackSetDefaultAddress(pNetIf);
        }
    }
    else if(connEvent & TCPIP_EV_CONN_ESTABLISHED)
    {
        if(pClient->flags.bDHCPEnabled != 0)
        {   // put it in wait connection mode
            // should be in this state anyway
            // but just in case we've missed the link down event
            _DHCPEnable(pNetIf, true);
        }
    }

    
}

// Register an DHCP event handler
// Use hNet == 0 to register on all interfaces available
// Returns a valid handle if the call succeeds,
// or a null handle if the call failed.
// Function has to be called after the DHCP is initialized
// The hParam is passed by the client and will be used by the DHCP when the notification is made.
// It is used for per-thread content or if more modules, for example, share the same handler
// and need a way to differentiate the callback.
DHCP_HANDLE DHCPRegisterHandler(TCPIP_NET_HANDLE hNet, DHCP_EVENT_HANDLER handler, const void* hParam)
{
    if(dhcpMemH)
    {
        DHCP_LIST_NODE* newNode = TCPIP_HEAP_Malloc(dhcpMemH, sizeof(*newNode));

        if(newNode)
        {
            newNode->handler = handler;
            newNode->hParam = hParam;
            newNode->hNet = hNet;
            SingleListAddTail(&dhcpRegisteredUsers, (SGL_LIST_NODE*)newNode);
            return newNode;
        }
    }

    return 0;
}

// deregister the event handler
bool DHCPDeRegisterHandler(DHCP_HANDLE hDhcp)
{
    if(hDhcp && dhcpMemH)
    {
        if(SingleListRemoveNode(&dhcpRegisteredUsers, (SGL_LIST_NODE*)hDhcp))
        {
            TCPIP_HEAP_Free(dhcpMemH, hDhcp);
            return true;
        }
    }

    return false;
}

static void _DHCPNotifyClients(NET_CONFIG* pNetIf, DHCP_EVENT_TYPE evType)
{
    DHCP_LIST_NODE* dNode;

    for(dNode = (DHCP_LIST_NODE*)dhcpRegisteredUsers.head; dNode != 0; dNode = dNode->next)
    {
        if(dNode->hNet == 0 || dNode->hNet == pNetIf)
        {   // trigger event
            (*dNode->handler)(pNetIf, evType, dNode->hParam);
        }
    }
    
}

#endif	//#if defined(TCPIP_STACK_USE_DHCP_CLIENT)
