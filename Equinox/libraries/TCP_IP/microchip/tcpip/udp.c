/*******************************************************************************
  User Datagram Protocol (UDP) Communications Layer

  Summary:
    Module for Microchip TCP/IP Stack
    
  Description:
    -Provides unreliable, minimum latency transport of application 
     datagram (packet) oriented data
    -Reference: RFC 768
*******************************************************************************/

/*******************************************************************************
FileName:   UDP.c
Copyright 2012 released Microchip Technology Inc.  All rights
reserved.

Microchip licenses to you the right to use, modify, copy and distribute
Software only when embedded on a Microchip microcontroller or digital signal
controller that is integrated into your product or third party product
(pursuant to the sublicense terms in the accompanying license agreement).

You should refer to the license agreement accompanying this Software for
additional information regarding your rights and obligations.

SOFTWARE AND DOCUMENTATION ARE PROVIDED “AS IS?WITHOUT WARRANTY OF ANY KIND,
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

#define __UDP_C

#include "tcpip_private.h"
#include "udp_private.h"

#if defined(TCPIP_STACK_USE_UDP)


/****************************************************************************
  Section:
	UDP Global Variables
  ***************************************************************************/
 
typedef struct
{
    UDP_SOCKET		activeUDPSocket;    // Indicates which UDP socket is currently active
    uint16_t            UDPRxCount;         // Number of bytes read from this UDP segment
    uint16_t            wGetOffset;         // Offset from beginning of payload from where data is to be read.
    UDP_SOCKET      LastPutSocket;  	// Indicates the last socket to which data was written

    // Stores various flags for the UDP module
    struct
    {
        unsigned char bFirstRead    : 1;		// No data has been read from this segment yet
        unsigned char bWasDiscarded : 1;	    // The data in this segment has been discarded
    } Flags;

    
    UDP_SOCKET      SocketWithRxData;    // Indicates which socket has currently received data for this loop
}UDP_DCPT;  // UDP layer descriptor

// Store an array of information pertaining to each UDP socket
// Need a descriptor per network interface
// use dynamic allocation instead of arrays
/*static*/ UDP_SOCKET_DCPT* UDPSocketDcpt = 0; 
static UDP_DCPT*        udpDcpt = 0;
static const void*      udpMemH = 0;        // memory handle

// last interface we're working with
// since there are functions without a socket parameter
// (UDPPut, UDPFlush, UDPGet, UDPDiscard)
// there's no way to detect what socket we refer to, hence what interface!
static int      lastUdpIx;  
static int      nUdpSockets;             // number of sockets in the current UDP configuration
static int        udpInitCount = 0;                 // initialization counter

/****************************************************************************
  Section:
	Function Prototypes
  ***************************************************************************/

static UDP_SOCKET FindMatchingSocket(NET_CONFIG* pPktIf, UDP_HEADER *h, void * remoteIP, void * localIP, MAC_ADDR * remoteMACAddr, IP_ADDRESS_TYPE addressType);

static UDP_PORT         UdpAllocateEphemeralPort(void);
static bool             UdpIsAvailablePort(UDP_PORT port);


/*static __inline__*/static  void /*__attribute__((always_inline))*/ _UdpSocketBind(UDP_SOCKET_DCPT* pSkt, NET_CONFIG* pNet)
{
    pSkt->pSktNet = pNet;
    pSkt->pTxPkt->netIf = pNet;
}

/*static __inline__*/static  void /*__attribute__((always_inline))*/ _UdpSocketInit(UDP_SOCKET_DCPT* pSkt)
{
	pSkt->localPort = INVALID_UDP_PORT;
	pSkt->remoteHost = 0x00000000;
	pSkt->smState = UDP_CLOSED;
	pSkt->pSktNet = 0;
    pSkt->pTxPkt = NULL;
}

// returns the associated socket descriptor, if such a socket is valid
// chkNotClosed also checks that the socket is not closed
/*static __inline__*/static  UDP_SOCKET_DCPT* /*__attribute__((always_inline))*/ _UdpSocketDcpt(UDP_SOCKET s, bool chkNotClosed)
{
    UDP_SOCKET_DCPT*    pSkt;
    
	if(s < 0 || s >= nUdpSockets)
    {
       return 0;
    }

    pSkt = UDPSocketDcpt + s;
    if(chkNotClosed && pSkt->smState == UDP_CLOSED)
    {
        return 0;
    }

    return pSkt;
    
}

/****************************************************************************
  Section:
	Connection Management Functions
  ***************************************************************************/

/*****************************************************************************
  Function:
	void UDPInit(const TCPIP_STACK_MODULE_CTRL* const stackCtrl, const UDP_MODULE_CONFIG* pUdpInit)

  Summary:
	Initializes the UDP module.

  Description:
	Initializes the UDP module.  This function initializes all the UDP 
	sockets to the closed state.

  Precondition:
	If passed as a parameter, the used stack heap should be initialized

  Parameters:
    stackCtrl  - pointer to a Stack initialization structure

    pUdpInit    - pointer to a UDP initialization structure containing:
                    - nSockets:     number of sockets to be created

                    - pUdpSktInit:     pointer to an initialization structure
                                    if !NULL, it's an array of nSockets initialization structures 

    NOTE: if pUdpSktInit != 0, then it must contain at least nSockets initialization structures
          Excess initialization structures are ingored.

          If pUdpSktInit == 0, then default values will be used
    

  Returns:
  	true if success,
    false otherwise
  	
  Remarks:
	This function is called only once per interface
    but it actually performs initialization just once
  ***************************************************************************/
bool UDPInit(const TCPIP_STACK_MODULE_CTRL* const stackCtrl, const UDP_MODULE_CONFIG* const pUdpInit)
{
    UDP_SOCKET s;
    int netIx;
    
    if(stackCtrl->stackAction == TCPIP_STACK_ACTION_IF_UP)
    {   // interface start up
        return true;    // do not store per interface data
    }
    
    // stack start up
    if(udpInitCount != 0)
    {   // initialize just once
        udpInitCount++;
        return true;
    }
    
    if(stackCtrl->memH == 0)
    {
        SYS_ERROR(SYS_ERROR_ERROR, "UDP NULL dynamic allocation handle");
        return false;
    }

    UDPSocketDcpt = (UDP_SOCKET_DCPT*)TCPIP_HEAP_Malloc(stackCtrl->memH, sizeof(UDP_SOCKET_DCPT) * pUdpInit->nSockets);
    udpDcpt = (UDP_DCPT*)TCPIP_HEAP_Malloc(stackCtrl->memH, sizeof(UDP_DCPT) * stackCtrl->nIfs);
    if(UDPSocketDcpt == 0 || udpDcpt == 0)
    {
        SYS_ERROR(SYS_ERROR_ERROR, "UDP Dynamic allocation failed");
        TCPIP_HEAP_Free(stackCtrl->memH, UDPSocketDcpt);
        TCPIP_HEAP_Free(stackCtrl->memH, udpDcpt);
        UDPSocketDcpt = 0;
        udpDcpt = 0;
        return false;
    }
    udpMemH = stackCtrl->memH;

    nUdpSockets = pUdpInit->nSockets;

    for ( s = 0; s < nUdpSockets; s++ )
    {
        _UdpSocketInit(UDPSocketDcpt +s);
    }

    for(netIx=0; netIx < stackCtrl->nIfs; netIx++)
    {
        udpDcpt[netIx].activeUDPSocket = udpDcpt[netIx].LastPutSocket = udpDcpt[netIx].SocketWithRxData = INVALID_UDP_SOCKET;
        udpDcpt[netIx].Flags.bWasDiscarded = 1;
        udpDcpt[netIx].Flags.bFirstRead = 0;
    }
    lastUdpIx = -1;

    udpInitCount++;
    return true;
}


/*****************************************************************************
  Function:
	void UDPDeInit(const TCPIP_STACK_MODULE_CTRL* const stackCtrl)

  Summary:
	De-Initializes the UDP module.

  Description:
	De-Initializes the UDP module.
    This function initializes each socket to the CLOSED state.
    If dynamic memory was allocated for the UDP sockets, the function
	will deallocate it.

  Precondition:
	UDPInit() should have been called

  Parameters:
	stackCtrl   - pointer to Stack data showing which interface is closing

  Returns:
    None
    
  Remarks:
  ***************************************************************************/
void UDPDeInit(const TCPIP_STACK_MODULE_CTRL* const stackCtrl)
{
    int ix;

    // interface is going down
    for(ix = 0; ix < nUdpSockets; ix++)
    {
        if(UDPSocketDcpt[ix].pSktNet == stackCtrl->pNetIf)    
        {   // close on this specific interface
            UDPClose(ix);
        }
    }

    if(stackCtrl->stackAction == TCPIP_STACK_ACTION_DEINIT)
    {   // stack shut down
        if(udpInitCount > 0)
        {   // we're up and running

            if(--udpInitCount == 0)
            {   // all closed
                // release resources


                if(UDPSocketDcpt)
                {
                    TCPIP_HEAP_Free(udpMemH, UDPSocketDcpt);
                    UDPSocketDcpt = 0;
                }
                if(udpDcpt)
                {
                    TCPIP_HEAP_Free(udpMemH, udpDcpt);
                    udpDcpt = 0;
                }

                nUdpSockets = 0;
            }
        }
    }
}

/*****************************************************************************
  Function:
	 UDP_SOCKET UDPOpenServer(IP_ADDRESS_TYPE addType, UDP_PORT localPort,  IP_MULTI_ADDRESS* localAddress)

  Summary:
	Opens a UDP socket as a server.
	
  Description:
	Provides a unified method for opening UDP server sockets. 

  Precondition:
    UDP is initialized.

  Parameters:
    IP_ADDRESS_TYPE addType			-	The type of address being used. Example: IP_ADDRESS_TYPE_IPV4.
    UDP_PORT localPort				-	UDP port to listen on or connect to:
										* Server sockets &#45; the local UDP port on which to
										  isten for connections.
    IP_MULTI_ADDRESS* localAddress	-	Local address to use.
	
  Returns:
 	INVALID_SOCKET -  No sockets of the specified type were available to be
                      opened.
    Otherwise -       A UDP_SOCKET handle. Save this handle and use it when
                      calling all other UDP APIs. 
 ***************************************************************************/
UDP_SOCKET UDPOpenServer(IP_ADDRESS_TYPE addType, UDP_PORT localPort,  IP_MULTI_ADDRESS* localAddress)
{
    UDP_SOCKET  skt;
    NET_CONFIG* pDefIf;
   
   if(addType == IP_ADDRESS_TYPE_IPV4 && localAddress != 0 && localAddress->v4Add.Val != 0)
   {
       pDefIf = _TCPIPStackIpAddToNet(&localAddress->v4Add, false);
       if(pDefIf == 0)
       {    // no such interface
           return INVALID_UDP_SOCKET;
       }
   }
   else
   {
       pDefIf =  (NET_CONFIG*)TCPIP_STACK_GetDefaultNet();
   }

    
    skt = UDPOpen(0, UDP_OPEN_SERVER, localPort, 0);
    if(skt != INVALID_UDP_SOCKET)
    {
        _UdpSocketBind(UDPSocketDcpt + skt, pDefIf);
    }

    return skt;
}

/*****************************************************************************
  Function:
	 UDPOpenClient(IP_ADDRESS_TYPE addType, UDP_PORT remotePort, IP_MULTI_ADDRESS* remoteAddress)

  Summary:
	Opens a UDP socket as a client.
	
  Description:
	Provides a unified method for opening UDP client sockets. 

  Precondition:
    UDP is initialized.

  Parameters:
    IP_ADDRESS_TYPE addType			-	The type of address being used. Example: IP_ADDRESS_TYPE_IPV4.
    UDP_PORT remotePort				-	UDP port to listen on or connect to:
										* Client sockets &#45; the remote UDP port to which a
                         				  connection should be made. The local port for client
                         				  sockets will be automatically picked by the UDP
                         				  module.
    IP_MULTI_ADDRESS* remoteAddress	-	The remote address to be used.
	
  Returns:
 	INVALID_SOCKET -  No sockets of the specified type were available to be
                      opened.
    Otherwise -       A UDP_SOCKET handle. Save this handle and use it when
                      calling all other UDP APIs. 
 ***************************************************************************/
UDP_SOCKET UDPOpenClient(IP_ADDRESS_TYPE addType, UDP_PORT remotePort, IP_MULTI_ADDRESS* remoteAddress)
{
#if defined (TCPIP_STACK_USE_IPV6)
    if(addType == IP_ADDRESS_TYPE_IPV6)
    {
        return UDPOpen((uint32_t)&remoteAddress->v6Add, UDP_OPEN_IPV6_ADDRESS, 0, remotePort);
    }
#endif

    IPV4_ADDR v4Add;
    v4Add.Val = remoteAddress?remoteAddress->v4Add.Val:0;
    return UDPOpen(v4Add.Val, UDP_OPEN_IP_ADDRESS, 0, remotePort); 

}
/*****************************************************************************
Function:
	UDP_SOCKET UDPOpen(uint32_t remoteHost, uint8_t remoteHostType, UDP_PORT localPort,
	UDP_PORT remotePort)

 Summary:
    Opens a UDP socket for a client.

 Description:
    Provides a unified method for opening UDP sockets. This function can open both client and 
    server   sockets. For client sockets, it can accept a host name string to query in DNS, an IP
    address as a string, an IP address in binary form, or a previously resolved NODE_INFO 
    structure containing the remote IP address and associated MAC address. When a host name
    or IP address only is provided, UDP module will internally perform the necessary DNSResolve
    and/or ARP resolution steps before reporting that the UDP socket is connected (via a call to
    UDPISOpen returning true). Server sockets ignore this destination parameter and listen 
    only on the indicated port.	Sockets are statically allocated on boot, but can be claimed with
    this function and freed using UDPClose .

Conditions:
UDPInit should be called.

Input:
    remoteHost -     Pointer to remote node info (MAC and IP address) for this connection.  
    If this is a     server socket (receives the first packet) or the destination is the broadcast 
    address, then this parameter should be NULL. For client sockets only. Provide a pointer to
    a null-terminated string of the remote host name (ex\:"www.microchip.com" or 
    "192.168.1.123"), a literal destination IP address (ex\: 0x7B01A8C0 or an IP_ADDR data type),
    or a pointer to a NODE_INFO structure with the remote IP address and remote node or gateway
    MAC address specified, If a string is provided.
    
    remoteHostType - Any one of the following flags to identify the meaning of the remoteHost parameter:
    	- UDP_OPEN_SERVER   = Open a server socket and ignore the remoteHost parameter. 
    		(e.g. - SNMP agent, DHCP server, Announce)
    	- UDP_OPEN_IP_ADDRESS = Open a client socket and connect it to a remote IP address.
    		Ex: 0x7B01A8C0 for 192.168.1.123 (uint32_t type). Note that the byte ordering is big endian.
    	- UDP_OPEN_NODE_INFO = Open a client socket and connect it to a remote IP and MAC 
    		addresses pair stored in a NODE_INFO structure.
     	- UDP_OPEN_RAM_HOST = Open a client socket and connect it to a remote host who's 
     		name is stored as a null terminated string in a RAM array. Ex:"www.microchip.com" or
     		"192.168.0.123"
     	- UDP_OPEN_ROM_HOST = Open a client socket and connect it to a remote host who's
     		name is stored as a null terminated string in a literal string or const array. Ex: 
     		"www.microchip.com" or "192.168.0.123" 

    localPort - UDP port number to listen on.  If 0, stack will dynamically assign a unique port 
    number to use.

    remotePort - For client sockets, the remote port number.

Return Values:
  	Success - 
		A UDP socket handle that can be used for subsequent UDP API calls.
	Failure -
		INVALID_UDP_SOCKET.  This function fails when no more UDP socket
		handles are available.  Increase UDP_MAX_SOCKETS to make more sockets 	available.
Remarks:
	When finished using the UDP socket handle, call the UDPClose() function to free the 
	socket and delete the handle.

*****************************************************************************/
UDP_SOCKET UDPOpen(uint32_t remoteHost, uint8_t remoteHostType, UDP_PORT localPort, UDP_PORT remotePort)
{
    UDP_SOCKET s;
    UDP_SOCKET_DCPT *p;
    int        udpIx;

    if(localPort == 0)
    {
        localPort = UdpAllocateEphemeralPort();
    }

    if(localPort == 0)
    {   // could'nt allocate a new port
        return INVALID_UDP_SOCKET;
    }


    p = UDPSocketDcpt;
    for ( s = 0; s < nUdpSockets; s++ )
    {
        if(p->localPort == INVALID_UDP_PORT)
        {
			p->localPort = localPort;	

            // the server should accept a request on any incoming
            // interface unless is BOUND! 
            p->pSktNet = (NET_CONFIG*)TCPIP_STACK_GetDefaultNet();

            if (remoteHostType == UDP_OPEN_IPV6_ADDRESS)
                p->pTxPkt = UDPAllocateTxPacketStruct (p->pSktNet, IP_ADDRESS_TYPE_IPV6);
            else
                p->pTxPkt = UDPAllocateTxPacketStruct (p->pSktNet, IP_ADDRESS_TYPE_IPV4);

            if (p->pTxPkt == NULL)
            {
                p->localPort = INVALID_UDP_SOCKET;
                return INVALID_UDP_SOCKET;
            }

			if((remoteHostType == UDP_OPEN_SERVER) || (remoteHost == 0))
			{
				//Set remote node as 0xFF ( broadcast address)
				// else Set broadcast address
#if defined (TCPIP_STACK_USE_IPV6)
                if (remoteHostType == UDP_OPEN_IPV6_ADDRESS)
                {
                    TCPIP_IPV6_SetDestAddress(p->pTxPkt,(IPV6_ADDR *)&IPV6_FIXED_ADDR_ALL_NODES_MULTICAST);
                    p->pTxPkt->remoteMACAddr.v[0] = 0x33;
                    p->pTxPkt->remoteMACAddr.v[0] = 0x33;
                    p->pTxPkt->remoteMACAddr.v[0] = 0x00;
                    p->pTxPkt->remoteMACAddr.v[0] = 0x00;
                    p->pTxPkt->remoteMACAddr.v[0] = 0x00;
                    p->pTxPkt->remoteMACAddr.v[0] = 0x01;
                }
                else
#endif
                {
                    TCPIP_IPV4_SetDestAddress(p->pTxPkt,0xFFFFFFFF);
                    memset((void*)&p->pTxPkt->remoteMACAddr, 0xFF, sizeof(p->pTxPkt->remoteMACAddr));
                }
                p->smState = UDP_OPENED;
                // bind the socket on a default interface
                  _UdpSocketBind(p, 0);
            }
            else
			{
				switch(remoteHostType)
				{
#if defined (TCPIP_STACK_CLIENT_MODE)
#if defined (TCPIP_STACK_USE_DNS)
					case UDP_OPEN_RAM_HOST:
					case UDP_OPEN_ROM_HOST:
						//set the UDP state to UDP_GET_DNS_MODULE
						p->remoteHost = remoteHost;
						p->flags.bRemoteHostIsROM = (remoteHostType == UDP_OPEN_ROM_HOST);
						p->smState = UDP_DNS_RESOLVE_IPV4;
					break;
#endif
					case UDP_OPEN_IP_ADDRESS:
    					// remoteHost is a literal IP address.	This doesn't need DNS and can skip directly to the Gateway ARPing step. 	
                        TCPIP_IPV4_SetDestAddress(p->pTxPkt, remoteHost);
                        if(remoteHost == 0)
                        {
                            p->smState = UDP_OPENED;
                        }
                        else
                        {
                            p->retryCount = 0;
                            p->retryInterval = (SYS_TICK_TicksPerSecondGet()/4);
                            p->smState = UDP_GATEWAY_SEND_ARP;
                        }
    					break;
#if defined (TCPIP_STACK_USE_IPV6)
                    case UDP_OPEN_IPV6_ADDRESS:
                        TCPIP_IPV6_SetDestAddress (p->pTxPkt, (IPV6_ADDR *)remoteHost);
                        p->retryCount = 0;
                        p->retryInterval = (SYS_TICK_TicksPerSecondGet()/4);
                        p->smState = UDP_OPENED;
                        break;
#endif
#endif						
					case UDP_OPEN_NODE_INFO:
					//skip DNS and ARP resolution steps if connecting to a remote node which we've already
                        TCPIP_IPV4_SetDestAddress(p->pTxPkt,((NODE_INFO *)(PTR_BASE)remoteHost)->IPAddr.Val);
						memcpy((void*)(uint8_t*)&p->pTxPkt->remoteMACAddr,(void*)&(((NODE_INFO *)(PTR_BASE)remoteHost)->MACAddr),sizeof(p->pTxPkt->remoteMACAddr));
						p->smState = UDP_OPENED;
        			break;
					default:
						break;
				}
			}
			p->remotePort   = remotePort;

			// Mark this socket as active.
			// Once an active socket is set, subsequent operation can be
			// done without explicitely supply socket identifier.
            if(p->pSktNet != 0)
            {
                udpIx = _TCPIPStackNetIx(p->pSktNet);
                udpDcpt[udpIx].activeUDPSocket = s;
                lastUdpIx = udpIx;
            }
			return s;
		}
		p++;
	}

	return (UDP_SOCKET)INVALID_UDP_SOCKET;

}

IP_PACKET * UDPAllocateTxPacketStruct (NET_CONFIG * pNetIf, IP_ADDRESS_TYPE addressType)
{
    IP_PACKET * pkt;
#if !defined (UDP_USE_TX_CHECKSUM) && !defined (TCPIP_STACK_USE_IPV6)
    UDP_HEADER * h;
#endif

    pkt = TCPIP_IP_AllocateTxPacket (pNetIf, addressType, 0, 0);

    if (pkt == NULL)
        return NULL;

    #if defined (UDP_USE_TX_CHECKSUM) || defined (TCPIP_STACK_USE_IPV6)
        if (TCPIP_IP_PutUpperLayerHeader (pkt, NULL, sizeof (UDP_HEADER), IP_PROT_UDP, UDP_CHECKSUM_OFFSET) == NULL)
        {
            TCPIP_IP_FreePacket (pkt);
            return NULL;
        }
    #else
        if (TCPIP_IP_PutUpperLayerHeader (pkt, NULL, sizeof (UDP_HEADER), IP_PROT_UDP, IP_NO_UPPER_LAYER_CHECKSUM) == NULL)
        {
            TCPIP_IP_FreePacket (pkt);
            return NULL;
        }
        h = TCPIP_IP_GetUpperLayerHeader (pkt);
        h->Checksum = 0x0000;
    #endif

    return pkt;
}

/*****************************************************************************
  Function:
	void UDPTask(NET_CONFIG* pNetIf)

  Summary:
	Performs state management and housekeeping for UDP.
	
  Description:
	Performs state management and housekeeping for UDP.  This is an internal
	function meant to be called by StackTask() (not a user API).

  Precondition:
	None

  Parameters:
	pNetIf - pointer to the network configuration

  Return Values:
  	None
  	
  Remarks:
	UDPTask() is called once per StackTask() iteration to ensure that calls 
	to UDPIsPutReady() always update the Ethernet Write pointer location 
	between StackTask() iterations.
  ***************************************************************************/
void UDPTask(NET_CONFIG* pNetIf)
{
	UDP_SOCKET ss;
    UDP_SOCKET_DCPT* pSkt;
    
	for (pSkt = UDPSocketDcpt, ss = 0; ss < nUdpSockets; ss++, pSkt++ )
	{

		if((pSkt->smState == UDP_OPENED) ||
			(pSkt->smState == UDP_CLOSED) || pSkt->pSktNet != pNetIf)
			continue;
		// A timeout has occured.  Respond to this timeout condition
		// depending on what state this socket is in.
		switch(pSkt->smState)
		{
			#if defined(TCPIP_STACK_CLIENT_MODE)
			#if defined(TCPIP_STACK_USE_DNS)
#if defined (TCPIP_STACK_USE_IPV6)
            case UDP_DNS_RESOLVE_IPV6:
                if(DNSBeginUsage(pNetIf) == DNS_RES_OK)
    			{
      				// call DNS Resolve function and move to UDP next State machine
    				pSkt->smState = UDP_DNS_IS_RESOLVED_IPV6;
    				DNSResolve((const char *)(PTR_BASE)pSkt->remoteHost, DNS_TYPE_AAAA);
    			}
                break;
            case UDP_DNS_IS_RESOLVED_IPV6:
    		{
    		    IPV6_ADDR ipResolvedDNSIP;
                DNS_RESULT  dnsRes;
                
				// See if DNS resolution has finished.
                dnsRes	 = DNSIsResolved((const char*)(PTR_BASE)pSkt->remoteHost, &ipResolvedDNSIP);
                if(dnsRes == DNS_RES_PENDING)
                {
                    break;
                }
                
                // Release the DNS usage
                DNSEndUsage(pNetIf);
                               
				if(dnsRes < 0)
				{   // some error occurred
                    pSkt->smState = UDP_DNS_RESOLVE_IPV4;
                }
                else
                {   // DNS_RES_OK
					TCPIP_IPV6_SetDestAddress(pSkt->pTxPkt, &ipResolvedDNSIP);
                    TCPIP_IP_SetPacketIPProtocol (pSkt->pTxPkt, IP_ADDRESS_TYPE_IPV6);
    				pSkt->smState = UDP_OPENED;
				}			
			}
			break;
#endif
			case UDP_DNS_RESOLVE_IPV4:
    			if(DNSBeginUsage(pNetIf) == DNS_RES_OK)
    			{
      				// call DNS Resolve function and move to UDP next State machine
    				pSkt->smState = UDP_DNS_IS_RESOLVED_IPV4;
    				DNSResolve((const char *)(PTR_BASE)pSkt->remoteHost, DNS_TYPE_A);
    			}
    			break;
			case UDP_DNS_IS_RESOLVED_IPV4:
    			{
    				IP_ADDR ipResolvedDNSIP;
                    DNS_RESULT dnsRes;
    				// See if DNS resolution has finished.	Note that if the DNS 
    				// fails, the &ipResolvedDNSIP will be written with 0x00000000. 
    				// MyTCB.remote.dwRemoteHost is unioned with 
    				// MyTCB.remote.niRemoteMACIP.IPAddr, so we can't directly write 
    				// the DNS result into MyTCB.remote.niRemoteMACIP.IPAddr.  We 
    				// must copy it over only if the DNS is resolution step was 
    				// successful.
                    dnsRes = DNSIsResolved((const char *)(PTR_BASE)pSkt->remoteHost, &ipResolvedDNSIP);
                    if (dnsRes == DNS_RES_PENDING)
                    {
                        break;
                    }

                    // Release the DNS Usage
                    DNSEndUsage (pNetIf);

                    if (dnsRes < 0)
                    {
                        // some error occured
#if defined (TCPIP_STACK_USE_IPV6)
                        pSkt->smState = UDP_DNS_RESOLVE_IPV6;
#else
                        pSkt->smState = UDP_DNS_RESOLVE_IPV4;
#endif
                    } 
                    else
                    {   // DNS_RES_OK
                        TCPIP_IPV4_SetDestAddress (pSkt->pTxPkt, ipResolvedDNSIP.Val);
                        TCPIP_IP_SetPacketIPProtocol (pSkt->pTxPkt, IP_ADDRESS_TYPE_IPV4);
                        //pSkt->smState = UDP_OPENED;
						pSkt->smState = UDP_GATEWAY_SEND_ARP;
						pSkt->retryCount = 0;
						pSkt->retryInterval = (SYS_TICK_TicksPerSecondGet()/4);
                    }
    			}
    			break;
#endif // #if defined(TCPIP_STACK_USE_DNS)

			case UDP_GATEWAY_SEND_ARP:
				// Obtain the MAC address associated with the server's IP address 
				//(either direct MAC address on same subnet, or the MAC address of the Gateway machine)
				pSkt->eventTime = SYS_TICK_Get();
				ARPResolve(pNetIf, (IP_ADDR *)&pSkt->pTxPkt->ipHeader.ipv4Header.DestAddress);
				pSkt->smState = UDP_GATEWAY_GET_ARP;
				break;

			case UDP_GATEWAY_GET_ARP:
			if(!ARPIsResolved(pNetIf, (IP_ADDR *)&pSkt->pTxPkt->ipHeader.ipv4Header.DestAddress,&(pSkt->pTxPkt->remoteMACAddr)))
			{
				// Time out if too much time is spent in this state
				// Note that this will continuously send out ARP 
				// requests for an infinite time if the Gateway 
				// never responds
				if(SYS_TICK_Get() - pSkt->eventTime> pSkt->retryInterval)
				{
					// Exponentially increase timeout until we reach 6 attempts then stay constant
					if(pSkt->retryCount < 6u)
					{
						pSkt->retryCount++;
						pSkt->retryInterval <<= 1;
					}
					// Retransmit ARP request
					pSkt->smState = UDP_GATEWAY_SEND_ARP;
				}				
			}
			else
			{
				pSkt->smState = UDP_OPENED;
			}
			break;
			default:
			case UDP_OPENED:
			case UDP_CLOSED:
			    // not used
			break;
#endif // #if defined(TCPIP_STACK_CLIENT_MODE)
		}
	}
} 

/******************************************************************************
  Function:
	  bool UDPISOpened(UDP_SOCKET s)
  
 Summary:
	  Determines if a socket has an established connection.

 Description:
	This function determines if a socket has an established connection to a remote node .  
	Call this function after calling UDPOpen to determine when the connection is set up 
	and ready for use.  

 Precondition:
	UDP is initialized.

 Parameters:
	s - The socket to check.

 Return Values:
	true - The socket has been opened and ARP has been resolved.
	false - The socket is not currently connected.

 Remarks:
	None
 *****************************************************************************/
bool UDPIsOpened(UDP_SOCKET s)
{
    UDP_SOCKET_DCPT* pSkt = _UdpSocketDcpt(s, false);

    if(pSkt)
    {
        return pSkt->smState == UDP_OPENED;
    }

    return false;
}




/*****************************************************************************
  Function:
	void UDPClose(UDP_SOCKET s)

  Summary:
	Closes a UDP socket and frees the handle.
	
  Description:
	Closes a UDP socket and frees the handle.  Call this function to release
	a socket and return it to the pool for use by future communications.

  Precondition:
	UDPInit() must have been previously called.

  Parameters:
	s - The socket handle to be released.  If an illegal handle value is 
		provided, the function safely does nothing.

  Returns:
  	None
  	
  Remarks:
	This function does not affect the previously designated active socket.
  ***************************************************************************/
void UDPClose(UDP_SOCKET s)
{
    UDP_SOCKET_DCPT* pSkt = _UdpSocketDcpt(s, true);

    if(pSkt)
    {
        if (pSkt->pTxPkt != NULL)
        {
            TCPIP_IP_FreePacket (pSkt->pTxPkt);
            pSkt->pTxPkt = NULL;
        }
        _UdpSocketInit(pSkt);
    }
}

/*****************************************************************************
  Function:
	bool UDPGetSocketInfo(UDP_SOCKET s, UDP_SOCKET_INFO* pInfo)

  Summary:
	Points handle at socket unfo for socket s
	
  Description:

  Precondition:

  Parameters:
  	UDP_SOCKET s - Socket to obtain info for.
	UDP_SOCKET_INFO* pInfo - pointer to reference info location.

  Returns:
  ***************************************************************************/
bool UDPGetSocketInfo(UDP_SOCKET s, UDP_SOCKET_INFO* pInfo)
{
    UDP_SOCKET_DCPT* pSkt = _UdpSocketDcpt(s, true);

    if(pSkt == 0)
    {
        return false;
    }


#if defined (TCPIP_STACK_USE_IPV6)
    if (pSkt->pTxPkt->flags.addressType == IP_ADDRESS_TYPE_IPV6)
    {
    	memcpy(&pInfo->remoteIPaddress.v6Add.v, (void*)TCPIP_IPV6_GetDestAddress(pSkt->pTxPkt), sizeof(IPV6_ADDR));
    	memcpy(&pInfo->localIPaddress.v6Add.v, (void*)TCPIP_IPV6_GetSourceAddress(pSkt->pTxPkt), sizeof(IPV6_ADDR));
        pInfo->addressType = IP_ADDRESS_TYPE_IPV6;
    }
    else
#endif
    {
        pInfo->remoteIPaddress.v4Add = TCPIP_IPV4_GetDestAddress(pSkt->pTxPkt);
        pInfo->localIPaddress.v4Add = TCPIP_IPV4_GetSourceAddress(pSkt->pTxPkt);
        pInfo->addressType = IP_ADDRESS_TYPE_IPV4;
    }

	pInfo->remotePort = pSkt->remotePort;
	pInfo->localPort = pSkt->localPort;

	return true;

}



/*****************************************************************************
  Function:
	bool UDPSetTxOffset(UDP_SOCKET s, uint16_t wOffset)

  Summary:
	Moves the pointer within the TX buffer.
	
  Description:
	This function allows the write location within the TX buffer to be 
	specified.  Future calls to UDPPut, UDPPutArray, UDPPutString, etc will
	write data from the indicated location.

  Precondition:
	UDPInit() must have been previously called and a socket is currently 
	active.

  Parameters:
    s       - UDP socket handle 
	wOffset - Offset from beginning of UDP packet data payload to place the
		write pointer.

  Returns:
  	true if the offset is a valid one
    false otherwise
  ***************************************************************************/
bool UDPSetTxOffset(UDP_SOCKET s, uint16_t wOffset)
{
    
    UDP_SOCKET_DCPT* pSkt = _UdpSocketDcpt(s, true);
	if(pSkt && pSkt->pSktNet != 0)
    {
        TCPIP_IP_SetTxBuffer(pSkt->pSktNet, wOffset+sizeof(UDP_HEADER));
        return true;
    }

    return false;
//	udpDcpt[_TCPIPStackNetIx(pNetIf)].wPutOffset = wOffset;
}


/*****************************************************************************
  Function:
	void UDPSetRxOffset(UDP_SOCKET s, uint16_t wOffset)

  Summary:
	Moves the pointer within the RX buffer.
	
  Description:
	This function allows the read location within the RX buffer to be 
	specified.  Future calls to UDPGet and UDPGetArray will read data from
	the indicated location forward.

  Precondition:
	UDPInit() must have been previously called and a socket is currently 
	active.

  Parameters:
    s       - UDP socket handle
	wOffset - Offset from beginning of UDP packet data payload to place the
		read pointer.

  Returns:
  	None
  ***************************************************************************/
void UDPSetRxOffset(UDP_SOCKET s, uint16_t wOffset)
{
    UDP_SOCKET_DCPT* pSkt = _UdpSocketDcpt(s, true);

    if(pSkt && pSkt->pSktNet)
    {
        TCPIP_IPV4_SetRxBuffer(pSkt->pSktNet, wOffset+sizeof(UDP_HEADER));
        udpDcpt[_TCPIPStackNetIx(pSkt->pSktNet)].wGetOffset = wOffset;
    }
}



/****************************************************************************
  Section:
	Transmit Functions
  ***************************************************************************/
  
/*****************************************************************************
  Function:
	uint16_t UDPIsPutReady(UDP_SOCKET s)

  Summary:
	Determines how many bytes can be written to the UDP socket.
	
  Description:
	This function determines if bytes can be written to the specified UDP
	socket.  It also prepares the UDP module for writing by setting the 
	indicated socket as the currently active connection.

  Precondition:
	UDPInit() must have been previously called.

  Parameters:
	s - The socket to be made active

  Returns:
  	The number of bytes that can be written to this socket.
  ***************************************************************************/
uint16_t UDPIsPutReady(UDP_SOCKET s)
{
    UDP_DCPT*   pDcpt;
    int         udpIx;
    NET_CONFIG* pSktNet;
    unsigned short txSize = EMAC_TX_BUFF_SIZE;
    IP_PACKET * pkt;

    UDP_SOCKET_DCPT* pSkt = _UdpSocketDcpt(s, true);
    if(pSkt == 0 || (pSktNet = pSkt->pSktNet) == 0)
    {   // non existent, unbound socket
        return 0;
    }

	if(!TCPIP_IP_IsTxReady(pSktNet))
    {
        return 0;
    }

    pkt = pSkt->pTxPkt;

    if (pkt != NULL)
    {
        if (pkt->flags.queued)
        {
            // Try to allocate a new transmit packet
            IP_PACKET * tempPtr = UDPAllocateTxPacketStruct (pSktNet, pkt->flags.addressType);
            if (tempPtr != NULL)
            {
                if (!TCPIP_IP_CopyTxPacketStruct (tempPtr, pkt))
                {
                    TCPIP_IP_FreePacket (tempPtr);
                    return 0;
                }
                pkt->flags.packetBound = false;
                pSkt->pTxPkt = tempPtr;
                pkt = tempPtr;
                UDPResetHeader(TCPIP_IP_GetUpperLayerHeaderPtr(pkt));
            }
            else
            {
                // We couldn't allocate a new packet.  Return 0 until we can 
                // or until a queued packet can be returned to this node.
                return 0;
            }
        }
    }
    else
    {
        // This should only happen if the user has made an inappropriate call to an 
        // unopened socket.
        return 0;
    }


    udpIx = _TCPIPStackNetIx(pSktNet);
    pDcpt = udpDcpt + udpIx;

	if(pDcpt->LastPutSocket != s)
	{
		pDcpt->LastPutSocket = s;
		UDPSetTxOffset(s, 0);
	}

	pDcpt->activeUDPSocket = s;
    lastUdpIx = udpIx;

    txSize -= sizeof (IP_HEADER);
    // If the user hasn't added the upper layer header yet, we need to subtract
    // space for it
    if (TCPIP_IP_GetUpperLayerHeaderLen(pkt) == 0)
        txSize -= sizeof (UDP_HEADER);
    txSize -= TCPIP_IP_GetPayloadLength (pkt);

    return TCPIP_IP_IsTxPutReady (pkt, txSize);
}

/*****************************************************************************
  Function:
	uint16_t UDPIsTxPutReady(UDP_SOCKET s)

  Summary:
	Determines how many bytes can be written to the UDP socket.
	
  Description:
	This function determines if bytes can be written to the specified UDP
	socket.  It also prepares the UDP module for writing by setting the 
	indicated socket as the currently active connection.

  Precondition:
	UDPInit() must have been previously called.

  Parameters:
	s - The socket to be made active
    count - Number of bytes to allocate

  Returns:
  	The number of bytes that can be written to this socket.
  ***************************************************************************/
uint16_t UDPIsTxPutReady(UDP_SOCKET s, unsigned short count)
{
    UDP_DCPT*   pDcpt;
    int         udpIx;
    NET_CONFIG* pSktNet;
    IP_PACKET * pkt;

    UDP_SOCKET_DCPT* pSkt = _UdpSocketDcpt(s, true);
    if( pSkt == 0 || (pSktNet = pSkt->pSktNet) == 0)
    {   // unbound socket
        return 0;
    }

	if(!TCPIP_IP_IsTxReady(pSktNet))
    {
        return 0;
    }

    pkt = pSkt->pTxPkt;

    if (pkt != NULL)
    {
        if (pkt->flags.queued)
        {
            // Try to allocate a new transmit packet
            IP_PACKET * tempPtr = UDPAllocateTxPacketStruct (pSktNet, pkt->flags.addressType);
            if (tempPtr != NULL)
            {
                if (!TCPIP_IP_CopyTxPacketStruct (tempPtr, pkt))
                {
                    TCPIP_IP_FreePacket (tempPtr);
                    return 0;
                }
                pkt->flags.packetBound = false;
                pSkt->pTxPkt = tempPtr;
                pkt = tempPtr;
                UDPResetHeader(TCPIP_IP_GetUpperLayerHeaderPtr(pkt));
            }
            else
            {
                // We couldn't allocate a new packet.  Return 0 until we can 
                // or until a queued packet can be returned to this node.
                return 0;
            }
        }
    }
    else
    {
        // This should only happen if the user has made an inappropriate call to an 
        // unopened socket.
        return 0;
    }


    udpIx = _TCPIPStackNetIx(pSktNet);
    pDcpt = udpDcpt + udpIx;
    
	if(pDcpt->LastPutSocket != s)
	{
		pDcpt->LastPutSocket = s;
		UDPSetTxOffset(s, 0);
	}

	pDcpt->activeUDPSocket = s;
    lastUdpIx = udpIx;

    return TCPIP_IP_IsTxPutReady (pkt, count);
}

/*****************************************************************************
  Function:
	bool UDPPut(UDP_SOCKET s, uint8_t v)

  Summary:
	Writes a byte to the currently active socket.
	
  Description:
	This function writes a single byte to the currently active UDP socket, 
	while incrementing the buffer length.  UDPIsPutReady should be used 
	before calling this function to specify the currently active socket.

  Precondition:
	UDPIsPutReady() was previously called to specify the current socket.

  Parameters:
    s   - UDP socket handle
	v - The byte to be loaded into the transmit buffer.

  Return Values:
  	true - The byte was successfully written to the socket.
  	false - The transmit buffer is already full and so the write failed.
  ***************************************************************************/
bool UDPPut(UDP_SOCKET s, uint8_t v)
{
    UDP_SOCKET_DCPT* pSkt = _UdpSocketDcpt(s, true);
    if(pSkt)
    {
        UDP_DCPT * pDcpt;

        pDcpt = udpDcpt + lastUdpIx;

        return TCPIP_IP_Put (UDPSocketDcpt[pDcpt->activeUDPSocket].pTxPkt, v);
    }

    return false;
}

/*****************************************************************************
  Function:
	uint16_t UDPPutArray(UDP_SOCKET s, const uint8_t *cData, uint16_t wDataLen)

  Summary:
	Writes an array of bytes to the currently active socket.
	
  Description:
	This function writes an array of bytes to the currently active UDP socket, 
	while incrementing the buffer length.  UDPIsPutReady should be used 
	before calling this function to specify the currently active socket.

  Precondition:
	UDPIsPutReady() was previously called to specify the current socket.

  Parameters:
	cData - The array to write to the socket.
	wDateLen - Number of bytes from cData to be written.
	
  Returns:
  	The number of bytes successfully placed in the UDP transmit buffer.  If
  	this value is less than wDataLen, then the buffer became full and the
  	input was truncated.
  ***************************************************************************/
uint16_t UDPPutArray(UDP_SOCKET s, const uint8_t *cData, uint16_t wDataLen)
{
    UDP_SOCKET_DCPT* pSkt = _UdpSocketDcpt(s, true);
    if(pSkt)
    {
        UDP_DCPT * pDcpt;

        pDcpt = udpDcpt + lastUdpIx;

        return TCPIP_IP_PutArray (UDPSocketDcpt[pDcpt->activeUDPSocket].pTxPkt, cData, wDataLen);
    }

    return false;
}


/*****************************************************************************
  Function:
	uint8_t* UDPPutString(UDP_SOCKET s, const uint8_t *strData)

  Summary:
	Writes null-terminated string to the currently active socket.
	
  Description:
	This function writes a null-terminated string to the currently active 
	UDP socket, while incrementing the buffer length.  UDPIsPutReady should 
	be used before calling this function to specify the currently active
	socket.

  Precondition:
	UDPIsPutReady() was previously called to specify the current socket.

  Parameters:
    s     - UDP socket handle
	cData - Pointer to the string to be written to the socket.
	
  Returns:
  	A pointer to the byte following the last byte written.  Note that this
  	is different than the UDPPutArray functions.  If this pointer does not
  	dereference to a NULL byte, then the buffer became full and the input
  	data was truncated.
  ***************************************************************************/
const uint8_t* UDPPutString(UDP_SOCKET s, const uint8_t *strData)
{
    UDP_SOCKET_DCPT* pSkt = _UdpSocketDcpt(s, true);
    if(pSkt)
    {
        return strData + UDPPutArray(s, strData, strlen((char*)strData));
    }

    return 0;
}

/*****************************************************************************
  Function:
	uint16_t UDPFlush(UDP_SOCKET s)

  Summary:
	Transmits all pending data in a UDP socket.
	
  Description:
	This function builds a UDP packet with the pending TX data and marks it 
	for transmission over the network interface.  Since UDP is a frame-based
	protocol, this function must be called before returning to the main
	stack loop whenever any data is written.

  Precondition:
	UDPIsPutReady() was previously called to specify the current socket, and
	data has been written to the socket using the UDPPut family of functions.

  Parameters:
	None
	
  Returns:
  	The number of bytes that currently were in the socket TX buffer
    and have been flushed.

  Remarks:
	Note that unlike TCPFlush, UDPFlush must be called before returning to 
	the main stack loop.  There is no auto transmit for UDP segments.
  ***************************************************************************/
uint16_t UDPFlush(UDP_SOCKET s)
{
    UDP_DCPT *              pDcpt;
    UDP_SOCKET_DCPT *       p;
    uint16_t                    wUDPLength;
    UDP_HEADER *            pUDPHeader;

    UDP_SOCKET_DCPT* pSkt = _UdpSocketDcpt(s, true);
    if(pSkt == 0)
    {
        return 0;
    }
    
    pDcpt = udpDcpt + lastUdpIx;
    p = &UDPSocketDcpt[pDcpt->activeUDPSocket];

    wUDPLength = TCPIP_IP_GetPayloadLength (p->pTxPkt);

    TCPIP_IP_PutHeader(p->pTxPkt, /*&p->remote.remoteIpv4,*/ IP_PROT_UDP);

    pUDPHeader = (UDP_HEADER *)TCPIP_IP_GetUpperLayerHeaderPtr(p->pTxPkt);

    pUDPHeader->SourcePort = swaps(p->localPort);
    pUDPHeader->DestinationPort = swaps (p->remotePort);
    pUDPHeader->Length = swaps (wUDPLength);

    TCPIP_IP_Flush (p->pTxPkt, NULL);

	// Reset packet size counter for the next TX operation
	pDcpt->LastPutSocket = INVALID_UDP_SOCKET;

    if (!TCPIP_IP_IsPacketQueued(p->pTxPkt))
    {
        TCPIP_IP_ResetTransmitPacketState (p->pTxPkt);
        UDPResetHeader(pUDPHeader);
    }
    return wUDPLength;
}

void UDPResetHeader(UDP_HEADER * h)
{
    h->Checksum = 0x0000;
    h->Length = 0x0000;
}

/*****************************************************************************
  Function:
	uint16_t UDPGetTxCount(UDP_SOCKET s)

  Summary:
	Returns the amount of bytes written into the active UDP socket.
	
  Description:
	This function returns the amount of bytes written into the active UDP socket, 

  Precondition:
	UDPIsPutReady() was previously called to specify the current socket.

  Parameters:
	s   - UDP socket handle

  Return Values:
  	number of bytes in the socket
  ***************************************************************************/
uint16_t UDPGetTxCount(UDP_SOCKET s)
{
    UDP_SOCKET_DCPT* pSkt = _UdpSocketDcpt(s, true);
    if(pSkt)
    {
        return TCPIP_IP_GetPayloadLength (UDPSocketDcpt[((UDP_DCPT *)(udpDcpt+lastUdpIx))->activeUDPSocket].pTxPkt);
    }

    return 0;
}

/****************************************************************************
  Section:
	Receive Functions
  ***************************************************************************/

/*****************************************************************************
  Function:
	uint16_t UDPIsGetReady(UDP_SOCKET s)

  Summary:
	Determines how many bytes can be read from the UDP socket.
	
  Description:
	This function determines if bytes can be read from the specified UDP
	socket.  It also prepares the UDP module for reading by setting the 
	indicated socket as the currently active connection.

  Precondition:
	UDPInit() must have been previously called.

  Parameters:
	s - The socket to be made active (which has already been opened or is
		listening)

  Returns:
  	The number of bytes that can be read from this socket.
  ***************************************************************************/
uint16_t UDPIsGetReady(UDP_SOCKET s)
{
    int         udpIx;
    UDP_DCPT*   pDcpt;
    TCPIP_MAC_HANDLE      hMac;
    NET_CONFIG* pSktNet;

    UDP_SOCKET_DCPT* pSkt = _UdpSocketDcpt(s, true);
    
    if(pSkt == 0 || (pSktNet = pSkt->pSktNet) == 0)
    {   // unbound
        return 0;
    }

    hMac = _TCPIPStackNetToMac(pSktNet);
    udpIx = _TCPIPStackNetIx(pSktNet);
    pDcpt = udpDcpt + udpIx;


    pDcpt->activeUDPSocket = s;
    lastUdpIx = udpIx;

	if(pDcpt->SocketWithRxData != s)
		return 0;

    // If this is the very first time we are accessing this packet, 
    // move the read point to the begining of the packet.
    if(pDcpt->Flags.bFirstRead)
    {
        pDcpt->Flags.bFirstRead = 0;
        UDPSetRxOffset(s, 0);
    }

    return pDcpt->UDPRxCount - pDcpt->wGetOffset;
}

/*****************************************************************************
  Function:
	bool UDPGet(UDP_SOCKET s, uint8_t *v)

  Summary:
	Reads a byte from the currently active socket.
	
  Description:
	This function reads a single byte from the currently active UDP socket, 
	while decrementing the remaining buffer length.  UDPIsGetReady should be 
	used before calling this function to specify the currently active socket.

  Precondition:
	UDPIsGetReady() was previously called to specify the current socket.

  Parameters:
    s   - socket handle
	v - The buffer to receive the data being read.

  Return Values:
  	true - A byte was successfully read
  	false - No data remained in the read buffer
  ***************************************************************************/
bool UDPGet(UDP_SOCKET s, uint8_t *v)
{
    TCPIP_MAC_HANDLE      hMac;
    UDP_DCPT*   pDcpt;
    
    UDP_SOCKET_DCPT* pSkt = _UdpSocketDcpt(s, true);
    if(pSkt == 0)
    {
        return false;
    }

    pDcpt = udpDcpt + lastUdpIx;
    hMac= _TCPIPStackNetToMac(UDPSocketDcpt[pDcpt->activeUDPSocket].pSktNet);
    
	// Make sure that there is data to return
    if((pDcpt->wGetOffset >= pDcpt->UDPRxCount) || (pDcpt->SocketWithRxData != pDcpt->activeUDPSocket))
        return false;

    *v = MACGet(hMac);
    pDcpt->wGetOffset++;

    return true;
}


/*****************************************************************************
  Function:
	uint16_t UDPGetArray(UDP_SOCKET s, uint8_t *cData, uint16_t wDataLen)

  Summary:
	Reads an array of bytes from the currently active socket.
	
  Description:
	This function reads an array of bytes from the currently active UDP socket, 
	while decrementing the remaining bytes available. UDPIsGetReady should be 
	used before calling this function to specify the currently active socket.

  Precondition:
	UDPIsGetReady() was previously called to specify the current socket.

  Parameters:
    s     - UDP socket handle
	cData - The buffer to receive the bytes being read.  If NULL, the bytes are 
			simply discarded without being written anywhere (effectively skips 
			over the bytes in the RX buffer, although if you need to skip a lot 
			of data, seeking using the UDPSetRxOffset() will be more efficient).
	wDateLen - Number of bytes to be read from the socket.
	
  Returns:
  	The number of bytes successfully read from the UDP buffer.  If this
  	value is less than wDataLen, then the buffer was emptied and no more 
  	data is available.
  ***************************************************************************/
uint16_t UDPGetArray(UDP_SOCKET s, uint8_t *cData, uint16_t wDataLen)
{
    TCPIP_MAC_HANDLE  hMac;
	uint16_t wBytesAvailable;
    UDP_DCPT*   pDcpt;
    
    UDP_SOCKET_DCPT* pSkt = _UdpSocketDcpt(s, true);
    if(pSkt == 0)
    {
        return 0;
    }

    pDcpt = udpDcpt + lastUdpIx;
	
	// Make sure that there is data to return
    if((pDcpt->wGetOffset >= pDcpt->UDPRxCount) || (pDcpt->SocketWithRxData != pDcpt->activeUDPSocket))
    {
		return 0;
    }

	// Make sure we don't try to read more data than exists
	wBytesAvailable = pDcpt->UDPRxCount - pDcpt->wGetOffset;
	if(wBytesAvailable < wDataLen)
    {
		wDataLen = wBytesAvailable;
    }

    hMac= _TCPIPStackNetToMac(UDPSocketDcpt[pDcpt->activeUDPSocket].pSktNet);
	wDataLen = TCPIP_IP_GetArray(hMac, cData, wDataLen);
    pDcpt->wGetOffset += wDataLen;

    return wDataLen;
}

/*****************************************************************************
  Function:
	void UDPDiscard(UDP_SOCKET s)

  Summary:
	Discards any remaining RX data from a UDP socket.
	
  Description:
	This function discards any remaining received data in the currently 
	active UDP socket.

  Precondition:
	UDPIsGetReady() was previously called to select the currently active
	socket.

  Parameters:
    s   - socket handle
	
  Returns:
  	None

  Remarks:
	It is safe to call this function more than is necessary.  If no data is
	available, this function does nothing.
  ***************************************************************************/
void UDPDiscard(UDP_SOCKET s)
{
    NET_CONFIG* pNetIf;
    
    UDP_SOCKET_DCPT* pSkt = _UdpSocketDcpt(s, true);

    if(pSkt && (pNetIf = pSkt->pSktNet) != 0)
    {   // just in case
        UDP_DCPT* pDcpt = udpDcpt + _TCPIPStackNetIx(pNetIf);
        if(!pDcpt->Flags.bWasDiscarded)
        {
            MACDiscardRx(_TCPIPStackNetToMac(pNetIf));
            pDcpt->UDPRxCount = 0;
            pDcpt->SocketWithRxData = INVALID_UDP_SOCKET;
            pDcpt->Flags.bWasDiscarded = 1;
        }
    }
}

void UDPDiscardNet(NET_CONFIG* pNetIf)
{
    if(pNetIf)
    {   // just in case
        UDP_DCPT* pDcpt = udpDcpt + _TCPIPStackNetIx(pNetIf);
        if(!pDcpt->Flags.bWasDiscarded)
        {
            MACDiscardRx(_TCPIPStackNetToMac(pNetIf));
            pDcpt->UDPRxCount = 0;
            pDcpt->SocketWithRxData = INVALID_UDP_SOCKET;
            pDcpt->Flags.bWasDiscarded = 1;
        }
    }
}


/****************************************************************************
  Section:
	Data Processing Functions
  ***************************************************************************/

/*****************************************************************************
  Function:
	bool UDPProcess(NET_CONFIG* pNetIf, NODE_INFO *remoteNode, IP_ADDR *localIP, uint16_t len)

  Summary:
	Handles an incoming UDP segment.
	
  Description:
	This function handles an incoming UDP segment to determine if it is 
	acceptable and should be handed to one of the stack applications for
	processing.

  Precondition:
	UDPInit() has been called an a UDP segment is ready in the MAC buffer.

  Parameters:
    pNetIf       - interface to use
	remoteNode - The remote node that sent this segment.
	localIP - The destination IP address for this segment.
	len - Total length of the UDP segment.
	
  Return Values:
  	true - A valid packet is waiting and the stack applications should be
  		called to handle it.
  	false - The packet was discarded.
  ***************************************************************************/
bool UDPProcess(NET_CONFIG* pNetIf, NODE_INFO *remoteNode, IP_ADDR *localIP, uint16_t len)
{
    UDP_HEADER		h;
    UDP_SOCKET		s;
    PSEUDO_HEADER	pseudoHeader;
    TCPIP_UINT32_VAL		checksums;
    UDP_DCPT*       pDcpt;
    TCPIP_MAC_HANDLE hMac;

    pDcpt = udpDcpt + _TCPIPStackNetIx(pNetIf);
	pDcpt->UDPRxCount = 0;

    // Retrieve UDP header.
    hMac = _TCPIPStackNetToMac(pNetIf);
    TCPIP_IP_GetArray(hMac, (uint8_t*)&h, sizeof(h));

    h.SourcePort        = swaps(h.SourcePort);
    h.DestinationPort   = swaps(h.DestinationPort);
    h.Length            = swaps(h.Length) - sizeof(UDP_HEADER);

	// See if we need to validate the checksum field (0x0000 is disabled)
#ifdef UDP_USE_RX_CHECKSUM
	if(h.Checksum)
	{
	    // Calculate IP pseudoheader checksum.
	    pseudoHeader.SourceAddress		= remoteNode->IPAddr;
	    pseudoHeader.DestAddress.Val	= localIP->Val;
	    pseudoHeader.Zero				= 0x0;
	    pseudoHeader.Protocol			= IP_PROT_UDP;
	    pseudoHeader.Length				= len;

	    TCPIP_IPV4_SwapPseudoHeader(pseudoHeader);
	
	    checksums.w[0] = ~CalcIPChecksum((uint8_t*)&pseudoHeader,
	                                    sizeof(pseudoHeader));
	
	
	    // Now calculate UDP packet checksum in NIC RAM -- should match pseudoHeader
	    TCPIP_IPV4_SetRxBuffer(pNetIf, 0);
	    checksums.w[1] = MACCalcIPBufferChecksum(hMac, len);
	
	    if(checksums.w[0] != checksums.w[1])
	    {
	        TCPIP_IP_DiscardRx(hMac);
	        return false;
	    }
	}
#endif // UDP_USE_RX_CHECKSUM
    s = FindMatchingSocket(pNetIf, &h, &remoteNode->IPAddr, localIP, &remoteNode->MACAddr, IP_ADDRESS_TYPE_IPV4);
    if(s == INVALID_UDP_SOCKET)
    {
        // If there is no matching socket, There is no one to handle
        // this data.  Discard it.
        TCPIP_IP_DiscardRx(hMac);
		return false;
    }
    else if(pNetIf == UDPSocketDcpt[s].pSktNet)    
    {   // packet from this interface
            
		pDcpt->SocketWithRxData = s;
        pDcpt->UDPRxCount = h.Length;
        pDcpt->Flags.bFirstRead = 1;
		pDcpt->Flags.bWasDiscarded = 0;
        return true;
    }

    return false;   // packet not belonging to this interface

}

#if defined (TCPIP_STACK_USE_IPV6)
bool UDPProcessIPv6(NET_CONFIG * pNetIf, IPV6_ADDR * localIP, IPV6_ADDR * remoteIP, uint16_t dataLen, uint16_t headerLen)
{
    UDP_HEADER          h;
    UDP_SOCKET          s;
    IPV6_PSEUDO_HEADER  pseudoHeader;
    TCPIP_UINT32_VAL           checksums;
    UDP_DCPT*       pDcpt;
    TCPIP_MAC_HANDLE hMac;

    pDcpt = udpDcpt + _TCPIPStackNetIx(pNetIf);
	pDcpt->UDPRxCount = 0;

    // Retrieve UDP header.
    hMac = _TCPIPStackNetToMac(pNetIf);
    TCPIP_IP_GetArray(hMac, (uint8_t*)&h, sizeof(h));

    h.SourcePort        = swaps(h.SourcePort);
    h.DestinationPort   = swaps(h.DestinationPort);
    h.Length            = swaps(h.Length) - sizeof(UDP_HEADER);

    // Calculate checksums
    memcpy (&pseudoHeader.SourceAddress, remoteIP, sizeof (IPV6_ADDR));
    memcpy (&pseudoHeader.DestAddress, localIP, sizeof (IPV6_ADDR));
    // Total payload length is the length of data + extension headers
    pseudoHeader.PacketLength = swaps(dataLen);
    pseudoHeader.zero1 = 0;
    pseudoHeader.zero2 = 0;
    pseudoHeader.NextHeader = IP_PROT_UDP;

    checksums.w[0] = ~CalcIPChecksum((uint8_t*)&pseudoHeader,
                                    sizeof(pseudoHeader));

    TCPIP_IPV6_SetRxBuffer(pNetIf, headerLen);
    checksums.w[1] = MACCalcIPBufferChecksum(hMac, dataLen);

    if(checksums.w[0] != checksums.w[1])
    {
        TCPIP_IP_DiscardRx(hMac);
        return false;
    }

    TCPIP_IPV6_SetRxBuffer(pNetIf, headerLen);

    s = FindMatchingSocket(pNetIf, &h, remoteIP, localIP, NULL, IP_ADDRESS_TYPE_IPV6);
    if(s == INVALID_UDP_SOCKET)
    {
        // Send ICMP Destination Unreachable Code 4 (Port unreachable) and discard packet
        TCPIP_IPV6_SendError (pNetIf, localIP, remoteIP, ICMPV6_ERR_DU_PORT_UNREACHABLE, ICMPV6_ERROR_DEST_UNREACHABLE, 0x00000000, dataLen + headerLen + sizeof (IPV6_HEADER));

        // If there is no matching socket, There is no one to handle
        // this data.  Discard it.
        TCPIP_IP_DiscardRx(hMac);
		return false;
    }
    else
    {
		pDcpt->SocketWithRxData = s;
        pDcpt->UDPRxCount = h.Length;
        pDcpt->Flags.bFirstRead = 1;
		pDcpt->Flags.bWasDiscarded = 0;
    }

    return true;
}
#endif

/*****************************************************************************
  Function:
	static UDP_SOCKET FindMatchingSocket(NET_CONFIG* pPktIf, UDP_HEADER *h, NODE_INFO *remoteNode,
                                     		IP_ADDR *localIP)

  Summary:
	Matches an incoming UDP segment to a currently active socket.
	
  Description:
	This function attempts to match an incoming UDP segment to a currently
	active socket for processing.

  Precondition:
	UDP segment header and IP header have both been retrieved.

  Parameters:
    pPktIf - interface the UDP packet/header belongs to  
	h - The UDP header that was received.
	remoteNode - IP and MAC of the remote node that sent this segment.
	localIP - IP address that this segment was destined for.
	
  Returns:
  	A UDP_SOCKET handle of a matching socket, or INVALID_UDP_SOCKET when no
  	match could be made.
  ***************************************************************************/
static UDP_SOCKET FindMatchingSocket(   NET_CONFIG* pPktIf, UDP_HEADER *h,
                                        void * remoteIP,
                                        void * localIP,
                                        MAC_ADDR * remoteMACAddr,
                                        IP_ADDRESS_TYPE addressType)
{
	UDP_SOCKET s;
	UDP_SOCKET partialMatch;
	UDP_SOCKET_DCPT *p;

// This is commented out because most applications don't need this type of filtering.  It comes at a performance cost.
//	// Filter out unicast packets that aren't for our IP address, but accept 
//	// all multicast and broadcast traffic
//	if(!((localIP->Val == NetConfig.MyIPAddr.Val) || (localIP->v[0] & 0x80) || (localIP->Val == (NetConfig.MyIPAddr.Val | (~NetConfig.MyMask.Val)))))
//		return INVALID_UDP_SOCKET;

	// Discard any packets received that were generated by ourself.  In 
	// structured Wi-Fi networks, the Access Point rebroadcasts our broadcast 
	// and multicast packets, causing self-reception to occur unless filtered 
	// out.
#if defined TCPIP_STACK_USE_IPV6
    if (addressType == IP_ADDRESS_TYPE_IPV6)
    {
        if (TCPIP_IPV6_FindAddress (pPktIf, remoteIP, IPV6_ADDR_TYPE_UNICAST) != NULL) 
    		return INVALID_UDP_SOCKET;
    }
    else
#endif
    {
    	if(_TCPIPStackNetByAddress((IP_ADDR *)remoteIP) != 0)
    		return INVALID_UDP_SOCKET;
    }
            
	partialMatch = INVALID_UDP_SOCKET;

	p = UDPSocketDcpt;
    for(s = 0; s < nUdpSockets; s++, p++)
	{
		// This packet is said to be matching with current socket:
		// 1. If its destination port matches with our local port and
		// 2. Packet source IP address matches with previously saved socket remote IP address and
		// 3. Packet source port number matches with previously saved socket remote port number
		if(p->localPort == h->DestinationPort)
		{
			if(p->remotePort == h->SourcePort)
			{
#if defined(TCPIP_STACK_USE_IPV6)
                if (addressType == IP_ADDRESS_TYPE_IPV6)
                {
                    if (p->pTxPkt->flags.addressType == IP_ADDRESS_TYPE_IPV6)
                    {
                        if(p->pSktNet != 0)
                        {
                            if(TCPIP_IPV6_FindAddress(p->pSktNet, TCPIP_IPV6_GetDestAddress(p->pTxPkt), IPV6_ADDR_TYPE_UNICAST) != 0)
                            {   // perfect match : port + interface

                                return s;
                            }
                        }
                        else
                        {   // incoming interface not specified and we have remote port match
                            partialMatch = s;
                        }
                    }
                }
                else
#endif
                {
                    if (p->pTxPkt->flags.addressType == IP_ADDRESS_TYPE_IPV4)
                    {
                        if(p->pSktNet == pPktIf)
                        {   // perfect match : port + interface
                            return s;
                        }
                        else if(p->pSktNet == 0)
                        {   // incoming interface not specified and we have remote port match
                            partialMatch = s;
                        }
                    }
                }
			}
            else if(p->remotePort == 0)
            {   // can accept requests from any port
                if(p->pSktNet == 0 || p->pSktNet == pPktIf)
                {
                    partialMatch = s;
                }
            }
		}
	}

	if(partialMatch != INVALID_UDP_SOCKET)
    {   // we have a server socket that can handle this packet
        p = UDPSocketDcpt + partialMatch;
        p->pSktNet = pPktIf;    // bind it
        p->remotePort = h->SourcePort;
        
#if defined (TCPIP_STACK_USE_IPV6)
        if (addressType == IP_ADDRESS_TYPE_IPV6)
        {
            TCPIP_IPV6_SetDestAddress(p->pTxPkt,remoteIP);
            TCPIP_IPV6_SetSourceAddress(p->pTxPkt, &(((IPV6_ADDR_STRUCT *)localIP)->address));
            TCPIP_IP_SetPacketIPProtocol (p->pTxPkt, IP_ADDRESS_TYPE_IPV6);
        }
        else
#endif
        {

            TCPIP_IPV4_SetDestAddress(p->pTxPkt,((IP_ADDR *)remoteIP)->Val);
            memcpy((void*)&p->pTxPkt->remoteMACAddr, remoteMACAddr, sizeof(p->pTxPkt->remoteMACAddr) );
            TCPIP_IP_SetPacketIPProtocol (p->pTxPkt, IP_ADDRESS_TYPE_IPV4);
        }

        return s;
    }


	return INVALID_UDP_SOCKET;
}


/*****************************************************************************
  Function:
	void UDPSocketSetNet(UDP_SOCKET s, TCPIP_NET_HANDLE hNet)

  Summary:
	Sets the network interface for an UDP socket
	
  Description:
	This function sets the network interface for an UDP socket

  Precondition:
	UDP socket should have been opened with UDPOpen().
    s - valid socket

  Parameters:
	s - The UDP socket
   	pNet - interface .
	
  Returns:
    None.
  ***************************************************************************/
void UDPSocketSetNet(UDP_SOCKET s, TCPIP_NET_HANDLE hNet)
{
    UDP_SOCKET_DCPT* pSkt = _UdpSocketDcpt(s, true);
    if(pSkt)
    {
        NET_CONFIG* pNet =  (NET_CONFIG*)hNet;
        if(pNet != pSkt->pSktNet)
        { // new interface
            int oldUdpIx, newUdpIx;

            if(pSkt->pSktNet != 0)
            {
                oldUdpIx = _TCPIPStackNetIx(pSkt->pSktNet);
                if(udpDcpt[oldUdpIx].activeUDPSocket == s )
                {   // changing the interface for the active UDP socket
                    udpDcpt[oldUdpIx].activeUDPSocket = INVALID_UDP_SOCKET;
                }
            }
            else
            {
                oldUdpIx = -1;
            }

            newUdpIx = _TCPIPStackNetIx(pNet);
            udpDcpt[newUdpIx].activeUDPSocket = s;
            _UdpSocketBind(pSkt, pNet);
            if(lastUdpIx == oldUdpIx)
            {
                lastUdpIx = newUdpIx;
            } 
        }
    }
}
/*****************************************************************************
  Function:
	TCPIP_MAC_HANDLE UDPSocketGetNet(UDP_SOCKET s)

  Summary:
	Gets the MAC interface of an UDP socket
	
  Description:
	This function returns the MAC interface id of an UDP socket

  Precondition:
	UDP socket should have been opened with UDPOpen().
    s - valid socket

  Parameters:
	s - The UDP socket
	
  Returns:
    IHandle of the interface that socket currently uses.
  ***************************************************************************/
TCPIP_NET_HANDLE UDPSocketGetNet(UDP_SOCKET s)
{
    UDP_SOCKET_DCPT* pSkt = _UdpSocketDcpt(s, true);
    
    return pSkt?pSkt->pSktNet:0;
}

// sets the source IP address of a packet
bool UDPSetSourceIPAddress(UDP_SOCKET s, IP_ADDRESS_TYPE addType, IP_MULTI_ADDRESS* localAddress)
{
    UDP_SOCKET_DCPT* pSkt = _UdpSocketDcpt(s, true);

    if(pSkt == 0 || pSkt->pTxPkt == 0 || pSkt->pTxPkt->flags.addressType != addType)
    {
        return false;
    }


    
#if defined (TCPIP_STACK_USE_IPV6)
    if (pSkt->pTxPkt->flags.addressType == IP_ADDRESS_TYPE_IPV6)
        TCPIP_IPV6_SetSourceAddress(pSkt->pTxPkt, localAddress?&localAddress->v6Add:0);
    else
#endif
    {
        TCPIP_IPV4_SetSourceAddress(pSkt->pTxPkt, localAddress?localAddress->v4Add.Val:0);
    }
    return true;

}

/*****************************************************************************
  Function:
    bool UDPBind(UDP_SOCKET s, IP_ADDRESS_TYPE addType, UDP_PORT localPort,  IP_MULTI_ADDRESS* localAddress);

  Summary:
    Bind a socket to a local address
    This function is meant for client sockets.
    It is similar to UDPSocketSetNet() that assigns a specific source interface for a socket.
    If localPort is 0 the stack will assign a unique local port

  Description:
    Sockets don't need specific binding, it is done automatically by the stack
    However, specific binding can be requested using these functions.
    Works for both client and server sockets.
    TBD: the call should fail if the socket is already bound to an interface
    (a server socket is conencted or a client socket already sent the data on an interface).
    Implementation pending

  Precondition:
	UDP is initialized.

  Parameters:
	s				-	The socket to bind.
	addType			-	The type of address being used. Example: IP_ADDRESS_TYPE_IPV4.
	localPort		-	The local port to bind to.
	localAddress	-   Local address to use.
	
  Returns:
	True if success
	False otherwise

  ***************************************************************************/
bool UDPBind(UDP_SOCKET s, IP_ADDRESS_TYPE addType, UDP_PORT localPort,  IP_MULTI_ADDRESS* localAddress)
{
    NET_CONFIG* pSktIf;
    UDP_SOCKET_DCPT* pSkt;

    if(addType == IP_ADDRESS_TYPE_IPV6)
    {
        return false;
    }

    if((pSkt = _UdpSocketDcpt(s, true)) == 0)
    {
        return false;
    }

    if(localPort == 0)
    {
        if((localPort = UdpAllocateEphemeralPort()) == 0)
        {
            return false;
        }
    }

    if(localAddress && localAddress->v4Add.Val != 0)
    {
        pSktIf = _TCPIPStackIpAddToNet(&localAddress->v4Add, false);
        if(pSktIf == 0)
        {    // no such interface
            return false;
        }
    }
    else
    {
        pSktIf = (NET_CONFIG*)TCPIP_STACK_GetDefaultNet();
    }


    _UdpSocketBind(pSkt, pSktIf);
    pSkt->localPort = localPort;


    return true;
}

/*****************************************************************************
  Function:
    bool UDPBind(UDP_SOCKET s, IP_ADDRESS_TYPE addType, UDP_PORT localPort,  IP_MULTI_ADDRESS* localAddress);

  Summary:
    Bind a socket to a remote address
    This function is meant for server sockets.

  Description:
    Sockets don't need specific binding, it is done automatically by the stack
    However, specific binding can be requested using these functions.
    Works for both client and server sockets.
    TBD: the call should fail if the socket is already bound to an interface
    (a server socket is conencted or a client socket already sent the data on an interface).
    Implementation pending

  Precondition:
	UDP is initialized.

  Parameters:
	s				-	The socket to bind.
	addType			-	The type of address being used. Example: IP_ADDRESS_TYPE_IPV4.
	localPort		-	The local port to bind to.
	localAddress	-   Local address to use.
	
  Returns:
	True if success
	False otherwise

  ***************************************************************************/
bool UDPRemoteBind(UDP_SOCKET s, IP_ADDRESS_TYPE addType, UDP_PORT remotePort,  IP_MULTI_ADDRESS* remoteAddress)
{

    if(UDPSetDestinationIPAddress(s, addType, remoteAddress))
    {
        (UDPSocketDcpt + s)->remotePort = remotePort;
        return true;
    }

    return false;
}

// Allows setting options to a socket like enable broadcast, Rx/Tx buffer size, etc
bool UDPSetOptions(UDP_SOCKET s, int option, int param)
{
    return false;
}


bool UDPSetDestinationIPAddress(UDP_SOCKET s, IP_ADDRESS_TYPE addType, IP_MULTI_ADDRESS* remoteAddress)
{

    NET_CONFIG      *pIf;
    UDP_SOCKET_DCPT *pSkt = _UdpSocketDcpt(s, true);
    

    if(pSkt == 0 || (pIf = pSkt->pSktNet) == 0 || pSkt->pTxPkt == 0 || pSkt->pTxPkt->flags.addressType != addType)
    {
        return false;
    }

#if defined (TCPIP_STACK_USE_IPV6)
    if (pSkt->pTxPkt->flags.addressType == IP_ADDRESS_TYPE_IPV6)
    {
        TCPIP_IPV6_SetDestAddress (pSkt->pTxPkt, remoteAddress?&remoteAddress->v6Add:0);
    }
    else
#endif
    {
        TCPIP_IPV4_SetDestAddress (pSkt->pTxPkt, remoteAddress?remoteAddress->v4Add.Val:0);
    }
    return true;
}

/*****************************************************************************
  Function:
	bool UDPSetBcastIPAddress(UDP_SOCKET s, IP_ADDRESS_TYPE addType, IP_MULTI_ADDRESS* bcastAddress)

  Summary:
	Sets the broadcast IP address of a socket
	Allows an UDP socket to send broadcasts.
	
  Description:
      - for now it checks for broadcast address
        and properly sets the MAC address to 
        ff:ff:ff:ff:ff:ff is this is a valid broadcast
        (this should be decided by the IP layer)
      - To be consistent with the current stack, if
        the destination is not a broadcast, an ARP 
        should be initiated.
        IT iS NOT!
        Use just for broadcast, for now!

  Precondition:
	UDP initialized

  Parameters:
	s				-	the UDP socket
	addType			-	Type of address
	bcastAddress	- 	bcast pointer to store value
	
  Returns:
    True if success
	False otherwisw
  ***************************************************************************/
bool UDPSetBcastIPAddress(UDP_SOCKET s, IP_ADDRESS_TYPE addType, IP_MULTI_ADDRESS* bcastAddress)
{
    NET_CONFIG*         pIf;
    bool                isBcast;
    UDP_SOCKET_DCPT*    pSkt;

    if (addType == IP_ADDRESS_TYPE_IPV6)
    {   
        return false;
    }



    pSkt = _UdpSocketDcpt(s, true);
    if(pSkt == 0 || (pIf = pSkt->pSktNet) == 0 || pSkt->pTxPkt == 0 || pSkt->pTxPkt->flags.addressType != addType)
    {
        return false;
    }

    while(true)
    {

        if( bcastAddress->v4Add.Val == 0xffffffff)
        {   // limited bcast address
            isBcast = true;
            break;
        }

        if( (( bcastAddress->v4Add.Val ^ pIf->MyIPAddr.Val) & pIf->MyMask.Val) == 0)
        {   // my network
            if( bcastAddress->v4Add.Val == (pIf->MyIPAddr.Val | (~pIf->MyMask.Val)) )
            {   // ok, my network directed bcast
                isBcast = true;
                break;
            }
        }


        // else either not my network
        // or another host on my network
        // anyway an ARP has to be done
        // and we don't do it now!
        isBcast = false;
        break;
    }

    if(!isBcast)
    {   // not supported for now
        return false;
    }

    // broadcast address
    TCPIP_IPV4_SetDestAddress (pSkt->pTxPkt, bcastAddress?bcastAddress->v4Add.Val:0);
    memset(&pSkt->pTxPkt->remoteMACAddr, 0xff, sizeof(pSkt->pTxPkt->remoteMACAddr));
    return true;
}


static UDP_PORT UdpAllocateEphemeralPort(void)
{
    int      num_ephemeral;
    int      count;
    UDP_PORT next_ephemeral;


    count = num_ephemeral = UDP_LOCAL_PORT_END_NUMBER - UDP_LOCAL_PORT_START_NUMBER + 1;

    next_ephemeral = UDP_LOCAL_PORT_END_NUMBER + (rand() % num_ephemeral);

    while(count--)
    {
        if(UdpIsAvailablePort(next_ephemeral))
        {
            return next_ephemeral;
        }

        if (next_ephemeral == UDP_LOCAL_PORT_END_NUMBER)
        {
            next_ephemeral = UDP_LOCAL_PORT_START_NUMBER;
        }
        else
        {
            next_ephemeral++;
        }
    }

    return 0;   // not found
}

static bool UdpIsAvailablePort(UDP_PORT port)
{
    UDP_SOCKET skt;
    UDP_SOCKET_DCPT *pSkt;

    // Find an available socket that matches the specified socket type
    for(skt = 0, pSkt = UDPSocketDcpt; skt < nUdpSockets; skt++, pSkt++)
    {
        if(pSkt->smState != UDP_CLOSED) 
        {
            if( pSkt->localPort == port)
            {
                return false;
            }
        }
    }

    return true;
}



#endif //#if defined(TCPIP_STACK_USE_UDP)
