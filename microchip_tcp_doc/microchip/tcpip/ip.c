/*******************************************************************************
  Internet Protocol (IP) Version 4 Communications Layer

  Summary:
    Module for Microchip TCP/IP Stack
    
  Description:
    -Provides a transport for TCP, UDP, and ICMP messages
    -Reference: RFC 791
*******************************************************************************/

/*******************************************************************************
FileName:   IP.c
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

#define __IP_C

#include "tcpip_private.h"
#include "ip_private.h"


// This is left shifted by 4.  Actual value is 0x04.
#define IPv4_VERSION        (0x40u)
// This is left shifted by 4.  Actual value is 0x06.
#define IPv6_VERSION        (0x60u)

// IHL (Internet Header Length) is # of 32 bit words in a header.
// Since, we do not support options, our IP header length will be
// minimum i.e. 20 bytes : IHL = 20 / 4 = 5.
#define IP_IHL              (0x05)

#define IP_SERVICE_NW_CTRL  (0x07)
#define IP_SERVICE_IN_CTRL  (0x06)
#define IP_SERVICE_ECP      (0x05)
#define IP_SERVICE_OVR      (0x04)
#define IP_SERVICE_FLASH    (0x03)
#define IP_SERVICE_IMM      (0x02)
#define IP_SERVICE_PRIOR    (0x01)
#define IP_SERVICE_ROUTINE  (0x00)

#define IP_SERVICE_N_DELAY  (0x00)
#define IP_SERCICE_L_DELAY  (0x08)
#define IP_SERVICE_N_THRPT  (0x00)
#define IP_SERVICE_H_THRPT  (0x10)
#define IP_SERVICE_N_RELIB  (0x00)
#define IP_SERVICE_H_RELIB  (0x20)

#define IP_SERVICE          (IP_SERVICE_ROUTINE | IP_SERVICE_N_DELAY)

#if defined(TCPIP_STACK_USE_ZEROCONF_MDNS_SD)
  #define MY_IP_TTL           (255)  // Time-To-Live in hops 
  // IP TTL is set to 255 for Multicast DNS compatibility. See mDNS-draft-08, section 4.
#else
  #define MY_IP_TTL           (100)  // Time-To-Live in hops
#endif

#if defined(TCPIP_STACK_USE_IPV6)
// The IPv6 unspecified address
const IPV6_ADDR IPV6_FIXED_ADDR_UNSPECIFIED = {{0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00}};
// The IPv6 all-nodes multicast addres
const IPV6_ADDR IPV6_FIXED_ADDR_ALL_NODES_MULTICAST = {{0xFF, 0x02, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01}};
// The IPv6 all-routers multicast address
const IPV6_ADDR IPV6_FIXED_ADDR_ALL_ROUTER_MULTICAST = {{0xFF, 0x02, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x02}};

static SystemTickHandle     ipInitTimerHandle = 0;      // Handle for the IPv6 initialization task timer
static int                  ipInitTickPending = 0;      // Pending flag for the IPv6 intialization task
static int                  ipInitCount = 0;            // Indicator of how many interfaces are initializing the IPv6 module

static SystemTickHandle     ipTimerHandle = 0;          // Handle for the IP task timer
static int                  ipTickPending = 0;          // Pending flag for the IP task
static int                  ipModuleInitCount = 0;      // Indicator of how many times the IP module has been initializes

static uint32_t             fragmentId = 0;             // Static ID to use when sending fragmented packets

static int                  nStackIfs = 0;              // number of interfaces the stack is currently running on

static SINGLE_LIST          ipv6RegisteredUsers = { 0 };

#endif

static uint16_t _Identifier = 0;                        // Static identifier value for IPv4 headers
static const void*      ipMemH = 0;                     // memory handle

// Enumeration defining IPv6 initialization states
enum
{
    IPV6_INIT_STATE_NONE = 0,                           // IPv6 initialization is not in progress
    IPV6_INIT_STATE_INITIALIZE,                         // Initializes IPv6 variables
    IPV6_INIT_STATE_DAD,                                // Duplicate address detection is being performed on the interfaces link-local unicast address
    IPV6_INIT_STATE_SOLICIT_ROUTER,                     // The interface is soliciting routers
    IPV6_INIT_STATE_DONE,                               // The interface is up
    IPV6_INIT_STATE_FAIL                                // The initialization has failed
} IPV6_INIT_STATE;

// Index of address selection rules for IPv6 default address selection
typedef enum
{
    ADDR_INVALID = 0,
    ADDR_USER_SPECIFIED,
    ADDR_INVALID_ADDRESS_SPECIFIED,             
    ADDR_UNDEFINED,
    ADDR_STILL_VALID,
    ADDR_SEL_RULE_1,
    ADDR_SEL_RULE_2,
    ADDR_SEL_RULE_3,
    ADDR_SEL_RULE_4,
    ADDR_SEL_RULE_5,
    ADDR_SEL_RULE_6,
    ADDR_SEL_RULE_7,
    ADDR_SEL_RULE_8,
    ADDR_SEL_RULE_9,
    ADDR_SEL_RULE_10,
} IPV6_ADDR_SEL_INDEX;

// IPv6 address policy table for default address selection
const IPV6_ADDRESS_POLICY gPolicyTable[] = {
    {{{0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01}},128,50,0},          // Loopback address
    {{{0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00}},0,40,1},            // Unspecified address
    {{{0x20, 0x02, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00}},16,30,2},           // 2002::/15 - 6to4
    {{{0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00}},96,20,3},           // IPv4-compatible address
    {{{0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xFF, 0xFF, 0x00, 0x00, 0x00, 0x00}},96,10,4},           // IPv4-mapped address
    {{{0x20, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00}},32,5,5},            // 2001::/32 - Teredo tunneling
    {{},0xFF,0,0},
    {{},0xFF,0,0},
    {{},0xFF,0,0},
    {{},0xFF,0,0},
};

static uint16_t InitCount = 0;

#if defined (TCPIP_STACK_USE_IPV6)
// Array of global configuration and state variables for an IPv6 interface
IPV6_INTERFACE_CONFIG ipv6Config[TCPIP_NETWORK_INTERFACES];
#endif

/************************************************************************/
/****************               Prototypes               ****************/
/************************************************************************/
static void SwapIPHeader(IP_HEADER* h);

#if defined (TCPIP_STACK_USE_IPV6)
    // Free all of the dynamically linked lists in an IPv6 net configuration
    void IPv6FreeConfigLists (IPV6_INTERFACE_CONFIG * pNetIf);
    // Returns true if addressTwo is preferred over addressOne
    unsigned char IPv6ASCompareSourceAddresses(NET_CONFIG * pNetIf, IPV6_ADDR_STRUCT * addressOne, IPV6_ADDR_STRUCT * addressTwo, IPV6_ADDR * dest, IPV6_ADDR_SEL_INDEX rule);
#endif


#if defined (TCPIP_STACK_USE_IPV6)
/*****************************************************************************
  Function:
	bool TCPIP_IPV6_Initialize(
        const TCPIP_STACK_MODULE_CTRL* const pStackInit, 
        const IPV6_MODULE_GONFIG* const pIpv6Init)

  Summary:
	Initializes an IPv6 interface.

  Description:
	This function initializes an interface for IPv6 operation.  It will 
    create the task timer for the IP task and the IPv6 initialization task 
    (if it hasn't already been created by a previous call to this function). 

  Precondition:
	The IP module initialization function must have been called.

  Parameters:
	pStackInit - Stack initialization parameters
    pIpv6Init - Unused supplementary data field

  Returns:
  	true if interface is being initialized, false otherwise
  	
  Remarks:
	None
  ***************************************************************************/
bool TCPIP_IPV6_Initialize(const TCPIP_STACK_MODULE_CTRL* const pStackInit, const IPV6_MODULE_GONFIG* const pIpv6Init)
{
    int netIx;
    NET_CONFIG* pNetIf;

    if(pStackInit->stackAction == TCPIP_STACK_ACTION_IF_UP)
    {   // interface going up
        return true;
    }

    // stack initialization

    pNetIf = pStackInit->pNetIf;
    netIx = pNetIf->netIfIx;

    // Check to see if this interface is already being initialized
    if (pNetIf->Flags.bIPv6InConfig == true)
        return true;

    pNetIf->Flags.bIPv6InConfig = true;
    ipv6Config[netIx].initState = IPV6_INIT_STATE_INITIALIZE;

    // Use ipTimerHandle as an indicator that global IPv6 services have been initialized
    if (ipTimerHandle == 0)
    {
        ipTimerHandle = SYS_TICK_TimerCreate(TCPIP_IP_TmoHandler);
        if (ipTimerHandle)
        {
            SYS_TICK_TimerSetRate (ipTimerHandle, SYS_TICK_ResolutionGet());
            ipTickPending = 0;

            // Initialize the global fragment ID
            fragmentId = 0;
            // save the max number of interfaces the stack is working on
            nStackIfs = pStackInit->nIfs;
        }
        else
        {
            pNetIf->Flags.bIPv6InConfig = false;
            ipv6Config[netIx].initState = IPV6_INIT_STATE_NONE;            
            return false;
        }
    }

    if (ipInitTimerHandle == 0)
    {
        ipInitTimerHandle = SYS_TICK_TimerCreate(TCPIP_IPV6_InitTmo);
        if (ipInitTimerHandle)
        {
            // Set the interrupt time for this task to 1/32 of a second
            SYS_TICK_TimerSetRate (ipInitTimerHandle, SYS_TICK_ResolutionGet() >> 5);
            ipInitTickPending = 0;
        }
        else
        {
            TCPIP_IPV6_InitializeStop (pNetIf);
            ipv6Config[netIx].initState = IPV6_INIT_STATE_NONE;            
            return false;
        }
        
    }    

    // Initialize the IPv6 parameters for this net
    DoubleListInit (&ipv6Config[netIx].listIpv6UnicastAddresses);
    DoubleListInit (&ipv6Config[netIx].listIpv6MulticastAddresses);
    DoubleListInit (&ipv6Config[netIx].listIpv6TentativeAddresses);
    SingleListInit (&ipv6Config[netIx].listNeighborCache);
    SingleListInit (&ipv6Config[netIx].listDefaultRouter);
    SingleListInit (&ipv6Config[netIx].listDestinationCache);
    SingleListInit (&ipv6Config[netIx].listPrefixList);

    ipv6Config[netIx].currentDefaultRouter = NULL;
    ipv6Config[netIx].baseReachableTime = IPV6_DEFAULT_BASE_REACHABLE_TIME;
    ipv6Config[netIx].reachableTime = IPV6_DEFAULT_BASE_REACHABLE_TIME;
    ipv6Config[netIx].retransmitTime = IPV6_DEFAULT_RETRANSMIT_TIME;
    ipv6Config[netIx].linkMTU = IPV6_DEFAULT_LINK_MTU;
    ipv6Config[netIx].multicastMTU = IPV6_DEFAULT_LINK_MTU;
    ipv6Config[netIx].mtuIncreaseTimer = 0;
    ipv6Config[netIx].curHopLimit = IPV6_DEFAULT_CUR_HOP_LIMIT;

    ipv6Config[netIx].policyPreferTempOrPublic = IPV6_PREFER_PUBLIC_ADDRESSES;

    ipInitCount++;
    ipModuleInitCount++;
    return true;
}

/*****************************************************************************
  Function:
	void TCPIP_IPV6_InitTmo (SYS_TICK curSysTick)

  Summary:
	Timeout function for IPv6 initialization task.

  Description:
	This function will be called by the system timer to indicate to the 
    stack that the IPv6 initialization task function should be called.

  Precondition:
	One or more interfaces must be undergoing IPv6 initialization.

  Parameters:
	curSysTick - The current system tick.

  Returns:
  	None
  	
  Remarks:
	None
  ***************************************************************************/
void TCPIP_IPV6_InitTmo (SYS_TICK curSysTick)
{
    ipInitTickPending++;
}

/*****************************************************************************
  Function:
	bool TCPIP_IPV6_InitTaskPending (void)

  Summary:
	Indicates that the IPv6 initialization task function must be called.

  Description:
	Indicates that the IPv6 initialization task function must be called.	

  Precondition:
	None

  Parameters:
	None

  Returns:
  	true if a task is pending, false otherwise
  	
  Remarks:
	None
  ***************************************************************************/
bool TCPIP_IPV6_InitTaskPending (void)
{
    return (ipInitTickPending == 0)?0:1;
}

/*****************************************************************************
  Function:
	void TCPIP_IPV6_Deinitialize(const TCPIP_STACK_MODULE_CTRL* const stackInit)

  Summary:
	Disables IPv6 functionality on the specified interface.

  Description:
	This function will disable IPv6 functionality on a specified interface. 
    It will free any dynamically allocated structures.

  Precondition:
	None

  Parameters:
	stackInit - Stack initialization parameters

  Returns:
  	None
  	
  Remarks:
	None
  ***************************************************************************/
void TCPIP_IPV6_Deinitialize(const TCPIP_STACK_MODULE_CTRL* const stackInit)
{
    NET_CONFIG* pNetIf;


    if(stackInit->stackAction == TCPIP_STACK_ACTION_IF_DOWN)
    {   // interface shutdown
        return ;
    }

    // stack shut down
    if(ipModuleInitCount)
    {
        pNetIf = stackInit->pNetIf;
        TCPIP_IPV6_InitializeStop (pNetIf);

        IPv6FreeConfigLists (&ipv6Config[pNetIf->netIfIx]);

        pNetIf->Flags.bIPv6Enabled = false;

        if(--ipModuleInitCount == 0)
        {
            SYS_TICK_TimerDelete(ipTimerHandle);
            ipTimerHandle = 0;
            ipTickPending = 0;
            nStackIfs = 0;

        }
    }
}

/*****************************************************************************
  Function:
	void TCPIP_IPV6_InitializeStop (NET_CONFIG * pNetIf)

  Summary:
	Stops IPv6 initialization

  Description:
	This function will halt IPv6 initialization for a specified interface. 
    If it determines that no interfaces are initializing IPv6, it will 
    destroy the IPv6 initialization task system timer.

  Precondition:
	None

  Parameters:
	pNetIf - The interface to top initialization for.

  Returns:
  	None
  	
  Remarks:
	None
  ***************************************************************************/
void TCPIP_IPV6_InitializeStop (NET_CONFIG * pNetIf)
{
    if(ipInitCount)
    {   // We're up and running
        if (pNetIf->Flags.bIPv6InConfig == true)
        {
            pNetIf->Flags.bIPv6InConfig = false;
            if (--ipInitCount == 0)
            {   // release
                SYS_TICK_TimerDelete(ipInitTimerHandle);
                ipInitTimerHandle = 0;
                ipInitTickPending = 0;
            }
        }
    }
}

/*****************************************************************************
  Function:
	void TCPIP_IPV6_InitializeTask (void)

  Summary:
	Task function for IPv6 initialization.

  Description:
    Task function for IPv6 initialization.

  Precondition:
	None

  Parameters:
	None

  Returns:
  	None
  	
  Remarks:
	None
  ***************************************************************************/
void TCPIP_IPV6_InitializeTask (void)
{
    IPV6_ADDR_STRUCT * localAddressPointer;
    IPV6_ADDR linkLocalAddress = {{0xFE, 0x80, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0 | 0x02, 0, 0, 0xFF, 0xFE, 0, 0, 0}};
    int netIx;
    NET_CONFIG * pNetIf;
    
    for (netIx = 0; netIx < TCPIP_STACK_NetworksNo(); netIx++)
    {
        pNetIf = (NET_CONFIG*)TCPIP_STACK_IxToNet (netIx);

        if ((pNetIf->Flags.bIPv6InConfig == true) && (MACIsLinked(_TCPIPStackNetToMac(pNetIf))))
        {
            linkLocalAddress.v[8] = pNetIf->MyMACAddr.v[0] | 0x02;
            linkLocalAddress.v[9] = pNetIf->MyMACAddr.v[1];
            linkLocalAddress.v[10] = pNetIf->MyMACAddr.v[2];
            linkLocalAddress.v[13] = pNetIf->MyMACAddr.v[3];
            linkLocalAddress.v[14] = pNetIf->MyMACAddr.v[4];
            linkLocalAddress.v[15] = pNetIf->MyMACAddr.v[5];

            switch (ipv6Config[netIx].initState)
            {
                case IPV6_INIT_STATE_INITIALIZE:
                    // Add the all-nodes multicast listener to this node
                    localAddressPointer = TCPIP_IPV6_AddMulticastListener (pNetIf, (IPV6_ADDR *)&IPV6_FIXED_ADDR_ALL_NODES_MULTICAST);
                    if (localAddressPointer == NULL)
                    {
                        ipv6Config[netIx].initState = IPV6_INIT_STATE_FAIL;
                        break;
                    }
                    // Configure link-local address
                    localAddressPointer = TCPIP_IPV6_AddUnicastAddress (pNetIf, &linkLocalAddress, false);
                    if (localAddressPointer == NULL)
                    {
                        ipv6Config[netIx].initState = IPV6_INIT_STATE_FAIL;
                        break;
                    }
        
                    // Enable IPv6 functionality for now so we can process ICMPv6 messages for stateless address autoconfiguration
                    pNetIf->Flags.bIPv6Enabled = true;
                    ipv6Config[netIx].initState = IPV6_INIT_STATE_DAD;
                    break;
                case IPV6_INIT_STATE_DAD:
                    if (TCPIP_IPV6_FindAddress (pNetIf, &linkLocalAddress, IPV6_ADDR_TYPE_UNICAST))
                    {
                        ipv6Config[netIx].initState = IPV6_INIT_STATE_SOLICIT_ROUTER;
                    }
                    else if (TCPIP_IPV6_FindAddress (pNetIf, &linkLocalAddress, IPV6_ADDR_TYPE_UNICAST_TENTATIVE) == NULL)
                    {
                        ipv6Config[netIx].initState = IPV6_INIT_STATE_FAIL;
                    }
                    break;
                case IPV6_INIT_STATE_SOLICIT_ROUTER:
                    TCPIP_NDP_RS_Start(pNetIf);
                    ipv6Config[netIx].initState = IPV6_INIT_STATE_DONE;            
                case IPV6_INIT_STATE_DONE:
                    TCPIP_IPV6_InitializeStop (pNetIf);
                    break;
                case IPV6_INIT_STATE_FAIL:
                    IPv6FreeConfigLists (&ipv6Config[netIx]);
                    TCPIP_IPV6_InitializeStop (pNetIf);
                    pNetIf->Flags.bIPv6Enabled = 0;
                    break;
                default:
                    break;
            }
        }
    }
}

/*****************************************************************************
  Function:
	bool TCPIP_IPV6_InterfaceIsUp (NET_CONFIG * pNetIf)

  Summary:
	Determines if an interface is ready for IPv6 transactions.

  Description:
	Determines if an interface is ready for IPv6 transactions.	

  Precondition:
	None

  Parameters:
	pNetIf - The interface to check

  Returns:
  	true if the interface has IPv6 functionality available, false otherwise
  	
  Remarks:
	None
  ***************************************************************************/
bool TCPIP_IPV6_InterfaceIsUp (NET_CONFIG * pNetIf)
{
    if (!pNetIf->Flags.bIPv6InConfig && pNetIf->Flags.bIPv6Enabled)
        return true;

    return false;
}

#endif

/*****************************************************************************
  Function:
	bool TCPIP_IP_Initialize(const TCPIP_STACK_MODULE_CTRL* const stackInit, 
        const IP_MODULE_GONFIG* pIpInit)

  Summary:
	Initializes the IP module.

  Description:
	Initializes the IP module.  Sets the dynamic heap used by this module.	

  Precondition:
	None

  Parameters:
	stackInit - Stack initialization parameters
    pIpInit - Unused supplementary data.

  Returns:
  	true
  	
  Remarks:
	None
  ***************************************************************************/
bool TCPIP_IP_Initialize(const TCPIP_STACK_MODULE_CTRL* const stackInit, const IP_MODULE_GONFIG* pIpInit)
{
    if(stackInit->stackAction == TCPIP_STACK_ACTION_IF_UP)
    {   // interface restart
        return true;
    }


	if (!InitCount)
	{
		ipMemH = stackInit->memH;
	}
	
	InitCount++;


    return true;
}

/*****************************************************************************
  Function:
	void TCPIP_IP_DeInitialize(const TCPIP_STACK_MODULE_CTRL* const stackInit)

  Summary:
	Deinitializes the IP module.

  Description:
	Deinitializes the IP module.	

  Precondition:
	None

  Parameters:
	stackInit - Stack initialization data

  Returns:
  	None
  	
  Remarks:
	None
  ***************************************************************************/
void TCPIP_IP_DeInitialize(const TCPIP_STACK_MODULE_CTRL* const stackInit)
{
    if(stackInit->stackAction == TCPIP_STACK_ACTION_DEINIT)
    {   // stack shut down

        InitCount = (InitCount > 0 ? InitCount - 1 : 0);
        if (!InitCount)
        {
            // release any allocated memory !!!
            ipMemH = 0;
        }
    }
}

/*****************************************************************************
  Function:
	IP_PACKET * TCPIP_IP_AllocateTxPacket (NET_CONFIG * pNetIf, IP_ADDRESS_TYPE addressType,
    IP_PACKET_ACK_FNC ackFnc, void* ackParam);


  Summary:
    Dynamically allocates a packet for transmitting IP protocol data.

  Description:
    Dynamically allocates a packet for transmitting IP protocol data.	

  Precondition:
	None

  Parameters:
	pNetIf - Interface of the outgoing packet.
    addressType - IP_ADDRESS_TYPE_IPV4, IP_ADDRESS_TYPE_IPV6
    ackFnc      - function to be called when IP is done with the TX packet
                  (finished transmitting)
    ackParam    - parameter to be used for this callback
                  This has meaning only for the caller of the
                  TCPIP_IP_AllocateTxPacket

  Returns:
  	IP_PACKET * - Pointer to the allocated packet.
  	
  Remarks:
	None
  ***************************************************************************/
IP_PACKET * TCPIP_IP_AllocateTxPacket (NET_CONFIG * pNetIf, IP_ADDRESS_TYPE addressType, IP_PACKET_ACK_FNC ackFnc, void* ackParam)
{
    IP_PACKET * ptrPacket = (IP_PACKET *)TCPIP_HEAP_Malloc (ipMemH, sizeof (IP_PACKET));

    if (ptrPacket == NULL)
        return NULL;

    ptrPacket->netIf = pNetIf;
    ptrPacket->flags.val = 0;

    ptrPacket->payload.dataLocation = (PTR_BASE)&ptrPacket->ipHeader;
#if defined (TCPIP_STACK_USE_IPV6)
    ptrPacket->payload.segmentSize = sizeof (IPV6_HEADER);
    ptrPacket->payload.segmentLen = (addressType == IP_ADDRESS_TYPE_IPV6)?sizeof (IPV6_HEADER):sizeof (IP_HEADER);
#else
    ptrPacket->payload.segmentSize = sizeof (IP_HEADER);
    ptrPacket->payload.segmentLen = sizeof (IP_HEADER);
#endif  // defined (TCPIP_STACK_USE_IPV6)

    ptrPacket->payload.memory = IP_DATA_PIC_RAM;
    ptrPacket->payload.segmentType = TYPE_IP_HEADER;
    ptrPacket->payload.nextSegment = NULL;

    TCPIP_IP_SetPacketIPProtocol (ptrPacket, addressType);

    ptrPacket->headerLen = 0;
    ptrPacket->flags.queued = false;
    ptrPacket->flags.sourceSpecified = false;
    ptrPacket->flags.useUnspecAddr = false;

#if defined (TCPIP_STACK_USE_IPV6)
    ptrPacket->neighbor = NULL;
#endif

    ptrPacket->payloadLen = 0;
    ptrPacket->upperLayerHeaderLen = 0;
    memset (&ptrPacket->remoteMACAddr, 0x00, sizeof (MAC_ADDR));

    ptrPacket->ackFnc = ackFnc;
    ptrPacket->ackParam = ackParam;

    return ptrPacket;
}

/*****************************************************************************
  Function:
	bool TCPIP_IP_CopyTxPacketStruct (IP_PACKET * destination, 
        IP_PACKET * source)

  Summary:
	Copies data from one IP_PACKET to another.

  Description:
	This function will copy essential transmission data from one IP packet 
    to another.  This includes the upper-layer header and IP header 
    information.

  Precondition:
	None

  Parameters:
	destination - The destination packet.
    source - The source packet

  Returns:
  	true if the data was copied successfully, false otherwise
  	
  Remarks:
	None
  ***************************************************************************/
bool TCPIP_IP_CopyTxPacketStruct (IP_PACKET * destination, IP_PACKET * source)
{
    IP_DATA_SEGMENT_HEADER *ptrSegmentSource, *ptrSegmentDest;

    if (destination == NULL || source == NULL)
    {
        return false;
    }

    memcpy ((void *)&destination->remoteMACAddr, (void *)&source->remoteMACAddr, sizeof (MAC_ADDR));
    destination->flags.useUnspecAddr = source->flags.useUnspecAddr;
    destination->flags.sourceSpecified = source->flags.sourceSpecified;
    destination->flags.addressType = source->flags.addressType;


    memcpy ((void *)&destination->payload, (void *)&source->payload, sizeof (IP_DATA_SEGMENT_HEADER));
    
#if defined (TCPIP_STACK_USE_IPV6)
    destination->neighbor = source->neighbor;
    memcpy ((void *)&destination->ipHeader, (void *)&source->ipHeader, sizeof (IPV6_HEADER));
    if (destination->flags.addressType == IP_ADDRESS_TYPE_IPV6)
    {
        destination->ipHeader.ipv6Header.PayloadLength = 0x0000;
    }
    else
    {
        destination->ipHeader.ipv4Header.TotalLength = 0x0000;
        destination->ipHeader.ipv4Header.HeaderChecksum = 0x0000;
    }
#else
    memcpy ((void *)&destination->ipHeader, (void *)&source->ipHeader, sizeof (IP_HEADER));
    destination->ipHeader.ipv4Header.TotalLength = 0x0000;
    destination->ipHeader.ipv4Header.HeaderChecksum = 0x0000;
#endif  // defined (TCPIP_STACK_USE_IPV6)

    destination->payload.dataLocation = (PTR_BASE)&destination->ipHeader;

    if ((ptrSegmentSource = TCPIP_IP_GetDataSegmentByType (destination, TYPE_UPPER_LAYER_HEADER)) != NULL)
    {
        if ((ptrSegmentDest = TCPIP_IP_GetDataSegmentByType (destination, ptrSegmentSource->segmentType)) == NULL)
        {
            ptrSegmentDest = TCPIP_IP_AllocateDataSegmentHeader (ptrSegmentSource->segmentSize);
            if (ptrSegmentDest == NULL)
                return false;
            TCPIP_IP_InsertIPPacketSegment (ptrSegmentDest, destination, ptrSegmentSource->segmentType);
        }

        memcpy ((void *)ptrSegmentDest->dataLocation, (void *)ptrSegmentSource->dataLocation, ptrSegmentSource->segmentLen);

        destination->upperLayerHeaderLen = source->upperLayerHeaderLen;
        destination->upperLayerChecksumOffset = source->upperLayerChecksumOffset;
        destination->upperLayerHeaderType = source->upperLayerHeaderType;        
    }

    return true;
}

/*****************************************************************************
  Function:
	void TCPIP_IP_SetPacketIPProtocol (IP_PACKET * ptrPacket, 
        IP_ADDRESS_TYPE addressType)

  Summary:
	Sets the packet IP protocol for a TX packet to IPv4 or IPv6.

  Description:
	Sets the packet IP protocol for a TX packet to IPv4 or IPv6.	

  Precondition:
	None

  Parameters:
	ptrPacket - Pointer to the target packet.
    addressType - IP_ADDRESS_TYPE_IPV4, IP_ADDRESS_TYPE_IPV6

  Returns:
  	None
  	
  Remarks:
	None
  ***************************************************************************/
void TCPIP_IP_SetPacketIPProtocol (IP_PACKET * ptrPacket, IP_ADDRESS_TYPE addressType)
{

#if defined (TCPIP_STACK_USE_IPV6)
    if (addressType == IP_ADDRESS_TYPE_IPV6)
        ptrPacket->payload.segmentLen = sizeof (IPV6_HEADER);
    else
#endif
        ptrPacket->payload.segmentLen = sizeof (IP_HEADER);            

    ptrPacket->flags.addressType = addressType;
}

void TCPIP_IP_PutHeader(IP_PACKET * ptrPacket, uint8_t protocol)
{
    void * ptrSegment = TCPIP_IP_GetDataSegmentContentsByType (ptrPacket, TYPE_IP_HEADER);

#if defined (TCPIP_STACK_USE_IPV6)
    if(ptrPacket->flags.addressType == IP_ADDRESS_TYPE_IPV6)
    {
        ((IPV6_HEADER *)ptrSegment)->V_T_F = (uint32_t)IPv6_VERSION;
        ((IPV6_HEADER *)ptrSegment)->HopLimit = 0;
    }
    else
#endif
    {
        ((IP_HEADER *)ptrSegment)->VersionIHL       = IPv4_VERSION | IP_IHL;
        ((IP_HEADER *)ptrSegment)->TypeOfService    = IP_SERVICE;
        ((IP_HEADER *)ptrSegment)->Identification   = ++_Identifier;
        ((IP_HEADER *)ptrSegment)->FragmentInfo     = 0;
        ((IP_HEADER *)ptrSegment)->TimeToLive       = MY_IP_TTL;
        ((IP_HEADER *)ptrSegment)->Protocol         = protocol;
        ((IP_HEADER *)ptrSegment)->HeaderChecksum   = 0;

        if (!ptrPacket->flags.sourceSpecified)
           	((IP_HEADER *)ptrSegment)->SourceAddress 	= ptrPacket->netIf->MyIPAddr;
    
        ptrPacket->flags.sourceSpecified = false;

        // Note : The upper layer protocol will already have copied the destination address into the packet header
    }
}

/*****************************************************************************
  Function:
	IP_DATA_SEGMENT_HEADER * TCPIP_IP_PutUpperLayerHeader (
        IP_PACKET * ptrPacket, void * header, unsigned short len, 
        unsigned char type, unsigned short checksumOffset)

  Summary:
	Adds an upper layer header to an IP TX packet.

  Description:
	This function will add an upper layer header to the chain of segments 
    in an IP_PACKET structure.  It will also initialize the packet's 
    upper-layer header variables.

  Precondition:
	None

  Parameters:
	ptrPacket - The packet that the upper layer header applies to.
    header - Pointer to the header data.
    len - Length of the header.
    type - Standard upper layer header type.
    checksumOffset - Offset of the upper-layer checksum in the header, or 
        IP_NO_UPPER_LAYER_CHECKSUM.

  Returns:
  	IP_DATA_SEGMENT_HEADER * - Pointer to the segment in the packet that 
        contains the header information.
  	
  Remarks:
	This function will automatically allocate memory to store the header data.
  ***************************************************************************/
IP_DATA_SEGMENT_HEADER * TCPIP_IP_PutUpperLayerHeader (IP_PACKET * ptrPacket, void * header, unsigned short len, unsigned char type, unsigned short checksumOffset)
{
    IP_DATA_SEGMENT_HEADER * ptrSegment = TCPIP_IP_AllocateDataSegmentHeader(len);

    if (ptrSegment == NULL)
        return NULL;

    ptrSegment->segmentLen = len;

    if (header != NULL)
        memcpy (ptrSegment->data, header, len);

    ptrPacket->upperLayerHeaderLen = len;
    ptrPacket->upperLayerHeaderType = type;
    ptrPacket->upperLayerChecksumOffset = checksumOffset;

    TCPIP_IP_InsertIPPacketSegment (ptrSegment, ptrPacket, TYPE_UPPER_LAYER_HEADER);

    return ptrSegment;
}

/*****************************************************************************
  Function:
	void TCPIP_IP_InsertIPPacketSegment (IP_DATA_SEGMENT_HEADER * ptrSegment, 
        IP_PACKET * ptrPacket, IP_SEGMENT_TYPE type)

  Summary:
	Inserts a data segment into an IP_PACKET structure.

  Description:
	This function inserts a data segment into an IP_PACKET structure.  It will 
    update the next header fields in existing segments, if applicable.

  Precondition:
	None

  Parameters:
	ptrSegment - The segment to insert.
    ptrPacket - The packet to insert the segment into.
    type - The segment type.  Defined by IP_SEGMENT_TYPE enumeration.

  Returns:
  	None
  	
  Remarks:
	None
  ***************************************************************************/
void TCPIP_IP_InsertIPPacketSegment (IP_DATA_SEGMENT_HEADER * ptrSegment, IP_PACKET * ptrPacket, IP_SEGMENT_TYPE type)
{
    IP_DATA_SEGMENT_HEADER * tempSegment;
#if defined (TCPIP_STACK_USE_IPV6)
    uint8_t * nextHeader;
#endif

    if ((ptrSegment == NULL) || (ptrPacket == NULL))
        return;

    tempSegment = &ptrPacket->payload; 

    ptrSegment->segmentType = type;

    if (type == TYPE_END_OF_LIST)
    {
        while (tempSegment->nextSegment != NULL)
            tempSegment = tempSegment->nextSegment;
    }
    else
    {
        while ((tempSegment->nextSegment != NULL) && (ptrSegment->segmentType > tempSegment->nextSegment->segmentType))
            tempSegment = tempSegment->nextSegment;
    }
    ptrSegment->nextSegment = tempSegment->nextSegment;
    tempSegment->nextSegment = ptrSegment;

    if (type == TYPE_END_OF_LIST)
    {
        ptrSegment->segmentType = TYPE_UPPER_LAYER_PAYLOAD;
    }

#if defined (TCPIP_STACK_USE_IPV6)
    if (tempSegment->segmentType < TYPE_UPPER_LAYER_HEADER)
    {
        if (tempSegment == &ptrPacket->payload)
        {
            nextHeader = &ptrPacket->ipHeader.ipv6Header.NextHeader;
        }
        else
        {
            nextHeader = (uint8_t *)&tempSegment->data;
        }
        switch (ptrSegment->segmentType)
        {
            case TYPE_EX_HEADER_HOP_BY_HOP_OPTIONS:
                *nextHeader = IP_PROT_HOP_BY_HOP_OPTIONS_HEADER;
                break;
            case TYPE_EX_HEADER_DESTINATION_OPTIONS_1:
            case TYPE_EX_HEADER_DESTINATION_OPTIONS_2:
                *nextHeader = IP_PROT_DESTINATION_OPTIONS_HEADER;
                break;
            case TYPE_EX_HEADER_ROUTING:
                *nextHeader = IP_PROT_ROUTING_HEADER;
                break;
            case TYPE_EX_HEADER_FRAGMENT:
                *nextHeader = IP_PROT_FRAGMENTATION_HEADER;
                break;
            case TYPE_EX_HEADER_AUTHENTICATION_HEADER:
                *nextHeader = IP_PROT_AUTHENTICATION_HEADER;
                break;
            case TYPE_EX_HEADER_ENCAPSULATING_SECURITY_PAYLOAD:
                *nextHeader = IP_PROT_ENCAPSULATING_SECURITY_PAYLOAD_HEADER;
                break;
            default:
                *nextHeader = ptrPacket->upperLayerHeaderType;
                break;
        }
    }

    if (ptrSegment->segmentType < TYPE_UPPER_LAYER_HEADER)
    {
        nextHeader = (uint8_t *)&ptrSegment->data;
        if (ptrSegment->nextSegment == NULL)
        {
            *nextHeader = IP_PROT_NONE;
        }
        else
        {
            switch (ptrSegment->nextSegment->segmentType)
            {
                case TYPE_EX_HEADER_HOP_BY_HOP_OPTIONS:
                    *nextHeader = IP_PROT_HOP_BY_HOP_OPTIONS_HEADER;
                    break;
                case TYPE_EX_HEADER_DESTINATION_OPTIONS_1:
                case TYPE_EX_HEADER_DESTINATION_OPTIONS_2:
                    *nextHeader = IP_PROT_DESTINATION_OPTIONS_HEADER;
                    break;
                case TYPE_EX_HEADER_ROUTING:
                    *nextHeader = IP_PROT_ROUTING_HEADER;
                    break;
                case TYPE_EX_HEADER_FRAGMENT:
                    *nextHeader = IP_PROT_FRAGMENTATION_HEADER;
                    break;
                case TYPE_EX_HEADER_AUTHENTICATION_HEADER:
                    *nextHeader = IP_PROT_AUTHENTICATION_HEADER;
                    break;
                case TYPE_EX_HEADER_ENCAPSULATING_SECURITY_PAYLOAD:
                    *nextHeader = IP_PROT_ENCAPSULATING_SECURITY_PAYLOAD_HEADER;
                    break;
                default:
                    *nextHeader = ptrPacket->upperLayerHeaderType;
                    break;
            }
        }
    }
#endif
}

/*****************************************************************************
  Function:
	IP_DATA_SEGMENT_HEADER * TCPIP_IP_GetDataSegmentByType (
        IP_PACKET * ptrPacket, IP_SEGMENT_TYPE type)

  Summary:
	Returns the data segment header of a segment of specified type from a 
    packet.

  Description:
	Returns the data segment header of a segment of specified type from a 
    packet.	

  Precondition:
	None

  Parameters:
	ptrPacket - The packet to search.
    type - IP_SEGMENT_TYPE segment type value to search for.

  Returns:
  	IP_DATA_SEGMENT_HEADER * - Pointer to the specified segment type, or NULL.
  	
  Remarks:
	There is a special IP_SEGMENT_TYPE value defined:
        - TYPE_BEGINNING_OF_WRITABLE_PART searches for the first upper layer 
            payload segment to which data can be written.
  ***************************************************************************/
IP_DATA_SEGMENT_HEADER * TCPIP_IP_GetDataSegmentByType (IP_PACKET * ptrPacket, IP_SEGMENT_TYPE type)
{
    IP_DATA_SEGMENT_HEADER * ptrSegment;

    if (ptrPacket == NULL)
        return NULL;

    ptrSegment = &ptrPacket->payload;

    if (type != TYPE_BEGINNING_OF_WRITABLE_PART)
    {
        while ((ptrSegment != NULL) && (ptrSegment->segmentType < type))
        {
            ptrSegment = ptrSegment->nextSegment;
        }
    }
    else
    {
        IP_DATA_SEGMENT_HEADER * ptrTempSegment;

        type = TYPE_UPPER_LAYER_PAYLOAD;

        while ((ptrSegment != NULL) && (ptrSegment->segmentType < type))
        {
            ptrSegment = ptrSegment->nextSegment;
        }

        while (ptrSegment != NULL)
        {
            // Move ptrSegment to the next dynamically allocated segment
            while ((ptrSegment != NULL) && (ptrSegment->memory != IP_DATA_DYNAMIC_BUFFER))
                ptrSegment = ptrSegment->nextSegment;
            // Break out of the loop if the pointer gets to the end of the linked list
            if (ptrSegment == NULL)
                break;
    
            // Initialize ptrTempSegment to the first segment after the beginning of the dynamically allocated segments
            ptrTempSegment = ptrSegment->nextSegment;
            // Check to see if the dynamically allocated section is contiguous at the end of the linked list (or find the break)
            while ((ptrTempSegment != NULL) && (ptrTempSegment->memory == IP_DATA_DYNAMIC_BUFFER))
                ptrTempSegment = ptrTempSegment->nextSegment;
    
            if (ptrTempSegment != NULL)
            {
                // If there is a non-dynamic segment, continue in the loop until we find the final sublist
                // of dynamically allocated segments
                ptrSegment = ptrTempSegment;
            }
            else
            {
                // If we have reached the final sublist of dynamic segments, advance to the 
                // section that is writable.
                ptrTempSegment = ptrSegment->nextSegment;
                while (ptrTempSegment != NULL)
                {
                    if (ptrTempSegment->segmentLen != 0)
                        ptrSegment = ptrTempSegment;
                    ptrTempSegment = ptrTempSegment->nextSegment;
                }
                break;
            }
        }
    }

    if (ptrSegment == NULL)
        return NULL;
    else if (ptrSegment->segmentType == type)
        return ptrSegment;
    else
        return NULL;
}

/*****************************************************************************
  Function:
	IP_DATA_SEGMENT_HEADER * TCPIP_IP_GetDataSegmentContentsByType (
        IP_PACKET * ptrPacket, IP_SEGMENT_TYPE type)

  Summary:
	Returns a pointer to the contents of a segment of specified type from a 
    packet.

  Description:
	Returns a pointer to the contents of a segment of specified type from a 
    packet.

  Precondition:
	None

  Parameters:
	ptrPacket - The packet to search.
    type - IP_SEGMENT_TYPE segment type value to search for.

  Returns:
  	None
  	
  Remarks:
	None
  ***************************************************************************/
void * TCPIP_IP_GetDataSegmentContentsByType (IP_PACKET * ptrPacket, IP_SEGMENT_TYPE type)
{
    IP_DATA_SEGMENT_HEADER * ptr;

    ptr = TCPIP_IP_GetDataSegmentByType(ptrPacket, type);

    if (ptr != NULL)
    {
        return (void *)ptr->dataLocation;
    }
    else
        return NULL;
}

/*****************************************************************************
  Function:
	unsigned short TCPIP_IP_IsTxPutReady (IP_PACKET * ptrPacket, 
        unsigned short count)

  Summary:
	Determines whether a TX packet can be written to.

  Description:
	Determines whether a TX packet can be written to.  This function will 
    allocate additional space to the packet to accomodate the user.	

  Precondition:
	None

  Parameters:
	ptrPacket - The packet to check.
    count - The amount of writable space to check for,

  Returns:
  	unsigned short - The amount of space available.
  	
  Remarks:
	None
  ***************************************************************************/
unsigned short TCPIP_IP_IsTxPutReady (IP_PACKET * ptrPacket, unsigned short count)
{
    IP_DATA_SEGMENT_HEADER * ptrSegment;
    unsigned short payloadSize = 0;
    unsigned short availableSpace;

    if (ptrPacket == NULL)
        return 0;

    payloadSize = (count > IP_DEFAULT_ALLOCATION_BLOCK_SIZE)?count:IP_DEFAULT_ALLOCATION_BLOCK_SIZE;

    ptrSegment = TCPIP_IP_GetDataSegmentByType (ptrPacket, TYPE_UPPER_LAYER_PAYLOAD);

    // Verify that there is a valid upper layer payload
    if (ptrSegment == NULL)
    {
        ptrSegment = TCPIP_IP_AllocateDataSegmentHeader(payloadSize);
        if (ptrSegment == NULL)
            return 0;

        TCPIP_IP_InsertIPPacketSegment (ptrSegment, ptrPacket, TYPE_END_OF_LIST);

        return payloadSize;        
    }

    ptrSegment = TCPIP_IP_GetDataSegmentByType (ptrPacket, TYPE_BEGINNING_OF_WRITABLE_PART);

    availableSpace = 0;

    if (ptrSegment != NULL)
    {
        while (ptrSegment != NULL)
        {
            availableSpace += ptrSegment->segmentSize - ptrSegment->segmentLen;
            ptrSegment = ptrSegment->nextSegment;
        }

        if (availableSpace >= count)
        {
            return availableSpace;
        }
    }
    
    // allocate new payload
    payloadSize -= availableSpace;
    if(payloadSize < IP_DEFAULT_ALLOCATION_BLOCK_SIZE)
    {
        payloadSize = IP_DEFAULT_ALLOCATION_BLOCK_SIZE;
    }

    ptrSegment = TCPIP_IP_AllocateDataSegmentHeader(payloadSize);
    if (ptrSegment == NULL)
        return 0;

    TCPIP_IP_InsertIPPacketSegment (ptrSegment, ptrPacket, TYPE_END_OF_LIST);

    return availableSpace + payloadSize ;        
}

/*****************************************************************************
  Function:
	bool TCPIP_IP_Put (IP_PACKET * pkt, unsigned char v)

  Summary:
	Writes a character of data to a packet.

  Description:
    Writes a character of data to a packet.

  Precondition:
	None

  Parameters:
	pkt - The packet.
  	v - The characeter.

  Returns:
    true if the character was written, false otherwise
  	
  Remarks:
	None
  ***************************************************************************/
bool TCPIP_IP_Put (IP_PACKET * pkt, unsigned char v)
{
    return (TCPIP_IP_PutArray (pkt, &v, 1) == 1)?true:false;
}

/*****************************************************************************
  Function:
	unsigned short TCPIP_IP_PutArrayHelper (IP_PACKET * ptrPacket, 
        const void * dataSource, uint8_t dataType, unsigned short len)

  Summary:
	Helper function to write data to a packet.

  Description:
	Helper function to write data to a packet.	

  Precondition:
	None

  Parameters:
	ptrPacket - The packet.
    dataSource - The address of the data on its medium.
    dataType - Descriptor of the data type (dynamic memory on PIC, in a 
                    network FIFO, in static PIC RAM)
    len - Length of the data.

  Returns:
  	unsigned short - The number of bytes of data written.
  	
  Remarks:
	None
  ***************************************************************************/
unsigned short TCPIP_IP_PutArrayHelper (IP_PACKET * ptrPacket, const void * dataSource, uint8_t dataType, unsigned short len)
{
    IP_DATA_SEGMENT_HEADER * ptrSegment;
    uint8_t * dataLocation;
    unsigned short txSize = 0;

    if (ptrPacket == NULL)
        return 0;

    ptrSegment = TCPIP_IP_GetDataSegmentByType (ptrPacket, TYPE_BEGINNING_OF_WRITABLE_PART);

    if (ptrSegment == NULL)
        return 0;

    dataLocation = (uint8_t *)ptrSegment->dataLocation;

    if (ptrSegment->segmentLen + len > ptrSegment->segmentSize)
    {
        txSize = ptrSegment->segmentSize - ptrSegment->segmentLen;
        if (dataType == IP_DATA_PIC_RAM)
            memcpy ((void *)(dataLocation + ptrSegment->segmentLen), dataSource, txSize);
        else if (dataType == IP_DATA_NETWORK_FIFO)
            MACGetArray(*((TCPIP_MAC_HANDLE *)dataSource), dataLocation + ptrSegment->segmentLen, txSize);
        else
            return 0;
        ptrSegment->segmentLen += txSize;
        ptrPacket->payloadLen += len;

        ptrSegment = TCPIP_IP_AllocateDataSegmentHeader(len - txSize);
        if (ptrSegment == NULL)
            return txSize;

        TCPIP_IP_InsertIPPacketSegment (ptrSegment, ptrPacket, TYPE_END_OF_LIST);
        dataLocation = (uint8_t *)ptrSegment->dataLocation;
        len -= txSize;
    }

    if ((ptrSegment->segmentLen + len <= ptrSegment->segmentSize))
    {
        if (dataType == IP_DATA_PIC_RAM)
            memcpy ((void *)(dataLocation + ptrSegment->segmentLen), dataSource, len);
        else if (dataType == IP_DATA_NETWORK_FIFO)
            MACGetArray(*((TCPIP_MAC_HANDLE *)dataSource), dataLocation + ptrSegment->segmentLen, len);
        else
            return 0;
        ptrSegment->segmentLen += len;
        ptrPacket->payloadLen += len;
        return len + txSize;
    }
    return 0;
}

/*****************************************************************************
  Function:
	unsigned short TCPIP_IP_SetPayload (IP_PACKET * ptrPacket, 
        PTR_BASE payload, unsigned short len)

  Summary:
	Allocates a segment on the end of a packet segment chain and uses it to 
    address pre-buffered data.

  Description:
	This function will allocate a data segment header and append it to the 
    end of a chain of segments in a TX packet.  It will set the data ptr in 
    the packet segment to a pre-existing buffer of data.

  Precondition:
	None

  Parameters:
	ptrPacket - The packet.
    payload - Address of the data payload.
    len - Length of the data payload

  Returns:
  	unsigned short - The amount of data added to the packet length.
  	
  Remarks:
	This function is useful for adding payloads to outgoing packets without 
    copying them if the data is in another preexisting buffer (i.e. TCP).
  ***************************************************************************/
unsigned short TCPIP_IP_SetPayload (IP_PACKET * ptrPacket, PTR_BASE payload, unsigned short len)
{
    IP_DATA_SEGMENT_HEADER * ptrSegment;

    ptrSegment = TCPIP_IP_AllocateDataSegmentHeader (0);
    if(ptrSegment == 0)
    {
        return 0;
    }

    ptrSegment->nextSegment = NULL;
    ptrSegment->dataLocation = payload;
    ptrSegment->segmentSize = len;
    ptrSegment->segmentLen = len;
    ptrSegment->memory = IP_DATA_PIC_RAM;

    TCPIP_IP_InsertIPPacketSegment (ptrSegment, ptrPacket, TYPE_END_OF_LIST);

    ptrPacket->payloadLen += len;

    return len;
}

/*****************************************************************************
  Function:
	unsigned short TCPIP_IP_GetPseudoHeaderChecksum (IP_PACKET * ptrPacket)

  Summary:
	Returns the 16-bit checksum of the pseudo-header for an IP packet.

  Description:
	Returns the 16-bit checksum of the pseudo-header for an IP packet.	

  Precondition:
	None

  Parameters:
	ptrPacket - The packet

  Returns:
  	unsigned short - The checksum
  	
  Remarks:
	A flag in the packet is used to detrmine the address type (IPv4/6) for 
    this calculation.
  ***************************************************************************/
unsigned short TCPIP_IP_GetPseudoHeaderChecksum (IP_PACKET * ptrPacket)
{
#if defined (TCPIP_STACK_USE_IPV6)
    if (ptrPacket->flags.addressType == IP_ADDRESS_TYPE_IPV6)
    {
        IPV6_PSEUDO_HEADER pseudoHeader;

        pseudoHeader.zero1 = 0;
        pseudoHeader.zero2 = 0;
        pseudoHeader.PacketLength = swaps (ptrPacket->upperLayerHeaderLen + ptrPacket->payloadLen);
        pseudoHeader.NextHeader = ptrPacket->upperLayerHeaderType;
        memcpy((void *)&pseudoHeader.SourceAddress, (void *)TCPIP_IPV6_GetSourceAddress (ptrPacket), sizeof (IPV6_ADDR));
        memcpy((void *)&pseudoHeader.DestAddress, (void *)TCPIP_IPV6_GetDestAddress (ptrPacket), sizeof (IPV6_ADDR));
        return CalcIPChecksum ((void *)&pseudoHeader, sizeof (pseudoHeader));
    }
    else
#endif
    {
        PSEUDO_HEADER pseudoHeader;

        pseudoHeader.SourceAddress = TCPIP_IPV4_GetSourceAddress (ptrPacket);
        pseudoHeader.DestAddress = TCPIP_IPV4_GetDestAddress (ptrPacket);
        pseudoHeader.Zero = 0;
        pseudoHeader.Protocol = ptrPacket->upperLayerHeaderType;
        pseudoHeader.Length = swaps (ptrPacket->upperLayerHeaderLen + ptrPacket->payloadLen);

        return CalcIPChecksum ((uint8_t *)&pseudoHeader, sizeof (pseudoHeader));
    }
}

#if defined (TCPIP_STACK_USE_IPV6)
/*****************************************************************************
  Function:
	void TCPIP_IPV6_SetHopLimit(IP_PACKET * ptrPacket, uint8_t hopLimit)

  Summary:
	Sets the hop limit of a TX packet to a specific value.

  Description:
	Sets the hop limit of a TX packet to a specific value.	

  Precondition:
	None

  Parameters:
	ptrPacket - The TX packet.
    hopLimit - The new hop limit value.

  Returns:
  	None
  	
  Remarks:
	None
  ***************************************************************************/
void TCPIP_IPV6_SetHopLimit(IP_PACKET * ptrPacket, uint8_t hopLimit)
{
    IPV6_HEADER * ptrHeader = TCPIP_IP_GetDataSegmentContentsByType (ptrPacket, TYPE_IP_HEADER);
    ptrHeader->HopLimit = hopLimit;
}
#endif

/*****************************************************************************
  Function:
	unsigned short TCPIP_IP_CalculatePayloadChecksum (IP_PACKET * ptrPacket)

  Summary:
	Calculates the checksum of an upper layer payload for a TX packet.

  Description:
	Calculates the checksum of an upper layer payload for a TX packet.

  Precondition:
	None

  Parameters:
	ptrPacket - The packet

  Returns:
  	unsigned short - The checksum.
  	
  Remarks:
	None
  ***************************************************************************/
unsigned short TCPIP_IP_CalculatePayloadChecksum (IP_PACKET * ptrPacket)
{
    IP_DATA_SEGMENT_HEADER * ptrSegment;
    TCPIP_UINT32_VAL checksum;
    TCPIP_MAC_HANDLE hMac;
    uint16_t tempChecksum = 0;
    unsigned short checksumByteCount = 0;

    ptrSegment = TCPIP_IP_GetDataSegmentByType (ptrPacket, TYPE_UPPER_LAYER_HEADER);

    checksum.Val = 0;
    if (ptrSegment != NULL)
    {
        checksum.w[0] = ~CalcIPChecksum((uint8_t *)ptrSegment->dataLocation, ptrPacket->upperLayerHeaderLen);
        checksumByteCount += ptrPacket->upperLayerHeaderLen;
    }

    ptrSegment = TCPIP_IP_GetDataSegmentByType (ptrPacket, TYPE_UPPER_LAYER_PAYLOAD);

    while (ptrSegment != NULL)
    {
        switch (ptrSegment->memory)
        {
            case IP_DATA_DYNAMIC_BUFFER:
            case IP_DATA_PIC_RAM:
                tempChecksum = ~CalcIPChecksum((uint8_t *)ptrSegment->dataLocation, ptrSegment->segmentLen);
                if (checksumByteCount % 2)
                {
                    tempChecksum = swaps (tempChecksum);
                }
                checksumByteCount += ptrSegment->segmentLen;
                break;
            case IP_DATA_NETWORK_FIFO:
                hMac = _TCPIPStackNetToMac (ptrPacket->netIf);
                MACSetReadPtr (hMac, ptrSegment->dataLocation);
                tempChecksum = ~MACCalcIPBufferChecksum(hMac, ptrSegment->segmentLen);
                if (checksumByteCount % 2)
                {
                    tempChecksum = swaps(tempChecksum);
                }
                checksumByteCount += ptrSegment->segmentLen;
                break;
            case IP_DATA_NONE:
                tempChecksum = 0;
                break;
        }
        checksum.Val += (uint32_t)tempChecksum;
        ptrSegment = ptrSegment->nextSegment;
    }

    checksum.Val = (uint32_t)checksum.w[0] + (uint32_t)checksum.w[1];
    checksum.w[0] += checksum.w[1];
    return ~checksum.w[0];
}

/*****************************************************************************
  Function:
	void TCPIP_IP_ResetTransmitPacketState (IP_PACKET * pkt)

  Summary:
	Removes payload data from a TX packet structure so it can be reused by 
    a socket for furthur communications.

  Description:
	Removes payload data from a TX packet structure so it can be reused by 
    a socket for furthur communications.	

  Precondition:
	None

  Parameters:
	pkt - The packet to reset.

  Returns:
  	None
  	
  Remarks:
	None
  ***************************************************************************/
void TCPIP_IP_ResetTransmitPacketState (IP_PACKET * pkt)
{
    IP_DATA_SEGMENT_HEADER * segmentHeader = &pkt->payload;
    IP_DATA_SEGMENT_HEADER * segmentHeader2 = 0;

    while ((segmentHeader != NULL) && (segmentHeader->segmentType != TYPE_UPPER_LAYER_PAYLOAD))
    {
        segmentHeader2 = segmentHeader;
        segmentHeader = segmentHeader->nextSegment;
    }

    if(segmentHeader2)
    {
        segmentHeader2->nextSegment = NULL;
    }

    while (segmentHeader != NULL)
    {
        segmentHeader2 = segmentHeader->nextSegment;
        TCPIP_HEAP_Free (ipMemH, segmentHeader);
        segmentHeader = segmentHeader2;
    }

    pkt->payloadLen = 0;
}

#if defined (TCPIP_STACK_USE_IPV6)
/*****************************************************************************
  Function:
	bool TCPIP_IPV6_AddressIsSolicitedNodeMulticast (IPV6_ADDR * address)

  Summary:
	Determines if the specified address is a solicited node multicast address.

  Description:
	Determines if the specified address is a solicited node multicast address.

  Precondition:
	None

  Parameters:
	address - The address.

  Returns:
  	bool - true if the address is a solciited-node multicast address, false 
        otherwise
  	
  Remarks:
	None
  ***************************************************************************/
bool TCPIP_IPV6_AddressIsSolicitedNodeMulticast (IPV6_ADDR * address)
{
    uint8_t solNodeMulticastFragment[] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0xFF, 0x00, 0x00, 0x00};

    if ((address->v[0] == 0xFF) && ((address->v[1] & 0x02) == 0x02) && (memcmp (solNodeMulticastFragment, &address->v[2], sizeof (IPV6_ADDR) - 5) == 0))
    {
        return true;
    }
    return false;
}
#endif

/*****************************************************************************
  Function:
	bool TCPIP_IP_Flush (IP_PACKET * ptrPacket, MAC_ADDR * remoteMACAddr)

  Summary:
	Flushes an IP TX packet.

  Description:
	Flushes an IP TX packet.  Determines the link-layer address if necessary.
    Calculates the upper-layer checksum if necessary.	

  Precondition:
	None

  Parameters:
	ptrPacket - The packet to flush.
    remoteMACAddr - An optional explicity specified MAC address (for IPv4).

  Returns:
  	bool - True if the packet has been transmitted, false if the packet 
        has been queued.
  	
  Remarks:
	None
  ***************************************************************************/
bool TCPIP_IP_Flush (IP_PACKET * ptrPacket, MAC_ADDR * remoteMACAddr)
{
#if defined (TCPIP_STACK_USE_IPV6)
    IPV6_HEAP_NDP_NC_ENTRY * neighborPointer;
#endif
    uint16_t * checksumPointer;

    if (ptrPacket == NULL)
        return false;

    // Write the Ethernet Header to the MAC TX Buffer
#if defined (TCPIP_STACK_USE_IPV6)
    if (ptrPacket->flags.addressType == IP_ADDRESS_TYPE_IPV6)
    {
        IPV6_ADDR * destinationAddress = TCPIP_IPV6_GetDestAddress(ptrPacket);
        IPV6_HEADER * ptrIpHeader = TCPIP_IP_GetDataSegmentContentsByType (ptrPacket, TYPE_IP_HEADER);

        if (ptrPacket->headerLen == 0u)
        {
            ptrIpHeader->NextHeader = ptrPacket->upperLayerHeaderType; 
        }

        ptrIpHeader->PayloadLength = swaps (ptrPacket->payloadLen + ptrPacket->upperLayerHeaderLen + ptrPacket->headerLen);

        if (ptrIpHeader->HopLimit == 0)
            ptrIpHeader->HopLimit = ipv6Config[_TCPIPStackNetIx(ptrPacket->netIf)].curHopLimit;

        // Check to see if a source address was specified
        {
            IPV6_ADDR_STRUCT * sourceAddress;

            if (ptrPacket->flags.sourceSpecified)
            {
                if (ptrPacket->flags.useUnspecAddr)
                {
                    TCPIP_IPV6_SetSourceAddress(ptrPacket, (IPV6_ADDR *)&IPV6_FIXED_ADDR_UNSPECIFIED);                    
                    ptrPacket->flags.sourceSpecified = true;
                }
                else
                {
                    sourceAddress = TCPIP_IPV6_DAS_SelectSourceAddress (ptrPacket->netIf, destinationAddress, &ptrIpHeader->SourceAddress);
                    if (sourceAddress == NULL)
                    {
                        ptrPacket->flags.sourceSpecified = false;
                    }
                }
            }

            if (!ptrPacket->flags.sourceSpecified)
            {
                sourceAddress = TCPIP_IPV6_DAS_SelectSourceAddress (ptrPacket->netIf, destinationAddress, NULL);
                if (sourceAddress != NULL)
                {
                    TCPIP_IPV6_SetSourceAddress(ptrPacket, &(sourceAddress->address));
                }
                else
                {
                    // This should never happen; we should always have at least our link-local auto-configured address
                    TCPIP_IPV6_SetSourceAddress(ptrPacket, (IPV6_ADDR *)&IPV6_FIXED_ADDR_UNSPECIFIED);
                }
            }
        }

        if (ptrPacket->upperLayerChecksumOffset != IP_NO_UPPER_LAYER_CHECKSUM)
        {
            checksumPointer = TCPIP_IP_GetDataSegmentContentsByType (ptrPacket, TYPE_UPPER_LAYER_HEADER);
            if (checksumPointer != NULL)
            {
                checksumPointer = (uint16_t *)(((uint8_t *)checksumPointer) + ptrPacket->upperLayerChecksumOffset);
                *checksumPointer = ~TCPIP_IP_GetPseudoHeaderChecksum (ptrPacket);
                *checksumPointer = TCPIP_IP_CalculatePayloadChecksum (ptrPacket);
            }
        }

        if (destinationAddress->v[0] == 0xFF)
        {
            // Determine the appropriate link-local address for a multicast address
            ptrPacket->remoteMACAddr.v[0] = 0x33;
            ptrPacket->remoteMACAddr.v[1] = 0x33;
            ptrPacket->remoteMACAddr.v[2] = destinationAddress->v[12];
            ptrPacket->remoteMACAddr.v[3] = destinationAddress->v[13];
            ptrPacket->remoteMACAddr.v[4] = destinationAddress->v[14];
            ptrPacket->remoteMACAddr.v[5] = destinationAddress->v[15];

            return TCPIP_IP_TransmitPacket (ptrPacket);
        }
        else
        {
            // Determine the appropriate neighbor to transmit the unicast packet to
            if ((neighborPointer = ptrPacket->neighbor) == NULL)
            {
                neighborPointer = TCPIP_NDP_GetNextHop (ptrPacket->netIf, destinationAddress);
            }

            if (neighborPointer == NULL)
            {
                // The device could not determine a next hop address
                return false;
            }

            switch (neighborPointer->reachabilityState)
            {
                case NDP_STATE_STALE:
                    TCPIP_NDP_SetReachability (ptrPacket->netIf, neighborPointer, NDP_STATE_DELAY);\
                    // Fall through
                case NDP_STATE_REACHABLE:
                case NDP_STATE_DELAY:
                case NDP_STATE_PROBE:
                    memcpy (&ptrPacket->remoteMACAddr, &neighborPointer->remoteMACAddr, sizeof (MAC_ADDR));
                    return TCPIP_IP_TransmitPacket (ptrPacket);
                case NDP_STATE_INCOMPLETE:
                    TCPIP_NDP_ResolveAddress (neighborPointer);
                    TCPIP_IPV6_QueuePacket (neighborPointer, ptrPacket);
                    return false;
                default:
                    TCPIP_NDP_ResolveAddress (neighborPointer);
                    TCPIP_IPV6_QueuePacket (neighborPointer, ptrPacket);
                    TCPIP_NDP_SetReachability (ptrPacket->netIf, neighborPointer, NDP_STATE_INCOMPLETE);
                    return false;
            }
        }
    }
    else
#endif
    {
        IP_HEADER * ptrIpHeader = TCPIP_IP_GetDataSegmentContentsByType (ptrPacket, TYPE_IP_HEADER);

        // Transmit IPv4 Packet
        ptrIpHeader->TotalLength = sizeof (IP_HEADER) + ptrPacket->upperLayerHeaderLen + ptrPacket->payloadLen;

        SwapIPHeader (ptrIpHeader);

        ptrIpHeader->HeaderChecksum = CalcIPChecksum((uint8_t *)ptrIpHeader, sizeof (IP_HEADER));

        if (ptrPacket->upperLayerChecksumOffset != IP_NO_UPPER_LAYER_CHECKSUM)
        {
            checksumPointer = TCPIP_IP_GetDataSegmentContentsByType (ptrPacket, TYPE_UPPER_LAYER_HEADER);
            checksumPointer = (uint16_t *)(((uint8_t *)checksumPointer) + ptrPacket->upperLayerChecksumOffset);
            *checksumPointer = ~TCPIP_IP_GetPseudoHeaderChecksum (ptrPacket);
            *checksumPointer = TCPIP_IP_CalculatePayloadChecksum (ptrPacket);
        }

        if (remoteMACAddr != NULL)
            memcpy (&ptrPacket->remoteMACAddr, remoteMACAddr, sizeof (MAC_ADDR));

        return TCPIP_IP_TransmitPacket (ptrPacket);
    }

    return false;
}

/*****************************************************************************
  Function:
	bool TCPIP_IP_TransmitPacket (IP_PACKET * pkt)

  Summary:
	Writes the formatted packet to the MAC layer and flushes it.

  Description:
	Writes the formatted packet to the MAC layer and flushes it.

  Precondition:
	None

  Parameters:
	pkt - The packet to transmit

  Returns:
  	bool - true if the packet was transmitted, false if the MAC is unlinked
  	
  Remarks:
	None
  ***************************************************************************/
bool TCPIP_IP_TransmitPacket (IP_PACKET * pkt)
{
    TCPIP_MAC_HANDLE hMac;
#if defined (TCPIP_STACK_USE_IPV6)
    uint16_t mtu;
    IPV6_HEAP_NDP_DC_ENTRY * ptrDestination;
#endif
 
    hMac = _TCPIPStackNetToMac (pkt->netIf);
    
    if (!MACIsLinked(hMac))
    {
        return false;
    }

    if (!TCPIP_IP_IsTxReady(pkt->netIf))
    {
        return false;
    }

#if defined (TCPIP_STACK_USE_IPV6)
    if (pkt->flags.addressType == IP_ADDRESS_TYPE_IPV6)
    {
        if (pkt->ipHeader.ipv6Header.DestAddress.v[0] == 0xFF)
        {
            mtu = ipv6Config[_TCPIPStackNetIx(pkt->netIf)].multicastMTU;
        }
        else
        {
            ptrDestination = TCPIP_NDP_FindRemoteNode (pkt->netIf, &pkt->ipHeader.ipv6Header.DestAddress, IPV6_HEAP_NDP_DC_ID);
            if (ptrDestination)
            {
                mtu = ptrDestination->pathMTU;
            }
            else
            {
                mtu = ipv6Config[_TCPIPStackNetIx(pkt->netIf)].linkMTU;
            }
        }
    
        if (swaps(pkt->ipHeader.ipv6Header.PayloadLength) + sizeof (IPV6_HEADER) + pkt->headerLen > mtu)
            return TCPIP_IP_TransmitPacketInFragments(pkt,mtu);
    }
#endif


    // Initialize MAC TX Write Pointer
    MACSetWritePtr (hMac, MACGetTxBaseAddr(hMac));

    #if defined (TCPIP_STACK_USE_IPV6)
    if (pkt->flags.addressType == IP_ADDRESS_TYPE_IPV6)
    {
        // Write the Ethernet Header to the MAC TX Buffer
        MACPutHeader (hMac, &pkt->remoteMACAddr, ETHERTYPE_IPV6, sizeof (IPV6_HEADER) + pkt->upperLayerHeaderLen + pkt->headerLen + pkt->payloadLen);
    }
    else
    #endif
    {
        // Write Ethernet Header to the MAC TX Buffer
        MACPutHeader (hMac, &pkt->remoteMACAddr, ETHERTYPE_IPV4, sizeof (IP_HEADER) + pkt->upperLayerHeaderLen + pkt->payloadLen);
    }

    // Write the payload data to the MAC
    TCPIP_IP_FlushDataSegments(hMac, pkt);

    // Transmit the packet
    MACFlush (hMac);

    return true;
}

#if defined (TCPIP_STACK_USE_IPV6)
/*****************************************************************************
  Function:
	bool TCPIP_IP_TransmitPacketInFragments (IP_PACKET * pkt, uint16_t mtu)

  Summary:
	Transmits a packet in fragments across a link with an MTU less than the 
    packet size.

  Description:
	Transmits a packet in fragments across a link with an MTU less than the 
    packet size.	

  Precondition:
	None

  Parameters:
	pkt - The packet
    mtu - The link MTU.

  Returns:
  	bool - true if the packet was transmitted, false otherwise.
  	
  Remarks:
	None
  ***************************************************************************/
bool TCPIP_IP_TransmitPacketInFragments (IP_PACKET * pkt, uint16_t mtu)
{
    TCPIP_MAC_HANDLE hMac;
    IPV6_FRAGMENT_HEADER * ptrFragmentHeader;
    IP_DATA_SEGMENT_HEADER * ptrSegment;
    IP_DATA_SEGMENT_HEADER * ptrFragmentablePart;
    uint16_t currentPayloadLen;
    uint16_t sentPayloadLen = 0;
    uint16_t totalPayloadLen;
    uint16_t unfragmentableLen = 0;
    uint16_t temp;
    uint16_t offsetInSegment = 0;

    hMac = _TCPIPStackNetToMac (pkt->netIf);

    if (!MACIsLinked(hMac))
    {
        return false;
    }

    if (!TCPIP_IP_IsTxReady(pkt->netIf))
    {
        return false;
    }

    ptrSegment = TCPIP_IP_AllocateDataSegmentHeader (sizeof (IPV6_FRAGMENT_HEADER));

    if (ptrSegment == NULL)
        return false;

    ptrSegment->segmentLen = sizeof (IPV6_FRAGMENT_HEADER);

    TCPIP_IP_InsertIPPacketSegment (ptrSegment, pkt, TYPE_EX_HEADER_FRAGMENT);

    ptrFragmentHeader = (IPV6_FRAGMENT_HEADER *)ptrSegment->data;

    ptrFragmentHeader->reserved = 0;
    ptrFragmentHeader->offsetM.bits.reserved2 = 0;
    ptrFragmentHeader->offsetM.bits.m = 0;
    ptrFragmentHeader->offsetM.bits.fragmentOffset = 0;
    ptrFragmentHeader->identification = swapl(fragmentId);
    fragmentId++;

    pkt->headerLen += sizeof (IPV6_FRAGMENT_HEADER);
    totalPayloadLen = pkt->payloadLen + pkt->headerLen + pkt->upperLayerHeaderLen + sizeof (IPV6_HEADER);

    // If a router specified that the path MTU is less than the minimum link MTU 
    // we just need to add a fragmentation header to the packet
    if (mtu < IPV6_MINIMUM_LINK_MTU)
    {
        if (totalPayloadLen < IPV6_DEFAULT_LINK_MTU)
            mtu = IPV6_DEFAULT_LINK_MTU;
    }

    ptrSegment = &pkt->payload;
    while ((ptrSegment != NULL) && (ptrSegment->segmentType <= TYPE_EX_HEADER_FRAGMENT))
    {
        unfragmentableLen += ptrSegment->segmentLen;
        ptrSegment = ptrSegment->nextSegment;
    }

    ptrFragmentablePart = ptrSegment;

    do
    {
        currentPayloadLen = unfragmentableLen;
        ptrSegment = ptrFragmentablePart;

        // Determine the length of the current payload
        while (ptrSegment != NULL)
        {
            if (currentPayloadLen + (ptrSegment->segmentLen - offsetInSegment) <= mtu)
            {
                currentPayloadLen += (ptrSegment->segmentLen - offsetInSegment);
            }
            else
            {
                if (mtu - currentPayloadLen > 8)
                {
                    currentPayloadLen = mtu - (mtu & 0b111);
                }
                else
                {
                    if (currentPayloadLen % 8 != 0)
                    {
                        if ((ptrSegment->segmentLen - offsetInSegment) > (8 - (currentPayloadLen & 0b111)))
                        {
                            currentPayloadLen += (8 - (currentPayloadLen & 0b111));
                        }
                        else
                        {
                            currentPayloadLen -= (currentPayloadLen & 0b111);
                        }
                    }
                }
                break;
            }
            ptrSegment = ptrSegment->nextSegment;
        }

        // Set M flag
        if (sentPayloadLen + currentPayloadLen == totalPayloadLen)
        {
            ptrFragmentHeader->offsetM.bits.m = 0;
        }
        else
        {
            ptrFragmentHeader->offsetM.bits.m = 1;
        }

        // Set fragment offset
        ptrFragmentHeader->offsetM.bits.fragmentOffset = sentPayloadLen >> 3;

        ptrFragmentHeader->offsetM.w = swaps (ptrFragmentHeader->offsetM.w);

        // Calculate new payload length        
        pkt->ipHeader.ipv6Header.PayloadLength = swaps (currentPayloadLen - sizeof (IPV6_HEADER));


        // Initialize MAC TX Write Pointer
        MACSetWritePtr (hMac, MACGetTxBaseAddr(hMac));

        // Write the Ethernet Header to the MAC TX Buffer
        MACPutHeader (hMac, &pkt->remoteMACAddr, ETHERTYPE_IPV6, currentPayloadLen);

        // Write the unfragmentable part
        ptrSegment = &pkt->payload;
        temp = unfragmentableLen;
        while (temp)
        {
            if (ptrSegment->segmentLen < temp)
            {
                MACPutArray(hMac, (uint8_t *)ptrSegment->dataLocation, ptrSegment->segmentLen);
                temp -= ptrSegment->segmentLen;
                ptrSegment = ptrSegment->nextSegment;
            }
            else
            {
                MACPutArray(hMac, (uint8_t *)ptrSegment->dataLocation, temp);
                break;
            }
        }

        // Write the fragmentable part
        ptrSegment = ptrFragmentablePart;
        temp = currentPayloadLen - unfragmentableLen;
        while (temp)
        {
            if (ptrSegment->segmentLen < temp)
            {
                MACPutArray(hMac, (uint8_t *)ptrSegment->dataLocation + offsetInSegment, ptrSegment->segmentLen);
                temp -= ptrSegment->segmentLen;
                ptrSegment = ptrSegment->nextSegment;
                offsetInSegment = 0;
            }
            else
            {
                MACPutArray(hMac, (uint8_t *)ptrSegment->dataLocation + offsetInSegment, temp);
                if (temp == ptrSegment->segmentLen)
                {
                    ptrSegment = ptrSegment->nextSegment;
                    offsetInSegment = 0;
                }
                else
                {
                    offsetInSegment = temp;
                }
                break;
            }
        }
        ptrFragmentablePart = ptrSegment;

        // Transmit the packet
        MACFlush (hMac);

        sentPayloadLen += currentPayloadLen - unfragmentableLen;
    } while (sentPayloadLen + unfragmentableLen != totalPayloadLen);
    return true;
}
#endif

/*****************************************************************************
  Function:
	void TCPIP_IP_FlushDataSegments (TCPIP_MAC_HANDLE hMac, 
        IP_PACKET * ptrPacket)

  Summary:
	Helper function for TCPIP_IP_TransmitPacket that writes all segments in 
    the packet data segment chain to the MAC layer.

  Description:
	Helper function for TCPIP_IP_TransmitPacket that writes all segments in 
    the packet data segment chain to the MAC layer.

  Precondition:
	None

  Parameters:
	hMac - The target MAC handle.
    ptrPacket - The packet.

  Returns:
  	None.
  	
  Remarks:
	None
  ***************************************************************************/
void TCPIP_IP_FlushDataSegments (TCPIP_MAC_HANDLE hMac, IP_PACKET * ptrPacket)
{
    IP_DATA_SEGMENT_HEADER * segmentHeader = &ptrPacket->payload;

    while (segmentHeader != NULL)
    {
        MACPutArray(hMac, (uint8_t *)segmentHeader->dataLocation, segmentHeader->segmentLen);
        segmentHeader = segmentHeader->nextSegment;
    }
}

/*****************************************************************************
  Function:
	IP_DATA_SEGMENT_HEADER * TCPIP_IP_AllocateDataSegmentHeader (uint16_t len)

  Summary:
	Allocates a data segment header and an optional payload

  Description:
	Allocates a data segment header and an optional payload	

  Precondition:
	None

  Parameters:
	len - Length of the optional dynamic payload to allocate for this segment.

  Returns:
  	IP_DATA_SEGMENT_HEADER * - Pointer to the new segment.
  	
  Remarks:
	None
  ***************************************************************************/
IP_DATA_SEGMENT_HEADER * TCPIP_IP_AllocateDataSegmentHeader (uint16_t len)
{
    IP_DATA_SEGMENT_HEADER * ptrSegment = (IP_DATA_SEGMENT_HEADER *)TCPIP_HEAP_Malloc (ipMemH, sizeof (IP_DATA_SEGMENT_HEADER) + len);

    if (ptrSegment == NULL)
        return NULL;

    if (len)
    {
        ptrSegment->dataLocation = (PTR_BASE)ptrSegment->data;
        ptrSegment->segmentSize = len;
        ptrSegment->segmentLen = 0;
        ptrSegment->memory = IP_DATA_DYNAMIC_BUFFER;
    }
    else
    {
        ptrSegment->memory = IP_DATA_NONE;
        ptrSegment->dataLocation = (PTR_BASE)NULL;
        ptrSegment->segmentSize = 0;
        ptrSegment->segmentLen = 0;
    }
    ptrSegment->nextSegment = NULL;

    return ptrSegment;
}

/*****************************************************************************
  Function:
	void TCPIP_IP_FreePacket (IP_PACKET * ptrPacket)

  Summary:
	Frees a TCP/IP Packet structure from dynamic memory.

  Description:
	Frees a TCP/IP Packet structure from dynamic memory.	

  Precondition:
	None

  Parameters:
	ptrPacket - The packet to free.

  Returns:
  	None
  	
  Remarks:
	None
  ***************************************************************************/
void TCPIP_IP_FreePacket (IP_PACKET * ptrPacket)
{
    if (ptrPacket == NULL)
        return;

    TCPIP_IP_FreePacketData (ptrPacket);
    TCPIP_HEAP_Free (ipMemH, ptrPacket);
}

/*****************************************************************************
  Function:
	void TCPIP_IP_FreePacketData (IP_PACKET * ptrPacket)

  Summary:
	Frees all dynamically allocated structures used by an IP_PACKET struct.

  Description:
	Frees all dynamically allocated structures used by an IP_PACKET struct.

  Precondition:
	None

  Parameters:
	ptrPacket - The packet to free.

  Returns:
  	None
  	
  Remarks:
	None
  ***************************************************************************/
void TCPIP_IP_FreePacketData (IP_PACKET * ptrPacket)
{
    IP_DATA_SEGMENT_HEADER * ptrSegment;
    IP_DATA_SEGMENT_HEADER * ptrSegment2;

    if (ptrPacket == NULL)
        return;

    // Set the initial segment to the segment after the IP header (which shouldn't be deallocated
    ptrSegment = ptrPacket->payload.nextSegment;

    while (ptrSegment != NULL)
    {
        ptrSegment2 = ptrSegment->nextSegment;
        TCPIP_HEAP_Free (ipMemH, ptrSegment);
        ptrSegment = ptrSegment2;
    }
}

/*********************************************************************
 * Function:        bool TCPIP_IP_GetHeader( NET_CONFIG* pNet,
 *                                    IP_ADDR    *localIP,
 *                                    NODE_INFO  *remote,
 *                                    uint8_t        *Protocol,
 *                                    uint16_t        *len)
 *
 * PreCondition:    MACGetHeader() == true
 *
 * Input:           pNet       - the interface for multihomed hosts
 *                  localIP     - Local node IP Address as received
 *                                in current IP header.
 *                                If this information is not required
 *                                caller may pass NULL value.
 *                  remote      - Remote node info
 *                  Protocol    - Current packet protocol
 *                  len         - Current packet data length
 *
 * Output:          true, if valid packet was received
 *                  false otherwise
 *
 * Side Effects:    None
 *
 * Note:            Only one IP message can be received.
 *                  Caller may not transmit and receive a message
 *                  at the same time.
 *
 ********************************************************************/
bool TCPIP_IP_GetHeader(NET_CONFIG* pNet, IP_ADDR *localIP, NODE_INFO *remote, uint8_t *protocol, uint16_t *len)
{
    uint8_t IPHeaderLen;
    TCPIP_UINT16_VAL    CalcChecksum;
    IP_HEADER   header;

#if defined(NON_MCHP_MAC)
    TCPIP_UINT16_VAL    ReceivedChecksum;
    uint16_t        checksums[2];
    uint8_t        optionsLen;
	#define MAX_OPTIONS_LEN     (40u)            // As per RFC 791.
    uint8_t        options[MAX_OPTIONS_LEN];
#endif
    TCPIP_MAC_HANDLE hMac;

    hMac = _TCPIPStackNetToMac(pNet);

    // Read IP header.
    MACGetArray(hMac, (uint8_t*)&header, sizeof(header));

    // Make sure that this is an IPv4 packet.
    if((header.VersionIHL & 0xf0) != IPv4_VERSION)
    	return false;

	// Throw this packet away if it is a fragment.  
	// We don't have enough RAM for IP fragment reconstruction.
	if(header.FragmentInfo & 0xFF1F)
		return false;

	IPHeaderLen = (header.VersionIHL & 0x0f) << 2;


#if !defined(NON_MCHP_MAC)
	// Validate the IP header.  If it is correct, the checksum 
	// will come out to 0x0000 (because the header contains a 
	// precomputed checksum).  A corrupt header will have a 
	// nonzero checksum.
	CalcChecksum.Val = MACCalcRxChecksum(hMac, 0, IPHeaderLen);

	// Seek to the end of the IP header
	MACSetReadPtrInRx(hMac, IPHeaderLen);

    if(CalcChecksum.Val)
#else
    // Calculate options length in this header, if there is any.
    // IHL is in terms of numbers of 32-bit words; i.e. actual
    // length is 4 times IHL.
    optionsLen = IPHeaderLen - sizeof(header);

    // If there is any option(s), read it so that we can include them
    // in checksum calculation.
    if ( optionsLen > MAX_OPTIONS_LEN )
        return false;

    if ( optionsLen > 0u )
        MACGetArray(hMac, options, optionsLen);

    // Save header checksum; clear it and recalculate it ourselves.
    ReceivedChecksum.Val = header.HeaderChecksum;
    header.HeaderChecksum = 0;

    // Calculate checksum of header including options bytes.
    checksums[0] = ~CalcIPChecksum((uint8_t*)&header, sizeof(header));

    // Calculate Options checksum too, if they are present.
    if ( optionsLen > 0u )
        checksums[1] = ~CalcIPChecksum((uint8_t*)options, optionsLen);
    else
        checksums[1] = 0;

    CalcChecksum.Val  = CalcIPChecksum((uint8_t*)checksums,
                                            2 * sizeof(uint16_t));

    // Make sure that checksum is correct
    if ( ReceivedChecksum.Val != CalcChecksum.Val )
#endif
    {
        // Bad packet. The function caller will be notified by means of the false 
        // return value and it should discard the packet.
        return false;
    }

    // Network to host conversion.
    SwapIPHeader(&header);

    // If caller is intrested, return destination IP address
    // as seen in this IP header.
    if ( localIP )
        localIP->Val    = header.DestAddress.Val;

    remote->IPAddr.Val  = header.SourceAddress.Val;
    *protocol           = header.Protocol;
    *len 				= header.TotalLength - IPHeaderLen;

    return true;
}

#if defined (TCPIP_STACK_USE_IPV6)
/*****************************************************************************
  Function:
	bool TCPIP_IPV6_GetHeader(NET_CONFIG * pNetIf, IPV6_ADDR * localIPAddr, 
        IPV6_ADDR * remoteIPAddr, uint8_t *protocol, uint16_t *len, 
        uint8_t * hopLimit)

  Summary:
	Reads IPv6 header information from the MAC.

  Description:
	Reads IPv6 header information from the MAC.

  Precondition:
	None

  Parameters:
	pNetIf - The interface to check for a header.
    localIPAddr - The incoming packet's destination address.
    remoteIPAddr - The incoming packet's source address.
    protocol - Return value for the next header protocol.
    len - Return value for the payload length.
    hopLimit - Return value for the hop limit of the packet

  Returns:
  	bool - True if a packet header was read, false otherwise.
  	
  Remarks:
	None
  ***************************************************************************/
bool TCPIP_IPV6_GetHeader(NET_CONFIG * pNetIf, IPV6_ADDR * localIPAddr, IPV6_ADDR * remoteIPAddr, uint8_t *protocol, uint16_t *len, uint8_t * hopLimit)
{
    IPV6_HEADER header;
    TCPIP_MAC_HANDLE hMac;

    hMac = _TCPIPStackNetToMac(pNetIf);

    // Read IP header.
    MACGetArray(hMac, (uint8_t*)&header, sizeof(header));

    // Make sure that this is an IPv6 packet.
    if ((header.V_T_F & 0x000000F0) != IPv6_VERSION)
    	return false;

    *hopLimit = header.HopLimit;

    *len = swaps(header.PayloadLength);

    *protocol = header.NextHeader;

    memcpy(localIPAddr, &header.DestAddress, sizeof (IPV6_ADDR));

    memcpy(remoteIPAddr, &header.SourceAddress, sizeof (IPV6_ADDR));

    return true;
}
#endif

/*********************************************************************
 * Function:        IPSetRxBuffer(NET_CONFIG* pNet, uint16_t Offset)
 *
 * PreCondition:    None 
 *
 * Input:           Offset from beginning of IP data field
 *
 * Output:          Next Read/Write access to receive buffer is
 *                  set to Offset 
 *
 * Side Effects:    None
 *
 * Note:            None
 *
 ********************************************************************/
void TCPIP_IPV4_SetRxBuffer(NET_CONFIG* pNet, uint16_t Offset) 
{
	MACSetReadPtrInRx(_TCPIPStackNetToMac(pNet), Offset + sizeof (IP_HEADER));
}

#if defined (TCPIP_STACK_USE_IPV6)
/*****************************************************************************
  Function:
    void TCPIP_IPV6_SetRxBuffer(NET_CONFIG* pNet, uint16_t Offset) 

  Summary:
	Sets the read pointer for the current RX buffer (IPv6 version).

  Description:
	Sets the read pointer for the current RX buffer to the specified offset
    after an IPv6 header.	

  Precondition:
	None

  Parameters:
	pNet - The interface to set the pointer for.
    Offset - The offset to set.

  Returns:
  	None
  	
  Remarks:
	None
  ***************************************************************************/
void TCPIP_IPV6_SetRxBuffer(NET_CONFIG* pNet, uint16_t Offset) 
{
	MACSetReadPtrInRx(_TCPIPStackNetToMac(pNet), Offset + sizeof (IPV6_HEADER));
}
#endif

/*****************************************************************************
  Function:
	static void SwapIPHeader(IP_HEADER* h)

  Summary:
	Swaps the endinaness of parameters in an IPv4 header.

  Description:
	Swaps the endinaness of parameters in an IPv4 header.	

  Precondition:
	None

  Parameters:
	h - The header

  Returns:
  	None
  	
  Remarks:
	None
  ***************************************************************************/
static void SwapIPHeader(IP_HEADER* h)
{
    h->TotalLength      = swaps(h->TotalLength);
    h->Identification   = swaps(h->Identification);
    h->HeaderChecksum   = swaps(h->HeaderChecksum);
}

#if defined (TCPIP_STACK_USE_IPV6)
/*****************************************************************************
  Function:
	void IPv6FreeConfigLists (IPV6_INTERFACE_CONFIG * pNetIf)

  Summary:
	Frees all dynamically allocated structures from linked lists used 
    for IPv6 configuration on an interface.

  Description:
	Frees all dynamically allocated structures from linked lists used 
    for IPv6 configuration on an interface.	

  Precondition:
	None

  Parameters:
	pNetIf - The interface.

  Returns:
  	None
  	
  Remarks:
	None
  ***************************************************************************/
void IPv6FreeConfigLists (IPV6_INTERFACE_CONFIG * pNetIf)
{
    TCPIP_NDP_SingleListFree(&pNetIf->listDestinationCache);
    TCPIP_NDP_SingleListFree(&pNetIf->listNeighborCache);
    TCPIP_NDP_SingleListFree(&pNetIf->listDefaultRouter);
    TCPIP_NDP_SingleListFree(&pNetIf->listPrefixList);
    TCPIP_NDP_SingleListFree(&pNetIf->fragments);
    TCPIP_NDP_DoubleListFree(&pNetIf->listIpv6UnicastAddresses);
    TCPIP_NDP_DoubleListFree(&pNetIf->listIpv6MulticastAddresses);
    TCPIP_NDP_DoubleListFree(&pNetIf->listIpv6TentativeAddresses);
}
#endif

#if defined (TCPIP_STACK_USE_IPV6)
/*****************************************************************************
  Function:
	IPV6_ADDR_STRUCT * TCPIP_IPV6_FindAddress(NET_CONFIG * pNetIf, IPV6_ADDR * addr, 
        unsigned char listType)

  Summary:
	Finds a local IPv6 address.

  Description:
	This function finds a local address in the list of tentative, unicast, 
    or multicast addresses.

  Precondition:
	None

  Parameters:
	pNetIf - The interface to check for the address.
    addr - The address to check for.
    listType - IPV6_ADDR_TYPE_UNICAST_TENTATIVE, IPV6_ADDR_TYPE_UNICAST, 
            IPV6_ADDR_TYPE_MULTICAST

  Returns:
  	IPV6_ADDR_STRUCT * - Pointer to the found address, or NULL
  	
  Remarks:
	None
  ***************************************************************************/
IPV6_ADDR_STRUCT * TCPIP_IPV6_FindAddress(NET_CONFIG * pNetIf, IPV6_ADDR * addr, unsigned char listType)
{
    IPV6_ADDR_STRUCT * nextEntryPointer; 
    int netIx;

    if (!pNetIf)
    {
        return NULL;
    }
    else
    {
        netIx = _TCPIPStackNetIx (pNetIf);
    }

    switch (listType)
    {
        case IPV6_ADDR_TYPE_UNICAST_TENTATIVE:
            nextEntryPointer = (IPV6_ADDR_STRUCT *)ipv6Config[netIx].listIpv6TentativeAddresses.head;
            break;
        case IPV6_ADDR_TYPE_UNICAST:
            nextEntryPointer = (IPV6_ADDR_STRUCT *)ipv6Config[netIx].listIpv6UnicastAddresses.head;
            break;
        case IPV6_ADDR_TYPE_MULTICAST:
            nextEntryPointer = (IPV6_ADDR_STRUCT *)ipv6Config[netIx].listIpv6MulticastAddresses.head;
            break;

        default:
            nextEntryPointer = 0;
            break;
    }

    while (nextEntryPointer != NULL)
    {
        if (!memcmp (addr, &(nextEntryPointer->address), sizeof (IPV6_ADDR)))
        {
            return nextEntryPointer;
        }
        nextEntryPointer = nextEntryPointer->next;
    }
    return NULL;
}

/*****************************************************************************
  Function:
    IPV6_ADDR_STRUCT * TCPIP_IPV6_FindSolicitedNodeMulticastAddress(
        NET_CONFIG * pNetIf, IPV6_ADDR * addr, unsigned char listType)

  Summary:
    Finds a unicast address based on a given solicited-node multicast address

  Description:
    Finds a unicast address based on a given solicited-node multicast address

  Precondition:
	None

  Parameters:
	pNetIf - The interface to check for the address.
    addr - The address to check for.
    listType - IPV6_ADDR_TYPE_UNICAST_TENTATIVE or IPV6_ADDR_TYPE_UNICAST

  Returns:
  	IPV6_ADDR_STRUCT * - Pointer to the found address, or NULL
  	
  Remarks:
	None
  ***************************************************************************/
IPV6_ADDR_STRUCT * TCPIP_IPV6_FindSolicitedNodeMulticastAddress(NET_CONFIG * pNetIf, IPV6_ADDR * addr, unsigned char listType)
{
    IPV6_ADDR_STRUCT * nextEntryPointer; 
    int netIx;

    if (!pNetIf)
    {
        return NULL;
    }
    else
    {
        netIx = _TCPIPStackNetIx (pNetIf);
    }

    switch (listType)
    {
        case IPV6_ADDR_TYPE_UNICAST_TENTATIVE:
            nextEntryPointer = (IPV6_ADDR_STRUCT *)ipv6Config[netIx].listIpv6TentativeAddresses.head;
            break;
        case IPV6_ADDR_TYPE_UNICAST:
            nextEntryPointer = (IPV6_ADDR_STRUCT *)ipv6Config[netIx].listIpv6UnicastAddresses.head;
            break;

        default:
            nextEntryPointer = 0;
    }

    while (nextEntryPointer != NULL)
    {
        if (!memcmp (&addr->v[13], &(nextEntryPointer->address.v[13]), 3))
        {
            return nextEntryPointer;
        }
        nextEntryPointer = nextEntryPointer->next;
    }
    return NULL;
}
#endif

#if defined (TCPIP_STACK_USE_IPV6)
/*****************************************************************************
  Function:
	IPV6_ADDR_STRUCT * TCPIP_IPV6_AddMulticastListener (NET_CONFIG * pNetIf, 
        IPV6_ADDR * address)

  Summary:
	Adds a multicast listener to an interface.

  Description:
	Adds a multicast listener to an interface.	

  Precondition:
	None

  Parameters:
	pNetIf - The interface to add the address to.
    address - The new listener

  Returns:
  	IPV6_ADDR_STRUCT * - Pointer to the new listener, or NULL
  	
  Remarks:
	None
  ***************************************************************************/
IPV6_ADDR_STRUCT * TCPIP_IPV6_AddMulticastListener (TCPIP_NET_HANDLE hNet, IPV6_ADDR * address)
{
    IPV6_ADDR_STRUCT * entryLocation;
    IPV6_ADDRESS_TYPE addressType;
    NET_CONFIG* pNetIf = (NET_CONFIG*)hNet;
    
    if (!pNetIf)
    {
        return NULL;
    }

    if ((entryLocation = TCPIP_IPV6_FindAddress (pNetIf, address, IPV6_ADDR_TYPE_MULTICAST)) == NULL)
    {
        entryLocation = (IPV6_ADDR_STRUCT *) TCPIP_HEAP_Malloc (ipMemH, sizeof (IPV6_ADDR_STRUCT));

        if (entryLocation != NULL)
        {
            memcpy (&entryLocation->address, address, sizeof (IPV6_ADDR));
            entryLocation->flags.type = IPV6_ADDR_TYPE_MULTICAST;
            addressType = TCPIP_IPV6_GetAddressType(pNetIf, address);
            entryLocation->flags.scope = addressType.bits.scope;
            TCPIP_NDP_LinkedListEntryInsert (pNetIf, entryLocation, IPV6_HEAP_ADDR_MULTICAST_ID);
            TCPIP_IPV6_NotifyClients(pNetIf, IPV6_EVENT_ADDRESS_ADDED);
        }
    }

    return entryLocation;
}
#endif

#if defined (TCPIP_STACK_USE_IPV6)
/*****************************************************************************
  Function:
	void TCPIP_IPV6_QueuePacket (IPV6_HEAP_NDP_NC_ENTRY * neighborPtr, IP_PACKET * pkt)

  Summary:
	Queues a packet for future transmission.

  Description:
	Queues a packet for future transmission. The packet will be transmitted 
    when its link-layer address has been resolved.

  Precondition:
	None

  Parameters:
	neighborPointer - The neighbor that this packet will be sent to.
    pkt - The packet to queue.

  Returns:
  	None
  	
  Remarks:
	None
  ***************************************************************************/
void TCPIP_IPV6_QueuePacket (IPV6_HEAP_NDP_NC_ENTRY * neighborPtr, IP_PACKET * pkt)
{
    IP_PACKET * packetPtr = neighborPtr->queuedPacketPointer;

#if defined (IPV6_QUEUE_ONLY_ONE_PACKET_PER_NEIGHBOR)
    if (packetPtr != NULL)
    {
        TCPIP_IPV6_RemoveQueuedPacket (neighborPtr, packetPtr);
    }
    neighborPtr->queuedPacketPointer = pkt;
#else
    if (packetPtr == NULL)
    {
        neighborPtr->queuedPacketPointer = pkt;
    }
    else
    {
        while (packetPtr->nextPacket != NULL)
        {
            packetPtr = packetPtr->nextPacket;
        }
        packetPtr->nextPacket = pkt;
    }
#endif

    pkt->nextPacket = NULL;
    pkt->flags.queued = true;
}
#endif

#if defined (TCPIP_STACK_USE_IPV6)
/*****************************************************************************
  Function:
	void TCPIP_IPV6_RemoveQueuedPacket (IPV6_HEAP_NDP_NC_ENTRY * neighborPointer, 
        IP_PACKET * pkt)

  Summary:
	Removes a queued packet from a neighbor cache entry and deallocates it 
    or returns it to its socket.

  Description:
	Removes a queued packet from a neighbor cache entry and deallocates it 
    or returns it to its socket.	

  Precondition:
	None

  Parameters:
	neighborPointer - The neighbor to remove the queued packet from.
    pkt - The packet.

  Returns:
  	None
  	
  Remarks:
	None
  ***************************************************************************/
void TCPIP_IPV6_RemoveQueuedPacket (IPV6_HEAP_NDP_NC_ENTRY * neighborPointer, IP_PACKET * pkt)
{
    IP_PACKET * tempPacket = neighborPointer->queuedPacketPointer;

    if (tempPacket == NULL)
        return;

    if (tempPacket == pkt)
    {
        neighborPointer->queuedPacketPointer = tempPacket->nextPacket;
        if (!pkt->flags.packetBound)
        {
            TCPIP_IP_FreePacket (pkt);
        }
        else
        {
            pkt->flags.queued = false;
        }
        return;
    }

    while ((tempPacket->nextPacket != pkt) && (tempPacket->nextPacket != NULL))
        tempPacket = tempPacket->nextPacket;

    if (tempPacket->nextPacket == NULL)
        return;

    tempPacket = tempPacket->nextPacket->nextPacket;

    if (!pkt->flags.packetBound)
    {
        TCPIP_IP_FreePacket (pkt);
    }
    else
    {
        pkt->flags.queued = false;
    }
}
#endif

#if defined (TCPIP_STACK_USE_IPV6)
/*****************************************************************************
  Function:
	IPV6_ADDRESS_TYPE TCPIP_IPV6_GetAddressType (NET_CONFIG * pNetIf, 
        IPV6_ADDR * address)

  Summary:
	Gets the scope and type of an IPv6 address.

  Description:
	Gets the scope and type of an IPv6 address.	

  Precondition:
	None

  Parameters:
	pNetIf - The interface (used to check for link-locality)
    address - The address to check.

  Returns:
  	IPV6_ADDRESS_TYPE - The address scope and type
  	
  Remarks:
	None
  ***************************************************************************/
IPV6_ADDRESS_TYPE TCPIP_IPV6_GetAddressType (NET_CONFIG * pNetIf, IPV6_ADDR * address)
{
    uint8_t b;
    IPV6_ADDRESS_TYPE returnVal;

    if (address->v[0] == 0xFF)
    {
        // First byte == 0xFF -> multicast
        returnVal.bits.type = IPV6_ADDR_TYPE_MULTICAST;
        b = address->v[1];
        b &= 0x0F;
        switch (b)
        {
            case 1:
            case 2:
            case 4:
            case 5:
            case 8:
            case 0x0E:
                returnVal.bits.scope = b;
                break;
            default:
                returnVal.bits.scope = IPV6_ADDR_SCOPE_UNKNOWN;
                break;
        }
    }
    else
    {
        // First byte != 0xFF -> unicast or anycast
        // Impossible to determine if it's an anycast addr unless
        // it's specifically identified as one somewhere
        returnVal.bits.type = IPV6_ADDR_TYPE_UNICAST;

        if (((address->v[0] == 0xFE) && (address->v[1] == 0x80)) || TCPIP_NDP_PrefixOnLinkStatus(pNetIf, address))
        {
            returnVal.bits.scope = IPV6_ADDR_SCOPE_LINK_LOCAL;
        }
        // Compare to loopback address
        else if ((address->d[0] == 0x00000000) && (address->d[1] == 0x00000000) && (address->d[2] == 0x00000000) && (address->v[12] == 0x00) && \
                     (address->v[13] == 0x00) && (address->v[14] == 0x00) && (address->v[15] == 0x01))
        {
            // This counts as link-local unicast
            returnVal.bits.scope = IPV6_ADDR_SCOPE_INTERFACE_LOCAL;
        }
        else
        {
            returnVal.bits.scope = IPV6_ADDR_SCOPE_GLOBAL;
        }
    }

    return returnVal;
}
#endif

#if defined (TCPIP_STACK_USE_IPV6)
/*****************************************************************************
  Function:
	void IPv6RemoveUnicastAddress (NET_CONFIG * pNetIf, IPV6_ADDR * address)

  Summary:
	Removed a configured unicast address from an interface.

  Description:
	Removed a configured unicast address from an interface.	

  Precondition:
	None

  Parameters:
	pNetIf - The interface.
    address - The address

  Returns:
  	None
  	
  Remarks:
	None
  ***************************************************************************/
void IPv6RemoveUnicastAddress (NET_CONFIG * pNetIf, IPV6_ADDR * address)
{
    IPV6_ADDR_STRUCT * entryLocation;

    entryLocation = TCPIP_IPV6_FindAddress (pNetIf, address, IPV6_ADDR_TYPE_UNICAST);
    if (entryLocation != NULL)
    {
        TCPIP_NDP_LinkedListEntryRemove (pNetIf, entryLocation, IPV6_HEAP_ADDR_UNICAST_ID);
        TCPIP_IPV6_NotifyClients(pNetIf, IPV6_EVENT_ADDRESS_REMOVED);
    }
}
#endif

#if defined (TCPIP_STACK_USE_IPV6)
/*****************************************************************************
  Function:
	void IPv6RemoveMulticastListener (NET_CONFIG * pNetIf, IPV6_ADDR * address)

  Summary:
	Removes a multicast listener from a given interface.

  Description:
	Removes a multicast listener from a given interface.	

  Precondition:
	None

  Parameters:
	pNetIf - The interface
    address - The address

  Returns:
  	None
  	
  Remarks:
	None
  ***************************************************************************/
void IPv6RemoveMulticastListener (NET_CONFIG * pNetIf, IPV6_ADDR * address)
{
    IPV6_ADDR_STRUCT * entryLocation;

    entryLocation = TCPIP_IPV6_FindAddress (pNetIf, address, IPV6_ADDR_TYPE_MULTICAST);
    if (entryLocation != NULL)
    {
        TCPIP_NDP_LinkedListEntryRemove (pNetIf, entryLocation, IPV6_HEAP_ADDR_MULTICAST_ID);
        TCPIP_IPV6_NotifyClients(pNetIf, IPV6_EVENT_ADDRESS_REMOVED);
    }
}
#endif

#if defined (TCPIP_STACK_USE_IPV6)
/*****************************************************************************
  Function:
	IPV6_ADDR_STRUCT * TCPIP_IPV6_AddUnicastAddress (NET_CONFIG * pNetIf, 
        IPV6_ADDR * address, uint8_t skipProcessing)

  Summary:
	Adds a unicast address to a specified interface

  Description:
	Adds a unicast address to a specified interface.  Starts duplicate address 
    detection if necessary.

  Precondition:
	None

  Parameters:
	pNetIf - The interface to add the address to.
    address - The address to add.
    skipProcessing - true to skip Duplicate address detection, false otherwise

  Returns:
  	IPV6_ADDR_STRUCT * - Pointer to the structure of the newly allocated
        address
   
  Remarks:
	None
  ***************************************************************************/
IPV6_ADDR_STRUCT * TCPIP_IPV6_AddUnicastAddress (NET_CONFIG * pNetIf, IPV6_ADDR * address, uint8_t skipProcessing)
{
    IPV6_ADDR_STRUCT * entryLocation;
    unsigned char label, precedence, prefixLen;
    IPV6_ADDRESS_TYPE i;

    if (((entryLocation = TCPIP_IPV6_FindAddress (pNetIf, address, IPV6_ADDR_TYPE_UNICAST)) == NULL) && 
        ((entryLocation = TCPIP_IPV6_FindAddress (pNetIf, address, IPV6_ADDR_TYPE_UNICAST_TENTATIVE)) == NULL))
    {
        entryLocation = (IPV6_ADDR_STRUCT *) TCPIP_HEAP_Malloc (ipMemH, sizeof (IPV6_ADDR_STRUCT));

        if (entryLocation != NULL)
        {
            memcpy (&entryLocation->address, address, sizeof (IPV6_ADDR));
            i = TCPIP_IPV6_GetAddressType (pNetIf, address);
            entryLocation->flags.type = i.bits.type;
            entryLocation->flags.scope = i.bits.scope;
            if (TCPIP_IPV6_DAS_GetPolicy(address, &label, &precedence, &prefixLen))
            {
                entryLocation->flags.precedence = precedence;
                entryLocation->flags.label = label & 0x0F;
            }
            else
            {
                entryLocation->flags.precedence = 0x00;
                entryLocation->flags.label = 0xF;                    
            }
            entryLocation->flags.temporary = 0;

            // Set the Stateless Address Autoconfiguration variables to default values.
            // The Stateless Address AutoConfiguration function will set it to something else
            // if necessary.
            entryLocation->validLifetime = 0xFFFFFFFF;
            entryLocation->preferredLifetime = 0xFFFFFFFF;
            entryLocation->prefixLen = 0;
            // The skipProcessing flag indicates that the address doesn't need duplicate address
            // detection or an associated solicited node multicast address.
            // This can be used to add loopback addresses, for example.
            if (!skipProcessing)
            {
                TCPIP_NDP_LinkedListEntryInsert (pNetIf, entryLocation, IPV6_HEAP_ADDR_UNICAST_TENTATIVE_ID);
                if (TCPIP_NDP_DAD_DetectDuplicateAddress (pNetIf, entryLocation) == -1)
                {
                    TCPIP_NDP_LinkedListEntryRemove (pNetIf, entryLocation, IPV6_HEAP_ADDR_UNICAST_TENTATIVE_ID);
                    entryLocation = NULL;
                }
            }
            else
            {
                TCPIP_NDP_LinkedListEntryInsert (pNetIf, entryLocation, IPV6_HEAP_ADDR_UNICAST_ID);
                TCPIP_IPV6_NotifyClients(pNetIf, IPV6_EVENT_ADDRESS_ADDED);
            }
        }
    }
    return entryLocation;
}
#endif

#if defined (TCPIP_STACK_USE_IPV6)
/*****************************************************************************
  Function:
	uint8_t TCPIP_IPV6_ProcessHopByHopOptionsHeader (NET_CONFIG * pNetIf, 
        uint8_t * nextHeader, uint16_t * length)

  Summary:
	Processes an IPv6 Hop-by-hop Extension header.

  Description:
	Processes an IPv6 Hop-by-hop Extension header.	

  Precondition:
	None

  Parameters:
	pNetIf - The interface the packet with this header was received on.
    nextHeader - Return value for the next header.
    length - Return value for the header length.

  Returns:
  	uint8_t - An action code for the TCPIP_IPV6_Process function.
  	
  Remarks:
	None
  ***************************************************************************/
uint8_t TCPIP_IPV6_ProcessHopByHopOptionsHeader (NET_CONFIG * pNetIf, uint8_t * nextHeader, uint16_t * length)
{
    uint8_t headerLength;
    uint8_t option;
    uint8_t optionLength;
    uint8_t data[8];
    uint8_t i;
    uint8_t j;
    TCPIP_MAC_HANDLE hMac = _TCPIPStackNetToMac (pNetIf);

    TCPIP_IPV6_GetOptionHeader (hMac, data, 1);

    *nextHeader = data[0];
    headerLength = data[1] + 1;
    *length = (uint16_t)headerLength << 3;

    option = data[2];

    i = 3;

    do
    {
        switch (option)
        {
            case IPV6_TLV_PAD_1:
                // If this option is present, let the post-switch code load new
                // data if necessary and get the next option
                break;
            case IPV6_TLV_PAD_N:
                optionLength = data[i++];
                if (optionLength <= (8-i))
                {
                    i+= optionLength;
                }
                else
                {
                    optionLength -= (8-i);
                    j = optionLength >> 3;
                    headerLength -= j;
                    while (j--)
                        TCPIP_IPV6_GetOptionHeader (hMac, data, 1);
                    optionLength -= (j << 3);
                    i = optionLength;
                }
                break;
            case IPV6_TLV_HBHO_PAYLOAD_JUMBOGRAM:
                break;
            case IPV6_TLV_HBHO_ROUTER_ALERT:
                break;
            default:
                switch (((IPV6_TLV_OPTION_TYPE)option).bits.unrecognizedAction)
                {
                    case IPV6_TLV_UNREC_OPT_SKIP_OPTION:
                        // Ignore this option (treat it like a padding option)
                        optionLength = data[i++];
                        if (optionLength <= (8-i))
                        {
                            i+= optionLength;
                        }
                        else
                        {
                            optionLength -= (8-i);
                            j = optionLength >> 3;
                            headerLength -= j;
                            while (j--)
                                TCPIP_IPV6_GetOptionHeader (hMac, data, 1);
                            optionLength -= (j << 3);
                            i = optionLength;
                        }
                        break;
                    case IPV6_TLV_UNREC_OPT_DISCARD_SILENT:
                        *length = *length - (headerLength << 3) + --i;
                        return IPV6_ACTION_DISCARD_SILENT;
                        break;
                    case IPV6_TLV_UNREC_OPT_DISCARD_PP:
                        *length = *length - (headerLength << 3) + --i;
                        return IPV6_ACTION_DISCARD_PP_2;
                        break;
                    case IPV6_TLV_UNREC_OPT_DISCARD_PP_NOT_MC:
                        *length = *length - (headerLength << 3) + --i;
                        return IPV6_ACTION_DISCARD_PP_2_NOT_MC;
                        break;
                }
                break;
        }
        if (i == 8)
        {
            headerLength--;
            i = 0;
            if (headerLength != 0)
                TCPIP_IPV6_GetOptionHeader (hMac, data, 1);
        }
        option = data[i++];
    }while (headerLength);

    return IPV6_ACTION_NONE;
}
#endif

#if defined (TCPIP_STACK_USE_IPV6)
/*****************************************************************************
  Function:
	uint8_t TCPIP_IPV6_ProcessDestinationOptionsHeader (NET_CONFIG * pNetIf, 
        uint8_t * nextHeader, uint16_t * length)

  Summary:
	Processes an IPv6 Destination Extension header.

  Description:
	Processes an IPv6 Destination Extension header.	

  Precondition:
	None

  Parameters:
	pNetIf - The interface the packet with this header was received on.
    nextHeader - Return value for the next header.
    length - Return value for the header length.

  Returns:
  	uint8_t - An action code for the TCPIP_IPV6_Process function.
  	
  Remarks:
	None
  ***************************************************************************/
uint8_t TCPIP_IPV6_ProcessDestinationOptionsHeader (NET_CONFIG * pNetIf, uint8_t * nextHeader, uint16_t * length)
{
    uint8_t headerLength;
    uint8_t option;
    uint8_t optionLength;
    uint8_t data[8];
    uint8_t i;
    uint8_t j;
    TCPIP_MAC_HANDLE hMac = _TCPIPStackNetToMac (pNetIf);

    TCPIP_IPV6_GetOptionHeader (hMac, data, 1);

    *nextHeader = data[0];
    headerLength = data[1] + 1;
    *length = (uint16_t)headerLength << 3;

    option = data[2];

    i = 3;

    do
    {
        switch (option)
        {
            // There are only two options current defined, and they're padding options
            case IPV6_TLV_PAD_1:
                // If this option is present, let the post-switch code load new
                // data if necessary and get the next option
                break;
            case IPV6_TLV_PAD_N:
                optionLength = data[i++];
                if (optionLength <= (8-i))
                {
                    i+= optionLength;
                }
                else
                {
                    optionLength -= (8-i);
                    j = optionLength >> 3;
                    headerLength -= (j + 1);
                    while (j--)
                        TCPIP_IPV6_GetOptionHeader (hMac, data, 1);
                    optionLength -= (j << 3);
                    i = optionLength;
                }
                break;
            default:
                switch (((IPV6_TLV_OPTION_TYPE)option).bits.unrecognizedAction)
                {
                    case IPV6_TLV_UNREC_OPT_SKIP_OPTION:
                        // Ignore this option (treat it like a padding option)
                        optionLength = data[i++];
                        if (optionLength <= (8-i))
                        {
                            i+= optionLength;
                        }
                        else
                        {
                            optionLength -= (8-i);
                            j = optionLength >> 3;
                            headerLength -= (j + 1);
                            while (j--)
                                TCPIP_IPV6_GetOptionHeader (hMac, data, 1);
                            optionLength -= (j << 3);
                            i = optionLength;
                        }
                        break;
                    case IPV6_TLV_UNREC_OPT_DISCARD_SILENT:
                        // Return the offset of the error in this header in the headerLength parameter
                        *length = *length - (headerLength << 3) + --i;
                        return IPV6_ACTION_DISCARD_SILENT;
                        break;
                    case IPV6_TLV_UNREC_OPT_DISCARD_PP:
                        // Return the offset of the error in this header in the headerLength parameter
                        *length = *length - (headerLength << 3) + --i;
                        return IPV6_ACTION_DISCARD_PP_2;
                        break;
                    case IPV6_TLV_UNREC_OPT_DISCARD_PP_NOT_MC:
                        // Return the offset of the error in this header in the headerLength parameter
                        *length = *length - (headerLength << 3) + --i;
                        return IPV6_ACTION_DISCARD_PP_2_NOT_MC;
                        break;
                }
                break;
        }
        if (i == 8)
        {
            headerLength--;
            i = 0;
            if (headerLength != 0)
                TCPIP_IPV6_GetOptionHeader (hMac, data, 1);
        }
        option = data[i++];
    }while (headerLength);

    return IPV6_ACTION_NONE;
}
#endif

#if defined (TCPIP_STACK_USE_IPV6)
/*****************************************************************************
  Function:
	void TCPIP_IPV6_FreeFragmentBuffer (IPV6_FRAGMENT_BUFFER * ptrFragment)

  Summary:
	Frees a fragment processing buffer.

  Description:
	Frees a fragment processing buffer.	

  Precondition:
	None

  Parameters:
    ptrFragment - The fragment to free.

  Returns:
  	None
  	
  Remarks:
	None
  ***************************************************************************/
void TCPIP_IPV6_FreeFragmentBuffer (IPV6_FRAGMENT_BUFFER * ptrFragment)
{
    if (ptrFragment == NULL)
        return;

    TCPIP_HEAP_Free (ipMemH, ptrFragment);
}
#endif

#if defined (TCPIP_STACK_USE_IPV6)
/*****************************************************************************
  Function:
    uint8_t TCPIP_IPV6_ProcessFragmentationHeader (NET_CONFIG * pNetIf, 
        IPV6_ADDR * remoteIP, IPV6_ADDR * localIP, uint8_t * nextHeader, 
        uint16_t dataCount, uint16_t headerLen, MAC_ADDR * remoteMACAddr, 
        uint16_t previousHeaderLen)

  Summary:
	Processes an IPv6 Fragmentation Extension header.

  Description:
	Processes an IPv6 Fragmentation Extension header.  This will usually results
    in the packet data being cached in a fragment buffer.

  Precondition:
	None

  Parameters:
	pNetIf - The interface the packet with this header was received on.
    remoteIP - The packet's source IP address
    localIP - The packet's destination IP address
    nextHeader - Return value for the next header
    dataCount - Length of the fragment header and everything after it
    headerLen - Length of all headers before the fragmentation header (including
        the IPv6 header)
    remoteMACAddr - The sender's MAC address
    previousHeaderLen - Length of the previous extension header (for finding 
        next header value)

  Returns:
  	uint8_t - An action code for the TCPIP_IPV6_Process function.
  	
  Remarks:
	None
  ***************************************************************************/
uint8_t TCPIP_IPV6_ProcessFragmentationHeader (NET_CONFIG * pNetIf, IPV6_ADDR * remoteIP, IPV6_ADDR * localIP, uint8_t * nextHeader, uint16_t dataCount, uint16_t headerLen, MAC_ADDR * remoteMACAddr, uint16_t previousHeaderLen)
{
    IPV6_FRAGMENT_BUFFER * ptrFragment;
    IPV6_FRAGMENT_HEADER fragmentHeader;
    int netIx;

    if ((pNetIf == NULL) || (dataCount < sizeof (IPV6_FRAGMENT_HEADER)))
        return IPV6_ACTION_DISCARD_SILENT;

    netIx = _TCPIPStackNetIx (pNetIf);

    TCPIP_IP_GetArray(pNetIf->hIfMac, (void *)&fragmentHeader, 8);
    dataCount -= sizeof (IPV6_FRAGMENT_HEADER);

    fragmentHeader.offsetM.w = swaps (fragmentHeader.offsetM.w);

    // Set fragment buffer pointer to the head of the linked list of fragmented packets    
    ptrFragment = (IPV6_FRAGMENT_BUFFER *)ipv6Config[netIx].fragments.head;

    // Find a packet being reassembled that matches this fragmented packet
    while (ptrFragment != NULL)
    {
        if (ptrFragment->identification == fragmentHeader.identification)
        {
            if ((!memcmp (ptrFragment->packet + IPV6_HEADER_OFFSET_SOURCE_ADDR, remoteIP, sizeof (IPV6_ADDR))) &&
                (!memcmp (ptrFragment->packet + IPV6_HEADER_OFFSET_DEST_ADDR, localIP, sizeof (IPV6_ADDR))))
            {
                break;
            }
        }
        ptrFragment = ptrFragment->next;
    }

    // If no existing fragment was found, this is the first fragment in the packet.
    // Create a fragment buffer for it and store the unfragmentable part.
    if (ptrFragment == NULL)
    {
        ptrFragment = (IPV6_FRAGMENT_BUFFER *)TCPIP_HEAP_Malloc (ipMemH, sizeof (IPV6_FRAGMENT_BUFFER));

        if (ptrFragment == NULL)
            return IPV6_ACTION_DISCARD_SILENT;

        ptrFragment->next = NULL;
        // The RFC specifies that the fragments must be reassembled in one minute or less
        ptrFragment->secondsRemaining = 60;;
        ptrFragment->identification = fragmentHeader.identification;
        ptrFragment->packetSize = headerLen;
        ptrFragment->bytesInPacket = headerLen;
        ptrFragment->firstFragmentLength = 0;

        // Reset the packet's read pointer
        MACSetReadPtrInRx(pNetIf->hIfMac, 0);

        // Copy the unfragmentable part of the packet data into the fragment buffer
        MACGetArray(pNetIf->hIfMac, ptrFragment->packet, headerLen);

        // Set the packet's read pointer to skip the fragment header
        MACSetReadPtrInRx(pNetIf->hIfMac, headerLen + sizeof (IPV6_FRAGMENT_HEADER));

        if (headerLen == sizeof (IPV6_HEADER))
            ptrFragment->packet[IPV6_HEADER_OFFSET_NEXT_HEADER] = fragmentHeader.nextHeader;
        else
            ptrFragment->packet[headerLen - previousHeaderLen] = fragmentHeader.nextHeader;

        SingleListAddTail(&ipv6Config[netIx].fragments, (SGL_LIST_NODE *)ptrFragment);

    }

    if (dataCount)
    {
        if (fragmentHeader.offsetM.bits.fragmentOffset == 0)
        {
            ptrFragment->firstFragmentLength = dataCount + headerLen;
        }

        if (fragmentHeader.offsetM.bits.m)
        {
            // More fragments
            // Check to ensure the packet's payload length is a multiple of eight bytes.
            if (((headerLen + dataCount) % 8) != 0)
            {
                TCPIP_IPV6_SendError (pNetIf, localIP, remoteIP, ICMPV6_ERR_PP_ERRONEOUS_HEADER, ICMPV6_ERROR_PARAMETER_PROBLEM, swapl(IPV6_HEADER_OFFSET_PAYLOAD_LENGTH), dataCount + headerLen + sizeof (IPV6_FRAGMENT_HEADER));
                return IPV6_ACTION_DISCARD_SILENT;
            }
            MACGetArray(pNetIf->hIfMac, ptrFragment->packet + headerLen + (fragmentHeader.offsetM.bits.fragmentOffset << 3), dataCount);
            ptrFragment->bytesInPacket += dataCount;
        }
        else
        {
            // No more fragments
            MACGetArray(pNetIf->hIfMac, ptrFragment->packet + headerLen + (fragmentHeader.offsetM.bits.fragmentOffset << 3), dataCount);
            ptrFragment->bytesInPacket += dataCount;
            ptrFragment->packetSize += (fragmentHeader.offsetM.bits.fragmentOffset << 3) + dataCount;
        }
    }

    // Todo : Just for safety we may want to insert a check to prevent this packet from being sent
    // Todo : if packetSize == headerLen.  This would occur if a fragment packet with no fragmentable part
    // Todo : was received.  That should never happen, though.
    if (ptrFragment->packetSize == ptrFragment->bytesInPacket)
    {
        PTR_BASE tempReadPtr;
        PTR_BASE tempBaseReadPtr;

        // Subtract the length of the IPV6 header from the payload
        ptrFragment->packetSize -= sizeof (IPV6_HEADER);

        ptrFragment->packet[IPV6_HEADER_OFFSET_PAYLOAD_LENGTH] = ((TCPIP_UINT16_VAL)ptrFragment->packetSize).v[1];
        ptrFragment->packet[IPV6_HEADER_OFFSET_PAYLOAD_LENGTH + 1] = ((TCPIP_UINT16_VAL)ptrFragment->packetSize).v[0];

        tempReadPtr = MACSetReadPtr (pNetIf->hIfMac, (PTR_BASE)ptrFragment->packet);
        tempBaseReadPtr = MACSetBaseReadPtr (pNetIf->hIfMac, (PTR_BASE)ptrFragment->packet - sizeof (ETHER_HEADER));

        TCPIP_IPV6_Process (pNetIf, remoteMACAddr);

        MACSetReadPtr (pNetIf->hIfMac, tempReadPtr);
        MACSetBaseReadPtr (pNetIf->hIfMac, tempBaseReadPtr);

        SingleListRemoveNode(&ipv6Config[netIx].fragments, (SGL_LIST_NODE *)ptrFragment);

        TCPIP_IPV6_FreeFragmentBuffer (ptrFragment);
    }

    return IPV6_ACTION_DISCARD_SILENT;
}

/*****************************************************************************
  Function:
	void TCPIP_IP_Task (void)

  Summary:
	IPv6 task function

  Description:
	IPv6 task function

  Precondition:
	None

  Parameters:
	None

  Returns:
  	None
  	
  Remarks:
	None
  ***************************************************************************/
void TCPIP_IP_Task (void)
{
    ipTickPending--;
    TCPIP_IPV6_FragmentTask();
    TCPIP_IPV6_UpdateTimestampsTask();
}

/*****************************************************************************
  Function:
	void TCPIP_IPV6_FragmentTask (void)

  Summary:
	IPv6 fragment processing task function.

  Description:
	IPv6 fragment processing task function.

  Precondition:
	None

  Parameters:
	None

  Returns:
  	None
  	
  Remarks:
	None
  ***************************************************************************/
void TCPIP_IPV6_FragmentTask (void)
{
    IPV6_FRAGMENT_BUFFER * ptrFragment;
    IPV6_FRAGMENT_BUFFER * ptrNextFragment;
    NET_CONFIG * pNetIf;
    int netIx;

    for (netIx = 0; netIx < nStackIfs; netIx++)
    {
        pNetIf = (NET_CONFIG*)TCPIP_STACK_IxToNet(netIx);
        if(_TCPIPStackIsNetUp(pNetIf))
        {
            ptrFragment = (IPV6_FRAGMENT_BUFFER *)ipv6Config[netIx].fragments.head;
            while (ptrFragment != NULL)
            {
                ptrFragment->secondsRemaining--;
                if ((char)(ptrFragment->secondsRemaining) == 0)
                {
                    if (ptrFragment->firstFragmentLength != 0)
                    {
                        PTR_BASE tempReadPtr;
                        PTR_BASE tempBaseReadPtr;

                        tempReadPtr = MACSetReadPtr (pNetIf->hIfMac, (PTR_BASE)ptrFragment->packet);
                        tempBaseReadPtr = MACSetBaseReadPtr (pNetIf->hIfMac, (PTR_BASE)ptrFragment->packet - sizeof (ETHER_HEADER));

                        // If we received the first fragment, send it and a Time Exceeded error message
                        TCPIP_IPV6_SendError (pNetIf, (IPV6_ADDR *)&ptrFragment->packet[IPV6_HEADER_OFFSET_DEST_ADDR], (IPV6_ADDR *)&ptrFragment->packet[IPV6_HEADER_OFFSET_SOURCE_ADDR], ICMPV6_ERR_TE_FRAG_ASSEMBLY_TIME_EXCEEDED, ICMPV6_ERROR_TIME_EXCEEDED, 0, ptrFragment->firstFragmentLength);

                        MACSetReadPtr (pNetIf->hIfMac, tempReadPtr);
                        MACSetBaseReadPtr (pNetIf->hIfMac, tempBaseReadPtr);
                    }
                    SingleListRemoveNode(&ipv6Config[netIx].fragments, (SGL_LIST_NODE *)ptrFragment);
                    ptrNextFragment = ptrFragment->next;
                    TCPIP_IPV6_FreeFragmentBuffer (ptrFragment);
                    ptrFragment = ptrNextFragment;
                }
                else
                    ptrFragment = ptrFragment->next;
            }
        }
    }
}

/*****************************************************************************
  Function:
	void TCPIP_IP_TmoHandler(SYS_TICK curSysTick)

  Summary:
	Timeout handler for the IP task system timer.

  Description:
	Timeout handler for the IP task system timer.	

  Precondition:
	None

  Parameters:
	curSysTick - The current system tick

  Returns:
  	None
  	
  Remarks:
	None
  ***************************************************************************/
void TCPIP_IP_TmoHandler(SYS_TICK curSysTick)
{
    ipTickPending++;
}

/*****************************************************************************
  Function:
	bool TCPIP_IP_TaskPending (void)

  Summary:
	Determines if an IP task is pending

  Description:
	Determines if an IP task is pending

  Precondition:
	None

  Parameters:
	None

  Returns:
    bool - 1 if a task is pending, 0 otherwise
  	
  Remarks:
	None
  ***************************************************************************/
bool TCPIP_IP_TaskPending (void)
{
    return (ipTickPending == 0)?0:1;
}

#endif // defined (TCPIP_STACK_USE_IPV6)

#if defined (TCPIP_STACK_USE_IPV6)
/*****************************************************************************
  Function:
	uint8_t TCPIP_IPV6_ProcessRoutingHeader (NET_CONFIG * pNetIf, 
        uint8_t * nextHeader, uint16_t * length)

  Summary:
	Processes an IPv6 Routing Extension header.

  Description:
	Processes an IPv6 Routing Extension header.	

  Precondition:
	None

  Parameters:
	pNetIf - The interface the packet with this header was received on.
    nextHeader - Return value for the next header.
    length - Return value for the header length.

  Returns:
  	uint8_t - An action code for the TCPIP_IPV6_Process function.
  	
  Remarks:
	None
  ***************************************************************************/
uint8_t TCPIP_IPV6_ProcessRoutingHeader (NET_CONFIG * pNetIf, uint8_t * nextHeader, uint16_t * length)
{
    uint8_t headerLength;
    uint8_t routingType;
    uint8_t segmentsLeft;
    uint8_t data[8];
    uint8_t i;
    TCPIP_MAC_HANDLE hMac = _TCPIPStackNetToMac (pNetIf);

    TCPIP_IPV6_GetOptionHeader (hMac, data, 1);

    *nextHeader = data[0];
    headerLength = data[1];
    *length = ((uint16_t)headerLength << 3) + 8;

    routingType = data[2];
    segmentsLeft = data[3];

    i = 4;

    switch (routingType)
    {
        // Type 0 routing headers were deprecated and we aren't a router,
        // so we don't support any routing options
        default:
            if (segmentsLeft == 0)
                break;
            else
            {
                // Set the 'length' parameter to the offset of the routingType field.
                // This will allow the TCPIP_IPV6_Process function to find the correct offset
                // for the ICMPv6 Parameter Problem error message.
                *length = 2;
                return IPV6_ACTION_DISCARD_PP_0;
            }
    }

    // If we get here, ignore the rest of the header
    // Since the header size is a multiple of 8 bytes, 
    // just discard the rest of the bytes in the first
    // 8-byte unit and read the full header size 
    while (headerLength--)
        TCPIP_IPV6_GetOptionHeader (hMac, data, 1);

    return IPV6_ACTION_NONE;
}
#endif

#if defined (TCPIP_STACK_USE_IPV6)
/*****************************************************************************
  Function:
	IPV6_ADDR_STRUCT * TCPIP_IPV6_DAS_SelectSourceAddress (NET_CONFIG * pNetIf, 
        IPV6_ADDR * dest, IPV6_ADDR * requestedSource)

  Summary:
	Determines the appropriate source address for a given destination 
    address.

  Description:
	Determines the appropriate source address for a given destination 
    address.

  Precondition:
	None

  Parameters:
	pNetIf - The given interface.
    dest - The destination address.
    requestedSource - A specified source.

  Returns:
  	IPV6_ADDR_STRUCT * - The selected source address.
  	
  Remarks:
	None
  ***************************************************************************/
IPV6_ADDR_STRUCT * TCPIP_IPV6_DAS_SelectSourceAddress (NET_CONFIG * pNetIf, IPV6_ADDR * dest, IPV6_ADDR * requestedSource)
{
    IPV6_ADDR_STRUCT * currentSource;
    IPV6_ADDR_STRUCT * previousSource;
    uint8_t ruleCounter = ADDR_SEL_RULE_8;
    int netIx;

    if (pNetIf == NULL)
    {
        return NULL;
    }
    else
    {
        netIx = _TCPIPStackNetIx (pNetIf);
    }

    // Check to see if the user is trying to force an address
    if (requestedSource != NULL)
    {
        currentSource = TCPIP_IPV6_FindAddress(pNetIf, requestedSource, IPV6_ADDR_TYPE_UNICAST);
        if (currentSource != NULL)
        {
            return currentSource;
        }
        else
        {
            return NULL;
        }
    } // End manual address selection

    // Simple case: there are no local addresses (this should never happen)
    if (DoubleListIsEmpty(&ipv6Config[netIx].listIpv6UnicastAddresses))
        return NULL;

    // Simple case: there's only one source address in the list
    if (DoubleListCount (&ipv6Config[netIx].listIpv6UnicastAddresses) == 1)
        return (IPV6_ADDR_STRUCT *)ipv6Config[netIx].listIpv6UnicastAddresses.head;

    // Complex case: Sort the addresses we found using the Address Selection rules

    // Sort the linked list.
    // There are 8 sorting rules.  Starting with the last rule and working to the most
    // important, using a stable sorting algorithm, will produce a sorted list most
    // efficiently.  The best average run time we'll get with a stable sort with O(1) 
    // memory usage is O(n^2), so we'll use an insertion sort.  This will usually be 
    // most efficient for small lists (which should be the typical case).

    do
    {
        // We know that the list has at least two elements, so these pointers will both have non-null values
        previousSource = (IPV6_ADDR_STRUCT *)ipv6Config[netIx].listIpv6UnicastAddresses.head;
        currentSource = previousSource->next;
        
        do
        {
            // Set previousSource to the node before the current node being evaluated
            previousSource = currentSource->prev;
    
            // Advance backwards through the list until we don't prefer currentSource over previousSource
            // (or until there are no previous source addresses)
            // The IPv6ASCompareSourceAddresses function will return true if we prefer currentSource over previousSource
            while ((previousSource != NULL) && (IPv6ASCompareSourceAddresses (pNetIf, previousSource, currentSource, dest, ruleCounter)))
            {
                previousSource = previousSource->prev;
            }
            
            // Move the currentSource node into the list after previousSource (or to the head if previousSource == NULL)
            currentSource = TCPIP_NDP_UnicastAddressMove (pNetIf, currentSource, previousSource);
        } while (currentSource != NULL);
    
        ruleCounter--;
        // Skip rules 4 and 5 for now; we don't support Mobile IPv6 or multiple interfaces at this time
        if (ruleCounter == ADDR_SEL_RULE_5)
            ruleCounter = ADDR_SEL_RULE_3;
    } while (ruleCounter >= ADDR_SEL_RULE_1);
    
    return (IPV6_ADDR_STRUCT *)ipv6Config[netIx].listIpv6UnicastAddresses.head;
}
#endif

#if defined (TCPIP_STACK_USE_IPV6)
/*****************************************************************************
  Function:
	uint8_t TCPIP_IPV6_DAS_GetPolicy (const IPV6_ADDR * addr, uint8_t * label, 
        uint8_t * precedence, uint8_t * prefixLen)

  Summary:
	Gets the default address selection policy for an address.

  Description:
	Gets the default address selection policy for an address.	

  Precondition:
	None

  Parameters:
	addr - The given address
    label - Return value for the address label
    precedence - Return value for the precedence
    prefixLen - Return value for the prefix length

  Returns:
  	None
  	
  Remarks:
	None
  ***************************************************************************/
uint8_t TCPIP_IPV6_DAS_GetPolicy (const IPV6_ADDR * addr, uint8_t * label, uint8_t * precedence, uint8_t * prefixLen)
{
    uint8_t i;
    uint8_t prefixMatch = 0;
    uint8_t matchingPrefix = 0xFF;

    for (i = 0; i < IPV6_ADDR_POLICY_TABLE_LEN; i++)
    {
        if (gPolicyTable[i].prefixLength != 0xFF)
        {
            // If we get to the 0-length prefix and we haven't found a
            // matching prefix, then assume a match
            if (gPolicyTable[i].prefixLength == 0)
            {
                if (prefixMatch == 0)
                    matchingPrefix = i;
            }
            else if (gPolicyTable[i].prefixLength > prefixMatch)
            {
                if (FindCommonPrefix ((uint8_t *)addr, (uint8_t *)&gPolicyTable[i].address, 16) >= gPolicyTable[i].prefixLength)
                {
                    matchingPrefix = i;
                    prefixMatch = gPolicyTable[i].prefixLength;
                }
            }
        }
    }

    if (matchingPrefix == 0xFF)
        return false;
    else
    {
        if (label != NULL)
            *label = gPolicyTable[matchingPrefix].label;
        if (precedence != NULL)
            *precedence = gPolicyTable[matchingPrefix].precedence;
        if (prefixLen != NULL)
            *prefixLen = gPolicyTable[matchingPrefix].prefixLength;
    }
    return true;
}
#endif

#if defined (TCPIP_STACK_USE_IPV6)
/*****************************************************************************
  Function:
	unsigned char IPv6ASCompareSourceAddresses(NET_CONFIG * pNetIf, 
        IPV6_ADDR_STRUCT * addressOne, IPV6_ADDR_STRUCT * addressTwo, 
        IPV6_ADDR * dest, IPV6_ADDR_SEL_INDEX rule)

  Summary:
	Compares two IPv6 addresses using specified rules to determine which 
    ones are prefereable.

  Description:
	Compares two IPv6 addresses using specified rules to determine which 
    ones are prefereable.

  Precondition:
	None

  Parameters:
	pNetIf - The given interface
    addressOne - One address to compare
    addressTwo - The other address to compare
    dest - A destination address (used for some comparisons)
    rule - The address comparison rule to use

  Returns:
  	bool - true if addressTwo is preferred over addressOne, false otherwise
  	
  Remarks:
	None
  ***************************************************************************/
unsigned char IPv6ASCompareSourceAddresses(NET_CONFIG * pNetIf, IPV6_ADDR_STRUCT * addressOne, IPV6_ADDR_STRUCT * addressTwo, IPV6_ADDR * dest, IPV6_ADDR_SEL_INDEX rule)
{
    unsigned char policy1 = 0;
    unsigned char policy2 = 0;
    unsigned char destPolicy;
    IPV6_ADDRESS_TYPE destScope;
    int netIx;

    if (pNetIf == NULL)
        return false;

    netIx = _TCPIPStackNetIx (pNetIf);

    switch (rule)
    {
        case ADDR_SEL_RULE_1:
            // We can assume the the addresses are different; the function to add a local
            // address won't add a new one if it's already in the IPv6 Heap.
            if (memcmp ((void *)&(addressTwo->address), (void *)dest, 16) == 0)
            {
                return true;
            }
            break;
        case ADDR_SEL_RULE_2:
            if (addressOne->flags.scope != addressTwo->flags.scope)
            {
                destScope = TCPIP_IPV6_GetAddressType (pNetIf, dest);
                destPolicy = destScope.bits.scope;

                if (addressOne->flags.scope < addressTwo->flags.scope)
                {
                    if (addressOne->flags.scope < destPolicy)
                    {
                        return true;
                    }
                }
                else
                {
                    if (addressTwo->flags.scope >= destPolicy)
                    {
                        return true;
                    }
                }
            }
            break;
        case ADDR_SEL_RULE_3:
            if (addressTwo->preferredLifetime && !(addressOne->preferredLifetime))
            {
                return true;
            }
            break;
        case ADDR_SEL_RULE_4:
            // We aren't supporting Mobile IPv6 at this time                
            break;
        case ADDR_SEL_RULE_5:
            // We aren't supporting multiple interfaces at this time
            break;
        case ADDR_SEL_RULE_6:
            if (!TCPIP_IPV6_DAS_GetPolicy (dest, &destPolicy, NULL, NULL))
            {
                // If there's no policy that corresponds to the destination, skip this step
                break;
            }
            if (!TCPIP_IPV6_DAS_GetPolicy (&(addressOne->address), &policy1, NULL, NULL))
            {
                if (TCPIP_IPV6_DAS_GetPolicy (&(addressTwo->address), &policy2, NULL, NULL))
                {
                    if (destPolicy == policy2)
                        return true;
                }
            }
            else
            {
                if (!TCPIP_IPV6_DAS_GetPolicy (&(addressTwo->address), &policy2, NULL, NULL))
                {
                    if (destPolicy == policy1)
                        return false;
                }
            }
            if (policy1 != policy2)
            {
                if (destPolicy == policy2)
                    return true;
            }
            break;
        case ADDR_SEL_RULE_7:
            if (addressOne->flags.temporary != addressTwo->flags.temporary)
            {
                if (((addressTwo->flags.temporary == false) && (ipv6Config[netIx].policyPreferTempOrPublic == IPV6_PREFER_PUBLIC_ADDRESSES)) ||
                    ((addressTwo->flags.temporary == true) && (ipv6Config[netIx].policyPreferTempOrPublic == IPV6_PREFER_TEMPORARY_ADDRESSES)))
                {
                    return true;
                }
            }
            break;
        case ADDR_SEL_RULE_8:
            policy1 = FindCommonPrefix ((uint8_t *)dest, (uint8_t *)&(addressOne->address), 16);
            policy2 = FindCommonPrefix ((uint8_t *)dest, (uint8_t *)&(addressTwo->address), 16);
            if (policy2 > policy1)
            {
                return true;
            }
            break;

        default:
            break;
    }
    // If there's no reason to prefer addressTwo, return false
    return false;
}
#endif

#if defined (TCPIP_STACK_USE_IPV6)
/*****************************************************************************
  Function:
	void TCPIP_IPV6_UpdateTimestampsTask (void)

  Summary:
	Task to update timestamps and check for validity for NDP structures.

  Description:
	Task to update timestamps and check for validity for NDP structures.

  Precondition:
	None

  Parameters:
	None

  Returns:
  	None
  	
  Remarks:
	None
  ***************************************************************************/
void TCPIP_IPV6_UpdateTimestampsTask (void)
{
    unsigned long timeElapsed;
    unsigned long currentTickTime = SYS_TICK_Get();
    unsigned long correctedCurrentTime;
    int i;
    IPV6_HEAP_NDP_PL_ENTRY * ptrPrefix;
    IPV6_HEAP_NDP_PL_ENTRY * tempPrefix;
    IPV6_ADDR_STRUCT * ptrAddress;
    IPV6_ADDR_STRUCT * tempAddress;
    IPV6_HEAP_NDP_DR_ENTRY * ptrRouter;
    IPV6_HEAP_NDP_DC_ENTRY * ptrDestination;
    NET_CONFIG * pNetIf;

    for (i = 0; i < nStackIfs; i++)
    {
        pNetIf = (NET_CONFIG*)TCPIP_STACK_IxToNet(i);
        if(_TCPIPStackIsNetUp(pNetIf))
        {
            // Update prefixes
            ptrPrefix = (IPV6_HEAP_NDP_PL_ENTRY *)ipv6Config[i].listPrefixList.head;

            while (ptrPrefix != NULL)
            {
                timeElapsed = currentTickTime - ptrPrefix->lastTickTime;
                timeElapsed /= SYS_TICK_ResolutionGet();
                correctedCurrentTime = currentTickTime - (timeElapsed % SYS_TICK_ResolutionGet());

                ptrPrefix->lastTickTime = correctedCurrentTime;
                if (timeElapsed < ptrPrefix->validLifetime)
                {
                    ptrPrefix->validLifetime -= timeElapsed;
                    tempPrefix = ptrPrefix->next;
                }
                else
                {
                    tempPrefix = ptrPrefix->next;
                    TCPIP_NDP_LinkedListEntryRemove ((NET_CONFIG*)TCPIP_STACK_IxToNet(i), ptrPrefix, IPV6_HEAP_NDP_PL_ID);
                }
                ptrPrefix = tempPrefix;
            }

            // Update addresses
            ptrAddress = (IPV6_ADDR_STRUCT *)ipv6Config[i].listIpv6UnicastAddresses.head;

            while (ptrAddress != NULL)
            {
                timeElapsed = currentTickTime - ptrAddress->lastTickTime;
                timeElapsed /= SYS_TICK_ResolutionGet();
                correctedCurrentTime = currentTickTime - (timeElapsed % SYS_TICK_ResolutionGet());

                ptrAddress->lastTickTime = correctedCurrentTime;
                if (timeElapsed < ptrAddress->preferredLifetime)
                {
                    ptrAddress->preferredLifetime -= timeElapsed;
                }
                else
                {
                    ptrAddress->preferredLifetime = 0;
                }
                if (timeElapsed < ptrAddress->validLifetime)
                {
                    ptrAddress->validLifetime -= timeElapsed;    
                    tempAddress = ptrAddress->next;
                }
                else
                {
                    tempAddress = ptrAddress->next;
                    IPv6RemoveUnicastAddress ((NET_CONFIG*)TCPIP_STACK_IxToNet(i), &ptrAddress->address);
                }
                ptrAddress = tempAddress;

            }

            // Update default routers
            ptrRouter = (IPV6_HEAP_NDP_DR_ENTRY *)ipv6Config[i].listDefaultRouter.head;

            while (ptrRouter != NULL)
            {
                if ((long)(currentTickTime - ptrRouter->tickTimer) >= SYS_TICK_ResolutionGet())
                {
                    ptrRouter->tickTimer += SYS_TICK_ResolutionGet();
                    ptrRouter->invalidationTimer--;
                    if (!ptrRouter->invalidationTimer)
                    {
                        ptrRouter = TCPIP_NDP_LinkedListEntryRemove ((NET_CONFIG*)TCPIP_STACK_IxToNet(i), ptrRouter, IPV6_HEAP_NDP_DR_ID);
                    }
                }
                else
                {
                    ptrRouter = ptrRouter->next;
                }
            }

            // Try to periodically increase path MTUs
            ptrDestination = (IPV6_HEAP_NDP_DC_ENTRY *)ipv6Config[i].listDestinationCache.head;

            while (ptrDestination != NULL)
            {
                if (ptrDestination->pathMTUIncreaseTimer != 0)
                {
                    if ((long)(currentTickTime - ptrDestination->pathMTUIncreaseTimer) > 0)
                    {
                        ptrDestination->pathMTU = IPV6_DEFAULT_LINK_MTU;
                        ptrDestination->pathMTUIncreaseTimer = 0;
                    }
                }
                ptrDestination = ptrDestination->next;
            }

            if (ipv6Config[i].mtuIncreaseTimer != 0)
            {
                if ((long)(currentTickTime - ipv6Config[i].mtuIncreaseTimer) > 0)
                {
                    ipv6Config[i].linkMTU = IPV6_DEFAULT_LINK_MTU;
                    ipv6Config[i].multicastMTU = IPV6_DEFAULT_LINK_MTU;
                    ipv6Config[i].mtuIncreaseTimer = 0;
                }
            }
        }
    }
}

#endif

#if defined (TCPIP_STACK_USE_IPV6)
/*****************************************************************************
  Function:
	void TCPIP_IPV6_Process (NET_CONFIG * pNetIf, MAC_ADDR * remoteMAC)

  Summary:
	Processes incoming IPv6 packets/

  Description:
	This function processes incoming IPv6 packets.

  Precondition:
	None

  Parameters:
	pNetIf - The interface the packet was received on
    remoteMAC - The remote node's MAC address

  Returns:
  	None
  	
  Remarks:
	None
  ***************************************************************************/
void TCPIP_IPV6_Process (NET_CONFIG * pNetIf, MAC_ADDR * remoteMAC)
{
    IPV6_ADDR tempLocalIPv6Addr;
    IPV6_ADDR tempRemoteIPv6Addr;
    IPV6_ADDR_STRUCT * localAddressPointer;
    uint16_t headerLen = 0;
    uint16_t extensionHeaderLen;
    uint8_t action;
    uint8_t hopLimit;
    bool addrType = 0;
    uint16_t currentOffset = 0;
    uint8_t cIPFrameType;
    uint16_t dataCount;

    // Break out of this processing function is IPv6 is not enabled on this node
    if (!pNetIf->Flags.bIPv6Enabled)
        return;

    // Get the relevant IPv6 header parameters
    if (!TCPIP_IPV6_GetHeader(pNetIf, &tempLocalIPv6Addr, &tempRemoteIPv6Addr, &cIPFrameType, &dataCount, &hopLimit))
        return;

    currentOffset += sizeof (IPV6_HEADER);

    // Determine if the address corresponds to one of the addresses used by our node
    if (tempLocalIPv6Addr.v[0] == 0xFF)
    {
        // Determine if the address is a solicited node multicast address
        if (TCPIP_IPV6_AddressIsSolicitedNodeMulticast (&tempLocalIPv6Addr))
        {
            // Determine if we are listening to this address
            if ((localAddressPointer = TCPIP_IPV6_FindSolicitedNodeMulticastAddress(pNetIf, &tempLocalIPv6Addr, IPV6_ADDR_TYPE_UNICAST)) != NULL)
            {
                addrType = IPV6_ADDR_TYPE_SOLICITED_NODE_MULTICAST;
            }
            else if ((localAddressPointer = TCPIP_IPV6_FindSolicitedNodeMulticastAddress(pNetIf, &tempLocalIPv6Addr, IPV6_ADDR_TYPE_UNICAST_TENTATIVE)) != NULL)
            {
                addrType = IPV6_ADDR_TYPE_UNICAST_TENTATIVE;
            }
        }
        else
        {
            // Find the address in the list of multicast addresses we're listening to
            localAddressPointer = TCPIP_IPV6_FindAddress(pNetIf, &tempLocalIPv6Addr, IPV6_ADDR_TYPE_MULTICAST);
            addrType = IPV6_ADDR_TYPE_MULTICAST;
        }
    }
    else
    {
        // Find the address in the list of unicast addresses assigned to our node
        localAddressPointer = TCPIP_IPV6_FindAddress (pNetIf, &tempLocalIPv6Addr, IPV6_ADDR_TYPE_UNICAST);
        addrType = IPV6_ADDR_TYPE_UNICAST;
    }

    // If the packet's destination address isn't one of the unicast/multicast addresses assigned to this node, check to see if it is a 
    // tentative address (ICMPv6/NDP still needs to receive packets addressed to tentative addresses for duplicate address detection).
    // If it is not tentative, return.
    if (localAddressPointer == NULL)
    {
        // If we didn't find a matching configured address try to find one in the tentative address list
        if ((localAddressPointer = TCPIP_IPV6_FindAddress (pNetIf, &tempLocalIPv6Addr, IPV6_ADDR_TYPE_UNICAST_TENTATIVE)) != NULL)
        {
            addrType = IPV6_ADDR_TYPE_UNICAST_TENTATIVE;
        }
        else
        {
            return;
        }
    }

    extensionHeaderLen = 0;
    action = IPV6_ACTION_BEGIN_EX_HEADER_PROCESSING;

    // Process frame
    while (cIPFrameType != IP_PROT_NONE)
    {
        switch (cIPFrameType)
        {
            // Process the frame's hop-by-hop options header
            case IP_PROT_HOP_BY_HOP_OPTIONS_HEADER:
                // Action should only equal 0xFF immediately after processing the IPv6 header.
                // The hop-by-hop options header must occur only after the IPv6 header.
                if (action != IPV6_ACTION_BEGIN_EX_HEADER_PROCESSING)
                {
                    goto ParameterProblem;
                }
                action = TCPIP_IPV6_ProcessHopByHopOptionsHeader (pNetIf, &cIPFrameType, &headerLen);
                dataCount -= headerLen;
                extensionHeaderLen += headerLen;
                break;
            // Process the frame's routing header
            case IP_PROT_ROUTING_HEADER:
                action = TCPIP_IPV6_ProcessRoutingHeader (pNetIf, &cIPFrameType, &headerLen);
                dataCount -= headerLen;
                extensionHeaderLen += headerLen;
                break;
            // Process the frame's fragmentation header
            case IP_PROT_FRAGMENTATION_HEADER:
                action = TCPIP_IPV6_ProcessFragmentationHeader (pNetIf, &tempRemoteIPv6Addr, &tempLocalIPv6Addr, &cIPFrameType, dataCount, extensionHeaderLen + sizeof (IPV6_HEADER), remoteMAC, headerLen);
                //action = IPV6_ACTION_DISCARD_SILENT;
                break;
            // Process the frame's ESP header
            case IP_PROT_ENCAPSULATING_SECURITY_PAYLOAD_HEADER:
                action = IPV6_ACTION_DISCARD_SILENT;
                break;
            // Process the frame's Authentication header
            case IP_PROT_AUTHENTICATION_HEADER:
                action = IPV6_ACTION_DISCARD_SILENT;
                break;
            // Process the frame's destination options header
            case IP_PROT_DESTINATION_OPTIONS_HEADER:
                action = TCPIP_IPV6_ProcessDestinationOptionsHeader(pNetIf, &cIPFrameType, &headerLen);
                dataCount -= headerLen;
                extensionHeaderLen += headerLen;
                break;
            // Process the frame's TCP header and payload
            case IP_PROT_TCP:
#if defined (TCPIP_STACK_USE_TCP)
                // If the address is tentative, do not process the TCP packet
                if (addrType == IPV6_ADDR_TYPE_UNICAST_TENTATIVE)
                {
                    cIPFrameType = IP_PROT_NONE;
                    break;
                }
                TCPProcessIPv6(pNetIf, &tempLocalIPv6Addr, &tempRemoteIPv6Addr, dataCount, extensionHeaderLen);
#endif
                cIPFrameType = IP_PROT_NONE;
                action = 0;
                break;
            // Process the frame's UDP header and payload
            case IP_PROT_UDP:
#if defined (TCPIP_STACK_USE_UDP)
                // If the address is tentative, do not process the TCP packet
                if (addrType == IPV6_ADDR_TYPE_UNICAST_TENTATIVE)
                {
                    cIPFrameType = IP_PROT_NONE;
                    break;
                }
                // Process the UDP packet
                UDPProcessIPv6(pNetIf, &tempLocalIPv6Addr, &tempRemoteIPv6Addr, dataCount, extensionHeaderLen);
#endif
                action = 0;
                return;
            // Process the frame's ICMPv6 header and payload
            case IP_PROT_ICMPV6:
                // Process the ICMPv6 packet
                TCPIP_ICMPV6_Process(pNetIf, localAddressPointer, &tempLocalIPv6Addr, &tempRemoteIPv6Addr, dataCount, extensionHeaderLen, hopLimit, addrType);
                cIPFrameType = IP_PROT_NONE;
                action = 0;
                break;
            // Unknown header type
            default:
ParameterProblem:
                // Send ICMP Parameter Problem Code 1 and discard packet
                {
                    // Action should only equal IPV6_ACTION_BEGIN_EX_HEADER_PROCESSING if we haven't been through this loop once
                    if (action == IPV6_ACTION_BEGIN_EX_HEADER_PROCESSING)
                        currentOffset = IPV6_HEADER_OFFSET_NEXT_HEADER;
                    else
                        currentOffset -= headerLen;

                    TCPIP_IPV6_SendError (pNetIf, &tempLocalIPv6Addr, &tempRemoteIPv6Addr, ICMPV6_ERR_PP_UNRECOGNIZED_NEXT_HEADER, ICMPV6_ERROR_PARAMETER_PROBLEM, swapl((uint32_t)currentOffset), dataCount + extensionHeaderLen + sizeof (IPV6_HEADER));
                    cIPFrameType = IP_PROT_NONE;
                }
                break;
        }

        // Add the length of the header to the current offset
        // If there was a parameter problem, this value will indicate the offset 
        // in the header at which the parameter problem occured.
        currentOffset += headerLen;

        // Take an action depending on the result of our header processing
        switch (action)
        {
            // Silently discard the packet
            case IPV6_ACTION_DISCARD_SILENT:
                cIPFrameType = IP_PROT_NONE;
                break;
            // Discard the packet and send an ICMPv6 Parameter Problem, code 0 error message to the packet's source address
            case IPV6_ACTION_DISCARD_PP_0:
                TCPIP_IPV6_SendError (pNetIf, &tempLocalIPv6Addr, &tempRemoteIPv6Addr, ICMPV6_ERR_PP_ERRONEOUS_HEADER, ICMPV6_ERROR_PARAMETER_PROBLEM, swapl((uint32_t)currentOffset), dataCount + extensionHeaderLen + sizeof (IPV6_HEADER));
                cIPFrameType = IP_PROT_NONE;
                break;
            // Discard the packet and send an ICMPv6 Parameter Problem, code 2 error message to the packet's source address
            case IPV6_ACTION_DISCARD_PP_2:
                TCPIP_IPV6_SendError (pNetIf, &tempLocalIPv6Addr, &tempRemoteIPv6Addr, ICMPV6_ERR_PP_UNRECOGNIZED_IPV6_OPTION, ICMPV6_ERROR_PARAMETER_PROBLEM, swapl((uint32_t)currentOffset), dataCount + extensionHeaderLen + sizeof (IPV6_HEADER));
                cIPFrameType = IP_PROT_NONE;
                break;
            // Discard the packet and send an ICMPv6 Parameter Problem, code 2 error message to the packet's source address if
            // the packet's destination address is not a multicast address.
            case IPV6_ACTION_DISCARD_PP_2_NOT_MC:
                // Check to ensure the packet's destination address wasn't a multicast address
                if (tempLocalIPv6Addr.v[0] != 0xFF)
                {
                    TCPIP_IPV6_SendError (pNetIf, &tempLocalIPv6Addr, &tempRemoteIPv6Addr, ICMPV6_ERR_PP_UNRECOGNIZED_IPV6_OPTION, ICMPV6_ERROR_PARAMETER_PROBLEM, swapl((uint32_t)currentOffset), dataCount + extensionHeaderLen + sizeof (IPV6_HEADER));
                }
                // Discard the packet
                cIPFrameType = IP_PROT_NONE;
                break;
            // No action was required
            case IPV6_ACTION_NONE:
            default:
                break;
        }
    }
}

/*****************************************************************************
  Function:
	void TCPIP_IPV6_SendError (NET_CONFIG * pNetIf, IPV6_ADDR * localIP, 
        IPV6_ADDR * remoteIP, uint8_t code, uint8_t type, 
        uint32_t additionalData, uint16_t packetLen)

  Summary:
	Sends an ICMPv6 error message to a remote node.

  Description:
	Sends an ICMPv6 error message to a remote node.	

  Precondition:
	None

  Parameters:
	pNetIf - The outgoing interface to send the message on.
    localIP - The local IP address to use.
    remoteIP - The destination IP address to use.
    code - The error code value.
    type - The error type.
    additionalData - Additional ICMPv6 header data.
    packetLen - Length of packet data to include from the packet RX buffer.

  Returns:
  	None
  	
  Remarks:
	None
  ***************************************************************************/
void TCPIP_IPV6_SendError (NET_CONFIG * pNetIf, IPV6_ADDR * localIP, IPV6_ADDR * remoteIP, uint8_t code, uint8_t type, uint32_t additionalData, uint16_t packetLen)
{
    IP_PACKET * pkt;

    if (packetLen + sizeof (IPV6_HEADER) + sizeof (ICMPV6_HEADER_ERROR) > IPV6_MINIMUM_LINK_MTU)
        packetLen = IPV6_MINIMUM_LINK_MTU - (sizeof (IPV6_HEADER) + sizeof (ICMPV6_HEADER_ERROR));

    // An ICMPv6 error message MUST NOT be sent as a result of receiving a packet destined to an IPv6 multicast address
    // Exception: Packet Too Big message, Parameter Problem code 2 message if the unrecognized option has its unrecognized action bits set to 0b10.
    // The unrecognized action bits were already examined in the option processing functions.
    if ((localIP->v[0] == 0xFF) && (type != ICMPV6_ERROR_PACKET_TOO_BIG) && ((type != ICMPV6_ERROR_PARAMETER_PROBLEM) || (code != ICMPV6_ERR_PP_UNRECOGNIZED_IPV6_OPTION)))
        return;

    // An ICMPv6 error message MUST NOT be sent as a result of receiving a packet whose source address does not uniquely identify a single node.
    if ((remoteIP->v[0] == 0xFF) || !memcmp (remoteIP, (IPV6_ADDR *)&IPV6_FIXED_ADDR_UNSPECIFIED, sizeof (IPV6_ADDR)))
        return;

    pkt = TCPIP_ICMPV6_PutHeaderError (pNetIf, localIP, remoteIP, code, type, additionalData);
    if (pkt != NULL)
    {
        MACSetReadPtrInRx (pNetIf->hIfMac, 0);
        if (TCPIP_IP_IsTxPutReady(pkt, packetLen) < packetLen)
        {
            TCPIP_IP_FreePacket (pkt);
            return;
        }
        TCPIP_IP_PutRxData (pNetIf->hIfMac, pkt, packetLen);
        TCPIP_ICMPV6_Flush (pkt);
    }
}

#endif  // defined (TCPIP_STACK_USE_IPV6)

/*****************************************************************************
  Function:
	void * TCPIP_IP_GetUpperLayerHeaderPtr(IP_PACKET * pkt)

  Summary:
	Returns a pointer to the upper layer header segment in a packet.

  Description:
	Returns a pointer to the upper layer header segment in a packet.	

  Precondition:
	None

  Parameters:
	pkt - The packet.

  Returns:
  	void * - Pointer to the upper layer header.
  	
  Remarks:
	None
  ***************************************************************************/
void * TCPIP_IP_GetUpperLayerHeaderPtr(IP_PACKET * pkt)
{
    return TCPIP_IP_GetDataSegmentContentsByType(pkt, TYPE_UPPER_LAYER_HEADER);
}

/*****************************************************************************
  Function:
	unsigned short TCPIP_IP_GetPayloadLength (IP_PACKET * pkt)

  Summary:
	Returns the current payload length of a packet.

  Description:
	Returns the current payload length of a packet.	

  Precondition:
	None

  Parameters:
	pkt - The packet.

  Returns:
  	unsigned short - The current payload length, in bytes.
  	
  Remarks:
	None
  ***************************************************************************/
unsigned short TCPIP_IP_GetPayloadLength (IP_PACKET * pkt)
{
    return pkt->payloadLen + pkt->upperLayerHeaderLen;
}


void  TCPIP_IPV4_SetDestAddress(IP_PACKET * p, uint32_t addrValue)
{
    p->ipHeader.ipv4Header.DestAddress.Val = addrValue;
}

IP_ADDR  TCPIP_IPV4_GetDestAddress(IP_PACKET * p)
{
    return p->ipHeader.ipv4Header.DestAddress;
}

void  TCPIP_IPV4_SetSourceAddress(IP_PACKET * p, uint32_t addrValue)
{
    p->ipHeader.ipv4Header.SourceAddress.Val = addrValue;
    p->flags.sourceSpecified = true;
}

IP_ADDR  TCPIP_IPV4_GetSourceAddress(IP_PACKET * p)
{
    return p->ipHeader.ipv4Header.SourceAddress;
}

#if defined (TCPIP_STACK_USE_IPV6)

IPV6_ADDR *  TCPIP_IPV6_GetDestAddress(IP_PACKET * p)
{
    return &p->ipHeader.ipv6Header.DestAddress;
}
void  TCPIP_IPV6_SetDestAddress(IP_PACKET * p, IPV6_ADDR * addr)
{
    if(addr)
    {
        memcpy (&p->ipHeader.ipv6Header.DestAddress, (void *)addr, sizeof (IPV6_ADDR));
    }
    else
    {
        memset (&p->ipHeader.ipv6Header.DestAddress, 0x0, sizeof (IPV6_ADDR));
    }
}
void  TCPIP_IPV6_SetSourceAddress(IP_PACKET * p, IPV6_ADDR * addr)
{
    if(addr)
    {
        memcpy (&p->ipHeader.ipv6Header.SourceAddress, addr, sizeof(IPV6_ADDR));
        p->flags.sourceSpecified = true;
    }
    else
    {
        memset (&p->ipHeader.ipv6Header.SourceAddress, 0x0, sizeof(IPV6_ADDR));
        p->flags.sourceSpecified = false;
    }
}
IPV6_ADDR *  TCPIP_IPV6_GetSourceAddress(IP_PACKET * p)
{
    return &p->ipHeader.ipv6Header.SourceAddress;
}


// Register an IPv6 event handler
// Use hNet == 0 to register on all interfaces available
IPV6_HANDLE TCPIP_IPV6_RegisterHandler(TCPIP_NET_HANDLE hNet, IPV6_EVENT_HANDLER handler, const void* hParam)
{
    if(ipMemH)
    {
        IPV6_LIST_NODE* newNode = TCPIP_HEAP_Malloc(ipMemH, sizeof(*newNode));

        if(newNode)
        {
            newNode->handler = handler;
            newNode->hParam = hParam;
            newNode->hNet = hNet;
            SingleListAddTail(&ipv6RegisteredUsers, (SGL_LIST_NODE*)newNode);
            return newNode;
        }
    }

    return 0;
}

// deregister the event handler
bool TCPIP_IPV6_DeRegisterHandler(IPV6_HANDLE hIpv6)
{
    if(hIpv6 && ipMemH)
    {
        if(SingleListRemoveNode(&ipv6RegisteredUsers, (SGL_LIST_NODE*)hIpv6))
        {
            TCPIP_HEAP_Free(ipMemH, hIpv6);
            return true;
        }
    }

    return false;
}

void TCPIP_IPV6_NotifyClients(NET_CONFIG* pNetIf, IPV6_EVENT_TYPE evType)
{
    IPV6_LIST_NODE* iNode;

    for(iNode = (IPV6_LIST_NODE*)ipv6RegisteredUsers.head; iNode != 0; iNode = iNode->next)
    {
        if(iNode->hNet == 0 || iNode->hNet == pNetIf)
        {   // trigger event
            (*iNode->handler)(pNetIf, evType, iNode->hParam);
        }
    }
    
}

#endif  // defined (TCPIP_STACK_USE_IPV6)
