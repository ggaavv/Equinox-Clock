/*******************************************************************************
  Neighbor Discovery Protocol (NDP)
*******************************************************************************/

/*******************************************************************************
FileName:   NDP.c
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
#include "ndp_private.h"

#include "link_list.h"

#if defined (TCPIP_STACK_USE_IPV6)

IPV6_ADDR tempAddress;

extern IPV6_INTERFACE_CONFIG ipv6Config[];

typedef struct
{
    IPV6_ADDR_STRUCT * addressPointer;
    uint32_t timer;
    NET_CONFIG * netConfig;
    uint8_t state;
    uint8_t transmitAttempt;
    uint8_t receivedSolicitations;
} DAD_STATIC_VARS;

enum
{
    DAD_STATE_INACTIVE = 0,
    DAD_STATE_TRANSMIT,
    DAD_STATE_WAIT,
    DAD_STATE_FAIL,
    DAD_STATE_DONE
} DUPLICATE_ADDR_DISCOVERY_STATE;

DAD_STATIC_VARS gDuplicateAddrDetectState[DUPLICATE_ADDR_DISCOVERY_THREADS];

typedef struct
{
    uint32_t timer;
    uint8_t state;
    uint8_t transmitAttempt;
    IPV6_ADDR_STRUCT * address;
} RS_STATIC_VARS;

RS_STATIC_VARS gRSState[TCPIP_NETWORK_INTERFACES];

uint32_t gInitialDelay;
static const void*      ndpMemH = 0;        // memory handle
static bool             ndpInit = false;
static int              nStackIfs = 0;      // max number of interfaces
enum
{
    RS_STATE_INACTIVE = 0,
    RS_STATE_INIT,
    RS_STATE_TRANSMIT,
    RS_STATE_WAIT
} ROUTER_SOLICITATION_STATE;

/*****************************************************************************
  Function:
	bool TCPIP_NDP_Initialize (const TCPIP_STACK_MODULE_CTRL* const stackInit)

  Summary:
	Initializes the Neighbor Discovery Protocol module.

  Description:
	This function initializes the Neighbor Discovery Protocol module, including
    setting up and initial random delay, the router solicitation state machine,
    and the heap used to dynamically allocate structures for NDP.

  Precondition:
	None

  Parameters:
	stackInit - TCP/IP Stack module initializer structure

  Returns:
  	true
  	
  Remarks:
	None
  ***************************************************************************/
bool TCPIP_NDP_Initialize (const TCPIP_STACK_MODULE_CTRL* const stackInit,
                    const void* const ndpData)
{
    uint8_t i;
    int netIx;

    if(stackInit->stackAction == TCPIP_STACK_ACTION_IF_UP)
    {   // interface restart
        return true;
    }

    // stack init
    netIx = stackInit->netIx;

    // Perform one-time initialization
    if (ndpInit == false)
    {
        ndpInit = true;
        for (i = 0; i < DUPLICATE_ADDR_DISCOVERY_THREADS; i++)
        {
            gDuplicateAddrDetectState[i].state = DAD_STATE_INACTIVE;
        }
    
        ndpMemH = stackInit->memH;
        nStackIfs = stackInit->nIfs;
    }

    gRSState[netIx].state = RS_STATE_INACTIVE;
    gRSState[netIx].transmitAttempt = 0;

    gInitialDelay = SYS_TICK_Get() + (((LFSRRand() % 40) * SYS_TICK_ResolutionGet() * MAX_RTR_SOLICITATION_DELAY) / 40);

    return true;
}

/*****************************************************************************
  Function:
	void TCPIP_NDP_Deinitialize(const TCPIP_STACK_MODULE_CTRL* const stackInit)

  Summary:
	Disabled NDP functionality on a per-interface basis.

  Description:
	This function will disable NDP functionality on a specified interface. 

  Precondition:
	None

  Parameters:
	stackInit - Stack initialization parameters

  Returns:
  	None
  	
  Remarks:
	None
  ***************************************************************************/
void TCPIP_NDP_Deinitialize(const TCPIP_STACK_MODULE_CTRL* const stackInit)
{
    //if(stackInit->stackAction == TCPIP_STACK_ACTION_DEINIT)   // stack down
    //if(stackInit->stackAction == TCPIP_STACK_ACTION_IF_DOWN)  // interface down

    gRSState[stackInit->netIx].state = RS_STATE_INACTIVE;
}

/*****************************************************************************
  Function:
	IPV6_HEAP_NDP_NC_ENTRY * TCPIP_NDP_NeighborEntryCreate (
        NET_CONFIG * pNetIf, IPV6_ADDR * remoteIPAddr, 
        MAC_ADDR * remoteMACAddr, unsigned char initialState, 
        unsigned char routerFlag, IPV6_ADDR_STRUCT * preferredSource)

  Summary:
	Creates an entry in the neighbor cache.

  Description:
	This function will dynamically allocate an entry and add it to the linked-
    list of neighbor cache entries for the specified interface.

  Precondition:
	The heap used by this module must be initialized.

  Parameters:
	pNetIf - The interface that the neighbor is on
    remoteIPAddress - The neighbor's IPv6 address
    remoteMACAddress - The neighbor's link-layer address (can be NULL)
    initialState - The neighbor's initial Neighbor Unreachability Detection 
        state
    routerFlag - true if the neighbor is a router, false otherwise
    preferredSource - The source address to use when sending traffic to this 
        neighbor (if possible)

  Returns:
  	IPV6_HEA_NDP_NC_ENTRY * - Pointer to the new Neighbor Cache Entry or 
                                NULL if one can't be allocated
  	
  Remarks:
	None
  ***************************************************************************/
IPV6_HEAP_NDP_NC_ENTRY * TCPIP_NDP_NeighborEntryCreate (NET_CONFIG * pNetIf, IPV6_ADDR * remoteIPAddr, MAC_ADDR * remoteMACAddr, unsigned char initialState, unsigned char routerFlag, IPV6_ADDR_STRUCT * preferredSource)
{
    IPV6_HEAP_NDP_NC_ENTRY * neighborPointer;

    neighborPointer = (IPV6_HEAP_NDP_NC_ENTRY *)TCPIP_HEAP_Malloc (ndpMemH, sizeof (IPV6_HEAP_NDP_NC_ENTRY));

    if (neighborPointer != NULL)
    {
        memcpy (&(neighborPointer->remoteIPAddress), remoteIPAddr, sizeof (IPV6_ADDR));
        if (remoteMACAddr != NULL)
            memcpy(&(neighborPointer->remoteMACAddr), remoteMACAddr, sizeof (MAC_ADDR));
        else
            memset (&(neighborPointer->remoteMACAddr), 0x00, sizeof (MAC_ADDR));

        TCPIP_NDP_SetReachability (pNetIf, neighborPointer, initialState);
        neighborPointer->unansweredProbes = 0;
        neighborPointer->nextNUDTime = 0;
        neighborPointer->queuedPacketPointer = NULL;
        if (routerFlag)
            neighborPointer->flags.bIsRouter = true;
        else
            neighborPointer->flags.bIsRouter = false;

        neighborPointer->preferredSource = preferredSource;

        TCPIP_NDP_LinkedListEntryInsert (pNetIf, neighborPointer, IPV6_HEAP_NDP_NC_ID);

        // Let the IP layer start address resolution if there is traffic to send
        neighborPointer->flags.bResolvingAddress = false;

//        if (initialState == NDP_STATE_INCOMPLETE)
//            TCPIP_NDP_ResolveAddress (neighborPointer);
    }
    return neighborPointer;
}

/*****************************************************************************
  Function:
    IPV6_HEAP_NDP_DR_ENTRY * TCPIP_NDP_DefaultRouterEntryCreate (
        NET_CONFIG * pNetIf, IPV6_HEAP_NDP_NC_ENTRY * neighbor, 
        unsigned long invalidationTime)	

  Summary:
	Creates an entry in the default router list.

  Description:
	This function will dynamically allocate an entry and add it to the linked-
    list of default router entries for the specified interface.

  Precondition:
	The heap used by this module must be initialized.

  Parameters:
	pNetIf - The interface that the router is on
    neighbor - The neighbor cache entry of the router
    invalidationTime - The router's invalidation timer

  Returns:
  	IPV6_HEA_NDP_DR_ENTRY * - Pointer to the new Default Router Entry or 
                                NULL if one can't be allocated
  	
  Remarks:
	None
  ***************************************************************************/
IPV6_HEAP_NDP_DR_ENTRY * TCPIP_NDP_DefaultRouterEntryCreate (NET_CONFIG * pNetIf, IPV6_HEAP_NDP_NC_ENTRY * neighbor, unsigned long invalidationTime)
{
    IPV6_HEAP_NDP_DR_ENTRY * routerPointer;

    routerPointer = (IPV6_HEAP_NDP_DR_ENTRY *)TCPIP_HEAP_Malloc(ndpMemH, sizeof(IPV6_HEAP_NDP_DR_ENTRY));

    if (routerPointer != NULL)
    {
        routerPointer->neighborInfo = neighbor;
        routerPointer->invalidationTimer = invalidationTime;
        routerPointer->tickTimer = SYS_TICK_Get();
        TCPIP_NDP_LinkedListEntryInsert (pNetIf, routerPointer, IPV6_HEAP_NDP_DR_ID);
    }

    return routerPointer;
}

/*****************************************************************************
  Function:
    IPV6_HEAP_NDP_DC_ENTRY * TCPIP_NDP_DestinationCacheEntryCreate (
        NET_CONFIG * pNetIf, IPV6_ADDR * remoteIPAddress, 
        unsigned long linkMTU, IPV6_HEAP_NDP_NC_ENTRY * neighbor)

  Summary:
	Creates an entry in the destination cache.

  Description:
	This function will dynamically allocate an entry and add it to the linked-
    list of destination cache entries for the specified interface.

  Precondition:
	The heap used by this module must be initialized.

  Parameters:
	pNetIf - The interface that the destination communicates with
    remoteIPAddress - The destination IP address
    linkMTU - The MTU of the link to the destination
    neighbor - The first hop to reach the destination.

  Returns:
  	IPV6_HEA_NDP_DC_ENTRY * - Pointer to the new Destination Cache Entry or 
                                NULL if one can't be allocated
  	
  Remarks:
	None
  ***************************************************************************/
IPV6_HEAP_NDP_DC_ENTRY * TCPIP_NDP_DestinationCacheEntryCreate (NET_CONFIG * pNetIf, IPV6_ADDR * remoteIPAddress, unsigned long linkMTU, IPV6_HEAP_NDP_NC_ENTRY * neighbor)
{
    IPV6_HEAP_NDP_DC_ENTRY * destinationPointer;

    destinationPointer = (IPV6_HEAP_NDP_DC_ENTRY *)TCPIP_HEAP_Malloc(ndpMemH, sizeof(IPV6_HEAP_NDP_DC_ENTRY));

    if (destinationPointer != NULL)
    {
        memcpy (&(destinationPointer->remoteIPAddress), remoteIPAddress, sizeof (IPV6_ADDR));
        destinationPointer->pathMTU = linkMTU;
        destinationPointer->nextHopNeighbor = neighbor;
        if (linkMTU != IPV6_DEFAULT_LINK_MTU)
            destinationPointer->pathMTUIncreaseTimer = SYS_TICK_Get() + (SYS_TICK_ResolutionGet() * IPV6_MTU_INCREASE_TIMEOUT);
        else
            destinationPointer->pathMTUIncreaseTimer = 0;
        TCPIP_NDP_LinkedListEntryInsert (pNetIf, destinationPointer, IPV6_HEAP_NDP_DC_ID);
    }

    return destinationPointer;
}

/*****************************************************************************
  Function:
    IPV6_HEAP_NDP_PL_ENTRY * TCPIP_NDP_PrefixListEntryCreate (
        NET_CONFIG * pNetIf, IPV6_ADDR * prefix, 
        unsigned char prefixLength, unsigned long validLifetime)

  Summary:
	Creates an entry in the prefix list.

  Description:
	This function will dynamically allocate an entry and add it to the linked-
    list of on-link prefix entries for the specified interface.

  Precondition:
	The heap used by this module must be initialized.

  Parameters:
	pNetIf - The interface that the prefix is on-link for.
    prefix - An IPv6 address containing the prefix
    prefixLength - Length of the prefix (bits)
    validLifetime - Amount of time the prefix can be considered valid

  Returns:
  	IPV6_HEA_NDP_PL_ENTRY * - Pointer to the new Prefix List Entry or 
                                NULL if one can't be allocated
  	
  Remarks:
	None
  ***************************************************************************/
IPV6_HEAP_NDP_PL_ENTRY * TCPIP_NDP_PrefixListEntryCreate (NET_CONFIG * pNetIf, IPV6_ADDR * prefix, unsigned char prefixLength, unsigned long validLifetime)
{
    IPV6_HEAP_NDP_PL_ENTRY * entryPointer;

    entryPointer = (IPV6_HEAP_NDP_PL_ENTRY *)TCPIP_HEAP_Malloc (ndpMemH, sizeof (IPV6_HEAP_NDP_PL_ENTRY));

    if (entryPointer != NULL)
    {
        memcpy (&entryPointer->prefix, prefix, sizeof (IPV6_ADDR));
        entryPointer->prefixLength = prefixLength;
        entryPointer->validLifetime = validLifetime;
        entryPointer->lastTickTime = SYS_TICK_Get();

        TCPIP_NDP_LinkedListEntryInsert (pNetIf, entryPointer, IPV6_HEAP_NDP_PL_ID);
    }

    return entryPointer;
}

/*****************************************************************************
  Function:
    IPV6_HEAP_NDP_NC_ENTRY * TCPIP_NDP_NeighborEntryDelete (
        NET_CONFIG * pNetIf, IPV6_HEAP_NDP_NC_ENTRY * entry)	

  Summary:
	Deletes a neighbor cache entry.

  Description:
	This function will delete a neighbor cache entry.  If the neighbor was 
    a router, it will remove it from the default router list.  It will 
    remove any destination cache entries that use this neighbor as 
    a first hop.  It will deallocate any queued packets for this neighbor.

  Precondition:
	None

  Parameters:
	pNetIf - The interface the neighbor is on.
    entry - The neighbor cache entry.

  Returns:
  	IPV6_HEAP_NDP_NC_ENTRY * - The neighbor cache entry in the linked list 
        after the entry that was removed.  NULL if the removed entry was the 
        last one in the list.
  	
  Remarks:
	None
  ***************************************************************************/
IPV6_HEAP_NDP_NC_ENTRY * TCPIP_NDP_NeighborEntryDelete (NET_CONFIG * pNetIf, IPV6_HEAP_NDP_NC_ENTRY * entry)
{
    IPV6_HEAP_NDP_NC_ENTRY * ptrNeighbor;
    IPV6_HEAP_NDP_DC_ENTRY * ptrDestination;

    int netIx;

    if (pNetIf == NULL || entry == NULL)
        return NULL;

    netIx = _TCPIPStackNetIx (pNetIf);

    while (entry->queuedPacketPointer != NULL)
    {
        TCPIP_IPV6_RemoveQueuedPacket (entry, entry->queuedPacketPointer);
    }

    // If this entry is a router, see if we need to remove it from the default router list
    if (entry->flags.bIsRouter)
    {
        IPV6_HEAP_NDP_DR_ENTRY * ptrRouter;

        if ((ptrRouter = (IPV6_HEAP_NDP_DR_ENTRY *)TCPIP_NDP_FindRemoteNode (pNetIf, &entry->remoteIPAddress, IPV6_HEAP_NDP_DR_ID)) != NULL)
        {
            TCPIP_NDP_LinkedListEntryRemove (pNetIf, ptrRouter, IPV6_HEAP_NDP_DR_ID);
        }
    }

    ptrNeighbor = TCPIP_NDP_LinkedListEntryRemove (pNetIf, entry, IPV6_HEAP_NDP_NC_ID);

    ptrDestination = (IPV6_HEAP_NDP_DC_ENTRY *)ipv6Config[netIx].listDestinationCache.head;

    while (ptrDestination != NULL)
    {
        if (ptrDestination->nextHopNeighbor == entry)
        {
            ptrDestination = TCPIP_NDP_LinkedListEntryRemove (pNetIf, ptrDestination, IPV6_HEAP_NDP_DC_ID);
        }
        else
        {
            ptrDestination = ptrDestination->next;
        }
    }

    return ptrNeighbor;
}

/*****************************************************************************
  Function:
    void TCPIP_NDP_NeighborConfirmReachability (
        NET_CONFIG * pNetIf, IPV6_ADDR * address)

  Summary:
	Confirms that a neighbor is reachable.

  Description:
	This function is used by upper-layer protocols to indicate that round-trip 
    communications were confirmed with a neighboring node.

  Precondition:
	None

  Parameters:
	pNetIf - The interface the neighbor is on.
    address - The address of the neighbor.

  Returns:
  	None.
  	
  Remarks:
	None
  ***************************************************************************/
void TCPIP_NDP_NeighborConfirmReachability (NET_CONFIG * pNetIf, IPV6_ADDR * address)
{
    IPV6_HEAP_NDP_NC_ENTRY * entry;

    if (pNetIf == NULL && address == NULL)
        return;

    if ((entry = TCPIP_NDP_FindRemoteNode (pNetIf, address, IPV6_HEAP_NDP_NC_ID)) !=  NULL)
    {
        TCPIP_NDP_SetReachability (pNetIf, entry, NDP_STATE_REACHABLE);
    }
}

/*****************************************************************************
  Function:
    IPV6_HEAP_NDP_DR_ENTRY * TCPIP_NDP_GetDefaultRouter (NET_CONFIG * pNetIf)

  Summary:
	Gets a default router for the given interface.

  Description:
	This function determines the default router that should be selected for 
    a given interface.  The algorithm will prefer routers with a known
    link-layer address.  If none are available, a default router will be 
    selected round-robin.

  Precondition:
	None

  Parameters:
	pNetIf - The interface to get the router for.

  Returns:
  	IPV6_HEAP_NDP_DR_ENTRY * - The default router.  NULL if no router 
                                is available.
  	
  Remarks:
	None
  ***************************************************************************/
IPV6_HEAP_NDP_DR_ENTRY * TCPIP_NDP_GetDefaultRouter (NET_CONFIG * pNetIf)
{
    IPV6_HEAP_NDP_DR_ENTRY * routerPointer;
    int netIx;

    if (pNetIf == NULL)
        return NULL;

    netIx = _TCPIPStackNetIx(pNetIf);

    routerPointer = (IPV6_HEAP_NDP_DR_ENTRY *)ipv6Config[netIx].listDefaultRouter.head;

    while (routerPointer != NULL)
    {
        if ((routerPointer->neighborInfo->flags.bIsRouter) && (routerPointer->invalidationTimer > 0) && (routerPointer->neighborInfo->reachabilityState != NDP_STATE_INCOMPLETE) && (routerPointer->neighborInfo->reachabilityState != NDP_STATE_NONE))
        {
            ipv6Config[netIx].currentDefaultRouter = routerPointer;
            return routerPointer;
        }
        routerPointer = routerPointer->next;
    }

    // Can't find any non-incomplete routers
    // Select the default router round-robin-style
    routerPointer = ipv6Config[netIx].currentDefaultRouter;
    if (routerPointer == NULL)
        routerPointer = (IPV6_HEAP_NDP_DR_ENTRY *)ipv6Config[netIx].listDefaultRouter.head;
    if (routerPointer != NULL)
    {
        if (routerPointer->next != NULL)
            ipv6Config[netIx].currentDefaultRouter = routerPointer->next;
        else
            ipv6Config[netIx].currentDefaultRouter = routerPointer;
    }
    else
    {
        ipv6Config[netIx].currentDefaultRouter = NULL;
    }

    return ipv6Config[netIx].currentDefaultRouter;
}

/*****************************************************************************
  Function:
	void TCPIP_NDP_DAD_Task (void)

  Summary:
	Performs tasks for Duplicate Address Discovery

  Description:
	This function performs Duplicate Address Discovery tasks.

  Precondition:
	None

  Parameters:
	None

  Returns:
  	None
  	
  Remarks:
	None
  ***************************************************************************/
void TCPIP_NDP_DAD_Task (void)
{
    IPV6_ADDR_STRUCT * addressPointer;
    char result;
    int netIx;

    for(netIx=0; netIx < nStackIfs; netIx++)    {
        addressPointer = (IPV6_ADDR_STRUCT *)ipv6Config[netIx].listIpv6TentativeAddresses.head;
    
        while (addressPointer != NULL)
        {
            result = TCPIP_NDP_DAD_GetAddressStatus (addressPointer);
            switch (result)
            {
                case DAD_ADDRESS_DUPLICATED:
                case DAD_BAD_ARGUMENT:
                    TCPIP_NDP_LinkedListEntryRemove ((NET_CONFIG*)TCPIP_STACK_IxToNet(netIx), addressPointer, IPV6_HEAP_ADDR_UNICAST_TENTATIVE_ID);
                    break;
                case DAD_OK:
                    addressPointer = TCPIP_NDP_TentativeAddressPromote ((NET_CONFIG*)TCPIP_STACK_IxToNet(netIx), addressPointer);
                    break;
                case DAD_PENDING:
                default:
                    addressPointer = addressPointer->next;
                    break;
            }
        }
    }
}

/*****************************************************************************
  Function:
    char TCPIP_NDP_DAD_DetectDuplicateAddress (
        NET_CONFIG * pNetIf, IPV6_ADDR_STRUCT * localAddressPointer)	

  Summary:
	Initiates duplicate address detection for a given address.

  Description:
	This function will begin duplicate address detection for a locally 
    configured unicast address.  Until the address is found to be unique it 
    will be treated as a tentative address.

  Precondition:
	None

  Parameters:
	pNetIf - The interface the address is configured on.
    localAddressPointer - The local address structure

  Returns:
  	char - DAD_UNAVAILABLE or the index of the duplicate address state 
            structure for this test.
  	
  Remarks:
	None
  ***************************************************************************/
char TCPIP_NDP_DAD_DetectDuplicateAddress (NET_CONFIG * pNetIf, IPV6_ADDR_STRUCT * localAddressPointer)
{
    unsigned char i;


    for (i = 0; i < DUPLICATE_ADDR_DISCOVERY_THREADS; i++)
    {
        if (gDuplicateAddrDetectState[i].state == DAD_STATE_INACTIVE)
        {
            gDuplicateAddrDetectState[i].state = DAD_STATE_TRANSMIT;
            gDuplicateAddrDetectState[i].addressPointer = localAddressPointer;
            gDuplicateAddrDetectState[i].transmitAttempt = 0;
            gDuplicateAddrDetectState[i].receivedSolicitations = 0;
            gDuplicateAddrDetectState[i].netConfig = pNetIf;
            return i;
        }
    }

    // No free IP addr slots or DAD thread slots
    return DAD_UNAVAILABLE;
}


/*****************************************************************************
  Function:
	char TCPIP_NDP_DAD_GetAddressStatus (IPV6_ADDR_STRUCT * localAddressPointer)

  Summary:
	Determines the status of an address undergoing duplicate address detection.

  Description:
	Determines the status of an address undergoing duplicate address detection.
	

  Precondition:
	None

  Parameters:
	localAddressPointer - Pointer to the address undergoing DAD.

  Returns:
    DAD_OK - The address is not a duplicate
    DAD_ADDRESS_DUPLICATED - The address is a duplicate
    DAD_PENDING
    DAD_BAD_ARGUMENT
  	
  Remarks:
	None
  ***************************************************************************/
char TCPIP_NDP_DAD_GetAddressStatus (IPV6_ADDR_STRUCT * localAddressPointer)
{
    uint8_t i;

    // Add a delay of 0-MAX_RTR_SOLICITATION_DELAY before the first transmission
    if (gInitialDelay)
    {
        if ((long)(SYS_TICK_Get() - gInitialDelay) > 0)
        {
            // Remove initial delay
            gInitialDelay = 0;
        }
        else
        {
            return DAD_PENDING;
        }
    }

    for (i = 0; i < DUPLICATE_ADDR_DISCOVERY_THREADS; i++)
    {
        // Determine if any of our active threads matches the address we just received
        if (gDuplicateAddrDetectState[i].addressPointer == localAddressPointer)
        {
            switch (gDuplicateAddrDetectState[i].state)
            {
                case DAD_STATE_INACTIVE:
                    return DAD_BAD_ARGUMENT;
                    break;
                case DAD_STATE_TRANSMIT:
                    {
                        IPV6_ADDR solicitedNodeMulticastAddr = {{0xFF, 0x02, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
                                                                    0x00, 0x00, 0x00, 0x01, 0xFF, 
                                                                    gDuplicateAddrDetectState[i].addressPointer->address.v[13],
                                                                    gDuplicateAddrDetectState[i].addressPointer->address.v[14], 
                                                                    gDuplicateAddrDetectState[i].addressPointer->address.v[15]}};
                        IP_PACKET * pkt;

                        pkt = TCPIP_ICMPV6_PutHeaderNeighborSolicitation (gDuplicateAddrDetectState[i].netConfig, (IPV6_ADDR *)&IPV6_FIXED_ADDR_UNSPECIFIED, &solicitedNodeMulticastAddr, &gDuplicateAddrDetectState[i].addressPointer->address);
                        if (pkt != NULL)
                        {
                            if (!TCPIP_ICMPV6_Flush(pkt))
                                return DAD_PENDING;
                            gDuplicateAddrDetectState[i].transmitAttempt++;
                            gDuplicateAddrDetectState[i].timer = SYS_TICK_Get() + ((SYS_TICK_ResolutionGet() * ipv6Config[_TCPIPStackNetIx(gDuplicateAddrDetectState[i].netConfig)].retransmitTime) / 1000);
                            gDuplicateAddrDetectState[i].state = DAD_STATE_WAIT;
                        }
                    }
                    return DAD_PENDING;
                    break;
                case DAD_STATE_WAIT:
                    if ((long)(SYS_TICK_Get() - gDuplicateAddrDetectState[i].timer) > 0)
                    {
                        if (gDuplicateAddrDetectState[i].transmitAttempt == DUPLICATE_ADDR_DETECT_TRANSMITS)
                            gDuplicateAddrDetectState[i].state = DAD_STATE_DONE;
                        else
                            gDuplicateAddrDetectState[i].state = DAD_STATE_TRANSMIT;
                    }
                    return DAD_PENDING;
                    break;
                case DAD_STATE_DONE:
                    gDuplicateAddrDetectState[i].state = DAD_STATE_INACTIVE;
                    return DAD_OK;
                    break;
                case DAD_STATE_FAIL:
                    gDuplicateAddrDetectState[i].state = DAD_STATE_INACTIVE;
                    return DAD_ADDRESS_DUPLICATED;
                    break;
                default:
                    gDuplicateAddrDetectState[i].state = DAD_STATE_INACTIVE;
                    return DAD_BAD_ARGUMENT;
                    break;
            }
        }
    }

    // Could not find matching entry
    return DAD_BAD_ARGUMENT;
}

/*****************************************************************************
  Function:
	void TCPIP_NDP_DAD_EventProcess (IPV6_ADDR_STRUCT * localAddressPointer, 
                                        uint8_t type)

  Summary:
	Handles reception of packets from duplicated addresses.

  Description:
	This function is called if a packet is received from an address that is 
    undergoing duplicate address detection.  It will deallocate that 
    local address.

  Precondition:
	None

  Parameters:
	localAddressPointer - Pointer to the address that is duplicated.
    type - Type of packet receives (this field is currently unused because 
            all available MAC/PHYs have loopback filtering).

  Returns:
  	None
  	
  Remarks:
	None
  ***************************************************************************/
void TCPIP_NDP_DAD_EventProcess (IPV6_ADDR_STRUCT * localAddressPointer, uint8_t type)
{
    uint8_t i;

    for (i = 0; i < DUPLICATE_ADDR_DISCOVERY_THREADS; i++)
    {
        // Determine if any of our active threads matches the address we just received
        if (gDuplicateAddrDetectState[i].addressPointer == localAddressPointer)
        {
            if (type == IPV6_NDP_DAD_NS_RECEIVED)
            {
//                if (++gDuplicateAddrDetectState[i].receivedSolicitations > gDuplicateAddrDetectState[i].transmitAttempt ||
  //                  gDuplicateAddrDetectState[i].receivedSolicitations > DUPLICATE_ADDR_DETECT_TRANSMITS)
    //            {
                    gDuplicateAddrDetectState[i].state = DAD_STATE_FAIL;
      //          }
            }
            else if (type == IPV6_NDP_DAD_NA_RECEIVED)
            {
                gDuplicateAddrDetectState[i].state = DAD_STATE_FAIL;
            }

            return;
        }
    }
}

/*****************************************************************************
  Function:
	void TCPIP_NDP_RS_Start (NET_CONFIG * pNetIf)

  Summary:
	Begins router solicitation.

  Description:
	This function will initialize the state machine for router solicitation.

  Precondition:
	None

  Parameters:
	pNetIf - The interface to solicit routers on.

  Returns:
  	None
  	
  Remarks:
	None
  ***************************************************************************/
void TCPIP_NDP_RS_Start (NET_CONFIG * pNetIf)
{
    int netIx;

    netIx = _TCPIPStackNetIx(pNetIf);
    gRSState[netIx].state = RS_STATE_TRANSMIT;
    gRSState[netIx].transmitAttempt = 0;
}

/*****************************************************************************
  Function:
    void TCPIP_NDP_RS_Task (void)	

  Summary:
	Task function for router solicitation.

  Description:
	This function performs periodic tasks required for router solicitation.

  Precondition:
	None

  Parameters:
	None

  Returns:
  	None
  	
  Remarks:
	None
  ***************************************************************************/
void TCPIP_NDP_RS_Task (void)
{
    NDP_OPTION_LLA sllaOption;
    IP_PACKET * pkt;
    int netIx;
    NET_CONFIG * pNetIf;

    for(netIx=0; netIx< sizeof(gRSState)/sizeof(*gRSState); netIx++)    {
        pNetIf = (NET_CONFIG*)TCPIP_STACK_IxToNet (netIx);

        switch (gRSState[netIx].state)
        {
            case RS_STATE_INACTIVE:
    
                break;
            case RS_STATE_TRANSMIT:
                if ((gRSState[netIx].address == NULL) || (gRSState[netIx].address->flags.scope != IPV6_ADDR_SCOPE_LINK_LOCAL))
                {
                    // Note: Addresses in this list are all unicast, so we can skip that check
                    gRSState[netIx].address = (IPV6_ADDR_STRUCT *)ipv6Config[netIx].listIpv6UnicastAddresses.head;
                    while ((gRSState[netIx].address != NULL) &&
                            (gRSState[netIx].address->flags.scope != IPV6_ADDR_SCOPE_LINK_LOCAL))
                    {
                        gRSState[netIx].address = gRSState[netIx].address->next;
                    }
    
                    if (gRSState[netIx].address == NULL)
                    {
                        // If no address can be found, send the solicitation with the unspecified address
                        pkt = TCPIP_ICMPV6_PutHeaderRouterSolicitation (pNetIf, (IPV6_ADDR *)&IPV6_FIXED_ADDR_UNSPECIFIED, (IPV6_ADDR *)&IPV6_FIXED_ADDR_ALL_ROUTER_MULTICAST);
                        TCPIP_ICMPV6_Flush (pkt);
                    }
                    else
                    {
                        sllaOption.vType = NDP_OPTION_TYPE_LLA_SOURCE;
                        sllaOption.vLength = 1;
                        memcpy(&sllaOption.mLinkLayerAddr , &(pNetIf->MyMACAddr), sizeof (MAC_ADDR));
                        pkt = TCPIP_ICMPV6_PutHeaderRouterSolicitation (pNetIf, &gRSState[netIx].address->address, (IPV6_ADDR *)&IPV6_FIXED_ADDR_ALL_ROUTER_MULTICAST);
                        if (pkt == NULL)
                            return;
                        if (TCPIP_IP_IsTxPutReady(pkt, sizeof(NDP_OPTION_LLA)))
                        {
                            TCPIP_IP_PutArray(pkt, (uint8_t*)&sllaOption, sizeof(NDP_OPTION_LLA));
                        }
                        else
                        {
                            TCPIP_IP_FreePacket (pkt);
                            return;
                        }
                        TCPIP_ICMPV6_Flush (pkt);
                    }
                }
                else
                {
                    sllaOption.vType = NDP_OPTION_TYPE_LLA_SOURCE;
                    sllaOption.vLength = 1;
                    memcpy(&sllaOption.mLinkLayerAddr , &(pNetIf->MyMACAddr), sizeof (MAC_ADDR));
                    // The previously selected IP address is still valid; use it
                    pkt = TCPIP_ICMPV6_PutHeaderRouterSolicitation (pNetIf, &gRSState[netIx].address->address, (IPV6_ADDR *)&IPV6_FIXED_ADDR_ALL_ROUTER_MULTICAST);
                    if (pkt == NULL)
                        return;
                    if (TCPIP_IP_IsTxPutReady(pkt, sizeof(NDP_OPTION_LLA)))
                    {
                        TCPIP_IP_PutArray(pkt, (uint8_t*)&sllaOption, sizeof(NDP_OPTION_LLA));
                    }
                    else
                    {
                        TCPIP_IP_FreePacket (pkt);
                        return;
                    }
                    TCPIP_ICMPV6_Flush (pkt);
                }
                gRSState[netIx].transmitAttempt++;
                gRSState[netIx].timer = SYS_TICK_Get() + (SYS_TICK_ResolutionGet() * RTR_SOLICITATION_INTERVAL);
                gRSState[netIx].state = RS_STATE_WAIT;
                break;
            case RS_STATE_WAIT:     
                if ((long)(SYS_TICK_Get() - gRSState[netIx].timer) > 0)
                {
                    if (gRSState[netIx].transmitAttempt == MAX_RTR_SOLICITATIONS)
                        gRSState[netIx].state = RS_STATE_INACTIVE;
                    else
                        gRSState[netIx].state = RS_STATE_TRANSMIT;
                }
                break;
        }
    }
}

/*****************************************************************************
  Function:
	void TCPIP_NDP_RS_Stop (NET_CONFIG * pNetIf)

  Summary:
	Stops router solicitation.

  Description:
	This function will stop router solicitation.  It will be called if the 
    soliciting interface receives a router advertisement.

  Precondition:
	None

  Parameters:
	pNetIf - The interface soliciting for routers.

  Returns:
  	None
  	
  Remarks:
	None
  ***************************************************************************/
void TCPIP_NDP_RS_Stop (NET_CONFIG * pNetIf)
{
    int netIx;

    if (pNetIf == NULL)
        return;

    netIx = _TCPIPStackNetIx (pNetIf);

    // Make sure we've sent AT LEAST 1 Router Solicitation
    // If so, disable router solicitations
    if (gRSState[netIx].transmitAttempt != 0)
        gRSState[netIx].state = RS_STATE_INACTIVE;
}

/*****************************************************************************
  Function:
	void * TCPIP_NDP_FindPrefix (NET_CONFIG * pNetIf, IPV6_ADDR * prefix, 
                                    unsigned char prefixLength, 
                                    unsigned char usePrefixList)

  Summary:
	Finds a prefix.

  Description:
	This function either finds a prefix with a set length in the prefix list 
    or finds an address that was created using the specified prefix.

  Precondition:
	None

  Parameters:
	pNetIf - The interface of the prefix.
    prefix - The prefix.
    prefixLength - The length of the prefix.
    usePrefixList - true if finding a matching prefix in the prefix list, 
                    false if finding an address autoconfigured using the prefix

  Returns:
  	void * - IPV6_HEAP_NDP_PL_ENTRY * or IPV6_ADDR_STRUCT * of any matching 
                nodes
  	
  Remarks:
	None
  ***************************************************************************/
void * TCPIP_NDP_FindPrefix (NET_CONFIG * pNetIf, IPV6_ADDR * prefix, unsigned char prefixLength, unsigned char usePrefixList)
{
    IPV6_HEAP_NDP_PL_ENTRY * prefixPointer;
    IPV6_ADDR_STRUCT * localAddressPointer;
    int netIx;

    if (pNetIf == NULL)
        return NULL;

    netIx = _TCPIPStackNetIx (pNetIf);

    prefixPointer = (IPV6_HEAP_NDP_PL_ENTRY *)ipv6Config[netIx].listPrefixList.head;
    localAddressPointer = (IPV6_ADDR_STRUCT *)ipv6Config[netIx].listIpv6UnicastAddresses.head;

    if (usePrefixList)
    {
        // Find a matching prefix in the prefix list
        while (prefixPointer != NULL)
        {
            if (prefixLength == prefixPointer->prefixLength)
            {
                if (memcmp (prefix, &prefixPointer->prefix, sizeof (IPV6_ADDR)))
                {
                    return prefixPointer;
                }
            }
            prefixPointer = prefixPointer->next;
        }
    }
    else
    {
        // Find an address that matches the prefix that was created using Stateless Address Autoconfiguration
        while (localAddressPointer != NULL)
        {
            if (localAddressPointer->prefixLen != 0)
            {
                if (prefixLength == localAddressPointer->prefixLen)
                {
                    if (FindCommonPrefix ((void *)prefix, (void *)&localAddressPointer->address, sizeof (IPV6_ADDR)) >= prefixLength)
                    {
                        return localAddressPointer;
                    }
                }
            }
            localAddressPointer = localAddressPointer->next;
        }
    }

    return NULL;
}

/*****************************************************************************
  Function:
	void * TCPIP_NDP_FindRemoteNode (NET_CONFIG * pNetIf, IPV6_ADDR * source, 
                                        uint8_t type)

  Summary:
	Finds an entry in the default router list, destination cache, or 
    neighbor cache by address.

  Description:
	Finds an entry in the default router list, destination cache, or 
    neighbor cache by address.	

  Precondition:
	None

  Parameters:
	pNetIf - Interface of the remote node.
    source - The neighbor or router address, or the destination address.
    type - IPV6_HEAP_NDP_DR_ID, IPV6_HEAP_NDP_DC_ID, IPV6_HEAP_NDP_NC_ID

  Returns:
  	void * - Pointer to a matching IPV6_HEAP_NDP_DR_ENTRY, 
                IPV6_HEAP_NDP_NC_ENTRY, or IPV6_HEAP_NDP_DC_ENTRY structure, 
                or NULL.
  	
  Remarks:
	None
  ***************************************************************************/
void * TCPIP_NDP_FindRemoteNode (NET_CONFIG * pNetIf, IPV6_ADDR * source, uint8_t type)
{
    void * nodePointer;
    int netIx;

    if (pNetIf == NULL)
        return NULL;

    netIx = _TCPIPStackNetIx (pNetIf);

    switch (type)
    {
        case IPV6_HEAP_NDP_DR_ID:
            nodePointer = ipv6Config[netIx].listDefaultRouter.head;
            while (nodePointer != NULL)
            {
                if (((IPV6_HEAP_NDP_DR_ENTRY *)nodePointer)->neighborInfo != NULL)
                    if (!memcmp ((void *) source, (void *)&(((IPV6_HEAP_NDP_DR_ENTRY *)nodePointer)->neighborInfo->remoteIPAddress), sizeof (IPV6_ADDR)))
                        return nodePointer;
                nodePointer = ((IPV6_HEAP_NDP_DR_ENTRY *)nodePointer)->next;
            }
            break;
        case IPV6_HEAP_NDP_NC_ID:
            nodePointer = ipv6Config[netIx].listNeighborCache.head;
            while (nodePointer != NULL)
            {
                if (!memcmp ((void *) source, (void *)&(((IPV6_HEAP_NDP_NC_ENTRY *)nodePointer)->remoteIPAddress), sizeof (IPV6_ADDR)))
                    return nodePointer;
                nodePointer = ((IPV6_HEAP_NDP_NC_ENTRY *)nodePointer)->next;
            }            
            break;
        case IPV6_HEAP_NDP_DC_ID:
            nodePointer = ipv6Config[netIx].listDestinationCache.head;
            while (nodePointer != NULL)
            {
                if (!memcmp ((void *) source, (void *)&(((IPV6_HEAP_NDP_DC_ENTRY *)nodePointer)->remoteIPAddress), sizeof (IPV6_ADDR)))
                    return nodePointer;
                nodePointer = ((IPV6_HEAP_NDP_DC_ENTRY *)nodePointer)->next;
            }
            break;
    }
    return NULL;
}

/*****************************************************************************
  Function:
    void TCPIP_NDP_SetReachability (NET_CONFIG * pNetIf, 
                                    IPV6_HEAP_NDP_NC_ENTRY * neighborPointer, 
                                    NEIGHBOR_UNREACHABILITY_DETECT_STATE newState)
	

  Summary:
	Sets the neighbor unreachability detection state of a neighbor.

  Description:
	Sets the neighbor unreachability detection state of a neighbor.  
    Initializes NUD time if necessary.	

  Precondition:
	None

  Parameters:
	pNetif - Interface of the neighbor.
    neighborPointer - The neighbor to update the state of.
    newState - The new NUD state.

  Returns:
    None.
  	
  Remarks:
	None
  ***************************************************************************/
void TCPIP_NDP_SetReachability (NET_CONFIG * pNetIf, IPV6_HEAP_NDP_NC_ENTRY * neighborPointer, NEIGHBOR_UNREACHABILITY_DETECT_STATE newState)
{
    neighborPointer->reachabilityState = newState;
    neighborPointer->staleStateTimeout = 0;
    if (newState == NDP_STATE_DELAY)
    {
        neighborPointer->unansweredProbes = 0;
        neighborPointer->nextNUDTime = SYS_TICK_Get() + (SYS_TICK_ResolutionGet() * DELAY_FIRST_PROBE_TIME);
    }
    else if (newState == NDP_STATE_REACHABLE)
    {
        neighborPointer->nextNUDTime = SYS_TICK_Get() + (ipv6Config[_TCPIPStackNetIx(pNetIf)].reachableTime * SYS_TICK_ResolutionGet());
        neighborPointer->flags.bResolvingAddress = false;
    }
    else if (newState == NDP_STATE_STALE)
    {
        neighborPointer->staleStateTimeout = SYS_TICK_Get() + (SYS_TICK_ResolutionGet() * IPV6_NEIGHBOR_CACHE_ENTRY_STALE_TIMEOUT);
    }
}

/*****************************************************************************
  Function:
    IPV6_HEAP_NDP_NC_ENTRY * TCPIP_NDP_GetNextHop (NET_CONFIG * pNetIf, 
                                                    IPV6_ADDR * address)

  Summary:
	Determines the next hop for a given address.

  Description:
	Determines the next hop neighbor for a given address.

  Precondition:
	None

  Parameters:
	pNetIf - The interface of the address.
    address - The destination address.

  Returns:
  	IPV6_HEAP_NDP_NC_ENTRY * - Pointer to the next-hop neighbor for the 
                                given address.
  	
  Remarks:
	None
  ***************************************************************************/
IPV6_HEAP_NDP_NC_ENTRY * TCPIP_NDP_GetNextHop (NET_CONFIG * pNetIf, IPV6_ADDR * address)
{
    IPV6_HEAP_NDP_NC_ENTRY * neighborPointer;
    IPV6_HEAP_NDP_DC_ENTRY * destinationPointer;
    IPV6_HEAP_NDP_DR_ENTRY * routerPointer;

    if (address->v[0] == 0xFF)
    {
        // Use on-link multicast
        return NULL;
    }

    if ((destinationPointer = (IPV6_HEAP_NDP_DC_ENTRY *)TCPIP_NDP_FindRemoteNode (pNetIf, address, IPV6_HEAP_NDP_DC_ID)) != NULL)
    {
        if (destinationPointer->nextHopNeighbor != NULL)
        {
            return destinationPointer->nextHopNeighbor;
        }
    }

    // Determine if the prefix is on-link
    if (!TCPIP_NDP_PrefixOnLinkStatus (pNetIf, address))
    {
        // Destination is probably off-link
        // Use a default router for now
        // If it is on-link, one of the routers
        // will fix it with a Redirect message
        if ((routerPointer = TCPIP_NDP_GetDefaultRouter(pNetIf)) != NULL)
        {
            if (routerPointer->neighborInfo != NULL)
            {
                TCPIP_NDP_DestinationCacheEntryCreate (pNetIf, address, ipv6Config[_TCPIPStackNetIx(pNetIf)].linkMTU, routerPointer->neighborInfo);
                return routerPointer->neighborInfo;
            }
        }
    }

    // Destination is either on-link or there are no
    // default routers available.  In either case,
    // treat the destination address as if it was on-link.
    // Use the destination address as the next-hop.

    if ((neighborPointer = (IPV6_HEAP_NDP_NC_ENTRY *)TCPIP_NDP_FindRemoteNode (pNetIf, address, IPV6_HEAP_NDP_NC_ID)) != NULL)
    {
        // This entry already has a neighbor cache entry
        TCPIP_NDP_DestinationCacheEntryCreate (pNetIf, address, ipv6Config[_TCPIPStackNetIx(pNetIf)].linkMTU, neighborPointer);
        return neighborPointer;
    }
    else
    {
        if ((neighborPointer = TCPIP_NDP_NeighborEntryCreate (pNetIf, address, NULL, NDP_STATE_NONE, 0, NULL)) != NULL)
        {
            TCPIP_NDP_DestinationCacheEntryCreate (pNetIf, address, ipv6Config[_TCPIPStackNetIx(pNetIf)].linkMTU, neighborPointer);
            return neighborPointer;
        }
        else
        {
            return NULL;
        }
    }
}


/*****************************************************************************
  Function:
    void TCPIP_NDP_SAA_PrefixInfoProcess (NET_CONFIG * pNetIf, 
                                            NDP_OPTION_PREFIX_INFO * prefixInfo)

  Summary:
	Processes a prefix information option from a router advertisement.

  Description:
	Processes a prefix information option from a router advertisement 
    according to the rules of stateless address autoconfiguration.

  Precondition:
	None

  Parameters:
	pNetIf - The interface the option was received from.
    prefixInfo - Pointer to the prefix information option.

  Returns:
  	None
  	
  Remarks:
	None
  ***************************************************************************/
void TCPIP_NDP_SAA_PrefixInfoProcess (NET_CONFIG * pNetIf, NDP_OPTION_PREFIX_INFO * prefixInfo)
{
    IPV6_ADDR_STRUCT * localAddressPointer;

    if ((prefixInfo->flags.bA) && !((prefixInfo->aPrefix.v[0] == 0xFE) && ((prefixInfo->aPrefix.v[1] & 0xC0) == 0x80)))
    {
        // Check to ensure that the prefix option preferred lifetime is less than the valid lifetime
        if (prefixInfo->dPreferredLifetime <= prefixInfo->dValidLifetime)
        {
            if ((localAddressPointer = TCPIP_NDP_FindPrefix (pNetIf, &prefixInfo->aPrefix, prefixInfo->vPrefixLen, false)) == NULL)
            {
                // Prefix is not equal to the prefix of an address configured by Stateless Address Autoconfiguration
                if (prefixInfo->dValidLifetime != 0)
                {
                    if (prefixInfo->vPrefixLen + IPV6_INTERFACE_ID_SIZE != 128)
                        return;
                    TCPIP_NDP_ConstructAddressFromPrefix (pNetIf, &tempAddress, &prefixInfo->aPrefix, prefixInfo->vPrefixLen);
                    localAddressPointer = TCPIP_IPV6_AddUnicastAddress (pNetIf, &tempAddress, false);
                    if (localAddressPointer != NULL)
                    {
                        localAddressPointer->prefixLen = prefixInfo->vPrefixLen;
                        localAddressPointer->preferredLifetime = prefixInfo->dPreferredLifetime;
                        localAddressPointer->validLifetime = prefixInfo->dValidLifetime;
                        localAddressPointer->lastTickTime = SYS_TICK_Get(); 
                    }
                }
            }
            else
            {
                // Prefix is equal to the prefix of an address configured by Stateless Address Autoconfiguration
                localAddressPointer->preferredLifetime = prefixInfo->dPreferredLifetime;
                if ((prefixInfo->dValidLifetime > NDP_VALID_LIFETIME_TWO_HOURS) ||
                    (prefixInfo->dValidLifetime > localAddressPointer->validLifetime))
                {
                    localAddressPointer->validLifetime = prefixInfo->dValidLifetime;
                }
                else if (localAddressPointer->validLifetime > NDP_VALID_LIFETIME_TWO_HOURS)
                {
                    localAddressPointer->validLifetime = NDP_VALID_LIFETIME_TWO_HOURS;                                
                }
                // If remaining lifetime is <= 2 hours, ignore the prefix info option
            }
        }
    }
}

/*****************************************************************************
  Function:
	void TCPIP_NDP_PrefixInfoProcessDetermineOnLinkStatus (
        NET_CONFIG * pNetIf, NDP_OPTION_PREFIX_INFO * prefixInfo)

  Summary:
	Processes a prefix information option from a router advertisement.

  Description:
	Processes a prefix information option from a router advertisement 
    according to the rules of on-link address determination.

  Precondition:
	None

  Parameters:
	pNetIf - The interface the option was received from.
    prefixInfo - Pointer to the prefix information option.

  Returns:
  	None
  	
  Remarks:
	None
  ***************************************************************************/
void TCPIP_NDP_PrefixInfoProcessDetermineOnLinkStatus (NET_CONFIG * pNetIf, NDP_OPTION_PREFIX_INFO * prefixInfo)
{
    IPV6_HEAP_NDP_PL_ENTRY * prefixPointer;

    if ((prefixInfo->flags.bL) && !((prefixInfo->aPrefix.v[0] == 0xFE) && ((prefixInfo->aPrefix.v[1] & 0xC0) == 0x80)))
    {
        if ((prefixPointer = TCPIP_NDP_FindPrefix (pNetIf, &prefixInfo->aPrefix, prefixInfo->vPrefixLen, true)) == NULL)
        {
            if (prefixInfo->dValidLifetime != 0)
            {
                // Store this prefix in the heap so it can be used to determine if
                // addresses are on-link
                prefixPointer = TCPIP_NDP_PrefixListEntryCreate (pNetIf, &prefixInfo->aPrefix, prefixInfo->vPrefixLen, prefixInfo->dValidLifetime);
            }
        }
        else
        {
            if (prefixInfo->dValidLifetime != 0)
            {
                prefixPointer->validLifetime = prefixInfo->dValidLifetime;                
            }
            else
            {
                TCPIP_NDP_LinkedListEntryRemove (pNetIf, prefixPointer, IPV6_HEAP_NDP_PL_ID);
            }            
        }
    }
}

/*****************************************************************************
  Function:
    uint8_t TCPIP_NDP_PrefixOnLinkStatus (NET_CONFIG * pNetIf, 
        IPV6_ADDR * address)	

  Summary:
	Determines if an address is on-link

  Description:
	This function determines if a given unicast address should be treated 
    as an on-link address.

  Precondition:
	None

  Parameters:
    pNetIf - Interface of the address.
    address - The address to check.

  Returns:
  	None
  	
  Remarks:
	None
  ***************************************************************************/
uint8_t TCPIP_NDP_PrefixOnLinkStatus (NET_CONFIG * pNetIf, IPV6_ADDR * address)
{
    IPV6_HEAP_NDP_PL_ENTRY * prefixPointer;
    
    if (pNetIf == NULL)
        return false;

    prefixPointer = (IPV6_HEAP_NDP_PL_ENTRY *)ipv6Config[_TCPIPStackNetIx(pNetIf)].listPrefixList.head;

    if ((address->v[0] == 0xFE) && ((address->v[1] & 0xC0) == 0x80))
        return true;

    while (prefixPointer != NULL)
    {
        if (FindCommonPrefix ((void *)address, (void *)&prefixPointer->prefix, sizeof (IPV6_ADDR)) >= prefixPointer->prefixLength)
        {
            return true;
        }
        prefixPointer = prefixPointer->next;
    }

    return false;
}

/*****************************************************************************
  Function:
    void TCPIP_NDP_ConstructAddressFromPrefix (NET_CONFIG * pNetIf, 
        IPV6_ADDR * destination, IPV6_ADDR * prefix, 
        unsigned char prefixLength)

  Summary:
	Constructs and IPv6 address from a given prefix.

  Description:
	This function will construct an address from a given 64-bit prefix.

  Precondition:
	The prefix length must be 64 bits.

  Parameters:
	pNetIf - Interface for the address.
    destination - Return value for the constructed address.
    prefix - Prefix.
    prefixLength - Length of the prefix (should be 8 bytes)

  Returns:
  	None
  	
  Remarks:
	None
  ***************************************************************************/
void TCPIP_NDP_ConstructAddressFromPrefix (NET_CONFIG * pNetIf, IPV6_ADDR * destination, IPV6_ADDR * prefix, unsigned char prefixLength)
{
    uint8_t offset = prefixLength >> 3;

    memcpy (destination, prefix, sizeof (IPV6_ADDR));

    // We'll never construct an address from a prefix
    // that isn't 64 bits (since our interface ID is also 64 bits).
    if ((prefixLength & 0b111) == 0)
    {
        destination->v[offset++] = pNetIf->MyMACAddr.v[0] | 0x02;
        destination->v[offset++] = pNetIf->MyMACAddr.v[1];
        destination->v[offset++] = pNetIf->MyMACAddr.v[2];
        destination->v[offset++] = 0xFF;
        destination->v[offset++] = 0xFE;
        memcpy (&destination->v[offset], &(pNetIf->MyMACAddr).v[3], 3);
    }
}

/*****************************************************************************
  Function:
	void TCPIP_NDP_NeighborCacheLinkLayerAddressUpdate (NET_CONFIG * pNetIf, 
        IPV6_HEAP_NDP_NC_ENTRY * neighborPointer, MAC_ADDR * linkLayerAddr, 
        uint8_t reachability)

  Summary:
    Updates the link-layer address of a neighbor cache entry.	

  Description:
	Updates the link-layer address of a neighbor cache entry.  If the entry 
    has queued packets, they will be transmitted if a valid link-layer 
    address is included.

  Precondition:
	None

  Parameters:
	pNetIf - The interface of the neighbor.
    neighborPointer - The neighbor to update.
    linkLayerAddr - The new link-layer address of the node.
    reachability - The NUD reachability state (NDP_STATE_NONE if this 
                    shouldn't be set)

  Returns:
  	None
  	
  Remarks:
	None
  ***************************************************************************/
void TCPIP_NDP_NeighborCacheLinkLayerAddressUpdate (NET_CONFIG * pNetIf, IPV6_HEAP_NDP_NC_ENTRY * neighborPointer, MAC_ADDR * linkLayerAddr, uint8_t reachability)
{
    memcpy (&neighborPointer->remoteMACAddr, linkLayerAddr, sizeof (MAC_ADDR));
    if (reachability != NDP_STATE_NONE)
    {
        TCPIP_NDP_SetReachability (pNetIf, neighborPointer, reachability);
    }

    switch (neighborPointer->reachabilityState)
    {
        case NDP_STATE_STALE:
            TCPIP_NDP_SetReachability (pNetIf, neighborPointer, NDP_STATE_DELAY);\
            // Fall through
        case NDP_STATE_REACHABLE:
        case NDP_STATE_DELAY:
        case NDP_STATE_PROBE:
            while (neighborPointer->queuedPacketPointer != NULL)
            {
                memcpy (&((IP_PACKET *)(neighborPointer->queuedPacketPointer))->remoteMACAddr, &neighborPointer->remoteMACAddr, sizeof(MAC_ADDR));
                if (TCPIP_IP_TransmitPacket (neighborPointer->queuedPacketPointer))
                {
                    TCPIP_IPV6_RemoveQueuedPacket (neighborPointer, neighborPointer->queuedPacketPointer);
                }
                else
                {
                    break;
                }
            }
            break;
        default:
            break;
    }
}

/*****************************************************************************
  Function:
	void TCPIP_NDP_NUD_Task (void)

  Summary:
	Neighbor Unreachability Detection task function.

  Description:
	Neighbor Unreachability Detection task function.

  Precondition:
	None

  Parameters:
	None

  Returns:
  	None
  	
  Remarks:
	None
  ***************************************************************************/
void TCPIP_NDP_NUD_Task (void)
{
    IPV6_HEAP_NDP_NC_ENTRY * neighborPointer;
    IPV6_HEAP_NDP_NC_ENTRY * lastNeighborPointer;
    int netIx;
    NET_CONFIG * pNetIf;
    uint32_t time = SYS_TICK_Get();
    NDP_OPTION_LLA sllaOption;

    for(netIx=0; netIx < nStackIfs; netIx++)
    {
        neighborPointer = (IPV6_HEAP_NDP_NC_ENTRY *)ipv6Config[netIx].listNeighborCache.head;
        pNetIf = (NET_CONFIG*)TCPIP_STACK_IxToNet (netIx);

        while (neighborPointer != NULL)
        {
            lastNeighborPointer = neighborPointer;
            switch (neighborPointer->reachabilityState)
            {
                case NDP_STATE_FAILED:
                    neighborPointer->flags.bResolvingAddress = false;
                    if (neighborPointer->queuedPacketPointer != NULL)
                    {
                        TCPIP_IP_FreePacket (neighborPointer->queuedPacketPointer);
                    }
                    neighborPointer = TCPIP_NDP_NeighborEntryDelete (pNetIf, neighborPointer);
                    break;
                case NDP_STATE_REACHABLE:
                    if ((long)(time - neighborPointer->nextNUDTime) > 0)
                    {
                        TCPIP_NDP_SetReachability (pNetIf, neighborPointer, NDP_STATE_STALE);
                        break;
                    }
                    if (neighborPointer->queuedPacketPointer != NULL)
                    {
                        if (TCPIP_IP_TransmitPacket (neighborPointer->queuedPacketPointer))
                            TCPIP_IPV6_RemoveQueuedPacket(neighborPointer, neighborPointer->queuedPacketPointer);
                    }
                    break;
                case NDP_STATE_STALE:
                    if (neighborPointer->queuedPacketPointer != NULL)
                    {
                        // Try to transmit the packet if we have a link-layer address for this node
                        if (TCPIP_IP_TransmitPacket (neighborPointer->queuedPacketPointer))
                            TCPIP_IPV6_RemoveQueuedPacket(neighborPointer, neighborPointer->queuedPacketPointer);
                        TCPIP_NDP_SetReachability (pNetIf, neighborPointer, NDP_STATE_DELAY);
                    }

                    if (neighborPointer->staleStateTimeout != 0)
                    {
                        // Check to see if the neighbor cache entry has timed out
                        if ((long)(time - neighborPointer->staleStateTimeout) > 0)
                        {
                            // Remove this node if it isn't being used as a router.
                            if (!neighborPointer->flags.bIsRouter)
                            {
                                neighborPointer = TCPIP_NDP_NeighborEntryDelete (pNetIf, neighborPointer);
                            }
                        }
                    }
                    break;
                case NDP_STATE_DELAY:
                    if ((long)(time - neighborPointer->nextNUDTime) > 0)
                    {
                        TCPIP_NDP_SetReachability (pNetIf, neighborPointer, NDP_STATE_PROBE);
                    }
                    else
                    {
                        break;
                    }
                    // Fall through if we've entered the probe state
                    // 
                case NDP_STATE_PROBE:
                    if ((long)(SYS_TICK_Get() - neighborPointer->nextNUDTime) > 0)
                    {
                        if (neighborPointer->unansweredProbes < MAX_UNICAST_SOLICIT)
                        {
                            IP_PACKET * pkt;
                            IPV6_ADDR_STRUCT * sourceAddress = NULL;

                            if (neighborPointer->preferredSource != NULL)
                            {
                                sourceAddress = TCPIP_IPV6_DAS_SelectSourceAddress (pNetIf, &neighborPointer->remoteIPAddress, &(neighborPointer->preferredSource->address));
                            }
                            if (sourceAddress == NULL)
                            {
                                sourceAddress = TCPIP_IPV6_DAS_SelectSourceAddress (pNetIf, &neighborPointer->remoteIPAddress, NULL);
                            }
                            if (sourceAddress == NULL)
                            {
                                neighborPointer->nextNUDTime = SYS_TICK_Get();
                                break;
                            }

                            sllaOption.vType = NDP_OPTION_TYPE_LLA_SOURCE;
                            sllaOption.vLength = 1;
                            memcpy (&sllaOption.mLinkLayerAddr, &pNetIf->MyMACAddr, sizeof (MAC_ADDR));
        

                            pkt = TCPIP_ICMPV6_PutHeaderNeighborSolicitation (pNetIf, &(sourceAddress->address), &(neighborPointer->remoteIPAddress), &(neighborPointer->remoteIPAddress));
                            if (pkt == NULL)
                            {
                                // If we can't allocate a transmit packet structure, 
                                // just reset the next probe timer to the current time.
                                // This will prevent rollover issues if we're unable to
                                // send a packet for a very very long amount of time.
                                neighborPointer->nextNUDTime = SYS_TICK_Get();
                            }
                            else
                            {
                                if (TCPIP_IP_IsTxPutReady(pkt, sizeof(NDP_OPTION_LLA)))
                                {
                                    TCPIP_IP_PutArray(pkt, (uint8_t*)&sllaOption, sizeof(NDP_OPTION_LLA));
                                    pkt->neighbor = neighborPointer;
                                    TCPIP_ICMPV6_Flush (pkt);
                                    neighborPointer->unansweredProbes++;
                                    neighborPointer->nextNUDTime = SYS_TICK_Get() + ((ipv6Config[_TCPIPStackNetIx(pNetIf)].retransmitTime * SYS_TICK_ResolutionGet()) / 1000);
                                }
                                else
                                {
                                    TCPIP_IP_FreePacket (pkt);
                                }
                            }
                        }
                        else
                        {
                            // The maximum number of probes has been sent.
                            // Remove this cache entry.  Subsequent traffic to the
                            // associated remote node will recreate it and re-perform
                            // address resolution if necessary.
                            neighborPointer = TCPIP_NDP_NeighborEntryDelete (pNetIf, neighborPointer);
                        }
                    }
                    break;
                case NDP_STATE_INCOMPLETE:
                    if ((long)(time - neighborPointer->nextNUDTime) > 0)
                    {
                        if (neighborPointer->unansweredProbes == MAX_MULTICAST_SOLICIT)
                        {
                            // The maximum number of probes has been sent.
                            // Remove this cache entry.  Subsequent traffic to the
                            // associated remote node will recreate it and re-perform
                            // address resolution if necessary.
                            neighborPointer = TCPIP_NDP_NeighborEntryDelete (pNetIf, neighborPointer);
                        }
                        else
                        {
                            IPV6_ADDR_STRUCT * sourceAddress = NULL;
                            IPV6_ADDR solicitedNodeMulticastAddr = {{0xFF, 0x02, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
                                                                        0x00, 0x00, 0x00, 0x01, 0xFF, 
                                                                        neighborPointer->remoteIPAddress.v[13],
                                                                        neighborPointer->remoteIPAddress.v[14], 
                                                                        neighborPointer->remoteIPAddress.v[15]}};
                            IP_PACKET * pkt;
        
                            if (neighborPointer->preferredSource != NULL)
                            {
                                sourceAddress = TCPIP_IPV6_DAS_SelectSourceAddress (pNetIf, &neighborPointer->remoteIPAddress, &neighborPointer->preferredSource->address);
                            }
                            if (sourceAddress == NULL)
                            {
                                sourceAddress = TCPIP_IPV6_DAS_SelectSourceAddress (pNetIf, &neighborPointer->remoteIPAddress, NULL);
                            }
                            if (sourceAddress == NULL)
                            {
                                TCPIP_NDP_SetReachability (pNetIf, neighborPointer, NDP_STATE_FAILED);
                                break;
                            }
                            else
                            {
                                neighborPointer->preferredSource = sourceAddress;
                            }    

                            sllaOption.vType = NDP_OPTION_TYPE_LLA_SOURCE;
                            sllaOption.vLength = 1;
                            memcpy (&sllaOption.mLinkLayerAddr, &pNetIf->MyMACAddr, sizeof (MAC_ADDR));
        
                            pkt = TCPIP_ICMPV6_PutHeaderNeighborSolicitation (pNetIf, &sourceAddress->address, &solicitedNodeMulticastAddr, &neighborPointer->remoteIPAddress);
                            if (pkt == NULL)
                            {
                                neighborPointer->nextNUDTime = SYS_TICK_Get();
                            }
                            else
                            {
                                if (TCPIP_IP_IsTxPutReady(pkt, sizeof(NDP_OPTION_LLA)))
                                {
                                    TCPIP_IP_PutArray(pkt, (uint8_t*)&sllaOption, sizeof(NDP_OPTION_LLA));
                                    TCPIP_ICMPV6_Flush (pkt);
                
                                    neighborPointer->unansweredProbes++;
                                    neighborPointer->nextNUDTime = SYS_TICK_Get() + ((ipv6Config[netIx].retransmitTime * SYS_TICK_ResolutionGet()) / 1000);
                                }
                                else
                                {
                                    TCPIP_IP_FreePacket (pkt);
                                }
                            }
                        }
                    }
                    break;
                default:
                    break;
            }

            if (neighborPointer == lastNeighborPointer)
                neighborPointer = neighborPointer->next;
        }
    }
}

/*****************************************************************************
  Function:
	void TCPIP_NDP_ResolveAddress (IPV6_HEAP_NDP_NC_ENTRY * entry)

  Summary:
	Perorms link-layer address resolution for a neighbor.

  Description:
	Begins link-layer address resolution for a given neighbor.

  Precondition:
	None

  Parameters:
	entry - The neighbor whose link-layer address must be resolved.

  Returns:
  	None
  	
  Remarks:
	None
  ***************************************************************************/
void TCPIP_NDP_ResolveAddress (IPV6_HEAP_NDP_NC_ENTRY * entry)
{
    if (entry->flags.bResolvingAddress)
        return;

    entry->unansweredProbes = 0;
    entry->nextNUDTime = SYS_TICK_Get();
    entry->flags.bResolvingAddress = true;
}

/*****************************************************************************
  Function:
    void TCPIP_NDP_Task (void)	

  Summary:
	Task function for the Neighbor Discovery Protocol.

  Description:
	This function calls each sub-task function used by the Neighbor Discovery 
    protocol.

  Precondition:
	None

  Parameters:
	None

  Returns:
  	None
  	
  Remarks:
	None
  ***************************************************************************/
void TCPIP_NDP_Task (void)
{
    TCPIP_NDP_RS_Task();
    TCPIP_NDP_DAD_Task();
    TCPIP_NDP_NUD_Task();
}

/*****************************************************************************
  Function:
    void TCPIP_NDP_DestinationCacheUpdate (NET_CONFIG * pNetIf, 
        IPV6_HEAP_NDP_NC_ENTRY * neighborEntry)

  Summary:
	Removes entries from the destination cache if their next-hop neighbor 
    becomes invalid.

  Description:
	Removes entries from the destination cache if their next-hop neighbor 
    becomes invalid.

  Precondition:
	None

  Parameters:
	pNetIf - Interface for the given neighbor.
    neighbor - The neighbor that will become invalid.

  Returns:
  	None
  	
  Remarks:
	None
  ***************************************************************************/
void TCPIP_NDP_DestinationCacheUpdate (NET_CONFIG * pNetIf, IPV6_HEAP_NDP_NC_ENTRY * neighborEntry)
{
    IPV6_HEAP_NDP_DC_ENTRY * destinationPointer;

    if (pNetIf == NULL)
        return;

    destinationPointer = (IPV6_HEAP_NDP_DC_ENTRY *)ipv6Config[_TCPIPStackNetIx(pNetIf)].listDestinationCache.head;

    while (destinationPointer != NULL)
    {
        if (destinationPointer->nextHopNeighbor == neighborEntry)
        {
            destinationPointer = TCPIP_NDP_LinkedListEntryRemove (pNetIf, destinationPointer, IPV6_HEAP_NDP_DC_ID);
        }
        else
        {
            destinationPointer = destinationPointer->next;
        }
    }
}

/*****************************************************************************
  Function:
	void * TCPIP_NDP_LinkedListEntryRemove (NET_CONFIG * pNetIf, 
        void * entry, uint8_t type)

  Summary:
	Removes an entry from an NDP linked list and frees the memory used for 
    that entry.

  Description:
	Removes an entry from an NDP linked list and frees the memory used for 
    that entry.

  Precondition:
	None

  Parameters:
	pNetIf - The interface for the entry that will be removed.
    entry - Pointer to the entry that should be removed.
    type -  IPV6_HEAP_NDP_DR_ID - Default router list
            IPV6_HEAP_NDP_DC_ID - Destination Cache
            IPV6_HEAP_NDP_NC_ID - Neighbor Cache
            IPV6_HEAP_NDP_PL_ID - Prefix List
            IPV6_HEAP_ADDR_UNICAST_ID - Unicast local address
            IPV6_HEAP_ADDR_MULTICAST_ID - Multicast listener
            IPV6_HEAP_ADDR_UNICAST_TENTATIVE_ID - Tentative local unicast address

  Returns:
  	void * - Pointer to the next node in the given list (or NULL if no more nodes)
  	
  Remarks:
	None
  ***************************************************************************/
void * TCPIP_NDP_LinkedListEntryRemove (NET_CONFIG * pNetIf, void * entry, uint8_t type)
{
    void * nextNode;
    int netIx;

    if ((entry == NULL) || (pNetIf == NULL))
        return NULL;

    netIx = _TCPIPStackNetIx (pNetIf);

    switch (type)
    {
        case IPV6_HEAP_NDP_DR_ID:
            TCPIP_NDP_DestinationCacheUpdate (pNetIf, ((IPV6_HEAP_NDP_DR_ENTRY *)entry)->neighborInfo);
            SingleListRemoveNode (&ipv6Config[netIx].listDefaultRouter, entry);
            nextNode = ((IPV6_HEAP_NDP_DR_ENTRY *)entry)->next;
            // Update the current default router if necessary
            if (entry == ipv6Config[netIx].currentDefaultRouter)
            {
                if (nextNode)
                    ipv6Config[netIx].currentDefaultRouter = nextNode;
                else
                    ipv6Config[netIx].currentDefaultRouter = (IPV6_HEAP_NDP_DR_ENTRY *)ipv6Config[netIx].listDefaultRouter.head;
            }
            break;
        case IPV6_HEAP_NDP_DC_ID:
            SingleListRemoveNode (&ipv6Config[netIx].listDestinationCache, entry);
            nextNode = ((IPV6_HEAP_NDP_DC_ENTRY *)entry)->next;
            break;
        case IPV6_HEAP_NDP_NC_ID:
            SingleListRemoveNode (&ipv6Config[netIx].listNeighborCache, entry);
            nextNode = ((IPV6_HEAP_NDP_NC_ENTRY *)entry)->next;
            break;
        case IPV6_HEAP_NDP_PL_ID:
            SingleListRemoveNode (&ipv6Config[netIx].listPrefixList, entry);
            nextNode = ((IPV6_HEAP_NDP_PL_ENTRY *)entry)->next;
            break;
        case IPV6_HEAP_ADDR_UNICAST_ID:
            DoubleListRemoveNode (&ipv6Config[netIx].listIpv6UnicastAddresses, entry);
            nextNode = ((IPV6_ADDR_STRUCT *)entry)->next;
            break;
        case IPV6_HEAP_ADDR_MULTICAST_ID:
            DoubleListRemoveNode (&ipv6Config[netIx].listIpv6MulticastAddresses, entry);
            nextNode = ((IPV6_ADDR_STRUCT *)entry)->next;
            break;
        case IPV6_HEAP_ADDR_UNICAST_TENTATIVE_ID:
            DoubleListRemoveNode (&ipv6Config[netIx].listIpv6TentativeAddresses, entry);
            nextNode = ((IPV6_ADDR_STRUCT *)entry)->next;
            break;

        default:
            nextNode = 0;
            break;
    }

    TCPIP_HEAP_Free (ndpMemH, entry);

    return nextNode;
}

/*****************************************************************************
  Function:
	void TCPIP_NDP_LinkedListEntryInsert (NET_CONFIG * pNetIf, void * entry, 
        uint8_t type)

  Summary:
	Inserts an entry into an NDP linked list

  Description:
	Inserts an entry into an NDP linked list

  Precondition:
	None

  Parameters:
	pNetIf - The interface for the entry that will be inserted.
    entry - Pointer to the entry that will be inserted.
    type -  IPV6_HEAP_NDP_DR_ID - Default router list
            IPV6_HEAP_NDP_DC_ID - Destination Cache
            IPV6_HEAP_NDP_NC_ID - Neighbor Cache
            IPV6_HEAP_NDP_PL_ID - Prefix List
            IPV6_HEAP_ADDR_UNICAST_ID - Unicast local address
            IPV6_HEAP_ADDR_MULTICAST_ID - Multicast listener
            IPV6_HEAP_ADDR_UNICAST_TENTATIVE_ID - Tentative local unicast address

  Returns:
  	void * - Pointer to the next node in the given list (or NULL if no more nodes)
  	
  Remarks:
	None
  ***************************************************************************/
void TCPIP_NDP_LinkedListEntryInsert (NET_CONFIG * pNetIf, void * entry, uint8_t type)
{
    int netIx;

    if ((entry == NULL) || (pNetIf == NULL))
        return;

    netIx = _TCPIPStackNetIx (pNetIf);

    switch (type)
    {
        case IPV6_HEAP_NDP_DR_ID:
            SingleListAddTail (&ipv6Config[netIx].listDefaultRouter, entry);
            break;
        case IPV6_HEAP_NDP_DC_ID:
            SingleListAddTail (&ipv6Config[netIx].listDestinationCache, entry);
            break;
        case IPV6_HEAP_NDP_NC_ID:
            SingleListAddTail (&ipv6Config[netIx].listNeighborCache, entry);
            break;
        case IPV6_HEAP_NDP_PL_ID:
            SingleListAddTail (&ipv6Config[netIx].listPrefixList, entry);
            break;
        case IPV6_HEAP_ADDR_UNICAST_ID:
            DoubleListAddTail (&ipv6Config[netIx].listIpv6UnicastAddresses, entry);
            break;
        case IPV6_HEAP_ADDR_MULTICAST_ID:
            DoubleListAddTail (&ipv6Config[netIx].listIpv6MulticastAddresses, entry);
            break;
        case IPV6_HEAP_ADDR_UNICAST_TENTATIVE_ID:
            DoubleListAddTail (&ipv6Config[netIx].listIpv6TentativeAddresses, entry);
            break;
    }
}

/*****************************************************************************
  Function:
	IPV6_ADDR_STRUCT * TCPIP_NDP_UnicastAddressMove (NET_CONFIG * pNetIf, 
        IPV6_ADDR_STRUCT * entryLocation, 
        IPV6_ADDR_STRUCT * previousEntryLocation)

  Summary:
    Inserts an IPv6 address (entryLocaion) into a linked list after the 
    address pointed to by previousEntryLocation	

  Description:
    Inserts an IPv6 address (entryLocaion) into a linked list after the 
    address pointed to by previousEntryLocation.  This function is used to 
    sort addresses for default address selection.	

  Precondition:
	None

  Parameters:
	pNetIf - Interface the given addresses are on.
    entryLocation - The address that will be inserted.
    previousEntryLocation - The address that will precede the newly inserted
        address

  Returns:
    IPV6_ADDR_STRUCT * - Pointer to the local address after the address that 
        was inserted.
  	
  Remarks:
	None
  ***************************************************************************/
IPV6_ADDR_STRUCT * TCPIP_NDP_UnicastAddressMove (NET_CONFIG * pNetIf, IPV6_ADDR_STRUCT * entryLocation, IPV6_ADDR_STRUCT * previousEntryLocation)
{
    IPV6_ADDR_STRUCT * returnVal;
    int netIx;

    if ((pNetIf == NULL) || (entryLocation == NULL))
        return NULL;

    netIx = _TCPIPStackNetIx (pNetIf);

    returnVal = entryLocation->next;

    if (previousEntryLocation != entryLocation->prev)
    {
        if (previousEntryLocation == NULL)
        {
            DoubleListRemoveNode (&ipv6Config[netIx].listIpv6UnicastAddresses, (DBL_LIST_NODE *)entryLocation);
            DoubleListAddHead (&ipv6Config[netIx].listIpv6UnicastAddresses, (DBL_LIST_NODE *)entryLocation);            
        }
        else if (ipv6Config[netIx].listIpv6UnicastAddresses.tail == (void *)previousEntryLocation)
        {
            DoubleListRemoveNode (&ipv6Config[netIx].listIpv6UnicastAddresses, (DBL_LIST_NODE *)entryLocation);
            DoubleListAddTail (&ipv6Config[netIx].listIpv6UnicastAddresses, (DBL_LIST_NODE *)entryLocation);
        }
        else
        {
            DoubleListRemoveNode (&ipv6Config[netIx].listIpv6UnicastAddresses, (DBL_LIST_NODE *)entryLocation);
            DoubleListAddMid (&ipv6Config[netIx].listIpv6UnicastAddresses, (DBL_LIST_NODE *)entryLocation, (DBL_LIST_NODE *)previousEntryLocation);
        }
    }

    return returnVal;
}

/*****************************************************************************
  Function:
	IPV6_ADDR_STRUCT * TCPIP_NDP_TentativeAddressPromote (
        NET_CONFIG * pNetIf, IPV6_ADDR_STRUCT * entryLocation)

  Summary:
	Promotes a tentative address to a usable local unicast address.

  Description:
	Promotes a tentative address to a usable local unicast address.	

  Precondition:
	None

  Parameters:
	pNetIf - The interface the address is on.
    entryLocation - The address that should be promoted.

  Returns:
  	IPV6_ADDR_STRUCT * - The tentative address after the address that is 
        promoted.
  	
  Remarks:
	None
  ***************************************************************************/
IPV6_ADDR_STRUCT * TCPIP_NDP_TentativeAddressPromote (NET_CONFIG * pNetIf, IPV6_ADDR_STRUCT * entryLocation)
{
    IPV6_ADDR_STRUCT * addressPointer;
    int netIx;

    if (pNetIf == NULL)
        return NULL;

    netIx = _TCPIPStackNetIx (pNetIf);

    if (entryLocation == NULL)
        return NULL;

    addressPointer = entryLocation->next;

    DoubleListRemoveNode (&ipv6Config[netIx].listIpv6TentativeAddresses, (DBL_LIST_NODE *)entryLocation);
    DoubleListAddTail (&ipv6Config[netIx].listIpv6UnicastAddresses, (DBL_LIST_NODE *)entryLocation);

    TCPIP_IPV6_NotifyClients(pNetIf, IPV6_EVENT_ADDRESS_ADDED);

    return addressPointer;
}

/*****************************************************************************
  Function:
	void TCPIP_NDP_DoubleListFree (void * list)

  Summary:
	Frees and deallocates all nodes from a doubly linked list.

  Description:
	Frees and deallocates all nodes from a doubly linked list.	

  Precondition:
	None

  Parameters:
	list - The list.

  Returns:
  	None
  	
  Remarks:
	None
  ***************************************************************************/
void TCPIP_NDP_DoubleListFree (void * list)
{
    DOUBLE_LIST * dList = (DOUBLE_LIST *) list;
    DBL_LIST_NODE * node;
    while ((node = DoubleListRemoveHead (dList)))
        TCPIP_HEAP_Free (ndpMemH, node);
}

/*****************************************************************************
  Function:
	void TCPIP_NDP_SingleListFree (void * list)

  Summary:
	Frees and deallocates all nodes from a singly linked list.

  Description:
	Frees and deallocates all nodes from a singly linked list.	

  Precondition:
	None

  Parameters:
	list - The list.

  Returns:
  	None
  	
  Remarks:
	None
  ***************************************************************************/
void TCPIP_NDP_SingleListFree (void * list)
{
    SINGLE_LIST * sList = (SINGLE_LIST *) list;
    SGL_LIST_NODE * node;
    while ((node = SingleListRemoveHead (sList)))
        TCPIP_HEAP_Free (ndpMemH, node);
}

#endif
