/*******************************************************************************
FileName:  ndp_manager.h 
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
#ifndef _NDP_MANAGER_H
#define _NDP_MANAGER_H

//************
// Stack types
//************
typedef enum
{
    NDP_STATE_FAILED = 0,
    NDP_STATE_NONE,
    NDP_STATE_INCOMPLETE,
    NDP_STATE_REACHABLE,
    NDP_STATE_STALE,
    NDP_STATE_DELAY,
    NDP_STATE_PROBE
} NEIGHBOR_UNREACHABILITY_DETECT_STATE;

typedef enum
{
    NDP_OPTION_TYPE_LLA_SOURCE =      1,
    NDP_OPTION_TYPE_LLA_TARGET =      2,
    NDP_OPTION_TYPE_PREFIX_INFO =     3,
    NDP_OPTION_TYPE_REDIRECT =        4,
    NDP_OPTION_TYPE_MTU =             5
} NDP_OPTION_TYPE;

typedef enum
{
    IPV6_NDP_DAD_NS_RECEIVED = 0,
    IPV6_NDP_DAD_NA_RECEIVED = 1
} IPV6_NDP_DAD_TYPE_RECEIVED;

typedef struct
{
    uint8_t vType;
    uint8_t vLength;
    MAC_ADDR mLinkLayerAddr;
} NDP_OPTION_LLA;

typedef struct __attribute__((__packed__))
{
    uint8_t vType;
    uint8_t vLength;
    uint8_t vPrefixLen;
    struct __attribute__((__packed__))
    {
        unsigned bReserved1     :6;
        unsigned bA             :1;
        unsigned bL             :1;
    } flags;
    uint32_t dValidLifetime;
    uint32_t dPreferredLifetime;
    uint32_t dReserved2;
    IPV6_ADDR aPrefix;
} NDP_OPTION_PREFIX_INFO;

typedef struct
{
    uint8_t vType;
    uint8_t vLength;
    uint16_t wReserved1;
    uint32_t dReserved2;
} NDP_OPTION_REDIRECTED;

typedef struct
{
    uint8_t vType;
    uint8_t vLength;
    uint16_t wReserved;
    uint32_t dMTU;
} NDP_OPTION_MTU;

typedef struct _IPV6_HEAP_NDP_NC_ENTRY
{
    struct _IPV6_HEAP_NDP_NC_ENTRY * next;
    IPV6_ADDR remoteIPAddress;
    MAC_ADDR remoteMACAddr;
    unsigned char reachabilityState;
    unsigned char unansweredProbes;
    unsigned long nextNUDTime;
    void * queuedPacketPointer;
    SYS_TICK staleStateTimeout;
    struct
    {
        unsigned bIsRouter     :1;
        unsigned bResolvingAddress : 1;
        unsigned Reserved        :6;
    } flags;
    IPV6_ADDR_STRUCT * preferredSource;
} IPV6_HEAP_NDP_NC_ENTRY;

typedef struct _IPV6_HEAP_NDP_DR_ENTRY
{
    struct _IPV6_HEAP_NDP_DR_ENTRY * next;
    IPV6_HEAP_NDP_NC_ENTRY * neighborInfo;
    unsigned long invalidationTimer;
    uint32_t tickTimer;
} IPV6_HEAP_NDP_DR_ENTRY;

typedef struct _IPV6_HEAP_NDP_DC_ENTRY
{
    struct _IPV6_HEAP_NDP_DC_ENTRY * next;
    IPV6_ADDR remoteIPAddress;
    unsigned long pathMTUIncreaseTimer;
    unsigned short pathMTU;
    IPV6_HEAP_NDP_NC_ENTRY * nextHopNeighbor;
} IPV6_HEAP_NDP_DC_ENTRY;

typedef struct _IPV6_HEAP_NDP_PL_ENTRY
{
    struct _IPV6_HEAP_NDP_PL_ENTRY * next;
    IPV6_ADDR prefix;
    unsigned long validLifetime;
    unsigned long lastTickTime;
    unsigned char prefixLength;
} IPV6_HEAP_NDP_PL_ENTRY;

typedef enum
{
    IPV6_HEAP_NDP_DR_ID = 0,
    IPV6_HEAP_NDP_NC_ID,
    IPV6_HEAP_NDP_DC_ID,
    IPV6_HEAP_NDP_PL_ID,
    IPV6_HEAP_NDP_AF_ID,
    IPV6_HEAP_ADDR_UNICAST_ID,
    IPV6_HEAP_ADDR_MULTICAST_ID,
    IPV6_HEAP_ADDR_UNICAST_TENTATIVE_ID,    
} IPV6_HEAP_ENTRY_TYPE;

//************
// Stack APIs
//************

bool TCPIP_NDP_Initialize (const TCPIP_STACK_MODULE_CTRL* const stackInit,
                    const void* const ndpData);
void TCPIP_NDP_Deinitialize(const TCPIP_STACK_MODULE_CTRL* const stackInit);

IPV6_HEAP_NDP_NC_ENTRY * TCPIP_NDP_NeighborEntryCreate (NET_CONFIG * pNetIf, IPV6_ADDR * remoteIPAddr, MAC_ADDR * remoteMACAddr, unsigned char initialState, unsigned char routerFlag, IPV6_ADDR_STRUCT * preferredSource);
IPV6_HEAP_NDP_DR_ENTRY * TCPIP_NDP_DefaultRouterEntryCreate (NET_CONFIG * pNetIf, IPV6_HEAP_NDP_NC_ENTRY * neighbor, unsigned long invalidationTime);
IPV6_HEAP_NDP_DC_ENTRY * TCPIP_NDP_DestinationCacheEntryCreate (NET_CONFIG * pNetIf, IPV6_ADDR * remoteIPAddress, unsigned long linkMTU, IPV6_HEAP_NDP_NC_ENTRY * neighbor);
IPV6_HEAP_NDP_PL_ENTRY * TCPIP_NDP_PrefixListEntryCreate (NET_CONFIG * pNetIf, IPV6_ADDR * prefix, unsigned char prefixLength, unsigned long validLifetime);

IPV6_HEAP_NDP_NC_ENTRY * TCPIP_NDP_NeighborEntryDelete (NET_CONFIG * pNetIf, IPV6_HEAP_NDP_NC_ENTRY * entry);

void * TCPIP_NDP_FindRemoteNode (NET_CONFIG * pNetIf, IPV6_ADDR * source, uint8_t type);

char TCPIP_NDP_DAD_DetectDuplicateAddress (NET_CONFIG * pNetIf, IPV6_ADDR_STRUCT * localAddressPointer);
void TCPIP_NDP_DAD_EventProcess (IPV6_ADDR_STRUCT * localAddressPointer, uint8_t type);

void TCPIP_NDP_RS_Start (NET_CONFIG * pNetIf);
void TCPIP_NDP_RS_Stop (NET_CONFIG * pNetIf);

void TCPIP_NDP_ResolveAddress (IPV6_HEAP_NDP_NC_ENTRY * entry);
IPV6_HEAP_NDP_NC_ENTRY * TCPIP_NDP_GetNextHop (NET_CONFIG * pNetIf, IPV6_ADDR * address);
void TCPIP_NDP_SetReachability (NET_CONFIG * pNetIf, IPV6_HEAP_NDP_NC_ENTRY * neighborPointer, NEIGHBOR_UNREACHABILITY_DETECT_STATE newState);

void TCPIP_NDP_PrefixInfoProcessDetermineOnLinkStatus (NET_CONFIG * pNetIf, NDP_OPTION_PREFIX_INFO * prefixInfo);
void TCPIP_NDP_SAA_PrefixInfoProcess (NET_CONFIG * pNetIf, NDP_OPTION_PREFIX_INFO * prefixInfo);
uint8_t TCPIP_NDP_PrefixOnLinkStatus (NET_CONFIG * pNetIf, IPV6_ADDR * address);

void TCPIP_NDP_Task (void);

void TCPIP_NDP_LinkedListEntryInsert (NET_CONFIG * pNetIf, void * entry, uint8_t type);
void * TCPIP_NDP_LinkedListEntryRemove (NET_CONFIG * pNetIf, void * entry, uint8_t type);

void TCPIP_NDP_SingleListFree (void * list);
void TCPIP_NDP_DoubleListFree (void * list);

IPV6_ADDR_STRUCT * TCPIP_NDP_UnicastAddressMove (NET_CONFIG * pNetIf, IPV6_ADDR_STRUCT * entryLocation, IPV6_ADDR_STRUCT * previousEntryLocation);

void TCPIP_NDP_NeighborCacheLinkLayerAddressUpdate (NET_CONFIG * pNetIf, IPV6_HEAP_NDP_NC_ENTRY * neighborPointer, MAC_ADDR * linkLayerAddr, uint8_t reachability);

#endif // _NDP_MANAGER_H

