/*******************************************************************************
  IP private manager API for Microchip TCP/IP Stack

  Summary:
    
  Description:
*******************************************************************************/

/*******************************************************************************
FileName:  ip_manager.h 
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

#ifndef _IP_MANAGER_H_
#define _IP_MANAGER_H_


// Stack structures

// IP Pseudo header as defined by RFC 793 (needed for TCP and UDP 
// checksum calculations/verification)
typedef struct _PSEUDO_HEADER
{
    IP_ADDR SourceAddress;
    IP_ADDR DestAddress;
    uint8_t Zero;
    uint8_t Protocol;
    uint16_t Length;
} PSEUDO_HEADER;

#if defined (TCPIP_STACK_USE_IPV6)
// IPv6 Pseudo header (needed for TCP and UDP checksum calculations/verification)
typedef struct
{
    IPV6_ADDR SourceAddress;
    IPV6_ADDR DestAddress;
    unsigned long PacketLength;
    unsigned short zero1;
    unsigned char zero2;
    unsigned char NextHeader;
} IPV6_PSEUDO_HEADER;

typedef struct
{
    IPV6_HEAP_NDP_DR_ENTRY * currentDefaultRouter;
    DOUBLE_LIST listIpv6UnicastAddresses;         // IPV6_ADDR_STRUCT list
    DOUBLE_LIST listIpv6MulticastAddresses;       // IPV6_ADDR_STRUCT list
    DOUBLE_LIST listIpv6TentativeAddresses;       // IPV6_ADDR_STRUCT list
    uint32_t baseReachableTime;
    uint32_t reachableTime;
    uint32_t retransmitTime;
    uint32_t mtuIncreaseTimer;
    uint16_t linkMTU;
    uint16_t multicastMTU;
    uint8_t curHopLimit;
    SINGLE_LIST listNeighborCache;                // IPV6_HEAP_NDP_NC_ENTRY list
    SINGLE_LIST listDefaultRouter;                // IPV6_HEAP_NDP_DR_ENTRY list
    SINGLE_LIST listDestinationCache;             // IPV6_HEAP_NDP_DC_ENTRY list
    SINGLE_LIST listPrefixList;                   // IPV6_HEAP_NDP_PL_ENTRY list
    SINGLE_LIST fragments;                        // IPV6_FRAGMENT_BUFFER list
    uint8_t initState;
    uint8_t policyPreferTempOrPublic;
} IPV6_INTERFACE_CONFIG;

#endif  // defined (TCPIP_STACK_USE_IPV6)


// stack private API
// 
   
bool TCPIP_IP_TaskPending (void);

void TCPIP_IP_Task (void);

bool TCPIP_IPV6_Initialize(const TCPIP_STACK_MODULE_CTRL* const pStackInit, const IPV6_MODULE_GONFIG* const pIpv6Init);
void TCPIP_IPV6_Deinitialize(const TCPIP_STACK_MODULE_CTRL* const stackInit);
void TCPIP_IPV6_InitializeTask (void);
void TCPIP_IPV6_InitTmo (SYS_TICK curSysTick);
bool TCPIP_IPV6_InitTaskPending (void);
void TCPIP_IPV6_InitializeStop (NET_CONFIG * pNetIf);

void TCPIP_IPV6_Process (NET_CONFIG * pNetIf, MAC_ADDR * remoteMAC);


// misc IP functions
// 

#define TCPIP_IPV4_SwapPseudoHeader(h)  (h.Length = swaps(h.Length))

#define TCPIP_IP_IsTxReady(pNet)       MACIsTxReady(_TCPIPStackNetToMac(pNet))
#define TCPIP_IP_SetTxBuffer(pNet, b) MACSetWritePtr(_TCPIPStackNetToMac(pNet), b + MACGetTxBaseAddr(_TCPIPStackNetToMac(pNet)) + sizeof(ETHER_HEADER) + sizeof(IP_HEADER))
bool TCPIP_IP_GetHeader(NET_CONFIG* pNet, IP_ADDR *localIP, NODE_INFO *remote, uint8_t *protocol, uint16_t *len);
#define TCPIP_IP_DiscardRx(a)         MACDiscardRx(a)

#define TCPIP_IP_GetReadPtrInRx(n)     MACGetReadPtrInRx(((NET_CONFIG *) n)->hIfMac)
#define TCPIP_IP_SetReadPtr(n,o)       MACSetReadPtr(((NET_CONFIG *) n)->hIfMac,o)

#define TCPIP_IP_CalcRxChecksum(a,b,c) MACCalcRxChecksum(a,b,c)

bool TCPIP_IP_Initialize(const TCPIP_STACK_MODULE_CTRL* const stackInit, const IP_MODULE_GONFIG* pIpInit);
void TCPIP_IP_DeInitialize(const TCPIP_STACK_MODULE_CTRL* const stackInit);

void TCPIP_IPV4_SetRxBuffer(NET_CONFIG* pNet, uint16_t Offset);

bool TCPIP_IP_CopyTxPacketStruct (IP_PACKET * destination, IP_PACKET * source);

void TCPIP_IP_SetPacketIPProtocol (IP_PACKET * pkt, IP_ADDRESS_TYPE addressType);

// Adds the upper layer header (TCP/UDP/ICMP/ICMPv6) to the packet tx structure
IP_DATA_SEGMENT_HEADER * TCPIP_IP_PutUpperLayerHeader (IP_PACKET * pkt, void * header, unsigned short len, unsigned char type, unsigned short checksumOffset);

// Populate the IP_PACKET ptrIPHeader and upperLayerHeaderType fields.
// This is a change from the original TCPIP_IP_PutHeader function; the old function
// would also write the Ethernet and IP header to the MAC TX buffer, including
// the link-layer address.  Now the link-layer address is passed in in the TCPIP_IP_Flush
// function.  The length of remoteIPAddr is 4 or 16, depending on pkt's IP protocol
// type.
void TCPIP_IP_PutHeader(IP_PACKET * pkt, uint8_t protocol);

// Interface functions
void  TCPIP_IPV4_SetDestAddress(IP_PACKET * p, uint32_t addrValue);

IP_ADDR  TCPIP_IPV4_GetDestAddress(IP_PACKET * p);

void  TCPIP_IPV4_SetSourceAddress(IP_PACKET * p, uint32_t addrValue);

IP_ADDR  TCPIP_IPV4_GetSourceAddress(IP_PACKET * p);

// Gets the payload length of a TX packet
unsigned short TCPIP_IP_GetPayloadLength (IP_PACKET * pkt);

// Returns the pointer to the upper-layer header
void * TCPIP_IP_GetUpperLayerHeaderPtr(IP_PACKET * pkt);

#define TCPIP_IP_GetUpperLayerHeaderLen(s) (s->upperLayerHeaderLen)

// Calcualtes the 1's complement checksum over the packet payload
unsigned short TCPIP_IP_CalculatePayloadChecksum (IP_PACKET * pkt);

unsigned short TCPIP_IP_PutArrayHelper (IP_PACKET * pkt, const void * dataSource, uint8_t dataType, unsigned short len);
#define TCPIP_IP_PutRxData(mac,pkt,len)        TCPIP_IP_PutArrayHelper(pkt, &mac, IP_DATA_NETWORK_FIFO, len)

// Resets the IP_PACKET state associated with a socket after transmitting a packet so that it
// can be used to transmit additional packets.
void TCPIP_IP_ResetTransmitPacketState (IP_PACKET * pkt);

// Transmits a packet section
bool TCPIP_IP_TransmitPacket (IP_PACKET * pkt);

#define TCPIP_IP_IsPacketQueued(p)       (((IP_PACKET *)p)->flags.queued)

#if defined (TCPIP_STACK_USE_IPV6)

IPV6_ADDR *  TCPIP_IPV6_GetDestAddress(IP_PACKET * p);

void  TCPIP_IPV6_SetDestAddress(IP_PACKET * p, IPV6_ADDR * addr);

void  TCPIP_IPV6_SetSourceAddress(IP_PACKET * p, IPV6_ADDR * addr);

IPV6_ADDR *  TCPIP_IPV6_GetSourceAddress(IP_PACKET * p);

void TCPIP_IPV6_SetHopLimit(IP_PACKET * ptrPacket, uint8_t hopLimit);

void TCPIP_IPV6_SetRxBuffer(NET_CONFIG* pNet, uint16_t Offset);

#define TCPIP_IPV6_GetHash(d,a,b)  ((((IPV6_ADDR *)d)->w[0] + ((IPV6_ADDR *)d)->w[1] + ((IPV6_ADDR *)d)->w[2] + ((IPV6_ADDR *)d)->w[3] + ((IPV6_ADDR *)d)->w[4] + ((IPV6_ADDR *)d)->w[5] + ((IPV6_ADDR *)d)->w[6] + ((IPV6_ADDR *)d)->w[7] + a) ^ b)

bool TCPIP_IPV6_GetHeader(NET_CONFIG * pNetIf, IPV6_ADDR * localIPAddr, IPV6_ADDR * remoteIPAddr, uint8_t *protocol, uint16_t *len, uint8_t * hopLimit);

IPV6_ADDR_STRUCT *  TCPIP_IPV6_FindAddress(NET_CONFIG * pNetIf, IPV6_ADDR * addr, unsigned char listType);
bool                TCPIP_IPV6_AddressIsSolicitedNodeMulticast (IPV6_ADDR * address);

void TCPIP_IPV6_RemoveQueuedPacket (IPV6_HEAP_NDP_NC_ENTRY * neighborPointer, IP_PACKET * pkt);

IPV6_ADDRESS_TYPE TCPIP_IPV6_GetAddressType (NET_CONFIG * pNetIf, IPV6_ADDR * address);

void TCPIP_IPV6_SendError (NET_CONFIG * pNetIf, IPV6_ADDR * localIP, IPV6_ADDR * remoteIP, uint8_t code, uint8_t type, uint32_t additionalData, uint16_t packetLen);

void TCPIP_IP_TmoHandler(SYS_TICK curSysTick);

void TCPIP_IPV6_NotifyClients(NET_CONFIG* pNetIf, IPV6_EVENT_TYPE evType);

#endif

#endif // _IP_MANAGER_H_



