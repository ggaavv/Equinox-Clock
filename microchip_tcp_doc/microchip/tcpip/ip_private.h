/*******************************************************************************
  IP private API for Microchip TCP/IP Stack

  Summary:
    
  Description:
*******************************************************************************/

/*******************************************************************************
FileName:  ip_private.h 
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

#ifndef _IP_PRIVATE_H_
#define _IP_PRIVATE_H_


// misc IP functions
// 

#define TCPIP_IPV4_SwapPseudoHeader(h)  (h.Length = swaps(h.Length))


bool TCPIP_IP_GetHeader(NET_CONFIG* pNet, IP_ADDR *localIP, NODE_INFO *remote, uint8_t *protocol, uint16_t *len);

// Allocate a data segment header
IP_DATA_SEGMENT_HEADER * TCPIP_IP_AllocateDataSegmentHeader (uint16_t len);

// Gets the IP-layer pseudo-header checksum
unsigned short TCPIP_IP_GetPseudoHeaderChecksum (IP_PACKET * pkt);

void TCPIP_IP_InsertIPPacketSegment (IP_DATA_SEGMENT_HEADER * ptrSegment, IP_PACKET * ptrPacket, IP_SEGMENT_TYPE);
IP_DATA_SEGMENT_HEADER * TCPIP_IP_GetDataSegmentByType (IP_PACKET * ptrPacket, IP_SEGMENT_TYPE type);
void * TCPIP_IP_GetDataSegmentContentsByType (IP_PACKET * ptrPacket, IP_SEGMENT_TYPE type);

// Flush the chain of data segments to the MAC TX buffer.
void TCPIP_IP_FlushDataSegments (TCPIP_MAC_HANDLE hMac, IP_PACKET * pkt);

// Deallocate all of the data segment headers and dynamically allocated payload segments in an IP_PACKET payload
void TCPIP_IP_FreePacketData (IP_PACKET * ptrPacket);
// Free a fragmented packet's reassembly buffer
void TCPIP_IPV6_FreeFragmentBuffer (IPV6_FRAGMENT_BUFFER * ptrFragment);

#if defined (TCPIP_STACK_USE_IPV6)    

    // IPV6 event registration
    
    typedef struct  _TAG_IPV6_LIST_NODE
    {
    	struct _TAG_IPV6_LIST_NODE*		next;		// next node in list
                                                    // makes it valid SGL_LIST_NODE node
        IPV6_EVENT_HANDLER              handler;    // handler to be called for event
        const void*                     hParam;     // handler parameter
        TCPIP_NET_HANDLE                hNet;       // interface that's registered for
                                                    // 0 if all    
    }IPV6_LIST_NODE;

    #define TCPIP_IPV6_GetOptionHeader(h,data, count)   MACGetArray (h,(unsigned char *)data, count << 3)
    uint8_t TCPIP_IPV6_ProcessHopByHopOptionsHeader (NET_CONFIG * pNetIf, uint8_t * nextHeader, uint16_t * length);
    uint8_t TCPIP_IPV6_ProcessDestinationOptionsHeader (NET_CONFIG * pNetIf, uint8_t * nextHeader, uint16_t * length);
    uint8_t TCPIP_IPV6_ProcessRoutingHeader (NET_CONFIG * pNetIf, uint8_t * nextHeader, uint16_t * length);
    uint8_t TCPIP_IPV6_ProcessFragmentationHeader (NET_CONFIG * pNetIf, IPV6_ADDR * source, IPV6_ADDR * dest, uint8_t * nextHeader, uint16_t dataCount, uint16_t headerLen, MAC_ADDR * remoteMACAddr, uint16_t previousHeader);
        
    IPV6_ADDR_STRUCT *  TCPIP_IPV6_FindSolicitedNodeMulticastAddress(NET_CONFIG * pNetIf, IPV6_ADDR * addr, unsigned char listType);

    void TCPIP_IPV6_QueuePacket (IPV6_HEAP_NDP_NC_ENTRY * neighborPtr, IP_PACKET * pkt);
        
    #define IPV6_ADDR_POLICY_TABLE_LEN      (sizeof (gPolicyTable) / sizeof (IPV6_ADDRESS_POLICY))
    unsigned char TCPIP_IPV6_DAS_GetPolicy (const IPV6_ADDR * addr, unsigned char * label, unsigned char * precedence, unsigned char * prefixLen);

    unsigned short TCPIP_IP_PutArrayHelper (IP_PACKET * pkt, const void * dataSource, uint8_t dataType, unsigned short len);
    #define TCPIP_IP_PutRxData(mac,pkt,len)        TCPIP_IP_PutArrayHelper(pkt, &mac, IP_DATA_NETWORK_FIFO, len)

    void TCPIP_IPV6_FragmentTask (void);
    void TCPIP_IPV6_UpdateTimestampsTask (void);

    bool TCPIP_IP_TransmitPacketInFragments (IP_PACKET * pkt, uint16_t mtu);

#endif

#endif // _IP_PRIVATE_H_



