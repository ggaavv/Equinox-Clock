/*******************************************************************************
  IP Defs for Microchip TCP/IP Stack

  Summary:
    
  Description:
*******************************************************************************/

/*******************************************************************************
FileName:  IP.h 
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

#ifndef __IP_H_
#define __IP_H_


// public definitions
// 

#include "link_list.h"

// Defines a list of next header types
typedef enum
{
    IP_PROT_HOP_BY_HOP_OPTIONS_HEADER =                      (0u),        // IPv6 Hop-by-Hop Opt. Header
    IP_PROT_ICMP =                                           (1u),
    IP_PROT_TCP =                                            (6u),
    IP_PROT_UDP =                                            (17u),
    IP_PROT_IPV6 =                                           (41u),       // IPv6 Protocol
    IP_PROT_ROUTING_HEADER =                                 (43u),       // IPv6 Routing Header
    IP_PROT_FRAGMENTATION_HEADER =                           (44u),       // IPv6 Fragmentation Header
    IP_PROT_ENCAPSULATING_SECURITY_PAYLOAD_HEADER =          (50u),       // Encapsulating Security Payload Header
    IP_PROT_AUTHENTICATION_HEADER =                          (51u),       // Authentication Header
    IP_PROT_ICMPV6 =                                         (58u),       // ICMPv6 Protocol
    IP_PROT_NONE =                                           (59u),       // No next header
    IP_PROT_DESTINATION_OPTIONS_HEADER =                     (60u)       // Destination Options Header
} IP_NEXT_HEADER_TYPE;

#define IP_VERSION_6    (1u)
#define IP_VERSION_4    (0u)

typedef enum
{
    IPV6_ACTION_NONE = 0,
    IPV6_ACTION_DISCARD_SILENT,
    IPV6_ACTION_DISCARD_PP_0,
    IPV6_ACTION_DISCARD_PP_2,
    IPV6_ACTION_DISCARD_PP_2_NOT_MC,
    IPV6_ACTION_BEGIN_EX_HEADER_PROCESSING
} IPV6_ACTION;

// IPv6 Type-length-value type code for the Pad 1 option
#define IPV6_TLV_PAD_1                          0u
// IPv6 Type-length-value type code for the Pad N option
#define IPV6_TLV_PAD_N                          1u
// IPv6 Type-length-value type code for the Hop-by-hop "Jumbogram Payload" option
#define IPV6_TLV_HBHO_PAYLOAD_JUMBOGRAM         0xC2u
// IPv6 Type-length-value type code for the Hop-by-hop "Router Alert" option
#define IPV6_TLV_HBHO_ROUTER_ALERT              0x05u

// IPv6 action code for the unrecognized option reaction to skip the option
#define IPV6_TLV_UNREC_OPT_SKIP_OPTION          0b00
// IPv6 action code for the unrecognized option reaction to discard the packet silently
#define IPV6_TLV_UNREC_OPT_DISCARD_SILENT       0b01
// IPv6 action code for the unrecognized option reaction to discard the packet and send 
// an ICMP parameter problem message
#define IPV6_TLV_UNREC_OPT_DISCARD_PP           0b10
// IPv6 action code for the unrecognized option reaction to discard the packet and send 
// an ICMP parameter problem message is the destination addr isn't a multicast address
#define IPV6_TLV_UNREC_OPT_DISCARD_PP_NOT_MC    0b11

#define IPV6_HEADER_OFFSET_PAYLOAD_LENGTH       (0x04u)
#define IPV6_HEADER_OFFSET_NEXT_HEADER          (0x06u)
#define IPV6_HEADER_OFFSET_SOURCE_ADDR          (0x08u)
#define IPV6_HEADER_OFFSET_DEST_ADDR            (0x08u + sizeof (IPV6_ADDR))

typedef enum
{
    IPV6_PREFER_PUBLIC_ADDRESSES = 0,
    IPV6_PREFER_TEMPORARY_ADDRESSES
} IPV6_ADDRESS_PREFERENCE;

typedef enum
{
    TYPE_IP_HEADER = 1u,
    TYPE_EX_HEADER_HOP_BY_HOP_OPTIONS,
    TYPE_EX_HEADER_DESTINATION_OPTIONS_1,
    TYPE_EX_HEADER_ROUTING,
    TYPE_EX_HEADER_FRAGMENT,
    TYPE_EX_HEADER_AUTHENTICATION_HEADER,
    TYPE_EX_HEADER_ENCAPSULATING_SECURITY_PAYLOAD,
    TYPE_EX_HEADER_DESTINATION_OPTIONS_2,
    TYPE_UPPER_LAYER_HEADER,
    TYPE_UPPER_LAYER_PAYLOAD,
    TYPE_BEGINNING_OF_WRITABLE_PART,
    TYPE_END_OF_LIST
} IP_SEGMENT_TYPE;

extern const IPV6_ADDR IPV6_FIXED_ADDR_UNSPECIFIED;
extern const IPV6_ADDR IPV6_FIXED_ADDR_ALL_NODES_MULTICAST;
extern const IPV6_ADDR IPV6_FIXED_ADDR_ALL_ROUTER_MULTICAST;

typedef union
{
    unsigned char b;
    struct
    {
        unsigned option             :6;
        unsigned unrecognizedAction :2;
    } bits;
} IPV6_TLV_OPTION_TYPE;

typedef union
{
    unsigned char byte;
    struct
    {
        unsigned scope:4;
        unsigned type:2;
    } bits;
} IPV6_ADDRESS_TYPE;

typedef struct
{
    IPV6_ADDR address;
    unsigned char prefixLength;
    unsigned char precedence;
    unsigned char label;
} IPV6_ADDRESS_POLICY;

typedef struct _IP_DATA_SEGMENT_HEADER
{
    PTR_BASE dataLocation;                          // Location of the data to transmit
    unsigned short segmentSize;                     // Size of this data segment
    unsigned short segmentLen;                      // Number of bytes of data in this segment
    unsigned char memory;                           // Type: IP_DATA_NONE, IP_DATA_DYNAMIC_BUFFER, IP_DATA_NETWORK_FIFO, IP_DATA_PIC_RAM
    unsigned char segmentType;                      // Type of segment contents
    struct _IP_DATA_SEGMENT_HEADER * nextSegment;   // Pointer to the next data segment
    void * data[0];                                 // Optional buffer space
} IP_DATA_SEGMENT_HEADER;

typedef struct _IPV6_FRAGMENT_BUFFER
{
    struct _IPV6_FRAGMENT_BUFFER * next;        // Next fragmented packet
    uint32_t identification;                       // Fragment id
    uint16_t bytesInPacket;                         // Number of bytes written to packet
    uint16_t packetSize;                            // Packet size (packet is complete when this matches bytesInPacket)
    uint16_t firstFragmentLength;                   // Length of the first fragment
    uint8_t secondsRemaining;                      // Number of seconds remaining during which the fragment can be reassembled
    uint8_t packet[1500];                          // Packet information
} IPV6_FRAGMENT_BUFFER;

typedef struct __attribute__((__packed__))
{
    uint8_t nextHeader;
    uint8_t reserved;
    union
    {
        struct __attribute__((__packed__))
        {
            unsigned m : 1;
            unsigned reserved2 : 2;
            unsigned fragmentOffset : 13;
        } bits;
        uint16_t w;
    } offsetM;
    uint32_t identification;
} IPV6_FRAGMENT_HEADER;

#define IP_DATA_NONE                (0x0u)          // The data segment is unused
#define IP_DATA_DYNAMIC_BUFFER      (0x1u)          // Data to transmit is allocated in dynamically allocated RAM
#define IP_DATA_NETWORK_FIFO        (0x2u)          // Data to transmit is stored in the Network Controller's FIFOs
#define IP_DATA_PIC_RAM             (0x3u)          // Data to transmit is stored in PIC RAM

#define IP_NO_UPPER_LAYER_CHECKSUM          (0xFFFFu)

#define IPV6_ADDR_SCOPE_UNKNOWN             0x00
#define IPV6_ADDR_SCOPE_INTERFACE_LOCAL     0x01
#define IPV6_ADDR_SCOPE_LINK_LOCAL          0x02
#define IPV6_ADDR_SCOPE_ADMIN_LOCAL         0x04
#define IPV6_ADDR_SCOPE_SITE_LOCAL          0x05
#define IPV6_ADDR_SCOPE_ORG_LOCAL           0x08
#define IPV6_ADDR_SCOPE_GLOBAL              0x0E

#define IPV6_ADDR_TYPE_UNICAST                      0x01        // Only link-local and global are currently valid for unicast
#define IPV6_ADDR_TYPE_ANYCAST                      0x02
#define IPV6_ADDR_TYPE_MULTICAST                    0x03
#define IPV6_ADDR_TYPE_SOLICITED_NODE_MULTICAST     0x04
#define IPV6_ADDR_TYPE_UNICAST_TENTATIVE            0x05

// IP packet header definition
typedef struct _IP_HEADER
{
    uint8_t    VersionIHL;
    uint8_t    TypeOfService;
    uint16_t    TotalLength;
    uint16_t    Identification;
    uint16_t    FragmentInfo;
    uint8_t    TimeToLive;
    uint8_t    Protocol;
    uint16_t    HeaderChecksum;
    IP_ADDR SourceAddress;
    IP_ADDR DestAddress;
} IP_HEADER;

#if defined (TCPIP_STACK_USE_IPV6)
// IPv6 packet header definition
typedef struct
{
    unsigned long   V_T_F;
    unsigned short  PayloadLength;
    unsigned char   NextHeader;
    unsigned char   HopLimit;
    IPV6_ADDR SourceAddress;
    IPV6_ADDR DestAddress;
} IPV6_HEADER;
#endif

#if defined (TCPIP_STACK_USE_IPV6)
typedef enum
{
    IPV6_EVENT_ADDRESS_ADDED = 1,
    IPV6_EVENT_ADDRESS_REMOVED,
}IPV6_EVENT_TYPE;

typedef const void * IPV6_HANDLE;

// prototype of an IPv6 event handler
// clients can register a handler with the IPv6 service
// Once an IPv6 event occurs the IPv6 service will called the registered handler
// The handler has to be short and fast.
// It is meant for setting an event flag, not for lengthy processing!
typedef void    (*IPV6_EVENT_HANDLER)(TCPIP_NET_HANDLE hNet, IPV6_EVENT_TYPE evType, const void* param);
#endif

// packet allocation callback function
// 1st parameter will be an IP_PACKET*
// 2nd parameter is supplied by the caller
typedef void (*IP_PACKET_ACK_FNC)(void*, void*);

// Packet structure/state tracking for IPv4/6 packets
typedef struct _IP_PACKET
{
	NET_CONFIG * netIf;					    // Net
#if defined (TCPIP_STACK_USE_IPV6)
    void * neighbor;      // The neighbor that the message was received from
#endif
    unsigned short payloadLen;				// Amount of data in payload buffer
    unsigned short headerLen;				// Total header length (IP header + IPv6 Extension headers)
    unsigned short upperLayerHeaderLen; 	// Total length of the upper layer header
    unsigned short upperLayerChecksumOffset;// Offset of the upper layer checksum
    unsigned char upperLayerHeaderType;		// Type definition for the upper-layer heaer
    union
    {
        struct
        {
	        unsigned reserved : 2;
            unsigned useUnspecAddr : 1;         // This packet should use the unspecified address
            unsigned sourceSpecified : 1;       // The upper layer or application layer specified a source address
            unsigned queued : 1;                // Packet has been queued
            unsigned packetBound : 1;           // Packet is currently bound to a socket
	        unsigned addressType : 2;		    // IP_ADDRESS_TYPE_IPV6 or IP_ADDRESS_TYPE_IPV4
        };
        unsigned char val;
    }flags;
    MAC_ADDR remoteMACAddr;                 // The packet's remote MAC address
    struct _IP_PACKET * nextPacket;         // Next packet in a queue
    IP_PACKET_ACK_FNC   ackFnc;             // function to be called when done with the packet
    void*               ackParam;           // parameter to be used
    IP_DATA_SEGMENT_HEADER payload;
    union
    {
#if defined (TCPIP_STACK_USE_IPV6)
        IPV6_HEADER ipv6Header;
#endif
        IP_HEADER ipv4Header;
    } ipHeader;
}IP_PACKET;


// public API
// 

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
bool TCPIP_IPV6_InterfaceIsUp (NET_CONFIG * pNetIf);

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
IPV6_ADDR_STRUCT * TCPIP_IPV6_DAS_SelectSourceAddress (NET_CONFIG * pNetIf, IPV6_ADDR * dest, IPV6_ADDR * requestedSource);

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
IP_PACKET * TCPIP_IP_AllocateTxPacket (NET_CONFIG * pNetIf, IP_ADDRESS_TYPE addressType, IP_PACKET_ACK_FNC ackFnc, void* ackParam);

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
unsigned short TCPIP_IP_IsTxPutReady (IP_PACKET * pkt, unsigned short count);

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
void TCPIP_IP_FreePacket (IP_PACKET * pkt);

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
bool TCPIP_IP_Put (IP_PACKET * pkt, unsigned char v);

/*****************************************************************************
  Function:
	unsigned short TCPIP_IP_PutArray (IP_PACKET * ptrPacket, 
        const void * dataSource, unsigned short len)

  Summary:
	Writes data to a packet

  Description:
	Writes data to an outgoing packet.

  Precondition:
	The TCPIP_IP_IsTxPutReady function must have returned a value greater 
    than or equal to 'len.'

  Parameters:
	ptrPacket - The packet.
    dataSource - Pointer to the data to copy to the packet.
    len - Length of the data.

  Returns:
  	unsigned short - The number of bytes of data written.
  	
  Remarks:
	None
  ***************************************************************************/
unsigned short TCPIP_IP_PutArrayHelper (IP_PACKET * pkt, const void * dataSource, uint8_t dataType, unsigned short len);
#define TCPIP_IP_PutArray(pkt,data,len)        TCPIP_IP_PutArrayHelper(pkt, data, IP_DATA_PIC_RAM, len)

/*****************************************************************************
  Function:
	unsigned short TCPIP_IP_SetPayload (IP_PACKET * ptrPacket, 
        PTR_BASE payload, unsigned short len)

  Summary:
	Appends a buffer of non-volatile data in the PIC RAM to the end of a 
    packet.

  Description:
	This function will append a segment of data in PIC RAM on to the end 
    of a TX packet that is being constructed without copying the data.  The 
    data must be maintained in RAM until the packet has been transmitted 
    (verified by the acknowledge function).

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
unsigned short TCPIP_IP_SetPayload (IP_PACKET * pkt, PTR_BASE payload, unsigned short len);


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
bool TCPIP_IP_Flush (IP_PACKET * pkt, MAC_ADDR * remoteMACAddr/*, pMACNotifyF pNofify*/);


/*****************************************************************************
  Function:
	uint8_t TCPIP_IP_Get (TCPIP_MAC_HANDLE hMac)

  Summary:
	Reads the next byte of data from the specified MAC.

  Description:
    Reads a character of data from a packet.

  Precondition:
	None

  Parameters:
	hMac - The MAC to read data from

  Returns:
    The data read.
  	
  Remarks:
	None
  ***************************************************************************/
#define TCPIP_IP_Get(hMac)                MACGet(hMac)

/*****************************************************************************
  Function:
	uint8_t TCPIP_IP_GetArray (TCPIP_MAC_HANDLE hMac, uint8_t *val, 
        uint16_t len);

  Summary:
	Reads the next byte of data from the specified MAC.

  Description:
    Reads a character of data from a packet.

  Precondition:
	None

  Parameters:
	hMac - The MAC to read data from
    val - The buffer to store the data
    len - The amount of data to read

  Returns:
    uint8_t - The number of bytes read.
  	
  Remarks:
	None
  ***************************************************************************/
#define TCPIP_IP_GetArray(hMac, val, len)     MACGetArray(hMac, val, len)

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
IPV6_ADDR_STRUCT *  TCPIP_IPV6_AddUnicastAddress (NET_CONFIG * pNetIf, IPV6_ADDR * address, uint8_t skipProcessing);

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
 void                TCPIP_IPV6_RemoveUnicastAddress (NET_CONFIG * pNetIf, IPV6_ADDR * address);

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
IPV6_ADDR_STRUCT *  TCPIP_IPV6_AddMulticastListener (TCPIP_NET_HANDLE hNet, IPV6_ADDR * address);

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
 void                TCPIP_IPV6_RemoveMulticastListener (NET_CONFIG * pNetIf, IPV6_ADDR * address);


IPV6_HANDLE TCPIP_IPV6_RegisterHandler(TCPIP_NET_HANDLE hNet, IPV6_EVENT_HANDLER handler, const void* hParam);
bool TCPIP_IPV6_DeRegisterHandler(IPV6_HANDLE hIpv6);

#endif // TCPIP_STACK_USE_IPV6

#endif // __IP_H_ 



