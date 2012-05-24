/*******************************************************************************
  MAC Module Defs for Microchip Stack

  Summary:
    
  Description:
*******************************************************************************/

/*******************************************************************************
FileName:  tcpip_mac.h 
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

#ifndef __TCPIP_MAC_H_
#define __TCPIP_MAC_H_

#include "hardware_profile.h"


#if defined(ENC_CS_TRIS) && defined(WF_CS_TRIS)
	#error "Error in hardware_profile.h.  Must select either the ENC28J60 or the MRF24WB10 but not both ENC_CS_TRIS and WF_CS_TRIS."
#endif
#if defined(ENC100_INTERFACE_MODE) && defined(WF_CS_TRIS)
	//#error "Error in hardware_profile.h.  Must select either the ENCX24J600 or the MRF24WB10 but not both ENC100_INTERFACE_MODE and WF_CS_TRIS."
#endif
#if defined(ENC100_INTERFACE_MODE) && defined(ENC_CS_TRIS)
	#error "Error in hardware_profile.h.  Must select either the ENC28J60 or the ENCX24J600 but not both ENC_CS_TRIS and ENC100_INTERFACE_MODE."
#endif



// Structure to contain a MAC address
typedef struct __attribute__((__packed__))
{
    uint8_t v[6];
} MAC_ADDR;

// A generic structure representing the Ethernet header starting all Ethernet
// frames
typedef struct  __attribute__((aligned(2), packed))
{
	MAC_ADDR        DestMACAddr;
	MAC_ADDR        SourceMACAddr;
	TCPIP_UINT16_VAL        Type;
} ETHER_HEADER;

// Definition to represent an IPv4 address
typedef union
{
    uint32_t Val;
    uint16_t w[2];
    uint8_t  v[4];
} IPV4_ADDR;
// backwards compatibility definition
typedef IPV4_ADDR   IP_ADDR;

// Definition to represent an IPv6 address
typedef union
{
    uint8_t  v[16];
    uint16_t w[8];
    uint32_t d[4];
} IPV6_ADDR;

// Definition of the supported address types
typedef enum
{
    // IP_ADDRESS_TYPE_ANY = 0, // either IPv4 or IPv6; unspecified; Not supported yet!
    IP_ADDRESS_TYPE_IPV4 = 1,   // IPv4 address type
    IP_ADDRESS_TYPE_IPV6        // IPv6 address type
}IP_ADDRESS_TYPE;
    

// definition to represent multiple IP addresses
typedef union
{
    IPV4_ADDR v4Add;
    IPV6_ADDR v6Add;
}IP_MULTI_ADDRESS;
    
        

// Address structure for a node
typedef struct __attribute__((__packed__))
{
    IP_ADDR     IPAddr;
    MAC_ADDR    MACAddr;
} NODE_INFO;

typedef struct __attribute__((__packed__)) _IPV6_ADDR_STRUCT
{
    struct _IPV6_ADDR_STRUCT * next;
    struct _IPV6_ADDR_STRUCT * prev;
    IPV6_ADDR address;
    unsigned long validLifetime;
    unsigned long preferredLifetime;
    unsigned long lastTickTime;
    unsigned char prefixLen;
    struct __attribute__((__packed__))
    {
        unsigned char precedence;                   // Allow preferences
        unsigned scope                  :4;         // Link-local, site-local, global.
        unsigned label                  :4;         // Policy label
        unsigned type                   :2;         // Uni-, Any-, Multi-cast
        unsigned temporary              :1;         // Indicates that the address is temporary (not public)
    }flags;
} IPV6_ADDR_STRUCT;


#define ETHERTYPE_IPV4      	(0x0800u)
#define ETHERTYPE_IPV6          (0x86DDu)
#define ETHERTYPE_ARP     	    (0x0806u)
#define ETHERTYPE_UNKNOWN 	    (0xFFFFu)



/************************************
 *  MAC parameterized interface
 *************************************/



typedef enum
{
    TCPIP_MAC_RES_OK,               // operation successful
    //
    TCPIP_MAC_RES_TYPE_ERR,         // unsupported type
    TCPIP_MAC_RES_IS_BUSY,          // device is in use
    TCPIP_MAC_RES_INIT_FAIL,        // generic initialization failure
    TCPIP_MAC_RES_PHY_INIT_FAIL,    // PHY initialization failure
    TCPIP_MAC_RES_EVENT_INIT_FAIL,  // Event system initialization failure
    TCPIP_MAC_RES_OP_ERR,           // unsupported operation
    TCPIP_MAC_RES_ALLOC_ERR,        // memory allocation error
    TCPIP_MAC_RES_INSTANCE_ERR,     // already instantiated, initialized error
    
    
}TCPIP_MAC_RES;         // list of return codes from MAC functions


typedef const void* TCPIP_MAC_HANDLE;     // handle to a MAC

typedef enum
{
    // Default/unspecified/unbound MAC interface
    TCPIP_MAC_ID_NONE          = 0,
    
    // External ENC28J60 device: room for 5 ENCJ60 devices
    TCPIP_MAC_ID_ENCJ60        = 1,       
    //
    
    // External ENCX24J600 device: room for 5 ENCJ600 devices 
    TCPIP_MAC_ID_ENCJ600       = 6,       
    //
    
    // ETH97J60 device: room for 5 97J60 devices
    TCPIP_MAC_ID_97J60         = 11,       
    //
    
    // internal/embedded PIC32 MAC: room for 5 PIC32 devices
    TCPIP_MAC_ID_PIC32INT      = 16,       
    //
    
    // MRF24W WiFi MAC:  room for 5 MRF24W devices
    TCPIP_MAC_ID_MRF24W     = 21,       
    //
    

    // room for additional MAC devices

    //
    //
}TCPIP_MAC_ID;        // list of the supported MACs


// all subsequent MAC functions take the hMac as the 1st parameter 
// hMac is a handle to a MAC
// This way multiple instances of the same MAC could be created
// for platforms that support it


// closes a MAC client
TCPIP_MAC_RES       MACClose(TCPIP_MAC_HANDLE hMac);
// link status
bool                MACIsLinked(TCPIP_MAC_HANDLE hMac);
// read header
int                 MACGetHeader(TCPIP_MAC_HANDLE hMac, MAC_ADDR *remote, uint16_t* type);
void                MACSetReadPtrInRx(TCPIP_MAC_HANDLE hMac, uint16_t offset);
PTR_BASE            MACGetReadPtrInRx(TCPIP_MAC_HANDLE hMac);
PTR_BASE            MACSetWritePtr(TCPIP_MAC_HANDLE hMac, PTR_BASE address);
PTR_BASE            MACSetReadPtr(TCPIP_MAC_HANDLE hMac, PTR_BASE address);
PTR_BASE            MACSetBaseReadPtr(TCPIP_MAC_HANDLE hMac, PTR_BASE address);
uint8_t                MACGet(TCPIP_MAC_HANDLE hMac);
uint16_t              MACGetArray(TCPIP_MAC_HANDLE hMac, uint8_t *val, uint16_t len);
void                MACDiscardRx(TCPIP_MAC_HANDLE hMac);
uint16_t              MACGetFreeRxSize(TCPIP_MAC_HANDLE hMac);
void                MACMemCopyAsync(TCPIP_MAC_HANDLE hMac, PTR_BASE destAddr, PTR_BASE sourceAddr, uint16_t len);
bool                MACIsMemCopyDone(TCPIP_MAC_HANDLE hMac);

void                MACPutHeader(TCPIP_MAC_HANDLE hMac, MAC_ADDR *remote, uint16_t type, uint16_t dataLen);
bool                MACIsTxReady(TCPIP_MAC_HANDLE hMac);
void                MACPut(TCPIP_MAC_HANDLE hMac, uint8_t val);
void                MACPutArray(TCPIP_MAC_HANDLE hMac, uint8_t *val, uint16_t len);
void                MACFlush(TCPIP_MAC_HANDLE hMac);
bool                MACCheckLink(TCPIP_MAC_HANDLE hMac);

PTR_BASE            MACGetTxBaseAddr(TCPIP_MAC_HANDLE hMac);     // replaced BASE_TX_ADDR
PTR_BASE            MACGetHttpBaseAddr(TCPIP_MAC_HANDLE hMac);   // replaced BASE_HTTPB_ADDR
PTR_BASE            MACGetSslBaseAddr(TCPIP_MAC_HANDLE hMac);    // replaced BASE_SSLB_ADDR

uint16_t              MACGetRxSize(TCPIP_MAC_HANDLE hMac);     // replaced RXSIZE			(EMAC_RX_BUFF_SIZE)
uint16_t              MACGetRamSize(TCPIP_MAC_HANDLE hMac);    // replaced RAMSIZE			(2*RXSIZE)


uint16_t              MACGetTxBuffSize(TCPIP_MAC_HANDLE hMac);        // replaced #define MAC_TX_BUFFER_SIZE			(1500ul)


uint16_t              MACCalcRxChecksum(TCPIP_MAC_HANDLE hMac, uint16_t offset, uint16_t len);
uint16_t              MACCalcIPBufferChecksum(TCPIP_MAC_HANDLE hMac, uint16_t len);       // replaced CalcIPBufferChecksum

void                MACSetRXHashTableEntry(TCPIP_MAC_HANDLE hMac, MAC_ADDR DestMACAddr);      // replaced SetRXHashTableEntry()

void                MACPowerDown(TCPIP_MAC_HANDLE hMac);
void	            MACEDPowerDown(TCPIP_MAC_HANDLE hMac);
void 	            MACPowerUp(TCPIP_MAC_HANDLE hMac);

void                MACProcess(TCPIP_MAC_HANDLE hMac);
bool                MACRxFilter(TCPIP_MAC_HANDLE hMac, NODE_INFO* pPkt);

void                MACConnect(TCPIP_MAC_HANDLE hMac);

#if defined(TCPIP_STACK_USE_EVENT_NOTIFICATION)
#include "tcpip/mac_events.h"

TCPIP_MAC_EVENT_RESULT    MACEventSetNotifyEvents(TCPIP_MAC_HANDLE hMac, TCPIP_MAC_EVENT_GROUP tcpEvGroup, TCPIP_EVENT tcpipEvents);
TCPIP_MAC_EVENT_RESULT    MACEventClearNotifyEvents(TCPIP_MAC_HANDLE hMac, TCPIP_MAC_EVENT_GROUP tcpEvGroup, TCPIP_EVENT tcpipEvents);
TCPIP_MAC_EVENT_RESULT    MACEventAck(TCPIP_MAC_HANDLE hMac, TCPIP_MAC_EVENT_GROUP tcpEvGroup, TCPIP_EVENT tcpipEvents);
TCPIP_EVENT               MACEventGetPending(TCPIP_MAC_HANDLE hMac, TCPIP_MAC_EVENT_GROUP tcpEvGroup);
TCPIP_MAC_EVENT_RESULT    MACEventSetNotifyHandler(TCPIP_MAC_HANDLE hMac, TCPIP_MAC_EVENT_GROUP tcpEvGroup, pMacEventF eventHandler, void* hParam);
#endif  // (TCPIP_STACK_USE_EVENT_NOTIFICATION)



	
#endif // __TCPIP_MAC_H_
