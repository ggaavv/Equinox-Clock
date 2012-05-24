/*******************************************************************************
FileName:  icmpv6_manager.h 
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
#ifndef _ICMPV6_MANAGER_H
#define _ICMPV6_MANAGER_H

//*************
// Stack types
//*************
#define ICMPV6_CHECKSUM_OFFSET                          2u

typedef struct __attribute__((__packed__))
{
    uint8_t vType;
    uint8_t vCode;
    uint16_t wChecksum;
    uint32_t Reserved;
} ICMPV6_HEADER_ROUTER_SOLICITATION;

typedef struct __attribute__((__packed__))
{
    uint8_t vType;
    uint8_t vCode;
    uint16_t wChecksum;
    uint8_t curHopLimit;
    struct __attribute__((__packed__))
    {
        unsigned Reserved : 6;
        unsigned O : 1;
        unsigned M : 1;
    } flags;
    uint16_t routerLifetime;
    uint32_t reachableTime;
    uint32_t retransTime;
} ICMPV6_HEADER_ROUTER_ADVERTISEMENT;

typedef struct  __attribute__((__packed__))
{
    uint8_t vType;
    uint8_t vCode;
    uint16_t wChecksum;
    uint32_t Reserved;
    IPV6_ADDR aTargetAddress;
} ICMPV6_HEADER_NEIGHBOR_SOLICITATION;

typedef struct  __attribute__((__packed__))
{
    uint8_t vType;
    uint8_t vCode;
    uint16_t wChecksum;
    union
    {
        struct __attribute__((__packed__))
        {
            unsigned Reserved : 5;
            unsigned O : 1;
            unsigned S : 1;
            unsigned R : 1;
        } bits;
        uint8_t Val;
    } flags;
    unsigned char Reserved1;
    unsigned short Reserved2;
    IPV6_ADDR aTargetAddress;
} ICMPV6_HEADER_NEIGHBOR_ADVERTISEMENT;

typedef struct __attribute__((__packed__))
{
    uint8_t vType;
    uint8_t vCode;
    uint16_t wChecksum;
    uint32_t Reserved;
    IPV6_ADDR aTargetAddress;
    IPV6_ADDR aDestinationAddress;
} ICMPV6_HEADER_REDIRECT;

typedef union
{
    ICMPV6_HEADER_ERROR                         header_Error;
    ICMPV6_HEADER_ECHO                          header_Echo;
    ICMPV6_HEADER_ROUTER_SOLICITATION           header_RS;
    ICMPV6_HEADER_ROUTER_ADVERTISEMENT          header_RA;
    ICMPV6_HEADER_NEIGHBOR_SOLICITATION         header_NS;
    ICMPV6_HEADER_NEIGHBOR_ADVERTISEMENT        header_NA;
    ICMPV6_HEADER_REDIRECT                      header_Rd;
} ICMPV6_HEADER_TYPES;

//*************
// Stack APIs
//*************
bool TCPIP_ICMPV6_Initialize (const TCPIP_STACK_MODULE_CTRL* const stackInit,
                       const void* const icmpv6Data);
void TCPIP_ICMPV6_Deinitialize (const TCPIP_STACK_MODULE_CTRL* const stackInit);

void TCPIP_ICMPV6_Process(NET_CONFIG * pNetIf, IPV6_ADDR_STRUCT * localIPStruct, IPV6_ADDR * localIP, IPV6_ADDR * remoteIP, uint16_t dataLen, uint16_t headerLen, uint8_t hopLimit, uint8_t addrType);

IP_PACKET * TCPIP_ICMPV6_PutHeaderError (NET_CONFIG * pNetIf, IPV6_ADDR * localIP, IPV6_ADDR * remoteIP, uint8_t code, uint8_t type, uint32_t additionalData);
IP_PACKET * TCPIP_ICMPV6_PutHeaderRouterSolicitation (NET_CONFIG * pNetIf, IPV6_ADDR * localIP, IPV6_ADDR * remoteIP);
IP_PACKET * TCPIP_ICMPV6_PutHeaderNeighborSolicitation (NET_CONFIG * pNetIf, IPV6_ADDR * localIP, IPV6_ADDR * remoteIP, IPV6_ADDR * targetAddr);
IP_PACKET * TCPIP_ICMPV6_PutHeaderNeighborAdvertisement (NET_CONFIG * pNetIf, IPV6_ADDR * localIP, IPV6_ADDR * remoteIP, IPV6_ADDR * targetAddr, bool solicited, bool override);

IP_PACKET * TCPIP_ICMPV6_Open (NET_CONFIG * pNetIf, IPV6_ADDR * localIP, IPV6_ADDR * remoteIP);


#endif

