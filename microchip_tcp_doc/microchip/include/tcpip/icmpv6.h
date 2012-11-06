/*******************************************************************************

  Summary:
    
  Description:
*******************************************************************************/

/*******************************************************************************
FileName:  ICMPv6.h 
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
#ifndef _ICMPV6_H
#define _ICMPV6_H


//*********************
// ICMPv6 Public Types
//*********************

// ICMPv6 packet types
typedef enum
{
    ICMPV6_ERROR_DEST_UNREACHABLE =      1u,        // Destination Unreachable error packet
    ICMPV6_ERROR_PACKET_TOO_BIG =        2u,        // Packet Too Big error packet
    ICMPV6_ERROR_TIME_EXCEEDED =         3u,        // Time Exceeded error packet
    ICMPV6_ERROR_PARAMETER_PROBLEM =     4u,        // Parameter Problem error packet
    ICMPV6_INFO_ECHO_REQUEST =           128u,      // Echo Request packet
    ICMPV6_INFO_ECHO_REPLY =             129u,      // Echo Reply packet
    ICMPV6_INFO_ROUTER_SOLICITATION =    133u,      // Router solicitation NDP packet
    ICMPV6_INFO_ROUTER_ADVERTISEMENT =   134u,      // Router advertisement NDP packet
    ICMPV6_INFO_NEIGHBOR_SOLICITATION =  135u,      // Neighbor Solicitation NDP packet
    ICMPV6_INFO_NEIGHBOR_ADVERTISEMENT = 136u,      // Neighbor Advertisement NDP packet
    ICMPV6_INFO_REDIRECT =               137u       // Redirect NDP packet
} ICMPV6_PACKET_TYPES;

// Definitions for ICMPv6 Destination Unreachable error code
typedef enum
{
    ICMPV6_ERR_DU_NO_ROUTE =                         0u,
    ICMPV6_ERR_DU_PROHIBITED =                       1u,
    ICMPV6_ERR_DU_OUTSIDE_SCOPE =                    2u,
    ICMPV6_ERR_DU_ADDR_UNREACHABLE =                 3u,
    ICMPV6_ERR_DU_PORT_UNREACHABLE =                 4u,
    ICMPV6_ERR_DU_SRC_FAILED_INGRESS_POLICY =        5u,
    ICMPV6_ERR_DU_REJECT_ROUTE =                     6u
} ICMPV6_ERR_DU_CODE;

// Definition for ICMPv6 Packet Too Big error code
#define ICMPV6_ERR_PTB_CODE                             0u

// Definitions for ICMPv6 Time Exceeded error code
typedef enum
{
    ICMPV6_ERR_TE_HOP_LIMIT_EXCEEDED =               0u,
    ICMPV6_ERR_TE_FRAG_ASSEMBLY_TIME_EXCEEDED =      1u
} ICMPV6_ERR_TE_CODE;

// Definitions for ICMPv6 Parameter Problem error code
typedef enum
{
    ICMPV6_ERR_PP_ERRONEOUS_HEADER =                 0u,
    ICMPV6_ERR_PP_UNRECOGNIZED_NEXT_HEADER =         1u,
    ICMPV6_ERR_PP_UNRECOGNIZED_IPV6_OPTION =         2u
} ICMPV6_ERR_PP_CODE;

// Definition for ICMPv6 Packet Echo Request info code
#define ICMPV6_INFO_EREQ_CODE                           0u

// Definition for ICMPv6 Packet Echo Reply info code
#define ICMPV6_INFO_ERPL_CODE                           0u

// Header for an ICMPv6 Error packet
typedef struct __attribute__((__packed__))
{
    uint8_t vType;
    uint8_t vCode;
    uint16_t wChecksum;
    uint32_t additionalData;   // Unused for Dest. Unreachable and Time Exceeded.  MTU for MTU.  Pointer for Parameter Problem.
} ICMPV6_HEADER_ERROR;

// Header for an ICMPv6 Echo Request/Reply packet
typedef struct __attribute__((__packed__))
{
    uint8_t vType;
    uint8_t vCode;
    uint16_t wChecksum;
    uint16_t identifier;
    uint16_t sequenceNumber;
} ICMPV6_HEADER_ECHO;


//*********************
// ICMPv6 Public APIs
//*********************
IP_PACKET * TCPIP_ICMPV6_PutHeaderEchoRequest (NET_CONFIG * pNetIf, IPV6_ADDR * localIP, IPV6_ADDR * remoteIP, uint8_t type, uint16_t identifier, uint16_t sequenceNumber);
#define TCPIP_ICMPV6_PutHeaderEchoReply TCPIP_ICMPV6_PutHeaderEchoRequest

bool TCPIP_ICMPV6_Flush (IP_PACKET * pkt);

void TCPIP_ICMPV6_Close (IP_PACKET * pkt);

// Registers a callback to allow the application layer to process incoming ICMPv6 packets
void TCPIP_ICMPV6_RegisterCallback (void (*callback)(uint8_t type, IPV6_ADDR * localIP, IPV6_ADDR * remoteIP, void * header));


#endif







