/*******************************************************************************
  UDP Module manager - private stack API

  Summary:
    
  Description:
*******************************************************************************/

/*******************************************************************************
FileName:  udp_manager.h 
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

#ifndef __UDP__MANAGER_H_
#define __UDP__MANAGER_H_


bool UDPInit(const TCPIP_STACK_MODULE_CTRL* const stackInit, const UDP_MODULE_CONFIG* const pUdpInit);
void UDPDeInit(const TCPIP_STACK_MODULE_CTRL* const stackInit);

void UDPTask(NET_CONFIG* pConfig);



IP_PACKET * UDPAllocateTxPacketStruct (NET_CONFIG * pConfig, IP_ADDRESS_TYPE addressType);

bool UDPProcess(NET_CONFIG* pConfig, NODE_INFO *remoteNode, IP_ADDR *localIP, uint16_t len);
bool UDPProcessIPv6(NET_CONFIG * pConfig, IPV6_ADDR * localIP, IPV6_ADDR * remoteIP, uint16_t dataLen, uint16_t headerLen);

void UDPResetHeader(UDP_HEADER * h);



// sets the source IP address of a packet
bool UDPSetSourceIPAddress(UDP_SOCKET s, IP_ADDRESS_TYPE addType, IP_MULTI_ADDRESS* localAddress);


// sets the destination IP address of a packet
bool UDPSetDestinationIPAddress(UDP_SOCKET s, IP_ADDRESS_TYPE addType, IP_MULTI_ADDRESS* remoteAddress);



void UDPDiscardNet(NET_CONFIG* pNetIf);


/*  BACKWARD COMPATIBILITY UDP calls.
 *  Internal use only!
 *  Do NOT use them for new projects!
 *  Support for them will be droped from the future releases!
*/


// Create a server socket and ignore dwRemoteHost.
#define UDP_OPEN_SERVER		0u
// Create a client socket and use dwRemoteHost as a RAM pointer to a hostname string.
#define UDP_OPEN_RAM_HOST	1u
// Create a client socket and use dwRemoteHost as a const pointer to a hostname string.
#define UDP_OPEN_ROM_HOST	2u
// Create a client socket and use dwRemoteHost as a literal IP address.
#define UDP_OPEN_IP_ADDRESS	3u
// Create a client socket and use dwRemoteHost as a literal IPv6 address
#define UDP_OPEN_IPV6_ADDRESS 4u

// Create a client socket and use dwRemoteHost as a pointer to a NODE_INFO structure containing the exact remote IP address and MAC address to use.
#define UDP_OPEN_NODE_INFO	5u

UDP_SOCKET          UDPOpen(uint32_t remoteHost, uint8_t remoteHostType, UDP_PORT localPort,UDP_PORT remotePort);
// translation from the obsolete form of UDPOpen; not in use anymore

  


#endif // __UDP__MANAGER_H_


