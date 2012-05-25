/*******************************************************************************
  TCP Manager internal stack API

  Summary:
    
  Description:
*******************************************************************************/

/*******************************************************************************
FileName:  tcp_manager.h 
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

#ifndef __TCP_MANAGER_H_
#define __TCP_MANAGER_H_


/****************************************************************************
  Section:
	Type Definitions
  ***************************************************************************/


/****************************************************************************
  Section:
	Function Declarations
  ***************************************************************************/


bool TCPInit(const TCPIP_STACK_MODULE_CTRL* const stackInit, const TCP_MODULE_CONFIG* const pTcpInit);
void TCPDeInit(const TCPIP_STACK_MODULE_CTRL* const stackInit);
bool TCPProcess(NET_CONFIG* pConfig, NODE_INFO* remote, IP_ADDR* localIP, uint16_t len);
bool TCPProcessIPv6(NET_CONFIG* pPktIf, IPV6_ADDR * localIP, IPV6_ADDR * remoteIP, uint16_t dataLen, uint16_t headerLen);
void TCPTick(void);
bool TCPTaskPending(void);




IP_PACKET * TCPAllocateTxPacketStruct (NET_CONFIG * pConfig, IP_ADDRESS_TYPE);


void TCPSSLPutRecordHeader(TCP_SOCKET hTCP, uint8_t* hdr, bool recDone);
uint16_t TCPSSLGetPendingTxSize(TCP_SOCKET hTCP);
void TCPSSLHandleIncoming(TCP_SOCKET hTCP);

bool TCPSetSourceIPAddress(TCP_SOCKET s, IP_ADDRESS_TYPE addType, IP_MULTI_ADDRESS* localAddress);

bool TCPSetDestinationIPAddress(TCP_SOCKET s, IP_ADDRESS_TYPE addType, IP_MULTI_ADDRESS* remoteAddress);


/*  BACKWARD COMPATIBILITY TCP calls.
 *  Internal use only!
 *  Do NOT use them for new projects!
 *  Support for them will be droped from the future releases!
*/

// Create a server socket and ignore dwRemoteHost.
#define TCP_OPEN_SERVER		0u
// Create a client socket and use dwRemoteHost as a RAM pointer to a hostname string.
#define TCP_OPEN_RAM_HOST	1u
// Create a client socket and use dwRemoteHost as a const pointer to a hostname string.
#define TCP_OPEN_ROM_HOST	2u
// Create a client socket and use dwRemoteHost as a literal IP address.
#define TCP_OPEN_IP_ADDRESS	3u
// Create a client socket and use dwRemoteHost as a pointer to a NODE_INFO structure containing the exact remote IP address and MAC address to use.
#define TCP_OPEN_NODE_INFO	4u
// Create a client socket and use dwRemoteHost as a pointer to an IPV6_ADDR structure containing a remote IPv6 address
#define TCP_OPEN_IPV6_ADDRESS 5u

// Define names of socket types
#define TCP_SOCKET_TYPES
	#define TCP_PURPOSE_GENERIC_TCP_CLIENT 		0
	#define TCP_PURPOSE_GENERIC_TCP_SERVER 		1
	#define TCP_PURPOSE_TELNET 			2
	#define TCP_PURPOSE_FTP_COMMAND 		3
	#define TCP_PURPOSE_FTP_DATA 			4
	#define TCP_PURPOSE_TCP_PERFORMANCE_TX 		5
	#define TCP_PURPOSE_TCP_PERFORMANCE_R 		6
	#define TCP_PURPOSE_UART_2_TCP_BRIDGE 		7
	#define TCP_PURPOSE_HTTP_SERVER 		8
	#define TCP_PURPOSE_DEFAULT 		 	9 
	#define TCP_PURPOSE_BERKELEY_SERVER 		10
	#define TCP_PURPOSE_BERKELEY_CLIENT 		11
	#define TCP_PURPOSE_ANY             		12      // generic, can be used for any purpose
#define END_OF_TCP_SOCKET_TYPES



TCP_SOCKET          TCPOpen(uint32_t dwRemoteHost, uint8_t vRemoteHostType, uint16_t wPort, uint8_t vSocketPurpose);


#endif  // __TCP_MANAGER_H_
