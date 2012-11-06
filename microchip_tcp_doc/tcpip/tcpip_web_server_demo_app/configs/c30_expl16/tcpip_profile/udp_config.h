/*******************************************************************************
  User Datagram Protocol (UDP) Configuration file

  Summary:
    UDP onfiguration file
    
  Description:
    This file contains the UDP module configuration options
    
*******************************************************************************/

/*******************************************************************************
FileName:   udp_config.h
Copyright © 2011 released Microchip Technology Inc.  All rights 
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

#ifndef _UDP_CONFIG_H_
#define _UDP_CONFIG_H_

// The dynamic port range defined by IANA consists of the 49152-65535 range
// and is meant for the selection of ephemeral ports (RFC 6056).
// Adjust to your needs but stay within the IANA range 
// First port number for randomized local port number selection
#define UDP_LOCAL_PORT_START_NUMBER (49152)
// Last port number for randomized local port number selection
#define UDP_LOCAL_PORT_END_NUMBER   (65535)

// global number of UDP sockets created dynamically
// you can COMMENT OUT this define and the number of
// needed UDP sockets will be calculated using the
// enabled TCPIP services
#define UDP_MAX_SOCKETS			(4u)//(10)

// default socket Tx buffer size
#define UDP_SOCKET_DEFAULT_TX_SIZE	256//512

// default socket Rx buffer size
#define UDP_SOCKET_DEFAULT_RX_SIZE	256//512

// This slows UDP TX performance by nearly 50%,
// except when using the ENCX24J600, which has a
// super fast DMA and incurs virtually no speed pentalty.
#define UDP_USE_TX_CHECKSUM

// check incoming packets to have proper checksum
#define UDP_USE_RX_CHECKSUM

// UDP layer configuration/initialization
typedef struct
{
    int             nSockets;   // number of sockets to be created
    const void*     pUdpSktInit;// pointer to an initialization structure
                                // if !NULL, it's an array of nSockets initialization structures 
    uint16_t        sktTxBuffLen;  // size of the socket tx buffer
    uint16_t        sktRxBuffLen;  // size of the socket rx buffer

}UDP_MODULE_CONFIG;

// This is a template of how the UDP module should be initialized and
// the parameters that it needs.
static const UDP_MODULE_CONFIG udpConfigData = 
{
	UDP_MAX_SOCKETS,
	0,
	UDP_SOCKET_DEFAULT_TX_SIZE,
	UDP_SOCKET_DEFAULT_RX_SIZE
};

#endif  // _UDP_CONFIG_H_
