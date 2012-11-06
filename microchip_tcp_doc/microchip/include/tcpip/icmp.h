/*******************************************************************************
  ICMP Module Defs for Microchip TCP/IP Stack

  Summary:
    
  Description:
*******************************************************************************/

/*******************************************************************************
FileName:  ICMP.h 
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

#ifndef __ICMP_H
#define __ICMP_H

#define ICMP_CHECKSUM_OFFSET        2u

typedef struct
{
}ICMP_MODULE_GONFIG;


bool ICMPInitialize(const TCPIP_STACK_MODULE_CTRL* const pStackInit, const ICMP_MODULE_GONFIG* const pIcmpInit);
void ICMPDeinitialize(const TCPIP_STACK_MODULE_CTRL* const pStackInit);

IP_PACKET * ICMPAllocateTxPacketStruct (NET_CONFIG * pNetIf);

void ICMPProcess(NET_CONFIG* pConfig, NODE_INFO *remote, uint16_t len);
void ICMPSendEchoRequest (NODE_INFO * remoteNode, uint16_t sequenceNumber, uint16_t identifier);

bool ICMPBeginUsage(NET_CONFIG* pNet);
void ICMPEndUsage(void);

void ICMPRegisterCallback (void (*callback)(IP_ADDR * remoteIP, void * data));

#endif
