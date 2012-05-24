/*******************************************************************************
  MAC Module private interface for Microchip TCP/IP Stack

  Summary:
    
  Description:
*******************************************************************************/

/*******************************************************************************
FileName:   mac_private.h
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

#ifndef _MAC_PRIVATE_H_
#define _MAC_PRIVATE_H_



// MAC module initialization data
typedef struct
{
    // TX descriptors and buffers
    size_t      _nTxDescriptors;       // number of descriptors to set
    size_t      _TxBuffSize;           // size of the corresponding buffer
    // RX descriptors and buffers
    size_t      _nRxDescriptors;       // number of descriptors to set
    size_t      _RxBuffSize;           // size of the corresponding buffer

}TCPIP_MAC_MODULE_GONFIG;


// function to initialize a MAC
TCPIP_MAC_RES       MACInitialize(const TCPIP_STACK_MODULE_CTRL* const stackData,
                                  const TCPIP_MAC_MODULE_GONFIG* const moduleData );


// function to de-initialize a MAC
TCPIP_MAC_RES       MACDeinitialize(const TCPIP_STACK_MODULE_CTRL* const stackData );


// function to open a MAC and get a client handle
TCPIP_MAC_HANDLE    MACOpen( TCPIP_MAC_ID macId );
	
#endif  // _MAC_PRIVATE_H_

