/*******************************************************************************
  SSLv3 Module Headers

  Summary:
    
  Description:
*******************************************************************************/

/*******************************************************************************
FileName:  SSL.h 
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

#ifndef __SSL_H
#define __SSL_H

/****************************************************************************
  Section:
	Function Prototypes
  ***************************************************************************/

uint8_t SSLStartSession(TCP_SOCKET hTCP, void * buffer, uint8_t supDataType);
void SSLTerminate(TCP_SOCKET hTCP, uint8_t sslStubId);
void SSLPeriodic(TCP_SOCKET hTCP, uint8_t sslStubID);
uint16_t SSLRxRecord(TCP_SOCKET hTCP, uint8_t sslStubID);
void SSLRxHandshake(TCP_SOCKET hTCP, uint8_t sslStubID);
void SSLTxRecord(TCP_SOCKET hTCP, uint8_t sslStubID, uint8_t txProtocol);
void SSLTxMessage(TCP_SOCKET hTCP, uint8_t sslStubID, uint8_t msg);



#endif
