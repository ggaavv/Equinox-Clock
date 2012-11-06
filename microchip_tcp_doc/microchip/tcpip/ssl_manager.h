/*******************************************************************************
  SSLv3 Module Manager API

  Summary:
    
  Description:
*******************************************************************************/

/*******************************************************************************
FileName:  ssl_manager.h
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

#ifndef __SSL_MANAGER_H_
#define __SSL_MANAGER_H_


bool SSLInit(const TCPIP_STACK_MODULE_CTRL* const stackData,
             const SSL_MODULE_CONFIG* const sslData);
void SSLDeInit(const TCPIP_STACK_MODULE_CTRL* const stackData);


void SSLMACBegin(uint8_t* MACSecret, uint32_t seq, uint8_t protocol, uint16_t len);
void SSLMACAdd(uint8_t* data, uint16_t len);
void SSLMACCalc(uint8_t* MACSecret, uint8_t* result);


void SSLStartPartialRecord(TCP_SOCKET hTCP, uint8_t sslStubID, uint8_t txProtocol, uint16_t wLen);
#define SSLFlushPartialRecord(a)		TCPSSLPutRecordHeader(a, NULL, false);
#define SSLFinishPartialRecord(a)		TCPSSLPutRecordHeader(a, NULL, true);


#endif  // __SSL_MANAGER_H_

