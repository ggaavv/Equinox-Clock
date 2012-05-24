/*******************************************************************************
  Alleged RC4 (ARCFOUR)

  Summary:
    ARCFOUR Cryptography Library
    
  Description:
    Library for Microchip TCP/IP Stack
        - Provides encryption and decryption capabilities for the ARCFOUR
          algorithm, typically used as a bulk cipher for SSL
        - Reference: http://tools.ietf.org/html/draft-kaukonen-cipher-arcfour-01
*******************************************************************************/

/*******************************************************************************
FileName:   ARCFOUR.c
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

#define __ARCFOUR_C

#include "tcpip_private.h"

#if defined(TCPIP_STACK_USE_SSL_SERVER) || defined(TCPIP_STACK_USE_SSL_CLIENT)

// To comply with US Export Control restrictions, the encryption 
// portion of the SSL module must be obtained separately from 
// Microchip. The library of Data Encryption Routines (SW300052) is 
// available for a nominal fee from www.microchipdirect.com. 
#error ARCFOUR encryption module requires SW300052 from www.microchipdirect.com

#endif //#if defined(TCPIP_STACK_USE_SSL_SERVER) || defined(TCPIP_STACK_USE_SSL_CLIENT)

