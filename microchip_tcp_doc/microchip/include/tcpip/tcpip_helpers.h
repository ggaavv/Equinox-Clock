/*******************************************************************************
  Header file for tcpip_stack_helpers

  Summary:
    SUMMARY
    
  Description:
    DESCRIPTION
*******************************************************************************/

/*******************************************************************************
FileName:  tcpip_helpers.h 
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

#ifndef __TCPIP_HELPERS_H_
#define __TCPIP_HELPERS_H_

bool	TCPIP_HELPER_StringToIPAddress(const char* str, IP_ADDR* IPAddress);

#if defined (TCPIP_STACK_USE_IPV6)
    bool TCPIP_HELPER_StringToIPv6Address (uint8_t * str, IPV6_ADDR * addr);
#endif

extern __inline__ bool __attribute__((always_inline)) TCPIP_HELPER_IsBcastAddress(IP_ADDR* IPAddress)
{
    return (IPAddress->Val == 0xFFFFFFFF);
}

extern __inline__ bool __attribute__((always_inline)) TCPIP_HELPER_IsMcastAddress(IP_ADDR* IPAddress)
{
    return ((IPAddress->v[0] & 0xf0) == 0xE0);
}


uint16_t   swaps(uint16_t v);
uint32_t   swapl(uint32_t v);

#endif  // __TCPIP_HELPERS_H_

