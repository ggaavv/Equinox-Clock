/*******************************************************************************
  User Datagram Protocol (UDPPERFORMANCETEST) Configuration file

  Summary:
    UDPPERFORMANCETEST Configuration file
    
  Description:
    This file contains the UDPPERFORMANCETEST module configuration options
    
*******************************************************************************/

/*******************************************************************************
FileName:  udpperformancetest_config.h 
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

#ifndef _UDPPERFORMANCETEST_CONFIG_H_
#define _UDPPERFORMANCETEST_CONFIG_H_

// If defined tells system to wait for DHCP module to finish
#define DELAY_UDP_PERFORMANCE_TEST

// Which UDP port to broadcast from for the UDP tests
#define PERFORMANCE_PORT	9

#endif  // _UDPPERFORMANCETEST_CONFIG_H_
