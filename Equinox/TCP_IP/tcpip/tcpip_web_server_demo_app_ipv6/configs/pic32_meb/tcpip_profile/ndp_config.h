/*******************************************************************************
  Neighbor Discovery Protocol (NDP) Configuration file

  Summary:
    NDP configuration file
    
  Description:
    This file contains the NDP module configuration options
    
*******************************************************************************/

/*******************************************************************************
FileName:   ndp_config.h
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

#ifndef _NDP_CONFIG_H_
#define _NDP_CONFIG_H_


// Neighbor Discovery Host constants
#define MAX_RTR_SOLICITATION_DELAY              1u              // 1 s
#define RTR_SOLICITATION_INTERVAL               4u              // 4 s
#define MAX_RTR_SOLICITATIONS                   3u              // 3 transmissions

// Neighbor Discovery Node constants
#define MAX_MULTICAST_SOLICIT                   3u              // 3 transmissions
#define MAX_UNICAST_SOLICIT                     3u              // 3 transmissions
#define MAX_ANYCAST_DELAY_TIME                  1u              // 1 s
#define MAX_NEIGHBOR_ADVERTISEMENT              3u              // 3 transmissions
#define REACHABLE_TIME                          30u             // 30 s
#define RETRANS_TIMER                           1u              // 1 s
#define DELAY_FIRST_PROBE_TIME                  5u              // 5 s

// Sets the lifetime to 2 hours
#define NDP_VALID_LIFETIME_TWO_HOURS            (60 * 60 * 2)

// Sets the maximum transmit unit increase timeout in seconds
#define IPV6_MTU_INCREASE_TIMEOUT               600ul           // 600 seconds


#endif  // _NDP_CONFIG_H_
