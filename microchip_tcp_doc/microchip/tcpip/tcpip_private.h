/*******************************************************************************
  Microchip TCP/IP Stack Include File

  Summary:
   Private include file for the TCPIP stack
    
  Description:
*******************************************************************************/

/*******************************************************************************
FileName:  tcpip_private.h 
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

#ifndef __TCPIP_STACK_PRIVATE_H__
#define __TCPIP_STACK_PRIVATE_H__


#include <string.h>
#include <stdlib.h>
#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>

#include "Compiler.h"
#include "hardware_profile.h"
        
#include "common/lfsr.h"
#include "common/hashes.h"
#include "common/big_int.h"

#include "system/system_services.h"
#include "system/system_debug.h"



// public module configuration
#include "tcpip_config.h"


// public module intereface
#include "tcpip/tcpip.h"
	
    
// private stack configuration and checking
#include "tcpip_config_private.h"


// private stack manager interface

#include "ndp_manager.h"

#include "ip_manager.h"

#include "icmpv6_manager.h"

#include "dhcp_manager.h"

#include "arp_manager.h"

#include "dns_manager.h"

#include "tcp_manager.h"

#include "udp_private.h"
#include "udp_manager.h"

#include "sntp_manager.h"

#include "ssl_manager.h"
#if defined(TCPIP_STACK_USE_SSL_SERVER) || defined(TCPIP_STACK_USE_SSL_CLIENT)
    #include "ssl_private.h"
#endif

// extra private headers
#include "tcpip/tcpip_types.h"

#include "tcpip_heap_alloc.h"

// list of modules private interface

#endif  // __TCPIP_STACK_PRIVATE_H__


