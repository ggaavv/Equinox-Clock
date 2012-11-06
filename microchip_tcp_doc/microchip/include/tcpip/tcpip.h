/*******************************************************************************
  Microchip TCP/IP Stack Include File

  Summary:
    
  Description:
*******************************************************************************/

/*******************************************************************************
FileName:  tcpip.h 
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

#ifndef __TCPIP_H__
#define __TCPIP_H__

#define TCPIP_STACK_VERSION 		"v6.00-Beta"		// TCP/IP stack version

#include <string.h>
#include <stdlib.h>
#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>

#include "Compiler.h"
#include "hardware_profile.h"
        
#include "system/system_services.h"

#include "tcpip/tcpip_types.h"

/*******************************************************************
 * List of the TCPIP stack supported modules
 *   The following enumeration lists all the modules supported
 *   by the TCPIP stack.
 *******************************************************************/

typedef enum
{
    TCPIP_MODULE_IP              = 0,
    TCPIP_MODULE_ICMP,      
    TCPIP_MODULE_ARP,
	TCPIP_MODULE_IPV6,
	TCPIP_MODULE_ICMPV6,
	TCPIP_MODULE_NDP,
    TCPIP_MODULE_UDP,
    TCPIP_MODULE_TCP,
    TCPIP_MODULE_DHCP_CLIENT,
    TCPIP_MODULE_DHCP_SERVER,
	TCPIP_MODULE_AUTOIP,
    TCPIP_MODULE_ANNOUNCE,
    TCPIP_MODULE_DNS_CLIENT,
	TCPIP_MODULE_NBNS,
    TCPIP_MODULE_SNTP,
    TCPIP_MODULE_FTP_SERVER,
    TCPIP_MODULE_HTTP_SERVER,
    TCPIP_MODULE_RSA,
    TCPIP_MODULE_SSL,
    TCPIP_MODULE_SNMP_SERVER,
    TCPIP_MODULE_DYNDNS_CLIENT,
    TCPIP_MODULE_BERKELEY,


     // total number of the TCPIP modules
    TCPIP_MODULES_NO                                     
}TCPIP_STACK_MODULE;

// TCPIP module initialization/configuration structure
// Each stack module will be configured
// with a user defined initialization/configuration structure 
typedef struct
{
	TCPIP_STACK_MODULE		moduleId;
	const void * const		configData;
}TCPIP_STACK_MODULE_CONFIG;

/*******************************************************************
 * User Configuration
 *   Load the user-specific configuration from tcpip_config.h
 *******************************************************************/
#include "tcpip_config.h"

/*******************************************************************
 * Configuration Rules Enforcement
 *   The following section enforces requirements for modules based 
 *   on configurations selected in tcpip_config.h
 *******************************************************************/

#include "tcpip/tcpip_mac.h"
#include "tcpip/tcpip_manager.h"

#include "tcpip_manager_private.h"

#include "tcpip/tcpip_helpers.h"
#include "tcpip/tcpip_events.h"
#include "tcpip/mac_events.h"
#include "tcpip/ndp.h"
#include "tcpip/ip.h"
#include "tcpip/icmpv6.h"
#include "tcpip/arp.h"
#include "tcpip/rsa.h"
#include "tcpip/arcfour.h"
#include "tcpip/auto_ip.h"
#include "tcpip/xeeprom.h"
#include "tcpip/spi_flash.h"
#include "tcpip/spi_ram.h"
#include "tcpip/udp.h"
#include "tcpip/tcp.h"
#include "tcpip/berkeley_api.h"
#include "tcpip/dhcp.h"
#include "tcpip/dns.h"
#include "tcpip/mpfs2.h"
#include "tcpip/ftp.h"
#include "tcpip/icmp.h"
#include "tcpip/announce.h"
#include "tcpip/nbns.h"
#include "tcpip/dns.h"
#include "tcpip/dyn_dns.h"
#include "tcpip/telnet.h"
#include "tcpip/smtp.h"
#include "tcpip/tftpc.h"
#include "tcpip/reboot.h"
#include "tcpip/sntp.h"
#include "tcpip/ssl.h"
#include "tcpip/tcpip_storage.h"


#if defined(TCPIP_STACK_USE_SNMP_SERVER)
	#include "tcpip/snmp.h"
	#include "mib.h"
#endif

#if defined(TCPIP_STACK_USE_UDP_PERFORMANCE_TEST)
	#include "tcpip/udp_performance_test.h"
#endif

#if defined(TCPIP_STACK_USE_TCP_PERFORMANCE_TEST)
	#include "tcpip/tcp_performance_test.h"
#endif

#if defined(TCPIP_IF_MRF24W)
	#include "../../tcpip/wifi/wf_mac.h"
#endif

#ifdef TCPIP_STACK_USE_MDD
	#include "tcpip/file_system.h"
	#include "tcpip/_http2.h"
#else
	#include "tcpip/http2.h"
#endif

#endif  // __TCPIP_H__
