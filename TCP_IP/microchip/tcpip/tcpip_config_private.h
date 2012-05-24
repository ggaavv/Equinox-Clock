/*******************************************************************************
  tcpip Configuration File

  Summary:
    
  Description:
*******************************************************************************/

/*******************************************************************************
FileName:   tcpip_config_private.h
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

#ifndef __TCPIP_CONFIG_PRIVATE_H_
#define __TCPIP_CONFIG_PRIVATE_H_

#include "tcpip/tcpip.h"

// number of the supported interfaces at build time
// 
#define TCPIP_NETWORK_INTERFACES  (sizeof(TCPIP_HOSTS_CONFIGURATION)/sizeof(*TCPIP_HOSTS_CONFIGURATION))

#include "tcpip_config.h"
#include "announce_config.h"
#include "arp_config.h"
#include "berekely_api_config.h"
#include "dhcp_config.h"
#include "dhcps_config.h"
#include "dns_config.h"
#include "dnss_config.h"
#include "dyndns_config.h"
#include "ftp_config.h"
#include "generic_tcp_example_config.h"
#include "http2_config.h"
#include "icmp_config.h"
#include "icmpv6_config.h"
#include "ip_config.h"
#include "mac_config.h"
#include "mpfs2_config.h"
#include "nbns_config.h"
#include "ndp_config.h"
#include "ping_config.h"
#include "reboot_config.h"
#include "smtp_config.h"
#include "snmp_config.h"
#include "snmpv3_config.h"
#include "sntp_config.h"
#include "ssl_config.h"
#include "tcp_config.h"
#include "tcp_performance_test_config.h"
#include "tcpip_storage_config.h"
#include "telnet_config.h"
#include "tftpc_config.h"
#include "udp_config.h"
#include "udp_performance_test_config.h"
#include "wf_config.h"

// Checks for configuration errors //

#if !defined(TCPIP_STACK_DRAM_SIZE)
    #error "This stack uses dynamic memory allocation."
    #error "Define TCPIP_STACK_DRAM_SIZE in the tcpip_config.h"
    #error "allocate enough heap space for your project."
#elif defined (__PIC32MX__)
    #if (TCPIP_STACK_DRAM_SIZE < 8*1024)
        #warning "You need at least 8KB of heap space for the TCPIP stack to run properly."
    #endif
#elif defined(__C30__)
    #if (TCPIP_STACK_DRAM_SIZE < 4*1024)
        #warning "You need at least 4KB of heap space for the TCPIP stack to run properly."
    #endif
#endif


#ifndef TCPIP_STACK_USE_MDD
	#if defined(TCPIP_STACK_USE_HTTP2_SERVER) || defined(TCPIP_STACK_USE_FTP_SERVER)
		#if !defined (TCPIP_STACK_USE_MPFS2)
    		#define TCPIP_STACK_USE_MPFS2
        #endif
	#endif

	#if defined(TCPIP_STACK_USE_SNMPV3_SERVER) && !defined (TCPIP_STACK_USE_SNMP_SERVER)
		#if !defined (TCPIP_STACK_USE_SNMP_SERVER)
    		#define TCPIP_STACK_USE_SNMP_SERVER
        #endif
	#endif

	#if defined(TCPIP_STACK_USE_SNMP_SERVER) //&& !defined(TCPIP_STACK_USE_MPFS) && !defined(TCPIP_STACK_USE_MPFS2)
		#if !defined (TCPIP_STACK_USE_MPFS2)
		    #define TCPIP_STACK_USE_MPFS2
        #endif
	#endif

	#if defined(TCPIP_STACK_USE_SNMP_SERVER) && defined (TCPIP_STACK_USE_SNMPV3_SERVER)
		#if !defined ( TCPIP_STACK_USE_MD5)
    		#define TCPIP_STACK_USE_MD5
        #endif
		#if !defined (TCPIP_STACK_USE_SHA1)
		    #define TCPIP_STACK_USE_SHA1
        #endif
	#endif
#endif

////
// HTTP Check
////
// Include modules required by specific HTTP demos
#if !defined(TCPIP_STACK_USE_HTTP2_SERVER)
	#undef HTTP_APP_USE_EMAIL
	#undef HTTP_APP_USE_MD5
	#undef HTTP_APP_USE_RECONFIG
	#define RESERVED_HTTP_MEMORY 0ul
#endif
#if defined(HTTP_APP_USE_EMAIL)
	#if !defined(TCPIP_STACK_USE_SMTP_CLIENT)
		#error HTTP E-mail Demo requires SMTP_CLIENT and HTTP2
	#endif
#endif
#if defined(HTTP_APP_USE_MD5)
	#if !defined(TCPIP_STACK_USE_MD5)
		#define TCPIP_STACK_USE_MD5
	#endif
#endif

// Can't do MPFS upload without POST or external memory
#if defined(HTTP_MPFS_UPLOAD)
	#if !defined(HTTP_USE_POST) || (!defined(MPFS_USE_EEPROM) && !defined(MPFS_USE_SPI_FLASH))
		#error HTTP_MPFS_UPLOAD requires both HTTP_USE_POST and external storage to be defined 
	#endif
#endif

#if (!defined(HTTP_MAX_CONNECTIONS) || HTTP_MAX_CONNECTIONS <= 0)
    #error Invalid HTTP_MAX_CONNECTIONS value specified.
#endif

// HTTP2 requires 2 MPFS2 handles per connection, plus one spare
#if defined(TCPIP_STACK_USE_HTTP2_SERVER)
	#if MAX_MPFS_HANDLES < ((HTTP_MAX_CONNECTIONS * 2) + 1)
		#error HTTP2 requires 2 MPFS2 file handles per connection, plus one additional.
	#endif
#endif

#if defined(HTTP_SAVE_CONTEXT_IN_PIC_RAM)
	#undef RESERVED_HTTP_MEMORY
	#define RESERVED_HTTP_MEMORY 0ul        // Macro indicating how much RAM to allocate on an ethernet controller to store HTTP state data.
#else
	#undef RESERVED_HTTP_MEMORY
	#define RESERVED_HTTP_MEMORY ((uint32_t)HTTP_MAX_CONNECTIONS * (uint32_t)sizeof(HTTP_CONN))
#endif

////
// MPFS Check
////
// SPI Flash MPFS images must start on a block boundary
#if (defined(TCPIP_STACK_USE_MPFS2)) && \
	defined(MPFS_USE_SPI_FLASH) && ((MPFS_RESERVE_BLOCK & 0x0fff) != 0)
	#error MPFS_RESERVE_BLOCK must be a multiple of 4096 for SPI Flash storage
#endif

////
// ICMP Check
////
// When IP Gleaning is enabled, ICMP must also be enabled.
#if defined(TCPIP_STACK_USE_IP_GLEANING)
    #if !defined(TCPIP_STACK_USE_ICMP_SERVER)
        #define TCPIP_STACK_USE_ICMP_SERVER
    #endif
#endif

////
// DNS Check
////
// Make sure that the DNS client is enabled if services require it
#if defined(TCPIP_STACK_USE_GENERIC_TCP_CLIENT_EXAMPLE) || \
	defined(TCPIP_STACK_USE_SNTP_CLIENT) || \
	defined(TCPIP_STACK_USE_DYNAMICDNS_CLIENT) || \
	defined(TCPIP_STACK_USE_SMTP_CLIENT)
    #if !defined(TCPIP_STACK_USE_DNS)
        #define TCPIP_STACK_USE_DNS
    #endif
#endif

////
// TCPIP_STACK_CLEINT_MODE Check
////
// Make sure that TCPIP_STACK_CLIENT_MODE is defined if a service depends on it
#if defined(TCPIP_STACK_USE_FTP_SERVER) || \
	defined(TCPIP_STACK_USE_SNMP_SERVER) || \
	defined(TCPIP_STACK_USE_DNS) || \
	defined(TCPIP_STACK_USE_GENERIC_TCP_CLIENT_EXAMPLE) || \
	defined(TCPIP_STACK_USE_TFTP_CLIENT) || \
	defined(TCPIP_STACK_USE_SMTP_CLIENT) || \
	defined(TCPIP_STACK_USE_ICMP_CLIENT) || \
	defined(TCPIP_STACK_USE_DYNAMICDNS_CLIENT) || \
	defined(TCPIP_STACK_USE_SNTP_CLIENT) || \
	defined(TCPIP_STACK_USE_BERKELEY_API) || \
	defined(TCPIP_STACK_USE_SSL_CLIENT) || \
	defined(TCPIP_STACK_USE_AUTO_IP)
	#if !defined(TCPIP_STACK_CLIENT_MODE)
		#define TCPIP_STACK_CLIENT_MODE
	#endif
#endif

////
// RSA Check
////
// When using SSL server, enable RSA decryption
#if defined(TCPIP_STACK_USE_SSL_SERVER)
	#if !defined TCPIP_STACK_USE_RSA_DECRYPT
    	#define TCPIP_STACK_USE_RSA_DECRYPT
    #endif
	#if !defined (TCPIP_STACK_USE_SSL)
    	#define TCPIP_STACK_USE_SSL
    #endif
#endif

// When using SSL client, enable RSA encryption
#if defined(TCPIP_STACK_USE_SSL_CLIENT)
	#if !defined (TCPIP_STACK_USE_RSA_ENCRYPT)
    	#define TCPIP_STACK_USE_RSA_ENCRYPT
    #endif
	#if !defined (TCPIP_STACK_USE_SSL)
    	#define TCPIP_STACK_USE_SSL
    #endif
#endif

// When using either RSA operation, include the RSA module
#if defined(TCPIP_STACK_USE_RSA_ENCRYPT) || defined(TCPIP_STACK_USE_RSA_DECRYPT)
	#if !defined (TCPIP_STACK_USE_RSA)
    	#define TCPIP_STACK_USE_RSA
    #endif
	#if !defined (TCPIP_STACK_USE_BIGINT)
    	#define TCPIP_STACK_USE_BIGINT
    #endif
#endif


#if defined(TCPIP_STACK_USE_RSA_ENCRYPT)
	#define BI_USE_CONSTRUCTOR
	#define BI_USE_ZERO
	#define BI_USE_MOD
	#define BI_USE_COMPARE
	#define BI_USE_MAG_DIFF
	#define BI_USE_MAG
	#define BI_USE_MSB
	#define BI_USE_MULTIPLY
	#define BI_USE_SQUARE
	#define BI_USE_COPY
#endif

#if defined(TCPIP_STACK_USE_RSA_DECRYPT)
	#define BI_USE_CONSTRUCTOR
	#define BI_USE_ZERO
	#define BI_USE_COMPARE
	#define BI_USE_MAG
	#define BI_USE_MSB
	#define BI_USE_MULTIPLY
	#define BI_USE_SQUARE
	#define BI_USE_ADD
	#define BI_USE_SUBTRACT
	#define BI_USE_COPY

    #define BI_USE_MAG_DIFF
    #define BI_USE_MOD
#endif


////
// SSL Check
//// 
#if defined(TCPIP_STACK_USE_SSL_SERVER) || defined(TCPIP_STACK_USE_SSL_CLIENT)
    #if (!defined(SSL_MAX_CONNECTIONS) || SSL_MAX_CONNECTIONS <= 0 )
        #error Invalid SSL_MAX_CONNECTIONS value specified.
    #endif
    #if (!defined(SSL_MAX_SESSIONS) || SSL_MAX_SESSIONS <= 0)
        #error Invalid SSL_MAX_SESSIONS value specified.
    #endif
    #if (!defined(SSL_MAX_HASHES) || SSL_MAX_HASHES <= 0 )
        #error Invalid SSL_MAX_HASHES value specified.
    #elif (SSL_MAX_HASHES > 16)
        #undef SSL_MAX_HASHES
        #define SSL_MAX_HASHES 16
    #endif
#else
    #undef SSL_MAX_CONNECTIONS
    #undef SSL_MAX_SESSIONS
    #define SSL_MAX_CONNECTIONS  0
    #define SSL_MAX_SESSIONS  0
#endif // TCPIP_STACK_USE_SSL_SERVER || TCPIP_STACK_USE_SSL_CLIENT

// If using SSL (either), include the rest of the support modules
#if defined(TCPIP_STACK_USE_SSL)
	#if !defined (TCPIP_STACK_USE_ARCFOUR)
    	#define TCPIP_STACK_USE_ARCFOUR
    #endif
	#if !defined (TCPIP_STACK_USE_MD5)
    	#define TCPIP_STACK_USE_MD5
    #endif
	#if !defined (TCPIP_STACK_USE_SHA1)
    	#define TCPIP_STACK_USE_SHA1
    #endif
	#if !defined (TCPIP_STACK_USE_RANDOM)
    	#define TCPIP_STACK_USE_RANDOM
    #endif
#endif


#if !defined(TCPIP_STACK_USE_SSL)
	#undef RESERVED_SSL_MEMORY
	#define RESERVED_SSL_MEMORY 0ul
#endif



////
// FTP Check
////
#if defined(TCPIP_STACK_USE_FTP_SERVER)
    #if (!defined(FTP_MAX_CONNECTIONS) || FTP_MAX_CONNECTIONS <= 0)
        #error Invalid FTP_MAX_CONNECTIONS value specified.
    #endif
#else
    #undef  FTP_MAX_CONNECTIONS
    #define FTP_MAX_CONNECTIONS  0
#endif // TCPIP_STACK_USE_FTP_SERVER
// FTP is not supported in MPFS2 or when MPFS is stored in internal program 
// memory (instead of external EEPROM).
#if ( (!defined(MPFS_USE_EEPROM) && !defined(MPFS_USE_SPI_FLASH)) || defined(TCPIP_STACK_USE_MPFS2) ) && defined(TCPIP_STACK_USE_FTP)
	#error FTP server is not supported with HTTP2 / MPFS2, or with internal Flash memory storage
#endif

////
// SMTP Check
////
#if defined(TCPIP_STACK_USE_SMTP_CLIENT)
    #if (!defined(MAX_SMTP_CONNECTIONS) || MAX_SMTP_CONNECTIONS <= 0)
        #error Invalid MAX_SMTP_CONNECTIONS value specified.
    #endif
#else
    #undef  MAX_SMTP_CONNECTIONS
    #define MAX_SMTP_CONNECTIONS  0
#endif // TCPIP_STACK_USE_SMTP_CLIENT

////
// TCP Check
////
#if defined(TCPIP_STACK_USE_GENERIC_TCP_CLIENT_EXAMPLE)
    #if (!defined(TCP_MAX_CLIENT_CONNECTIONS) || TCP_MAX_CLIENT_CONNECTIONS <= 0)
        #error Invalid TCP_MAX_CLIENT_CONNECTIONS value specified.
    #endif
#else
    #undef  TCP_MAX_CLIENT_CONNECTIONS
    #define TCP_MAX_CLIENT_CONNECTIONS  0
#endif // TCPIP_STACK_USE_GENERIC_TCP_CLIENT_EXAMPLE

#if defined(TCPIP_STACK_USE_GENERIC_TCP_SERVER_EXAMPLE)
    #if (!defined(TCP_MAX_SERVER_CONNECTIONS) || TCP_MAX_SERVER_CONNECTIONS <= 0)
        #error Invalid TCP_MAX_SERVER_CONNECTIONS value specified.
    #endif
#else
    #undef  TCP_MAX_SERVER_CONNECTIONS
    #define TCP_MAX_SERVER_CONNECTIONS  0
#endif // TCPIP_STACK_USE_GENERIC_TCP_SERVER_EXAMPLE

// If TCP is not enabled, clear all memory allocations
#if !defined(TCPIP_STACK_USE_TCP)
	#undef TCP_ETH_RAM_SIZE
	#undef TCP_PIC_RAM_SIZE
	#undef TCP_SPI_RAM_SIZE
	#define TCP_ETH_RAM_SIZE 0u
	#define TCP_PIC_RAM_SIZE 0u
	#define TCP_SPI_RAM_SIZE 0u
#endif

// Make sure that TCPIP_STACK_USE_TCP is defined if a service 
// depends on it
#if defined(TCPIP_STACK_USE_HTTP2_SERVER) || \
	defined(TCPIP_STACK_USE_FTP_SERVER) || \
	defined(TCPIP_STACK_USE_TELNET_SERVER) || \
	defined(TCPIP_STACK_USE_GENERIC_TCP_CLIENT_EXAMPLE) || \
	defined(TCPIP_STACK_USE_GENERIC_TCP_SERVER_EXAMPLE) || \
	defined(TCPIP_STACK_USE_SMTP_CLIENT) || \
	defined(TCPIP_STACK_USE_TCP_PERFORMANCE_TEST) || \
	defined(TCPIP_STACK_USE_DYNAMICDNS_CLIENT) || \
	defined(TCPIP_STACK_USE_BERKELEY_API) || \
	defined(TCPIP_STACK_USE_SSL_CLIENT) || \
	defined(TCPIP_STACK_USE_SSL_SERVER)
    #if !defined(TCPIP_STACK_USE_TCP)
        #define TCPIP_STACK_USE_TCP
    #endif
#endif

////
// TELNET Check
////
#if defined(TCPIP_STACK_USE_TELNET_SERVER)
    #if (!defined(MAX_TELNET_CONNECTIONS) || MAX_TELNET_CONNECTIONS <= 0)
        #error Invalid MAX_TELNET_CONNECTIONS value specified.
    #endif
#else
    #undef  MAX_TELNET_CONNECTIONS
    #define MAX_TELNET_CONNECTIONS  0
#endif // TCPIP_STACK_USE_TELNET_SERVER
// TELNET Check
#if defined(TCPIP_STACK_USE_TELNET_SERVER)
	#include "tcpip/telnet.h"
#endif

////
// TCP Performance Test Check
////
#if defined(TCPIP_STACK_USE_TCP_PERFORMANCE_TEST)
    #if (!defined(TCP_MAX_PERFORMANCE_CONNECTIONS) || TCP_MAX_PERFORMANCE_CONNECTIONS <= 0)
        #error Invalid TCP_MAX_PERFORMANCE_CONNECTIONS value specified.
    #endif
#else
    #undef TCP_MAX_PERFORMANCE_CONNECTIONS
    #define TCP_MAX_PERFORMANCE_CONNECTIONS  0
#endif // TCPIP_STACK_USE_TCP_PERFORMANCE_TEST

////
// Berkely API Check
////
#if defined(TCPIP_STACK_USE_BERKELEY_API)
    #if (!defined(MAX_BSD_SERVER_CONNECTIONS) || MAX_BSD_SERVER_CONNECTIONS <= 0)
        #error Invalid MAX_BSD_SERVER_CONNECTIONS value specified.
    #endif
    #if (!defined(MAX_BSD_CLIENT_CONNECTIONS) || MAX_BSD_CLIENT_CONNECTIONS <= 0)
        #error Invalid MAX_BSD_CLIENT_CONNECTIONS value specified.
    #endif
#else
    #undef  MAX_BSD_SERVER_CONNECTIONS
    #undef  MAX_BSD_CLIENT_CONNECTIONS
    #define MAX_BSD_SERVER_CONNECTIONS  0
    #define MAX_BSD_CLIENT_CONNECTIONS  0
#endif // TCPIP_STACK_USE_BERKELEY_API


////
// default number of TCP sockets calculation
////
#if defined(TCP_DYNAMIC_CONFIGURATION)
    #if !defined(TCP_MAX_SOCKETS)
        // define the number of TCP_MAX_SOCKETS based on the selected protocols
        #define TCP_MAX_SOCKETS	(16)
        #if 0
            #define TCP_MAX_SOCKETS	(HTTP_MAX_CONNECTIONS + SSL_MAX_CONNECTIONS + \
			    	FTP_MAX_CONNECTIONS + MAX_SMTP_CONNECTIONS + TCP_MAX_CLIENT_CONNECTIONS + \
				    TCP_MAX_SERVER_CONNECTIONS + MAX_TELNET_CONNECTIONS + TCP_MAX_PERFORMANCE_CONNECTIONS + \
				    MAX_BSD_SERVER_CONNECTIONS + MAX_BSD_CLIENT_CONNECTIONS)
        #endif
    #elif ( TCP_MAX_SOCKETS <= 0 )
        #error Invalid TCP_MAX_SOCKETS value specified.
    #endif  // !defined(TCP_MAX_SOCKETS)
#else
    #undef  TCP_MAX_SOCKETS
    #define TCP_MAX_SOCKETS	(sizeof(TCPSocketInitializer)/sizeof(TCPSocketInitializer[0]))
#endif  // defined(TCP_DYNAMIC_CONFIGURATION)

////
// UDP Check
////
#if ( !defined(UDP_MAX_SOCKETS) ||  UDP_MAX_SOCKETS <= 0 )
    #error Invalid UDP_MAX_SOCKETS value specified.
#endif  // !defined(UDP_MAX_SOCKETS)

// Make sure that TCPIP_STACK_USE_UDP is defined if a service 
// depends on it
#if defined(TCPIP_STACK_USE_DHCP_CLIENT) || \
	defined(TCPIP_STACK_USE_DHCP_SERVER) || \
	defined(TCPIP_STACK_USE_DNS) || \
	defined(TCPIP_STACK_USE_NBNS) || \
	defined(TCPIP_STACK_USE_SNMP_SERVER) || \
	defined(TCPIP_STACK_USE_TFTP_CLIENT) || \
	defined(TCPIP_STACK_USE_ANNOUNCE) || \
	defined(TCPIP_STACK_USE_UDP_PERFORMANCE_TEST) || \
	defined(TCPIP_STACK_USE_SNTP_CLIENT) || \
	defined(TCPIP_STACK_USE_BERKELEY_API)
    #if !defined(TCPIP_STACK_USE_UDP)
        #define TCPIP_STACK_USE_UDP
    #endif
#endif

////
// LCD Check
////
// Enable the LCD if configured in the hardware profile
#if defined(LCD_DATA_IO) || defined(LCD_DATA0_IO)
	#if !defined (SYS_OUT_ENABLE)
	    #define SYS_OUT_ENABLE
    #endif
#endif


// Storage check
#if defined(TCPIP_STACK_USE_STORAGE)  && (defined(SPIFLASH_CS_TRIS) || defined(EEPROM_CS_TRIS))
    #define _TCPIP_STACK_STORAGE_ENABLED
    #if (TCPIP_STORAGE_STACK_CHECK_BUILD_VERSION)
        #define _TCPIP_STACK_CHECK_STORAGE_VERSION
        #define _TCPIP_STORAGE_USER_LABEL_SIZE  2
    #else 
        #define _TCPIP_STORAGE_USER_LABEL_SIZE  TCPIP_STORAGE_USER_LABEL_SIZE   // use default
    #endif
#endif  // defined(TCPIP_STACK_USE_STORAGE)  && (defined(SPIFLASH_CS_TRIS) || defined(EEPROM_CS_TRIS))

/*****************************************************************************
The number of ticks per second to generate a stack tick.
Used by the stack state machine to check for the MAC link, etc
Note: the System Tick resolution in system_profile.h (SYS_TICKS_PER_SECOND) has to 
be fine enough to allow for this stack tick granularity.  
 *****************************************************************************/
#if !defined(TCPIP_STACK_TICKS_PER_SECOND)
    #define TCPIP_STACK_TICKS_PER_SECOND        (10)        // 100 ms default tick 
#endif

#include "tcpip_helpers_private.h"

#endif  //  __TCPIP_CONFIG_PRIVATE_H_
