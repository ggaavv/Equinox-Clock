/*******************************************************************************
Microchip TCP/IP Stack Demo Application Configuration Header

  Summary:
    
  Description:

*******************************************************************************/

/*******************************************************************************
FileName: 	tcpip_config.h
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

#ifndef __TCPIP_CONFIG_H_
#define __TCPIP_CONFIG_H_

// =======================================================================
//   Stack Configuration/Build Options
// =======================================================================

// TCPIP Stack Module Selection
//   Uncomment or comment the following lines to enable or
//   disabled the following high-level application modules.

#define TCPIP_STACK_USE_ICMP_SERVER			// Ping query and response capability
#define TCPIP_STACK_USE_HTTP2_SERVER			// New HTTP server with POST, Cookies, Authentication, etc.
//#define TCPIP_STACK_USE_SSL_SERVER			// SSL server socket support (Requires SW300052)
//#define TCPIP_STACK_USE_SSL_CLIENT			// SSL client socket support (Requires SW300052)
#define TCPIP_STACK_USE_DHCP_CLIENT			// Dynamic Host Configuration Protocol client for obtaining IP address and other parameters
#define TCPIP_STACK_USE_SMTP_CLIENT			// Simple Mail Transfer Protocol for sending email
#define TCPIP_STACK_USE_GENERIC_TCP_CLIENT_EXAMPLE	// HTTP Client example in GenericTCPClient.c
#define TCPIP_STACK_USE_GENERIC_TCP_SERVER_EXAMPLE	// ToUpper server example in GenericTCPServer.c
#define TCPIP_STACK_USE_TELNET_SERVER			// Telnet server
#define TCPIP_STACK_USE_ANNOUNCE				// Microchip Embedded Ethernet Device Discoverer server/client
#define TCPIP_STACK_USE_DNS					// Domain Name Service Client for resolving hostname strings to IP addresses
#define TCPIP_STACK_USE_NBNS					// NetBIOS Name Service Server for repsonding to NBNS hostname broadcast queries
//#define TCPIP_STACK_USE_REBOOT_SERVER			// Module for resetting this PIC remotely.  Primarily useful for a Bootloader.
#define TCPIP_STACK_USE_SNTP_CLIENT			// Simple Network Time Protocol for obtaining current date/time from Internet
//#define TCPIP_STACK_USE_UDP_PERFORMANCE_TEST	// Module for testing UDP TX performance characteristics.  NOTE: Enabling this will cause a huge amount of UDP broadcast packets to flood your network on the discard port.  Use care when enabling this on production networks, especially with VPNs (could tunnel broadcast traffic across a limited bandwidth connection).
//#define TCPIP_STACK_USE_TCP_PERFORMANCE_TEST	// Module for testing TCP TX performance characteristics
#define TCPIP_STACK_USE_DYNAMICDNS_CLIENT		// Dynamic DNS client updater module
#define TCPIP_STACK_USE_BERKELEY_API			// Berekely Sockets APIs are available
#define TCPIP_STACK_USE_IPV6                    // enable IPv6 functionality
#define TCPIP_STACK_USE_TCP                     // Enable the TCP module
#define TCPIP_STACK_USE_UDP                     // Enable the UDP module
#define TCPIP_STACK_USE_MPFS2				    // Enable data storage options
//#define TCPIP_STACK_USE_STORAGE			    // Enable the TCPIP storage service
                                                // needs hardware support: EEPROM/SPIFlash
#define TCPIP_STACK_CLIENT_MODE				    // Enable the CLIENT mode. 
							                    // In CLIENT mode, some functions specific to client operation
							                    // are enabled.


// =======================================================================
//   NOT PORTED/TESTED. DO NOT ENABLE!
// =======================================================================

//#define TCPIP_STACK_USE_IP_GLEANING
//#define TCPIP_STACK_USE_ICMP_CLIENT			// Ping transmission capability
//#define TCPIP_STACK_USE_AUTO_IP               // Dynamic link-layer IP address automatic configuration protocol
//#define TCPIP_STACK_USE_DHCP_SERVER			// Single host DHCP server
//#define TCPIP_STACK_USE_FTP_SERVER			// File Transfer Protocol (old)
//#define TCPIP_STACK_USE_SNMP_SERVER			// Simple Network Management Protocol v2C Community Agent
//#define TCPIP_STACK_USE_TFTP_CLIENT			// Trivial File Transfer Protocol client
//#define TCPIP_STACK_USE_DNS_SERVER			// Domain Name Service Server for redirection to the local device
//#define TCPIP_STACK_USE_ZEROCONF_LINK_LOCAL		// Zeroconf IPv4 Link-Local Addressing
//#define TCPIP_STACK_USE_ZEROCONF_MDNS_SD		// Zeroconf mDNS and mDNS service discovery


// =======================================================================
//   Dynamic memory allocation Options
// =======================================================================
// This is the total amount of dynamic memory that the TCPIP stack will use:
// for MAC buffers, for TCP and UDP sockets, etc.
#define TCPIP_STACK_DRAM_SIZE       			(12288) 


// =======================================================================
//   Event Notifications Options
//   ENC624J600, ENC24J60 and MRF24W do not support (yet) event notifications
// =======================================================================
//#define TCPIP_STACK_USE_EVENT_NOTIFICATION

    // The default interrupt priority to use for the TCPIP interrupts
	#define	TCPIP_EVENT_IPL			5
	#define	TCPIP_EVENT_SIPL		1

// =======================================================================
//   TCPIP network configuration file
// =======================================================================
#include "network_profile.h"



// =======================================================================
//   TCPIP modules configuration files
// =======================================================================
#include "tcpip_modules_config.h"

#endif  // __TCPIP_CONFIG_H_


