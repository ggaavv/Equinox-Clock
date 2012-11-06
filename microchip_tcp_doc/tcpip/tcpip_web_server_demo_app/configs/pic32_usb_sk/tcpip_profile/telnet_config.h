/*******************************************************************************
  Telnet Configuration file

  Summary:
    Configuration file
    
  Description:
    This file contains the Telnet module configuration options
    
*******************************************************************************/

/*******************************************************************************
FileName:   telnet_config.h
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

#ifndef _TELNET_CONFIG_H_
#define _TELNET_CONFIG_H_


// Set up configuration parameter defaults if not overridden in 
// TCPIPConfig.h
#if !defined(TELNET_PORT)
	// Unsecured Telnet port
	#define TELNET_PORT			23
#endif

#if !defined(TELNETS_PORT)	
	// SSL Secured Telnet port (ignored if TCPIP_STACK_USE_SSL_SERVER is undefined)
	#define TELNETS_PORT		992	
#endif

// Force all connecting clients to be SSL secured and connected via
// TELNETS_PORT.  Connections on port TELNET_PORT will be ignored.  If
// TCPIP_STACK_USE_SSL_SERVER is undefined, this entire setting is ignored
// (server will accept unsecured connections on TELNET_PORT and won't even
// listen on TELNETS_PORT).
//#define TELNET_REJECT_UNSECURED

#if !defined(MAX_TELNET_CONNECTIONS)
	// Maximum number of Telnet connections
	#define MAX_TELNET_CONNECTIONS	(3u)
#endif

#if !defined(TELNET_USERNAME)
	// Default Telnet user name
	#define TELNET_USERNAME		"admin"
#endif

#if !defined(TELNET_PASSWORD)
	// Default Telnet password
	#define TELNET_PASSWORD		"microchip"
#endif


#endif  // _TELNET_CONFIG_H_
