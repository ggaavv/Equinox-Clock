/*******************************************************************************
  HyperText Transfer Protocol (HTTP2) Configuration file

  Summary:
    HTTP2 configuration file
    
  Description:
    This file contains the HTTP2 module configuration options
    
*******************************************************************************/

/*******************************************************************************
FileName:   http2_config.h
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

#ifndef _HTTP2_CONFIG_H_
#define _HTTP2_CONFIG_H_


// Set to length of longest string above
#define HTTP_MAX_HEADER_LEN		(15u)

// Max lifetime (sec) of static responses as string
#define HTTP_CACHE_LEN			("600")	

// Max time (sec) to await more data before timing out and disconnecting the socket
#define HTTP_TIMEOUT			(45u)

// Listening port for HTTP server
// Define the listening port for the HTTP server
#if !defined(HTTP_PORT)
#define HTTP_PORT               	(80u)

// Maximum numbers of simultaneous supported HTTP connections.
#define HTTP_MAX_CONNECTIONS		(4)

// Optional setting to use PIC RAM instead of Ethernet/Wi-Fi RAM for
// storing HTTP Connection Context variables (HTTP_CONN structure for each 
// HTTP connection).  Undefining this macro results in the Ethernet/Wi-Fi 
// RAM being used (minimum PIC RAM usage, lower performance).  Defining 
// this macro results in PIC RAM getting used (higher performance, but uses 
// PIC RAM).
// The performance increase of having this option defined is only apparent when 
// the HTTP server is servicing multiple connections simultaneously.
#define HTTP_SAVE_CONTEXT_IN_PIC_RAM

// Indicate what file to serve when no specific one is requested
#define HTTP_DEFAULT_FILE		"index.htm"
#define HTTPS_DEFAULT_FILE		"index.htm"

// For buffer overrun protection.
// Set to longest length of above two strings.
#define HTTP_DEFAULT_LEN		(10u)		

// Configure MPFS over HTTP updating
// Comment this line to disable updating via HTTP
#define HTTP_MPFS_UPLOAD		"mpfsupload"

// Require password for MPFS uploads
// Certain firewall and router combinations cause the MPFS2 Utility to fail
// when uploading.  If this happens, comment out this definition.
//#define HTTP_MPFS_UPLOAD_REQUIRES_AUTH	

// Define which HTTP modules to use
// If not using a specific module, comment it to save resources
// Enable POST support
#define HTTP_USE_POST					

// Enable cookie support
#define HTTP_USE_COOKIES

// Enable basic authentication support
#define HTTP_USE_AUTHENTICATION

// Uncomment to require SSL before requesting a password
//#define HTTP_NO_AUTH_WITHOUT_SSL

// Files beginning with this character will only be served over HTTPS
// Set to 0x00 to require for all files
// Set to 0xff to require for no files
#define HTTP_SSL_ONLY_CHAR		(0xFF)

// Define the listening port for the HTTPS server (if TCPIP_STACK_USE_SSL_SERVER is enabled)
#define HTTPS_PORT				(443u)

// Define the listening port for the HTTP server
#define HTTP_PORT               (80u)

// Use the NetConfig web page in the Demo App (~2.5kb ROM, ~0b RAM)
#define HTTP_APP_USE_RECONFIG

// Use the MD5 Demo web page (~5kb ROM, ~160b RAM)
#define HTTP_APP_USE_MD5

// Use the e-mail demo web page
#define HTTP_APP_USE_EMAIL

#endif

// Listening port for HTTPS server (when SSL enabled)
#if !defined(HTTPS_PORT)
#define HTTPS_PORT			(443u)	
#endif

// Define the maximum data length for reading cookie and GET/POST arguments (bytes)
#if !defined(HTTP_MAX_DATA_LEN)
#define HTTP_MAX_DATA_LEN		(100u)
#endif

// Define the minimum number of bytes free in the TX FIFO before executing callbacks
#if !defined(HTTP_MIN_CALLBACK_FREE)
#define HTTP_MIN_CALLBACK_FREE	(16u)
#endif

// Authentication requires Base64 decoding
// Enable basic authentication support
#if defined(HTTP_USE_AUTHENTICATION)
#ifndef TCPIP_STACK_USE_BASE64_DECODE
#define TCPIP_STACK_USE_BASE64_DECODE
#endif // TCPIP_STACK_USE_BASE64_DECODE
#endif // HTTP_USE_AUTHENTICATION


#endif  // _HTTP2_CONFIG_H_
