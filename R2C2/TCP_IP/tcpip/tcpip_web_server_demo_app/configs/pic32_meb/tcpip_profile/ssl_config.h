/*******************************************************************************
  Secure Sockets Layer (SSL) Configuration file

  Summary:
    SSL configuration file
    
  Description:
    This file contains the SSL module configuration options
    
*******************************************************************************/

/*******************************************************************************
FileName:   ssl_config.h
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

#ifndef _SSL_CONFIG_H_
#define _SSL_CONFIG_H_


// Client configuration options

// SSL version number
#define SSL_VERSION (0x0300u) // Moved from Microchip\Include\TCPIP Stack\SSL.h

// SSL version number (high byte)
#define SSL_VERSION_HI (0x03u)

// SSL version number (low byte)
#define SSL_VERSION_LO (0x00u)

// Identifier for invalid SSL allocations
#define SSL_INVALID_ID (0xFFu)
	
// Minimum lifetime for SSL Sessions
// Sessions cannot be reallocated until this much time (seconds) has elapsed
#define SSL_MIN_SESSION_LIFETIME (1ul)
	
// Lifetime extension for RSA operations
// Sessions lifetime is extended by this amount (seconds) when an RSA calculation is made
#define SSL_RSA_LIFETIME_EXTENSION (8ul)

// Maximum number of simultaneous connections via SSL
#define SSL_MAX_CONNECTIONS	(8)

// Max number of cached SSL sessions
#define SSL_MAX_SESSIONS	(8)

// SSL on all interfaces in a multi-homed host
// set to 0 to work only on one interface
#define SSL_MULTIPLE_INTERFACES 1

// if 1, the SSL context is saved in regular PIC RAM
// otherwise (0) it will be saved in MAC RAM
// use 0 on smaller parts, with less RAM
// so that the SSL context data can be saved externally,
// on MAC RAM. Slower.
#define SSL_SAVE_CONTEXT_IN_PIC_RAM 1   	

// Max # of SSL buffers (2 per socket)
#define SSL_MAX_BUFFERS		(16ul)	

// Max # of SSL hashes  (2 per, plus 1 to avoid deadlock)
#define SSL_MAX_HASHES		(16ul)	

#if SSL_MAX_HASHES > 16
	#error "SSL_MAX_HASHES may not exceed 16"
#endif

// Bits in SSL RSA key.  This parameter is used for SSL sever
// connections only.  The only valid value is 512 bits (768 and 1024
// bits do not work at this time).  Note, however, that SSL client
// operations do currently work up to 1024 bit RSA key length.
#define SSL_RSA_KEY_SIZE	    (1024ul)
#define SSL_RSA_CLIENT_SIZE     (1024ul)    

// Size of Encryption Buffer (must be larger than key size)
// This will represent the maximum key size in use by an external server that this client can connect to.

#if SSL_RSA_CLIENT_SIZE < SSL_RSA_KEY_SIZE
	#error "SSL_RSA_CLIENT_SIZE must be >= SSL_RSA_KEY_SIZE"
#endif


// SSL layer configuration/initialization
typedef struct
{
} SSL_MODULE_CONFIG;


#endif  // _SSL_CONFIG_H_
