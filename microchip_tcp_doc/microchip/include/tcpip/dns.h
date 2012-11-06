/*******************************************************************************
  Domain Name System (DNS) client Header file

  Summary:
    DNS definitions and interface file
    
  Description:
    This source file contains the DNS client module API
*******************************************************************************/

/*******************************************************************************
FileName:  dns.h 
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

#ifndef __DNS_H
#define __DNS_H

typedef enum
{
    
    DNS_TYPE_A      = 1,		// Constant for record type in DNSResolve.  Indicates an A (standard address) record.
    DNS_TYPE_MX     = 15,		// Constant for record type in DNSResolve.  Indicates an MX (mail exchanger) record.
    DNS_TYPE_AAAA   = 28u       // Constant for record type in DNSResolve.  Indicates a quad-A (IPv6 address) address record.
}DNS_RESOLVE_TYPE;


typedef enum
{
    // success codes
    DNS_RES_OK                  = 0,    // operation succeeded
    DNS_RES_PENDING,                    // operation is ongoing


    // failure codes
    DNS_RES_NO_ENTRY            = -1,   // no such entry exists
    DNS_RES_CACHE_FULL          = -2,   // the cache is full and no entry could be
    DNS_RES_OPEN_TMO            = -3,   // DNS client couldn't get a socket
    DNS_RES_SERVER_TMO          = -4,   // DNS server response tmo

    // backward compatibility results
    DNS_RES_NO_INTERFACE        = -10,   // an active/requested interface could not be found
    DNS_RES_BUSY                = -11,   // module is in use by other task; retry later
    DNS_RES_ARP_TMO             = -12,   // ARP tmo

}DNS_RESULT;


DNS_RESULT  DNSBeginUsage(TCPIP_NET_HANDLE netH);
DNS_RESULT  DNSResolve(const char* HostName, DNS_RESOLVE_TYPE Type);
DNS_RESULT  DNSIsResolved(const char* HostName, void* HostIP);
DNS_RESULT  DNSEndUsage(TCPIP_NET_HANDLE netH);



#endif
