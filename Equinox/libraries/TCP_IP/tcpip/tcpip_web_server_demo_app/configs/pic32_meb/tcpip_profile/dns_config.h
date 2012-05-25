/*******************************************************************************
  Domain Name Service (CNS) Configuration file

  Summary:
    DNS configuration file
    
  Description:
    This file contains the DNS module configuration options
    
*******************************************************************************/

/*******************************************************************************
FileName:   dns_config.h
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

#ifndef _DNS_CONFIG_H_
#define _DNS_CONFIG_H_



// Default port for DNS resolutions
#define DNS_CLIENT_PORT			53

// When the DNS Client performs an ARP request for the DNS Server
// this is the elapsed time after which a ARP resolution is considered
// to have timed out
// In seconds
#define DNS_CLIENT_ARP_TMO		(1)		

// The DNS Client performs an UDP socket open request
// to communicate with the DNS Server
// This is the elapsed time after which an open request is considered
// to have timed out
// In seconds
#define DNS_CLIENT_OPEN_TMO		(1)

// When the DNS Client connected to the DNS Server
// this is the elapsed time after which an the communication is considered
// to have timed failed if there was no reply from the server
// In seconds
#define DNS_CLIENT_SERVER_TMO		(1)

// DNS Client task processing rate, in milli-seconds
// the DNS Client module will process a timer event with this rate
// for processing its own state machine, etc.
//
#define DNS_CLIENT_TASK_PROCESS_RATE	(500)

// DNS Client version
// Depending on the version of the DNS implementation
// the DNS task process will be performed differently
#define DNS_CLIENT_VERSION_NO		1           

typedef struct
{
}DNS_CLIENT_MODULE_GONFIG;


#endif  // _DNS_CONFIG_H_



