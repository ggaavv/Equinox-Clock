/*******************************************************************************
  Network Time Protocol (SNTP) Configuration file

  Summary:
    SNTP configuration file
    
  Description:
    This file contains the SNTP module configuration options
    
*******************************************************************************/

/*******************************************************************************
FileName:   sntp_config.h
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

#ifndef _SNTP_CONFIG_H_
#define _SNTP_CONFIG_H_


// for multi-homed hosts, the default SNTP interface
#define MY_DEFAULT_SNTP_IF      "PIC32INT"

// Port for contacting NTP servers
#define NTP_SERVER_PORT			(123ul)

// Reference Epoch to use.  (default: 01-Jan-1970 00:00:00)
#define NTP_EPOCH 				(86400ul * (365ul * 70ul + 17ul))

// Defines how long to wait before assuming the query has failed, seconds
#define NTP_REPLY_TIMEOUT		(6ul)

// These are normally available network time servers.
// The actual IP returned from the pool will vary every
// minute so as to spread the load around stratum 1 timeservers.
// For best accuracy and network overhead you should locate the 
// pool server closest to your geography, but it will still work
// if you use the global pool.ntp.org address or choose the wrong 
// one or ship your embedded device to another geography.
#define NTP_SERVER	"pool.ntp.org"
//#define NTP_SERVER	"europe.pool.ntp.org"
//#define NTP_SERVER	"asia.pool.ntp.org"
//#define NTP_SERVER	"oceania.pool.ntp.org"
//#define NTP_SERVER	"north-america.pool.ntp.org"
//#define NTP_SERVER	"south-america.pool.ntp.org"
//#define NTP_SERVER	"africa.pool.ntp.org"

// Defines how frequently to resynchronize the date/time, seconds (default: 10 minutes)
#define NTP_QUERY_INTERVAL		(10ul*60ul)

// Defines how long to wait to retry an update after a failure, seconds.
// Updates may take up to 6 seconds to fail, so this 14 second delay is actually only an 8-second retry.
#define NTP_FAST_QUERY_INTERVAL	(14ul)


typedef struct
{
}SNTP_MODULE_CONFIG;




#endif // _SNTP_CONFIG_H_
