/*******************************************************************************
  Dynamic Host Configuration Protocol (DCHP) Configuration file

  Summary:
    DCHP configuration file
    
  Description:
    This file contains the DCHP module configuration options
    
*******************************************************************************/

/*******************************************************************************
FileName:   dhcp_config.h
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

#ifndef _DCHP_CONFIG_H_
#define _DCHP_CONFIG_H_


// Defines how long to wait before a DHCP request times out, seconds
#define DHCP_TIMEOUT				(2ul)

// UDP client port for DHCP Client transactions
#define DHCP_CLIENT_PORT                (68u)
// UDP listening port for DHCP Server messages
#define DHCP_SERVER_PORT                (67u)

// enable/disable the DHCP client at stack start up
#define DHCP_CLIENT_ENABLED        1


// DHCP layer configuration/initialization
typedef struct
{
    bool     dhcpEnable;
}DHCP_MODULE_CONFIG;


// This is a template of how the DHCP module should be initialized and
// the parameters that it needs.
static const DHCP_MODULE_CONFIG dhcpConfigData = 
{
    DHCP_CLIENT_ENABLED,
};




#endif  // _DCHP_CONFIG_H_
