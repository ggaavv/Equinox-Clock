/*******************************************************************************
  Simple Network Managment Protocol (SNMPv3) Configuration file

  Summary:
    SNMPv3 configuration file
    
  Description:
    This file contains the SNMPv3 module configuration options
    
*******************************************************************************/

/*******************************************************************************
FileName:   snmpv3_config.h
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

#ifndef _SNMPv3_CONFIG_H_
#define _SNMPv3_CONFIG_H_

// Client configuration options

#define SNMPV3_USM_MAX_USER	3

// Sets the max length for the user security name
#define USER_SECURITY_NAME_LEN 16

// Sets the max length of the password key
#define AUTH_LOCALIZED_PASSWORD_KEY_LEN	20

//#define PRIV_LOCALIZED_PASSWORD_KEY_LEN	16
#define PRIV_LOCALIZED_PASSWORD_KEY_LEN	20

#endif  // _SNMPv3_CONFIG_H_
