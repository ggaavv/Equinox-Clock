/*******************************************************************************
  AutoIP Defs for Microchip TCP/IP Stack

  Summary:
    
  Description:
*******************************************************************************/

/*******************************************************************************
FileName:  AutoIP.h 
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

#ifndef __AUTOIP_H
#define __AUTOIP_H

// AutoIP State Machine
typedef enum _SM_AUTOIP
{
    SM_AUTOIP_DISABLED = 0,
    SM_AUTOIP_INIT_RNG,
    SM_AUTOIP_CHECK_ADDRESS,
    SM_AUTOIP_SETUP_MESSAGE,
    SM_AUTOIP_GRATUITOUS_ARP1,
    SM_AUTOIP_GRATUITOUS_ARP2,
    SM_AUTOIP_GRATUITOUS_ARP3,
    SM_AUTOIP_DELAY,
    SM_AUTOIP_RATE_LIMIT_SET,
    SM_AUTOIP_RATE_LIMIT_WAIT,
    SM_AUTOIP_CONFIGURED,
    SM_AUTOIP_DEFEND
} SM_AUTOIP;


// Function prototypes
bool AutoIPInit(const TCPIP_STACK_MODULE_CTRL* const stackData,
                const void* const autoIpData);
void AutoIPTasks(NET_CONFIG* pConfig);
void AutoIPConflict(NET_CONFIG* pConfig);
bool AutoIPIsConfigured(NET_CONFIG* pConfig);
bool AutoIPConfigIsInProgress (NET_CONFIG* pConfig);
bool AutoIPIsDisabled(NET_CONFIG* pConfig);
void AutoIPDisable(NET_CONFIG* pConfig);
void AutoIPEnable(NET_CONFIG* pConfig);


#endif
