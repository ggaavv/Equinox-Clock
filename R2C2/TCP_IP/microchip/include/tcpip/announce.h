/*******************************************************************************
  Announce Module Header

  Summary:
    
  Description:
*******************************************************************************/

/*******************************************************************************
FileName:  Announce.h 
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

#ifndef __ANNONCE_H
#define __ANNONCE_H

// Announce layer configuration/initialization
typedef struct
{
} ANNOUNCE_MODULE_CONFIG;

void ANNOUNCE_Task(NET_CONFIG * pNetIf);
void ANNOUNCE_Notify(TCPIP_NET_HANDLE hNet, uint8_t evType, const void * param);
void ANNOUNCE_Send(void);
bool ANNOUNCE_TaskPending(void);
bool ANNOUNCE_Init(const TCPIP_STACK_MODULE_CTRL* const stackData, const ANNOUNCE_MODULE_CONFIG* const announceData);
void ANNOUNCE_DeInit(const TCPIP_STACK_MODULE_CTRL* const stackData);


#endif
