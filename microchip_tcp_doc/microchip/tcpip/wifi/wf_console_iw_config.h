/*******************************************************************************
  MRF24W10C Driver iwconfig

  Summary:
    Module for Microchip TCP/IP Stack
    
  Description:
    -Provides access to MRF24W10C WiFi controller
    -Reference: MRF24W10C Data sheet, IEEE 802.11 Standard
*******************************************************************************/

/*******************************************************************************
FileName:  WFConsoleIwconfig.h 
Copyright � 2012 released Microchip Technology Inc.  All rights
reserved.

Microchip licenses to you the right to use, modify, copy and distribute
Software only when embedded on a Microchip microcontroller or digital signal
controller that is integrated into your product or third party product
(pursuant to the sublicense terms in the accompanying license agreement).

You should refer to the license agreement accompanying this Software for
additional information regarding your rights and obligations.

SOFTWARE AND DOCUMENTATION ARE PROVIDED �AS IS� WITHOUT WARRANTY OF ANY KIND,
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

#ifndef __WFCONSOLE_IWCONFIG_H
#define __WFCONSOLE_IWCONFIG_H

//============================================================================
//                                  Globals
//============================================================================

typedef struct
{
	uint8_t powerSaveState;	// power save state

	uint8_t cpId; 	// ID of a connection profle that is created by this
					// iwconfig cmd
	uint8_t connState;	// connection state
	bool	isIdle;		// true if connState is WF_CSTATE_NOT_CONNECTED
} tWFIwconfigCb;	// iwconfig control block

extern tWFIwconfigCb iwconfigCb;

//============================================================================
//                                  Function Prototypes
//============================================================================
extern void do_iwconfig_cmd(void);

//============================================================================
//                                  Function Prototypes (used by iwpriv)
//============================================================================
extern bool iwconfigSetCb(void);

#endif /* __WFCONSOLE_IWCONFIG_H */
