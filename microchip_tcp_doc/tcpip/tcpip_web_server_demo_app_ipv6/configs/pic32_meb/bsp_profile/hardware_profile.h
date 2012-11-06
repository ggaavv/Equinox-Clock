/*******************************************************************************
Hardware specific definitions

  Summary:
    
  Description:

*******************************************************************************/

/*******************************************************************************
FileName: 	hardware_profile.h
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


#ifndef __HARDWARE_PROFILE_H
#define __HARDWARE_PROFILE_H

#include "Compiler.h"

// Define a macro describing this hardware set up (used in other files)
// PIC32 Ethernet Starter Kit (04-02146) with PIC32MX795F512L processor and National DP83848 10/100 PHY
#define PIC32_ENET_SK_DM320004


#define		BSP_BOARD_XTAL_FREQ		8000000 	// external Xtal, Hz
#define    BSP_SOSC_XTAL_FREQ         32768     // external SOSC Xtal, Hz

// Hardware IO pin mappings

// using the PIC32 internal MAC interface
#define TCPIP_IF_PIC32INT

// External DP83848 PHY configuration
#define	PHY_RMII				// external PHY runs in RMII mode
#define	PHY_CONFIG_ALTERNATE	// alternate configuration used
#define	PHY_ADDRESS			0x1	// the address of the National DP83848 PHY

// Hardware mappings
#define LED0_TRIS			(TRISDbits.TRISD0)	// Ref LED1
#define LED0_IO				(LATDbits.LATD0)
#define LED1_TRIS			(TRISDbits.TRISD1)	// Ref LED2
#define LED1_IO				(LATDbits.LATD1)
#define LED2_TRIS			(TRISDbits.TRISD2)	// Ref LED3
#define LED2_IO				(LATDbits.LATD2)
#define LED3_TRIS			(LED2_TRIS)			// No such LED
#define LED3_IO				(LATDbits.LATD6)
#define LED4_TRIS			(LED2_TRIS)			// No such LED
#define LED4_IO				(LATDbits.LATD6)
#define LED5_TRIS			(LED2_TRIS)			// No such LED
#define LED5_IO				(LATDbits.LATD6)
#define LED6_TRIS			(LED2_TRIS)			// No such LED
#define LED6_IO				(LATDbits.LATD6)
#define LED7_TRIS			(LED2_TRIS)			// No such LED
#define LED7_IO				(LATDbits.LATD6)

#define LED_GET()			((BYTE)LATD & 0x07)
#define LED_PUT(a)			do{LATD = (LATD & 0xFFF8) | ((a)&0x07);}while(0)

#define BUTTON0_TRIS		(TRISDbits.TRISD6)	// Ref SW1
#define	BUTTON0_IO			(PORTDbits.RD6)
#define BUTTON1_TRIS		(TRISDbits.TRISD7)	// Ref SW2
#define	BUTTON1_IO			(PORTDbits.RD7)
#define BUTTON2_TRIS		(TRISDbits.TRISD13)	// Ref SW3
#define	BUTTON2_IO			(PORTDbits.RD13)
#define BUTTON3_TRIS		(TRISDbits.TRISD13)	// No BUTTON3 on this board
#define	BUTTON3_IO			(1)

// Note, it is not possible to use a MRF24W 802.11 WiFi PICtail 
// Plus card with this starter kit.  The required interrupt signal, among 
// possibly other I/O pins aren't available on the Starter Kit board.

// specify we want to use the MRF24W interface
#define TCPIP_IF_MRF24W
#if defined(TCPIP_IF_MRF24W)
	#define MRF24W_SPI_CHN   4   // 1 - SPI1, 2 - SPI2
	// IO mapping for general control pins, inlcuding CS, RESET and HIBERNATE
	#define WF_CS_TRIS			(TRISFbits.TRISF12)
	#define WF_CS_IO			(LATFbits.LATF12)
	#define WF_RESET_TRIS		(TRISAbits.TRISA10)
	#define WF_RESET_IO 		(LATAbits.LATA10)
	#define WF_HIBERNATE_TRIS	(TRISBbits.TRISB8)
	#define WF_HIBERNATE_IO 	(PORTBbits.RB8)	
	// Interrupt source selection
	#define MRF24W_USE_CN_INT
	//#define MRF24W_USE_INT3_INT
#endif

#include "system_profile.h"

#endif

