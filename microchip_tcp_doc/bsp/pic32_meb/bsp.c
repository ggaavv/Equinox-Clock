/*******************************************************************************
  PIC32 MEB BSP initialization file

  Summary:
    BSP Module for PIC32 MEB
    
  Description:
    Performs the BSP initialization of the PIC32 MEB board
 *******************************************************************************/

/*******************************************************************************
FileName:  bsp.c 
Copyright 2012 released Microchip Technology Inc.  All rights
reserved.

Microchip licenses to you the right to use, modify, copy and distribute
Software only when embedded on a Microchip microcontroller or digital signal
controller that is integrated into your product or third party product
(pursuant to the sublicense terms in the accompanying license agreement).

You should refer to the license agreement accompanying this Software for
additional information regarding your rights and obligations.

SOFTWARE AND DOCUMENTATION ARE PROVIDED “AS IS?WITHOUT WARRANTY OF ANY KIND,
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


#include "hardware_profile.h"


// PIC32 MEB configuration fuses
#pragma config FPLLODIV = DIV_1, FPLLMUL = MUL_20, FPLLIDIV = DIV_2, FWDTEN = OFF, FPBDIV = DIV_1, POSCMOD = XT, FNOSC = PRIPLL, CP = OFF
#pragma config FMIIEN = OFF, FETHIO = OFF	// external PHY in RMII/alternate configuration



/****************************************************************************
  Function:
    void BSP_Initialize(void)

  Description:
    This routine initializes the hardware. 

  Precondition:
    None

  Parameters:
    None - None

  Returns:
    None

  Remarks:
    None
 ***************************************************************************/
void BSP_Initialize(void)
{
    // LEDs
    LED0_TRIS = 0;
    LED1_TRIS = 0;
    LED2_TRIS = 0;
    LED3_TRIS = 0;
    LED4_TRIS = 0;
    LED5_TRIS = 0;
    LED6_TRIS = 0;
    LED7_TRIS = 0;
    LED_PUT(0x00);


    LED_PUT(0x00); // Turn the LEDs off

    CNPUESET = 0x00098000; // Turn on weak pull ups on CN15, CN16, CN19 (RD5, RD7, RD13), which is connected to buttons on PIC32 Starter Kit boards
    // Initialize MEB CPLD
    // Graphics 16-bit
    LATBCLR = 0x100; // WiFi CE signal
    TRISBCLR = 0x100;

    LATGSET = 0x4000; // G14: 0=8-bit, 1=16-bit
    TRISGCLR = 0x4000;

    // SPI2 input
    LATGCLR = 0x1000; // G12: 0=SPI3A, 1=SPI2
    TRISGCLR = 0x1000;

    LATACLR = 0xc0; // Clear SPI MUX
    // 00 - 16MB SPI Flash on board
    // 01 - 802.11 module
    // 10 - Expansion Connector
    // 11 - Reserved
    LATASET = 0x40; // 01 - 802.11
    TRISACLR = 0xc0;


    // ADC
    AD1CON1 = 0x84E4; // Turn on, auto sample start, auto-convert, 12 bit mode (on parts with a 12bit A/D)
    AD1CON2 = 0x0404; // AVdd, AVss, int every 2 conversions, MUXA only, scan
    AD1CON3 = 0x1003; // 16 Tad auto-sample, Tad = 3*Tcy
    AD1CSSL = 1 << 2; // Scan pot


}


