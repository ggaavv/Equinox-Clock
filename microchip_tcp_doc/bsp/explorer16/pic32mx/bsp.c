/*******************************************************************************
  PIC32 Explorer16 BSP initialization file

  Summary:
    BSP Module for PIC32 Explorer16
    
  Description:
    Performs the BSP initialization of the PIC32 Explorer16 board
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


// PIC32 Explorer16 configuration fuses
#pragma config FPLLODIV = DIV_1, FPLLMUL = MUL_20, FPLLIDIV = DIV_2, FWDTEN = OFF, FPBDIV = DIV_1, POSCMOD = XT, FNOSC = PRIPLL, CP = OFF

// extern references to init functions
void XEEInit(void);
void SPIRAMInit(void);
void SPIFlashInit(void);		


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
    LED_PUT(0x00);


    DDPCONbits.JTAGEN = 0;

    CNPUESET = 0x00098000; // Turn on weak pull ups on CN15, CN16, CN19 (RD5, RD7, RD13), which is connected to buttons on PIC32 Starter Kit boards

    AD1CHS = 0; // Input to AN0 (potentiometer)
    AD1PCFGbits.PCFG4 = 0; // Disable digital input on AN4 (TC1047A temp sensor)
#if defined(__32MX460F512L__) || defined(__32MX795F512L__)	// PIC32MX460F512L and PIC32MX795F512L PIMs has different pinout to accomodate USB module
    AD1PCFGbits.PCFG2 = 0; // Disable digital input on AN2 (potentiometer)
#else
    AD1PCFGbits.PCFG5 = 0; // Disable digital input on AN5 (potentiometer)
#endif

    // ADC
    AD1CON1 = 0x84E4; // Turn on, auto sample start, auto-convert, 12 bit mode (on parts with a 12bit A/D)
    AD1CON2 = 0x0404; // AVdd, AVss, int every 2 conversions, MUXA only, scan
    AD1CON3 = 0x1003; // 16 Tad auto-sample, Tad = 3*Tcy
#if defined(__32MX460F512L__) || defined(__32MX795F512L__)	// PIC32MX460F512L and PIC32MX795F512L PIMs has different pinout to accomodate USB module
    AD1CSSL = 1 << 2; // Scan pot
#else
    AD1CSSL = 1 << 5; // Scan pot
#endif

#if defined(ENC_CS_TRIS)
    ENC_CS_IO = 1;
    ENC_CS_TRIS = 0;
#endif
#if defined(ENC100_CS_TRIS)
    ENC100_CS_IO = (ENC100_INTERFACE_MODE == 0);
    ENC100_CS_TRIS = 0;
#endif
#if defined(EEPROM_CS_TRIS)
    EEPROM_CS_IO = 1;
    EEPROM_CS_TRIS = 0;
#endif
#if defined(SPIRAM_CS_TRIS)
    SPIRAM_CS_IO = 1;
    SPIRAM_CS_TRIS = 0;
#endif
#if defined(SPIFLASH_CS_TRIS)
    SPIFLASH_CS_IO = 1;
    SPIFLASH_CS_TRIS = 0;
#endif

#if defined(SPIRAM_CS_TRIS)
    SPIRAMInit();
#endif
#if defined(EEPROM_CS_TRIS)
    XEEInit();
#endif
#if defined(SPIFLASH_CS_TRIS)
    SPIFlashInit();
#endif

}


