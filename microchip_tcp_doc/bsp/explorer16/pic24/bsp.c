/*******************************************************************************
  PIC24/dsPIC Explorer16 BSP initialization file

  Summary:
    BSP Module for PIC24/dsPIC Explorer16
    
  Description:
    Performs the BSP initialization of the PIC24/dsPIC Explorer16 board
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


// PIC24/dsPIC Explorer16 configuration fuses
#if defined(__PIC24F__)
// All other PIC24F PIMs
    _CONFIG2(FNOSC_PRIPLL & POSCMOD_XT)		// Primary XT OSC with 4x PLL
    _CONFIG1(JTAGEN_OFF & FWDTEN_OFF & ICS_PGx2)		// JTAG off, watchdog timer off, Communication channel is PGC2/PGD2
#elif defined(__dsPIC33F__) || defined(__PIC24H__) || defined(__dsPIC33E__)|| defined(__PIC24E__)
    // All dsPIC33F and PIC24H PIMs
    _FOSCSEL(FNOSC_PRIPLL)			// PLL enabled
    _FOSC(OSCIOFNC_OFF & POSCMD_XT)	// XT Osc
    _FWDT(FWDTEN_OFF)				// Disable Watchdog timer
    // JTAG should be disabled as well
#endif

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


#if defined(__dsPIC33F__) || defined(__PIC24H__)
    // Crank up the core frequency
    PLLFBD = 38; // Multiply by 40 for 160MHz VCO output (8MHz XT oscillator)
    //	CLKDIV = 0x0000;			// FRC: divide by 2, PLLPOST: divide by 2, PLLPRE: divide by 2
    CLKDIVbits.PLLPOST = 0; /* N1 = 2	*/
    CLKDIVbits.PLLPRE = 0; /* N2 = 2	*/
    OSCTUN = 0;

    /*	Initiate Clock Switch to Primary
     *	Oscillator with PLL (NOSC= 0x3)*/
    __builtin_write_OSCCONH(0x03);
    __builtin_write_OSCCONL(0x01);
    // Disable Watch Dog Timer
    RCONbits.SWDTEN = 0;
    while (OSCCONbits.COSC != 0x3);
    while (_LOCK == 0); /* Wait for PLL lock at 60 MIPS */
    // Port I/O
    AD1PCFGHbits.PCFG23 = 1; // Make RA7 (BUTTON1) a digital input
    AD1PCFGHbits.PCFG20 = 1; // Make RA12 (INT1) a digital input for MRF24W PICtail Plus interrupt

    // ADC
    AD1CHS0 = 0; // Input to AN0 (potentiometer)
    AD1PCFGLbits.PCFG5 = 0; // Disable digital input on AN5 (potentiometer)
    AD1PCFGLbits.PCFG4 = 0; // Disable digital input on AN4 (TC1047A temp sensor)

    TRISFbits.TRISF6 = 0;
    TRISFbits.TRISF7 = 0;
    TRISFbits.TRISF8 = 0;

#elif defined(__dsPIC33E__)||defined(__PIC24E__)

    // Crank up the core frequency
    PLLFBD = 38; /* M  = 30	*/
    CLKDIVbits.PLLPOST = 0; /* N1 = 2	*/
    CLKDIVbits.PLLPRE = 0; /* N2 = 2	*/
    OSCTUN = 0;

    /*	Initiate Clock Switch to Primary
     *	Oscillator with PLL (NOSC= 0x3)*/
    __builtin_write_OSCCONH(0x03);
    __builtin_write_OSCCONL(0x01);
    // Disable Watch Dog Timer
    RCONbits.SWDTEN = 0;
    while (OSCCONbits.COSC != 0x3);
    while (_LOCK == 0); /* Wait for PLL lock at 60 MIPS */
    // Port I/O
    ANSELAbits.ANSA7 = 0; //Make RA7 (BUTTON1) a digital input
#if defined ENC100_INTERFACE_MODE > 0
    ANSELEbits.ANSE0 = 0; // Make these PMP pins as digital output when the interface is parallel.
    ANSELEbits.ANSE1 = 0;
    ANSELEbits.ANSE2 = 0;
    ANSELEbits.ANSE3 = 0;
    ANSELEbits.ANSE4 = 0;
    ANSELEbits.ANSE5 = 0;
    ANSELEbits.ANSE6 = 0;
    ANSELEbits.ANSE7 = 0;
    ANSELBbits.ANSB10 = 0;
    ANSELBbits.ANSB11 = 0;
    ANSELBbits.ANSB12 = 0;
    ANSELBbits.ANSB13 = 0;
    ANSELBbits.ANSB15 = 0;
#endif

    ANSELEbits.ANSE8 = 0; // Make RE8(INT1) a digital input for ZeroG ZG2100M PICtail

    AD1CHS0 = 0; // Input to AN0 (potentiometer)
    ANSELBbits.ANSB0 = 1; // Input to AN0 (potentiometer)
    ANSELBbits.ANSB5 = 1; // Disable digital input on AN5 (potentiometer)
    ANSELBbits.ANSB4 = 1; // Disable digital input on AN4 (TC1047A temp sensor)
    ANSELDbits.ANSD7 = 0; //  Digital Pin Selection for S3(Pin 83) and S4(pin 84).
    ANSELDbits.ANSD6 = 0;

    ANSELGbits.ANSG6 = 0; // Enable Digital input for RG6 (SCK2)
    ANSELGbits.ANSG7 = 0; // Enable Digital input for RG7 (SDI2)
    ANSELGbits.ANSG8 = 0; // Enable Digital input for RG8 (SDO2)
    ANSELGbits.ANSG9 = 0; // Enable Digital input for RG9 (CS)
#if defined ENC100_INTERFACE_MODE == 0	// SPI Interface, UART can be used for debugging. Not allowed for other interfaces.
    RPOR9 = 0x0300; //RP101= U2TX
    RPINR19 = 0X0064; //RP100= U2RX
#endif

#if defined WF_CS_TRIS
    RPINR1bits.INT3R = 30;
    WF_CS_IO = 1;
    WF_CS_TRIS = 0;

#endif

#else	//defined(__PIC24F__)
#if defined(__PIC24F__)
    CLKDIVbits.RCDIV = 0; // Set 1:1 8MHz FRC postscalar
#endif

    // ADC
#if defined(__PIC24FJ256DA210__) || defined(__PIC24FJ256GB210__)
    // Disable analog on all pins
    ANSA = 0x0000;
    ANSB = 0x0000;
    ANSC = 0x0000;
    ANSD = 0x0000;
    ANSE = 0x0000;
    ANSF = 0x0000;
    ANSG = 0x0000;
#else
    AD1CHS = 0; // Input to AN0 (potentiometer)
    AD1PCFGbits.PCFG4 = 0; // Disable digital input on AN4 (TC1047A temp sensor)
    AD1PCFGbits.PCFG5 = 0; // Disable digital input on AN5 (potentiometer)
#endif
#endif

    // ADC
    AD1CON1 = 0x84E4; // Turn on, auto sample start, auto-convert, 12 bit mode (on parts with a 12bit A/D)
    AD1CON2 = 0x0404; // AVdd, AVss, int every 2 conversions, MUXA only, scan
    AD1CON3 = 0x1003; // 16 Tad auto-sample, Tad = 3*Tcy
    AD1CSSL = 1 << 5; // Scan pot

    // Deassert all chip select lines so there isn't any problem with
    // initialization order.  Ex: When ENC28J60 is on SPI2 with Explorer 16,
    // MAX3232 ROUT2 pin will drive RF12/U2CTS ENC28J60 CS line asserted,
    // preventing proper 25LC256 EEPROM operation.
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
#if defined(TCPIP_IF_MRF24W)
    // Removed CS operation here for better code organization
    // Tested fine with removing these two lines
#endif

#if defined(PIC24FJ64GA004_PIM)
    __builtin_write_OSCCONL(OSCCON & 0xBF); // Unlock PPS

    // Remove some LED outputs to regain other functions
    LED1_TRIS = 1; // Multiplexed with BUTTON0
    LED5_TRIS = 1; // Multiplexed with EEPROM CS
    LED7_TRIS = 1; // Multiplexed with BUTTON1

    // Inputs
    RPINR19bits.U2RXR = 19; //U2RX = RP19
    RPINR22bits.SDI2R = 20; //SDI2 = RP20
    RPINR20bits.SDI1R = 17; //SDI1 = RP17

    // Outputs
    RPOR12bits.RP25R = U2TX_IO; //RP25 = U2TX
    RPOR12bits.RP24R = SCK2OUT_IO; //RP24 = SCK2
    RPOR10bits.RP21R = SDO2_IO; //RP21 = SDO2
    RPOR7bits.RP15R = SCK1OUT_IO; //RP15 = SCK1
    RPOR8bits.RP16R = SDO1_IO; //RP16 = SDO1

    AD1PCFG = 0xFFFF; //All digital inputs - POT and Temp are on same pin as SDO1/SDI1, which is needed for ENC28J60 commnications

    __builtin_write_OSCCONL(OSCCON | 0x40); // Lock PPS
#endif

#if defined(__PIC24FJ256DA210__)
    __builtin_write_OSCCONL(OSCCON & 0xBF); // Unlock PPS

    // Inputs
    RPINR19bits.U2RXR = 11; // U2RX = RP11
    RPINR20bits.SDI1R = 0; // SDI1 = RP0
    RPINR0bits.INT1R = 34; // Assign RE9/RPI34 to INT1 (input) for MRF24W Wi-Fi PICtail Plus interrupt

    // Outputs
    RPOR8bits.RP16R = 5; // RP16 = U2TX
    RPOR1bits.RP2R = 8; // RP2 = SCK1
    RPOR0bits.RP1R = 7; // RP1 = SDO1

    __builtin_write_OSCCONL(OSCCON | 0x40); // Lock PPS
#endif

#if defined(__PIC24FJ256GB110__) || defined(__PIC24FJ256GB210__)
    __builtin_write_OSCCONL(OSCCON & 0xBF); // Unlock PPS

    // Configure SPI1 PPS pins (ENC28J60/ENCX24J600/MRF24W or other PICtail Plus cards)
    RPOR0bits.RP0R = 8; // Assign RP0 to SCK1 (output)
    RPOR7bits.RP15R = 7; // Assign RP15 to SDO1 (output)
    RPINR20bits.SDI1R = 23; // Assign RP23 to SDI1 (input)

    // Configure SPI2 PPS pins (25LC256 EEPROM on Explorer 16)
    RPOR10bits.RP21R = 11; // Assign RG6/RP21 to SCK2 (output)
    RPOR9bits.RP19R = 10; // Assign RG8/RP19 to SDO2 (output)
    RPINR22bits.SDI2R = 26; // Assign RG7/RP26 to SDI2 (input)

    // Configure UART2 PPS pins (MAX3232 on Explorer 16)
#if !defined(ENC100_INTERFACE_MODE) || (ENC100_INTERFACE_MODE == 0) || defined(ENC100_PSP_USE_INDIRECT_RAM_ADDRESSING)
    RPINR19bits.U2RXR = 10; // Assign RF4/RP10 to U2RX (input)
    RPOR8bits.RP17R = 5; // Assign RF5/RP17 to U2TX (output)
#endif

    // Configure INT1 PPS pin (MRF24W Wi-Fi PICtail Plus interrupt signal when in SPI slot 1)
    RPINR0bits.INT1R = 33; // Assign RE8/RPI33 to INT1 (input)

    // Configure INT3 PPS pin (MRF24W Wi-Fi PICtail Plus interrupt signal when in SPI slot 2)
    RPINR1bits.INT3R = 40; // Assign RC3/RPI40 to INT3 (input)

    __builtin_write_OSCCONL(OSCCON | 0x40); // Lock PPS
#endif

#if defined(__PIC24FJ256GA110__)
    __builtin_write_OSCCONL(OSCCON & 0xBF); // Unlock PPS

    // Configure SPI2 PPS pins (25LC256 EEPROM on Explorer 16 and ENC28J60/ENCX24J600/MRF24W or other PICtail Plus cards)
    // Note that the ENC28J60/ENCX24J600/MRF24W PICtails SPI PICtails must be inserted into the middle SPI2 socket, not the topmost SPI1 slot as normal.  This is because PIC24FJ256GA110 A3 silicon has an input-only RPI PPS pin in the ordinary SCK1 location.  Silicon rev A5 has this fixed, but for simplicity all demos will assume we are using SPI2.
    RPOR10bits.RP21R = 11; // Assign RG6/RP21 to SCK2 (output)
    RPOR9bits.RP19R = 10; // Assign RG8/RP19 to SDO2 (output)
    RPINR22bits.SDI2R = 26; // Assign RG7/RP26 to SDI2 (input)

    // Configure UART2 PPS pins (MAX3232 on Explorer 16)
    RPINR19bits.U2RXR = 10; // Assign RF4/RP10 to U2RX (input)
    RPOR8bits.RP17R = 5; // Assign RF5/RP17 to U2TX (output)

    // Configure INT3 PPS pin (MRF24W PICtail Plus interrupt signal)
    RPINR1bits.INT3R = 36; // Assign RA14/RPI36 to INT3 (input)

    __builtin_write_OSCCONL(OSCCON | 0x40); // Lock PPS
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


