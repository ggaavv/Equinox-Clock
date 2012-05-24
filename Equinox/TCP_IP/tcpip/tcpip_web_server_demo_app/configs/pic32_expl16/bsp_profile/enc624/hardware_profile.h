/*******************************************************************************
Hardware specific definitions

  Summary:
    
  Description:
    - Explorer 16
    - PIC32MX
    - Fast 100Mbps Ethernet PICtail Plus (ENC624J600)
*******************************************************************************/

/*******************************************************************************
FileName: hardware_profile.h	
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

#ifndef HARDWARE_PROFILE_H
#define HARDWARE_PROFILE_H

#include "Compiler.h"

// Define a macro describing this hardware set up (used in other files)
#define EXPLORER_16


#define     BSP_BOARD_XTAL_FREQ		8000000 	// external Xtal, Hz
#define     BSP_SOSC_XTAL_FREQ		32768     // external SOSC Xtal, Hz

#define TCPIP_IF_ENCX24J600

// Hardware I/O pin mappings

// LEDs
#define LED0_TRIS			(TRISAbits.TRISA0)	// Ref D3
#define LED0_IO				(LATAbits.LATA0)
#define LED1_TRIS			(TRISAbits.TRISA1)	// Ref D4
#define LED1_IO				(LATAbits.LATA1)
#define LED2_TRIS			(TRISAbits.TRISA2)	// Ref D5
#define LED2_IO				(LATAbits.LATA2)
#define LED3_TRIS			(TRISAbits.TRISA3)	// Ref D6
#define LED3_IO				(LATAbits.LATA3)
#define LED4_TRIS			(TRISAbits.TRISA4)	// Ref D7
#define LED4_IO				(LATAbits.LATA4)
#define LED5_TRIS			(TRISAbits.TRISA5)	// Ref D8
#define LED5_IO				(LATAbits.LATA5)
#define LED6_TRIS			(TRISAbits.TRISA6)	// Ref D9
#define LED6_IO				(LATAbits.LATA6)
#define LED7_TRIS			(LATAbits.LATA7)	// Ref D10;  Note: This is multiplexed with BUTTON1, so this LED can't be used.  However, it will glow very dimmly due to a weak pull up resistor.
#define LED7_IO				(LATAbits.LATA7)
#define LED_GET()			(*((volatile unsigned char*)(&LATA)))
#define LED_PUT(a)			(*((volatile unsigned char*)(&LATA)) = (a))

// Momentary push buttons
#define BUTTON0_TRIS		(TRISDbits.TRISD13)	// Ref S4
#define	BUTTON0_IO			(PORTDbits.RD13)
#define BUTTON1_TRIS		(TRISAbits.TRISA7)	// Ref S5;  Note: This is multiplexed with LED7
#define	BUTTON1_IO			(PORTAbits.RA7)
#define BUTTON2_TRIS		(TRISDbits.TRISD7)	// Ref S6
#define	BUTTON2_IO			(PORTDbits.RD7)
#define BUTTON3_TRIS		(TRISDbits.TRISD6)	// Ref S3
#define	BUTTON3_IO			(PORTDbits.RD6)


// ENC624J600 Interface Configuration
// Comment out ENC100_INTERFACE_MODE if you don't have an ENC624J600 or 
// ENC424J600.  Otherwise, choose the correct setting for the interface you 
// are using.  Legal values are:
//  - Commented out: No ENC424J600/624J600 present or used.  All other 
//                   ENC100_* macros are ignored.
//	- 0: SPI mode using CS, SCK, SI, and SO pins
//  - 1: 8-bit demultiplexed PSP Mode 1 with RD and WR pins
//  - 2: *8-bit demultiplexed PSP Mode 2 with R/Wbar and EN pins
//  - 3: *16-bit demultiplexed PSP Mode 3 with RD, WRL, and WRH pins
//  - 4: *16-bit demultiplexed PSP Mode 4 with R/Wbar, B0SEL, and B1SEL pins
//  - 5: 8-bit multiplexed PSP Mode 5 with RD and WR pins
//  - 6: *8-bit multiplexed PSP Mode 6 with R/Wbar and EN pins
//  - 9: 16-bit multiplexed PSP Mode 9 with AL, RD, WRL, and WRH pins
//  - 10: *16-bit multiplexed PSP Mode 10 with AL, R/Wbar, B0SEL, and B1SEL 
//        pins
// *IMPORTANT NOTE: DO NOT USE PSP MODE 2, 4, 6, OR 10 ON EXPLORER 16! 
// Attempting to do so will cause bus contention with the LCD module which 
// shares the PMP.  Also, PSP Mode 3 is risky on the Explorer 16 since it 
// can randomly cause bus contention with the 25LC256 EEPROM.
#define ENC100_INTERFACE_MODE			0

#if (ENC100_INTERFACE_MODE == 0)
	#define ENCX24_SPI_CHN	1	// 1 - SPI1, 2 - SPI2
	//#define ENCX24_SPI_CHN 2
#endif // (ENC100_INTERFACE_MODE == 0)

// If using a parallel interface, direct RAM addressing can be used (if all 
// addresses wires are connected), or a reduced number of pins can be used 
// for indirect addressing.  If using an SPI interface or PSP Mode 9 or 10 
// (multiplexed 16-bit modes), which require all address lines to always be 
// connected, then this option is ignored. Comment out or uncomment this 
// macro to match your hardware connections.
#define ENC100_PSP_USE_INDIRECT_RAM_ADDRESSING

// ENC424J600/624J600 parallel indirect address remapping macro function.
// This section translates SFR and RAM addresses presented to the 
// ReadMemory() and WriteMemory() APIs in ENCX24J600.c to the actual 
// addresses that must be presented on the parallel interface.  This macro 
// must be modified to match your hardware if you are using an indirect PSP 
// addressing mode (ENC100_PSP_USE_INDIRECT_RAM_ADDRESSING is defined) and 
// have some of your address lines tied off to Vdd.  If you are using the 
// SPI interface, then this section can be ignored or deleted.
#if (ENC100_INTERFACE_MODE == 1) || (ENC100_INTERFACE_MODE == 2) || (ENC100_INTERFACE_MODE == 5) || (ENC100_INTERFACE_MODE == 6) // 8-bit PSP
	// Several TCP/IP evaluation boards use the PMA8/U2TX pin for UART communication.
	// This function will remap the A8 address signal used in PMP modes 1, 2, 6 and 6
	// to PMPCS1 (PMA14), which will allow continued UART2 functionality on these boards.
	#define ENC100_TRANSLATE_TO_PIN_ADDR(a)		((((a)&0x0100)<<6) | ((a)&0x00FF))
#elif (ENC100_INTERFACE_MODE == 3) || (ENC100_INTERFACE_MODE == 4) // 16-bit PSP
	#define ENC100_TRANSLATE_TO_PIN_ADDR(a)		(a)
#endif

// Auto-crossover pins on Fast 100Mbps Ethernet PICtail/PICtail Plus.  If 
// your circuit doesn't have such a feature, delete these two defines.
#if defined(__dsPIC33E__)|| defined (__PIC24E__)
	#define ENC100_MDIX_TRIS				(TRISAbits.TRISA9)
	#define ENC100_MDIX_IO					(LATAbits.LATA9)
#else
	#define ENC100_MDIX_TRIS				(TRISBbits.TRISB3)
	#define ENC100_MDIX_IO					(LATBbits.LATB3)
#endif

#include "system_profile.h"

// ENC624J600 I/O control and status pins
// If a pin is not required for your selected ENC100_INTERFACE_MODE 
// interface selection (ex: WRH/B1SEL for PSP modes 1, 2, 5, and 6), then 
// you can ignore, delete, or put anything for the pin definition.  Also, 
// the INT and POR pins are entirely optional.  If not connected, comment 
// them out.
#define ENC100_INT_TRIS				(TRISEbits.TRISE9)		// INT signal is optional and currently unused in the Microchip TCP/IP Stack.  Leave this pin disconnected and comment out this pin definition if you don't want it.
#define ENC100_INT_IO				(PORTEbits.RE9)
#if (ENC100_INTERFACE_MODE >= 1)	// Parallel mode
	// PSP control signal pinout
	#define ENC100_CS_TRIS					(TRISAbits.TRISA5)	// CS is optional in PSP mode.  If you are not sharing the parallel bus with another device, tie CS to Vdd and comment out this pin definition.
	#define ENC100_CS_IO					(LATAbits.LATA5)
	#define ENC100_POR_TRIS					(TRISCbits.TRISC1)	// POR signal is optional.  If your application doesn't have a power disconnect feature, comment out this pin definition.
	#define ENC100_POR_IO					(LATCbits.LATC1)
	#define ENC100_SO_WR_B0SEL_EN_TRIS		(TRISDbits.TRISD4)
	#define ENC100_SO_WR_B0SEL_EN_IO		(LATDbits.LATD4)
	#define ENC100_SI_RD_RW_TRIS			(TRISDbits.TRISD5)
	#define ENC100_SI_RD_RW_IO				(LATDbits.LATD5)
	#define ENC100_SCK_AL_TRIS				(TRISBbits.TRISB15)
	#define ENC100_SCK_AL_IO				(LATBbits.LATB15)
#else
	// SPI pinout
	#define ENC100_CS_TRIS					(TRISDbits.TRISD14)	// CS is mandatory when using the SPI interface
	#define ENC100_CS_IO					(LATDbits.LATD14)
	#define ENC100_POR_TRIS					(TRISDbits.TRISD15)	// POR signal is optional.  If your application doesn't have a power disconnect feature, comment out this pin definition.
	#define ENC100_POR_IO					(LATDbits.LATD15)
#endif

// ENC624J600 SPI SFR register selection (controls which SPI peripheral to 
// use on PICs with multiple SPI peripherals).  If a parallel interface is 
// used (ENC100_INTERFACE_MODE is >= 1), then the SPI is not used and this 
// section can be ignored or deleted.
#define ENC100_ISR_ENABLE		(IEC0bits.INT2IE)
#define ENC100_ISR_FLAG			(IFS0bits.INT2IF)
#define ENC100_ISR_POLARITY		(INTCONbits.INT2EP)
#define ENC100_ISR_PRIORITY		(IPC2bits.INT2IP)
#define ENC100_SPI_ENABLE		(ENC100_SPICON1bits.ON)
#define ENC100_SPI_IF			(IFS0bits.SPI1RXIF)
#define ENC100_SSPBUF			(SPI1BUF)
#define ENC100_SPICON1			(SPI1CON)
#define ENC100_SPISTATbits		(SPI1STATbits)
#define ENC100_SPICON1bits		(SPI1CONbits)
#define ENC100_SPIBRG			(SPI1BRG)

// 25LC256 I/O pins
#define EEPROM_CS_TRIS		(TRISDbits.TRISD12)
#define EEPROM_CS_IO		(LATDbits.LATD12)
#define EEPROM_SCK_TRIS		(TRISGbits.TRISG6)
#define EEPROM_SDI_TRIS		(TRISGbits.TRISG7)
#define EEPROM_SDO_TRIS		(TRISGbits.TRISG8)
#define EEPROM_SPI_IF		(IFS1bits.SPI2RXIF)
#define EEPROM_SSPBUF		(SPI2BUF)
#define EEPROM_SPICON1		(SPI2CON)
#define EEPROM_SPICON1bits	(SPI2CONbits)
#define EEPROM_SPIBRG		(SPI2BRG)
#define EEPROM_SPISTAT		(SPI2STAT)
#define EEPROM_SPISTATbits	(SPI2STATbits)

// LCD Module I/O pins.  NOTE: On the Explorer 16, the LCD is wired to the 
// same PMP lines required to communicate with an ENCX24J600 in parallel 
// mode.  Since the LCD does not have a chip select wire, if you are using 
// the ENC424J600/624J600 in parallel mode, the LCD cannot be used.
#if !defined(ENC100_INTERFACE_MODE) || (ENC100_INTERFACE_MODE == 0)	// SPI only
	#define LCD_DATA_TRIS		(*((volatile unsigned char*)&TRISE))
	#define LCD_DATA_IO			(*((volatile unsigned char*)&LATE))
	#define LCD_RD_WR_TRIS		(TRISDbits.TRISD5)
	#define LCD_RD_WR_IO		(LATDbits.LATD5)
	#define LCD_RS_TRIS			(TRISBbits.TRISB15)
	#define LCD_RS_IO			(LATBbits.LATB15)
	#define LCD_E_TRIS			(TRISDbits.TRISD4)
	#define LCD_E_IO			(LATDbits.LATD4)
#endif

//// Serial Flash/SRAM PICtail Plus attached to SPI2 (middle pin group)
//// This daughter card is not in production, but if you custom attach an SPI 
//// RAM or SPI Flash chip to your board, then use these definitions as a 
//// starting point.
//#define SPIRAM_CS_TRIS			(TRISGbits.TRISG9)
//#define SPIRAM_CS_IO			(LATGbits.LATG9)
//#define SPIRAM_SCK_TRIS			(TRISGbits.TRISG6)
//#define SPIRAM_SDI_TRIS			(TRISGbits.TRISG7)
//#define SPIRAM_SDO_TRIS			(TRISGbits.TRISG8)
//#define SPIRAM_SPI_IF			(IFS1bits.SPI2RXIF)
//#define SPIRAM_SSPBUF			(SPI2BUF)
//#define SPIRAM_SPICON1			(SPI2CON)
//#define SPIRAM_SPICON1bits		(SPI2CONbits)
//#define SPIRAM_SPIBRG			(SPI2BRG)
//
//#define SPIFLASH_CS_TRIS		(TRISBbits.TRISB8)
//#define SPIFLASH_CS_IO			(LATBbits.LATB8)
//#define SPIFLASH_SCK_TRIS		(TRISGbits.TRISG6)
//#define SPIFLASH_SDI_TRIS		(TRISGbits.TRISG7)
//#define SPIFLASH_SDI_IO			(PORTGbits.RG7)
//#define SPIFLASH_SDO_TRIS		(TRISGbits.TRISG8)
//#define SPIFLASH_SPI_IF			(IFS1bits.SPI2RXIF)
//#define SPIFLASH_SSPBUF			(SPI2BUF)
//#define SPIFLASH_SPICON1		(SPI2CON)
//#define SPIFLASH_SPICON1bits	(SPI2CONbits)
//#define SPIFLASH_SPISTATbits	(SPI2STATbits)
//#define SPIFLASH_SPIBRG			(SPI2BRG)

// Other configurations not supported
// For example, on PIC32MX460F512L and PIC32MX795F512L
// RST and CE are on RG2 and RG3 which are multiplexed with USB D+ and D-."

#endif // #ifndef HARDWARE_PROFILE_H
