/*******************************************************************************
  Hardware specific definitions

  Summary:
    
  Description:
    Explorer 16
    PIC24F, PIC24H, dsPIC33F
    Ethernet PICtail Plus (ENC28J60)

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
#define     BSP_SOSC_XTAL_FREQ        32768     // external SOSC Xtal, Hz


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

// ENC28J60 I/O Definition
// specify we want to use the ENC28J60 interface
#define TCPIP_IF_ENC28J60
	
#ifdef TCPIP_IF_ENC28J60
	#if defined (__dsPIC33E__) || defined (__PIC24E__)

		// Specify which SPI to use for the ENC28J60. 
		// dsPIC33E USB PIM + ENC PIC tail uses SPI2 only
		#define ENC28_SPI_CHN   2   // 1 - SPI1, 2 - SPI2

		#if (ENC28_SPI_CHN == 1)	
			#error dsPIC33E PIC24E USB PIM and Ex16 uses SPI2 only.
		#endif
		// ENC28J60 I/O pins
		#define ENC28_CS_TRIS 		(TRISFbits.TRISF12)
		#define ENC28_CS_IO			(PORTFbits.RF12)
		/***************************************************************************
		 RST pin not connected by default. It is okay to leave this pin completely 
		 unconnected, in which case this macro should simply be left undefined.
		***************************************************************************/
		//#define ENC28_RST_TRIS	(TRISDbits.TRISD15)	
		//#define ENC28_RST_IO		(PORTDbits.RD15)
	#else
		// Specify which SPI to use for the ENC28J60 or ENC624J600. 
		// USB starter kit + IO expansion board + ENC PIC tail used SPI2
		#define ENC28_SPI_CHN   1   // 1 - SPI1, 2 - SPI2
	
		// ENC28J60 I/O pins
		#define ENC28_CS_TRIS 		(TRISDbits.TRISD14)
		#define ENC28_CS_IO			(PORTDbits.RD14)
		/***************************************************************************
		 RST pin not connected by default. It is okay to leave this pin completely 
		 unconnected, in which case this macro should simply be left undefined.
		***************************************************************************/
		//#define ENC28_RST_TRIS	(TRISDbits.TRISD15)	
		//#define ENC28_RST_IO		(PORTDbits.RD15)
	#endif
#endif

#include "system_profile.h"


// 25LC256 I/O pins
#define EEPROM_CS_TRIS		(TRISDbits.TRISD12)
#define EEPROM_CS_IO		(LATDbits.LATD12)
#define EEPROM_SCK_TRIS		(TRISGbits.TRISG6)
#define EEPROM_SDI_TRIS		(TRISGbits.TRISG7)
#define EEPROM_SDO_TRIS		(TRISGbits.TRISG8)
#define EEPROM_SPI_IF		(IFS2bits.SPI2IF)
#define EEPROM_SSPBUF		(SPI2BUF)
#define EEPROM_SPICON1		(SPI2CON1)
#define EEPROM_SPICON1bits	(SPI2CON1bits)
#define EEPROM_SPICON2		(SPI2CON2)
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
//#define SPIRAM_SPI_IF			(IFS2bits.SPI2IF)
//#define SPIRAM_SSPBUF			(SPI2BUF)
//#define SPIRAM_SPICON1			(SPI2CON1)
//#define SPIRAM_SPICON1bits		(SPI2CON1bits)
//#define SPIRAM_SPICON2			(SPI2CON2)
//#define SPIRAM_SPISTAT			(SPI2STAT)
//#define SPIRAM_SPISTATbits		(SPI2STATbits)
//
//#define SPIFLASH_CS_TRIS		(TRISBbits.TRISB8)
//#define SPIFLASH_CS_IO			(LATBbits.LATB8)
//#define SPIFLASH_SCK_TRIS		(TRISGbits.TRISG6)
//#define SPIFLASH_SDI_TRIS		(TRISGbits.TRISG7)
//#define SPIFLASH_SDI_IO			(PORTGbits.RG7)
//#define SPIFLASH_SDO_TRIS		(TRISGbits.TRISG8)
//#define SPIFLASH_SPI_IF			(IFS2bits.SPI2IF)
//#define SPIFLASH_SSPBUF			(SPI2BUF)
//#define SPIFLASH_SPICON1		(SPI2CON1)
//#define SPIFLASH_SPICON1bits	(SPI2CON1bits)
//#define SPIFLASH_SPICON2		(SPI2CON2)
//#define SPIFLASH_SPISTAT		(SPI2STAT)
//#define SPIFLASH_SPISTATbits	(SPI2STATbits)

#endif // #ifndef HARDWARE_PROFILE_H
