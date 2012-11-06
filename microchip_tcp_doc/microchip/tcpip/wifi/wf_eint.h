/*******************************************************************************
  Wifi interrupt hanlding header for PIC32

  Summary:
    
  Description:
*******************************************************************************/

/*******************************************************************************
FileName:  WF_Eint.h 
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

#ifndef _WF_EINT_H_
#define _WF_EINT_H_

#include "Compiler.h"
#include "hardware_profile.h"

#if defined(TCPIP_IF_MRF24W)

#if defined(MRF24W_USE_CN_INT)
    #define MRFWB0M_INT_SOURCE  PLIB_INT_SOURCE_CHANGE_NOTICE

#elif defined(MRF24W_USE_INT3_INT)
    #define MRFWB0M_INT_SOURCE  PLIB_INT_SOURCE_EXTERNAL_3	

#elif defined( MRF24W_IN_SPI2 )
    #define MRFWB0M_INT_SOURCE  PLIB_INT_SOURCE_EXTERNAL_3

#elif defined(MRF24W_IN_SPI1) || defined(MRF24W_USE_INT1_INT)
    #define MRFWB0M_INT_SOURCE  PLIB_INT_SOURCE_EXTERNAL_1

#else
    #error  "Either MRF24W_IN_SPI1 or MRF24W_IN_SPI2 have to be defined for MRF24W module!"
#endif


#if defined(MRF24W_USE_CN_INT)	
	#define WF_INT_TRIS 		(TRISGbits.TRISG7) // CN9 on ESK+MEB
	#define WF_INT_IO			(PORTGbits.RG7)
	#define WF_INT_CNPUE_SET	CNPUESET 
	#define WF_INT_CN_ENABLE	(CNCONbits.ON)
	#define WF_INT_CNEN_SET 	CNENSET
	#define WF_INT_IE			(IEC1bits.CNIE)
	#define WF_INT_IF			(IFS1bits.CNIF)
	#define WF_INT_IE_CLEAR 	IEC1CLR
	#define WF_INT_IF_CLEAR 	IFS1CLR
	#define WF_INT_IE_SET		IEC1SET
	#define WF_INT_IF_SET		IFS1SET	
	#define WF_INT_BIT			0x00000001
	#define WF_INT_BIT_MASK    (0x01<<9)
	#define WF_INT_IPCSET		IPC6SET
	#define WF_INT_IPCCLR		IPC6CLR
	#define WF_INT_IPC_MASK 	0x00FF0000
	#define WF_INT_IPC_VALUE	0x000C0000
#elif defined(MRF24W_USE_INT3_INT)
	#if defined(__PIC24FJ256GB110__) || defined(__PIC24FJ256GB210__)
		#define WF_INT_TRIS			(TRISCbits.TRISC3)	// INT3
		#define WF_INT_IO			(PORTCbits.RC3)
	#else
		#define WF_INT_TRIS			(TRISAbits.TRISA14)	// INT3
		#define WF_INT_IO			(PORTAbits.RA14)
	#endif

	#if defined (__C32__)
		#define WF_INT_EDGE			(INTCONbits.INT3EP)
		#define WF_INT_IE			(IEC0bits.INT3IE)
		#define WF_INT_IF			(IFS0bits.INT3IF)
		#define WF_INT_IE_CLEAR		IEC0CLR
		#define WF_INT_IF_CLEAR		IFS0CLR
		#define WF_INT_IE_SET		IEC0SET
		#define WF_INT_IF_SET		IFS0SET
		#define WF_INT_BIT			0x00008000
		#define WF_INT_IPCSET		IPC3SET
		#define WF_INT_IPCCLR		IPC3CLR
		#define WF_INT_IPC_MASK		0xFF000000
		#define WF_INT_IPC_VALUE	0x0C000000
	#elif defined (__C30__)
		#define WF_INT_EDGE			(INTCON2bits.INT3EP)
		#define WF_INT_IE			(IEC3bits.INT3IE)
		#define WF_INT_IF			(IFS3bits.INT3IF)
	#endif
#elif defined(MRF24W_USE_INT1_INT)      

	#if defined(__dsPIC33FJ256GP710__) || defined(__PIC24HJ256GP610__) || defined (__dsPIC33E__)||defined(__PIC24E__)
		#define WF_INT_TRIS	    (TRISAbits.TRISA12)
		#define WF_INT_IO		(PORTAbits.RA12)
	#else
		#define WF_INT_TRIS	    (TRISEbits.TRISE8)  // INT1
		#define WF_INT_IO		(PORTEbits.RE8)
	#endif

	#if defined (__C32__)
		#define WF_INT_EDGE			(INTCONbits.INT1EP) 
		#define WF_INT_IE			(IEC0bits.INT1IE)
		#define WF_INT_IF			(IFS0bits.INT1IF)
		#define WF_INT_IE_CLEAR		IEC0CLR
		#define WF_INT_IF_CLEAR		IFS0CLR
		#define WF_INT_IE_SET		IEC0SET
		#define WF_INT_IF_SET		IFS0SET
		#define WF_INT_BIT			0x00000080
		#define WF_INT_IPCSET		IPC1SET
		#define WF_INT_IPCCLR		IPC1CLR
		#define WF_INT_IPC_MASK		0xFF000000
		#define WF_INT_IPC_VALUE	0x0C000000
	#elif defined (__C30__)
		#define WF_INT_TRIS			(TRISAbits.TRISA12)// INT1
		#define WF_INT_IO			(PORTAbits.RA12)
		#define WF_INT_EDGE			(INTCON2bits.INT1EP)
		#define WF_INT_IE			(IEC1bits.INT1IE)
		#define WF_INT_IF			(IFS1bits.INT1IF)
	#endif
#else
	#error "No appropriate WIFI interrupt selected"
#endif

#endif /* TCPIP_IF_MRF24W */

#endif

