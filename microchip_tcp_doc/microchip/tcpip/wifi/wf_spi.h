/*******************************************************************************
  Wifi SPI header for PIC32

  Summary:
    
  Description:
*******************************************************************************/

/*******************************************************************************
FileName:  WF_Spi.h 
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

#ifndef _WF_SPI_H_
#define _WF_SPI_H_

#if defined(TCPIP_IF_MRF24W)

#if defined (__PIC24F__)
	#define WF_MAX_SPI_FREQ 		(8000000ul)	// Hz
#else
	#define WF_MAX_SPI_FREQ 		(10000000ul)	// Hz	
#endif
// Using IO  as SPI chip select
#define WF_CS_Init()			{WF_CS_TRIS = 0;}
#define WF_CS_Assert()			{WF_CS_IO   = 0;}
#define WF_CS_Deassert()		{WF_CS_IO   = 1;}

#endif /* TCPIP_IF_MRF24W */

#endif

