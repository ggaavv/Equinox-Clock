/*******************************************************************************
  Data SPI RAM Access Routines

  Summary:
    
  Description:
    -Tested with AMI Semiconductor N256S0830HDA
*******************************************************************************/

/*******************************************************************************
FileName:  spi_ram.h 
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

#ifndef __SPIRAM_H
#define __SPIRAM_H

#include "hardware_profile.h"

#if defined(SPIRAM_CS_TRIS)
	void SPIRAMInit(void);
	void SPIRAMGetArray(uint16_t wAddress, uint8_t *vData, uint16_t wLength);
	void SPIRAMPutArray(uint16_t wAddress, const uint8_t *vData, uint16_t wLength);
	
	#define SPIRAMPutString(a,b)			SPIRAMPutArray(a, strlen((char*)b))
	
#else
	// If you get any of these linker errors, it means that you either have an 
	// error in your hardware_profile.h or tcpip_config.h definitions.  The code 
	// is attempting to call a function that can't possibly work because you 
	// have not specified what pins and SPI module the physical SPI SRAM chip 
	// is connected to.  Alternatively, if you don't have an SPI SRAM chip, it 
	// means you have enabled a stack feature that requires SPI SRAM hardware.
	// In this case, you need to edit tcpip_config.h and disable this stack 
	// feature.  The linker error tells you which object file this error was 
	// generated from.  It should be a clue as to what feature you need to 
	// disable.
	void You_cannot_call_the_SPIRAMInit_function_without_defining_SPIRAM_CS_TRIS_in_HardwareProfile_h_first(void);
	void You_cannot_call_the_SPIRAMGetArray_function_without_defining_SPIRAM_CS_TRIS_in_HardwareProfile_h_first(void);
	void You_cannot_call_the_SPIRAMPutArray_function_without_defining_SPIRAM_CS_TRIS_in_HardwareProfile_h_first(void);
	void You_cannot_call_the_SPIRAMPutString_function_without_defining_SPIRAM_CS_TRIS_in_HardwareProfile_h_first(void);
	void You_cannot_call_the_SPIRAMPutROMArray_function_without_defining_SPIRAM_CS_TRIS_in_HardwareProfile_h_first(void);
	void You_cannot_call_the_SPIRAMPutROMString_function_without_defining_SPIRAM_CS_TRIS_in_HardwareProfile_h_first(void);
	#define SPIRAMInit()				You_cannot_call_the_SPIRAMInit_function_without_defining_SPIRAM_CS_TRIS_in_HardwareProfile_h_first()
	#define SPIRAMGetArray(a,b,c)		You_cannot_call_the_SPIRAMGetArray_function_without_defining_SPIRAM_CS_TRIS_in_HardwareProfile_h_first()
	#define SPIRAMPutArray(a,b,c)		You_cannot_call_the_SPIRAMPutArray_function_without_defining_SPIRAM_CS_TRIS_in_HardwareProfile_h_first()
	#define SPIRAMPutString(a,b)		You_cannot_call_the_SPIRAMPutString_function_without_defining_SPIRAM_CS_TRIS_in_HardwareProfile_h_first()
#endif


#endif
