/*******************************************************************************
  SPI Flash Memory Driver Header

  Summary:
    SUMMARY
    
  Description:
    - Tested to be compatible with SST25VF016B
    - Expected compatibility with other SST (Microchip) SST25 series 
      devices
*******************************************************************************/

/*******************************************************************************
FileName:  spi_flash.h 
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

#ifndef __SPIFLASH_H
#define __SPIFLASH_H

#include "hardware_profile.h"

#define SPI_FLASH_SECTOR_SIZE		(4096ul)
#define SPI_FLASH_PAGE_SIZE			(0ul)		// SST has no page boundary requirements

#define SPI_FLASH_SECTOR_MASK		(SPI_FLASH_SECTOR_SIZE - 1)


#if defined(SPIFLASH_CS_TRIS)
	void SPIFlashInit(void);		
	void SPIFlashReadArray(uint32_t dwAddress, uint8_t *vData, uint16_t wLen);
	void SPIFlashBeginWrite(uint32_t dwAddr);
	void SPIFlashWrite(uint8_t vData);
	void SPIFlashWriteArray(uint8_t *vData, uint16_t wLen);
	void SPIFlashEraseSector(uint32_t dwAddr);
#else
	// If you get any of these linker errors, it means that you either have an 
	// error in your hardware_profile.h or tcpip_config.h definitions.  The code 
	// is attempting to call a function that can't possibly work because you 
	// have not specified what pins and SPI module the physical SPI Flash chip 
	// is connected to.  Alternatively, if you don't have an SPI Flash chip, it 
	// means you have enabled a stack feature that requires SPI Flash hardware.
	// In this case, you need to edit tcpip_config.h and disable this stack 
	// feature.  The linker error tells you which object file this error was 
	// generated from.  It should be a clue as to what feature you need to 
	// disable.
	void You_cannot_call_the_SPIFlashInit_function_without_defining_SPIFLASH_CS_TRIS_in_HardwareProfile_h_first(void);
	void You_cannot_call_the_SPIFlashReadArray_function_without_defining_SPIFLASH_CS_TRIS_in_HardwareProfile_h_first(void);
	void You_cannot_call_the_SPIFlashBeginWrite_function_without_defining_SPIFLASH_CS_TRIS_in_HardwareProfile_h_first(void);
	void You_cannot_call_the_SPIFlashWrite_function_without_defining_SPIFLASH_CS_TRIS_in_HardwareProfile_h_first(void);
	void You_cannot_call_the_SPIFlashWriteArray_function_without_defining_SPIFLASH_CS_TRIS_in_HardwareProfile_h_first(void);
	void You_cannot_call_the_SPIFlashEraseSector_function_without_defining_SPIFLASH_CS_TRIS_in_HardwareProfile_h_first(void);
	#define SPIFlashInit()				You_cannot_call_the_SPIFlashInit_function_without_defining_SPIFLASH_CS_TRIS_in_HardwareProfile_h_first()
	#define SPIFlashReadArray(a,b,c)	You_cannot_call_the_SPIFlashReadArray_function_without_defining_SPIFLASH_CS_TRIS_in_HardwareProfile_h_first()
	#define SPIFlashBeginWrite(a)		You_cannot_call_the_SPIFlashBeginWrite_function_without_defining_SPIFLASH_CS_TRIS_in_HardwareProfile_h_first()
	#define SPIFlashWrite(a)			You_cannot_call_the_SPIFlashWrite_function_without_defining_SPIFLASH_CS_TRIS_in_HardwareProfile_h_first()
	#define SPIFlashWriteArray(a,b)		You_cannot_call_the_SPIFlashWriteArray_function_without_defining_SPIFLASH_CS_TRIS_in_HardwareProfile_h_first()
	#define SPIFlashEraseSector(a)		You_cannot_call_the_SPIFlashEraseSector_function_without_defining_SPIFLASH_CS_TRIS_in_HardwareProfile_h_first()
#endif

#endif
