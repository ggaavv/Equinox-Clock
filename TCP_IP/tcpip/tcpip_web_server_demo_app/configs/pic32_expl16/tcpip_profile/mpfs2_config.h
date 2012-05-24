/*******************************************************************************
  Microchip File System (MPFS) Configuration file

  Summary:
    MPFS configuration file
    
  Description:
    This file contains the MPFS module configuration options
    
*******************************************************************************/

/*******************************************************************************
FileName:  mpfs2_config.h 
Copyright © 2011 released Microchip Technology Inc.  All rights
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

#ifndef _MPFS_CONFIG_H_
#define _MPFS_CONFIG_H_


//Supports long file names to 64 characters
#define MAX_FILE_NAME_LEN				(64u)

// MPFS Storage Location
// 	If html pages are stored in internal program memory,
// 	comment both MPFS_USE_EEPROM and MPFS_USE_SPI_FLASH, then
// 	include an MPFS image (.c or .s file) in the project.
// 	If html pages are stored in external memory, uncomment the
// 	appropriate definition.
// 	Supported serial flash parts include the SST25VFxxxB series.
#define MPFS_USE_EEPROM
//#define MPFS_USE_SPI_FLASH

// EEPROM Addressing Selection
// 	If using the 1Mbit EEPROM, uncomment this line
//#define USE_EEPROM_25LC1024

// EEPROM Reserved Area
// 	Number of EEPROM bytes to be reserved before MPFS storage starts.
// 	These bytes host application configurations such as IP Address,
// 	MAC Address, and any other required variables.
//
// 	For MPFS Classic, this setting must match the Reserved setting
// 	on the Advanced Settings page of the MPFS2 Utility.
// 	Note: Depending on the enabled services, features and interfaces 
// 	the storage needed for configuration increases.
// 	Adjust accordingly!
#define MPFS_RESERVE_BLOCK				(200ul)

// MPFS File Handles
// 	Maximum number of simultaneously open MPFS2 files.
// 	For MPFS Classic, this has no effect
#define MAX_MPFS_HANDLES				(17ul)

/*	
#if defined(TCPIP_STACK_USE_MPFS) && defined(TCPIP_STACK_USE_MPFS2)
	#error Both MPFS and MPFS2 are included
#endif
*/

#if defined(MPFS_USE_EEPROM)
	#if defined(USE_EEPROM_25LC1024)
		// Defines the size of a page in EEPROM
		#define MPFS_WRITE_PAGE_SIZE		(256u)
	#else
		// Defines the size of a page in EEPROM
		#define MPFS_WRITE_PAGE_SIZE		(64u)	
	#endif
#endif



#endif // _MPFS_CONFIG_H_
