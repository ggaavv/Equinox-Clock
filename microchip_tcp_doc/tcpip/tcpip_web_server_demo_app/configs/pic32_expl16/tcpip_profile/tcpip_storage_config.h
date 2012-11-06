/*******************************************************************************
  TCPIP Storage Configuration file

  Summary:
    TCPIP Storage configuration file
    
  Description:
    This file contains the TCPIP Storage module configuration options
    
*******************************************************************************/

/*******************************************************************************
FileName:  tcpip_storage_config.h 
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

#ifndef _TCPIP_STORAGE_CONFIG_H_
#define _TCPIP_STORAGE_CONFIG_H_


// provide a way for the user to store proprietary data bytes in the storage

// enforces the checking of the build version that's stored
// in the external storage
// If enabled, a newer build of the stack will override
// the saved configuration
// Needs TCPIP_STACK_USE_STORAGE to be defined
// Also the hardware board has to support an external EEPROM/Flash storage
#define TCPIP_STORAGE_STACK_CHECK_BUILD_VERSION           1


// size of the proprietary/extra data field, in bytes
// that will be saved together with the stack configuration.
// if 0, then no additional data is stored
// NOTE: changing the size of the label size will invalidate the storage!
// When TCPIP_STORAGE_STACK_CHECK_BUILD_VERSION is enabled the stack will use
// a 16 bit checksum to validate the build
// The size specified here is ignored and the storage label will be 2 bytes long
#define TCPIP_STORAGE_USER_LABEL_SIZE   		2


#endif  // _TCPIP_STORAGE_CONFIG_H_
