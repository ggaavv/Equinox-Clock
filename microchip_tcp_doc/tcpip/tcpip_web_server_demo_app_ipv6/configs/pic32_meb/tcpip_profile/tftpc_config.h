/*******************************************************************************
  Trivial File Transfer Protocol (TFTP) Configuration file

  Summary:
    (TFTP) Configuration file
    
  Description:
    This file contains the TFTP module configuration options
    
*******************************************************************************/

/*******************************************************************************
FileName:   tftpc_config.h
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

#ifndef _TFTP_CONFIG_H_
#define _TFTP_CONFIG_H_

// Number of seconds to wait before declaring TIMEOUT error on Get, seconds.
#define TFTP_GET_TIMEOUT_VAL        (3ul)

// Number of seconds to wait before declaring TIMEOUT error on Put, seconds
#define TFTP_ARP_TIMEOUT_VAL        (3ul)

// Number of attempts before declaring TIMEOUT error.
#define TFTP_MAX_RETRIES            (3u)

// The TFTP Client port - a unique port on this device
#define TFTP_CLIENT_PORT        65352L       

// The TFTP Server Port
#define TFTP_SERVER_PORT        (69L)

// The size of a TFTP block - 512 bytes
#define TFTP_BLOCK_SIZE         (0x200L)

// The MSB of the TFTP_BLOCK_SIZE
#define TFTP_BLOCK_SIZE_MSB     (0x02u)

#endif  // _TFTP_CONFIG_H_
