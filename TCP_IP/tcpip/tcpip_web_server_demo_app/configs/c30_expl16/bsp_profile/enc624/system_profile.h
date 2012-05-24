/*******************************************************************************

  Summary:
    
  Description:
*******************************************************************************/

/*******************************************************************************
FileName:  system_profile.h 
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

#ifndef __SYSTEM_PROFILE_H_
#define __SYSTEM_PROFILE_H_

/*****************************************************************************
 * The number of ticks per second. 
 * This is used to define timeouts throughout the system.
 * This implementation uses the core timer
 *****************************************************************************/
#define SYS_TICKS_PER_SECOND        500

/*****************************************************************************
 * The interrupt priority (1-lowest to 7-highest) and sub-priority (0 to 3) for the timer used to generate the system ticks
 *****************************************************************************/
#define	SYS_TICK_IPL		4
#define	SYS_TICK_SIPL		1


/*****************************************************************************
 * The system debug enable
 * Use to enable the debug messages.
 *****************************************************************************/
//#define SYS_DEBUG_ENABLE


/*****************************************************************************
 * The current system error level
 * System error functions use this level 
 * to suppress/ or not the output of a
 * SYS_ERROR message.
 *****************************************************************************/
//#define SYSTEM_CURRENT_ERROR_LEVEL  SYS_ERROR_DEBUG



/*****************************************************************************
 * The system debug peripheral
 * Specify the debug port.
 *****************************************************************************/
//#define SYS_DEBUG_PORT          SYS_MODULE_UART_2

/*****************************************************************************
 * The system debug peripheral
 * Specify the debug port baud rate.
 *****************************************************************************/
//#define SYS_DEBUG_BAUDRATE      19200

/*****************************************************************************
 * The system console enable
 * Use to enable the system console
 *****************************************************************************/
//#define SYS_CONSOLE_ENABLE


/*****************************************************************************
 * The system console comm channel
 * Specify the console port.
 *****************************************************************************/
//#define SYS_CONSOLE_PORT          SYS_MODULE_UART_2

/*****************************************************************************
 * The system console comm channel
 * Specify the console bit rate.
 *****************************************************************************/
//#define SYS_CONSOLE_BAUDRATE     19200

/*****************************************************************************
 * The system console buffer space
 * Specify the length of the buffering for the console operations.
 *****************************************************************************/
//#define SYS_CONSOLE_BUFFER_LEN   200



/*****************************************************************************
 * Enable the system output functionality.
 * The output port is usually imnplemented
 * using an LCD on boards that support it.
 *****************************************************************************/
#define SYS_OUT_ENABLE

/*****************************************************************************
 * The system out channel
 * Specify the output port.
 *****************************************************************************/
#define SYS_OUT_PORT          SYS_MODULE_CHAR_LCD

/*****************************************************************************
 * The system random module
 * Use to enable the system random functionality
 *****************************************************************************/
#define SYS_RANDOM_ENABLE


#endif	// __SYSTEM_PROFILE_H_

