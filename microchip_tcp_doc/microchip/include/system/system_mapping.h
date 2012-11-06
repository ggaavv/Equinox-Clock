/*******************************************************************************

  Summary: System Debugging and messaging interface
    
  Description:

*******************************************************************************/

/*******************************************************************************
FileName: 	system_mapping.h
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


#ifndef _SYSTEM_MAPPING_H_
#define _SYSTEM_MAPPING_H_

#include <stdint.h>
#include <stdbool.h>
#include <stdarg.h>
#include "system_profile.h"


#include "../../system/system_debug_private.h"
#include "../../system/system_random_private.h"


// *****************************************************************************
// *****************************************************************************
// Section: System Interface Functions
// *****************************************************************************
// *****************************************************************************

/*********************************************************************
 * Function:        void SYS_ASSERT(bool testExpr, const char* message)
 *
 * PreCondition:    None
 *
 * Input:           None
 *
 * Output:          If the test expression is false
 *                  displays a message to the system console and
 *                  halts the system 
 *
 * Side Effects:    None
 *
 * Overview:        This function performs an assert and halts the system
 *                  if the test expression is false.
 *
 * Note:            None
 ********************************************************************/
#if defined(SYS_DEBUG_ENABLE) 
    #define SYS_ASSERT(testExpr, message)   do{if((testExpr) == false) _SYS_ASSERT(__LINE__, __FILE__, message); }while(0)
#else
    #define SYS_ASSERT(testExpr, message)
#endif

/*********************************************************************
 * Function:        void SYS_ERROR_MESSAGE(SYS_ERROR_LEVEL level, const char *message)
 *
 * PreCondition:    None
 *
 * Input:           None
 *
 * Output:          It displays an error message to the system console
 *
 * Side Effects:    None
 *
 * Overview:        If the current system error level is less than the level parameter,
 *                  the function will display the message to the system console
 *
 * Note:            None
 ********************************************************************/
#if defined(SYS_DEBUG_ENABLE) 
    #define SYS_ERROR(level, message)   do{if( (level) < SYSTEM_CURRENT_ERROR_LEVEL) _SYS_ERROR(__LINE__, __FILE__, message); }while(0)
#else
    #define SYS_ERROR(level, message)
#endif

/*********************************************************************
 * Function:        void SYS_ERROR_PRINT(SYS_ERROR_LEVEL level, const char *format, ...)
 *
 * PreCondition:    None
 *
 * Input:           None
 *
 * Output:          It displays a formatted error message to the system console
 *
 * Side Effects:    None
 *
 * Overview:        If the current system error level is less than the level parameter,
 *                  the function will display the message to the system console
 *
 * Note:            None
 ********************************************************************/
#if defined(SYS_DEBUG_ENABLE) 
	#define		SYS_ERROR_PRINT(level, format, ...) _SYS_ERROR_PRINT(level, format, __VA_ARGS__)
#else
	#define		SYS_ERROR_PRINT(level, format, ...)
#endif

#if defined(SYS_DEBUG_ENABLE) 
	#define		SYS_PRINT(format, ...) 	_SYS_PRINT(format, __VA_ARGS__)
#else
	#define		SYS_PRINT(level, format, ...)
#endif

/*********************************************************************
 * Function:        void SYS_CONSOLE_MESSAGE(const char *message)
 *
 * PreCondition:    None
 *
 * Input:           None
 *
 * Output:          It displays a formatted message to the system console
 *
 * Side Effects:    None
 *
 * Overview:        The function will display the message to the system console
 *
 * Note:            None
 ********************************************************************/
#if defined(SYS_CONSOLE_ENABLE)
	#define 	SYS_CONSOLE_MESSAGE(message) _SYS_CONSOLE_MESSAGE(message)
#else
	#define 	SYS_CONSOLE_MESSAGE(message)
#endif


/*********************************************************************
 * Function:        void SYS_CONSOLE_PRINT(const char *format, ...)
 *
 * PreCondition:    None
 *
 * Input:           None
 *
 * Output:          It displays a formatted message to the system console
 *
 * Side Effects:    None
 *
 * Overview:        The function will display the message to the system console
 *
 * Note:            None
 ********************************************************************/
#if defined(SYS_CONSOLE_ENABLE)
	#define 	SYS_CONSOLE_PRINT(format, ...) _SYS_CONSOLE_PRINT(format, __VA_ARGS__)
#else
	#define 	SYS_CONSOLE_PRINT(format, ...)
#endif

#if defined(SYS_CONSOLE_ENABLE)
	#define 	SYS_CONSOLE_DATA_RDY() _SYS_CONSOLE_DATA_RDY()
#else
	#define 	SYS_CONSOLE_DATA_RDY()
#endif

#if defined(SYS_CONSOLE_ENABLE)
	#define 	SYS_CONSOLE_GETC() _SYS_CONSOLE_GETC()
#else
	#define 	SYS_CONSOLE_GETC()
#endif

#if defined(SYS_CONSOLE_ENABLE)
	#define 	SYS_CONSOLE_PUTC(c) _SYS_CONSOLE_PUTC(c)
#else
	#define 	SYS_CONSOLE_PUTC()
#endif


#if defined(SYS_CONSOLE_ENABLE) 
	#define 	SYS_CONSOLE_KBHIT() _SYS_CONSOLE_KBHIT()
#else
	#define 	SYS_CONSOLE_KBHIT() 0
#endif

#if defined(SYS_CONSOLE_ENABLE) 
	#define 	SYS_CONSOLE_GETLINE(buffer, bufferLen) _SYS_CONSOLE_GETLINE(buffer, bufferLen)
#else
	#define 	SYS_CONSOLE_GETLINE(buffer, bufferLen) 0
#endif

#if defined(SYS_CONSOLE_ENABLE) 
	#define 	SYS_CONSOLE_SCANF(format, ...) _SYS_CONSOLE_SCANF(format, __VA_ARGS__)
#else
	#define 	SYS_CONSOLE_SCANF(format, ...)
#endif

#if defined(SYS_OUT_ENABLE) 
	#define 	SYS_OUT_MESSAGE(msg) _SYS_OUT_MESSAGE(msg)
#else
	#define 	SYS_OUT_MESSAGE(msg)
#endif

#if defined(SYS_OUT_ENABLE) 
	#define 	SYS_OUT_MESSAGE_LINE(msg, line) _SYS_OUT_MESSAGE_LINE(msg, line)
#else
	#define 	SYS_OUT_MESSAGE_LINE(msg, line) 
#endif

#if defined(SYS_OUT_ENABLE) 
	#define 	SYS_OUT_MESSAGE_OFFSET(msg, offset) _SYS_OUT_MESSAGE_OFFSET(msg, offset)
#else
	#define 	SYS_OUT_MESSAGE_OFFSET(msg, offset)
#endif

#if defined(SYS_OUT_ENABLE) 
	#define 	SYS_OUT_MESSAGE_LINE_COUNT() _SYS_OUT_MESSAGE_LINE_COUNT()
#else
	#define 	SYS_OUT_MESSAGE_LINE_COUNT() 0
#endif


#if defined(SYS_OUT_ENABLE) 
	#define 	SYS_OUT_MESSAGE_LINE_LENGTH() _SYS_OUT_MESSAGE_LINE_LENGTH()
#else
	#define 	SYS_OUT_MESSAGE_LINE_LENGTH() 0
#endif

#endif  // _SYSTEM_MAPPING_H_

