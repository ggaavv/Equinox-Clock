/*******************************************************************************

  Summary: System Debugging and messaging interface
    
  Description:

*******************************************************************************/

/*******************************************************************************
FileName: 	system_debug.h
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


#ifndef _SYSTEM_DEBUG_H_
#define _SYSTEM_DEBUG_H_


// *****************************************************************************
/* SYS_ERROR_LEVEL enumeration

   Summary:
    System error levels

   Description:
    This enumeration type defines the current system error level.

   Remarks:
    None
*/

	typedef enum 
	{
		SYS_ERROR_NONE	/*DOM-IGNORE-BEGIN*/ = 0 /*DOM-IGNORE-END*/,

		SYS_ERROR_FATAL	/*DOM-IGNORE-BEGIN*/ = 1 /*DOM-IGNORE-END*/,

		SYS_ERROR_ERROR	/*DOM-IGNORE-BEGIN*/ = 2 /*DOM-IGNORE-END*/,

		SYS_ERROR_WARN	/*DOM-IGNORE-BEGIN*/ = 3 /*DOM-IGNORE-END*/,

		SYS_ERROR_INFO	/*DOM-IGNORE-BEGIN*/ = 4 /*DOM-IGNORE-END*/,

		SYS_ERROR_DEBUG	/*DOM-IGNORE-BEGIN*/ = 5 /*DOM-IGNORE-END*/	

	} SYS_ERROR_LEVEL;

// *****************************************************************************



// *****************************************************************************
// *****************************************************************************
// Section: System Interface Functions
// *****************************************************************************
// *****************************************************************************

/*********************************************************************
 * Function:        void SYS_ASSERT(bool testExpr, const char *message)
 *
 * PreCondition:    None
 *
 * Input:           None
 *
 * Output:          If testExpr is false it displays a message to the system console and
 *                  halts the system 
 *
 * Side Effects:    None
 *
 * Overview:        This function performs an assert and halts the system
 *                  if the test expression is false.
 *
 * Note:            None
 ********************************************************************/
void        SYS_ASSERT(bool testExpr, const char *message);

/*********************************************************************
 * Function:        void SYS_ERROR(SYS_ERROR_LEVEL level, const char *message)
 *
 * PreCondition:    None
 *
 * Input:           None
 *
 * Output:          It displays an error message to the system console
 *
 * Side Effects:    None
 *
 * Overview:        The function will display the message to the system console
 *
 * Note:            None
 ********************************************************************/
void        SYS_ERROR(SYS_ERROR_LEVEL level, const char *message);

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
void 		SYS_ERROR_PRINT(SYS_ERROR_LEVEL level, const char *format, ...);

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
void 		SYS_CONSOLE_MESSAGE(const char *message);


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
void        SYS_CONSOLE_PRINT(const char *format, ...);

/*********************************************************************
 * Function:		char SYS_CONSOLE_DATA_RDY(void)
 *
 * PreCondition:	None
 *
 * Input:			None
 *
 * Output:			true or false
 *
 * Side Effects:	None
 *
 * Overview:		The function will return the status of console receive
 *					
 *
 * Note:			None
 ********************************************************************/
bool SYS_CONSOLE_DATA_RDY();


/*********************************************************************
 * Function:        char SYS_CONSOLE_GETC(void)
 *
 * PreCondition:    None
 *
 * Input:           None
 *
 * Output:          It gets a character from the system console
 *
 * Side Effects:    None
 *
 * Overview:        The function will display the message to the 
 *					system console.
 * Overview:        If the current system error level is less than the level parameter,
 *                  the function will display the message to the system console
 *
 * Note:            None
 ********************************************************************/
char 		SYS_CONSOLE_GETC();

/*********************************************************************
 * Function:		char SYS_CONSOLE_PUTC(void)
 *
 * PreCondition:	None
 *
 * Input:			c
 *
 * Output:			It puts a character out to the system console
 *
 * Side Effects:	None
 *
 * Overview:		The function will display the character to the 
 *					system console.
 *
 * Note:			None
 ********************************************************************/
void        SYS_CONSOLE_PUTC(char c);


/*********************************************************************
 * Function:        int SYS_CONSOLE_KBHIT()
 *
 * PreCondition:    None
 *
 * Input:           None
 *
 * Output:          It checks for key hit from the console
 *
 * Side Effects:    None
 *
 * Overview:        The function returns the pending status of the 
 *					input from system console.
 *
 * Note:            None
 ********************************************************************/
int 		SYS_CONSOLE_KBHIT();

/*********************************************************************
 * Function:        int SYS_CONSOLE_GETLINE(char* line, int bufferLen)
 *
 * PreCondition:    None
 *
 * Input:           None
 *
 * Output:          It gets a line from the system console
 *
 * Side Effects:    None
 *
 * Overview:        The function will display the message to the 
 *					system console.
 *
 * Note:            None
 ********************************************************************/
int 		SYS_CONSOLE_GETLINE(char* buffer, int bufferLen);

/*********************************************************************
 * Function:        int SYS_CONSOLE_SCANF(const char *format, ...)
 *
 * PreCondition:    None
 *
 * Input:           None
 *
 * Output:          It scans inputs from the system console
 *
 * Side Effects:    None
 *
 * Overview:        The function scans formatted inputs from the  
 *					system console.
 *
 * Note:            None
 ********************************************************************/
int 		SYS_CONSOLE_SCANF(const char *format, ...);

/*********************************************************************
 * Function:        void SYS_OUT_MESSAGE(const char* msg)
 *
 * PreCondition:    None
 *
 * Input:           None
 *
 * Output:          It displays a message to the system output
 *
 * Side Effects:    None
 *
 * Overview:        The function will display the message to the
 *					system output.
 *
 * Note:            None
 ********************************************************************/
void 		SYS_OUT_MESSAGE(const char *msg);


/*********************************************************************
 * Function:        void SYS_OUT_MESSAGE_LINE(const char *msg, int line)
 *
 * PreCondition:    None
 *
 * Input:           None
 *
 * Output:          It displays a message to the system output on the 
 *					specified line
 *
 * Side Effects:    None
 *
 * Overview:        The function will display the message to the
 *					system output on the specified line.
 *
 * Note:            None
 ********************************************************************/
void 		SYS_OUT_MESSAGE_LINE(const char *msg, int line);

/*********************************************************************
 * Function:        void SYS_OUT_MESSAGE_OFFSET(const char *msg, int offset)
 *
 * PreCondition:    None
 *
 * Input:           None
 *
 * Output:          It displays a message to the system output starting  
 *					at the specified offset
 *
 * Side Effects:    None
 *
 * Overview:        The function will display the message to the
 *					system output on the specified offset.
 *
 * Note:            None
 ********************************************************************/
void 		SYS_OUT_MESSAGE_OFFSET(const char *msg, int offset);

/*********************************************************************
 * Function:        int SYS_OUT_MESSAGE_LINE_COUNT(void)
 *
 * PreCondition:    None
 *
 * Input:           None
 *
 * Output:          It returns number of characters per line available
 *					for system output
 *
 * Side Effects:    None
 *
 * Overview:        The function returns number of characters per 
 *					line available for system output
 *
 * Note:            None
 ********************************************************************/
 int 		SYS_OUT_MESSAGE_LINE_COUNT(void);


/*********************************************************************
 * Function:        int SYS_OUT_MESSAGE_LINE_LENGTH(void)
 *
 * PreCondition:    None
 *
 * Input:           None
 *
 * Output:          It returns number of linee available
 *					for system output
 *
 * Side Effects:    None
 *
 * Overview:        The function returns number lines 
 *					available for system output
 *
 * Note:            None
 ********************************************************************/
 int 		SYS_OUT_MESSAGE_LINE_LENGTH(void);


#include "system/system_mapping.h"


#endif  // _SYSTEM_DEBUG_H_

