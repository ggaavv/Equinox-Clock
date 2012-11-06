/*******************************************************************************

  Summary: System Debugging and messaging interface
    
  Description:

*******************************************************************************/

/*******************************************************************************
FileName:	system_debug.c
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
#include <stdlib.h>
#include <stdio.h>
#include <string.h>

#include "system/system_services.h"
#include "system_profile.h"
#include "system_debug_private.h"

#include "./drivers/usart.h"
#include "./drivers/lcd.h"


#if defined (__C32__)
extern __inline__ void __attribute__((always_inline, nomips16)) __sys_assert(bool smth)
{
		if((smth)==false)
		{
				while(1);
		}
}
#elif defined (__C30__)
extern __inline__ void __attribute__((always_inline)) __sys_assert(bool smth)
{
		if((smth)==false)
		{
					while(1);
		}
}
#endif



void* debug_handle = NULL;
void* console_handle = NULL;

/*********************************************************************
 * Function:        void* _SYS_DEBUG_OPEN(SYS_MODULE_ID debug_port)
 *
 * PreCondition:    None
 *
 * Input:           None
 *
 * Output:          It opens the system debug module.
 *
 * Side Effects:    None
 *
 * Overview:        The function opens the system debug module.
 *
 * Note:            None
 ********************************************************************/
#if defined(SYS_DEBUG_ENABLE) 
void* _SYS_DEBUG_OPEN(SYS_MODULE_ID debug_port)
{
	return((void*)USART_OPEN(debug_port));
}
#endif


/*********************************************************************
 * Function:        bool _SYS_DEBUG_INIT(int debug_port)
 *
 * PreCondition:    None
 *
 * Input:           None
 *
 * Output:          It initializes the system debug module.
 * Side Effects:    None
 *
 * Overview:        The function initializes the system debug module.
 *
 * Note:            None
 ********************************************************************/
#if defined(SYS_DEBUG_ENABLE) 
bool _SYS_DEBUG_INIT(int debug_port)
{
    bool initRes = USART_INIT(debug_port, SYS_DEBUG_BAUDRATE);
    if(initRes)
    {
        debug_handle = _SYS_DEBUG_OPEN(debug_port);
        return debug_handle != 0;
    }

    return false;
}    
#endif


/*********************************************************************
 * Function:        void _SYS_ASSERT(int linenumber, const char *filename, const char *message)
 *
 * PreCondition:    None
 *
 * Input:           None
 *
 * Output:          It displays a message to the system console and
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
void _SYS_ASSERT(int linenumber, const char* filename, const char* message)
{
	char buff[SYS_CONSOLE_BUFFER_LEN];
	snprintf(buff, SYS_CONSOLE_BUFFER_LEN, "\nSYS_ASSERT occurred in file %s on line %d. ", filename, linenumber);
	USART_PRINT((USART_MODULE_CTRL*)debug_handle, buff);
	USART_PRINT((USART_MODULE_CTRL*)debug_handle, (char *)message);	
	__sys_assert(0);
}
#endif  // defined(SYS_DEBUG_ENABLE) 


/*********************************************************************
 * Function:        void _SYS_ERROR(int linenumber, const char *filename, const char *message)
 *
 * PreCondition:    None
 *
 * Input:           None
 *
 * Output:          It displays an error message to the system console
 *
 * Side Effects:    None
 *
 * Overview:        The function wil display the message to the system console
 *
 * Note:            None
 ********************************************************************/
#if defined(SYS_DEBUG_ENABLE) 
void _SYS_ERROR(int linenumber, const char* filename, const char* message)
{
	char buff[SYS_CONSOLE_BUFFER_LEN];
	snprintf(buff, SYS_CONSOLE_BUFFER_LEN, "\nSYS_ERROR occurred in file %s on line %d. ", filename, linenumber);
	USART_PRINT((USART_MODULE_CTRL*)debug_handle, buff);
	USART_PRINT((USART_MODULE_CTRL*)debug_handle, (char *)message);
}
#endif  // defined(SYS_DEBUG_ENABLE) 



/*********************************************************************
 * Function:        void _SYS_ERROR_PRINT(SYS_ERROR_LEVEL level, 
 *										const char* format, ...)
 *
 * PreCondition:    None
 *
 * Input:           None
 *
 * Output:          It displays a formatted message to the system debug port
 *
 * Side Effects:    None
 *
 * Overview:        The function wil display the message to the system debug port
 *
 * Note:            None
 ********************************************************************/
#if defined(SYS_DEBUG_ENABLE) 
void _SYS_ERROR_PRINT(SYS_ERROR_LEVEL level, const char* format, ...)
{
    char buff[SYS_CONSOLE_BUFFER_LEN];
    va_list args; 
    if( (level) < SYSTEM_CURRENT_ERROR_LEVEL){
		va_start( args, format ); 
		
		snprintf(buff, SYS_CONSOLE_BUFFER_LEN, "\nSYS_ERROR_PRINT issued: ");
		USART_PRINT((USART_MODULE_CTRL*)debug_handle, buff);
		vsnprintf(buff, SYS_CONSOLE_BUFFER_LEN, format, args);
		USART_PRINT((USART_MODULE_CTRL*)debug_handle, buff);
		
		va_end( args );	
    }
}
#endif  // defined(SYS_DEBUG_ENABLE) 


/*********************************************************************
 * Function:        void _SYS_PRINT(const char *format, ...)
 *
 * PreCondition:    None
 *
 * Input:           None
 *
 * Output:          It displays a formatted message to the system console
 *
 * Side Effects:    None
 *
 * Overview:        The function wil display the message to the system console
 *
 * Note:            None
 ********************************************************************/
 #if defined(SYS_DEBUG_ENABLE) 
void _SYS_PRINT(const char* format, ...)
{	
    char buff[SYS_CONSOLE_BUFFER_LEN];
    va_list args; 
    va_start( args, format ); 
    
	snprintf(buff, SYS_CONSOLE_BUFFER_LEN, "\nSYS_PRINT issued:");
	USART_PRINT((USART_MODULE_CTRL*)debug_handle, buff);
	vsnprintf(buff, SYS_CONSOLE_BUFFER_LEN, format, args);
    USART_PRINT((USART_MODULE_CTRL*)debug_handle, buff);
    
    va_end( args );        
}
#endif

/*********************************************************************
 * Function:        void* _SYS_CONSOLE_OPEN(SYS_MODULE_ID console_port)
 *
 * PreCondition:    None
 *
 * Input:           None
 *
 * Output:          It opens the system console module.
 *
 * Side Effects:    None
 *
 * Overview:        The function opens the system console module.
 *
 * Note:            None
 ********************************************************************/
#if defined(SYS_CONSOLE_ENABLE) 
void* _SYS_CONSOLE_OPEN(SYS_MODULE_ID console_port)
{
	return((void*)USART_OPEN(console_port));
}
#endif

/*********************************************************************
 * Function:        bool _SYS_CONSOLE_INIT(int console_port)
 *
 * PreCondition:    None
 *
 * Input:           None
 *
 * Output:          It initializes the system console module.
 *
 * Side Effects:    None
 *
 * Overview:        The function initializes the system console module.
 *
 * Note:            None
 ********************************************************************/
#if defined(SYS_CONSOLE_ENABLE) 
bool _SYS_CONSOLE_INIT(int console_port)
{
    
    bool initRes = USART_INIT(console_port, SYS_CONSOLE_BAUDRATE);
    if(initRes)
    {
        console_handle = _SYS_CONSOLE_OPEN(console_port);
        return console_handle != 0;
    }

    return false;
}
#endif

/*********************************************************************
 * Function:        void _SYS_CONSOLE_PRINT(const char *format, ...)
 *
 * PreCondition:    None
 *
 * Input:           None
 *
 * Output:          It displays a formatted message to the system console
 *
 * Side Effects:    None
 *
 * Overview:        The function will display the formatted message 
 *					to the system console.
 *
 * Note:            None
 ********************************************************************/
#if defined(SYS_CONSOLE_ENABLE) 
void _SYS_CONSOLE_PRINT(const char* format, ...)
{
	va_list arg_list;
    char buff[SYS_CONSOLE_BUFFER_LEN];
	
	USART_PRINT((USART_MODULE_CTRL*)console_handle, "\nSYS_CONSOLE_PRINT issued: ");
	va_start(arg_list, format);
	vsnprintf(buff, SYS_CONSOLE_BUFFER_LEN, format, arg_list);
	va_end(arg_list);
    USART_PRINT(console_handle, buff);	
}
#endif

/*********************************************************************
 * Function:        void _SYS_CONSOLE_MESSAGE(const char* msg)
 *
 * PreCondition:    None
 *
 * Input:           None
 *
 * Output:          It displays a message to the system console
 *
 * Side Effects:    None
 *
 * Overview:        The function will display the message to the 
 *					system console.
 *
 * Note:            None
 ********************************************************************/
#if defined(SYS_CONSOLE_ENABLE) 
void _SYS_CONSOLE_MESSAGE(const char* msg)
{
	USART_PRINT((USART_MODULE_CTRL*)console_handle, (char*)msg);
}
#endif

/*********************************************************************
 * Function:		char _SYS_CONSOLE_DATA_RDY(void)
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
#if defined(SYS_CONSOLE_ENABLE) 
bool _SYS_CONSOLE_DATA_RDY()
{
	return(USART_DATA_RDY((USART_MODULE_CTRL*)console_handle));
}
#endif


/*********************************************************************
 * Function:        char _SYS_CONSOLE_GETC(void)
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
 *
 * Note:            None
 ********************************************************************/
#if defined(SYS_CONSOLE_ENABLE) 
char _SYS_CONSOLE_GETC()
{
	return(USART_GETC((USART_MODULE_CTRL*)console_handle));
}
#endif
	
/*********************************************************************
 * Function:		char _SYS_CONSOLE_PUTC(void)
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
#if defined(SYS_CONSOLE_ENABLE) 
void _SYS_CONSOLE_PUTC(char c)
{
	USART_PUTC((USART_MODULE_CTRL*)console_handle, c);
}
#endif



/*********************************************************************
 * Function:        int _SYS_CONSOLE_GETLINE(char* line, int bufferLen)
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
#if defined(SYS_CONSOLE_ENABLE) 
int _SYS_CONSOLE_GETLINE(char* line, int bufferLen)
{
	int i=0;
	
	do
		*line=USART_GETC((USART_MODULE_CTRL*)console_handle);	
	while((*line++ != '\n') && (i++ < bufferLen));
	
	return(i);
}
#endif

/*********************************************************************
 * Function:        int _SYS_CONSOLE_SCANF(const char *format, ...)
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

/*********************************************************************
 * Function:        int _SYS_CONSOLE_KBHIT()
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
#if defined(SYS_CONSOLE_ENABLE)
int _SYS_CONSOLE_KBHIT()
{
	return(USART_GETC_PENDING((USART_MODULE_CTRL*)console_handle));
}
#endif

/*********************************************************************
 * Function:        bool _SYS_OUT_INIT(int out_port)
 *
 * PreCondition:    None
 *
 * Input:           None
 *
 * Output:          It initializes the system output module.
 *
 * Side Effects:    None
 *
 * Overview:        The function initializes the system output module.
 *
 * Note:            None
 ********************************************************************/
#if defined(SYS_OUT_ENABLE) 
bool _SYS_OUT_INIT(int out_port)
{
    LCD_INIT();
	return true;
}
#endif

/*********************************************************************
 * Function:        void _SYS_OUT_MESSAGE(const char* msg)
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
#if defined(SYS_OUT_ENABLE) 
void _SYS_OUT_MESSAGE(const char* msg)
{
	LCD_ERASE();
	_SYS_OUT_MESSAGE_OFFSET(msg, 0);
}
#endif

/*********************************************************************
 * Function:        void _SYS_OUT_MESSAGE_LINE(const char *msg, int line)
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
#if defined(SYS_OUT_ENABLE) 
void _SYS_OUT_MESSAGE_LINE(const char *msg, int line)
{
	_SYS_OUT_MESSAGE_OFFSET(msg, line*LCD_CHARS_PER_LINE);
}
#endif

/*********************************************************************
 * Function:        void _SYS_OUT_MESSAGE_OFFSET(const char *msg, int offset)
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
#if defined(SYS_OUT_ENABLE) 
void _SYS_OUT_MESSAGE_OFFSET(const char *msg, int offset)
{
	extern char LCDText[];
	strcpy((char*)LCDText, msg);
	LCD_UPDATE_OFFSET(offset);	
}
#endif


