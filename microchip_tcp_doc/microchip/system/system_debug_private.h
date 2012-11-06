/*******************************************************************************

  Summary: System Debugging private messaging API
    
  Description:

*******************************************************************************/

/*******************************************************************************
FileName: 	system_debug_private.h
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


#ifndef _SYSTEM_DEBUG_PRIVATE_H_
#define _SYSTEM_DEBUG_PRIVATE_H_


void        _SYS_ASSERT(int linenumber, const char *filename, const char *message);

void        _SYS_ERROR(int linenumber, const char *filename, const char *message);

void 		_SYS_ERROR_PRINT(SYS_ERROR_LEVEL level, const char *format, ...);


void 		_SYS_CONSOLE_MESSAGE(const char *message);


void        _SYS_CONSOLE_PRINT(const char *format, ...);

char 		_SYS_CONSOLE_GETC(void);

void        _SYS_CONSOLE_PUTC(char c);

int 		_SYS_CONSOLE_KBHIT(void);

bool        _SYS_CONSOLE_DATA_RDY();


int 		_SYS_CONSOLE_GETLINE(char* buffer, int bufferLen);

int 		_SYS_CONSOLE_SCANF(const char *format, ...);

void 		_SYS_OUT_MESSAGE(const char *msg);

void 		_SYS_OUT_MESSAGE_LINE(const char *msg, int line);

void 		_SYS_OUT_MESSAGE_OFFSET(const char *msg, int offset);

int 		_SYS_OUT_MESSAGE_LINE_COUNT(void);

int 		_SYS_OUT_MESSAGE_LINE_LENGTH(void);


bool        _SYS_DEBUG_INIT(int debug_port);

bool        _SYS_CONSOLE_INIT(int console_port);

bool        _SYS_OUT_INIT(int out_port);


#endif  // _SYSTEM_DEBUG_PRIVATE_H_

