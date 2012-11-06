/*******************************************************************************

  Summary:
    
  Description:

*******************************************************************************/

/*******************************************************************************
FileName: 	usart.h
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

#ifndef _USART_H_
#define _USART_H_

#include <stdint.h>
#include <stdbool.h>
#include <stdarg.h>


#if defined(__PIC32MX__)
#include <p32xxxx.h>
#elif defined(__PIC24F__)
#include <p24fxxxx.h>
#elif defined(__PIC24H__)
#include <p24hxxxx.h>
#elif defined(__dsPIC33F__) 
#include <p33fxxxx.h>
#elif defined(__dsPIC33E__) 
#include <p33exxxx.h>
#endif

#include "system/system_services.h"

#if defined(__PIC32MX__)
#define  USART_REG volatile uint32_t 
#define  USART_MODE_CTRL volatile __U1MODEbits_t 
#define  USART_STAT_CTRL volatile __U1STAbits_t 
#else
#define  USART_REG volatile uint16_t 
#define  USART_MODE_CTRL volatile UxMODEBITS
#define  USART_STAT_CTRL volatile UxSTABITS
#endif

typedef enum
{
	USART1_ID=0,
	USART2_ID
} USART_MODULE_ID;


typedef struct __usart_module_ctrl
{
	USART_REG *UxMODE;
	USART_REG *UxSTA;
	USART_REG *UxTXREG;
	USART_REG *UxRXREG;
	USART_REG *UxBRG;
} USART_MODULE_CTRL;


bool USART_INIT(SYS_MODULE_ID port_id, uint32_t baud_rate);

USART_MODULE_CTRL* USART_OPEN(SYS_MODULE_ID port_id);
	
void USART_PRINT(USART_MODULE_CTRL* pusart_module_ctrl, char* str);

bool USART_DATA_RDY(USART_MODULE_CTRL* pusart_module_ctrl);

char USART_GETC(USART_MODULE_CTRL* pusart_module_ctrl);

void USART_PUTC(USART_MODULE_CTRL* pusart_module_ctrl,char c);

int USART_GETC_PENDING(USART_MODULE_CTRL* pusart_module_ctrl);

#endif
