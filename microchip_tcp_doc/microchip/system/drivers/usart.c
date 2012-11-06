/*******************************************************************************

  Summary:
    
  Description:

*******************************************************************************/

/*******************************************************************************
FileName: 	usart.c
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
INCLUDING BUT NOT LIMITED TO ANY INCIDENTAL, SPECIAL, INDIRECT, PzUNITIVE OR
CONSEQUENTIAL DAMAGES, LOST PROFITS OR LOST DATA, COST OF PROCUREMENT OF
SUBSTITUTE GOODS, TECHNOLOGY, SERVICES, OR ANY CLAIMS BY THIRD PARTIES
(INCLUDING BUT NOT LIMITED TO ANY DEFENSE THEREOF), OR OTHER SIMILAR COSTS.
*******************************************************************************/

#include "system_profile.h"

#if defined (SYS_CONSOLE_ENABLE) || defined (SYS_DEBUG_ENABLE)


#include "usart.h"


USART_MODULE_CTRL usart_module_ctrl[]={
	{(USART_REG*)&U1MODE, (USART_REG*)&U1STA, (USART_REG*)&U1TXREG, (USART_REG*)&U1RXREG, (USART_REG*)&U1BRG},
	{(USART_REG*)&U2MODE, (USART_REG*)&U2STA, (USART_REG*)&U2TXREG, (USART_REG*)&U2RXREG, (USART_REG*)&U2BRG},
//	{&U3MODE, &U3STA, &U3TXREG, &U3RXREG, &U3BRG},
//	{&U4MODE, &U4STA, &U4TXREG, &U4RXREG, &U4BRG},
};



// *****************************************************************************
// *****************************************************************************
// Section: USART Functions
// *****************************************************************************
// *****************************************************************************


#define _USART_BAUD_RATE_FACTOR 4


bool USART_INIT(SYS_MODULE_ID port_id, uint32_t baud_rate){
	USART_MODULE_CTRL *pusart_module_ctrl;
	USART_MODULE_ID module_id; 
	
	if(port_id == SYS_MODULE_UART_1)
		module_id = USART1_ID;
	else if(port_id == SYS_MODULE_UART_2)
		module_id = USART2_ID;
//	else if(port_id == SYS_MODULE_UART_3)
//		module_id = USART_ID_3;
//	else if(port_id == SYS_MODULE_UART_4)
//		module_id = USART_ID_4;
//	else if(port_id == SYS_MODULE_UART_5)
//		module_id = USART_ID_5;
//	else if(port_id == SYS_MODULE_UART_6)
//		module_id = USART_ID_6;
	else
		return false;

	pusart_module_ctrl = &usart_module_ctrl[module_id];
	*(pusart_module_ctrl->UxBRG) = (((SYS_CLK_PeripheralClockGet())/(baud_rate)/_USART_BAUD_RATE_FACTOR) - 1);
	*(pusart_module_ctrl->UxMODE) = 0;
	((USART_MODE_CTRL*)(pusart_module_ctrl->UxMODE))->BRGH = 1;
	*(pusart_module_ctrl->UxSTA) = 0;
	((USART_MODE_CTRL*)(pusart_module_ctrl->UxMODE))->UARTEN = 1;
	((USART_MODE_CTRL*)(pusart_module_ctrl->UxMODE))->STSEL = 0;
	((USART_STAT_CTRL*)(pusart_module_ctrl->UxSTA))->UTXEN = 1;
	#if defined (__PIC32MX__)
		((USART_STAT_CTRL*)(pusart_module_ctrl->UxSTA))->URXEN = 1;
    #endif	
	((USART_STAT_CTRL*)(pusart_module_ctrl->UxSTA))->OERR = 0;
	
	return true;
}

USART_MODULE_CTRL* USART_OPEN(SYS_MODULE_ID port_id){
	
	USART_MODULE_ID module_id = -1;
	if(port_id == SYS_MODULE_UART_1)
		module_id = USART1_ID;
	else if(port_id == SYS_MODULE_UART_2)
		module_id = USART2_ID;
//	else if(port_id == SYS_MODULE_UART_3)
//		module_id = USART_ID_3;
//	else if(port_id == SYS_MODULE_UART_4)
//		module_id = USART_ID_4;
//	else if(port_id == SYS_MODULE_UART_5)
//		module_id = USART_ID_5;
//	else if(port_id == SYS_MODULE_UART_6)
//		module_id = USART_ID_6;
	else
		return NULL;
	
	return(&usart_module_ctrl[module_id]);
}
	
void USART_PRINT(USART_MODULE_CTRL* pusart_module_ctrl, char* str){
	unsigned char c;
    while((c = *str++)){
	    while(((USART_STAT_CTRL*)(pusart_module_ctrl->UxSTA))->UTXBF == 1);
	    *(pusart_module_ctrl->UxTXREG) = c;	

	}
}

bool USART_DATA_RDY(USART_MODULE_CTRL* pusart_module_ctrl)
{
	return (((USART_STAT_CTRL*)(pusart_module_ctrl->UxSTA))->URXDA == 1);
}

char USART_GETC(USART_MODULE_CTRL* pusart_module_ctrl){
	while(((USART_STAT_CTRL*)(pusart_module_ctrl->UxSTA))->URXDA != 1);
	return(*(pusart_module_ctrl->UxRXREG));
}

void USART_PUTC(USART_MODULE_CTRL* pusart_module_ctrl,char c)
{
	while(((USART_STAT_CTRL*)(pusart_module_ctrl->UxSTA))->UTXBF == 1);
	*(pusart_module_ctrl->UxTXREG) = c; 
}

int USART_GETC_PENDING(USART_MODULE_CTRL* pusart_module_ctrl){
	return((((USART_STAT_CTRL*)(pusart_module_ctrl->UxSTA))->URXDA == 1)?1:0);
}



#endif  // defined (SYS_CONSOLE_ENABLE) || defined (SYS_DEBUG_ENABLE)

