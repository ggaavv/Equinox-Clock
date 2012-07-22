/*
 * coms.h
 *
 *  Created on: 31 Mar 2011
 *      Author: Jamie
 */

#ifndef COMS_H_
#define COMS_H_

#ifdef __cplusplus
	extern "C" {
#endif

#include "lpc17xx_uart.h"
#include "term_io.h"
#include <stdio.h>
#include <string.h>

#define OK "[OK]-"
#define ERR "[ERR]-"
#define INFO "[INFO]-"

#if 1
#ifndef WIN32
#define __SHORT_FILE__ ((strrchr(__FILE__, '/') ? : __FILE__- 1) + 1)
#else
#define __SHORT_FILE__ ((strrchr(__FILE__, '\\') ? strrchr(__FILE__, '\\') + 1 : __FILE__))
#endif
#endif

#define _L_ __LINE__
//#define __SHORT_FILE__ (strrchr(__FILE__, '/') ? strrchr(__FILE__, '/') + 1 : __FILE__)
//#define __SHORT_FILE__ (strrchr(__FILE__, '\\') ? strrchr(__FILE__, '\\') + 1 : __FILE__)
//#define __SHORT_FILE__ (strrchr(__FILE__, '\\') ? strrchr(__FILE__, '\\') + 1 : __FILE__)
#define _F_ __SHORT_FILE__
//#define _F_ __FILE__
#define _FL_ __F__,__LINE__
//#define _FUN_ __FUNCTION__
#define FL() xprintf(" (%s:%d)",_F_,_L_)
#define FL_() xprintf(" (%s:%d)\n",_F_,_L_)
#define FFL() xprintf(" (%s:%s():%d)",_F_,__func__,_L_)
#define FFL_() xprintf(" (%s:%s():%d)\n",_F_,__func__,_L_)
//#define FFL_() xprintf(" (%s:%s:%d)\n",_F_,__func__,_L_)

void exec_cmd(char *cmd);
int comm_test(void);
//int8_t __putchar(int8_t ch);
uint8_t comm_get2(int name);
uint8_t comm_get(void);
void comm_flush(void);
//uint8_t comm_gets(void);
void comm_put(uint8_t d);
void comm_init(void);

void UART0_IRQHandler(void);
void UART_IntReceive(void);
void UART_IntTransmit(void);
void UART_IntErr(uint8_t bLSErrType);
uint32_t UARTSend(LPC_UART_TypeDef *UARTPort, uint8_t *txbuf, uint8_t buflen);
uint32_t UARTReceive(LPC_UART_TypeDef *UARTPort, uint8_t *rxbuf, uint8_t buflen);

#ifdef __cplusplus
 }
#endif

#endif /* COMS_H_ */
