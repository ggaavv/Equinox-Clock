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

int8_t __putchar(int8_t ch);
uint8_t comm_get(void);
uint8_t comm_gets(void);
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
