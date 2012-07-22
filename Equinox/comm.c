/*
 * coms.cpp
 *
 *  Created on: 31 Mar 2011
 *      Author: Jamie
 */

#include <string.h>
#include "comm.h"

#include "lpc17xx_uart.h"
#include "lpc17xx_pinsel.h"
#include "lpc17xx_gpio.h"
#include "lpc17xx_wdt.h"
#include "sys_timer.h"
#include "pinout.h"
#include "ShiftPWM.h"

//Watchodog time out in 2 seconds
#define WDT_TIMEOUT 	2000000


#define USER_FLASH_START 0x3000 // For USB bootloader
//#define USER_FLASH_START 0x0 // No USB bootloader
#define BOOTLOADER_START 0x0 // To enter bootloader

volatile int LINE_READY=0;
volatile int RX_TOG=0;
volatile int TX_TOG=0;

volatile uint8_t UART_LINE[50];
volatile uint32_t UART_LINE_LEN=0;

#define USARTx LPC_UART0

/************************** PRIVATE DEFINTIONS *************************/
/* buffer size definition */
#define UART_RING_BUFSIZE 256

/* Buf mask */
#define __BUF_MASK (UART_RING_BUFSIZE-1)
/* Check buf is full or not */
#define __BUF_IS_FULL(head, tail) ((tail&__BUF_MASK)==((head+1)&__BUF_MASK))
/* Check buf will be full in next receiving or not */
#define __BUF_WILL_FULL(head, tail) ((tail&__BUF_MASK)==((head+2)&__BUF_MASK))
/* Check buf is empty */
#define __BUF_IS_EMPTY(head, tail) ((head&__BUF_MASK)==(tail&__BUF_MASK))
/* Reset buf */
#define __BUF_RESET(bufidx)	(bufidx=0)
#define __BUF_INCR(bufidx)	(bufidx=(bufidx+1)&__BUF_MASK)


/************************** PRIVATE TYPES *************************/

/** @brief UART Ring buffer structure */
typedef struct
{
    __IO uint32_t tx_head;                /*!< UART Tx ring buffer head index */
    __IO uint32_t tx_tail;                /*!< UART Tx ring buffer tail index */
    __IO uint32_t rx_head;                /*!< UART Rx ring buffer head index */
    __IO uint32_t rx_tail;                /*!< UART Rx ring buffer tail index */
    __IO uint8_t  tx[UART_RING_BUFSIZE];  /*!< UART Tx data ring buffer */
    __IO uint8_t  rx[UART_RING_BUFSIZE];  /*!< UART Rx data ring buffer */
} UART_RING_BUFFER_T;


/************************** PRIVATE VARIABLES *************************/
// UART Ring buffer
UART_RING_BUFFER_T rb;

// Current Tx Interrupt enable state
__IO FlagStatus TxIntStat;

void exec_cmd(char *cmd){
	if(stricmp(cmd,"RS")==0){
		xprintf(INFO "reseting" " (%s:%d)\n",_F_,_L_);
		WDT_Init(WDT_CLKSRC_IRC, WDT_MODE_RESET);
		WDT_Start(WDT_TIMEOUT);
		while(1);//lockup, wdt will reset board
		//WDT_ClrTimeOutFlag();
	}
	else if(stricmp(cmd,"BL")==0){
		xprintf(INFO "resetting to bootloader" " (%s:%d)\n",_F_,_L_);
		(*(void(*)())BOOTLOADER_START)();//doesn't work
	}
	else if(stricmp(cmd,"test")==0){
		xprintf(INFO "tests running" " (%s:%d)\n",_F_,_L_);
		LED_test();
	}
	else if(stricmp(cmd,"")==0){
		xprintf(INFO "\r\nRS-Resets board\r\nBL-Resets to bootloader(does not work)\r\n" " (%s:%d)\n",_F_,_L_);
	}
	else{
		xprintf(INFO "Command not found (cmd=%s)" " (%s:%d)\n",cmd,_F_,_L_);
	}
	return;
}

int comm_test(void){
	xprintf("%s{\n",__func__);
//	delay_ms(1000);
	return ( LPC_UART0->LSR & UART_LSR_RDR ) ? 1 : 0;
//	return 0;//always avaliable

    // Wait for incomming char...
//    while (USART_GetFlagStatus(USART1, USART_FLAG_RXNE) == RESET) ;
//	return ( USART_GetFlagStatus(USARTx, USART_FLAG_RXNE) == RESET ) ? 0 : 1;
}

#if 0
//for printf??
int8_t __putchar(int8_t ch){
	xprintf("%s{\n",__func__);
  if (ch == '\n')
	  comm_put('\r');
  comm_put(ch);
	xprintf("%s}\n",__func__);
}

//for scanf??
int8_t __getchar(void){
	xprintf("%s{\n",__func__);
	return comm_get();
}
//for scanf??
int8_t _getc(void){
	xprintf("%s{\n",__func__);
	return comm_get();
}
#endif


uint8_t comm_get2(int name){
	(void)name; /* avoid warning */
	uint8_t buffer[2], len;
	while (len == 0){
		len = UART_Receive(LPC_UART0, buffer, sizeof(buffer), NONE_BLOCKING);
	}
	return buffer;// buffer;
}

uint8_t comm_get(void){
#if 0
//	xprintf("%s{\n",__func__);
	uint8_t tmp = 0;
	UART_Receive(LPC_UART0, &tmp, 1, BLOCKING);
	return(tmp);
#endif
#if 1
	uint8_t buffer[1], len;
	while (len == 0){
		len = UARTReceive(LPC_UART0, buffer, 1);
	}
	UART_LINE_LEN=0;
	return buffer;
#endif
}

#if 0
uint8_t comm_gets(void){
	uint8_t buffer[100];
	uint32_t len;
	while (len == 0){
		len = UART_Receive(LPC_UART0, buffer, sizeof(buffer), NONE_BLOCKING);
	}
	return buffer;
}
#endif

void comm_put(uint8_t d){
////	UARTSend(LPC_UART_TypeDef *UARTPort, uint8_t *txbuf, uint8_t buflen)
////	UARTSend(LPC_UART0, d, 1);
////	UART_SendByte(LPC_UART0, d);
//	UART_Send(LPC_UART0, &d, 1, BLOCKING);//without interrupt
	UARTSend(LPC_UART0, &d, 1);//with interrupt
////	serial_writechar(d);
}

#if 0
void comm_puts(const void *str){
	uint8_t *s = (uint8_t *) str;

	while (*s){
		UARTPutChar(LPC_UART0, *s++);
	}
}
#endif

void comm_init(void){

	UART_LINE_LEN=0;

	// UART Configuration structure variable
	UART_CFG_Type UARTConfigStruct;
	// UART FIFO configuration Struct variable
//	UART_FIFO_CFG_Type UARTFIFOConfigStruct;
	// Pin configuration for UART0
	PINSEL_CFG_Type PinCfg;
/*
	uint32_t idx, len;
	__IO FlagStatus exitflag;
	uint8_t buffer[100];
*/
	/*
	 * Initialize UART0 pin connect
	 */
	PinCfg.Funcnum = 1;
	PinCfg.OpenDrain = 0;
	PinCfg.Pinmode = 0;
	PinCfg.Pinnum = 2;
	PinCfg.Portnum = 0;
	PINSEL_ConfigPin(&PinCfg);
	PinCfg.Pinnum = 3;
	PINSEL_ConfigPin(&PinCfg);

	UART_ConfigStructInit(&UARTConfigStruct);
	// Re-configure baudrate to 115200bps
	/* Initialize UART Configuration parameter structure to default state:
	 * Baudrate = 115200bps
	 * 8 data bit
	 * 1 Stop bit
	 * None parity
	 */
	UARTConfigStruct.Baud_rate = 115200;
	UARTConfigStruct.Parity = UART_PARITY_NONE;
	UARTConfigStruct.Stopbits = UART_STOPBIT_1;
	UARTConfigStruct.Databits = UART_DATABIT_8;

	// Initialize UART0 peripheral with given to corresponding parameter
	UART_Init(LPC_UART0, &UARTConfigStruct);


	/* Initialize FIFOConfigStruct to default state:
	 * 				- FIFO_DMAMode = DISABLE
	 * 				- FIFO_Level = UART_FIFO_TRGLEV0
	 * 				- FIFO_ResetRxBuf = ENABLE
	 * 				- FIFO_ResetTxBuf = ENABLE
	 * 				- FIFO_State = ENABLE
	 */
//	UART_FIFOConfigStructInit(&UARTFIFOConfigStruct);

	// Initialize FIFO for UART0 peripheral
//	UART_FIFOConfig((LPC_UART_TypeDef *)LPC_UART0, &UARTFIFOConfigStruct);


	// Enable UART Transmit
	UART_TxCmd(LPC_UART0, ENABLE);
#if 1
    /* Enable UART Rx interrupt */
	UART_IntConfig((LPC_UART_TypeDef *)LPC_UART0, UART_INTCFG_RBR, ENABLE);
	/* Enable UART line status interrupt */
	UART_IntConfig((LPC_UART_TypeDef *)LPC_UART0, UART_INTCFG_RLS, ENABLE);
	/*
	 * Do not enable transmit interrupt here, since it is handled by
	 * UART_Send() function, just to reset Tx Interrupt state for the
	 * first time
	 */
	TxIntStat = RESET;

	// Reset ring buf head and tail idx
	__BUF_RESET(rb.rx_head);
	__BUF_RESET(rb.rx_tail);
	__BUF_RESET(rb.tx_head);
	__BUF_RESET(rb.tx_tail);

    /* preemption = 1, sub-priority = 1 */
    NVIC_SetPriority(UART0_IRQn, ((0x01<<3)|0x01));
	/* Enable Interrupt for UART0 channel */
    NVIC_EnableIRQ(UART0_IRQn);
#endif
}


#if 1
/*----------------- INTERRUPT SERVICE ROUTINES --------------------------*/
/*********************************************************************//**
 * @brief		UART0 interrupt handler sub-routine
 * @param[in]	None
 * @return 		None
 **********************************************************************/
void UART0_IRQHandler(void)
{
	uint32_t intsrc, tmp, tmp1;

	/* Determine the interrupt source */
	intsrc = UART_GetIntId(LPC_UART0);
	tmp = intsrc & UART_IIR_INTID_MASK;


	// Receive Line Status
	if (tmp == UART_IIR_INTID_RLS){
		// Check line status
		tmp1 = UART_GetLineStatus(LPC_UART0);
		// Mask out the Receive Ready and Transmit Holding empty status
		tmp1 &= (UART_LSR_OE | UART_LSR_PE | UART_LSR_FE \
				| UART_LSR_BI | UART_LSR_RXFE);
		// If any error exist
		if (tmp1) {
			UART_IntErr(tmp1);
		}
	}

	// Receive Data Available or Character time-out
	if ((tmp == UART_IIR_INTID_RDA) || (tmp == UART_IIR_INTID_CTI)){
		UART_IntReceive();
	}

	// Transmit Holding Empty
	if (tmp == UART_IIR_INTID_THRE){
		UART_IntTransmit();
	}

}

/********************************************************************//**
 * @brief 		UART receive function (ring buffer used)
 * @param[in]	None
 * @return 		None
 *********************************************************************/
void UART_IntReceive(void)
{
	uint8_t tmpc;
	uint32_t rLen;

	if(RX_TOG)
		GPIO_SetValue(LED_3_PORT, LED_3_BIT);
	else
		GPIO_ClearValue(LED_3_PORT, LED_3_BIT);
	RX_TOG=!RX_TOG;

	while(1){
		// Call UART read function in UART driver
		rLen = UART_Receive((LPC_UART_TypeDef *)LPC_UART0, &tmpc, 1, NONE_BLOCKING);
		// If data received
		if (rLen){
			UART_LINE[UART_LINE_LEN++]=tmpc;
//			xprintf("%s|%s;---%s{=%s\n",tmpc,UART_LINE[UART_LINE_LEN],__func__,tmpc);

//			xprintf("r---%s{=%s\n",__func__,tmpc);
			if((tmpc=='\r')||(tmpc=='\n')){
				LINE_READY = 1;
				UART_LINE[UART_LINE_LEN-1]='\0';
				UART_LINE_LEN=0;
//				UART_LINE[0]='\0';
//				xprintf("LINE_READY = 1;---%s{=%s\n",__func__,tmpc);
			}
			/* Check if buffer is more space
			 * If no more space, remaining character will be trimmed out
			 */
			if (!__BUF_IS_FULL(rb.rx_head,rb.rx_tail)){
				rb.rx[rb.rx_head] = tmpc;
				__BUF_INCR(rb.rx_head);
			}
		}
		// no more data
		else {
			break;
		}
	}
}

/********************************************************************//**
 * @brief 		UART transmit function (ring buffer used)
 * @param[in]	None
 * @return 		None
 *********************************************************************/
void UART_IntTransmit(void)
{
	if(TX_TOG)
		GPIO_SetValue(LED_2_PORT, LED_2_BIT);
	else
		GPIO_ClearValue(LED_2_PORT, LED_2_BIT);
	TX_TOG=!TX_TOG;

    // Disable THRE interrupt
    UART_IntConfig((LPC_UART_TypeDef *)LPC_UART0, UART_INTCFG_THRE, DISABLE);

	/* Wait for FIFO buffer empty, transfer UART_TX_FIFO_SIZE bytes
	 * of data or break whenever ring buffers are empty */
	/* Wait until THR empty */
    while (UART_CheckBusy((LPC_UART_TypeDef *)LPC_UART0) == SET);

	while (!__BUF_IS_EMPTY(rb.tx_head,rb.tx_tail))
    {
        /* Move a piece of data into the transmit FIFO */
    	if (UART_Send((LPC_UART_TypeDef *)LPC_UART0, (uint8_t *)&rb.tx[rb.tx_tail], 1, NONE_BLOCKING)){
        /* Update transmit ring FIFO tail pointer */
        __BUF_INCR(rb.tx_tail);
    	} else {
    		break;
    	}
    }

    /* If there is no more data to send, disable the transmit
       interrupt - else enable it or keep it enabled */
	if (__BUF_IS_EMPTY(rb.tx_head, rb.tx_tail)) {
    	UART_IntConfig((LPC_UART_TypeDef *)LPC_UART0, UART_INTCFG_THRE, DISABLE);
    	// Reset Tx Interrupt state
    	TxIntStat = RESET;
    }
    else{
      	// Set Tx Interrupt state
		TxIntStat = SET;
    	UART_IntConfig((LPC_UART_TypeDef *)LPC_UART0, UART_INTCFG_THRE, ENABLE);
    }
}


/*********************************************************************//**
 * @brief		UART Line Status Error
 * @param[in]	bLSErrType	UART Line Status Error Type
 * @return		None
 **********************************************************************/
void UART_IntErr(uint8_t bLSErrType)
{
	uint8_t test;
	// Loop forever
	while (1){
		// For testing purpose
		test = bLSErrType;
	}
}

/*-------------------------PRIVATE FUNCTIONS------------------------------*/
/*********************************************************************//**
 * @brief		UART transmit function for interrupt mode (using ring buffers)
 * @param[in]	UARTPort	Selected UART peripheral used to send data,
 * 				should be UART0
 * @param[out]	txbuf Pointer to Transmit buffer
 * @param[in]	buflen Length of Transmit buffer
 * @return 		Number of bytes actually sent to the ring buffer
 **********************************************************************/
uint32_t UARTSend(LPC_UART_TypeDef *UARTPort, uint8_t *txbuf, uint8_t buflen)
{
    uint8_t *data = (uint8_t *) txbuf;
    uint32_t bytes = 0;

	/* Temporarily lock out UART transmit interrupts during this
	   read so the UART transmit interrupt won't cause problems
	   with the index values */
    UART_IntConfig(UARTPort, UART_INTCFG_THRE, DISABLE);

	/* Loop until transmit run buffer is full or until n_bytes
	   expires */
	while ((buflen > 0) && (!__BUF_IS_FULL(rb.tx_head, rb.tx_tail)))
	{
		/* Write data from buffer into ring buffer */
		rb.tx[rb.tx_head] = *data;
		data++;

		/* Increment head pointer */
		__BUF_INCR(rb.tx_head);

		/* Increment data count and decrement buffer size count */
		bytes++;
		buflen--;
	}

	/*
	 * Check if current Tx interrupt enable is reset,
	 * that means the Tx interrupt must be re-enabled
	 * due to call UART_IntTransmit() function to trigger
	 * this interrupt type
	 */
	if (TxIntStat == RESET) {
		UART_IntTransmit();
	}
	/*
	 * Otherwise, re-enables Tx Interrupt
	 */
	else {
		UART_IntConfig(UARTPort, UART_INTCFG_THRE, ENABLE);
	}

    return bytes;
}


/*********************************************************************//**
 * @brief		UART read function for interrupt mode (using ring buffers)
 * @param[in]	UARTPort	Selected UART peripheral used to send data,
 * 				should be UART0
 * @param[out]	rxbuf Pointer to Received buffer
 * @param[in]	buflen Length of Received buffer
 * @return 		Number of bytes actually read from the ring buffer
 **********************************************************************/
uint32_t UARTReceive(LPC_UART_TypeDef *UARTPort, uint8_t *rxbuf, uint8_t buflen)
{
    uint8_t *data = (uint8_t *) rxbuf;
    uint32_t bytes = 0;

	/* Temporarily lock out UART receive interrupts during this
	   read so the UART receive interrupt won't cause problems
	   with the index values */
	UART_IntConfig(UARTPort, UART_INTCFG_RBR, DISABLE);

	/* Loop until receive buffer ring is empty or
		until max_bytes expires */
	while ((buflen > 0) && (!(__BUF_IS_EMPTY(rb.rx_head, rb.rx_tail))))
	{
		/* Read data from ring buffer into user buffer */
		*data = rb.rx[rb.rx_tail];
		data++;

		/* Update tail pointer */
		__BUF_INCR(rb.rx_tail);

		/* Increment data count and decrement buffer size count */
		bytes++;
		buflen--;
	}

	/* Re-enable UART interrupts */
	UART_IntConfig(UARTPort, UART_INTCFG_RBR, ENABLE);

    return bytes;
}
#endif
#if 0
/*********************************************************************//**
 * @brief	Print Welcome Screen Menu subroutine
 * @param	None
 * @return	None
 **********************************************************************/
void print_menu(void)
{
	uint32_t tmp, tmp2;
	uint8_t *pDat;

	tmp = sizeof(menu1);
	tmp2 = 0;
	pDat = (uint8_t *)&menu1[0];
	while(tmp) {
		tmp2 = UARTSend((LPC_UART_TypeDef *)LPC_UART0, pDat, tmp);
		pDat += tmp2;
		tmp -= tmp2;
	}

	tmp = sizeof(menu2);
	tmp2 = 0;
	pDat = (uint8_t *)&menu2[0];
	while(tmp) {
		tmp2 = UARTSend((LPC_UART_TypeDef *)LPC_UART0, pDat, tmp);
		pDat += tmp2;
		tmp -= tmp2;
	}
}
/*-------------------------MAIN FUNCTION------------------------------*/
/*********************************************************************//**
 * @brief		c_entry: Main UART program body
 * @param[in]	None
 * @return 		int
 **********************************************************************/
int entry(void)
{
	// UART Configuration structure variable
	UART_CFG_Type UARTConfigStruct;
	// UART FIFO configuration Struct variable
	UART_FIFO_CFG_Type UARTFIFOConfigStruct;
	// Pin configuration for UART0
	PINSEL_CFG_Type PinCfg;

	uint32_t idx, len;
	__IO FlagStatus exitflag;
	uint8_t buffer[100];

	/*
	 * Initialize UART0 pin connect
	 */
	PinCfg.Funcnum = 1;
	PinCfg.OpenDrain = 0;
	PinCfg.Pinmode = 0;
	PinCfg.Pinnum = 2;
	PinCfg.Portnum = 0;
	PINSEL_ConfigPin(&PinCfg);
	PinCfg.Pinnum = 3;
	PINSEL_ConfigPin(&PinCfg);


	/* Initialize UART Configuration parameter structure to default state:
	 * Baudrate = 9600bps
	 * 8 data bit
	 * 1 Stop bit
	 * None parity
	 */
	UART_ConfigStructInit(&UARTConfigStruct);

	// Initialize UART0 peripheral with given to corresponding parameter
	UART_Init((LPC_UART_TypeDef *)LPC_UART0, &UARTConfigStruct);


	/* Initialize FIFOConfigStruct to default state:
	 * 				- FIFO_DMAMode = DISABLE
	 * 				- FIFO_Level = UART_FIFO_TRGLEV0
	 * 				- FIFO_ResetRxBuf = ENABLE
	 * 				- FIFO_ResetTxBuf = ENABLE
	 * 				- FIFO_State = ENABLE
	 */
	UART_FIFOConfigStructInit(&UARTFIFOConfigStruct);

	// Initialize FIFO for UART0 peripheral
	UART_FIFOConfig((LPC_UART_TypeDef *)LPC_UART0, &UARTFIFOConfigStruct);


	// Enable UART Transmit
	UART_TxCmd((LPC_UART_TypeDef *)LPC_UART0, ENABLE);

    /* Enable UART Rx interrupt */
	UART_IntConfig((LPC_UART_TypeDef *)LPC_UART0, UART_INTCFG_RBR, ENABLE);
	/* Enable UART line status interrupt */
	UART_IntConfig((LPC_UART_TypeDef *)LPC_UART0, UART_INTCFG_RLS, ENABLE);
	/*
	 * Do not enable transmit interrupt here, since it is handled by
	 * UART_Send() function, just to reset Tx Interrupt state for the
	 * first time
	 */
	TxIntStat = RESET;

	// Reset ring buf head and tail idx
	__BUF_RESET(rb.rx_head);
	__BUF_RESET(rb.rx_tail);
	__BUF_RESET(rb.tx_head);
	__BUF_RESET(rb.tx_tail);

    /* preemption = 1, sub-priority = 1 */
    NVIC_SetPriority(UART0_IRQn, ((0x01<<3)|0x01));
	/* Enable Interrupt for UART0 channel */
    NVIC_EnableIRQ(UART0_IRQn);


	// print welcome screen
//	print_menu();

	// reset exit flag
	exitflag = RESET;

    /* Read some data from the buffer */
    while (exitflag == RESET)
    {
       len = 0;
        while (len == 0)
        {
            len = UARTReceive((LPC_UART_TypeDef *)LPC_UART0, buffer, sizeof(buffer));
        }

        /* Got some data */
        idx = 0;
        while (idx < len)
        {
            if (buffer[idx] == 27)
            {
                /* ESC key, set exit flag */
            	UARTSend((LPC_UART_TypeDef *)LPC_UART0, menu3, sizeof(menu3));
                exitflag = SET;
            }
            else if (buffer[idx] == 'r')
            {
                print_menu();
            }
            else
            {
                /* Echo it back */
            	UARTSend((LPC_UART_TypeDef *)LPC_UART0, &buffer[idx], 1);
            }
            idx++;
        }
    }

    // wait for current transmission complete - THR must be empty
    while (UART_CheckBusy((LPC_UART_TypeDef *)LPC_UART0));

    // DeInitialize UART0 peripheral
    UART_DeInit((LPC_UART_TypeDef *)LPC_UART0);

    /* Loop forever */
    while(1);
    return 1;
}
#endif

