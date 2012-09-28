/* Copyright (c) 2011 Jamie Clarke - jamie.clarke.jc@gmail.com       */
/* All rights reserved.

   Redistribution and use in source and binary forms, with or without
   modification, are permitted provided that the following conditions are met:

   * Redistributions of source code must retain the above copyright
     notice, this list of conditions and the following disclaimer.
   * Redistributions in binary form must reproduce the above copyright
     notice, this list of conditions and the following disclaimer in
     the documentation and/or other materials provided with the
     distribution.
   * Neither the name of the copyright holders nor the names of
     contributors may be used to endorse or promote products derived
     from this software without specific prior written permission.

  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
  ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
  LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
  CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
  SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
  INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
  CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
  ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
  POSSIBILITY OF SUCH DAMAGE.
*/

#include "ShiftPWM.h"
#include "pinout.h"
#include "lpc17xx_pinsel.h"
#include "lpc17xx_gpio.h"
#include "lpc17xx_ssp.h"
#include "lpc17xx_nvic.h"
#include "lpc17xx_gpdma.h"
#include "lpc17xx_timer.h"
#include "hsv2rgb.h"
#include "comm.h"
#include "sys_timer.h"

#define DMA

//#define MAX_BAM_BITS 16
#define MAX_BAM_BITS 8		// number of BITORDER bits to cycle through
#define SSP_SPEED 30000000
#define DelayLatchIn 300*(30000000/SSP_SPEED)	// delay before chip select is toggled

const uint8_t BITORDER[] = { 0,1,2,3,4,5,6,7,7,6,5,4,3,2,1,0 };		// which of the 8 bits to send
//const uint8_t BITORDER[] = { 0,7,2,5,4,3,6,1,1,6,3,4,5,2,7,0 };
//const uint8_t BITORDER[] = { 5,3,1,7,2,4,6,0,5,3,1,7,2,4,6 };

#define START_TIME 4096 //largest time inteval??
//#define START_TIME 2000 //largest time inteval??

//#define START_TIME 32 //smallest time inteval 33us
//#define START_TIME 32/2 //fastest possible
//#define START_TIME 32*1000 //for uart

volatile uint8_t SENDSEQ;
volatile uint32_t DELAY_TIME; 		//so RIT ms is not set to 0
volatile uint32_t NEXT_DELAY_TIME; 	//so RIT ms is not set to 0
volatile uint8_t SEND_BIT;			// which of the 8 bits to send
volatile uint8_t NEXT_SEND_BIT;
//volatile uint32_t LED_SEND;

/*
uint32_t SEQ_BIT[] = {
		BITORDER[0],
		BITORDER[1],
		BITORDER[2],
		BITORDER[3],
		BITORDER[4],
		BITORDER[5],
		BITORDER[6],
		BITORDER[7],
		BITORDER[8],
		BITORDER[9],
		BITORDER[10],
		BITORDER[11],
		BITORDER[12],
		BITORDER[13],
		BITORDER[14],
		BITORDER[15]
};
uint32_t SEQ_TIME[] = {
		BITTIME[BITORDER[0]],
		BITTIME[BITORDER[1]],
		BITTIME[BITORDER[2]],
		BITTIME[BITORDER[3]],
		BITTIME[BITORDER[4]],
		BITTIME[BITORDER[5]],
		BITTIME[BITORDER[6]],
		BITTIME[BITORDER[7]],
		BITTIME[BITORDER[8]],
		BITTIME[BITORDER[9]],
		BITTIME[BITORDER[10]],
		BITTIME[BITORDER[11]],
		BITTIME[BITORDER[12]],
		BITTIME[BITORDER[13]],
		BITTIME[BITORDER[14]],
		BITTIME[BITORDER[15]]
};
*/
#if 1
const uint32_t BITTIME[] = {
		START_TIME/128, //Bit 0 time (LSB)
		START_TIME/64, //Bit 1 time
		START_TIME/32, //Bit 2 time
		START_TIME/16, //Bit 3 time
		START_TIME/8, //Bit 4 time
		START_TIME/4, //Bit 5 time
		START_TIME/2, //Bit 6 time
		START_TIME //Bit 7 time (MSB)
};
/*
const uint32_t BITTIME[] = {
		START_TIME*1, //Bit 0 time (LSB)
		START_TIME*2, //Bit 1 time
		START_TIME*3, //Bit 2 time
		START_TIME*4, //Bit 3 time
		START_TIME*6, //Bit 4 time
		START_TIME*12, //Bit 5 time
		START_TIME*20, //Bit 6 time
		START_TIME*25 //Bit 7 time (MSB)
};
*/
#else
const uint32_t BITTIME[] = {
		START_TIME*1, //Bit 0 time (LSB)
		START_TIME*2, //Bit 1 time
		START_TIME*4, //Bit 2 time
		START_TIME*8, //Bit 3 time
		START_TIME*16, //Bit 4 time
		START_TIME*32, //Bit 5 time
		START_TIME*64, //Bit 6 time
		START_TIME*128 //Bit 7 time (MSB)
};
#endif
#if 0
#define REGS 1 //12
#define RGBS 5 //60
#define LEDS RGBS*3
#define BITS MAX_BAM_BITS
#else
#define REGS 12
#define RGBS 60
#define LEDS RGBS*3
#define LEDS16 REGS*16
#define BITS 8
#endif
uint16_t SEQ_BIT[16];
uint32_t SEQ_TIME[16];

volatile uint8_t LED_RAW[LEDS];				// 8bit brightness
volatile uint16_t LED_PRECALC[REGS][BITS];	// 16bit value sent to LED drivers with high bit staying on for longest
volatile uint16_t LED_PRECALC1[REGS][BITS];	// buffered as above
volatile uint16_t LED_PRECALC2[REGS][BITS];	// buffered as above
volatile uint8_t BUFFER=1;					// 1 if above precalcs are buffered
GPDMA_LLI_Type LinkerList[REGS][BITS];
GPDMA_Channel_CFG_Type GPDMACfg;
uint32_t Channel0_TC;	// Terminal Counter flag for Channel 0
uint32_t Channel0_Err;	// Error Counter flag for Channel 0
uint32_t Channel1_TC;	// Terminal Counter flag for Channel 1
uint32_t Channel1_Err;	// Error Counter flag for Channel 1

//uint16_t BITINREG[LEDS];
//uint16_t WHICHREG[LEDS];
//uint32_t LAST_MS;
//uint32_t TOTAL_MS;
extern volatile uint32_t TOG[4];
//volatile uint32_t UPDATE_COUNT;
//volatile uint32_t LED_UPDATE_REQUIRED;

void TIMER0_IRQHandler(void){
//	xprintf("TIMER0_IRQ");
	//if (TIM_GetIntStatus(LPC_TIM0,TIM_MR0_INT)==SET){
	if ((LPC_TIM0->IR)& TIM_IR_CLR(TIM_MR0_INT)){
//		FIO_SetValue(LED_OE_PORT, LED_OE_BIT);//LED's off. active low
//		FIO_ClearValue(LED_LE_PORT, LED_LE_BIT);
#if 0
//		char tempstr[50];

		if(TOG[0])
//			FIO_SetValue(LED_LE_PORT, LED_LE_BIT);
			GPIO_SetValue(LED_4_PORT, LED_4_BIT);
		else
//			FIO_ClearValue(LED_LE_PORT, LED_LE_BIT);
			GPIO_ClearValue(LED_4_PORT, LED_4_BIT);
		TOG[0]=!TOG[0];

//		TIM_ClearIntPending(LPC_TIM0, TIM_MR0_INT);
//		return;
#endif
//		xprintf(INFO "RIT N=%d B=%x NXT_T=%6d TX=%03b\n",SENDSEQ,SEND_BIT,DELAY_TIME,LED_PRECALC[0][SEND_BIT]);

		//Set bit/time
		DELAY_TIME=NEXT_DELAY_TIME;
		SEND_BIT=NEXT_SEND_BIT;
/*
		//longest
		if(SEND_BIT==)
			LED_SEND=1;
		else
			LED_SEND=0;
*/
		//Retart sequence if required
		SENDSEQ+=1;
		if (SENDSEQ>=MAX_BAM_BITS)
			SENDSEQ=0;
//		SENDSEQ=0;

		//Setup new timing for next Timer
		NEXT_DELAY_TIME=SEQ_TIME[SENDSEQ];
		NEXT_SEND_BIT=SEQ_BIT[SENDSEQ];

		//TIM_UpdateMatchValue(LPC_TIM0,0,NEXT_DELAY_TIME);
		LPC_TIM0->MR0 = NEXT_DELAY_TIME;
//		FIO_SetValue(LED_LE_PORT, LED_LE_BIT);

#ifdef DMA
		GPDMACfg.DMALLI = (uint32_t) &LinkerList[0][SEND_BIT];
		GPDMA_Setup(&GPDMACfg);
		GPDMA_ChannelCmd(0, ENABLE);
//		GPDMA_ChannelCmd(1, ENABLE);
#else
		uint8_t reg;
		for(reg=6; 0<reg;reg--){
			xprintf("%d ",reg-1);
#if 0
			if(BUFFER==1)
				SSP_SendData(LED_SPI_CHN, LED_PRECALC1[reg][SEND_BIT]);
			else
				SSP_SendData(LED_SPI_CHN, LED_PRECALC2[reg][SEND_BIT]);
			//SSP_SendData(LED_SPI_CHN, LED_PRECALC[reg][SEND_BIT]<<6);
#endif
			//WaitForSend();//Wait if TX buffer full
			//while(LED_SPI_CHN->SR & SSP_STAT_BUSY);
			while(SSP_GetStatus(LED_SPI_CHN, SSP_STAT_BUSY)){
				uint32_t w1;
				w1+=1;
//				xprintf("w1");
			};
			SSP_SendData(LED_SPI_CHN, LED_PRECALC[reg-1][SEND_BIT]);
			xprintf("%4x ",(LED_PRECALC[reg-1][SEND_BIT]));
//			LED_SPI_CHN->DR = LED_PRECALC[reg-1][SEND_BIT];
		}
//		xprintf("Regs2\n");
		for(reg=12; reg>6;reg--){
			xprintf("%d ",reg-1);
#if 0
			if(BUFFER==1)
				SSP_SendData(LED_SPI_CHN, LED_PRECALC1[reg][SEND_BIT]);
			else
				SSP_SendData(LED_SPI_CHN, LED_PRECALC2[reg][SEND_BIT]);
			//SSP_SendData(LED_SPI_CHN, LED_PRECALC[reg][SEND_BIT]<<6);
#endif
			//WaitForSend();//Wait if TX buffer full
//			while(LED_SPI_CHN->SR & SSP_STAT_BUSY);
			while(SSP_GetStatus(LED_SPI_CHN, SSP_STAT_BUSY)){
				uint32_t w2;
				w2+=1;
//				xprintf("w2");
			}
			SSP_SendData(LED_SPI_CHN, LED_PRECALC[reg-1][SEND_BIT]);
//			if (reg==7){
				xprintf("%4x ",(LED_PRECALC[reg-1][SEND_BIT]));
//			}
//			LED_SPI_CHN->DR = LED_PRECALC[reg-1][SEND_BIT];

		}
		xprintf("\n");
		LatchIn();
//		LPC_GPIO0->FIOSET = LED_LE_BIT;
//		LPC_GPIO0->FIOCLR = LED_LE_BIT;
#endif

//		FIO_ClearValue(LED_LE_PORT, LED_LE_BIT);
/*
		UPDATE_COUNT+=1;
		if(UPDATE_COUNT>=(1600/MAX_BAM_BITS)){
			UPDATE_COUNT=0;
			LED_UPDATE_REQUIRED=1;
		}
*/
//		FIO_ClearValue(LED_OE_PORT, LED_OE_BIT);//LED's on. active low

//		FIO_SetValue(LED_LE_PORT, LED_LE_BIT);
	}
	//TIM_ClearIntPending(LPC_TIM0, TIM_MR0_INT);
	LPC_TIM0->IR |= TIM_IR_CLR(TIM_MR0_INT);
}

/*
===========================================
buffer size = 8
proc clk	= 100,000,000 Mhz
SSP speed	= 25,000,000 MB/s
SSP speed /	16	= 1,562,500 16bit cyles per sec
above / buffer size = 195,312.5

transfer time for 16 bits * buffer size of 8
============================================
1sec / 25,000,000 = 0.000,000,04 = 40ns to transfer 1 bit
40ns * 16bits * buffer 8 = 5.12us + 50% = 7.86
*/

inline void LatchIn(void){
#if 1
	for(uint32_t i=0;i<500;i++){
		asm("nop");
	}
#endif
	FIO_SetValue(LED_LE_PORT, LED_LE_BIT);
#if 0
	for(uint32_t i=0;i<10;i++){
		asm("nop");
	}
#endif
	FIO_ClearValue(LED_LE_PORT, LED_LE_BIT);
}

inline void WaitForSend(void){
	while(SSP_GetStatus(LED_SPI_CHN,SSP_STAT_BUSY)){
#if 0
		for(uint32_t i=0;i<20;i++){
			asm("mov r0,r0");
		}
#endif
	}
//	while(!SSP_GetStatus(LED_SPI_CHN,SSP_STAT_TXFIFO_EMPTY));
}


void DMA_IRQHandler (void)
{
//	xprintf("DMA_IRQ");
	// check GPDMA interrupt on channel 0
	if (GPDMA_IntGetStatus(GPDMA_STAT_INT, 0)){
		// Check counter terminal status
		if(GPDMA_IntGetStatus(GPDMA_STAT_INTTC, 0)){
			// Clear terminate counter Interrupt pending
			GPDMA_ClearIntPending (GPDMA_STATCLR_INTTC, 0);
//			Channel0_TC++;
//			xprintf("ch0_TC:%d\n",Channel0_TC);
			LatchIn();
		}
		// Check error terminal status
		if (GPDMA_IntGetStatus(GPDMA_STAT_INTERR, 0)){
			// Clear error counter Interrupt pending
			GPDMA_ClearIntPending (GPDMA_STATCLR_INTERR, 0);
			Channel0_Err++;
//			xprintf("ch0_Err:%d\n",Channel0_Err);
		}
	}
#if 0
	// check GPDMA interrupt on channel 1
	if (GPDMA_IntGetStatus(GPDMA_STAT_INT, 1)){
		// Check counter terminal status
		if(GPDMA_IntGetStatus(GPDMA_STAT_INTTC, 1)){
			// Clear terminate counter Interrupt pending
			GPDMA_ClearIntPending (GPDMA_STATCLR_INTTC, 1);
				Channel1_TC++;
				xprintf("ch1_TC:%d\n",Channel0_TC);
		}
		// Check error terminal status
		if (GPDMA_IntGetStatus(GPDMA_STAT_INTERR, 1)){
			// Clear error counter Interrupt pending
			GPDMA_ClearIntPending (GPDMA_STATCLR_INTERR, 1);
			Channel1_Err++;
			xprintf("ch1_Err:%d\n",Channel0_Err);
		}
	}
	xprintf("Rx:0x%x\n",LED_PRECALC1[0][0]);
#endif
/*	xprintf("DMA_IRQHandler GPDMA_STAT_INT:%d\n",GPDMA_IntGetStatus(GPDMA_STAT_INT, 0));
	xprintf("DMA_IRQHandler GPDMA_STAT_INTTC:%d\n",GPDMA_IntGetStatus(GPDMA_STAT_INTTC, 0));
	xprintf("DMA_IRQHandler GPDMA_STAT_INTERR:%d\n",GPDMA_IntGetStatus(GPDMA_STAT_INTERR, 0));
	xprintf("DMA_IRQHandler GPDMA_STAT_RAWINTTC:%d\n",GPDMA_IntGetStatus(GPDMA_STAT_RAWINTTC, 0));
	xprintf("DMA_IRQHandler GPDMA_STAT_RAWINTERR:%d\n",GPDMA_IntGetStatus(GPDMA_STAT_RAWINTERR, 0));
	xprintf("DMA_IRQHandler GPDMA_STAT_ENABLED_CH:%d\n",GPDMA_IntGetStatus(GPDMA_STAT_ENABLED_CH, 0));
*/
}

void SSP0_IRQHandler (void)
{
//	SSP_ClearIntPending(LED_SPI_CHN, uint32_t IntType)
}

void LED_init(){
	uint8_t tmp;

	setPot(0x50);

	SENDSEQ=0;
	DELAY_TIME=1; //so RIT ms is not set to 0
	NEXT_DELAY_TIME=1; //so RIT ms is not set to 0
	SEND_BIT=0;
	NEXT_SEND_BIT=0;
//	UPDATE_COUNT=0;
//	LED_UPDATE_REQUIRED=0;
//	LED_SEND=0;

	GPIO_SetDir(LED_OE_PORT, LED_OE_BIT, 1);
	GPIO_SetValue(LED_OE_PORT, LED_OE_BIT);//turn off leds active low

	GPIO_SetDir(LED_LE_PORT, LED_LE_BIT, 1);
	GPIO_ClearValue(LED_LE_PORT, LED_LE_BIT);

	LatchIn();//reset

	for (tmp=0;tmp<MAX_BAM_BITS;tmp++){
		SEQ_BIT[tmp] = BITORDER[tmp];
	}
	for (tmp=0;tmp<MAX_BAM_BITS;tmp++){
		SEQ_TIME[tmp] = BITTIME[BITORDER[tmp]];
	}
	//clear led bits
	for(uint8_t a=0; a<LEDS; a++){
		LED_RAW[a]=0;
	}
	//clear precalc bits
	for(uint8_t a=0; a<REGS; a++){
		for (uint8_t b=0;b<BITS;b++){
			LED_PRECALC[a][b]=0;
		}
	}
	for(uint8_t a=0; a<REGS; a++){
		for (uint8_t b=0;b<BITS;b++){
			LED_PRECALC1[a][b]=0;
		}
	}
	for(uint8_t a=0; a<REGS; a++){
		for (uint8_t b=0;b<BITS;b++){
			LED_PRECALC2[a][b]=0;
		}
	}
	resetLeds();

#if 0
	//which bit in register
	tmp = 0;
	for(uint8_t a=0; a<LEDS; a++){
		BITINREG[a]=tmp++;
		if (tmp == 15)
			tmp = 0;
	}
	//which register
	tmp = 0;
	for(uint8_t a=0; a<LEDS; a++,tmp++){
		WHICHREG[a]=tmp2;
		if (tmp == 15)
			tmp2++;
	}
#endif

	// Initialize SPI pin connect
	PINSEL_CFG_Type PinCfg;
	/* LE1 */
	PinCfg.Funcnum   = PINSEL_FUNC_0;
	PinCfg.OpenDrain = PINSEL_PINMODE_NORMAL;
	PinCfg.Pinmode   = PINSEL_PINMODE_PULLDOWN;
	PinCfg.Pinnum    = LED_LE_PIN;
	PinCfg.Portnum   = LED_LE_PORT;
	PINSEL_ConfigPin(&PinCfg);
	/* SSEL1 */
	PinCfg.Funcnum   = PINSEL_FUNC_0;
	PinCfg.OpenDrain = PINSEL_PINMODE_NORMAL;
	PinCfg.Pinmode   = PINSEL_PINMODE_PULLDOWN;
	PinCfg.Pinnum    = LED_OE_PIN;
	PinCfg.Portnum   = LED_OE_PORT;
	PINSEL_ConfigPin(&PinCfg);
	/* SCK1 */
	PinCfg.Funcnum   = PINSEL_FUNC_2;
	PinCfg.OpenDrain = PINSEL_PINMODE_NORMAL;
	PinCfg.Pinmode   = PINSEL_PINMODE_PULLUP;
	PinCfg.Pinnum    = LED_SCK_PIN;
	PinCfg.Portnum   = LED_SCK_PORT;
	PINSEL_ConfigPin(&PinCfg);
	/* MISO1 */
	PinCfg.Funcnum   = PINSEL_FUNC_2;
	PinCfg.OpenDrain = PINSEL_PINMODE_NORMAL;
	PinCfg.Pinmode   = PINSEL_PINMODE_PULLUP;
	PinCfg.Pinnum    = LED_MISO_PIN;
	PinCfg.Portnum   = LED_MISO_PORT;
	PINSEL_ConfigPin(&PinCfg);
	PinCfg.Funcnum   = PINSEL_FUNC_2;
	/* MOSI1 */
	PinCfg.Funcnum   = PINSEL_FUNC_2;
	PinCfg.OpenDrain = PINSEL_PINMODE_NORMAL;
	PinCfg.Pinmode   = PINSEL_PINMODE_PULLUP;
	PinCfg.Pinnum    = LED_MOSI_PIN;
	PinCfg.Portnum   = LED_MOSI_PORT;
	PINSEL_ConfigPin(&PinCfg);

	/* initialize SSP configuration structure */
	SSP_CFG_Type SSP_ConfigStruct;
//	SSP_ConfigStructInit(&SSP_ConfigStruct);
	SSP_ConfigStruct.CPHA = SSP_CPHA_SECOND;
//	SSP_ConfigStruct.CPOL = SSP_CPOL_HI;
//	SSP_ConfigStruct.CPHA = SSP_CPHA_FIRST;
	SSP_ConfigStruct.CPOL = SSP_CPOL_LO;
	SSP_ConfigStruct.ClockRate = SSP_SPEED; /* TLC5927 max freq = 30Mhz */
	SSP_ConfigStruct.FrameFormat = SSP_FRAME_SPI;
	SSP_ConfigStruct.Databit = SSP_DATABIT_16;
	SSP_ConfigStruct.Mode = SSP_MASTER_MODE;
	SSP_Init(LED_SPI_CHN, &SSP_ConfigStruct);

	/* Enable SSP peripheral */
	SSP_Cmd(LED_SPI_CHN, ENABLE);

	// Turn all LEDS off
	xprintf(OK "// Turn all LEDS off");FFL_();
	for (uint8_t i=0;i<REGS;i++){
//		xprintf(OK "%d",i+1);FFL_();
		SSP_SendData(LED_SPI_CHN, 0x0000);
		while(LED_SPI_CHN->SR & SSP_STAT_BUSY);
	};
	GPIO_SetValue(LED_LE_PORT, LED_LE_BIT);
	GPIO_ClearValue(LED_LE_PORT, LED_LE_BIT);
	GPIO_ClearValue(LED_OE_PORT, LED_OE_BIT);//LED's on. active low
//	delay_ms(10000);
	xprintf(OK "// All LEDS should be off");FFL_();

#if 1 // If using interupts
	TIM_TIMERCFG_Type TIM_ConfigStruct;
	TIM_MATCHCFG_Type TIM_MatchConfigStruct;

	// Initialize timer 0, prescale count time of 1us //1000000uS = 1S
	TIM_ConfigStruct.PrescaleOption = TIM_PRESCALE_USVAL;
	TIM_ConfigStruct.PrescaleValue	= 1;

	// use channel 0, MR0
	TIM_MatchConfigStruct.MatchChannel = 0;
	// Enable interrupt when MR0 matches the value in TC register
	TIM_MatchConfigStruct.IntOnMatch   = TRUE;
	//Enable reset on MR0: TIMER will reset if MR0 matches it
//	TIM_MatchConfigStruct.ResetOnMatch = FALSE;
	TIM_MatchConfigStruct.ResetOnMatch = TRUE;
	//Stop on MR0 if MR0 matches it
	TIM_MatchConfigStruct.StopOnMatch  = FALSE;
	//Toggle MR0.0 pin if MR0 matches it
	TIM_MatchConfigStruct.ExtMatchOutputType = TIM_EXTMATCH_NOTHING;
	// Set Match value, count value of 1000000 (1000000 * 1uS = 1000000us = 1s --> 1 Hz)
//	TIM_MatchConfigStruct.MatchValue   = DELAY_TIME*1000000;;
	TIM_MatchConfigStruct.MatchValue   = BITTIME[1];

	// Set configuration for Tim_config and Tim_MatchConfig
	TIM_Init(LPC_TIM0, TIM_TIMER_MODE,&TIM_ConfigStruct);
	TIM_ConfigMatch(LPC_TIM0,&TIM_MatchConfigStruct);

	xprintf(OK "TIM_ConfigMatch");FFL_();

	/* preemption = 1, sub-priority = 1 */
	NVIC_SetPriority(TIMER0_IRQn, 0);
	/* Enable interrupt for timer 0 */
	NVIC_EnableIRQ(TIMER0_IRQn);
	// To start timer 0
	TIM_Cmd(LPC_TIM0,ENABLE);
#endif
//	FIO_ClearValue(LED_OE_PORT, LED_OE_BIT);//LED's on. active low
#ifdef DMA
//	GPDMA_Channel_CFG_Type GPDMACfg;
	/* Configure GPDMA channel 0 -------------------------------------------------------------*/
	NVIC_SetPriority(DMA_IRQn, 0);	// set according to main.c
	NVIC_EnableIRQ(DMA_IRQn);
	GPDMA_Init();				// Initialize GPDMA controller */
	NVIC_DisableIRQ (DMA_IRQn);	// Disable interrupt for DMA
	NVIC_SetPriority(DMA_IRQn, 0);	// set according to main.c

	uint8_t reg, bit, linkerListNo;
	GPDMACfg.ChannelNum = 0;	// DMA Channel 0
	GPDMACfg.SrcMemAddr = 0;	// Source memory - not used - will be sent in interrupt so independent bit Linker Lists can be chosen
	GPDMACfg.DstMemAddr = 0;	// Destination memory - not used - only used when destination is memory
	GPDMACfg.TransferSize = 1;	// Transfer size
	GPDMACfg.TransferWidth = GPDMA_WIDTH_HALFWORD;	// Transfer width - not used
	GPDMACfg.TransferType = GPDMA_TRANSFERTYPE_M2P;	// Transfer type
	GPDMACfg.SrcConn = 0;		// Source connection - not used
	GPDMACfg.DstConn = GPDMA_CONN_SSP0_Tx;	// Destination connection - not used
	GPDMACfg.DMALLI = (uint32_t) &LinkerList[0][0];	// Linker List Item - Pointer to linker list
	GPDMA_Setup(&GPDMACfg);		// Setup channel with given parameter
//	Channel0_TC = 0;			// Reset terminal counter
//	Channel0_Err = 0;			// Reset Error counter

	// Linker list 32bit Control
	uint32_t LinkerListControl = 0;
	LinkerListControl = GPDMA_DMACCxControl_TransferSize((uint32_t)GPDMACfg.TransferSize) \
						| GPDMA_DMACCxControl_SBSize((uint32_t)GPDMA_BSIZE_1) \
						| GPDMA_DMACCxControl_DBSize((uint32_t)GPDMA_BSIZE_1) \
						| GPDMA_DMACCxControl_SWidth((uint32_t)GPDMACfg.TransferWidth) \
						| GPDMA_DMACCxControl_DWidth((uint32_t)GPDMACfg.TransferWidth) \
						| GPDMA_DMACCxControl_SI;

	for (bit=0;bit<BITS;bit++){
		linkerListNo=0;
		for (reg=5; 0<reg;reg--,linkerListNo++){
			xprintf("bit:%d reg:%d SrcAddr:0x%x DstAddr:0x%x NextLLI:0x%x linkerListNo:0x%x\n",bit,reg,(uint32_t) &LED_PRECALC[reg][bit],(uint32_t) &LPC_SSP0->DR,(uint32_t) &LinkerList[linkerListNo+1][bit],linkerListNo);
			LinkerList[linkerListNo][bit].SrcAddr = (uint32_t) &LED_PRECALC[reg][bit];	/**< Source Address */
			LinkerList[linkerListNo][bit].DstAddr = (uint32_t) &LPC_SSP0->DR;			/**< Destination address */
			LinkerList[linkerListNo][bit].NextLLI = (uint32_t) &LinkerList[linkerListNo+1][bit];	/**< Next LLI address, otherwise set to '0' */
			LinkerList[linkerListNo][bit].Control = LinkerListControl;
			xprintf("SrcAddr:0x%x DstAddr:0x%x NextLLI:0x%x\n",(uint32_t) &LinkerList[linkerListNo][bit].SrcAddr,(uint32_t) &LinkerList[linkerListNo][bit].DstAddr,(uint32_t) &LinkerList[linkerListNo][bit].NextLLI,(uint32_t) &LinkerList[linkerListNo][bit].Control);
		}
//		if (reg==0){
		xprintf("bit:%d reg:%d SrcAddr:0x%x DstAddr:0x%x NextLLI:0x%x linkerListNo:0x%x\n",bit,reg,(uint32_t) &LED_PRECALC[reg][bit],(uint32_t) &LPC_SSP0->DR,(uint32_t) &LinkerList[linkerListNo+1][bit],linkerListNo);
		LinkerList[linkerListNo][bit].SrcAddr = (uint32_t) &LED_PRECALC[reg][bit];	/**< Source Address */
		LinkerList[linkerListNo][bit].DstAddr = (uint32_t) &LPC_SSP0->DR;			/**< Destination address */
		LinkerList[linkerListNo][bit].NextLLI = (uint32_t) &LinkerList[linkerListNo+1][bit];/**< Next LLI address, otherwise set to '0' */
		LinkerList[linkerListNo][bit].Control = LinkerListControl;
		linkerListNo++;
		xprintf("SrcAddr:0x%x DstAddr:0x%x NextLLI:0x%x\n",(uint32_t) &LinkerList[linkerListNo][bit].SrcAddr,(uint32_t) &LinkerList[linkerListNo][bit].DstAddr,(uint32_t) &LinkerList[linkerListNo][bit].NextLLI,(uint32_t) &LinkerList[linkerListNo][bit].Control);
//		}
		for (reg=11; reg>6;reg--,linkerListNo++){
			xprintf("bit:%d reg:%d SrcAddr:0x%x DstAddr:0x%x NextLLI:0x%x linkerListNo:0x%x\n",bit,reg,(uint32_t) &LED_PRECALC[reg][bit],(uint32_t) &LPC_SSP0->DR,(uint32_t) &LinkerList[linkerListNo+1][bit],linkerListNo);
			LinkerList[linkerListNo][bit].SrcAddr = (uint32_t) &LED_PRECALC[reg][bit];	/**< Source Address */
			LinkerList[linkerListNo][bit].DstAddr = (uint32_t) &LPC_SSP0->DR;			/**< Destination address */
			LinkerList[linkerListNo][bit].NextLLI = (uint32_t) &LinkerList[linkerListNo+1][bit];	/**< Next LLI address, otherwise set to '0' */
			LinkerList[linkerListNo][bit].Control = LinkerListControl;
			xprintf("SrcAddr:0x%x DstAddr:0x%x NextLLI:0x%x\n",(uint32_t) &LinkerList[linkerListNo][bit].SrcAddr,(uint32_t) &LinkerList[linkerListNo][bit].DstAddr,(uint32_t) &LinkerList[linkerListNo][bit].NextLLI,(uint32_t) &LinkerList[linkerListNo][bit].Control);
		}
//		if (reg==7){
		xprintf("bit:%d reg:%d SrcAddr:0x%x DstAddr:0x%x NextLLI:0x%x linkerListNo:0x%x\n",bit,reg,(uint32_t) &LED_PRECALC[reg][bit],(uint32_t) &LPC_SSP0->DR,(uint32_t) &LinkerList[linkerListNo+1][bit],linkerListNo);
		LinkerList[linkerListNo][bit].SrcAddr = (uint32_t) &LED_PRECALC[reg][bit];	/**< Source Address */
		LinkerList[linkerListNo][bit].DstAddr = (uint32_t) &LPC_SSP0->DR;			/**< Destination address */
		LinkerList[linkerListNo][bit].NextLLI = 0;									/**< Next LLI address, otherwise set to '0' */
		LinkerList[linkerListNo][bit].Control = LinkerListControl;
		linkerListNo++;
		xprintf("SrcAddr:0x%x DstAddr:0x%x NextLLI:0x%x NextLLI_V:0x%x\n",(uint32_t) &LinkerList[linkerListNo][bit].SrcAddr,(uint32_t) &LinkerList[linkerListNo][bit].DstAddr,(uint32_t) &LinkerList[linkerListNo][bit].NextLLI,LinkerList[linkerListNo][bit].NextLLI);
//		}
	}
	xprintf(OK "DMA Setup");FFL_();
	SSP_DMACmd (LED_SPI_CHN, SSP_DMA_TX, ENABLE);	// Enable Tx DMA on SSP0
//	GPDMA_ChannelCmd(0, ENABLE);	// Enable GPDMA channel 0
	NVIC_EnableIRQ (DMA_IRQn);		// Enable interrupt for DMA
#endif
#if 0 // SSP Rx DMA
	GPDMA_Channel_CFG_Type GPDMACfg1;
	/* Configure GPDMA channel 1 -------------------------------------------------------------*/
	GPDMACfg1.ChannelNum = 1;	// DMA Channel 0
	GPDMACfg1.SrcMemAddr = 0;	// Source memory - not used - will be sent in interrupt so independent bit Linker Lists can be chosen
	GPDMACfg1.DstMemAddr = (uint32_t) &LED_PRECALC1[0][0];	// Destination memory - not used - only used when destination is memory
	GPDMACfg1.TransferSize = 1;	// Transfer size
	GPDMACfg1.TransferWidth = GPDMA_WIDTH_HALFWORD;	// Transfer width
	GPDMACfg1.TransferType = GPDMA_TRANSFERTYPE_P2M;	// Transfer type
	GPDMACfg1.SrcConn = GPDMA_CONN_SSP0_Rx;		// Source connection - not used
	GPDMACfg1.DstConn = 0;	// Destination connection - not used
	GPDMACfg1.DMALLI = 0;	// Linker List Item - Pointer to linker list
	GPDMA_Setup(&GPDMACfg1);		// Setup channel with given parameter
//	Channel1_TC = 0;			// Reset terminal counter
//	Channel1_Err = 0;			// Reset Error counter
	xprintf(OK "DMA Rx Setup");FFL_();
	SSP_DMACmd (LED_SPI_CHN, SSP_DMA_RX, ENABLE);	// Enable Tx DMA on SSP0
//	GPDMA_ChannelCmd(1, ENABLE);	// Enable GPDMA channel 0
#endif
}


void LED_test(){
#if 0
	while(1){
		for (uint8_t l=0;l<REGS;l++){
			for (uint8_t i=0;i<15;i++){
				xprintf(OK "%d",l*REGS+i+1);FFL_();
				GPIO_ClearValue(LED_LE_PORT, LED_LE_BIT);
				for (uint8_t j=0;j<(REGS-l);j++){
					SSP_SendData(LED_SPI_CHN, 0x0000);
					while(LED_SPI_CHN->SR & SSP_STAT_BUSY);
				}
				SSP_SendData(LED_SPI_CHN, 0x0001<<i);
				while(LED_SPI_CHN->SR & SSP_STAT_BUSY);
				for (uint8_t k=0;k<(0+l);k++){
					SSP_SendData(LED_SPI_CHN, 0x0000);
					while(LED_SPI_CHN->SR & SSP_STAT_BUSY);
				}
				GPIO_SetValue(LED_LE_PORT, LED_LE_BIT);
				GPIO_ClearValue(LED_LE_PORT, LED_LE_BIT);
				GPIO_ClearValue(LED_OE_PORT, LED_OE_BIT);//LED's on. active low
				delay_ms(2);
			}
		}
	}
#endif
#if 0 //simple all colors
//	while(1){
	for (uint8_t i=0;i<16;i++){
		xprintf(OK "%d",i+1);FFL_();
		GPIO_ClearValue(LED_LE_PORT, LED_LE_BIT);
//		for (uint8_t j=0;j<11;j++){
//			SSP_SendData(LED_SPI_CHN, 0x0000);
//			while(LED_SPI_CHN->SR & SSP_STAT_BUSY);
//		}
//		SSP_SendData(LED_SPI_CHN, 0x0001<<i);
		SSP_SendData(LED_SPI_CHN, 0x1249);
		while(LED_SPI_CHN->SR & SSP_STAT_BUSY);
//		for (uint8_t k=0;k<0;k++){
//			SSP_SendData(LED_SPI_CHN, 0x0000);
//			while(LED_SPI_CHN->SR & SSP_STAT_BUSY);
//		}
		GPIO_SetValue(LED_LE_PORT, LED_LE_BIT);
		GPIO_ClearValue(LED_LE_PORT, LED_LE_BIT);
		GPIO_ClearValue(LED_OE_PORT, LED_OE_BIT);//LED's on. active low
		delay_ms(150);
	};
	for (uint8_t i=0;i<16;i++){
		xprintf(OK "%d",i+1);FFL_();
		GPIO_ClearValue(LED_LE_PORT, LED_LE_BIT);
//		for (uint8_t j=0;j<11;j++){
//			SSP_SendData(LED_SPI_CHN, 0x0000);
//			while(LED_SPI_CHN->SR & SSP_STAT_BUSY);
//		}
//		SSP_SendData(LED_SPI_CHN, 0x0001<<i);
		SSP_SendData(LED_SPI_CHN, 0x2492);
		while(LED_SPI_CHN->SR & SSP_STAT_BUSY);
//		for (uint8_t k=0;k<0;k++){
//			SSP_SendData(LED_SPI_CHN, 0x0000);
//			while(LED_SPI_CHN->SR & SSP_STAT_BUSY);
//		}
		GPIO_SetValue(LED_LE_PORT, LED_LE_BIT);
		GPIO_ClearValue(LED_LE_PORT, LED_LE_BIT);
		GPIO_ClearValue(LED_OE_PORT, LED_OE_BIT);//LED's on. active low
		delay_ms(150);
	};
	for (uint8_t i=0;i<16;i++){
		xprintf(OK "%d",i+1);FFL_();
		GPIO_ClearValue(LED_LE_PORT, LED_LE_BIT);
//		for (uint8_t j=0;j<11;j++){
//			SSP_SendData(LED_SPI_CHN, 0x0000);
//			while(LED_SPI_CHN->SR & SSP_STAT_BUSY);
//		}
//		SSP_SendData(LED_SPI_CHN, 0x0001<<i);
		SSP_SendData(LED_SPI_CHN, 0x4924);
		while(LED_SPI_CHN->SR & SSP_STAT_BUSY);
//		for (uint8_t k=0;k<0;k++){
//			SSP_SendData(LED_SPI_CHN, 0x0000);
//			while(LED_SPI_CHN->SR & SSP_STAT_BUSY);
//		}
		GPIO_SetValue(LED_LE_PORT, LED_LE_BIT);
		GPIO_ClearValue(LED_LE_PORT, LED_LE_BIT);
		GPIO_ClearValue(LED_OE_PORT, LED_OE_BIT);//LED's on. active low
		delay_ms(150);
	};
	for (uint8_t i=0;i<16;i++){
		xprintf(OK "%d",i+1);FFL_();
		GPIO_ClearValue(LED_LE_PORT, LED_LE_BIT);
//		for (uint8_t j=0;j<11;j++){
//			SSP_SendData(LED_SPI_CHN, 0x0000);
//			while(LED_SPI_CHN->SR & SSP_STAT_BUSY);
//		}
//		SSP_SendData(LED_SPI_CHN, 0x0001<<i);
		SSP_SendData(LED_SPI_CHN, 0xffff);
		while(LED_SPI_CHN->SR & SSP_STAT_BUSY);
//		for (uint8_t k=0;k<0;k++){
//			SSP_SendData(LED_SPI_CHN, 0x0000);
//			while(LED_SPI_CHN->SR & SSP_STAT_BUSY);
//		}
		GPIO_SetValue(LED_LE_PORT, LED_LE_BIT);
		GPIO_ClearValue(LED_LE_PORT, LED_LE_BIT);
		GPIO_ClearValue(LED_OE_PORT, LED_OE_BIT);//LED's on. active low
		delay_ms(150);
	};
//	}

#endif
#if 0 //led test
	while(1){
	for(uint8_t led=0; led<LEDS;led++){
		SetLED(led,0xff);
		calulateLEDMIBAMBits();
		delay_ms(100);
		SetLED(led,0x00);
	}
	}

#endif
#if 1
//	while(1){
	for(uint8_t led=0;led<LEDS;led+=3){ // loop over all LED's
//		xprintf("LED no %d ",(led/3));
//		if ((led % 3)==0) xprintf("Red\n");
//		if ((led % 3)==1) xprintf("Green\n");
//		if ((led % 3)==2) xprintf("Blue\n");
		for(int16_t b=0;b<=0x7f;b++){
			SetLED(led,b);
			calulateLEDMIBAMBits();
			delay_ms(1);
		}
//		for(int16_t b=0x7f;b>=0;b--){
//			SetLED(led,b);
//			calulateLEDMIBAMBits();
//			delay_ms(1);
//		}
	}
//	}
#endif
}

void SetRGB(int32_t group, uint8_t v0, uint8_t v1, uint8_t v2){
//	if(group<0)
//		group = RGBS + group;
/*
	if(group==-1)
		group = 59;
	if(group==-2)
		group = 58;
	if(group==-3)
		group = 57;
	if(group==-4)
		group = 56;
	if(group==-5)
		group = 55;
	if(group==-6)
		group = 54;
*/
	LED_RAW[group*3]=v0 & 0x7F;
	LED_RAW[group*3+1]=v1 & 0x7F;
	LED_RAW[group*3+2]=v2 & 0x7F;
}
void SetLED(uint8_t led, uint8_t v0){
	LED_RAW[led]=v0 & 0x7F;
}

void resetLeds(void){
	//clear led bits
	for(uint8_t a=0; a<LEDS; a++){
		LED_RAW[a]=0;
	}
	calulateLEDMIBAMBits();
}
//LED_PRECALC[reg][bit] = (0x01<<bit) & LED_RAW[led];

void calulateLEDMIBAMBits(){
//	xprintf(INFO "calulateLEDMIBAMBits()");
	uint8_t led,bitinreg;
//	xprintf("bit reg LED_PRECALC[bit][reg]\n");
	for(uint8_t bit=0; bit<BITS; bit++){
//	uint32_t bit=7;
		led=0;
		for(uint8_t reg=0; reg<REGS; reg++){
			bitinreg=0;
/*
			_DBG("[INFO]-LED_RAW[0]= ");_DBH(LED_RAW[0]);_DBG(" (");_DBG(__FILE__);_DBG(":");_DBD16(__LINE__);_DBG(")\r\n");
			_DBG("[INFO]-bitinreg= ");_DBH(bitinreg);_DBG(" (");_DBG(__FILE__);_DBG(":");_DBD16(__LINE__);_DBG(")\r\n");
			_DBG("[INFO]-(LED_RAW[led++]<<bitinreg)&bitinreg++= ");_DBH16((LED_RAW[led++]<<bitinreg)&(1<<bitinreg++));_DBG("\r\n");//;_DBG(" (");_DBG(__FILE__);_DBG(":");_DBD16(__LINE__);_DBG(")\r\n");
			_DBG("[INFO]-(LED_RAW[led++]<<bitinreg)&bitinreg++= ");_DBH16((LED_RAW[led++]<<bitinreg)&(1<<bitinreg++));_DBG("\r\n");//;_DBG(" (");_DBG(__FILE__);_DBG(":");_DBD16(__LINE__);_DBG(")\r\n");
			_DBG("[INFO]-(LED_RAW[led++]<<bitinreg)&bitinreg++= ");_DBH16((LED_RAW[led++]<<bitinreg)&(1<<bitinreg++));_DBG("\r\n");//;_DBG(" (");_DBG(__FILE__);_DBG(":");_DBD16(__LINE__);_DBG(")\r\n");
			_DBG("[INFO]-(LED_RAW[led++]<<bitinreg)&bitinreg++= ");_DBH16((LED_RAW[led++]<<bitinreg)&(1<<bitinreg++));_DBG("\r\n");//;_DBG(" (");_DBG(__FILE__);_DBG(":");_DBD16(__LINE__);_DBG(")\r\n");
			_DBG("[INFO]-(LED_RAW[led++]<<bitinreg)&bitinreg++= ");_DBH16((LED_RAW[led++]<<bitinreg)&(1<<bitinreg++));_DBG("\r\n");//;_DBG(" (");_DBG(__FILE__);_DBG(":");_DBD16(__LINE__);_DBG(")\r\n");
			_DBG("[INFO]-(LED_RAW[led++]<<bitinreg)&bitinreg++= ");_DBH16((LED_RAW[led++]<<bitinreg)&(1<<bitinreg++));_DBG("\r\n");//;_DBG(" (");_DBG(__FILE__);_DBG(":");_DBD16(__LINE__);_DBG(")\r\n");
			_DBG("[INFO]-(LED_RAW[led++]<<bitinreg)&bitinreg++= ");_DBH16((LED_RAW[led++]<<bitinreg)&(1<<bitinreg++));_DBG("\r\n");//;_DBG(" (");_DBG(__FILE__);_DBG(":");_DBD16(__LINE__);_DBG(")\r\n");
			_DBG("[INFO]-(LED_RAW[led++]<<bitinreg)&bitinreg++= ");_DBH16((LED_RAW[led++]<<bitinreg)&(1<<bitinreg++));_DBG("\r\n");//;_DBG(" (");_DBG(__FILE__);_DBG(":");_DBD16(__LINE__);_DBG(")\r\n");
			_DBG("[INFO]-(LED_RAW[led++]<<bitinreg)&bitinreg++= ");_DBH16((LED_RAW[led++]<<bitinreg)&(1<<bitinreg++));_DBG("\r\n");//;_DBG(" (");_DBG(__FILE__);_DBG(":");_DBD16(__LINE__);_DBG(")\r\n");
			_DBG("[INFO]-(LED_RAW[led++]<<bitinreg)&bitinreg++= ");_DBH16((LED_RAW[led++]<<bitinreg)&(1<<bitinreg++));_DBG("\r\n");//;_DBG(" (");_DBG(__FILE__);_DBG(":");_DBD16(__LINE__);_DBG(")\r\n");
			_DBG("[INFO]-(LED_RAW[led++]<<bitinreg)&bitinreg++= ");_DBH16((LED_RAW[led++]<<bitinreg)&(1<<bitinreg++));_DBG("\r\n");//;_DBG(" (");_DBG(__FILE__);_DBG(":");_DBD16(__LINE__);_DBG(")\r\n");
			_DBG("[INFO]-(LED_RAW[led++]<<bitinreg)&bitinreg++= ");_DBH16((LED_RAW[led++]<<bitinreg)&(1<<bitinreg++));_DBG("\r\n");//;_DBG(" (");_DBG(__FILE__);_DBG(":");_DBD16(__LINE__);_DBG(")\r\n");
			_DBG("[INFO]-(LED_RAW[led++]<<bitinreg)&bitinreg++= ");_DBH16((LED_RAW[led++]<<bitinreg)&(1<<bitinreg++));_DBG("\r\n");//;_DBG(" (");_DBG(__FILE__);_DBG(":");_DBD16(__LINE__);_DBG(")\r\n");
			_DBG("[INFO]-(LED_RAW[led++]<<bitinreg)&bitinreg++= ");_DBH16((LED_RAW[led++]<<bitinreg)&(1<<bitinreg++));_DBG("\r\n");//;_DBG(" (");_DBG(__FILE__);_DBG(":");_DBD16(__LINE__);_DBG(")\r\n");
			_DBG("[INFO]-(LED_RAW[led++]<<bitinreg)&bitinreg++= ");_DBH16((LED_RAW[led++]<<bitinreg)&(1<<bitinreg++));_DBG("\r\n");//;_DBG(" (");_DBG(__FILE__);_DBG(":");_DBD16(__LINE__);_DBG(")\r\n");
*/
///*
//			bitinreg=0;
//			for (uint_8 rbit=15; rbit<16; rbit++)
//			LED_PRECALC[reg][bit] &= (0x01<<rbit) & LED_RAW[led++]<<rbit;

/*
			for(uint8_t tmp=0; tmp<BITS; tmp++){
				led=0;
				_DBG("[INFO]-led=");_DBH(led);_DBG("\r\n");
				_DBG("[INFO]-LED_RAW[led]=");_DBH(LED_RAW[led]);_DBG("\r\n");
				_DBG("[INFO]-tmp=");_DBD(tmp);_DBG("-");_DBH16((LED_RAW[led++]<<tmp)&(1<<tmp));_DBG("\r\n");
			}
*/
			uint16_t tt[16],l;
			for(uint8_t t=0;t<15;t++,led++){
				l=LED_RAW[led]>>(bit);
//				xprintf(INFO "l=%d 1<<bitinreg=%15b" " (%s:%d)\n",l,1<<bitinreg,_F_,_L_);
				tt[t] = (l<<bitinreg)&(1<<bitinreg);
				tt[15] = 0;
				bitinreg++;
//				if (bit==7){
//					xprintf("bit:%d reg:%d LED_PRECALC[%d][%d]:%d\n",bit,reg,bit,reg,LED_PRECALC[reg][bit]);
//				xprintf("bit reg LED_PRECALC[%d][%d]:%d\n",bit,reg,bit,reg,LED_PRECALC[reg][bit]);
//				xprintf("%d   %d          %d\n",bit,reg,LED_PRECALC[reg][bit]);
//				}
			}
#if 1
			LED_PRECALC[reg][bit] =
				tt[0] |
				tt[1] |
				tt[2] |
				tt[3] |
				tt[4] |
				tt[5] |
				tt[6] |
				tt[7] |
				tt[8] |
				tt[9] |
				tt[10] |
				tt[11] |
				tt[12] |
				tt[13] |
				tt[14] |
				tt[15];

//			if (led==91){//|led==1|led==2){
//				xprintf("reg:%d brightness:%d\n",reg,LED_PRECALC[reg][bit]);
//				xprintf("LED_RAW:%d\n",LED_RAW[led]);
//			}

#else
			if(BUFFER==1){
				LED_PRECALC1[reg][bit] =
					tt[0] |
					tt[1] |
					tt[2] |
					tt[3] |
					tt[4] |
					tt[5] |
					tt[6] |
					tt[7] |
					tt[8] |
					tt[9] |
					tt[10] |
					tt[11] |
					tt[12] |
					tt[13] |
					tt[14];
				BUFFER=1;
			}
			else{
				LED_PRECALC2[reg][bit] =
					tt[0] |
					tt[1] |
					tt[2] |
					tt[3] |
					tt[4] |
					tt[5] |
					tt[6] |
					tt[7] |
					tt[8] |
					tt[9] |
					tt[10] |
					tt[11] |
					tt[12] |
					tt[13] |
					tt[14];
				BUFFER=2;
			}
#endif
//			_DBH16(LED_PRECALC[reg][bit]);_DBG("=LED_PRECALC=");_DBG("\r\n");
/*
			LED_PRECALC[reg][bit] =
					(LED_RAW[led++]<<bitinreg)&(1<<bitinreg++) |
					(LED_RAW[led++]<<bitinreg)&(1<<bitinreg++) |
					(LED_RAW[led++]<<bitinreg)&(1<<bitinreg++) |
					(LED_RAW[led++]<<bitinreg)&(1<<bitinreg++) |
					(LED_RAW[led++]<<bitinreg)&(1<<bitinreg++) |
					(LED_RAW[led++]<<bitinreg)&(1<<bitinreg++) |
					(LED_RAW[led++]<<bitinreg)&(1<<bitinreg++) |
					(LED_RAW[led++]<<bitinreg)&(1<<bitinreg++) |
					(LED_RAW[led++]<<bitinreg)&(1<<bitinreg++) |
					(LED_RAW[led++]<<bitinreg)&(1<<bitinreg++) |
					(LED_RAW[led++]<<bitinreg)&(1<<bitinreg++) |
					(LED_RAW[led++]<<bitinreg)&(1<<bitinreg++) |
					(LED_RAW[led++]<<bitinreg)&(1<<bitinreg++) |
					(LED_RAW[led++]<<bitinreg)&(1<<bitinreg++);
			_DBG("[INFO]-led=");_DBH(led);_DBG("\r\n");
			_DBG("[INFO]-bitinreg=");_DBH(bitinreg);_DBG("\r\n");
*/

//*/
		}
	}
	//	UPDATE_REQUIRED=true;
//	_DBG("[INFO]-MIBAM Precal time = ");_DBD32(end-start);_DBG(" (");_DBG(__FILE__);_DBG(":");_DBD16(__LINE__);_DBG(")\r\n");
}
/*
static inline void calulateLEDMIBAMBit(uint8 LED){
	uint8_t bitinreg = BITINREG[LED];
	uint8_t whichreg = WHICHREG[LED];
//	for(uint32_t bits=0; bit<BITS; bit++){
	LED_PRECALC[whichreg][bit] = LED_RAW[led++]<<bitinreg &
									LED_RAW[led++]<<bitinreg &;
//	}
}
*/
/*
for(uint32_t led=0; led<LEDS; led++)
for(uint32_t bit=0; bit<BITS; bit++)
for(uint32_t reg=0; reg<REGS; reg++)
*/
/*
16bitcol
16bit*60=120bytes
120hz
8ms/refresh
8mhz

sensors

battery

128	160
64	30
32	10
16	4
8	2
4	1.5
2	1.2
1	1

0.0000326797	1/120/255
16000000	16mhz
0.00000006	1/16mhz
522.875817	clocks
*/
