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

//#define MAX_BAM_BITS 16
#define MAX_BAM_BITS 8

const uint32_t BITORDER[] = { 0,1,2,3,4,5,6,7,7,6,5,4,3,2,1,0 };
//const uint32_t BITORDER[] = { 0,7,2,5,4,3,6,1,1,6,3,4,5,2,7,0 };
//const uint32_t BITORDER[] = { 5,3,1,7,2,4,6,0,5,3,1,7,2,4,6 };

#define START_TIME 4096 //largest time inteval??
//#define START_TIME 2000 //largest time inteval??
//#define START_TIME 32 //smallest time inteval 33us
//#define START_TIME 32/2 //fastest possible
//#define START_TIME 32*1000 //for uart

volatile uint32_t SENDSEQ;
volatile uint32_t DELAY_TIME; //so RIT ms is not set to 0
volatile uint32_t NEXT_DELAY_TIME; //so RIT ms is not set to 0
volatile uint32_t SEND_BIT;
volatile uint32_t NEXT_SEND_BIT;
volatile uint32_t LED_SEND;

/* For DMA controller */
#define DMA_DATA_SIZE	65

// Terminal Counter flag for Channel 0
__IO uint32_t Channel0_TC;

// Error Counter flag for Channel 0
__IO uint32_t Channel0_Err;

// DMA source variable
uint8_t dma_src[DMA_DATA_SIZE];

// DMA source variable
uint8_t dma_dst[DMA_DATA_SIZE];

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
#if 1
#define REGS 1 //12
#define RGBS 5 //60
#define LEDS RGBS*3
#define BITS 8
#else
#define REGS 12
#define RGBS 60
#define LEDS RGBS*3
#define BITS 8
#endif
uint16_t SEQ_BIT[16];
uint32_t SEQ_TIME[16];

volatile uint32_t LED_RAW[LEDS];
volatile uint32_t LED_PRECALC[REGS][BITS];
volatile uint32_t LED_PRECALC1[REGS][BITS];
volatile uint32_t LED_PRECALC2[REGS][BITS];
volatile uint32_t BUFFER=1;

//uint16_t BITINREG[LEDS];
//uint16_t WHICHREG[LEDS];
//uint32_t LAST_MS;
//uint32_t TOTAL_MS;
extern volatile uint32_t TOG[4];
volatile uint32_t UPDATE_COUNT;
volatile uint32_t LED_UPDATE_REQUIRED;

void TIMER0_IRQHandler(void){
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

		for(uint32_t reg=0; reg<REGS;reg++){
#if 0
			if(BUFFER==1)
				SSP_SendData(LED_SPI_CHN, LED_PRECALC1[reg][SEND_BIT]);
			else
				SSP_SendData(LED_SPI_CHN, LED_PRECALC2[reg][SEND_BIT]);
			//SSP_SendData(LED_SPI_CHN, LED_PRECALC[reg][SEND_BIT]<<6);
#endif
			//SSP_SendData(LED_SPI_CHN, LED_PRECALC[reg][SEND_BIT]);
			LED_SPI_CHN->DR = LED_PRECALC[reg][SEND_BIT];

			//WaitForSend();//Wait if TX buffer full
			while(LED_SPI_CHN->SR & SSP_STAT_BUSY);
		}
		//LatchIn();
		LPC_GPIO0->FIOSET = LED_LE_BIT;
		LPC_GPIO0->FIOCLR = LED_LE_BIT;

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

inline void LatchIn(void){
#if 0
	for(uint32_t i=0;i<15;i++){
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
	// check GPDMA interrupt on channel 0
	if (GPDMA_IntGetStatus(GPDMA_STAT_INT, 0)){
		// Check counter terminal status
		if(GPDMA_IntGetStatus(GPDMA_STAT_INTTC, 0)){
			// Clear terminate counter Interrupt pending
			GPDMA_ClearIntPending (GPDMA_STATCLR_INTTC, 0);
				Channel0_TC++;
		}
		// Check error terminal status
		if (GPDMA_IntGetStatus(GPDMA_STAT_INTERR, 0)){
			// Clear error counter Interrupt pending
			GPDMA_ClearIntPending (GPDMA_STATCLR_INTERR, 0);
			Channel0_Err++;
		}
	}
}


void LED_init(){
	uint32_t tmp;

	SENDSEQ=0;
	DELAY_TIME=1; //so RIT ms is not set to 0
	NEXT_DELAY_TIME=1; //so RIT ms is not set to 0
	SEND_BIT=0;
	NEXT_SEND_BIT=0;
	UPDATE_COUNT=0;
	LED_UPDATE_REQUIRED=0;
	LED_SEND=0;

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
	for(uint32_t a=0; a<LEDS; a++){
		LED_RAW[a]=0;
	}
	//clear precalc bits
	for(uint32_t a=0; a<REGS; a++){
		for (uint32_t b=0;b<BITS;b++){
			LED_PRECALC1[a][b]=0;
		}
	}
	for(uint32_t a=0; a<REGS; a++){
		for (uint32_t b=0;b<BITS;b++){
			LED_PRECALC2[a][b]=0;
		}
	}
	for(uint32_t a=0; a<REGS; a++){
		for (uint32_t b=0;b<BITS;b++){
			LED_PRECALC[a][b]=0;
		}
	}
	resetLeds();

#if 0
	//which bit in register
	tmp = 0;
	for(uint32_t a=0; a<LEDS; a++){
		BITINREG[a]=tmp++;
		if (tmp == 15)
			tmp = 0;
	}
	//which register
	tmp = 0;
	for(uint32_t a=0; a<LEDS; a++,tmp++){
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
	PinCfg.Pinmode   = PINSEL_PINMODE_PULLUP;
	PinCfg.Pinnum    = LED_LE_PIN;
	PinCfg.Portnum   = LED_LE_PORT;
	PINSEL_ConfigPin(&PinCfg);
	/* SSEL1 */
	PinCfg.Funcnum   = PINSEL_FUNC_0;
	PinCfg.OpenDrain = PINSEL_PINMODE_NORMAL;
	PinCfg.Pinmode   = PINSEL_PINMODE_PULLUP;
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
	SSP_ConfigStructInit(&SSP_ConfigStruct);
	SSP_ConfigStruct.CPHA = SSP_CPHA_SECOND;
//	SSP_ConfigStruct.CPOL = SSP_CPOL_HI;
//	SSP_ConfigStruct.CPHA = SSP_CPHA_FIRST;
	SSP_ConfigStruct.CPOL = SSP_CPOL_LO;
//	SSP_ConfigStruct.ClockRate = 500000;
//	SSP_ConfigStruct.ClockRate = 10000000;
	SSP_ConfigStruct.ClockRate = 30000000; /* TLC5927 max freq = 30Mhz */
	SSP_ConfigStruct.FrameFormat = SSP_FRAME_SPI;
	SSP_ConfigStruct.Databit = SSP_DATABIT_16;
	SSP_ConfigStruct.Mode = SSP_MASTER_MODE;
	SSP_Init(LED_SPI_CHN, &SSP_ConfigStruct);

	/* Enable SSP peripheral */
	SSP_Cmd(LED_SPI_CHN, ENABLE);

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

	/* preemption = 1, sub-priority = 1 */
	NVIC_SetPriority(TIMER0_IRQn, 0);
	/* Enable interrupt for timer 0 */
	NVIC_EnableIRQ(TIMER0_IRQn);
	// To start timer 0
	TIM_Cmd(LPC_TIM0,ENABLE);
	FIO_ClearValue(LED_OE_PORT, LED_OE_BIT);//LED's on. active low
#if 0 //dma not used yet
	GPDMA_Channel_CFG_Type GPDMACfg;

	/* GPDMA Interrupt configuration section ------------------------------------------------- */
	NVIC_SetPriority(DMA_IRQn, 0); // set according to main.c
	/* Enable SSP0 interrupt */
	NVIC_EnableIRQ(DMA_IRQn);

	/* Initializing Buffer section ----------------------------------------------------------- */
	Buffer_Init();

	/* Initialize GPDMA controller */
	GPDMA_Init();


	/* Setting GPDMA interrupt */
	// Disable interrupt for DMA
	NVIC_DisableIRQ (DMA_IRQn);
	NVIC_SetPriority(DMA_IRQn, 0); // set according to main.c


	/* Configure GPDMA channel 0 -------------------------------------------------------------*/
	/* DMA Channel 0 */
	GPDMACfg.ChannelNum = 0;
	// Source memory
	GPDMACfg.SrcMemAddr = (uint32_t) &dma_src;
	// Destination memory - Not used
	GPDMACfg.DstMemAddr = 0;
	// Transfer size
	GPDMACfg.TransferSize = sizeof(dma_src);
	// Transfer width - not used
	GPDMACfg.TransferWidth = 0;
	// Transfer type
	GPDMACfg.TransferType = GPDMA_TRANSFERTYPE_M2P;
	// Source connection - unused
	GPDMACfg.SrcConn = 0;
	// Destination connection
	GPDMACfg.DstConn = GPDMA_CONN_SSP0_Tx; //TODO this needs to be different
	// Linker List Item - unused
	GPDMACfg.DMALLI = 0;
	// Setup channel with given parameter
	GPDMA_Setup(&GPDMACfg);
	/* Reset terminal counter */
	Channel0_TC = 0;
	/* Reset Error counter */
	Channel0_Err = 0;

//	_DBG_("Start transfer...");

    // Enable Tx DMA on SSP0
	SSP_DMACmd (LED_SPI_CHN, SSP_DMA_TX, ENABLE);

	// Enable GPDMA channel 0
	GPDMA_ChannelCmd(0, ENABLE);

    // Enable interrupt for DMA
    NVIC_EnableIRQ (DMA_IRQn);

	/* Wait for GPDMA processing complete */
	while (((Channel0_TC == 0) && (Channel0_Err == 0)) \
			|| ((Channel1_TC == 0) && (Channel1_Err ==0)));

	/* Verify buffer */
//	Buffer_Verify();
#endif

}


void LED_test(){
//#if 1 //led test
	for(uint32_t led=0; led<LEDS;led++){
		SetLED(led,1);
		calulateLEDMIBAMBits();
		delay_ms(200);
		SetLED(led,0);
	}
//#endif
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
	LED_RAW[group*3]=v0;
	LED_RAW[group*3+1]=v1;
	LED_RAW[group*3+2]=v2;
}
void SetLED(int32_t led, uint8_t v0){
	LED_RAW[led]=v0;
}

void resetLeds(void){
	//clear led bits
	for(uint32_t a=0; a<LEDS; a++){
		LED_RAW[a]=0;
	}
	calulateLEDMIBAMBits();
}
//LED_PRECALC[reg][bit] = (0x01<<bit) & LED_RAW[led];

void calulateLEDMIBAMBits(){
	uint32_t led,bitinreg;

	for(uint32_t bit=0; bit<BITS; bit++){
//	uint32_t bit=7;
		led=0;
		for(uint32_t reg=0; reg<REGS; reg++){
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
			uint32_t tt[16],l;
			for(uint32_t t=0;t<16;t++){
				l=LED_RAW[led]>>(bit);
//				xprintf(INFO "l=%d 1<<bitinreg=%15b" " (%s:%d)\n",l,1<<bitinreg,_F_,_L_);
				tt[t] = (l<<bitinreg)&(1<<bitinreg);
				bitinreg++;
				led++;
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
				tt[14];
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
