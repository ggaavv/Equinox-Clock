/*

*/

#include "debug_frmwrk.h"
#include "ShiftPWM.h"

#include "pinout.h"
#include "lpc17xx_pinsel.h"
#include "lpc17xx_gpio.h"
#include "lpc17xx_ssp.h"
#include "lpc17xx_rit.h"

#define MAX_BAM_BITS 16

volatile uint32_t SENDSEQ=0;
volatile uint32_t DELAY_TIME=1; //so RIT ms is not set to 0
volatile uint32_t NEXT_DELAY_TIME=1; //so RIT ms is not set to 0
volatile uint32_t SEND_BIT=0;
volatile uint32_t NEXT_SEND_BIT=0;

const uint32_t BITORDER[] = { 0,1,2,3,4,5,6,7,7,6,5,4,3,2,1,0 };
const uint32_t BITTIME[] = {
		1, //Bit 0 time (LSB)
		2, //Bit 1 time
		4, //Bit 2 time
		8, //Bit 3 time
		16, //Bit 4 time
		32, //Bit 5 time
		64, //Bit 6 time
		128 //Bit 7 time (MSB)
};
uint32_t SEQ_BIT[16];
uint32_t SEQ_TIME[16];



void RIT_IRQHandler(void)
{
	RIT_GetIntStatus(LPC_RIT); //call this to clear interrupt flag

	//Set bit/time
	DELAY_TIME=NEXT_DELAY_TIME;
	SEND_BIT=NEXT_SEND_BIT;

	//Retart sequence if required
	SENDSEQ+=1;
	if (SENDSEQ>MAX_BAM_BITS)
		SENDSEQ=0;

	//Setup new timing for next RIT
	NEXT_DELAY_TIME=SEQ_TIME[SENDSEQ];
	NEXT_SEND_BIT=SEQ_BIT[SENDSEQ];

	RIT_TimerConfig(LPC_RIT,DELAY_TIME);

//	send_bits();
}


void LED_init(){

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

	//TODO lsb first spi mode 0 0?

	GPIO_SetDir(LED_OE_PORT, LED_OE_BIT, 1);
	GPIO_SetValue(LED_OE_PORT, LED_OE_BIT);

	GPIO_SetDir(LED_LE_PORT, LED_LE_BIT, 1);
	GPIO_SetValue(LED_LE_PORT, LED_LE_BIT);

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
	/* MOSI1 */
	PinCfg.Funcnum   = PINSEL_FUNC_2;
	PinCfg.OpenDrain = PINSEL_PINMODE_NORMAL;
	PinCfg.Pinmode   = PINSEL_PINMODE_PULLUP;
	PinCfg.Pinnum    = LED_MOSI_PIN;
	PinCfg.Portnum   = LED_MOSI_PORT;
	PINSEL_ConfigPin(&PinCfg);

	/* initialize SSP configuration structure */
	SSP_CFG_Type SSP_ConfigStruct;
	SSP_ConfigStruct.CPHA = SSP_CPHA_FIRST;
	SSP_ConfigStruct.CPOL = SSP_CPOL_LO;
	SSP_ConfigStruct.ClockRate = 30000000; /* TLC5927 max freq = 30Mhz */
	SSP_ConfigStruct.Databit = SSP_DATABIT_16;
	SSP_ConfigStruct.Mode = SSP_MASTER_MODE;
	SSP_ConfigStruct.FrameFormat = SSP_FRAME_SPI;
	SSP_Init(LPC_SSP1, &SSP_ConfigStruct);//TODO: change to LPC_SSP0 after debug

	/* Enable SSP peripheral */
	SSP_Cmd(LPC_SSP1, ENABLE);//TODO: change to LPC_SSP0 after debug

	RIT_Init(LPC_RIT);
	/* Configure time_interval for RIT
	 * In this case: time_interval = 1000 ms = 1s
	 * So, RIT will generate interrupt each 1s
	 */
	RIT_TimerConfig(LPC_RIT,DELAY_TIME);

//	_DBG("The time interval is: ");
//	_DBD32(DELAY_TIME); _DBG_(" millisecond..");

	NVIC_EnableIRQ(RIT_IRQn);

}


void LED_test(){
	uint16_t t, temp, send_data;
	_DBG("[INFO]-LED_test()");_DBG(" (");_DBG(__FILE__);_DBG(":");_DBD16(__LINE__);_DBG(")\r\n");

	FIO_ClearValue(LED_OE_PORT, LED_OE_BIT);//LED's on.

	_DBG("[INFO]-Sending: ");
//	while(1){
	for(temp=0; temp<3; temp++){
		send_data = 1<<temp;
		_DBH16(send_data);_DBG(", ");

		SSP_SendData(LPC_SSP1, send_data);
		while(!SSP_GetStatus(LPC_SSP1,SSP_STAT_BUSY));//Wait if TX buffer full
//		delay_ms(1);

		FIO_SetValue(LED_LE_PORT, LED_LE_BIT);
		FIO_ClearValue(LED_LE_PORT, LED_LE_BIT);
		if(1)
			delay_ms(100);
		else {
			_DBG("[INPUT]*press any key*");_DBG(" (");_DBG(__FILE__);_DBG(":");_DBD16(__LINE__);_DBG(")\r\n");
			t = _DG;//wait for key press
		}
	}
//	}
	_DBG("\r\n");

}
