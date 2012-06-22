/*

*/

#include "debug_frmwrk.h"
#include "ShiftPWM.h"

#include "pinout.h"
#include "lpc17xx_pinsel.h"
#include "lpc17xx_gpio.h"
#include "lpc17xx_ssp.h"


void LED_init(){
	//TODO lsb first spi mode 0 0?

	GPIO_SetDir(LED_CS_PORT, LED_CS_BIT, 1);
	GPIO_SetValue(LED_CS_PORT, LED_CS_BIT);

	GPIO_SetDir(LED_LE_PORT, LED_LE_BIT, 1);
	GPIO_SetValue(LED_LE_PORT, LED_LE_BIT);

	GPIO_SetDir(WF_HIBERNATE_PORT, WF_HIBERNATE_BIT, 0);
	GPIO_SetValue(WF_HIBERNATE_PORT, WF_HIBERNATE_BIT);

	// Initialize SPI pin connect
	PINSEL_CFG_Type PinCfg;
	SSP_CFG_Type SSP_ConfigStruct;
	/* SCK1 */
	PinCfg.Funcnum   = PINSEL_FUNC_2;
	PinCfg.OpenDrain = PINSEL_PINMODE_NORMAL;
	PinCfg.Pinmode   = PINSEL_PINMODE_PULLDOWN;
	PinCfg.Pinnum    = LED_SCK_PIN;
	PinCfg.Portnum   = LED_SCK_PORT;
	PINSEL_ConfigPin(&PinCfg);
	/* MISO1 */
	PinCfg.Pinmode   = PINSEL_PINMODE_PULLUP;
	PinCfg.Pinnum    = LED_MISO_PIN;
	PinCfg.Portnum   = LED_MISO_PORT;
	PINSEL_ConfigPin(&PinCfg);
	/* MOSI1 */
	PinCfg.Pinnum    = LED_MOSI_PIN;
	PinCfg.Portnum   = LED_MOSI_PORT;
	PINSEL_ConfigPin(&PinCfg);

	/* initialize SSP configuration structure */
	SSP_ConfigStruct.CPHA = SSP_CPHA_FIRST;
	SSP_ConfigStruct.CPOL = SSP_CPOL_HI;
	SSP_ConfigStruct.ClockRate = 10000000; /* TLC5927 max freq = 30Mhz */
	SSP_ConfigStruct.Databit = SSP_DATABIT_16;
	SSP_ConfigStruct.Mode = SSP_MASTER_MODE;
	SSP_ConfigStruct.FrameFormat = SSP_FRAME_SPI;
	SSP_Init(LPC_SSP1, &SSP_ConfigStruct);

	/* Enable SSP peripheral */
	SSP_Cmd(LPC_SSP1, ENABLE);

}


void LED_test(){
	uint8_t ttt;
	uint16_t temp, send_data;
	_DBG("[INFO]-LED_test()");_DBG(" (");_DBG(__FILE__);_DBG(":");_DBD(__LINE__);_DBG(")\r\n");

	for(temp=0; temp<16; temp++){
		send_data = 1<<temp;
		_DBG("Sending: ");_DBH16(send_data);_DBG("\r\n");
		FIO_ClearValue(LED_LE_PORT, LED_LE_BIT);

		SSP_SendData(LPC_SSP1, send_data);
		while(SSP_GetStatus(LPC_SSP1,SSP_STAT_RXFIFO_FULL)){
			_DBG("[ERR]-LPC_SSP1 buffer full *press any key*");_DBG(" (");_DBG(__FILE__);_DBG(":");_DBD(__LINE__);_DBG(")\r\n");
			ttt = _DG;//wait for key press
			delay_ms(2000);
		}

		FIO_SetValue(WF_CS_PORT, WF_CS_BIT);
		delay_ms(100);
	}

}
