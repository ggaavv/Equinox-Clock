

#include "debug_frmwrk.h"
#include "pinout.h"
#include "lpc17xx_pinsel.h"
#include "lpc17xx_gpio.h"
#include "lpc17xx_ssp.h"
#include "g2100.h"


void WiFi_init(){

	GPIO_SetDir(WF_CS_PORT, WF_CS_PIN, 1);
	GPIO_SetValue(WF_CS_PORT, WF_CS_PIN);

	GPIO_SetDir(WF_RESET_PORT, WF_RESET_PIN, 1);
	GPIO_ClearValue(WF_RESET_PORT, WF_RESET_PIN);
	delay_ms(100);
	GPIO_SetValue(WF_RESET_PORT, WF_RESET_PIN);

	GPIO_SetDir(WF_HIBERNATE_PORT, WF_HIBERNATE_PIN, 1);
	GPIO_SetValue(WF_HIBERNATE_PORT, WF_HIBERNATE_PIN);

	// Initialize SPI pin connect
	PINSEL_CFG_Type PinCfg;
	SSP_CFG_Type SSP_ConfigStruct;
	/* SCK1 */
	PinCfg.Funcnum   = PINSEL_FUNC_2;
	PinCfg.OpenDrain = PINSEL_PINMODE_PULLUP;
	PinCfg.Pinmode   = PINSEL_PINMODE_TRISTATE;
	PinCfg.Pinnum    = WF_SCK_PIN;
	PinCfg.Portnum   = WF_SCK_PORT;
	PINSEL_ConfigPin(&PinCfg);
//	GPIO_SetDir(WF_SCK_PORT, WF_SCK_PIN, 1);
	/* MISO1 */
	PinCfg.Funcnum   = PINSEL_FUNC_2;
	PinCfg.OpenDrain = PINSEL_PINMODE_PULLUP;
	PinCfg.Pinmode   = PINSEL_PINMODE_TRISTATE;
	PinCfg.Pinnum    = WF_MISO_PIN;
	PinCfg.Portnum   = WF_MISO_PORT;
	PINSEL_ConfigPin(&PinCfg);
//	GPIO_SetDir(WF_MISO_PORT, WF_MISO_PIN, 0);
	/* MOSI1 */
	PinCfg.Funcnum   = PINSEL_FUNC_2;
	PinCfg.OpenDrain = PINSEL_PINMODE_PULLUP;
	PinCfg.Pinmode   = PINSEL_PINMODE_TRISTATE;
	PinCfg.Pinnum    = WF_MOSI_PIN;
	PinCfg.Portnum   = WF_MOSI_PORT;
	PINSEL_ConfigPin(&PinCfg);
//	GPIO_SetDir(WF_MOSI_PORT, WF_MOSI_PIN, 1);
	/* EINT0 */
	PinCfg.Funcnum   = PINSEL_FUNC_1;
	PinCfg.OpenDrain = PINSEL_PINMODE_NORMAL;
	PinCfg.Pinmode   = PINSEL_PINMODE_PULLUP;
	PinCfg.Pinnum    = WF_EINT2_PIN;//TODO: change to eint0 after debug
	PinCfg.Portnum   = WF_EINT2_PORT;//TODO: change to eint0 after debug
	PINSEL_ConfigPin(&PinCfg);
	GPIO_SetDir(WF_EINT2_PORT, WF_EINT2_PIN, 0);

	NVIC_EnableIRQ(EINT2_IRQn); //TODO: change to eint0 after debug

	/* initialize SSP configuration structure */
	SSP_ConfigStruct.CPHA = SSP_CPHA_FIRST;
	SSP_ConfigStruct.CPOL = SSP_CPOL_HI;
	SSP_ConfigStruct.ClockRate = 10000; /* 10Mhz WF max frequency = 25mhz*/
	SSP_ConfigStruct.Databit = SSP_DATABIT_8;
	SSP_ConfigStruct.Mode = SSP_MASTER_MODE;
	SSP_ConfigStruct.FrameFormat = SSP_FRAME_SPI;
	SSP_Init(LPC_SSP1, &SSP_ConfigStruct);

	/* Enable SSP peripheral */
	SSP_Cmd(LPC_SSP1, ENABLE);

//	attachInterrupt(INT_PIN, zg_isr, FALLING);

	zg_init();
	_DBG("[OK]-WiFi_init() - zg_init();");_DBG(__LINE__);_DBG_(__FILE__);

	while(zg_get_conn_state() != 1) {
		zg_drv_process();
	}
	_DBG("[OK]-WiFi_init() - while(zg_get_conn_state() != 1) {");_DBG(__LINE__);_DBG_(__FILE__);

	_DBG("[OK]-WiFi_init() - stack_init();");_DBG(__LINE__);_DBG_(__FILE__);
	stack_init();
}


void WiFi_loop(){
	_DBG("WiFi_loop()");_DBG(__LINE__);_DBG_(__FILE__);
	stack_process();
	zg_drv_process();
}


// This is the webpage that is served up by the webserver
const char webpage[] = {"HTTP/1.1 200 OK\r\nContent-Type: text/html\r\n\r\n<center><h1>Hello World!! I am WiShield</h1><form method=\"get\" action=\"0\">Toggle LED:<input type=\"submit\" name=\"0\" value=\"LED1\"></input></form></center>"};

