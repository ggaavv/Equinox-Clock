/* Copyright (c) 2011 Jorge Pinto - casainho@gmail.com       */
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

#include "debug_frmwrk.h"
#include "r2c2.h"
#include "pinout.h"
#include "ext_interrupts.h"
#include "g2100.h"
	void stack_init(void);
	void stack_process(void);

#define USER_FLASH_START 0x30000 /* For USB bootloader */
//#define USER_FLASH_START 0x0 /* No USB bootloader */

extern int app_main (void);

void startup_delay(void){
	for (volatile unsigned long i = 0; i < 5000; i++) { ; }
}

void WiFi_init(){

	pin_mode(WF_CS_PORT, WF_CS_PIN, OUTPUT);
	digital_write(WF_CS_PORT, WF_CS_PIN, HIGH);

	pin_mode(WF_RESET_PORT, WF_RESET_PIN, OUTPUT);
	digital_write(WF_RESET_PORT, WF_RESET_PIN, HIGH);

	pin_mode(WF_HIBERNATE_PORT, WF_HIBERNATE_PIN, INPUT);
	digital_write(WF_HIBERNATE_PORT, WF_HIBERNATE_PIN, HIGH);

	// Initialize SPI pin connect
	PINSEL_CFG_Type PinCfg;
	SSP_CFG_Type SSP_ConfigStruct;
	/* SCK1 */
	PinCfg.Funcnum   = PINSEL_FUNC_2;
	PinCfg.OpenDrain = PINSEL_PINMODE_NORMAL;
	PinCfg.Pinmode   = PINSEL_PINMODE_PULLDOWN;
	PinCfg.Pinnum    = WF_SCK_PIN;
	PinCfg.Portnum   = WF_SCK_PORT;
	PINSEL_ConfigPin(&PinCfg);
	/* MISO1 */
	PinCfg.Pinmode   = PINSEL_PINMODE_PULLUP;
	PinCfg.Pinnum    = WF_MISO_PIN;
	PinCfg.Portnum   = WF_MISO_PORT;
	PINSEL_ConfigPin(&PinCfg);
	/* MOSI1 */
	PinCfg.Pinnum    = WF_MOSI_PIN;
	PinCfg.Portnum   = WF_MOSI_PORT;
	PINSEL_ConfigPin(&PinCfg);

	/* initialize SSP configuration structure */
	SSP_ConfigStruct.CPHA = SSP_CPHA_FIRST;
	SSP_ConfigStruct.CPOL = SSP_CPOL_HI;
	SSP_ConfigStruct.ClockRate = 10000000; /* 10Mhz */
	SSP_ConfigStruct.Databit = SSP_DATABIT_8;
	SSP_ConfigStruct.Mode = SSP_MASTER_MODE;
	SSP_ConfigStruct.FrameFormat = SSP_FRAME_SPI;
	SSP_Init(LPC_SSP1, &SSP_ConfigStruct);

	/* Enable SSP peripheral */
	SSP_Cmd(LPC_SSP1, ENABLE);

	attachInterrupt(INT_PIN, zg_isr, FALLING);

	zg_init();


	while(zg_get_conn_state() != 1) {
		zg_drv_process();
	}

	stack_init();
}




// This is the webpage that is served up by the webserver
const prog_char webpage[] PROGMEM = {"HTTP/1.1 200 OK\r\nContent-Type: text/html\r\n\r\n<center><h1>Hello World!! I am WiShield</h1><form method=\"get\" action=\"0\">Toggle LED:<input type=\"submit\" name=\"0\" value=\"LED1\"></input></form></center>"};

// Web Server config
#define WIRELESS_MODE_INFRA	1
#define WIRELESS_MODE_ADHOC	2

// Wireless configuration parameters ----------------------------------------
unsigned char local_ip[] = {192,168,1,2};	// IP address of WiShield
unsigned char gateway_ip[] = {192,168,1,1};	// router or gateway IP address
unsigned char subnet_mask[] = {255,255,255,0};	// subnet mask for the local network
const prog_char ssid[] PROGMEM = {"ASYNCLABS"};		// max 32 bytes

unsigned char security_type = 0;	// 0 - open; 1 - WEP; 2 - WPA; 3 - WPA2

// WPA/WPA2 passphrase
const prog_char security_passphrase[] PROGMEM = {"12345678"};	// max 64 characters

// WEP 128-bit keys
// sample HEX keys
prog_uchar wep_keys[] PROGMEM = {	0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08, 0x09, 0x0a, 0x0b, 0x0c, 0x0d,	// Key 0
									0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,	0x00,	// Key 1
									0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,	0x00,	// Key 2
									0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,	0x00	// Key 3
								};

// setup the wireless mode
// infrastructure - connect to AP
// adhoc - connect to another WiFi device
unsigned char wireless_mode = WIRELESS_MODE_INFRA;
unsigned char ssid_len;
unsigned char security_passphrase_len;

/*********************************************************************//**
 * @brief	Main sub-routine
 **********************************************************************/
int main(void){

	long timer1, steptimeout, discard;

	//Debug functions output to com1/8n1/115200
	//does this need to be first??
	//TODO
	debug_frmwrk_init();
	_DBG_("[OK]-debug_frmwrk_init()");//_DBG(__LINE__);_DBG(__FILE__);
	//discard=_DBG_("**press any key**");_DG();//wait for key press @ debug port.

	// DeInit NVIC and SCBNVIC
	NVIC_DeInit();
	NVIC_SCBDeInit();

	/* Configure the NVIC Preemption Priority Bits:
	* two (2) bits of preemption priority, six (6) bits of sub-priority.
	* Since the Number of Bits used for Priority Levels is five (5), so the
	* actual bit number of sub-priority is three (3)
	*/
	NVIC_SetPriorityGrouping(0x05);

	/* Change the Vector Table to the USER_FLASH_START
	in case the user application uses interrupts */
	SCB->VTOR = (USER_FLASH_START & 0x1FFFFF80);

	// Initialize USB<->Serial
	serial_init();
	_DBG_("[OK]-serial_init()");//_DBG(__LINE__);_DBG(__FILE__);
	serial_writestr("Start\r\nOK\r\n");
  
	SysTickTimer_Init(); // Initialize the timer for millis()

	// wifi init
	WiFi_init();
	_DBG("[OK]-WiFi_init()");//_DBG(__LINE__);_DBG(__FILE__);



	// main loop
	for (;;){
		stack_process();
		zg_drv_process();

		/* Power save - Do every 100ms */
		#define DELAY1 100
		if (timer1 < millis())
		{
			timer1 = millis() + DELAY1;

			/* If there are no activity during 30 seconds, power off the machine */
			if (steptimeout > (30 * 1000/DELAY1))
				{
					power_off();
				}
			else
				{
					steptimeout++;
				}
		}
	}

	/* should never get here */
	while(1) ;
}

