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
//#include "eq_clock.h"
//#include "ShiftPWM.h"
#include "wifi.h"
#include "g2100.h"
//	void stack_init(void);
//	void stack_process(void);

#include "uart.h"   // TODO: remove after debugging
#include "lpc17xx_gpio.h"
#include "debug.h"

#define USER_FLASH_START 0x3000 /* For USB bootloader */
//#define USER_FLASH_START 0x0 /* No USB bootloader */

void startup_delay(void){
	for (volatile unsigned long i = 0; i < 50000; i++) { ; }
}

/*********************************************************************//**
 * @brief	Main sub-routine
 **********************************************************************/
int main(void){

	LPC_GPIO1->FIODIR = 1 << 23;
	LPC_GPIO1->FIODIR = 1 << 21;
	LPC_GPIO1->FIOPIN = 1 << 23; // make LED ON to indicate that button may be pressed to enter bootloader
//	LPC_GPIO1->FIOPIN ^= 1 << 23; //  LED flasher
//	LPC_GPIO1->FIOCLR = 1 << 23;

	// DeInit NVIC and SCBNVIC
	NVIC_DeInit();
	NVIC_SCBDeInit();
	/* Configure the NVIC Preemption Priority Bits */
	// b100 bxxx.yy000    Group priority bits:[7:5]    Subpriority bits:[4:3]    Group priorities:8   Subpriorities:4 = 4
	// assign LED to group 0 sub-priority 0 = 0
	// assign USB to group 1 sub-priority 0 = 4
	// assign Wifi to group 2 sub-priority 0 = 8
	// assign RTC to group 2 sub-priority 0 = 8
	NVIC_SetPriorityGrouping(4);

	/* Change the Vector Table to the USER_FLASH_START
	in case the user application uses interrupts */
	SCB->VTOR = (USER_FLASH_START & 0x1FFFFF80);

	//Debug functions output to com1/8n1/115200
	//does this need to be first??
	//TODO
	debug_frmwrk_init();_DBG_("\r\n\r\n\r\n\r\n\r\n**BOOTED**");_DBG("[OK]-debug_frmwrk_init()");_DBG(" (");_DBG(__FILE__);_DBG(":");_DBD32(__LINE__);_DBG(")\r\n");

	//eraseScreen
//	_DBG(0x1B);//_DBG(ESCAPE);
//	_DBG('[');//_DBG(BRACE);
//	_DBG('1');
//	_DBG('J');

	// Initialize the timer for millis()
	SYSTICK_InternalInit(1); // from NXP - 1ms interval
	SYSTICK_IntCmd(ENABLE);
	SYSTICK_Cmd(ENABLE);_DBG("[OK]-SYSTICK_Cmd()");_DBG(" (");_DBG(__FILE__);_DBG(":");_DBD32(__LINE__);_DBG(")\r\n");

	// Initialize USB<->Serial
	serial_init();_DBG("[OK]-serial_init()");_DBG(" (");_DBG(__FILE__);_DBG(":");_DBD32(__LINE__);_DBG(")\r\n");

	uart_writestr("[OK]-uart_Start");
	serial_writestr("[OK]-serial_Start");
	_DBG(" (");_DBG(__FILE__);_DBG(":");_DBD32(__LINE__);_DBG(")\r\n");

	// Init RTC module
    RTC_time_Init();_DBG("[OK]-RTC_time_Init()");_DBG(" (");_DBG(__FILE__);_DBG(":");_DBD32(__LINE__);_DBG(")\r\n");

    LED_init();_DBG("[OK]-LED_init()");_DBG(" (");_DBG(__FILE__);_DBG(":");_DBD32(__LINE__);_DBG(")\r\n");
    LED_test();_DBG("[OK]-LED_test()");_DBG(" (");_DBG(__FILE__);_DBG(":");_DBD32(__LINE__);_DBG(")\r\n");


	// Wifi init
	WiFi_init();_DBG("[OK]-WiFi_init()");_DBG(" (");_DBG(__FILE__);_DBG(":");_DBD32(__LINE__);_DBG(")\r\n");

	// main loop
	long timer1, steptimeout, discard;
	for (;;){
		// Wifi Loop
		WiFi_loop();
//		LED_loop();

		/* Power save - Do every 100ms */
		#define DELAY1 100


		if (timer1 < sys_millis())
		{
			timer1 = sys_millis() + DELAY1;

			/* If there are no activity during 30 seconds, power off the machine */
			if (steptimeout > (30 * 1000/DELAY1))
				{
//					power_off();
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

//delay_ms(2000);

