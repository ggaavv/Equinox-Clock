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

//	while(1) ;

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

	// Initialize the timer for millis()
	SYSTICK_InternalInit(1); // from NXP not R2C2 - 1ms interval
	SYSTICK_IntCmd(ENABLE);
	SYSTICK_Cmd(ENABLE);
	
	long timer1, steptimeout, discard;

	//Debug functions output to com1/8n1/115200
	//does this need to be first??
	//TODO
	debug_frmwrk_init();
	_DBG("[OK]-debug_frmwrk_init()");_DBD(__LINE__);_DBG_(__FILE__);
	//discard=_DBG_("**press any key**");_DG();//wait for key press @ debug port.

	_DBG("[OK]-SYSTICK_Cmd()");_DBD(__LINE__);_DBG_(__FILE__);

	// Initialize USB<->Serial
	serial_init();
	_DBG("[OK]-serial_init()");_DBD(__LINE__);_DBG_(__FILE__);

	uart_writestr("Start\r\nOK\r\n");
	serial_writestr("Start\r\nOK\r\n");

	// Init RTC module
	RTC_Init(LPC_RTC);
	_DBG("[OK]-serial_init()");_DBD(__LINE__);_DBG_(__FILE__);

	//TODO check if RTC IRQ needs disabling
	/* Disable RTC interrupt */
    NVIC_DisableIRQ(RTC_IRQn);
    /* preemption = 1, sub-priority = 1 */
    NVIC_SetPriority(RTC_IRQn, ((0x01<<3)|0x01));
    //TODO check if rtc is running
/*    if (!RTC.isrunning()) {
    	_DBG("[ERR]-!RTC.isrunning()");_DBD(__LINE__);_DBG_(__FILE__);
        // following line sets the RTC to the date & time this sketch was compiled
        RTC.adjust(DateTime(__DATE__, __TIME__));
    }
*/

	//Setup SSP port for led drivers
	//LED_init();
	_DBG_("[OK]-LED_init()");_DBD(__LINE__);_DBG_(__FILE__);

	// wifi init
	WiFi_init();
	_DBG("[OK]-WiFi_init()");_DBD(__LINE__);_DBG_(__FILE__);


	// main loop
	_DBG("[INFO]-WiFi_init()");_DBD(__LINE__);_DBG_(__FILE__);
	// main loop
	for (;;){
		// Wifi Loop
		//TODO change to interrupt
		WiFi_loop();

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

