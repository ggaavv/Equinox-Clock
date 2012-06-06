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
//#include "wifi.h"
//#include "g2100.h"
//	void stack_init(void);
//	void stack_process(void);

#define USER_FLASH_START 0x30000 /* For USB bootloader */
//#define USER_FLASH_START 0x0 /* No USB bootloader */

void startup_delay(void){
	for (volatile unsigned long i = 0; i < 5000; i++) { ; }
}

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
//	serial_init();
	_DBG_("[OK]-serial_init()");//_DBG(__LINE__);_DBG(__FILE__);
//	serial_writestr("Start\r\nOK\r\n");
  
//	SysTickTimer_Init(); // Initialize the timer for millis()
	SYSTICK_InternalInit(); // Initialize the timer for millis(), from NXP not R2C2

	//Setup SSP port for led drivers
	//LED_init();
	//_DBG("[OK]-LED_init()");//_DBG(__LINE__);_DBG(__FILE__);

	// wifi init
//	WiFi_init();
	_DBG("[OK]-WiFi_init()");//_DBG(__LINE__);_DBG(__FILE__);


	// main loop
	_DBG("[INFO]-WiFi_init()");//_DBG(__LINE__);_DBG(__FILE__);
	// main loop
	for (;;){
//		stack_process();
//		zg_drv_process();

		/* Power save - Do every 100ms */
		#define DELAY1 100
		if (timer1 < SYSTICK_GetCurrentValue())
		{
			timer1 = SYSTICK_GetCurrentValue() + DELAY1;

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

