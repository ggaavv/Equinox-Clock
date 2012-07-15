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

#include "Wifi.h"
#include "rtc.h"
#include <stdio.h>
#include <stdint.h>
#include <stdarg.h>

extern "C" {
	#include "g2100.h"
	#include "LPC17xx.h"
	#include "lpc17xx_gpio.h"
	#include "lpc17xx_exti.h"
	#include "lpc17xx_nvic.h"
	#include "lpc17xx_systick.h"
	#include "lpc17xx_rtc.h"
	#include "debug_frmwrk.h"
	#include "sys_timer.h"
	#include "serial.h"
	#include "ShiftPWM.h"
	#include "comm.h"
#include "term_io.h"
#include "syscalls.h"
#include <time.h>
}

#define USER_FLASH_START 0x3000 // For USB bootloader
//#define USER_FLASH_START 0x0 // No USB bootloader
#define BOOTLOADER_START 0x0 // To enter bootloader

/* External function prototypes ----------------------------------------------*/
extern "C" char* get_heap_end(void);
extern "C" char* get_stack_top(void);

/*
void execute_bootloader(void){
 //  void (*user_code_entry)(void);

// Change the Vector Table to the USER_FLASH_START
// in case the user application uses interrupts

//    volatile const uint32_t *stack_adr = 0x00000000;
//    volatile const uint32_t *start_adr = 0x00000004;

//    __set_PSP(*stack_adr);

//    NVIC_SetVTOR(BOOTLOADER_START);

//    user_code_entry = (void (*)(void))(*start_adr);
//    user_code_entry();
}

void EINT0_IRQHandler (void)
{
	// Clear interrupt flag
	EXTI_ClearEXTIFlag(EXTI_EINT0);
	// Disable interrupts
	NVIC_DeInit();
	delay_ms(100);

	execute_bootloader();
}
*/
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
	// assign RTC to group 2 sub-priority 0 = 8
	// assign Wifi to group 2 sub-priority 1 = 9
	NVIC_SetPriorityGrouping(4);

	/* Change the Vector Table to the USER_FLASH_START
	in case the user application uses interrupts */
	SCB->VTOR = (USER_FLASH_START & 0x1FFFFF80);


	//Debug functions output to com1/8n1/115200
	//does this need to be first??
	//TODO
	debug_frmwrk_init();_DBG_("\r\n\r\n\r\n\r\n\r\n**BOOTED**");_DBG("[OK]-debug_frmwrk_init()");_DBG(" (");_DBG(__FILE__);_DBG(":");_DBD16(__LINE__);_DBG(")\r\n");

	//eraseScreen
//	_DBG(0x1B);//_DBG(ESCAPE);
//	_DBG('[');//_DBG(BRACE);
//	_DBG('1');
//	_DBG('J');

	// Initialize the timer for millis()
	SYSTICK_InternalInit(1); // from NXP - 1ms interval
	SYSTICK_IntCmd(ENABLE);
	SYSTICK_Cmd(ENABLE);_DBG("[OK]-SYSTICK_Cmd()");_DBG(" (");_DBG(__FILE__);_DBG(":");_DBD16(__LINE__);_DBG(")\r\n");


    comm_init();
    xprintf("xprintf now works\nFile=" __FILE__ "\n");
    xprintf("xprintf now works\nLine= %b \n",__LINE__);
    delay_ms(1000);
 //   xprintf("Hello from a C++ demo by Martin Thomas\nVersion \n");
    printf("\r\nprintf now works, line= %d\r\n", __LINE__);
    delay_ms(1000);

	// Initialize USB<->Serial
	serial_init();_DBG("[OK]-serial_init()");_DBG(" (");_DBG(__FILE__);_DBG(":");_DBD16(__LINE__);_DBG(")\r\n");

//	uart_writestr("[OK]-uart_Start");_DBG(" (");_DBG(__FILE__);_DBG(":");_DBD16(__LINE__);_DBG(")\r\n");
//	serial_writestr("[OK]-serial_Start");_DBG(" (");_DBG(__FILE__);_DBG(":");_DBD16(__LINE__);_DBG(")\r\n");

	// Init RTC module
    RTC_time_Init();_DBG("[OK]-RTC_time_Init()");_DBG(" (");_DBG(__FILE__);_DBG(":");_DBD16(__LINE__);_DBG(")\r\n");

    LED_init();_DBG("[OK]-LED_init()");_DBG(" (");_DBG(__FILE__);_DBG(":");_DBD16(__LINE__);_DBG(")\r\n");
//    LED_test();_DBG("[OK]-LED_test()");_DBG(" (");_DBG(__FILE__);_DBG(":");_DBD16(__LINE__);_DBG(")\r\n");

//    time_t rawtime;
 //   	  struct tm * timeinfo;

//    	  time ( &rawtime );
 //   	  timeinfo = localtime ( &rawtime );
  //  	  xprintf ( "The current date/time is: %s", asctime (timeinfo) );

	// Wifi init
//	WiFi_init();_DBG("[OK]-WiFi_init()");_DBG(" (");_DBG(__FILE__);_DBG(":");_DBD16(__LINE__);_DBG(")\r\n");

	// main loop
	long timer1, steptimeout, count1, tcount=Getunix();
	for (;;){
		// Wifi Loop
//		WiFi_loop();
//		delay_ms(1000);
//		RTC_print_time();
//		LED_loop();

		/* Power save - Do every 5000ms */
		#define DELAY 10
//		_DBD16(tcount);_DBG(" (");_DBG(__FILE__);_DBG(":");_DBD16(__LINE__);_DBG(")\r\n");
		if(tcount<=Getunix()){
//		if(RTC_GetTime(LPC_RTC, RTC_TIMETYPE_SECOND)==0){
			tcount=DELAY+Getunix();
//			_DBD16(tcount);_DBG(" (");_DBG(__FILE__);_DBG(":");_DBD16(__LINE__);_DBG(")\r\n");
			_DBG("[INFO]-for (;;) ");_DBD32(Getunix());_DBG(" (");_DBG(__FILE__);_DBG(":");_DBD16(__LINE__);_DBG(")\r\n");
//			RTC_print_time();
		}
/*
		if (timer1 < sys_millis()){
			timer1 = sys_millis() + 100;
			count1++;
//			_DBD16(count1);_DBG(" (");_DBG(__FILE__);_DBG(":");_DBD16(__LINE__);_DBG(")\r\n");


			// If there are no activity during 30 seconds, power off the machine
			if (steptimeout > (30*1000/100))
				{
//					power_off();
				}
			else
				{
					steptimeout=0;
				}
		}
*/
	}

	/* should never get here */
	while(1) ;
}

