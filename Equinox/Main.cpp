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

#include "Wifi.h"
#include "rtc.h"
#include <stdio.h>
#include <stdint.h>
#include <stdarg.h>
#include <stdio.h>
#include <time.h>
#include <sys/time.h>

extern "C" {
	#include "g2100.h"
	#include "LPC17xx.h"
	#include "lpc17xx_gpio.h"
	#include "lpc17xx_exti.h"
	#include "lpc17xx_nvic.h"
	#include "lpc17xx_systick.h"
	#include "lpc17xx_rtc.h"
	#include "lpc17xx_wdt.h"
	#include "debug_frmwrk.h"
	#include "sys_timer.h"
	#include "serial.h"
	#include "ShiftPWM.h"
	#include "comm.h"
	#include "uart.h"
	#include "pinout.h"
	#include "syscalls.h"
	#include "hsv2rgb.h"
	#include "sys_timer.h"
	#include "canlib.h"
	#include "i2c.h"
	#include "tmp100.h"
}

#define USER_FLASH_START 0x3000 // For USB bootloader
//#define USER_FLASH_START 0x0 // No USB bootloader
#define BOOTLOADER_START 0x0 // To enter bootloader

/* External function prototypes ----------------------------------------------*/
extern "C" char* get_heap_end(void);
extern "C" char* get_stack_top(void);
extern volatile uint32_t LED_UPDATE_REQUIRED;
extern volatile uint32_t LED_SEND;

volatile uint32_t TOG[4] = {0,0,0,0};
extern volatile uint32_t USER_MILLIS;
extern volatile uint32_t SEC_MILLIS;

extern volatile int LINE_READY;
extern volatile uint8_t UART_LINE[50];
extern volatile uint32_t UART_LINE_LEN;

/*
void execute_bootloader(void){
   void (*user_code_entry)(void);

// Change the Vector Table to the USER_FLASH_START
// in case the user application uses interrupts

    volatile const uint32_t *stack_adr = 0x00000000;
    volatile const uint32_t *start_adr = 0x00000004;

    __set_PSP(*stack_adr);

    NVIC_SetVTOR(BOOTLOADER_START);

    user_code_entry = (void (*)(void))(*start_adr);
    user_code_entry();
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
	char buffer[100];
	uint32_t tto=2000;

#ifdef DEV
	GPIO_SetDir(LED_1_PORT, LED_1_BIT, 1);
	GPIO_SetValue(LED_1_PORT, LED_1_BIT);
	GPIO_ClearValue(LED_1_PORT, LED_1_BIT);

	GPIO_SetDir(LED_2_PORT, LED_2_BIT, 1);
	GPIO_SetValue(LED_2_PORT, LED_2_BIT);
	GPIO_ClearValue(LED_2_PORT, LED_2_BIT);

	GPIO_SetDir(LED_3_PORT, LED_3_BIT, 1);
	GPIO_SetValue(LED_3_PORT, LED_3_BIT);
	GPIO_ClearValue(LED_3_PORT, LED_3_BIT);

	GPIO_SetDir(LED_4_PORT, LED_4_BIT, 1);
	GPIO_SetValue(LED_4_PORT, LED_4_BIT);
	GPIO_ClearValue(LED_4_PORT, LED_4_BIT);
#endif

//	LPC_GPIO1->FIODIR = 1 << 23;
//	LPC_GPIO1->FIODIR = 1 << 21;
//	LPC_GPIO1->FIOPIN = 1 << 23; // make LED ON to indicate that button may be pressed to enter bootloader
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

	// Initialize the timer for millis()
	SYSTICK_InternalInit(1); // from NXP - 1ms interval
	SYSTICK_IntCmd(ENABLE);
	SYSTICK_Cmd(ENABLE);//xprintf(OK "SYSTICK_Cmd()");FFL_();

	//Debug functions output to com1/8n1/115200
//	debug_frmwrk_init();//_DBG("[OK]-debug_frmwrk_init()");_DBG(" (");_DBG(__FILE__);_DBG(":");_DBD16(__LINE__);_DBG(")\n");

	//Start Watchdog timer 30 sec
//	WDT_Init (WDT_CLKSRC_RTC, WDT_MODE_RESET);
//	WDT_Start(30000000);

	// Initialize USB<->Serial
	serial_init();xprintf(OK "serial_init()");FFL_();
//	uart_writestr("[OK]-uart_Start");
	serial_writestr("[OK]-usb serial_Start\n");

	// Initialize UART
	comm_init();
	comm_flush();
	usb_flush();
//	xprintf("\033[2J");//clear screen
	xprintf("\n\n\n\n" BOARD " Board\n**BOOTED**");FFL_();

#if 0
//	getc(); //working
//	getchar(); //working

//	fflush(stdin);
//	xprintf( "Press ANY key to continue..." "(%s:%d)\n",_F_,_L_);
	char n[80];
//	gets(n);
//	fgetc(NULL);

//	xprintf("\nscanf 2i" " (%s:%d)\n",_F_,_L_);
	scanf("%2i",&n);//works gets 10 chars
	xprintf("\nscanf n=%s" " (%s:%d)\n",n,_F_,_L_);
	
	n[0]=0;
//	xprintf("scanf 10[^\n]" " (%s:%d)\n",_F_,_L_);
	scanf("%10[^\n]",&n);//works
	xprintf("\nscanf n=%s" " (%s:%d)\n",n,_F_,_L_);
	
	n[0]=0;
//	xprintf("scanf [^\n]" " (%s:%d)\n",_F_,_L_);
//	scanf("%[^\n]",&n);//doesnt works - never exits
	xprintf("\nscanf n=%s" " (%s:%d)\n",n,_F_,_L_);
	
//	xprintf( "Pressed=%s\n", n);
//	get_line(n, 10);//sizeof(n));
//	comm_get();
//	char tmp[10];
//	get_line(tmp,4);
/*
	n[0]=0;
	uint32_t inumber,i2;
	char string[80];
	
	xprintf("fgets(string,100,NULL)" " (%s:%d)\n",_F_,_L_);
	fgets(string,100,NULL);//not working (no call to _read??)
	xprintf("scanf string=%s" " (%s:%d)\n",string,_F_,_L_);
	
	
	xprintf("sscanf" "(%s:%d)\n",_F_,_L_);
	sscanf(buffer,"%d %s",&inumber,string);//not working (no call to _read??)
	xprintf("sscanf inumber=%d string=%s" "(%s:%d)\n",inumber,string,_F_,_L_);

//	printf("inumber=%d  string=%s\n",inumber,string);
//	xprintf("scanf" "(%s:%d)\n",_F_,_L_);

	scanf("%d %s",&inumber,string);//gets only 1 char
	printf("inumber=%d  string=%s\n",inumber,string);
	int last=0xff,curr;
	while(0){
		curr = xavail();
		if(last!=curr)
			xprintf("curr=%b\n",curr);
		if(xgetc()=='f')
			xprintf("xgetc()=%b\n",xgetc());
		last = curr;
	}
*/
	xprintf("%b",xgetc());//works
#endif

	// Init RTC module
    RTC_time_Init();xprintf(OK "RTC_time_Init()");FFL_();

#ifdef DEV
	// CAN init
	CAN_init();xprintf(OK "CAN_init()");FFL_();
#endif

	//I2C init
	I2C_init();xprintf(OK "I2C_init()");FFL_();

#ifndef DEV
	// Wifi init
	WiFi_init();xprintf(OK "WiFi_init()");FFL_();

    // Init LED module
    LED_init();xprintf(OK "LED_init()");FFL_();
    LED_test();xprintf(OK "LED_test()");FFL_();
#endif

#if 0
    time_t rawtime;
    struct tm * timeinfo;
    time( &rawtime );
    timeinfo = localtime ( &rawtime );
    xprintf( "The current date/time is: %s", asctime (timeinfo) );

//    rawtime = time (NULL);
//    printf ("%ld hours since January 1, 1970", asctime (time (NULL)));///3600);

#endif


	// main loop
	long timer1, steptimeout, count1, tcount=Getunix(), colorshift=0;
	int hue, sat, val;
	unsigned char red, green, blue;
	for (;;){
		//Reset Watchdog timer (30sec)
//		WDT_UpdateTimeOut(30);

#ifndef DEV
		// Wifi Loop
		WiFi_loop();
#endif

#ifdef DEV
		//CAN Loop
		CAN_loop();
#endif

//		RTC_print_time();
//		LED_loop();

		/* Power save - Do every 5000ms */
		#define DELAY 20
//		_DBD16(tcount);
#if 1
		if(tcount<=Getunix()){
			tcount=DELAY+Getunix();
//			xprintf(INFO "for (;;) %d",Getunix());FFL_();
#ifndef DEV
			tmp100_gettemp();
#endif
			time_t rawtime;
			struct tm * timeinfo;
			time( &rawtime );
			timeinfo = localtime ( &rawtime );
			strftime(buffer,80,"Now it's %I:%M:%S%p.",timeinfo);
			xprintf( "%s", buffer);FFL_();
//			xprintf( "date/time is: %s", asctime (timeinfo));FFL_();
//			RTC_print_time();
		}
#endif
#if 1
		if(LINE_READY){
			exec_cmd(UART_LINE);
			LINE_READY=0;
		}
#endif
#ifndef DEV
		#define LEDDELAY 10
		if(USER_MILLIS>=10){
//		if(LED_UPDATE_REQUIRED){
			LED_UPDATE_REQUIRED=0;
			USER_MILLIS=0;
			colorshift+=1;
			if(colorshift==360)
				colorshift=0;
//			for(int cycle=0;cycle<numCycles;cycle++){ // shift the raibom numCycles times
//				for(int led=0;led<RGBS;led++){ // loop over all LED's
				for(int led=0;led<1;led++){ // loop over all LED's
					hue = ((led*1)*360/1+colorshift)%360; // Set hue from 0 to 360 from first to last led and shift the hue
					sat = 255;
					val = 255;
//					hsv2rgb(hue, sat, val, &red, &green, &blue, maxBrightness); // convert hsv to rgb values
					hsv2rgb(hue, sat, val, &red, &green, &blue, 0x7f); // convert hsv to rgb values
					SetRGB(led, red, green, blue); // write rgb values
				}
				calulateLEDMIBAMBits();
//			}
		}
#endif
#if 0
		if(timeUpdate()) {
			resetLeds();
	//	colourToRGBled(time_now.hour12()*5,RED1,RED2,RED3,false,0);
	//	colourToRGBled(time_now.minute(),GREEN1,GREEN2,GREEN3,false,0);
	//	colourToRGBled(time_now.second(),BLUE1,BLUE2,BLUE3,false,0);

	/* good
			SetRGB(time_now.second()-4,0,0,1);
			SetRGB(time_now.second()-3,0,0,2);
			SetRGB(time_now.second()-2,0,0,4);
			SetRGB(time_now.second()-1,0,0,100);
			SetRGB(time_now.second()-0,0,0,255);

			SetRGB(time_now.minute()-2,3,0,5);
			SetRGB(time_now.minute()-1,10,0,25);
			SetRGB(time_now.minute()-0,20,0,180);

			SetRGB(time_now.hour12()*5+(time_now.minute()/12),85,0,130);
	*/

			SetRGB(time_now.second()-0,0,0,1);
	//		SetRGB(time_now.second()-3,0,0,2);
	//		SetRGB(time_now.second()-2,0,0,4);
	//		SetRGB(time_now.second()-1,0,0,100);
	//		SetRGB(time_now.second()-0,0,0,255);

			SetRGB(time_now.minute()-1,1,0,1);
			SetRGB(time_now.minute()-0,1,0,1);
	//		SetRGB(time_now.minute()-0,20,0,180);

			SetRGB(time_now.hour12()*5+(time_now.minute()/12),1,1,1);


	//		delay(100);
		}
#endif

/*
		if (timer1 < sys_millis()){
			timer1 = sys_millis() + 100;
			count1++;
//			xprintf(INFO "count=",count1);FFL_();


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

