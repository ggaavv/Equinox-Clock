/*
 * rtc.c
 *
 *  Created on: 10 Jun 2012
 *      Author: gavin
 */

#include "debug_frmwrk.h"
#include "lpc17xx_rtc.h"

#define DOW_LEN 3
#define NUM_DAYS_OF_WEEK 7
#define DAYS_OF_WEEK_MAX_LEN 10
static unsigned char DayOfWeekName[NUM_DAYS_OF_WEEK][DAYS_OF_WEEK_MAX_LEN] = {
"Sunday",
"Monday",
"Tuesday",
"Wednesday",
"Thursday",
"Friday",
"Saturday"
};

#define MONTH_LEN 3
#define NUM_MONTHS 12
#define MONTH_MAX_LEN 10
static unsigned char Month_of_the_year[NUM_MONTHS][MONTH_MAX_LEN] = {
"January",
"February",
"March",
"April",
"May",
"June",
"July",
"August",
"September",
"October",
"November",
"December"
};

void RTC_IRQHandler(void){
	uint32_t secval;

	/* This is increment counter interrupt*/
	if (RTC_GetIntPending(LPC_RTC, RTC_INT_COUNTER_INCREASE))
	{
		secval = RTC_GetTime (LPC_RTC, RTC_TIMETYPE_SECOND);

		/* Send debug information */
//		_DBG ("Second: "); _DBD(secval);
//		_DBG_("");

		// Clear pending interrupt
		RTC_ClearIntPending(LPC_RTC, RTC_INT_COUNTER_INCREASE);

	}

	/* Continue to check the Alarm match*/
	if (RTC_GetIntPending(LPC_RTC, RTC_INT_ALARM))
	{
		/* Send debug information */
		_DBG_ ("ALARM 10s matched!");

		// Clear pending interrupt
		RTC_ClearIntPending(LPC_RTC, RTC_INT_ALARM);
	}

	RTC_print_time();
}


void RTC_time_Init(){

	// Init RTC module
	RTC_Init(LPC_RTC);
	/* Disable RTC interrupt */
    NVIC_DisableIRQ(RTC_IRQn);
    NVIC_SetPriority(RTC_IRQn, 8); // set according to main.c
	RTC_Cmd(LPC_RTC, ENABLE);
	RTC_CalibCounterCmd(LPC_RTC, DISABLE);

    // Set bit 32 of General purpose register 4 to one after configuring time
    // If no 1 set to default time
//	RTC_WriteGPREG(LPC_RTC, 4, 0xaa);
//    uart_send_32_Hex(RTC_ReadGPREG(LPC_RTC, 4));
    if (!(RTC_ReadGPREG(LPC_RTC, 4)==(0xaa)))
    {
    	_DBG("[INFO]-Setting clock time");_DBG(" (");_DBG(__FILE__);_DBG(":");_DBD16(__LINE__);_DBG(")\r\n");
    	_DBG("[INFO]-__DATE__=");_DBG(__DATE__);_DBG(", __TIME__=");_DBG_(__TIME__);
		/* Enable rtc (starts increase the tick counter and second counter register) */
		RTC_ResetClockTickCounter(LPC_RTC);
		//				 yyyy  mm  dd  Dom Dow  ss  mm  hh
//		RTC_time_SetTime(2012,  6, 11, 1,  163, 10, 50, 20);
		RTC_set_default_time_to_compiled();
//		RTC_WriteGPREG(LPC_RTC, 4, 0xaa);
	        // following line sets the RTC to the date & time this sketch was compiled
//	        RTC.adjust(DateTime(__DATE__, __TIME__)); //TODO: get this to work
    }
	RTC_CntIncrIntConfig (LPC_RTC, RTC_TIMETYPE_SECOND, ENABLE);

    RTC_print_time();

    /* Enable RTC interrupt */
    NVIC_EnableIRQ(RTC_IRQn);

}

void RTC_time_SetTime(uint32_t year, uint32_t month, uint32_t dayOfM, uint32_t dayOfW, uint32_t dayOfY, uint32_t hour, uint32_t min, uint32_t sec) {

	if (year!=NULL) 	RTC_SetTime (LPC_RTC, RTC_TIMETYPE_YEAR, year);
	if (month!=NULL)	RTC_SetTime (LPC_RTC, RTC_TIMETYPE_MONTH, month);
	if (dayOfM!=NULL)	RTC_SetTime (LPC_RTC, RTC_TIMETYPE_DAYOFMONTH, dayOfM);
	if (dayOfW!=NULL)	RTC_SetTime (LPC_RTC, RTC_TIMETYPE_DAYOFWEEK, dayOfW);
	if (dayOfY!=NULL)	RTC_SetTime (LPC_RTC, RTC_TIMETYPE_DAYOFYEAR, dayOfY);
	if (sec!=NULL)		RTC_SetTime (LPC_RTC, RTC_TIMETYPE_SECOND, sec);
	if (min!=NULL)		RTC_SetTime (LPC_RTC, RTC_TIMETYPE_MINUTE, min);
	if (hour!=NULL)		RTC_SetTime (LPC_RTC, RTC_TIMETYPE_HOUR, hour);

}

void RTC_time_GetTime(uint32_t year, uint32_t month, uint32_t dayOfM, uint32_t dayOfW, uint32_t dayOfY, uint32_t hour, uint32_t min, uint32_t sec) {

}

static uint8_t conv2d(const char* p) {
    uint8_t v = 0;
    if ('0' <= *p && *p <= '9')
        v = *p - '0';
    return 10 * v + *++p - '0';
}

void RTC_set_default_time_to_compiled(void) {

	uint8_t month_as_number;

    switch (__DATE__[0]) {
    	case 'J':
    		switch (__DATE__[1]) {
    			case 'a':
    				month_as_number = 1;
    				break;
    			default:
    				switch (__DATE__[2]) {
    					case 'n':
    						month_as_number = 6;
    						break;
    					case 'l':
    						month_as_number = 7;
    						break;
    				}
    		}
    		break;
        case 'F':
        	month_as_number = 2;
        	break;
//        case 'A': month_as_number = __DATE__[2] == 'r' ? 4 : 8; break;
        case 'A':
        	switch (__DATE__[2]) {
        		case 'r':
        			month_as_number = 4;
        			break;
        		case 'g':
        			month_as_number = 8;
        			break;
        	}
//        case 'M': month_as_number = __DATE__[2] == 'r' ? 3 : 5; break;
        case 'M':
        	switch (__DATE__[2]) {
    			case 'r':
    				month_as_number = 3;
    				break;
    			case 'y':
    				month_as_number = 5;
    				break;
        	}
        case 'S':
        	month_as_number = 9;
        	break;
        case 'O':
        	month_as_number = 10;
        	break;
        case 'N':
        	month_as_number = 11;
        	break;
        case 'D':
        	month_as_number = 12;
        	break;
    }

    _DBG("[INFO]-__DATE__[0]=");_DBD(__DATE__[0]);_DBG(" (");_DBG(__FILE__);_DBG(":");_DBD16(__LINE__);_DBG(")\r\n");
    _DBG("[INFO]-conv2d(__DATE__[0]=");_DBD(conv2d(__DATE__[0]));_DBG(" (");_DBG(__FILE__);_DBG(":");_DBD16(__LINE__);_DBG(")\r\n");
    _DBG("[INFO]-conv2d(__DATE__[0] + 9)=");_DBD(conv2d(__DATE__[0] + 9));_DBG(" (");_DBG(__FILE__);_DBG(":");_DBD16(__LINE__);_DBG(")\r\n");
    _DBG("[INFO]-month_as_number");_DBD(month_as_number);_DBG(" (");_DBG(__FILE__);_DBG(":");_DBD16(__LINE__);_DBG(")\r\n");
    RTC_SetTime (LPC_RTC, RTC_TIMETYPE_YEAR, conv2d(__DATE__ + 9));
	RTC_SetTime (LPC_RTC, RTC_TIMETYPE_MONTH, month_as_number);
	RTC_SetTime (LPC_RTC, RTC_TIMETYPE_DAYOFMONTH, conv2d(__DATE__ + 4));
	RTC_SetTime (LPC_RTC, RTC_TIMETYPE_HOUR, conv2d(__TIME__));
	RTC_SetTime (LPC_RTC, RTC_TIMETYPE_MINUTE, conv2d(__TIME__ + 3));
	RTC_SetTime (LPC_RTC, RTC_TIMETYPE_SECOND, conv2d(__TIME__ + 6));

}

void RTC_print_time(void){
	/*
//	uart_writestr("\r\n");
	uart_writestr(RTC_GetTime(LPC_RTC, RTC_TIMETYPE_DAYOFMONTH));
	uart_writestr("/");
	uart_writestr(Month_of_the_year[RTC_GetTime(LPC_RTC, RTC_TIMETYPE_DAYOFMONTH)]);
	uart_writestr("/");
	uart_writestr(RTC_GetTime(LPC_RTC, RTC_TIMETYPE_YEAR));

	uart_writestr("\r\n");
	uart_writestr(RTC_GetTime(LPC_RTC, RTC_TIMETYPE_HOUR));
	uart_writestr(":");
	uart_writestr(RTC_GetTime(LPC_RTC, RTC_TIMETYPE_MINUTE));
	uart_writestr(":");
	uart_writestr(RTC_GetTime(LPC_RTC, RTC_TIMETYPE_SECOND));
	uart_writestr("\r\n");
*/

	_DBG("[INFO]-Date=");
	_DBD(RTC_GetTime(LPC_RTC, RTC_TIMETYPE_DAYOFMONTH));
	_DBG("/");
	_DBD(RTC_GetTime(LPC_RTC, RTC_TIMETYPE_MONTH));
	_DBG("/");
	_DBD(RTC_GetTime(LPC_RTC, RTC_TIMETYPE_YEAR));
	_DBG("\r\n");

	_DBG("[INFO]-Time=");
	_DBD(RTC_GetTime(LPC_RTC, RTC_TIMETYPE_HOUR));
	_DBG(":");
	_DBD(RTC_GetTime(LPC_RTC, RTC_TIMETYPE_MINUTE));
	_DBG(":");
	_DBD(RTC_GetTime(LPC_RTC, RTC_TIMETYPE_SECOND));
	_DBG("\r\n");
}
