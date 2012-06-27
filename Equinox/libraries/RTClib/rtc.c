/*
 * rtc.c
 *
 *  Created on: 10 Jun 2012
 *      Author: gavin
 */

#include "debug_frmwrk.h"
#include "lpc17xx_rtc.h"

#define DSTEurope 1
#define SECONDS_PER_DAY 86400L
#define SECONDS_FROM_1970_TO_2000 946684800
static uint8_t daysInMonth [12] = { 31,28,31,30,31,30,31,31,30,31,30,31 };

#define DOW_LEN 3
#define NUM_DAYS_OF_WEEK 7
#define DAYS_OF_WEEK_MAX_LEN 10
static unsigned char DayOfWeekName[NUM_DAYS_OF_WEEK][DAYS_OF_WEEK_MAX_LEN] = {
"Monday",
"Tuesday",
"Wednesday",
"Thursday",
"Friday",
"Saturday",
"Sunday"
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

void RTC_time_SetTime(uint16_t year, uint8_t month, uint8_t dayOfM, uint8_t hour, uint8_t min, uint8_t sec) {
	if (year!=NULL) 	RTC_SetTime (LPC_RTC, RTC_TIMETYPE_YEAR, year);
	if (month!=NULL)	RTC_SetTime (LPC_RTC, RTC_TIMETYPE_MONTH, month);
	if (dayOfM!=NULL)	RTC_SetTime (LPC_RTC, RTC_TIMETYPE_DAYOFMONTH, dayOfM);
//	if ((year!=NULL) && (month!=NULL) && (dayOfM!=NULL))	RTC_SetTime (LPC_RTC, RTC_TIMETYPE_DAYOFWEEK, dayOfWeekManual(year, month, dayOfM)); //TODO: no idea why this wont compile
//	if ((year!=NULL) && (month!=NULL) && (dayOfM!=NULL))	RTC_SetTime (LPC_RTC, RTC_TIMETYPE_DAYOFYEAR, dateoftheyear(year, month, dayOfM)); //TODO: no idea why this wont compile
	if (sec!=NULL)		RTC_SetTime (LPC_RTC, RTC_TIMETYPE_SECOND, sec);
	if (min!=NULL)		RTC_SetTime (LPC_RTC, RTC_TIMETYPE_MINUTE, min);
	if (hour!=NULL)		RTC_SetTime (LPC_RTC, RTC_TIMETYPE_HOUR, hour);
}

void RTC_time_GetTime(uint16_t* year, uint8_t* month, uint8_t* dayOfM, uint8_t* dayOfW, uint8_t* dayOfY, uint8_t* hour, uint8_t* min, uint8_t* sec) {
	year = RTC_GetTime(LPC_RTC, RTC_TIMETYPE_YEAR);
	month = RTC_GetTime(LPC_RTC, RTC_TIMETYPE_MONTH);
	dayOfM = RTC_GetTime(LPC_RTC, RTC_TIMETYPE_DAYOFMONTH);
	dayOfW = RTC_GetTime(LPC_RTC, RTC_TIMETYPE_DAYOFWEEK);
	dayOfY = RTC_GetTime(LPC_RTC, RTC_TIMETYPE_DAYOFYEAR);
	hour = RTC_GetTime(LPC_RTC, RTC_TIMETYPE_HOUR);
	min = RTC_GetTime(LPC_RTC, RTC_TIMETYPE_SECOND);
	sec = RTC_GetTime(LPC_RTC, RTC_TIMETYPE_HOUR);
}

// number of days since 2000/01/01, valid for 2001..2099
uint16_t date2days(uint16_t year, uint8_t month, uint8_t dayOfM) {
    if (year >= 2000)
    	year -= 2000;
    uint16_t days = dayOfM;
    for (uint8_t i = 1; i < month; ++i)
        days += daysInMonth[i] - 1;
    if (month > 2 && year % 4 == 0)
        ++days;
    return days + (year + 3) / 4 - 1;  //TODO: check if this is correct!!!
}

// number of days since 2000/01/01, valid for 2001..2099
uint16_t dateoftheyear(uint16_t year, uint8_t month, uint8_t dayOfM) {
    if (year >= 2000)
    	year -= 2000;
    uint16_t days = dayOfM;
    for (uint8_t i = 1; i < month; ++i)
        days += daysInMonth[i] - 1;
    if (month > 2 && year % 4 == 0)
        ++days;
    return days + 365 * year + (year + 3) / 4 - 1;
}

// convert date into seconds
long time2long(uint16_t days, uint8_t hour, uint8_t min, uint8_t sec) {
    return ((days * 24L + hour) * 60 + min) * 60 + sec;
}

// find day of the week
uint8_t dayOfWeekManual(uint16_t year, uint8_t month, uint8_t dayOfM) {
    uint16_t day = date2days(year, month, dayOfM);
    return (day + 6) % 7; // Jan 1, 2000 is a Saturday, i.e. returns 6
}

// convert char to uint8_t for time
uint8_t conv2d(const char* p) {
    uint8_t v = 0;
    if ('0' <= *p && *p <= '9')
        v = *p - '0';
    return 10 * v + *++p - '0';
}

uint32_t RTC_time_GetUnixtime() {
  uint32_t t;
  uint16_t days = date2days(RTC_GetTime (LPC_RTC, RTC_TIMETYPE_DAYOFMONTH), RTC_GetTime (LPC_RTC, RTC_TIMETYPE_MONTH), RTC_GetTime (LPC_RTC, RTC_TIMETYPE_DAYOFMONTH));
  t = time2long(days, RTC_GetTime (LPC_RTC, RTC_TIMETYPE_HOUR), RTC_GetTime (LPC_RTC, RTC_TIMETYPE_MINUTE), RTC_GetTime (LPC_RTC, RTC_TIMETYPE_SECOND));
  t += SECONDS_FROM_1970_TO_2000; // seconds from 1970 to 2000
  return t;
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
    			break;
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
        	break;
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
        	break;
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
    RTC_SetTime (LPC_RTC, RTC_TIMETYPE_YEAR, conv2d(__DATE__ + 7)*100 + conv2d(__DATE__ + 9));
	RTC_SetTime (LPC_RTC, RTC_TIMETYPE_MONTH, month_as_number);
	RTC_SetTime (LPC_RTC, RTC_TIMETYPE_DAYOFMONTH, conv2d(__DATE__ + 4));
	RTC_SetTime (LPC_RTC, RTC_TIMETYPE_HOUR, conv2d(__TIME__));
	RTC_SetTime (LPC_RTC, RTC_TIMETYPE_MINUTE, conv2d(__TIME__ + 3));
	RTC_SetTime (LPC_RTC, RTC_TIMETYPE_SECOND, conv2d(__TIME__ + 6));
}

void RTC_print_time(void){
	_DBG("[INFO]-Day Of x: ");
//	_DBG("Week: ");_DBD(RTC_GetTime(LPC_RTC, RTC_TIMETYPE_DAYOFWEEK));


	_DBG("\r\n");

	_DBG("[INFO]-Date=");
	_DBD(RTC_GetTime(LPC_RTC, RTC_TIMETYPE_DAYOFMONTH));
	_DBG("/");
	_DBD(RTC_GetTime(LPC_RTC, RTC_TIMETYPE_MONTH));
	_DBG("/");
	_DBD16(RTC_GetTime(LPC_RTC, RTC_TIMETYPE_YEAR));
	_DBG("\r\n");

	_DBG("[INFO]-Time=");
	_DBD(RTC_GetTime(LPC_RTC, RTC_TIMETYPE_HOUR));
	_DBG(":");
	_DBD(RTC_GetTime(LPC_RTC, RTC_TIMETYPE_MINUTE));
	_DBG(":");
	_DBD(RTC_GetTime(LPC_RTC, RTC_TIMETYPE_SECOND));
	_DBG("\r\n");
}
