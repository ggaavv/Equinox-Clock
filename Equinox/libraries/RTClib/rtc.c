/*
 * rtc.c
 *
 *  Created on: 10 Jun 2012
 *      Author: gavin
 */

#include "debug_frmwrk.h"
#include "lpc17xx_rtc.h"
#include "rtc.h"

#define DSTEurope
#define DSTUSA
#define DS3231_I2C_ADDRESS 0x68
#define SECONDS_PER_DAY 86400L
#define SECONDS_FROM_1970_TO_2000 946684800

//For the United States:
//Begin DST: Sunday April (2+6*y-y/4) mod 7+1
//End DST: Sunday October (31-(y*5/4+1) mod 7)
//Valid for years 1900 to 2006, though DST wasn't adopted until the 1950s-1960s. 2007 and after:
//Begin DST: Sunday March 14 - (1 + y*5/4) mod 7
//End DST: Sunday November 7 - (1 + y*5/4) mod 7;
//
//European Economic Community:
//Begin DST: Sunday March (31 - (5*y/4 + 4) mod 7) at 1h U.T.
//End DST: Sunday October (31 - (5*y/4 + 1) mod 7) at 1h U.T.
//Since 1996, valid through 2099
//
//(Equations by Wei-Hwa Huang (US), and Robert H. van Gent (EC))

#ifdef DSTEurope
#define begin_DST_Month 3
#define begin_DST_Hour 1
#define begin_DST_Min 0
#define begin_DST_dayOfMonth(x) (31 - (5* x /4 + 4) % 7)
#define end_DST_Month 10
#define end_DST_Hour 1
#define end_DST_Min 0
#define end_DST_dayOfMonth(x) (31 - (5 * x /4 + 1) % 7)
#define DST_CORRECTION_VALUE 1
//#elif DSTUSA
#else
//TODO correct month/hour
#define begin_DST_Month=3;
#define begin_DST_Hour=1;
#define begin_DST_Min=0;
//#define begin_DST_dayOfMonth(x) ( ( 2 + 6 * x - x /4 ) % 7 + 1)
#define end_DST_Month=10;
#define end_DST_Hour=1;
#define end_DST_Min=0;
//#define end_DST_dayOfMonth(x) ( 31 - ( x * 5 /4 + 1 ) % 7)
#define DST_CORRECTION_VALUE 0
//#else
//#error echo "no dst set"
#endif
#define begin_DST_unix(x) Convert_To_Unixtime(x, begin_DST_Month, begin_DST_dayOfMonth(x), begin_DST_Hour, begin_DST_Min, 0)
#define end_DST_unix(x) Convert_To_Unixtime(x, end_DST_Month, end_DST_dayOfMonth(x), end_DST_Hour, end_DST_Min, 0)

//volitile
uint32_t DST_CORRECTION=0;
uint32_t LAST_DST_UPDATE_YEAR=0;

uint32_t SECOND_INC=0;

struct {
	uint32_t unix;
	uint16_t year;
	uint8_t month;
	uint8_t dom;
	uint8_t dow;
	uint8_t doy;
	uint8_t hh;
	uint8_t mm;
	uint8_t ss;
	uint32_t DST_begin_calculated;
	uint32_t DST_end_calculated;
	uint16_t dst_last_update_year;
} time;

struct {
	uint32_t unix;
	uint16_t year;
	uint8_t month;
	uint8_t dom;
	uint8_t dow;
	uint8_t doy;
	uint8_t hh;
	uint8_t mm;
	uint8_t ss;
} time_temp;

//uint32_t begin_DST_unix(uint32_t year) {
//	return Convert_To_Unixtime(year, begin_DST_Month, (31 - (5* year /4 + 4) % 7), begin_DST_Hour, begin_DST_Min, 0);
//}

//uint32_t end_DST_unix(uint32_t year) {
//	return Convert_To_Unixtime(year, end_DST_Month, (31 - (5 * year /4 + 1) % 7), end_DST_Hour, end_DST_Min, 0);
//}

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


		SECOND_INC=1;

			//Check for serial input
		//	checkSerial();

		//run daily checks at 00:00:00
		if(!RTC_GetTime(LPC_RTC, RTC_TIMETYPE_HOUR)&&!RTC_GetTime(LPC_RTC, RTC_TIMETYPE_MINUTE)&&!RTC_GetTime(LPC_RTC, RTC_TIMETYPE_SECOND))
			dailyCheck();

		//run  checks at xx:00:00
		if(!RTC_GetTime(LPC_RTC, RTC_TIMETYPE_MINUTE)&&!RTC_GetTime(LPC_RTC, RTC_TIMETYPE_SECOND))
			hourlyCheck();

		//run  checks at xx:xx:00
		if(!RTC_GetTime(LPC_RTC, RTC_TIMETYPE_SECOND))
			minutelyCheck();

		secondlyCheck();

		//custom alarms
	//	for(uint8_t alarmNo=0; alarms[alarmNo]; alarmNo++) {
	//
	//	}

	}

	/* Continue to check the Alarm match*/
	if (RTC_GetIntPending(LPC_RTC, RTC_INT_ALARM))
	{
		/* Send debug information */
		_DBG_ ("ALARM 10s matched!");

		// Clear pending interrupt
		RTC_ClearIntPending(LPC_RTC, RTC_INT_ALARM);
	}

//	RTC_print_time();
}

void RTC_time_Init(){

	// Init RTC module
	RTC_Init(LPC_RTC);
	/* Disable RTC interrupt */
    NVIC_DisableIRQ(RTC_IRQn);
    NVIC_SetPriority(RTC_IRQn, 8); // set according to main.c
	RTC_Cmd(LPC_RTC, ENABLE);
	RTC_CalibCounterCmd(LPC_RTC, DISABLE);

    //Set time if no data in GPREG
//    if (!(RTC_ReadGPREG(LPC_RTC, 4)==(0xaa)))
//    {
    	_DBG("[INFO]-Setting clock time");_DBG(" (");_DBG(__FILE__);_DBG(":");_DBD16(__LINE__);_DBG(")\r\n");
    	_DBG("[INFO]-__DATE__=");_DBG(__DATE__);_DBG(", __TIME__=");_DBG_(__TIME__);
		/* Enable rtc (starts increase the tick counter and second counter register) */
		RTC_ResetClockTickCounter(LPC_RTC);
		//				 yyyy  mm  dd  Dom Dow  ss  mm  hh
//		RTC_time_SetTime(2012,  6, 11, 1,  163, 10, 50, 20);
		RTC_set_default_time_to_compiled();
		RTC_WriteGPREG(LPC_RTC, 4, 0xaa);
//    }
		RTC_CntIncrIntConfig (LPC_RTC, RTC_TIMETYPE_SECOND, ENABLE);
//		RTC_CntIncrIntConfig (LPC_RTC, RTC_TIMETYPE_MINUTE, ENABLE);

    RTC_print_time();

    /* Enable RTC interrupt */
    NVIC_EnableIRQ(RTC_IRQn);

}

void dailyCheck(void) {
	//calculate sunrise/sunset at
//	mySun.Rise(time_now.month(),time_now.day());
//	mySun.Set(time_now.month(),time_now.day());
}

void hourlyCheck(void) {
}

void minutelyCheck(void) {
	//TODO update brightness

	//Checks and adjusts time for DST
	//Calculate DST's every year
	if(time.dst_last_update_year != time.year) {
		time.DST_begin_calculated = begin_DST_unix(time.year);
		time.DST_end_calculated = end_DST_unix(time.year);
		time.dst_last_update_year = time.year;
	}
	DST_check_and_correct();
}

void secondlyCheck(void) {
	uint16_t year;
	uint8_t month, dom, dow, doy, hour, min, sec;
	year = RTC_GetTime(LPC_RTC, RTC_TIMETYPE_YEAR);
	month = RTC_GetTime(LPC_RTC, RTC_TIMETYPE_MONTH);
	dom = RTC_GetTime(LPC_RTC, RTC_TIMETYPE_DAYOFMONTH);
	dow = RTC_GetTime(LPC_RTC, RTC_TIMETYPE_DAYOFWEEK);
	doy = RTC_GetTime(LPC_RTC, RTC_TIMETYPE_DAYOFYEAR);
	hour = RTC_GetTime(LPC_RTC, RTC_TIMETYPE_HOUR);
	min = RTC_GetTime(LPC_RTC, RTC_TIMETYPE_MINUTE);
	sec = RTC_GetTime(LPC_RTC, RTC_TIMETYPE_HOUR);

	time.year = year;
	time.month = month;
	time.dom = dom;
	time.dow = dow;
	time.doy = doy;

	time_temp.unix = RTC_time_GetUnixtime();
}

uint16_t GetY() {
	return RTC_GetTime(LPC_RTC, RTC_TIMETYPE_YEAR);
}

uint8_t GetM() {
	return RTC_GetTime(LPC_RTC, RTC_TIMETYPE_MONTH);
}

uint8_t GetDOM() {
	return RTC_GetTime(LPC_RTC, RTC_TIMETYPE_DAYOFMONTH);
}

uint8_t GetDOW() {
	return RTC_GetTime(LPC_RTC, RTC_TIMETYPE_DAYOFWEEK);
}

uint8_t GetDOY() {
	return RTC_GetTime(LPC_RTC, RTC_TIMETYPE_DAYOFYEAR);
}

uint8_t GetHH() {
	return time.hh;
}

uint8_t GetMM() {
	return time.mm;
}

uint8_t GetSS() {
	return time.ss;
}

void RTC_time_SetTime(uint16_t year, uint8_t month, uint8_t dayOfM, uint8_t hour, uint8_t min, uint8_t sec) {
	if (year!=NULL) 	RTC_SetTime (LPC_RTC, RTC_TIMETYPE_YEAR, year);
	if (month!=NULL)	RTC_SetTime (LPC_RTC, RTC_TIMETYPE_MONTH, month);
	if (dayOfM!=NULL)	RTC_SetTime (LPC_RTC, RTC_TIMETYPE_DAYOFMONTH, dayOfM);
	if ((year!=NULL) && (month!=NULL) && (dayOfM!=NULL))	RTC_SetTime (LPC_RTC, RTC_TIMETYPE_DAYOFWEEK, dayOfWeekManual(year, month, dayOfM)); //TODO: no idea why this wont compile
	if ((year!=NULL) && (month!=NULL) && (dayOfM!=NULL))	RTC_SetTime (LPC_RTC, RTC_TIMETYPE_DAYOFYEAR, dateoftheyear(year, month, dayOfM)); //TODO: no idea why this wont compile
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
	min = RTC_GetTime(LPC_RTC, RTC_TIMETYPE_MINUTE);
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

uint32_t Convert_To_Unixtime(uint16_t year, uint8_t month, uint8_t dayOfM, uint8_t hour, uint8_t min, uint8_t sec) {
  uint32_t t;
  uint16_t days = date2days(year, month, dayOfM);
  t = time2long(days, hour, min, sec);
  t += SECONDS_FROM_1970_TO_2000;  // seconds from 1970 to 2000
  return t;
}

void DST_check_and_correct() {
	if(time.DST_begin_calculated<=time.unix && time.unix<=time.DST_end_calculated) {

	}
}

void RTC_set_default_time_to_compiled(void) {

	uint8_t month_as_number;

	switch (__DATE__[0]) {
	 // case 'J': month_as_number = __DATE__[1] == 'a' ? 1 : month_as_number = __DATE__[2] == 'n' ? 6 : 7; break;
	    case 'J': month_as_number = __DATE__[1] == 'a' ? 1 : __DATE__[2] == 'n' ? 6 : 7; break;
	    case 'F': month_as_number = 2; break;
	    case 'A': month_as_number = __DATE__[2] == 'r' ? 4 : 8; break;
	    case 'M': month_as_number = __DATE__[2] == 'r' ? 3 : 5; break;
	    case 'S': month_as_number = 9; break;
	    case 'O': month_as_number = 10; break;
	    case 'N': month_as_number = 11; break;
	    case 'D': month_as_number = 12; break;
	}
//	_DBG("[INFO] - month_as_number=");_DBD(month_as_number);_DBG("\r\n");

	RTC_time_SetTime(
			conv2d(__DATE__ + 7)*100 + conv2d(__DATE__ + 9),
			month_as_number,
			conv2d(__DATE__ + 4),
			conv2d(__TIME__),
			conv2d(__TIME__ + 3),
			conv2d(__TIME__ + 6));

}

void RTC_print_time(void){
	_DBG("[INFO]-Date=");
	_DBD(GetDOM());
	_DBG("/");
	_DBD(GetM());
	_DBG("/");
	_DBD16(GetY());
	_DBG("\r\n");

	_DBG("[INFO]-Time=");
	_DBD(GetHH());
	_DBG(":");
	_DBD(GetMM());
	_DBG(":");
	_DBD(GetSS());
	_DBG("\r\n");
/*
	_DBG("[INFO]-Day Of week: ");_DBD(RTC_GetTime(LPC_RTC, RTC_TIMETYPE_DAYOFWEEK));_DBG("\r\n");
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
*/
}
