/*
 * rtc.c
 *
 *  Created on: 10 Jun 2012
 *      Author: gavin
 */

#include "debug_frmwrk.h"
#include "lpc17xx_rtc.h"
#include "rtc.h"
#include "sunrise.h"
#include <stdio.h>

#define DSTEurope
//#define DSTUSA

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
#define DST_CORRECTION_VALUE_SEC 60*60
#endif
#ifdef DSTUSA
//TODO correct month/hour
#define begin_DST_Month=3;
#define begin_DST_Hour=1;
#define begin_DST_Min=0;
//#define begin_DST_dayOfMonth(x) ( ( 2 + 6 * x - x /4 ) % 7 + 1)
#define end_DST_Month=10;
#define end_DST_Hour=1;
#define end_DST_Min=0;
//#define end_DST_dayOfMonth(x) ( 31 - ( x * 5 /4 + 1 ) % 7)
#define DST_CORRECTION_VALUE_SEC 0
//#else
//#error echo "no dst set"
#endif
#define begin_DST_unix(x) RTC_time_FindUnixtime(x, begin_DST_Month, begin_DST_dayOfMonth(x), begin_DST_Hour, begin_DST_Min, 0)
#define end_DST_unix(x) RTC_time_FindUnixtime(x, end_DST_Month, end_DST_dayOfMonth(x), end_DST_Hour, end_DST_Min, 0)

//volitile
uint32_t DST_CORRECTION=0;
uint32_t LAST_DST_UPDATE_YEAR=0;

uint32_t SECOND_INC=0;

void RTC_IRQHandler(void){
	uint32_t secval;
	// This is increment counter interrupt
	if (RTC_GetIntPending(LPC_RTC, RTC_INT_COUNTER_INCREASE)){
		secval = RTC_GetTime (LPC_RTC, RTC_TIMETYPE_SECOND);
		// Clear pending interrupt
		RTC_ClearIntPending(LPC_RTC, RTC_INT_COUNTER_INCREASE);
		time.second_inc=1;
		//run  checks at xx:xx:00
		if(!RTC_GetTime(LPC_RTC, RTC_TIMETYPE_SECOND)){
			//run  checks at xx:00:00
			if(!RTC_GetTime(LPC_RTC, RTC_TIMETYPE_MINUTE)){
				//run daily checks at 00:00:00
				if(!RTC_GetTime(LPC_RTC, RTC_TIMETYPE_HOUR)){
					//run weekly checks at Mon 00:00:00
					if(1 == RTC_GetTime(LPC_RTC, RTC_TIMETYPE_DAYOFWEEK)){
						//run weekly checks at 1st Mon 00:00:00
						if(1 == RTC_GetTime(LPC_RTC, RTC_TIMETYPE_DAYOFMONTH)){
							//run weekly checks at Jan 1st Mon 00:00:00
							if(1 == RTC_GetTime(LPC_RTC, RTC_TIMETYPE_MONTH)){
								yearlyCheck();
							}
							monthlyCheck();
						}
						weeklyCheck();
					}
					dailyCheck();
				}
				hourlyCheck();
			}
			minutelyCheck();
		}
		secondlyCheck();
	}

	// Continue to check the Alarm match
	if (RTC_GetIntPending(LPC_RTC, RTC_INT_ALARM)){
		//custom alarms
	//	for(uint8_t alarmNo=0; alarms[alarmNo]; alarmNo++) {
	//
	//	}
		/* Send debug information */
		_DBG_ ("ALARM 10s matched!");
		// Clear pending interrupt
		RTC_ClearIntPending(LPC_RTC, RTC_INT_ALARM);
	}
}

void RTC_time_Init(){
	// Init RTC module
	RTC_Init(LPC_RTC);
	// Disable RTC interrupt
    NVIC_DisableIRQ(RTC_IRQn);
    NVIC_SetPriority(RTC_IRQn, 8); // set according to main.c
	RTC_Cmd(LPC_RTC, ENABLE);
	update_time();
//	RTC_CalibCounterCmd(LPC_RTC, DISABLE);
//	RTC_WriteGPREG(LPC_RTC, 4, 0x55);
    //Set time if no data in GPREG
    if (!(RTC_ReadGPREG(LPC_RTC, 4)==(0xaa)))
    {
    	_DBG("[INFO]-Set time");_DBG(" (");_DBG(__FILE__);_DBG(":");_DBD16(__LINE__);_DBG(" (");_DBG(__FILE__);_DBG(":");_DBD16(__LINE__);_DBG(")\r\n");
    	_DBG("[INFO]-__DATE__=");_DBG(__DATE__);_DBG(", __TIME__=");_DBG(__TIME__);_DBG(" (");_DBG(__FILE__);_DBG(":");_DBD16(__LINE__);_DBG(")\r\n");
		// Enable rtc (starts increase the tick counter and second counter register)
		RTC_ResetClockTickCounter(LPC_RTC);
		//				 yyyy  mm  dd  Dom Dow  ss  mm  hh  st
//		RTC_time_SetTime(2012,  6, 11, 1,  163, 10, 50, 20, 00);
		RTC_set_default_time_to_compiled();
		RTC_WriteGPREG(LPC_RTC, 4, 0xaa);
    }
	yearlyCheck();
	weeklyCheck();
	dailyCheck();
	hourlyCheck();
	minutelyCheck();
	secondlyCheck();
	RTC_print_time();
    // Enable 1 sec interrupt
	RTC_CntIncrIntConfig (LPC_RTC, RTC_TIMETYPE_SECOND, ENABLE);
    // Enable RTC interrupt
    NVIC_EnableIRQ(RTC_IRQn);
}

void secondlyCheck(void) {
//	RTC_print_time();
	//Checks and adjusts time for DST
	update_time();
	DST_check_and_correct();
}

void hourlyCheck(void) {
}

void minutelyCheck(void) {
	time.day_night = NIGHT;
	if ((time.sunrise_unix < time.unix) && (time.sunset_unix < time.unix)){
		if ((time.no_set_rise != RISE_ONLY) || (time.no_set_rise != SET_ONLY) || (time.no_set_rise != ALL_DAY) || (time.no_set_rise != ALL_NIGHT)){
			time.day_night = DAY;
		} else{
			// TODO but not important
		}
		//TODO update brightness
	}
}

void dailyCheck(void) {
	//calculate sunrise/sunset for the day
	if ((0 < Sunrise_Compute(time.month, time.dom, READ_SUNRISE)) && (0 < Sunrise_Compute(time.month, time.dom, READ_SUNSET))){
//		_DBG("[INFO]-dailyCheck()");_DBG(" (");_DBG(__FILE__);_DBG(":");_DBD16(__LINE__);_DBG(")\r\n");
		uint32_t unix_day = RTC_time_FindUnixtime(time.year, time.month, time.dom, 0, 0, 0);
		time.sunrise_unix = unix_day + (60 * Sunrise_Compute(time.month, time.dom, READ_SUNRISE));
		time.sunset_unix = unix_day + (60 * Sunrise_Compute(time.month, time.dom, READ_SUNSET));
		time.noon_unix = unix_day + (60 * Sunrise_Compute(time.month, time.dom, READ_NOON));
		if(dst_correction_needed()){
			time.sunrise_unix_utc = time.sunrise_unix - DST_CORRECTION_VALUE_SEC;
			time.sunset_unix_utc = time.sunset_unix - DST_CORRECTION_VALUE_SEC;
			time.noon_unix_utc = time.noon_unix - DST_CORRECTION_VALUE_SEC;
		}else{
			time.sunrise_unix_utc = time.sunrise_unix;
			time.sunset_unix_utc = time.sunset_unix;
			time.noon_unix_utc = time.noon_unix;
		}
	} else {
		//TODO but not important - Calculation for if the is no sunrise/set/both
/*		if ((0 > Sunrise_Compute(time.month, time.dom, READ_SUNRISE)) && (0 > Sunrise_Compute(time.month, time.dom, READ_SUNSET))){

		} else {
			if (0 > Sunrise_Compute(time.month, time.dom, READ_SUNRISE)){
				time.sunrise_unix = time.unix + (60 * Sunrise_Compute(time.month, time.dom, READ_SUNRISE));
				time.sunset_unix = time.unix + (24 * 60 * 60);
				time.no_set_rise = RISE_ONLY;
			}
			if (0 > Sunrise_Compute(time.month, time.dom, READ_SUNSET)){
				time.sunset_unix = time.unix + (60 * Sunrise_Compute(time.month, time.dom, READ_SUNSET));
				time.sunrise_unix = time.unix + (24 * 60 * 60);
				time.no_set_rise = SET_ONLY;
			}
		}*/
	}
}

void weeklyCheck(void){
}

void monthlyCheck(void){
}

void yearlyCheck(void){
	//Calculate DST's every year
	DSTyearly();
}

uint32_t GetUNIX() {
	return time.unix;
}

uint16_t GetY() {
	return time.year;
}

uint8_t GetM() {
	return time.month;
}

uint8_t GetDOM() {
	return time.dom;
}

uint8_t GetDOW() {
	return time.dow;
}

uint8_t GetDOY() {
	return time.doy;
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

int8_t GetDST_correction() {
	return time.dst_correction;
}

void RTC_time_SetTime(uint16_t year, uint8_t month, uint8_t dom, uint8_t hh, uint8_t mm, uint8_t ss, int8_t st) {
	uint32_t unixt = RTC_time_FindUnixtime(year, month, dom, hh, mm, ss);
	time.year = year;
	time.month = month;
	time.dom = dom;
	time.dow = dayOfWeekManual(year, month, dom);
	time.doy = days_from_20xx(year, month, dom);
	time.hh = hh;
	time.mm = mm;
	time.ss = ss;
	time.unix = unixt;

	yearlyCheck();
	if(dst_correction_needed()) {
		//correct for dst active
		_DBG("[INFO]-(set)dst_correction_needed()");_DBG(" (");_DBG(__FILE__);_DBG(":");_DBD16(__LINE__);_DBG(")\r\n");
		time.unix_utc = unixt - DST_CORRECTION_VALUE_SEC;
		unix_to_hh_mm_ss(time.unix_utc, &time.hh_utc, &time.mm_utc, &time.ss_utc);
		time.unix_dst_last_update = time.unix;
	}else{
		time.unix_utc = time.unix;
		time.hh_utc = time.hh;
		time.mm_utc = time.mm;
		time.ss_utc = time.ss;
	}
	RTC_SetTime (LPC_RTC, RTC_TIMETYPE_YEAR, year);
	RTC_SetTime (LPC_RTC, RTC_TIMETYPE_MONTH, month);
	RTC_SetTime (LPC_RTC, RTC_TIMETYPE_DAYOFMONTH, dom);
	RTC_SetTime (LPC_RTC, RTC_TIMETYPE_DAYOFWEEK, time.dow);
	RTC_SetTime (LPC_RTC, RTC_TIMETYPE_DAYOFYEAR, time.doy);
	RTC_SetTime (LPC_RTC, RTC_TIMETYPE_SECOND, time.ss);
	RTC_SetTime (LPC_RTC, RTC_TIMETYPE_MINUTE, time.mm);
	RTC_SetTime (LPC_RTC, RTC_TIMETYPE_HOUR, time.hh);

	weeklyCheck();
	dailyCheck();
	hourlyCheck();
	minutelyCheck();
	secondlyCheck();
//    RTC_print_time();
}

// number of days since 20xx/01/01, valid for 2001..2099
uint16_t days_from_20xx(uint16_t year, uint8_t month, uint8_t dayOfM) {
    if (year >= 2000)
    	year -= 2000;
    uint16_t days = dayOfM;
    for (uint8_t i = 1; i < month; ++i)
        days += daysInMonth[i-1];
    if (month > 2 && year % 4 == 0)
        ++days;
    return days + (year + 3) / 4 - 1;  //TODO: check if this is correct!!!
}

// number of days since 2000/01/01, valid for 2001..2099
uint16_t days_from_2000(uint16_t y, uint8_t m, uint8_t d) {
    if (y >= 2000)
        y -= 2000;
    uint16_t days = d;
    for (uint8_t i = 1; i < m; i++)
    	days += daysInMonth[i-1];
    if (m > 2 && y % 4 == 0)
        days++;
    return days + 365 * y + (y + 3) / 4 - 1;
}

// convert date into seconds
long time2long(uint16_t days, uint8_t hour, uint8_t min, uint8_t sec) {
    return ((days * 24L + hour) * 60 + min) * 60 + sec;
}

// find day of the week
uint8_t dayOfWeekManual(uint16_t year, uint8_t month, uint8_t dayOfM) {
    uint16_t day = days_from_2000(year, month, dayOfM);
    return (day + 5) % 7; // Jan 1, 2000 is a Saturday, i.e. returns 5   Week Begins on a Monday = 0
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
  uint16_t days = days_from_2000(RTC_GetTime (LPC_RTC, RTC_TIMETYPE_YEAR), RTC_GetTime (LPC_RTC, RTC_TIMETYPE_MONTH), RTC_GetTime (LPC_RTC, RTC_TIMETYPE_DAYOFMONTH));
  t = time2long(days, RTC_GetTime (LPC_RTC, RTC_TIMETYPE_HOUR), RTC_GetTime (LPC_RTC, RTC_TIMETYPE_MINUTE), RTC_GetTime (LPC_RTC, RTC_TIMETYPE_SECOND));
  t += SECONDS_FROM_1970_TO_2000; // seconds from 1970 to 2000
  return t;
}

uint32_t RTC_time_FindUnixtime(uint16_t year, uint8_t month, uint8_t dayOfM, uint8_t hour, uint8_t min, uint8_t sec) {
  uint16_t days = days_from_2000(year, month, dayOfM);
  uint32_t t = time2long(days, hour, min, sec);
  t += SECONDS_FROM_1970_TO_2000; // seconds from 1970 to 2000
  return t;
}

void update_time(){
	time.year = RTC_GetTime(LPC_RTC, RTC_TIMETYPE_YEAR);
	time.month = RTC_GetTime(LPC_RTC, RTC_TIMETYPE_MONTH);
	time.dom = RTC_GetTime(LPC_RTC, RTC_TIMETYPE_DAYOFMONTH);
	time.dow = RTC_GetTime(LPC_RTC, RTC_TIMETYPE_DAYOFWEEK);
	time.doy = RTC_GetTime(LPC_RTC, RTC_TIMETYPE_DAYOFYEAR);
	time.hh = RTC_GetTime(LPC_RTC, RTC_TIMETYPE_HOUR);
	time.mm = RTC_GetTime(LPC_RTC, RTC_TIMETYPE_MINUTE);
	time.ss = RTC_GetTime(LPC_RTC, RTC_TIMETYPE_SECOND);
}

void DST_check_and_correct() {
	time.unix = RTC_time_FindUnixtime(time.year, time.month, time.dom, time.hh, time.mm, time.ss);
//	_DBG("[INFO]-time.hh=");_DBD(time.hh);_DBG(" (");_DBG(__FILE__);_DBG(":");_DBD16(__LINE__);_DBG(")\r\n");
	// Only update time once for DST when it ticks over
	if(dst_correction_needed()) {
		_DBG("dst_correction_needed");_DBG(" (");_DBG(__FILE__);_DBG(":");_DBD16(__LINE__);_DBG(")\r\n");
		if(time.DST_begin_calculated == time.unix){
			time.unix_dst_last_update = time.unix;
			time.unix += DST_CORRECTION_VALUE_SEC;
			unix_to_hh_mm_ss(time.unix, &time.hh, &time.mm, &time.ss);
			RTC_SetTime (LPC_RTC, RTC_TIMETYPE_SECOND, time.ss);
			RTC_SetTime (LPC_RTC, RTC_TIMETYPE_MINUTE, time.mm);
			RTC_SetTime (LPC_RTC, RTC_TIMETYPE_HOUR, time.hh);
		}
		if(time.DST_end_calculated == time.unix){
			if(time.unix_dst_last_update != time.DST_end_calculated){
				time.unix_dst_last_update = time.unix;
				time.unix -= DST_CORRECTION_VALUE_SEC;
				unix_to_hh_mm_ss(time.unix, &time.hh, &time.mm, &time.ss);
				RTC_SetTime (LPC_RTC, RTC_TIMETYPE_SECOND, time.ss);
				RTC_SetTime (LPC_RTC, RTC_TIMETYPE_MINUTE, time.mm);
				RTC_SetTime (LPC_RTC, RTC_TIMETYPE_HOUR, time.hh);
			}
		}
		time.unix_utc = time.unix - DST_CORRECTION_VALUE_SEC;
		unix_to_hh_mm_ss(time.unix_utc, &time.hh_utc, &time.mm_utc, &time.ss_utc);
//		_DBD(time.hh_utc);_DBG(" (");_DBG(__FILE__);_DBG(":");_DBD16(__LINE__);_DBG(")\r\n");
	}else{
		time.unix_utc = time.unix;
		time.hh_utc = time.hh;
		time.mm_utc = time.mm;
		time.ss_utc = time.ss;
	}

}

void RTC_set_default_time_to_compiled(void) {
	uint8_t month_as_number;
	switch (__DATE__[0]) {
	    case 'J': month_as_number = __DATE__[1] == 'a' ? 1 : __DATE__[2] == 'n' ? 6 : 7; break;
	    case 'F': month_as_number = 2; break;
	    case 'A': month_as_number = __DATE__[2] == 'r' ? 4 : 8; break;
	    case 'M': month_as_number = __DATE__[2] == 'r' ? 3 : 5; break;
	    case 'S': month_as_number = 9; break;
	    case 'O': month_as_number = 10; break;
	    case 'N': month_as_number = 11; break;
	    case 'D': month_as_number = 12; break;
	}
	RTC_time_SetTime(
			conv2d(__DATE__ + 7)*100 + conv2d(__DATE__ + 9),
			month_as_number,
			conv2d(__DATE__ + 4),
			conv2d(__TIME__),
			conv2d(__TIME__ + 3),
			conv2d(__TIME__ + 6),
			0);
}

void RTC_print_time(void){

	_DBG("[INFO]-Date=");
	_DBD(GetDOM());
	_DBG("/");
	_DBD(GetM());
	_DBG("/");
	_DBD16(GetY());
	_DBG(" (");_DBG(__FILE__);_DBG(":");_DBD16(__LINE__);_DBG(")\r\n");

	_DBG("[INFO]-Time=");
	_DBD(GetHH());
	_DBG(":");
	_DBD(GetMM());
	_DBG(":");
	_DBD(GetSS());
	_DBG(" (");_DBG(__FILE__);_DBG(":");_DBD16(__LINE__);_DBG(")\r\n");

	_DBG("[INFO]-UTC-Time=");
	_DBD(time.hh_utc);
	_DBG(":");
	_DBD(time.mm_utc);
	_DBG(":");
	_DBD(time.ss_utc);
	_DBG(" (");_DBG(__FILE__);_DBG(":");_DBD16(__LINE__);_DBG(")\r\n");

	_DBG("[INFO]-Unix: ");
	_DBD32(time.unix);
	_DBG("  Sunrise: ");
	_DBD32(time.sunrise_unix);
	_DBG("  Sunset: ");
	_DBD32(time.sunset_unix);
	_DBG("  Noon: ");
	_DBD32(time.noon_unix);
	_DBG("  Day/Night: ");
	_DBD(time.day_night);
	_DBG(" (");_DBG(__FILE__);_DBG(":");_DBD16(__LINE__);_DBG(")\r\n");

	_DBG("[INFO]-DST begin: ");
	_DBD32(time.DST_begin_calculated);
	_DBG("  end: ");
	_DBD32(time.DST_end_calculated);
	_DBG(" (");_DBG(__FILE__);_DBG(":");_DBD16(__LINE__);_DBG(")\r\n");
}

void unix_to_hh_mm_ss (uint32_t t, uint8_t * hh, uint8_t * mm, uint8_t * ss) {
	t -= SECONDS_FROM_1970_TO_2000;    // bring to 2000 timestamp from 1970
	*ss = (uint32_t)(t % 60);
	t /= 60;
	*mm = (uint32_t)t % 60;
	t /= 60;
	*hh = (uint32_t)t % 24;
}

uint32_t dst_correction_needed() {
	if(time.DST_begin_calculated<=time.unix && time.unix<=time.DST_end_calculated)
		return DST_CORRECTION_VALUE_SEC;
	else
		return 0;
}

void DSTyearly() {
	//Calculate DST's every year
//	_DBG("[INFO]-time.dst_last_update_year=");_DBD16(time.dst_last_update_year);_DBG(" (");_DBG(__FILE__);_DBG(":");_DBD16(__LINE__);_DBG(")\r\n");
//	_DBG("[INFO]-time.year=");_DBD16(time.year);_DBG(" (");_DBG(__FILE__);_DBG(":");_DBD16(__LINE__);_DBG(")\r\n");
//	if(time.dst_last_update_year != time.year) {
//		_DBG("[INFO]-time.dst_last_update_year != time.year");_DBG(" (");_DBG(__FILE__);_DBG(":");_DBD16(__LINE__);_DBG(")\r\n");
		time.DST_begin_calculated = begin_DST_unix(time.year);
		time.DST_end_calculated = end_DST_unix(time.year);
//		time.dst_last_update_year = time.year;
//	}
}

void GetSunRiseHH_MM_SS(char *str){
	uint8_t hh, mm, ss;
	unix_to_hh_mm_ss (time.sunrise_unix, &hh, &mm, &ss);
	sprintf(str,"%.2d:%.2d:%.2d",hh,mm,ss);
}

void GetSunSetHH_MM_SS(char *str){
	uint8_t hh, mm, ss;
	unix_to_hh_mm_ss (time.sunset_unix, &hh, &mm, &ss);
	sprintf(str,"%.2d:%.2d:%.2d",hh,mm,ss);
}

void GetNoonHH_MM_SS(char *str){
	uint8_t hh, mm, ss;
	unix_to_hh_mm_ss (time.noon_unix, &hh, &mm, &ss);
	sprintf(str,"%.2d:%.2d:%.2d",hh,mm,ss);
}
