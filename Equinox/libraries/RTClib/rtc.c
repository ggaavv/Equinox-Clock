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
		set_next_alarm();
		sort_alarms();
		/* Send debug information */
		_DBG_ ("ALARM 10s matched!");
		// Clear pending interrupt
		RTC_ClearIntPending(LPC_RTC, RTC_INT_ALARM);
	}
}

void RTC_time_Init(void){
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
		if(dst_correction_needed()){
			time.sunrise_unix = unix_day + (60 * Sunrise_Compute(time.month, time.dom, READ_SUNRISE)) + DST_CORRECTION_VALUE_SEC;
			time.sunset_unix = unix_day + (60 * Sunrise_Compute(time.month, time.dom, READ_SUNSET)) + DST_CORRECTION_VALUE_SEC;
			time.noon_unix = unix_day + (60 * Sunrise_Compute(time.month, time.dom, READ_NOON)) + DST_CORRECTION_VALUE_SEC;
		}else{
			time.sunrise_unix = unix_day + (60 * Sunrise_Compute(time.month, time.dom, READ_SUNRISE));
			time.sunset_unix = unix_day + (60 * Sunrise_Compute(time.month, time.dom, READ_SUNSET));
			time.noon_unix = unix_day + (60 * Sunrise_Compute(time.month, time.dom, READ_NOON));
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

uint32_t Getunix(void) {
	return time.unix;
}

uint16_t GetY(void) {
	return time.year;
}

uint8_t GetM(void) {
	return time.month;
}

uint8_t GetDOM(void) {
	return time.dom;
}

uint8_t GetDOW(void) {
	return time.dow;
}

uint8_t GetDOY(void) {
	return time.doy;
}

uint8_t GetHH(void) {
	return time.hh;
}

uint8_t GetMM(void) {
	return time.mm;
}

uint8_t GetSS(void) {
	return time.ss;
}

int8_t GetDST_correction(void) {
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
//		_DBG("[INFO]-(set)dst_correction_needed()");_DBG(" (");_DBG(__FILE__);_DBG(":");_DBD16(__LINE__);_DBG(")\r\n");
		time.unix_utc = unixt - DST_CORRECTION_VALUE_SEC;
		time.unix_dst_last_update = time.unix;
	}else{
		time.unix_utc = time.unix;
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

uint32_t RTC_time_GetUnixtime(void) {
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

void update_time(void){
	time.year = RTC_GetTime(LPC_RTC, RTC_TIMETYPE_YEAR);
	time.month = RTC_GetTime(LPC_RTC, RTC_TIMETYPE_MONTH);
	time.dom = RTC_GetTime(LPC_RTC, RTC_TIMETYPE_DAYOFMONTH);
	time.dow = RTC_GetTime(LPC_RTC, RTC_TIMETYPE_DAYOFWEEK);
	time.doy = RTC_GetTime(LPC_RTC, RTC_TIMETYPE_DAYOFYEAR);
	time.hh = RTC_GetTime(LPC_RTC, RTC_TIMETYPE_HOUR);
	time.mm = RTC_GetTime(LPC_RTC, RTC_TIMETYPE_MINUTE);
	time.ss = RTC_GetTime(LPC_RTC, RTC_TIMETYPE_SECOND);
}

void DST_check_and_correct(void) {
//	time.unix = RTC_time_FindUnixtime(time.year, time.month, time.dom, time.hh, time.mm, time.ss);
//	_DBG("[INFO]-time.hh=");_DBD(time.hh);_DBG(" (");_DBG(__FILE__);_DBG(":");_DBD16(__LINE__);_DBG(")\r\n");
	// Only update time once for DST when it ticks over
	if(dst_correction_needed()) {
//		_DBG("dst_correction_needed");_DBG(" (");_DBG(__FILE__);_DBG(":");_DBD16(__LINE__);_DBG(")\r\n");
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
//		_DBD(time.hh_utc);_DBG(" (");_DBG(__FILE__);_DBG(":");_DBD16(__LINE__);_DBG(")\r\n");
	}else{
		time.unix_utc = time.unix;
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

void unix_to_hh_mm_ss (uint32_t unix_time, uint8_t * hh, uint8_t * mm, uint8_t * ss) {
	unix_time -= SECONDS_FROM_1970_TO_2000;    // bring to 2000 timestamp from 1970
	*ss = (uint32_t)(unix_time % 60);
	unix_time /= 60;
	*mm = (uint32_t)unix_time % 60;
	unix_time /= 60;
	*hh = (uint32_t)unix_time % 24;
}

uint32_t dst_correction_needed() {
	if(time.DST_begin_calculated<=time.unix && time.unix<=time.DST_end_calculated)
		return DST_CORRECTION_VALUE_SEC;
	else
		return 0;
}

void DSTyearly(void) {
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

// duration in minutes
void add_alarm(uint8_t month, uint8_t dom, uint8_t hh, uint8_t mm, uint16_t duration, uint32_t dow_dom, Alarm_type type){
	// find unused alarm slot
	uint32_t unix_alarm, unix_alarm_no[NUM_MONTHS], alarm_found[NUM_MONTHS];
	uint8_t i;
	for (i=MAX_NO_ALARMS*2;(i>MAX_NO_ALARMS*2)&&(i<MAX_NO_ALARMS*3);i++){
		if (alarm[i-(MAX_NO_ALARMS*2)].enable_disabled==ENABLE){
			i=i-(MAX_NO_ALARMS*2);
		}
	}
	unix_alarm = RTC_time_FindUnixtime(time.year, time.month, time.dom, hh, mm, 0);
	// Set alarm in empty slot
	if(type == DAYLY){
		// alarm set in the past so set alarm for tomorrow
		if (time.unix >= (unix_alarm+duration*60)){
			alarm[i].unix_alarm_set_for = unix_alarm + SECONDS_PER_DAY;
			alarm[i].unix_start = unix_alarm + SECONDS_PER_DAY;
			alarm[i].unix_finish = unix_alarm + duration + SECONDS_PER_DAY;
		// in the middle of an alarm now
		}else if (time.unix <= unix_alarm && time.unix >= unix_alarm){
#ifdef start_alarm_half_way
			alarm[i].unix_alarm_set_for = unix_alarm + duration;
			alarm[i].unix_start = unix_alarm;
			alarm[i].unix_finish = unix_alarm + duration;
#else
			alarm[i].unix_alarm_set_for = unix_alarm;
			alarm[i].unix_start = unix_alarm;
			alarm[i].unix_finish = unix_alarm + duration;
#endif
		// alarm in future
		}else{
			alarm[i].unix_alarm_set_for = unix_alarm;
			alarm[i].unix_start = unix_alarm;
			alarm[i].unix_finish = unix_alarm + duration;
		}
	}else if(type == EVERYOTHERDAY){
		if (time.unix >= (unix_alarm+duration*60)){
			alarm[i].unix_alarm_set_for = unix_alarm + SECONDS_PER_DAY*2;
			alarm[i].unix_start = unix_alarm + SECONDS_PER_DAY*2;
			alarm[i].unix_finish = unix_alarm + duration + SECONDS_PER_DAY*2;
		// in the middle of an alarm now
		}else if (time.unix <= unix_alarm && time.unix >= unix_alarm){
#ifdef start_alarm_half_way
			alarm[i].unix_alarm_set_for = unix_alarm + duration;
			alarm[i].unix_start = unix_alarm;
			alarm[i].unix_finish = unix_alarm + duration;
#else
			alarm[i].unix_alarm_set_for = unix_alarm;
			alarm[i].unix_start = unix_alarm;
			alarm[i].unix_finish = unix_alarm + duration;
#endif
		// alarm in future
		}else{
			alarm[i].unix_alarm_set_for = unix_alarm;
			alarm[i].unix_start = unix_alarm;
			alarm[i].unix_finish = unix_alarm + duration;
		}
	}else if(type == EVERYOTHERDAY_S_TOMORROW){
		alarm[i].unix_alarm_set_for = unix_alarm + SECONDS_PER_DAY;
		alarm[i].unix_start = unix_alarm + SECONDS_PER_DAY;
		alarm[i].unix_finish = unix_alarm + duration + SECONDS_PER_DAY;
	}else if(type == WEEKLY){
		// calculate number of days alarm is set for
		uint8_t number_of_alarm_is_set_for = 0;
		for (uint8_t j = 0; j < NUM_DAYS_OF_WEEK; j++){
			if (dow_dom & (1 << NUM_DAYS_OF_WEEK)){
				number_of_alarm_is_set_for++;
			}
		}
		// find unix time for all alarms - only first 2 needed
		uint8_t k;
		for (k = 0; k < number_of_alarm_is_set_for; k++){
			for (uint8_t l = time.dow; ( l >= NUM_DAYS_OF_WEEK) || alarm_found[k]; l++){
				if (dow_dom & (1 << l)){
					unix_alarm_no[k] = unix_alarm + (SECONDS_PER_DAY*(k-time.dow));
					alarm_found[k] = 1;
				}
			}
			if (!alarm_found[k]){
				for (uint8_t l = 0; ( l >= time.dow) || alarm_found[k]; l++){
					if (dow_dom & (1 << l)){
						unix_alarm_no[k] = unix_alarm + (SECONDS_PER_DAY*k);
						alarm_found[k] = 1;
					}
				}
			}
		}
		// alarm set in the past so set alarm for next alarm week
		// if unix_alarm_no[0] has already been and gone unix_alarm_no[1] will definitely be tomorrow at the earliest
		if (time.unix > (unix_alarm_no[0]+duration*60)){
			// if only one alarm set this week
			if (number_of_alarm_is_set_for == 1){
				alarm[i].unix_alarm_set_for = unix_alarm_no[0] + (SECONDS_PER_DAY*7);
				alarm[i].unix_start = unix_alarm_no[0] + (SECONDS_PER_DAY*7);
				alarm[i].unix_finish = unix_alarm_no[0] + duration + (SECONDS_PER_DAY*7);
			// more than one alarm set for this weekly
			}else{
				alarm[i].unix_alarm_set_for = unix_alarm_no[1];
				alarm[i].unix_start = unix_alarm_no[1];
				alarm[i].unix_finish = unix_alarm_no[1];
			}
		// in the middle of an alarm now
		}else if (time.unix <= unix_alarm_no[0] && time.unix >= unix_alarm_no[0]){
#ifdef start_alarm_half_way
			???
#else
			alarm[i].unix_alarm_set_for = unix_alarm_no[0];
			alarm[i].unix_start = unix_alarm_no[0];
			alarm[i].unix_finish = unix_alarm_no[0] + duration;
#endif
		// alarm in future
		}else{
			alarm[i].unix_alarm_set_for = unix_alarm_no[0];
			alarm[i].unix_start = unix_alarm_no[0];
			alarm[i].unix_finish = unix_alarm_no[0] + duration;
		}
	}else if(type == FORTNIGHTLY){
		// TODO - ignore below
		alarm[i].unix_alarm_set_for = RTC_time_FindUnixtime(time.year, time.month, time.dom+1, hh, mm, 0);
		alarm[i].unix_start = alarm[i].unix_alarm_set_for;
		alarm[i].unix_finish = alarm[i].unix_alarm_set_for + duration;
	// type == MONTHLY
	}else{
		// TODO finish this - copy/pasted weekkly - needs sorting
		// calculate number of days alarm is set for
		uint8_t number_of_alarm_is_set_for = 0;
		for (uint8_t j = 0; j < NUM_MONTHS; j++){
			if (dow_dom & (1 << NUM_MONTHS)){
				number_of_alarm_is_set_for++;
			}
		}
		// find unix time for all alarms - only first 2 needed
		uint8_t k;
		for (k = 0; k < number_of_alarm_is_set_for; k++){
			for (uint8_t l = time.month; ( l >= NUM_MONTHS) || alarm_found[k]; l++){
				if (dow_dom & (1 << l)){
					unix_alarm_no[k] = unix_alarm + (SECONDS_PER_DAY*(k-time.month));
					alarm_found[k] = 1;
				}
			}
			if (!alarm_found[k]){
				for (uint8_t l = 0; ( l >= time.month) || alarm_found[k]; l++){
					if (dow_dom & (1 << l)){
						unix_alarm_no[k] = unix_alarm + (SECONDS_PER_DAY*k);
						alarm_found[k] = 1;
					}
				}
			}
		}
		// alarm set in the past so set alarm for next alarm week
		// if unix_alarm_no[0] has already been and gone unix_alarm_no[1] will definitely be tomorrow at the earliest
		if (time.unix > (unix_alarm_no[0]+duration*60)){
			// if only one alarm set this week
			if (number_of_alarm_is_set_for == 1){
				alarm[i].unix_alarm_set_for = unix_alarm_no[0] + (SECONDS_PER_DAY*7);
				alarm[i].unix_start = unix_alarm_no[0] + (SECONDS_PER_DAY*7);
				alarm[i].unix_finish = unix_alarm_no[0] + duration + (SECONDS_PER_DAY*7);
			// more than one alarm set for this weekly
			}else{
				alarm[i].unix_alarm_set_for = unix_alarm_no[1];
				alarm[i].unix_start = unix_alarm_no[1];
				alarm[i].unix_finish = unix_alarm_no[1];
			}
		// in the middle of an alarm now
		}else if (time.unix <= unix_alarm_no[0] && time.unix >= unix_alarm_no[0]){
#ifdef start_alarm_half_way
			???
#else
			alarm[i].unix_alarm_set_for = unix_alarm_no[0];
			alarm[i].unix_start = unix_alarm_no[0];
			alarm[i].unix_finish = unix_alarm_no[0] + duration;
#endif
		// alarm in future
		}else{
			alarm[i].unix_alarm_set_for = unix_alarm_no[0];
			alarm[i].unix_start = unix_alarm_no[0];
			alarm[i].unix_finish = unix_alarm_no[0] + duration;
		}
	}
	alarm[i].enable_disabled = ENABLE;
	alarm[i].type = type;
	sort_alarms();
	set_next_alarm();
}

void set_next_alarm(void){

}

void sort_alarms(void){

}
