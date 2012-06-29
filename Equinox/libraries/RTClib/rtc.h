/*
 * rtc.h
 *
 *  Created on: 27 Jun 2012
 *      Author: gavin
 */

#ifndef RTC_H_
#define RTC_H_


#include "debug_frmwrk.h"
#include "lpc17xx_rtc.h"
#include "rtc.h"


struct {
	uint32_t unix;		// updated once a second
	uint32_t unix_utc;		// updated once a second
	uint16_t year;		// updated once a second
	uint8_t month;		// updated once a second
	uint8_t dom;		// updated once a second
	uint8_t dow;		// updated once a second
	uint8_t doy;		// updated once a second
	uint8_t hh;			// updated once a second
	uint8_t mm;			// updated once a second
	uint8_t ss;			// updated once a second
	uint8_t hh_utc;			// updated once a second
	uint8_t mm_utc;			// updated once a second
	uint8_t ss_utc;			// updated once a second
	uint8_t set_hh;			// updated once a second
	uint8_t set_mm;			// updated once a second
	uint8_t set_ss;			// updated once a second
	uint8_t get_hh;			// updated once a second
	uint8_t get_mm;			// updated once a second
	uint8_t get_ss;			// updated once a second
	uint32_t DST_begin_calculated;	// updated once a year
	uint32_t DST_end_calculated;	// updated once a year
	uint16_t dst_last_update_year;	// updated once a year
	uint32_t dst_correction;		// updated once
	uint32_t sunrise_unix; // updated once a day
	uint32_t sunset_unix;  // updated once a day
//	uint32_t use_utc;
} time;

void RTC_IRQHandler(void);
void RTC_time_Init();
void dailyCheck(void);
void hourlyCheck(void);
void minutelyCheck(void);
void secondlyCheck(void);
uint16_t GetY();
uint8_t GetM();
uint8_t GetDOM();
uint8_t GetDOW();
uint8_t GetDOY();
uint8_t GetHH();
uint8_t GetMM();
uint8_t GetSS();
void RTC_time_SetTime(uint16_t year, uint8_t month, uint8_t dayOfM, uint8_t hour, uint8_t min, uint8_t sec);
// number of days since 2000/01/01, valid for 2001..2099
uint16_t date2days(uint16_t year, uint8_t month, uint8_t dayOfM);
// number of days since 2000/01/01, valid for 2001..2099
uint16_t dateoftheyear(uint16_t year, uint8_t month, uint8_t dayOfM);
// convert date into seconds
long time2long(uint16_t days, uint8_t hour, uint8_t min, uint8_t sec);
// find day of the week
uint8_t dayOfWeekManual(uint16_t year, uint8_t month, uint8_t dayOfM);
// convert char to uint8_t for time
uint8_t conv2d(const char* p);
uint32_t RTC_time_GetUnixtime();
uint32_t RTC_time_FindUnixtime(uint16_t year, uint8_t month, uint8_t dayOfM, uint8_t hour, uint8_t min, uint8_t sec);
void DST_check_and_correct();
void RTC_set_default_time_to_compiled(void);
void RTC_print_time(void);
void unix_to_hh_mm_ss_get(uint32_t t);
void unix_to_hh_mm_ss_set(uint32_t t);
uint32_t dst_correction_needed();
uint32_t dst_correction_needed_t_y(uint32_t t, uint16_t year);
void DSTyearly();


#endif /* RTC_H_ */
