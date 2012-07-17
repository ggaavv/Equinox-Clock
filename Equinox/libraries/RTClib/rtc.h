/*
 * rtc.h
 *
 *  Created on: 27 Jun 2012
 *      Author: gavin
 */

#ifndef RTC_H_
#define RTC_H_

extern "C" {
	#include "lpc_types.h"
	#include "comm.h"
}

typedef enum {
	NIGHT,
	DAY,
	RISE_ONLY,
	SET_ONLY,
	ALL_DAY,
	ALL_NIGHT
} Day_Night_Num;

typedef enum {
	DAYLY,
	EVERYOTHERDAY,
	EVERYOTHERDAY_S_TOMORROW,
	WEEKLY,
	FORTNIGHTLY,
	MONTHLY
} Alarm_type;

#define MAX_NO_ALARMS  10
struct {
	FunctionalState enable_disabled;	// Enable/Disable,
	uint32_t unix_alarm_set_for;	// Unix start time - to sort by
	uint32_t unix_start;	// Unix start time
	uint32_t unix_finish;	// Unix finish time
	uint32_t dow_dom;			// day of the week - bit0 = Monday -  bit1 = Tueday/1stMonth -  bit2 = Wednesday/2ndMonth
	Alarm_type type;		// type from structure
} alarm[MAX_NO_ALARMS];

#define DOW_LEN 3
#define DOW_LEN_MAX 10 // +1 to include NULL
#define NUM_DAYS_OF_WEEK 7
const char DayOfWeekName[NUM_DAYS_OF_WEEK][DOW_LEN_MAX] = {
"Monday",
"Tuesday",
"Wednesday",
"Thursday",
"Friday",
"Saturday",
"Sunday"
};

#define MONTH_LEN 3
#define MONTH_LEN_MAX 10 // +1 to include NULL
#define NUM_MONTHS 12
const char Month_of_the_year[NUM_MONTHS][MONTH_LEN_MAX] = {
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

const uint8_t daysInMonth [12] = { 31,28,31,30,31,30,31,31,30,31,30,31 };

struct {
	uint32_t unix_utc;	// updated once a second
	uint32_t unix;		// updated once a second
	uint16_t year;		// updated once a second //format 2xxx
	uint8_t month;		// updated once a second //format 1-12
	uint8_t dom;		// updated once a second //format 1-31
	uint8_t dow;		// updated once a second //format 0-6
	uint8_t doy;		// updated once a second //format 1-366
	uint8_t hh;			// updated once a second //format 0-23
	uint8_t mm;			// updated once a second //format 0-59
	uint8_t ss;			// updated once a second //format 0-59
	uint32_t DST_begin_calculated;	// updated once a year
	uint32_t DST_end_calculated;	// updated once a year
	uint32_t unix_dst_last_update;	// updated once a year
	int32_t dst_correction;		// updated once
	uint32_t sunrise_unix;	// updated once a day
	uint32_t sunset_unix;	// updated once a day
	uint32_t noon_unix;  	// updated once a day
	uint8_t day_night;  	// updated once a day
	uint8_t no_set_rise;  	// updated once a day
	uint8_t second_inc;		// updated once a second
	uint8_t alarm_no_active;// updated when alarm set and after it has been activated
//	uint32_t use_utc;
} time2;


struct {
	uint32_t unix_utc;	// updated once a second
	uint32_t unix;		// updated once a second
	uint16_t year;		// updated once a second //format 2xxx
	uint8_t month;		// updated once a second //format 1-12
	uint8_t dom;		// updated once a second //format 1-31
	uint8_t dow;		// updated once a second //format 0-6
	uint8_t doy;		// updated once a second //format 1-366
	uint8_t hh;			// updated once a second //format 0-23
	uint8_t mm;			// updated once a second //format 0-59
	uint8_t ss;			// updated once a second //format 0-59
	uint32_t DST_begin_calculated;	// updated once a year
	uint32_t DST_end_calculated;	// updated once a year
	uint32_t unix_dst_last_update;	// updated once a year
	int32_t dst_correction;		// updated once
	uint32_t sunrise_unix;	// updated once a day
	uint32_t sunset_unix;	// updated once a day
	uint32_t noon_unix;  	// updated once a day
	uint8_t day_night;  	// updated once a day
	uint8_t no_set_rise;  	// updated once a day
	uint8_t second_inc;		// updated once a second
	uint8_t alarm_no_active;// updated when alarm set and after it has been activated
//	uint32_t use_utc;
} timestruct;

void RTC_time_Init(void);

// Interrupt checks
void secondlyCheck(void);
void hourlyCheck(void) ;
void minutelyCheck(void);
void dailyCheck(void);
void weeklyCheck(void);
void monthlyCheck(void);
void yearlyCheck(void);

uint32_t Getunix(void);
uint16_t GetY(void);
uint8_t GetM(void);
uint8_t GetDOM(void);
uint8_t GetDOW(void);
uint8_t GetDOY(void);
uint8_t GetHH(void);
uint8_t GetMM(void);
uint8_t GetSS(void);
void GetSunRiseHH_MM_SS(char * str);
void GetSunSetHH_MM_SS(char * str);
void GetNoonHH_MM_SS(char * str);
int8_t GetDST_correction(void);
void RTC_time_SetTime(uint16_t year, uint8_t month, uint8_t dayOfM, uint8_t hour, uint8_t min, uint8_t sec, int8_t st);
// number of days since 20xx/01/01, valid for 2001..2099
uint16_t days_from_20xx(uint16_t year, uint8_t month, uint8_t dayOfM);
// number of days since 2000/01/01, valid for 2001..2099
uint16_t days_from_2000(uint16_t year, uint8_t month, uint8_t dayOfM);
// convert date into seconds
long time2long(uint16_t days, uint8_t hour, uint8_t min, uint8_t sec);
// find day of the week
uint8_t dayOfWeekManual(uint16_t year, uint8_t month, uint8_t dayOfM);
// convert char to uint8_t for time
uint8_t conv2d(const char* p);
uint32_t RTC_time_GetUnixtime(void);
uint32_t RTC_time_FindUnixtime(uint16_t year, uint8_t month, uint8_t dayOfM, uint8_t hour, uint8_t min, uint8_t sec);
void DST_check_and_correct(void);
void RTC_set_default_time_to_compiled(void);
void RTC_print_time(void);
void unix_to_hh_mm_ss(uint32_t t, uint8_t * hh, uint8_t * mm, uint8_t * ss);
uint32_t dst_correction_needed();
uint32_t dst_correction_needed_t_y(uint32_t t, uint16_t year);
void DSTyearly(void);
void update_time(void);
void add_alarm(uint8_t month, uint8_t dom, uint8_t hh, uint8_t mm, uint16_t duration, uint32_t dow_dom, Alarm_type type);
void set_next_alarm(void);
void sort_alarms(void);
void setDSTstatus(uint32_t set_or_clear);
uint32_t getDSTstatus(void);

#endif /* RTC_H_ */
