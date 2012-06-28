/*
 * rtc.h
 *
 *  Created on: 27 Jun 2012
 *      Author: gavin
 */

#ifndef RTC_H_
#define RTC_H_

/*
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
static unsigned char Month_of_the_year[NUM_MONTHS][MONTH_MAX_LEN]
*/

void RTC_IRQHandler(void);
void RTC_time_Init()
void RTC_time_SetTime(uint16_t year, uint8_t month, uint8_t dayOfM, uint8_t hour, uint8_t min, uint8_t sec);
void RTC_time_GetTime(uint16_t* year, uint8_t* month, uint8_t* dayOfM, uint8_t* dayOfW, uint8_t* dayOfY, uint8_t* hour, uint8_t* min, uint8_t* sec);
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
void RTC_set_default_time_to_compiled(void);
void RTC_print_time(void);



#endif /* RTC_H_ */
