/*
 * rtc.h
 *
 *  Created on: 27 Jun 2012
 *      Author: gavin
 */

#ifndef RTC_H_
#define RTC_H_

/*
 * rtc.c
 *
 *  Created on: 10 Jun 2012
 *      Author: gavin
 */

#include "debug_frmwrk.h"
#include "lpc17xx_rtc.h"



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
void RTC_time_GetTime(uint16_t* year, uint8_t* month, uint8_t* dayOfM, uint8_t* dayOfW, uint8_t* dayOfY, uint8_t* hour, uint8_t* min, uint8_t* sec);
uint8_t RTC_DST(uint16_t year, uint8_t month, uint8_t dayOfM, uint8_t hour);
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
uint32_t Convert_To_Unixtime(uint16_t year, uint8_t month, uint8_t dayOfM, uint8_t hour, uint8_t min, uint8_t sec);
void DST_check_and_correct();
void RTC_set_default_time_to_compiled(void);
void RTC_print_time(void);


#endif /* RTC_H_ */
