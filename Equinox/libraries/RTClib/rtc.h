/*
 * rtc.h
 *
 *  Created on: 27 Jun 2012
 *      Author: gavin
 */

#ifndef RTC_H_
#define RTC_H_


uint16_t dateoftheyear(uint16_t year, uint8_t month, uint8_t dayOfM);
long time2long(uint16_t days, uint8_t hour, uint8_t min, uint8_t sec);
uint8_t dayOfWeekManual(uint16_t year, uint8_t month, uint8_t dayOfM);



#endif /* RTC_H_ */
