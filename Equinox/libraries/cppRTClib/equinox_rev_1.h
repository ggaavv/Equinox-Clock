/*
 * dotklok_rev_1_2.h
 *
 *  Created on: 3 Apr 2011
 *      Author: Jamie
 */

#ifndef DOTKLOK_REV_1_2_H_
#define DOTKLOK_REV_1_2_H_

const prog_uchar stringDOTKLOK[] PROGMEM = {"DOTKLOK "};
const prog_uchar stringRTC_NOT_running[] PROGMEM = {"RTC NOT running!\r\n"};
const prog_uchar stringRTC_running[] PROGMEM = {"RTC running\r\n"};

int availableMemory();
void dailyCheck(void);
void minutelyCheck(void);
char timeUpdate(void);
void setup();
void loop();
int main(void);

#endif /* DOTKLOK_REV_1_2_H_ */
