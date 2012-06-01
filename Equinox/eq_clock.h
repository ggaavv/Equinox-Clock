/*
 * dotklok_rev_1_2.h
 *
 *  Created on: 3 Apr 2011
 *      Author: Jamie
 */

#ifndef CLOCK_H_
#define CLOCK_H_

#include "lpc_types.h"

void LED_init();

const prog_uchar stringDOTKLOK[] = {"DOTKLOK "};
const prog_uchar stringRTC_NOT_running[] = {"RTC NOT running!\r\n"};
const prog_uchar stringRTC_running[] = {"RTC running\r\n"};

int availableMemory();
void dailyCheck(void);
void minutelyCheck(void);
char timeUpdate(void);
void setup();
void loop();
int main(void);

#endif /* CLOCK_H_ */
