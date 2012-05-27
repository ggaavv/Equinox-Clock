/*
 * coms.h
 *
 *  Created on: 31 Mar 2011
 *      Author: Jamie
 */

#ifndef COMS_H_
#define COMS_H_

#include <avr/pgmspace.h>

#define BUF_MAX 30
#define toSetTimeLength 60

void PROGMEMprint(const prog_uchar str[], uint8_t length);
void PROGMEMprint(const prog_uchar str[]);
void checkSerial();
void echoDebug(const char *file, const unsigned long line, const char *msg);

#endif /* COMS_H_ */
