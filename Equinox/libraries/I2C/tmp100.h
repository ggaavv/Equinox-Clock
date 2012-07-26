/* Copyright © 2012 Demetris Stavrou
 * version 1.0
 * ---------------------------------
 * Minimal Library for the TMP100 temperature sensor from TI
 *
 * Just include this file in your project
 *
 * tm_gettemp() will return 2 bytes containing the temperature
 * tm_setconf(byte) will write to the configuration of tmp100. You can
 * use that for changing the temperature resolution
 *
 * Example:
 *
 * include "tmp100.h"
 * ...
 * char str_tmpr[8]; // string to hold the temperature in text
 *
 * OpenI2C(MASTER,SLEW_OFF);
 * tm_setconf(0x60); // set to maximum resolution
 * raw_tmpr = tm_gettemp(); // get temperature reading
 * tm_tostr(raw_tmpr,str_tmpr); // create the temperature text
 * printf("Temperature is %s\r",str_tmpr); // output through printf
 *
 */

#ifndef __TMP100_H
#define __TMP100_H

#include "lpc17xx_i2c.h"

/*---------------------*/
#define TMP100_REG_TEMP		0x00	// Temperature Register (READ Only)
#define TMP100_REG_CFG		0x01	// Configuration Register (READ/WRITE)
#define TMP100_REG_T_LOW	0x02	// TLOW Register (READ/WRITE)
#define TMP100_REG_T_HIGH	0x03	// THIGH Register (READ/WRITE)
/*----------------------*/

/*====TMP100_REG_CFG====*/
/*-----Resolution-bits--*/
#define TMP100_RES_9bits	0x00	// 40ms
#define TMP100_RES_10bits	0x20	// 80ms
#define TMP100_RES_11bits	0x40	// 160ms
#define TMP100_RES_12bits	0x50	// 320ms
#define TMP100_RES_BITS		0x50
/*--------Faults--------*/
#define TMP100_RES_0_F		0x00
#define TMP100_RES_2_F		0x08
#define TMP100_RES_4_F		0x10
#define TMP100_RES_6_F		0x18
#define TMP100_RES_ALL_F	0x18
/*----------------------*/





#define TMP100_ADDR 		0x48   // 7bit address

/*
 * tmp100.c
 *
 *  Created on: 1 Jul 2012
 *      Author: gavin
 */

#include "tmp100.h"
#include "lpc17xx_i2c.h"
#include "comm.h"


uint8_t tmp100_conf(unsigned char cfg);

int tmp100_gettemp();

void tmp100_tostr(unsigned int temp);//, char* tempstr)


#endif /* TMP100_H_ */
