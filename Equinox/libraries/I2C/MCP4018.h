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

#ifndef __MCP4018_H
#define __MCP4018_H

#include "lpc17xx_i2c.h"

#define MCP4018_ADDR 		0x5e

#define MCP4018_OHM 		50000
#define MCP4018_MAX 		0x7f
#define MCP4018_MID 		0x3f
#define MCP4018_MIN 		0x00

#endif /* MCP4018_H_ */
