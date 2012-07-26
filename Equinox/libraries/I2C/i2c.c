/*
 * i2c.c
 *
 *  Created on: 1 Jul 2012
 *      Author: gavin
 */

#include "pinout.h"
#include "lpc17xx_pinsel.h"
#include "lpc17xx_gpio.h"
#include "lpc17xx_i2c.h"
#include "i2c.h"
#include "tmp100.h"
#include "MCP4018.h"


void I2C_init() {
	// Initialize SDA pins
	PINSEL_CFG_Type PinCfg;
	/* SSEL1 */
	PinCfg.Funcnum = PINSEL_FUNC_3;
	PinCfg.OpenDrain = PINSEL_PINMODE_OPENDRAIN;
	PinCfg.Pinmode = PINSEL_PINMODE_TRISTATE;
	PinCfg.Pinnum = TMP100_SDA1_PIN;
	PinCfg.Portnum = TMP100_SDA1_PORT;
	PINSEL_ConfigPin(&PinCfg);
	/* SCK1 */
	PinCfg.Funcnum = PINSEL_FUNC_3;
	PinCfg.OpenDrain = PINSEL_PINMODE_OPENDRAIN;
	PinCfg.Pinmode = PINSEL_PINMODE_TRISTATE;
	PinCfg.Pinnum = TMP100_SLC1_PIN;
	PinCfg.Portnum = TMP100_SLC1_PORT;
	PINSEL_ConfigPin(&PinCfg);

	// LPC_I2C1 supports fast mode 400kHz and standard mode 100 kHz
	// TMP100 supports fast up to 400kHz and high-speed up to 3.4MHz
	I2C_Init(LPC_I2C1, 400000);
	I2C_Cmd(LPC_I2C1, ENABLE);

	// Configure TMP100 - temperature sensor
//	tmp100_set_reg();

	// Configure Digital Pot
	// TODO
}

