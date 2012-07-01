/*
 * tmp100.c
 *
 *  Created on: 1 Jul 2012
 *      Author: gavin
 */

#include "tmp100.h"
#include "pinout.h"
#include "lpc17xx_pinsel.h"
#include "lpc17xx_gpio.h"
#include "lpc17xx_i2c.h"


void Tmp100_init(){
	// Initialize SDA pins
	PINSEL_CFG_Type PinCfg;
	/* SSEL1 */
	PinCfg.Funcnum   = PINSEL_FUNC_3;
	PinCfg.OpenDrain = PINSEL_PINMODE_OPENDRAIN;
	PinCfg.Pinmode   = PINSEL_PINMODE_TRISTATE;
	PinCfg.Pinnum    = TMP100_SDA1_PIN;
	PinCfg.Portnum   = TMP100_SDA1_PORT;
	PINSEL_ConfigPin(&PinCfg);
	/* SCK1 */
	PinCfg.Funcnum   = PINSEL_FUNC_3;
	PinCfg.OpenDrain = PINSEL_PINMODE_OPENDRAIN;
	PinCfg.Pinmode   = PINSEL_PINMODE_TRISTATE;
	PinCfg.Pinnum    = TMP100_SLC1_PIN;
	PinCfg.Portnum   = TMP100_SLC1_PORT;
	PINSEL_ConfigPin(&PinCfg);

	// LPC_I2C1 supports fast mode 400kHz and standard mode 100 kHz
	// TMP100 supports fast up to 400kHz and high-speed up to 3.4MHz
	I2C_Init(LPC_I2C1, 400000);
	I2C_Cmd(LPC_I2C1, ENABLE);
}

void tm_setres(unsigned char tmpres)
{
    StartI2C();
    WriteI2C(0x90); // Write to point register
    WriteI2C(0x01); // Write configuration reg to point register;
    WriteI2C(tmpres<<5);
    StopI2C();
}

void tm_setconf(unsigned char conf)
{
    StartI2C();
    WriteI2C(0x90); // Write to point register
    WriteI2C(0x01); // Write configuration reg to point register;
    WriteI2C(conf);
    StopI2C();
}

int tm_gettemp()
{
    unsigned char rec_char;
    unsigned int rec_temp;

    StartI2C();
    WriteI2C(0x90); // Write to point register
    WriteI2C(0x00); // Write Temp reg to point register;
    StartI2C();
    WriteI2C(0x91);
    rec_char = ReadI2C();
    rec_temp = (int)rec_char;
    AckI2C();
    rec_char = ReadI2C();
    rec_temp = rec_temp << 8;
    rec_temp = rec_temp | rec_char;
    StopI2C();
    return rec_temp;
}

void tm_tostr(unsigned int temp, char* tempstr)
{
    unsigned int dec;
    dec = ((temp >> 4) & 0x000F)*625;
    sprintf(tempstr,"%d.%04d",temp>>8,dec);
}
