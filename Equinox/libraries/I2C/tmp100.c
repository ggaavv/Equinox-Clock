/*
 * tmp100.c
 *
 *  Created on: 1 Jul 2012
 *      Author: gavin
 */

#include "tmp100.h"
#include "lpc17xx_i2c.h"



void tmp100_set_reg(unsigned char tmpres)
{
	StartI2C();
	WriteI2C(0x90); // Write to point register
	WriteI2C(0x01); // Write configuration reg to point register;
	WriteI2C(tmpres<<5);
	StopI2C();
}

void tmp100_setconf(unsigned char conf)
{
	StartI2C();
	WriteI2C(0x90); // Write to point register
	WriteI2C(0x01); // Write configuration reg to point register;
	WriteI2C(conf);
	StopI2C();
}

int tmp100_gettemp()
{
	unsigned char rec_char;
	unsigned int rec_temp;
	uint8_t Tx_Buf[3];
	uint8_t Rx_Buf[3];
	I2C_M_SETUP_Type transferMCfg;
	transferMCfg.sl_addr7bit = TMP100_ADDR;
	transferMCfg.tx_data = Tx_Buf;

	Tx_Buf[0] = 0x90; // Write to point register
	Tx_Buf[1] = 0x00; // Write Temp reg to point register;

	transferMCfg.tx_length = 2;
	transferMCfg.rx_data = NULL;
	transferMCfg.rx_length = 0;
	transferMCfg.retransmissions_max = 3;
	I2C_MasterTransferData(LPC_I2C1, &transferMCfg, I2C_TRANSFER_POLLING);

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

void tmp100_tostr(unsigned int temp, char* tempstr)
{
	unsigned int dec;
	dec = ((temp >> 4) & 0x000F)*625;
	sprintf(tempstr,"%d.%04d",temp>>8,dec);
}
