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
	uint8_t Tx_Buf[3];
	I2C_M_SETUP_Type transferMCfg;
	transferMCfg.sl_addr7bit = TMP100_ADDR;

	Tx_Buf[0] = 0x01; // Write configuration reg to point register;
	Tx_Buf[1] = tmpres << 5;
	transferMCfg.tx_data = Tx_Buf;
	transferMCfg.tx_length = 3;
	transferMCfg.rx_data = NULL;
	transferMCfg.rx_length = 0;
	transferMCfg.retransmissions_max = 3;
	I2C_MasterTransferData(LPC_I2C1, &transferMCfg, I2C_TRANSFER_POLLING);
}

void tmp100_setconf(unsigned char conf)
{
	uint8_t Tx_Buf[3];
	I2C_M_SETUP_Type transferMCfg;
	transferMCfg.sl_addr7bit = TMP100_ADDR;

	Tx_Buf[0] = 0x01; // Write configuration reg to point register;
	Tx_Buf[1] = conf;
	transferMCfg.tx_data = Tx_Buf;
	transferMCfg.tx_length = 2;
	transferMCfg.rx_data = NULL;
	transferMCfg.rx_length = 0;
	transferMCfg.retransmissions_max = 3;
	I2C_MasterTransferData(LPC_I2C1, &transferMCfg, I2C_TRANSFER_POLLING);
}

int tmp100_gettemp()
{
	unsigned char rec_char;
	unsigned int rec_temp;
	uint8_t Tx_Buf[1];
	uint8_t Rx_Buf[2];
	I2C_M_SETUP_Type transferMCfg;
	transferMCfg.sl_addr7bit = TMP100_ADDR;

	Tx_Buf[1] = 0x00; // Write Temp reg to point register;
	transferMCfg.tx_data = Tx_Buf;
	transferMCfg.tx_length = 1;
	transferMCfg.rx_data = NULL;
	transferMCfg.rx_length = 0;
	transferMCfg.retransmissions_max = 3;
	I2C_MasterTransferData(LPC_I2C1, &transferMCfg, I2C_TRANSFER_POLLING);

	transferMCfg.tx_data = NULL;
	transferMCfg.tx_length = 0;
	transferMCfg.rx_data = Rx_Buf;
	transferMCfg.rx_length = 2;
	transferMCfg.retransmissions_max = 3;
	I2C_MasterTransferData(LPC_I2C1, &transferMCfg, I2C_TRANSFER_POLLING);

	rec_temp = (int)Rx_Buf[0];
	rec_temp = rec_temp | (Rx_Buf[1] << 8);

	return rec_temp;
}

void tmp100_tostr(unsigned int temp, char* tempstr)
{
	unsigned int dec;
	dec = ((temp >> 4) & 0x000F)*625;
	sprintf(tempstr,"%d.%04d",temp>>8,dec);
}
