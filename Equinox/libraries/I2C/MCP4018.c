/*
 * MCP4018.c
 *
 *  Created on: 1 Jul 2012
 *      Author: gavin
 */

#include "MCP4018.h"
#include "lpc17xx_i2c.h"



void MCP4018_set_reg(unsigned char tmpres)
{
	uint8_t Tx_Buf[3];
	I2C_M_SETUP_Type transferMCfg;
	transferMCfg.sl_addr7bit = MCP4018_ADDR;
	transferMCfg.tx_data = Tx_Buf;
	transferMCfg.rx_data = NULL;

	Tx_Buf[0] = 0x90; // Write to point register
	Tx_Buf[1] = 0x91; // Write configuration reg to point register;
	Tx_Buf[2] = tmpres << 5;
	transferMCfg.tx_length = 3;
	transferMCfg.rx_data = NULL;
	transferMCfg.rx_length = 0;
	transferMCfg.retransmissions_max = 3;
	I2C_MasterTransferData(LPC_I2C1, &transferMCfg, I2C_TRANSFER_POLLING);
}

void MCP4018_setconf(unsigned char conf)
{
	uint8_t Tx_Buf[3];
	I2C_M_SETUP_Type transferMCfg;
	transferMCfg.sl_addr7bit = MCP4018_ADDR;
	transferMCfg.tx_data = Tx_Buf;
	transferMCfg.rx_data = NULL;

	Tx_Buf[0] = 0x90; // Write to point register
	Tx_Buf[1] = 0x01; // Write configuration reg to point register;
	Tx_Buf[2] = conf;
	transferMCfg.tx_length = 3;
	transferMCfg.rx_data = NULL;
	transferMCfg.rx_length = 0;
	transferMCfg.retransmissions_max = 3;
	I2C_MasterTransferData(LPC_I2C1, &transferMCfg, I2C_TRANSFER_POLLING);

}

int MCP4018_gettemp()
{
	unsigned char rec_char;
	unsigned int rec_temp;
	uint8_t Tx_Buf[2];
	uint8_t Rx_Buf[2];
	I2C_M_SETUP_Type transferMCfg;
	transferMCfg.sl_addr7bit = MCP4018_ADDR;

	Tx_Buf[0] = 0x90; // Write to point register
	Tx_Buf[1] = 0x00; // Write Temp reg to point register;
	transferMCfg.tx_data = Tx_Buf;
	transferMCfg.tx_length = 2;
	transferMCfg.rx_data = NULL;
	transferMCfg.rx_length = 0;
	transferMCfg.retransmissions_max = 3;
	I2C_MasterTransferData(LPC_I2C1, &transferMCfg, I2C_TRANSFER_POLLING);

	Tx_Buf[0] = 0x91;
	transferMCfg.tx_data = Tx_Buf;
	transferMCfg.tx_length = 1;
	transferMCfg.rx_data = Rx_Buf;
	transferMCfg.rx_length = 2;
	transferMCfg.retransmissions_max = 3;
	I2C_MasterTransferData(LPC_I2C1, &transferMCfg, I2C_TRANSFER_POLLING);

	rec_temp = (int)Rx_Buf[0];
	rec_temp = rec_temp | (Rx_Buf[1] << 8);

	return rec_temp;
}

void MCP4018_tostr(unsigned int temp, char* tempstr)
{
	unsigned int dec;
	dec = ((temp >> 4) & 0x000F)*625;
	sprintf(tempstr,"%d.%04d",temp>>8,dec);
}
