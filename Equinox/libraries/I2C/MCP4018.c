/*
 * MCP4018.c
 *
 *  Created on: 1 Jul 2012
 *      Author: gavin
 */

#include "MCP4018.h"
#include "lpc17xx_i2c.h"

uint8_t setPot(uint8_t pot){

	//max value
	if( pot > 0x7f)
		pot = 0x7f;
	//min value
	if( pot < 0x01)
		pot = 0x01;

	uint8_t Tx_Buf[3];
	I2C_M_SETUP_Type transferMCfg;
	transferMCfg.sl_addr7bit = MCP4018_ADDR;

	Tx_Buf[0] = pot; // Write value
	transferMCfg.tx_data = Tx_Buf;
	transferMCfg.tx_length = 1;
	transferMCfg.rx_data = NULL;
	transferMCfg.rx_length = 0;
	transferMCfg.retransmissions_max = 3;
	return I2C_MasterTransferData(LPC_I2C1, &transferMCfg, I2C_TRANSFER_POLLING);
}

uint8_t readPot( void ){

	uint8_t Rx_Buf[3];
	I2C_M_SETUP_Type transferMCfg;
	transferMCfg.sl_addr7bit = MCP4018_ADDR;
	transferMCfg.tx_data = NULL;
	transferMCfg.tx_length = 0;
	transferMCfg.rx_data = Rx_Buf;
	transferMCfg.rx_length = 1;
	transferMCfg.retransmissions_max = 3;
	I2C_MasterTransferData(LPC_I2C1, &transferMCfg, I2C_TRANSFER_POLLING);
	return Rx_Buf[0];
}
