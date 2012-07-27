/*
 * tmp100.c
 *
 *  Created on: 1 Jul 2012
 *      Author: gavin
 */

#include "tmp100.h"
#include "lpc17xx_i2c.h"
#include "comm.h"


uint8_t tmp100_conf(unsigned char cfg){
	uint8_t Tx_Buf[2];
	uint8_t Rx_Buf[1];
	I2C_M_SETUP_Type transferMCfg;
	transferMCfg.sl_addr7bit = TMP100_ADDR;

	Tx_Buf[0] = TMP100_REG_CFG; // Write configuration reg to point register;
	Tx_Buf[1] = cfg;
	transferMCfg.tx_data = Tx_Buf;
	transferMCfg.tx_length = 2;
	transferMCfg.rx_data = Rx_Buf;
	transferMCfg.rx_length = 1;
	transferMCfg.retransmissions_max = 3;
	I2C_MasterTransferData(LPC_I2C1, &transferMCfg, I2C_TRANSFER_POLLING);
//	xprintf("%d",I2C_MasterTransferData(LPC_I2C1, &transferMCfg, I2C_TRANSFER_POLLING));
	return Rx_Buf[0];
}

int tmp100_gettemp() {
	float temperature;
	int32_t int_temperature;
	uint8_t Tx_Buf[2];
	uint8_t Rx_Buf[2];
	I2C_M_SETUP_Type transferMCfg;
	transferMCfg.sl_addr7bit = TMP100_ADDR;

	Tx_Buf[0] = TMP100_REG_TEMP; // Write Temp reg to point register;
	transferMCfg.tx_data = Tx_Buf;
	transferMCfg.tx_length = 1;
	transferMCfg.rx_data = Rx_Buf;
	transferMCfg.rx_length = 2;
	transferMCfg.retransmissions_max = 3;
	I2C_MasterTransferData(LPC_I2C1, &transferMCfg, I2C_TRANSFER_POLLING);
//	xprintf("I2C mater transfer status:%d",I2C_MasterTransferData(LPC_I2C1, &transferMCfg, I2C_TRANSFER_POLLING));FFL_();

//	temperature_count = ((Rx_Buf[0]&0x80)<<8)|((Rx_Buf[0]&0x7F)<<4)|(((Rx_Buf[1]&0xF0)>>4));
//	temperature = temperature_count * 0.0625;

	xprintf("%d degC",Rx_Buf[0]);FFL_();

#if 0
	xprintf("temperature-Hex=0x%x, Dec=%d",Rx_Buf[0],Rx_Buf[0]);FFL_();
	xprintf("temperature-Hex=0x%x, Dec=%d",Rx_Buf[1],Rx_Buf[1]);FFL_();
	temperature = (int)Rx_Buf[0] + Rx_Buf[1]>>4;
	int_temperature = (int)temperature - 5;
	xprintf("temperature-Hex=0x%x, Dec=%d",int_temperature,int_temperature);FFL_();

	unsigned int rec_temp;
	rec_temp = (int)Rx_Buf[0];
	rec_temp = rec_temp | (Rx_Buf[1] << 8);
//	printf("%d",rec_temp);FFL_();

	tmp100_tostr(rec_temp);
#endif

	return temperature;
}

void tmp100_tostr(unsigned int temp){//, char* tempstr)
	unsigned int dec;
//	dec = ((temp >> 4) & 0x000F)*625;
	xprintf("%d.%04d \n",temp>>8,dec);
//	sprintf(tempstr,"%d.%04d",temp>>8,dec);
}
