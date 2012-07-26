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
	xprintf("%d",I2C_MasterTransferData(LPC_I2C1, &transferMCfg, I2C_TRANSFER_POLLING));
	return Rx_Buf[0];
}

int tmp100_gettemp() {
	float temperature;
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
	xprintf("I2C mater transfer status:%d/n",I2C_MasterTransferData(LPC_I2C1, &transferMCfg, I2C_TRANSFER_POLLING));


	xprintf("%x" " (%s:%d)\n",Rx_Buf[0],_F_,_L_);
	xprintf("%d" " (%s:%d)\n",Rx_Buf[0],_F_,_L_);
	temperature = (int)Rx_Buf[0];
	xprintf("%f" " (%s:%d)\n",temperature,_F_,_L_);

	xprintf("%x" " (%s:%d)\n",Rx_Buf[1],_F_,_L_);
	xprintf("%d" " (%s:%d)\n",Rx_Buf[1],_F_,_L_);
	temperature += ((int)Rx_Buf[1]/16);
	xprintf("%d" " (%s:%d)\n",(int)temperature,_F_,_L_);

	unsigned int rec_temp;
	rec_temp = (int)Rx_Buf[0];
	rec_temp = rec_temp | (Rx_Buf[1] << 8);
	xprintf("%d" " (%s:%d)\n",rec_temp,_F_,_L_);

	tmp100_tostr(rec_temp);

	return temperature;
}

void tmp100_tostr(unsigned int temp){//, char* tempstr)
	unsigned int dec;
	dec = ((temp >> 4) & 0x000F)*625;
	xprintf("%d.%04d \n",temp>>8,dec);
//	sprintf(tempstr,"%d.%04d",temp>>8,dec);
}
