
#include "lpc17xx_can.h"

// avrdude -p atmega162 -c avrisp2 -P usb0 -u -U //flash:w:/home/jamie/avr/Unilink/AVR-Unilink_v2.05/main.hex:i
/*************************************************************************
**  Sony Unilink Interface (Main Procedure)
**  by Michael Wolf
**
**  Released under GNU GENERAL PUBLIC LICENSE
**	See LICENSE.TXT for details
**
**	See README.TXT for serial command details
**
**  Contact: michael@mictronics.de
**  homepage: www.mictronics.de
**
**  Revision History
**
**  when         what  who	why
**
**  2003-08-05   2.01  MIC  complete code revision
**                          + added jumper check during runtime
**                          - bugfix in time update command
**  2003-09-06	 2.02  MIC  * changed watchdog routine
**                          + added serial commands to change playmodes
**	2004-01-05   2.03  MIC  + added support for LIST function in CD mode
**														(used in Custom File CDC to choose a disc
**														by name from a list shown by the LIST button)
**													* changed display update routine for play modes
**													- bugfix in BANK mode display
**													* improved unilink_parse routine, this increase
**														reply speed like hell ;)
**	2004-02-27   2.04  MIC	* changed value for "missed ping" counter from
**														5 to 10 to solve problems with lots of bus
**														members especially with bus multiplexer units
**  2005-02-03   2.05  MIC  - removed all deprecaded avr-libc macros to be
**                            combatible with avr-libc v1.2.1 and higher
**
**  Used develompent tools (download @ www.avrfreaks.net):
**  Programmers Notepad v2.0.5.32
**  WinAVR (GCC) 3.4.1
**  AvrStudio4 for simulating and debugging
**	Tab Width: 2
**
**  [           Legend:          ]
**  [ + Added feature            ]
**  [ * Improved/changed feature ]
**  [ - Bug fixed (I hope)       ]
**
**************************************************************************/
//#include "config.h"
//#include "main.h"
#include "canlib.h"
#include "comm.h"
#include "sys_timer.h"
#include <string.h>
#include "lpc17xx_pinsel.h"

/** CAN variable definition **/
CAN_MSG_Type TXMsg, RXMsg; // messages for test Bypass mode
uint32_t CANRxCount, CANTxCount = 0;

//unsigned char ScreenLitData[12];
//unsigned char ScreenAsciiData[34];
//unsigned char ScreenSendPackets[5][8];
unsigned char PrevScreenText[10];
unsigned char Source;
volatile unsigned char ReturnScreen;
volatile unsigned int ScreenTimeoutCounter;
volatile unsigned int OffTimeout;
volatile unsigned char ScreenTempData[36];
volatile unsigned char RecieveComplete;
volatile unsigned char RX_In_progress;
volatile unsigned char ButtomPressed;
volatile unsigned char source_change;
volatile unsigned char ButtonTempData[2];

/*
**---------------------------------------------------------------------------
**
** Abstract: can_ini
**
**
** Parameters: none
**
**
** Returns: never
**
**---------------------------------------------------------------------------
*/
void CAN_init (void){

	PINSEL_CFG_Type PinCfg;
	ram_timestamp_status = 0;

	/* Pin configuration
	 * CAN1: select P0.0 as RD1. P0.1 as TD1
	 * CAN2: select P2.7 as RD2, P2.8 as RD2
	 */
	PinCfg.Funcnum = 1;
	PinCfg.OpenDrain = 0;
	PinCfg.Pinmode = 0;
//can1
//	PinCfg.Portnum = 0;
//	PinCfg.Pinnum = 0;
//	PINSEL_ConfigPin(&PinCfg);
//	PinCfg.Pinnum = 1;
//	PINSEL_ConfigPin(&PinCfg);

	PinCfg.Portnum = 2;
	PinCfg.Pinnum = 7;
	PINSEL_ConfigPin(&PinCfg);
	PinCfg.Pinnum = 8;
	PINSEL_ConfigPin(&PinCfg);

	/* Initialize CAN2 peripheral
	* Note: Self-test mode doesn't require pin selection
	*/
	CAN_Init(LPC_CAN2, 500000);

	//Enable self-test mode
	CAN_ModeConfig(LPC_CAN2, CAN_SELFTEST_MODE, ENABLE);
	//CAN_ModeConfig(LPC_CAN2, CAN_LISTENONLY_MODE, ENABLE);

	//Enable Interrupt
	CAN_IRQCmd(LPC_CAN2, CANINT_RIE, ENABLE);
	CAN_IRQCmd(LPC_CAN2, CANINT_TIE1, ENABLE);

	//Enable CAN Interrupt
	NVIC_EnableIRQ(CAN_IRQn);
	CAN_SetAFMode(LPC_CANAF,CAN_AccBP);

    // read status of time stamp setting
//    ram_timestamp_status = eeprom_read_byte (&ee_timestamp_status);
}


/*
**---------------------------------------------------------------------------
**
** Abstract: Execution of command received via USART
**
**
** Parameters: Command Buffer
**
**
** Returns: status of command execution
**          CR = OK
**          ERROR = Error
**
**---------------------------------------------------------------------------
*/
#define VERBOSE
uint8_t exec_usart_cmd (uint8_t * cmd_buf){
    uint8_t cmd_len = strlen (cmd_buf);	// get command length //no_usart_rx_chars-1; //
    uint8_t *cmd_buf_pntr = &(*cmd_buf);	// point to start of received string
    cmd_buf_pntr++;		// skip command identifier
    // check if all chars are valid hex chars
    while (*cmd_buf_pntr) {
        if (!isxdigit (*cmd_buf_pntr)){
#ifdef VERBOSE
        	xprintf("Not a valid asci char (exec_usart_cmd1).%x\r",*cmd_buf_pntr);
#endif
            return ERROR;
		}
		++cmd_buf_pntr;
    }
#ifdef VERBOSE
	//xprintf("cmd_len - %x\r",cmd_len);
#endif

    cmd_buf_pntr = &(*cmd_buf);	// reset pointer

    uint8_t *tmp_pntr;		// temporary pointer for ACR/AMR
    uint8_t tmp_regdata;	// temporary used for register data

    switch (*cmd_buf_pntr) {
            // get serial number
        case GET_SERIAL:
            xprintf("%c%s",GET_SERIAL,SERIAL);
            return CR;

            // get hard- and software version
        case GET_VERSION:
        	xprintf("%c%x%x",GET_VERSION,HW_VER,SW_VER);
            return CR;

            // get only software version
        case GET_SW_VERSION:
        	xprintf("%c%x%x",GET_SW_VERSION,SW_VER_MAJOR,SW_VER_MINOR);
            return CR;

        	// toggle time stamp option
        case TIME_STAMP:
#if 0
            // read stored status
//            ram_timestamp_status = eeprom_read_byte (&ee_timestamp_status);
            // toggle status
            if (ram_timestamp_status != 0)
                ram_timestamp_status = 0;	// disable time stamp
            else {
                ram_timestamp_status = 0xA5;	// enable time stamp
                timestamp = 0;	// reset time stamp counter
            }
            // store new status
//            eeprom_write_byte (&ee_timestamp_status, ram_timestamp_status);
#endif
            return CR;

            // read status flag
        case READ_STATUS:
            // check if CAN controller is in reset mode
            if (!CANBUS_ON()){
#ifdef VERBOSE
            	xprintf("ERR-Bus is not on.\n");FFL_();
#endif
				return ERROR;
            }

//            comm_put(READ_STATUS);
//            usart_byte2ascii ((uint8_t) (CAN_flags >> 8));
            xprintf("%c00",READ_STATUS);

            // turn off Bus Error indication
//            LED_PORT &= ~_BV (LED_RED);
            // reset error flags
//            CAN_flags &= 0x00FF;
            return CR;

            // set AMR
        case SET_AMR:
            // set ACR
        case SET_ACR:
            // check valid cmd length and if CAN was initialized before
            if (cmd_len != 9){
#ifdef VERBOSE
            	xprintf("ERR-Wrong command length should be 9 but is %d.\r",cmd_len);FFL_();
#endif
            	return ERROR;	// check valid cmd length
            }
#if 0
            // check if CAN controller is in reset mode
            if (CHECKBIT (CAN_flags, BUS_ON)){
#ifdef VERBOSE
            	xprintf("ERR-Bus is on.\n");
#endif
            	return ERROR;
            }

            // assign pointer to AMR or ACR values depending on command
            if (*cmd_buf_pntr == SET_AMR)
                tmp_pntr = CAN_init_val.amr;
            else
                tmp_pntr = CAN_init_val.acr;

            // store AMR or ACR values
            *tmp_pntr = ascii2byte (++cmd_buf_pntr);
            *tmp_pntr <<= 4;
            *(tmp_pntr++) |= ascii2byte (++cmd_buf_pntr);
            *tmp_pntr = ascii2byte (++cmd_buf_pntr);
            *tmp_pntr <<= 4;
            *(tmp_pntr++) |= ascii2byte (++cmd_buf_pntr);
            *tmp_pntr = ascii2byte (++cmd_buf_pntr);
            *tmp_pntr <<= 4;
            *(tmp_pntr++) |= ascii2byte (++cmd_buf_pntr);
            *tmp_pntr = ascii2byte (++cmd_buf_pntr);
            *tmp_pntr <<= 4;
            *tmp_pntr |= ascii2byte (++cmd_buf_pntr);
            // init CAN controller with new values
            return init_CAN ();
#endif
            return CR;
            // set bitrate via BTR
        case SET_BTR:
            // set fix bitrate
        case SET_BITRATE:
#if 0
            if ((cmd_len != 5) && (cmd_len != 2)){
#ifdef VERBOSE
            	xprintf("ERR-Wrong command length should be 2 or 5 but is %d.\r",cmd_len);
#endif
            	return ERROR;	// check valid cmd length
            }

            // check if CAN controller is in reset mode
            if (CHECKBIT (CAN_flags, BUS_ON)){
#ifdef VERBOSE
            	xprintf("ERR-Bus is on.\n");
#endif
            	return ERROR;
            }
            // store user or fixed bit rate
            if (*cmd_buf_pntr == SET_BTR) {
                CAN_init_val.btr0 = ascii2byte (++cmd_buf_pntr);
                CAN_init_val.btr0 <<= 4;
                CAN_init_val.btr0 |= ascii2byte (++cmd_buf_pntr);
                CAN_init_val.btr1 = ascii2byte (++cmd_buf_pntr);
                CAN_init_val.btr1 <<= 4;
                CAN_init_val.btr1 |= ascii2byte (++cmd_buf_pntr);
                CAN_init_val.fixed_rate = 0;
            }
            else {
                CAN_init_val.fixed_rate = *(++cmd_buf_pntr);
            }
            // init CAN controller
            SETBIT (CAN_flags, CAN_INIT);	// indicate initialized controller
            return init_CAN ();
#endif
            return CR;

            // open CAN channel
        case OPEN_CAN_CHAN:
#if 0
            // return error if controller is not initialized or already open
            if (!CHECKBIT (CAN_flags, CAN_INIT)){
#ifdef VERBOSE
            	xprintf("ERR-Bus has not been initialized.\n");
#endif
            	return ERROR;
            }

            // check if CAN controller is in reset mode
            if (CHECKBIT (CAN_flags, BUS_ON)){
#ifdef VERBOSE
            	xprintf("ERR-Bus is on.\n");
#endif
            	return ERROR;
            }

            // switch to oper mode
            do {
              #if defined(ENABLE_SELFTEST)
                  ModeControlReg = _BV (STM_Bit);
              #else
                  ModeControlReg &= ~_BV (RM_RR_Bit);
              #endif
            } while ((ModeControlReg & _BV (RM_RR_Bit)) == _BV (RM_RR_Bit));
            SETBIT (CAN_flags, BUS_ON);
#endif
            return CR;

            // close CAN channel
        case CLOSE_CAN_CHAN:
#if 0
            // check if CAN controller is in reset mode
            if (!CHECKBIT (CAN_flags, BUS_ON))
#ifdef USART_HELP_BUS
            {
            	xprintf("ERR-Bus is not on.\n");
            	return ERROR;
            }
#else
				return ERROR;
#endif
            // switch to reset mode
            do {		// do as long as RM_RR_Bit is not set
              #if defined(ENABLE_SELFTEST)
                  ModeControlReg = _BV (RM_RR_Bit) | _BV (STM_Bit);
              #else
                  ModeControlReg = _BV (RM_RR_Bit);
              #endif
            } while ((ModeControlReg & _BV (RM_RR_Bit)) != _BV (RM_RR_Bit));
            CLEARBIT (CAN_flags, BUS_ON);
#endif
            return CR;

            // send 11bit ID message
        case SEND_R11BIT_ID:
#if 0
            // check if CAN controller is in reset mode or busy
            if (!CHECKBIT (CAN_flags, BUS_ON) || CHECKBIT (CAN_flags, TX_BUSY))
                return ERROR;
            // check valid cmd length (only 5 bytes for RTR)
            if (cmd_len != 5)
                return ERROR;

            CAN_tx_msg.rtr = 1;	// remote transmission request

            // store std. frame format
            CAN_tx_msg.format = 0;
            // store ID
            CAN_tx_msg.id = ascii2byte (++cmd_buf_pntr);
            CAN_tx_msg.id <<= 4;
            CAN_tx_msg.id += ascii2byte (++cmd_buf_pntr);
            CAN_tx_msg.id <<= 4;
            CAN_tx_msg.id += ascii2byte (++cmd_buf_pntr);
            // store data length
            CAN_tx_msg.len = ascii2byte (++cmd_buf_pntr);

            // if transmit buffer was empty send message
            //return transmit_CAN ();
            if (CAN_SendMsg(LPC_CAN2, &TXMsg)==SUCCESS)
            	return CR;
            else
            	return ERROR;
#endif
        	return CR;

        case SEND_11BIT_ID:
#if 1
            // check if CAN controller is in reset mode or busy
            if (!CANBUS_ON()||!TX_BUSY()){
#ifdef VERBOSE
            	xprintf("ERR-Bus is not on or tx_busy.\n");FFL_();
#endif
            	return ERROR;
			}
#endif

            if ((cmd_len < 5) || (cmd_len > 21)){
#ifdef VERBOSE
            	xprintf("ERR-Wrong command length should be less than 5 or more than 21 but is %d.",cmd_len);FFL_();
#endif
            	return ERROR;	// check valid cmd length
			}

            TXMsg.type = DATA_FRAME;// no remote transmission request

            // store std. frame format
            TXMsg.format = STD_ID_FORMAT;

            // store ID
            TXMsg.id = ascii2byte (++cmd_buf_pntr);
            TXMsg.id <<= 4;
            TXMsg.id += ascii2byte (++cmd_buf_pntr);
            TXMsg.id <<= 4;
            TXMsg.id += ascii2byte (++cmd_buf_pntr);


            // store data length
            TXMsg.len = ascii2byte (++cmd_buf_pntr);

            // check number of data bytes supplied against data lenght byte
            if (TXMsg.len != ((cmd_len - 5) / 2)){
#ifdef VERBOSE
            	xprintf("ERR-Wrong length compared with actual TXMsg.len, is %d.\r",TXMsg.len);FFL_();
#endif
            	return ERROR;
			}

            // check for valid length
            //if (CAN_tx_msg.len > 8)
            if (TXMsg.len > 8){
#ifdef VERBOSE
            	xprintf("ERR-Length of TXMsg.len is too long, is %d.\r",TXMsg.len);FFL_();
#endif
                return ERROR;	// check valid length
			}
            else {		// store data
                // cmd_len is no longer needed, so we can use it as counter here
                for (cmd_len = 0; cmd_len < TXMsg.len; cmd_len++) {
                	if(cmd_len<4){
                		cmd_buf_pntr++;
                		TXMsg.dataA[cmd_len] = ascii2byte (cmd_buf_pntr);
                		TXMsg.dataA[cmd_len] <<= 4;
                		cmd_buf_pntr++;
                		TXMsg.dataA[cmd_len] += ascii2byte (cmd_buf_pntr);
                	}
                	else{//(cmd_len>4)
                		cmd_buf_pntr++;
                		TXMsg.dataB[cmd_len-4] = ascii2byte (cmd_buf_pntr);
                		TXMsg.dataB[cmd_len-4] <<= 4;
                		cmd_buf_pntr++;
                		TXMsg.dataB[cmd_len-4] += ascii2byte (cmd_buf_pntr);
                	}
                }

            }
            CAN_SetCommand(LPC_CAN2, CAN_CMR_SRR); //Self Reception request
            // if transmit buffer was empty send message
            if (CAN_SendMsg(LPC_CAN2, &TXMsg)==SUCCESS)
            	return CR;
            else
            	return ERROR;

            // send 29bit ID message
        case SEND_R29BIT_ID:
#if 0
            // check if CAN controller is in reset mode or busy
            if (!CHECKBIT (CAN_flags, BUS_ON) || CHECKBIT (CAN_flags, TX_BUSY))
                return ERROR;

            if (cmd_len != 10)
                return ERROR;	// check valid cmd length

            CAN_tx_msg.rtr = 1;	// remote transmission request

            // store ext. frame format
            CAN_tx_msg.format = 1;
            // store ID
            CAN_tx_msg.id = ascii2byte (++cmd_buf_pntr);
            CAN_tx_msg.id <<= 4;
            CAN_tx_msg.id += ascii2byte (++cmd_buf_pntr);
            CAN_tx_msg.id <<= 4;
            CAN_tx_msg.id += ascii2byte (++cmd_buf_pntr);
            CAN_tx_msg.id <<= 4;
            CAN_tx_msg.id += ascii2byte (++cmd_buf_pntr);
            CAN_tx_msg.id <<= 4;
            CAN_tx_msg.id += ascii2byte (++cmd_buf_pntr);
            CAN_tx_msg.id <<= 4;
            CAN_tx_msg.id += ascii2byte (++cmd_buf_pntr);
            CAN_tx_msg.id <<= 4;
            CAN_tx_msg.id += ascii2byte (++cmd_buf_pntr);
            CAN_tx_msg.id <<= 4;
            CAN_tx_msg.id += ascii2byte (++cmd_buf_pntr);
            // store data length
            CAN_tx_msg.len = ascii2byte (++cmd_buf_pntr);
            // if transmit buffer was empty send message
            return transmit_CAN ();
#endif
            return CR;

        case SEND_29BIT_ID:
#if 0
            // check if CAN controller is in reset mode or busy
            if (!CHECKBIT (CAN_flags, BUS_ON) || CHECKBIT (CAN_flags, TX_BUSY))
                return ERROR;

            if ((cmd_len < 10) || (cmd_len > 26))
                return ERROR;	// check valid cmd length

            CAN_tx_msg.rtr = 0;	// no remote transmission request

            // store ext. frame format
            CAN_tx_msg.format = 1;
            // store ID
            CAN_tx_msg.id = ascii2byte (++cmd_buf_pntr);
            CAN_tx_msg.id <<= 4;
            CAN_tx_msg.id += ascii2byte (++cmd_buf_pntr);
            CAN_tx_msg.id <<= 4;
            CAN_tx_msg.id += ascii2byte (++cmd_buf_pntr);
            CAN_tx_msg.id <<= 4;
            CAN_tx_msg.id += ascii2byte (++cmd_buf_pntr);
            CAN_tx_msg.id <<= 4;
            CAN_tx_msg.id += ascii2byte (++cmd_buf_pntr);
            CAN_tx_msg.id <<= 4;
            CAN_tx_msg.id += ascii2byte (++cmd_buf_pntr);
            CAN_tx_msg.id <<= 4;
            CAN_tx_msg.id += ascii2byte (++cmd_buf_pntr);
            CAN_tx_msg.id <<= 4;
            CAN_tx_msg.id += ascii2byte (++cmd_buf_pntr);
            // store data length
            CAN_tx_msg.len = ascii2byte (++cmd_buf_pntr);
            // check number of data bytes supplied against data lenght byte
            if (CAN_tx_msg.len != ((cmd_len - 10) / 2))
                return ERROR;

            // check for valid length
            if (CAN_tx_msg.len > 8)
                return ERROR;
            else {		// store data
                // cmd_len is no longer needed, so we can use it as counter here
                for (cmd_len = 0; cmd_len < CAN_tx_msg.len; cmd_len++) {
                    cmd_buf_pntr++;
                    CAN_tx_msg.data_bytes[cmd_len] = ascii2byte (cmd_buf_pntr);
                    CAN_tx_msg.data_bytes[cmd_len] <<= 4;
                    cmd_buf_pntr++;
                    CAN_tx_msg.data_bytes[cmd_len] += ascii2byte (cmd_buf_pntr);
                }
            }
            // if transmit buffer was empty send message
            return transmit_CAN ();
#endif
            return CR;

            // read Error Capture Register
            // read Arbitration Lost Register
        case READ_ECR:
        case READ_ALCR:
#if 0
            // check if CAN controller is in reset mode
            if (!CHECKBIT (CAN_flags, BUS_ON))
#ifdef USART_HELP_BUS
            {
            	xprintf("ERR-Bus is not on.\n");
            	return ERROR;
            }
#else
				return ERROR;
#endif

            if (*cmd_buf_pntr == READ_ECR) {
                comm_put(READ_ECR);
                usart_byte2ascii (last_ecr);
            }
            else {
                comm_put(READ_ALCR);
                usart_byte2ascii (last_alc);
            }
#endif
            return CR;

            // read SJA1000 register
        case READ_REG:
#if 0
			if (cmd_len != 3)
#ifdef USART_HELP_LENGTH
			{
				xprintf("ERR-Wrong command length should be 3 but is %d.\r",cmd_len);
            	return ERROR;
            }
#else
				return ERROR;	// check valid cmd length
#endif

            // cmd_len is no longer needed, so we can use it as buffer
            // get register number
            cmd_len = ascii2byte (++cmd_buf_pntr);
            cmd_len <<= 4;
            cmd_len |= ascii2byte (++cmd_buf_pntr);
            comm_put(READ_REG);
            usart_byte2ascii (read_CAN_reg (cmd_len));
#endif
            return CR;

            // write SJA1000 register
        case WRITE_REG:
#if 0
            if (cmd_len != 5)
#ifdef USART_HELP_LENGTH
			{
            	xprintf("ERR-Wrong command length should be 5 but is %d.\r",cmd_len);
            	return ERROR;
            }
#else
				return ERROR;	// check valid cmd length
#endif

            // cmd_len is no longer needed, so we can use it as buffer
            // get register number
            cmd_len = ascii2byte (++cmd_buf_pntr);
            cmd_len <<= 4;
            cmd_len |= ascii2byte (++cmd_buf_pntr);
            // get register data
            tmp_regdata = ascii2byte (++cmd_buf_pntr);
            tmp_regdata <<= 4;
            tmp_regdata |= ascii2byte (++cmd_buf_pntr);
            write_CAN_reg (cmd_len, tmp_regdata);
#endif
            return CR;

        case LISTEN_ONLY:
        	//CAN_ModeConfig(LPC_CAN2, CAN_LISTENONLY_MODE, ENABLE);
            return CR;

            // end with error on unknown commands
        default:
#ifdef VERBOSE
        	xprintf("ERR-Unrecognised command - %s\r",*cmd_buf_pntr);FFL_();
#endif
            return ERROR;
    }				// end switch

    // we should never reach this return
#ifdef VERBOSE
    xprintf("CAN INI ERR.\n");FFL_();
#endif
    return ERROR;
}				// end exec_usart_cmd


/*
**---------------------------------------------------------------------------
**
** Abstract: Convert 1 char ASCII to 1 low nibble binary
**
**
** Parameters: Pointer to ASCII char
**
**
** Returns: Byte value
**
**
**---------------------------------------------------------------------------
*/
uint8_t ascii2byte (uint8_t * val){
    uint8_t temp = *val;

    if (temp > 0x60)
        temp -= 0x27;		// convert chars a-f
    else if (temp > 0x40)
        temp -= 0x07;		// convert chars A-F
    temp -= 0x30;		// convert chars 0-9

    return temp & 0x0F;
}

// Main loop
void CAN_loop(void){
	uint8_t i;			// for loop counter
#if 0
	if (CHECKBIT(CAN_flags, MSG_WAITING)){
    	// check frame format
		if (RXMsg.format==STD_ID_FORMAT){
			// Standart Frame
			if(RXMsg.type==DATA_FRAME)
				comm_put(SEND_11BIT_ID);// send command tag
			else
				comm_put(SEND_R11BIT_ID);
			// send high byte of ID
			if (((RXMsg.id >> 8) & 0x0F) < 10)
				comm_put(((uint8_t) (RXMsg.id >> 8) & 0x0F) + 48);
			else
				comm_put(((uint8_t) (RXMsg.id >> 8) & 0x0F) + 55);
			// send low byte of ID
			xprintf("%x",(uint8_t) RXMsg.id & 0xFF);
		}
		else{
			// Extented Frame
			if (RXMsg.type==DATA_FRAME){
				comm_put(SEND_29BIT_ID);
			}		// send command tag
			else
				comm_put(SEND_R29BIT_ID);

			xprintf("%x",(uint8_t) (RXMsg.id >> 24) & 0xFF);
			xprintf("%x",(uint8_t) (RXMsg.id >> 16) & 0xFF);
			xprintf("%x",(uint8_t) (RXMsg.id >> 8) & 0xFF);
			xprintf("%x",(uint8_t) RXMsg.id & 0xFF);
		}
		// send data length code
		comm_put(RXMsg.len + '0');
		if (RXMsg.type==DATA_FRAME){
			// send data only if no remote frame request
			// send data bytes
			for (i = 0; i < RXMsg.len; i++){
				if(RXMsg.len<4)
					xprintf("%x",RXMsg.dataA[i]);
				else//RXMsg.len>4)
					xprintf("%x",RXMsg.dataB[i]);
			}
		}
		// send time stamp if required
		if (ram_timestamp_status != 0)
			xprintf("%x",sys_millis());

		// send end tag
		comm_put(CR);
		CLEARBIT(CAN_flags, MSG_WAITING);
	}
#else
	if (CHECKBIT(CAN_flags, MSG_WAITING)){

		unsigned char rec_121, RxScreenPacketNo;
//		CAN_rx_msg.Last_id = CAN_rx_msg.id;

		switch(RXMsg.id){
			case 0x121: // Screen data
				rec_121 = (RXMsg.dataA[0] & 0x0F);
				RxScreenPacketNo = rec_121*7;
				ScreenTempData[0+RxScreenPacketNo] = RXMsg.dataA[1];
				ScreenTempData[1+RxScreenPacketNo] = RXMsg.dataA[2];
				ScreenTempData[2+RxScreenPacketNo] = RXMsg.dataA[3];
				ScreenTempData[3+RxScreenPacketNo] = RXMsg.dataB[0];
				ScreenTempData[4+RxScreenPacketNo] = RXMsg.dataB[1];
				ScreenTempData[5+RxScreenPacketNo] = RXMsg.dataB[2];
				ScreenTempData[6+RxScreenPacketNo] = RXMsg.dataB[3];
//				if (rec_121==0){
//					RX_In_progress = 1;
//				}
				if ( ((rec_121 == 3)||(rec_121==4)) && (ScreenTempData[6+RxScreenPacketNo] == 0x81)){
					//RX_In_progress = 0;
					RecieveComplete++;
				}
			case 0x521: // Acknowledged screen data
//				Screen_tx.NoAck++;
				break;
			case 0x0A9: // Button Pressed
				ButtonTempData[0] = RXMsg.dataA[2];
				ButtonTempData[1] = RXMsg.dataA[3];
				ButtomPressed++;
				break;
			//Power off timeout
			case 0x3CF:
			case 0x3DF:
				OffTimeout=0xFF;
				break;
			default:
				return;
		}

		switch(RXMsg.id){
/*
stereo
0x0A9 0389(00)(01)(A2)A2A2A2 Source Right
0x0A9 0389(00)(02)(A2)A2A2A2 Source Left
0x0A9 0389(00)(03)(A2)A2A2A2 Volume up
0x0A9 0389(00)(04)(A2)A2A2A2 Volume down
0x0A9 0389(00)(05)(A2)A2A2A2 Pause
0x0A9 0389(00)(0A)(22)A2A2A2 Enter
0x0A9 0389(01)(41)(A2)A2A2A2 Track back
0x0A9 0389(01)(01)(A2)A2A2A2 Track next
0x4A9 appears to be ok sig

0x121 seems to be screen data
0x521 seems to be ok sig

0x3CF seems to be contantly polling
0x3DF seems to be ok sig

0x1C1 seems to be polling
0x5C1 seems to be ok sig

0x5B1 seems to be polling
0x1B1 seems to be ok sig

Send sreen data
0x4A9 7481818181818181
0x121 1019766001545220
0x521 300100A2A2A2A2A2
0x121 2130362043441043
0x521 300100A2A2A2A2A2
0x121 2244202020545220
0x521 300100A2A2A2A2A2
0x121 2330362020008181
0x521 74A2A2A2A2A2A2A2
*/
			case 0x5B1: // Ping recieved
			//case 0x1B1: // Ping recieved
				//exec_usart_cmd("t3DF87900818181818181");
				//exec_usart_cmd("t3CF86900A2A2A2A2A2A2");
				break;
			case 0x5C1: // Ping recieved
			//case 0x1C1: // Ping recieved
				//exec_usart_cmd("t3DF87900818181818181");
				//exec_usart_cmd("t3CF86900A2A2A2A2A2A2");
				break;
			case 0x521: // Ok recieved for screen data
				flags.recieved_521 = true;
				// send next screen data.
				break;
			case 0x3CF: // Ping recieved
			//case 0x3DF: // Ping recieved
				//exec_usart_cmd("t3DF87900818181818181");
				//exec_usart_cmd("t3CF86900A2A2A2A2A2A2");
				break;
			case 0x09A: // Remote pressed
				switch(RXMsg.dataA[2]){
					case 0x00:
						switch(RXMsg.dataA[3]){
							case 0x05:
								xprintf("cp");
								//pause
								break;
							case 0x03:
								xprintf("cv+");
								//vol up
								break;
							case 0x04:
								xprintf("cv-\r");
								//vol down
								break;
							case 0x01:
								xprintf("cs+\r");
								//Sou Right
								break;
							case 0x02:
								xprintf("cs-\r");
								flags.source_change = 1;
								//Sou Left
								break;
							case 0x0A:
								xprintf("ce\r");
								flags.source_change = 1;
								//Enter
								break;
						}
						break;
					case 0x01:
						switch(RXMsg.dataA[3]){
							case 0x41:
								xprintf("ct-\r");
								//Track back
								break;
							case 0x01:
								xprintf("ct+\r");
								//Track next
								break;
						}
						break;
				}
				//exec_usart_cmd("t49Aetc");
				break;
			case 0x49A: // Ok from remote recieved
				flags.recieved_49A = true;
				break;
			default:
				xprintf("ID not found.\r");
				break;
		}
		if(flags.source_change){
			source_selected++;
			if(source_selected == number_of_sources)
				source_selected = 1;
			switch(source_selected){
				case CD:
					xprintf("CD\r");
					break;
				case IPOD:
					xprintf("IPOD\r");
					break;
				case PC:
					xprintf("PC\r");
					break;
			}
			xprintf(" selected.\r");
			flags.source_change = 0;
		}
/*
		comm_put(CAN_rx_msg.len);
		for (i = 0; i < CAN_rx_msg.len; i++)
			usart_byte2ascii (CAN_rx_msg.data_bytes[i]);
		comm_put(CR);
*/
		CLEARBIT(CAN_flags, MSG_WAITING);
	}
#endif
}

void CAN_IRQHandler(){
	uint8_t IntStatus;
//	uint32_t data1;
	/* Get CAN status */
	IntStatus = CAN_GetCTRLStatus(LPC_CAN2, CANCTRL_STS);
	//check receive buffer status
	if((IntStatus>>0)&0x01){
		CAN_ReceiveMsg(LPC_CAN2,&RXMsg);
		xprintf("Received buffer:\r");
		PrintMessage(&RXMsg);
		SETBIT(CAN_flags, MSG_WAITING);
	}
}

void PrintMessage(CAN_MSG_Type* CAN_Msg){
	xprintf("Message ID:     %8x\r",CAN_Msg->id);
	xprintf("Message length: %8x BYTES\r",CAN_Msg->len);
	xprintf("Message type:   %s\r",CAN_Msg->type==DATA_FRAME ? "DATA FRAME\r" : "REMOTE FRAME\r");
	xprintf("Message format: %s\r",CAN_Msg->format==STD_ID_FORMAT ? "STANDARD ID FRAME FORMAT\r" : "EXTENDED ID FRAME FORMAT\r");
	xprintf("Message dataA:  %8x\r",(CAN_Msg->dataA[0])|(CAN_Msg->dataA[1]<<8)|(CAN_Msg->dataA[2]<<16)|(CAN_Msg->dataA[3]<<24));
	xprintf("Message dataB:  %8x\r",(CAN_Msg->dataB[0])|(CAN_Msg->dataB[1]<<8)|(CAN_Msg->dataB[2]<<16)|(CAN_Msg->dataB[3]<<24));
}
