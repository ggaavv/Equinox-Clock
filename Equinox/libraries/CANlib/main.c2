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
//#include <avr/signal.h>
#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/eeprom.h>
#include <string.h>
#include <ctype.h>
#include <stdint.h>
#include <avr/pgmspace.h>
#include <avr/wdt.h>
#include "config.h"
#include "main.h"
#include "unilink.h"
#include "unilink.c"
#include "usart.c"
#include "general.c"
#include "sja1000.c"

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
//int16_t can_ini (void)
void can_ini (void)
{
    // LED pins as output
    LED_DIR |= _BV (LED_RED) | _BV (LED_GREEN);

    // power up init for CAN controller with default values
    init_CAN ();

    // set wait states
    EMCUCR |= _BV(SRW11)|_BV(SRW10)|_BV(SRW01)|_BV(SRW00);

    // enables Bus-Keeper who will ensure a defined logic on AD7..0 when they would
    // otherwise be tristatet
    SFIOR |= _BV (XMBK);

    //PortC is completely used for AD8..15, as we don't use them, it's better do disable this
    SFIOR |= _BV (XMM2) | _BV (XMM1) | _BV (XMM0);

    // enable external RAM = SJA1000 interface
    // if the communication with the CAN doesn't work, please add wait-states
    MCUCR |= _BV (SRE);

    /* INT0 must be low level activated, see AN97076 page 38 */
    GICR |= _BV (INT0);

    // setup time stamp counter
    OCR0 = OCR_VALUE;		// timer0 capture at 1ms
    TCCR0 = _BV (WGM01) | _BV (CS01) | _BV (CS00);	// set Timer0 CTC mode, prescaler 64
    TIMSK = _BV (OCIE0);	// enable timer0 output compare interrupt
    timestamp = 0;

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
uint8_t exec_usart_cmd (uint8_t * cmd_buf)
{
    uint8_t cmd_len = strlen (cmd_buf);	// get command length //no_usart_rx_chars-1; //
    uint8_t *cmd_buf_pntr = &(*cmd_buf);	// point to start of received string
    cmd_buf_pntr++;		// skip command identifier
    // check if all chars are valid hex chars
    while (*cmd_buf_pntr) {
        if (!isxdigit (*cmd_buf_pntr))
        {
#ifdef USART_HELP
        	usart_puts("Not a valid asci char (exec_usart_cmd1).");
        	usart_byte2ascii(*cmd_buf_pntr);
#endif
            return ERROR;
		}
		++cmd_buf_pntr;
    }
#ifdef USART_HELP
	usart_puts("cmd_len - ");
	usart_byte2ascii(cmd_len);
	usart_putc(CR);
#endif

    cmd_buf_pntr = &(*cmd_buf);	// reset pointer

    uint8_t *tmp_pntr;		// temporary pointer for ACR/AMR
    uint8_t tmp_regdata;	// temporary used for register data

    switch (*cmd_buf_pntr) {
            // get serial number
        case GET_SERIAL:
            usart_putc (GET_SERIAL);
            usart_puts (SERIAL);
            return CR;

            // get hard- and software version
        case GET_VERSION:
            usart_putc (GET_VERSION);
            usart_byte2ascii (HW_VER);
            usart_byte2ascii (SW_VER);
            return CR;

            // get only software version
        case GET_SW_VERSION:
            usart_putc (GET_SW_VERSION);
            usart_byte2ascii (SW_VER_MAJOR);
            usart_byte2ascii (SW_VER_MINOR);
            return CR;

        	// toggle time stamp option
        case TIME_STAMP:
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
            return CR;

            // read status flag
        case READ_STATUS:
            // check if CAN controller is in reset mode
            if (!CHECKBIT (CAN_flags, BUS_ON))
#ifdef USART_HELP_BUS
            {
            	usart_puts("ERR-Bus is not on.\n");
            	return ERROR;
            }
#else
				return ERROR;
#endif

            usart_putc (READ_STATUS);
            usart_byte2ascii ((uint8_t) (CAN_flags >> 8));

            // turn off Bus Error indication
            LED_PORT &= ~_BV (LED_RED);
            // reset error flags
            CAN_flags &= 0x00FF;
            return CR;

            // set AMR
        case SET_AMR:
            // set ACR
        case SET_ACR:
            // check valid cmd length and if CAN was initialized before
            if (cmd_len != 9)
#ifdef USART_HELP_LENGTH
            {
            	usart_puts("ERR-Wrong command length should be 9 but is ");
            	usart_byte2ascii(cmd_len);
            	usart_putc('.');
            	usart_putc(CR);
            	return ERROR;	// check valid cmd length
            }
#else
				return ERROR;	// check valid cmd length
#endif

            // check if CAN controller is in reset mode
            if (CHECKBIT (CAN_flags, BUS_ON))
#ifdef USART_HELP_BUS
            {
            	usart_puts("ERR-Bus is on.\n");
            	return ERROR;
            }
#else
				return ERROR;
#endif

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

            // set bitrate via BTR
        case SET_BTR:
            // set fix bitrate
        case SET_BITRATE:
            if ((cmd_len != 5) && (cmd_len != 2))
#ifdef USART_HELP_LENGTH
            {
            	usart_puts("ERR-Wrong command length should be 2 or 5 but is ");
            	usart_byte2ascii(cmd_len);
            	usart_putc('.');
            	usart_putc(CR);
            	return ERROR;	// check valid cmd length
            }
#else
				return ERROR;	// check valid cmd length
#endif

            // check if CAN controller is in reset mode
            if (CHECKBIT (CAN_flags, BUS_ON))
#ifdef USART_HELP_BUS
            {
            	usart_puts("ERR-Bus is on.\n");
            	return ERROR;
            }
#else
				return ERROR;
#endif

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

            // open CAN channel
        case OPEN_CAN_CHAN:
            // return error if controller is not initialized or already open
            if (!CHECKBIT (CAN_flags, CAN_INIT))
#ifdef USART_HELP_BUS
			{
            	usart_puts("ERR-Bus has not been initialized.\n");
            	return ERROR;
            }
#else
				return ERROR;
#endif

            // check if CAN controller is in reset mode
            if (CHECKBIT (CAN_flags, BUS_ON))
#ifdef USART_HELP_BUS
			{
            	usart_puts("ERR-Bus is on.\n");
            	return ERROR;
            }
#else
				return ERROR;
#endif

            // switch to oper mode
            do {
              #if defined(ENABLE_SELFTEST)
                  ModeControlReg = _BV (STM_Bit);
              #else
                  ModeControlReg &= ~_BV (RM_RR_Bit);
              #endif
            } while ((ModeControlReg & _BV (RM_RR_Bit)) == _BV (RM_RR_Bit));
            SETBIT (CAN_flags, BUS_ON);
            return CR;

            // close CAN channel
        case CLOSE_CAN_CHAN:
            // check if CAN controller is in reset mode
            if (!CHECKBIT (CAN_flags, BUS_ON))
#ifdef USART_HELP_BUS
            {
            	usart_puts("ERR-Bus is not on.\n");
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
            return CR;

            // send 11bit ID message
        case SEND_R11BIT_ID:
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
            return transmit_CAN ();

        case SEND_11BIT_ID:
            // check if CAN controller is in reset mode or busy
            if (!CHECKBIT (CAN_flags, BUS_ON) || CHECKBIT (CAN_flags, TX_BUSY))
#ifdef USART_HELP_BUS
            {
            	usart_puts("ERR-Bus is not on or tx_busy.\n");
            	return ERROR;
			}
#else
				return ERROR;
#endif

            if ((cmd_len < 5) || (cmd_len > 21))
#ifdef USART_HELP_LENGTH
            {
            	usart_puts("ERR-Wrong command length should be less than 5 or more than 21 but is ");
            	usart_byte2ascii(cmd_len);
            	usart_putc('.');
            	usart_putc(CR);
            	return ERROR;	// check valid cmd length
			}
#else
                return ERROR;	// check valid cmd length
#endif

            CAN_tx_msg.rtr = 0;	// no remote transmission request

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
            // check number of data bytes supplied against data lenght byte
            if (CAN_tx_msg.len != ((cmd_len - 5) / 2))
#ifdef USART_HELP_LENGTH
            {
            	usart_puts("ERR-Wrong length compared with actual CAN_tx_msg.len, is ");
            	usart_byte2ascii(CAN_tx_msg.len);
            	usart_putc('.');
            	usart_putc(CR);
            	return ERROR;
			}
#else
                return ERROR;	// check valid length
#endif

            // check for valid length
            if (CAN_tx_msg.len > 8)
#ifdef USART_HELP_LENGTH
            {
            	usart_puts("ERR-Length of CAN_tx_msg.len is too long, is ");
            	usart_byte2ascii(CAN_tx_msg.len);
            	usart_putc('.');
            	usart_putc(CR);
                return ERROR;	// check valid length
			}
#else
				return ERROR;	// check valid length
#endif
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

            // send 29bit ID message
        case SEND_R29BIT_ID:
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

        case SEND_29BIT_ID:
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

            // read Error Capture Register
            // read Arbitration Lost Register
        case READ_ECR:
        case READ_ALCR:
            // check if CAN controller is in reset mode
            if (!CHECKBIT (CAN_flags, BUS_ON))
#ifdef USART_HELP_BUS
            {
            	usart_puts("ERR-Bus is not on.\n");
            	return ERROR;
            }
#else
				return ERROR;
#endif

            if (*cmd_buf_pntr == READ_ECR) {
                usart_putc (READ_ECR);
                usart_byte2ascii (last_ecr);
            }
            else {
                usart_putc (READ_ALCR);
                usart_byte2ascii (last_alc);
            }
            return CR;

            // read SJA1000 register
        case READ_REG:
			if (cmd_len != 3)
#ifdef USART_HELP_LENGTH
			{
            	usart_puts("ERR-Wrong command length should be 3 but is ");
            	usart_byte2ascii(cmd_len);
            	usart_putc('.');
            	usart_putc(CR);
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
            usart_putc (READ_REG);
            usart_byte2ascii (read_CAN_reg (cmd_len));
            return CR;

            // write SJA1000 register
        case WRITE_REG:
            if (cmd_len != 5)
#ifdef USART_HELP_LENGTH
			{
            	usart_puts("ERR-Wrong command length should be 5 but is ");
            	usart_byte2ascii(cmd_len);
            	usart_putc('.');
            	usart_putc(CR);
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
            return CR;

        case LISTEN_ONLY:
            // return error if controller is not initialized or already open
            if (!CHECKBIT (CAN_flags, CAN_INIT))
#ifdef USART_HELP_BUS
			{
            	usart_puts("ERR-Bus has not been initialized.\n");
            	return ERROR;
            }
#else
				return ERROR;	// check valid cmd length
#endif

            // check if CAN controller is in reset mode
            if (CHECKBIT (CAN_flags, BUS_ON))
#ifdef USART_HELP_BUS
            {
            	usart_puts("ERR-Bus is on.\n");
            	return ERROR;
            }
#else
				return ERROR;
#endif

            // switch to listen only mode
            do {
                ModeControlReg = _BV (LOM_Bit);
            } while ((ModeControlReg & _BV (RM_RR_Bit)) == _BV (RM_RR_Bit));
            SETBIT (CAN_flags, BUS_ON);
            return CR;

            // end with error on unknown commands
        default:
#ifndef CANUSB_COMPATIBLE
        	usart_puts("ERR-Unrecognised command - 0x");
        	usart_byte2ascii(*cmd_buf_pntr);
        	usart_putc(CR);
#endif
            return ERROR;
    }				// end switch

    // we should never reach this return
#ifndef CANUSB_COMPATIBLE
    usart_puts("CAN INI ERR.\n");
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
uint8_t ascii2byte (uint8_t * val)
{
    uint8_t temp = *val;

    if (temp > 0x60)
        temp -= 0x27;		// convert chars a-f
    else if (temp > 0x40)
        temp -= 0x07;		// convert chars A-F
    temp -= 0x30;		// convert chars 0-9

    return temp & 0x0F;
}


/*
**---------------------------------------------------------------------------
**
** Abstract: Timer0 output capture interrupt handler
**           called in 1ms intervall
**
**
** Parameters: none
**
**
** Returns: time stamp counter 0-59999ms
**
**---------------------------------------------------------------------------
*/


SIGNAL(TIMER0_COMP_vect)
{
    timestamp++;
    if (timestamp > 59999)
        timestamp = 0;
}


//#endif //CAN

// Main loop
int main(void)
{

	// command buffer (CAN)
	uint8_t cmd_buf[CMD_BUFFER_LENGTH];

	// command buffer index (CAN)
	uint8_t buf_ind = 0;
	uint8_t usart_rx_char;
	uint8_t i;			// for loop counter

	usart_init();                   	// init UART
	can_ini();
	unilink_ini();

	sei();                    	     	// enable interrupts

#ifdef nothing
	if(bit_is_set(PINB, b_SS)) // indicates bus is off (BUS_ON )
	{
		// wait for idle time between two packets, then start receiving
		while(bit_is_clear(TIFR, TOV0))                             // wait for MOSI high for minimum of 6ms
		{
   			if (bit_is_clear(PINB, b_MOSI)) TCNT0 = c_6ms;		    		// load Timer0 if MOSI is not high
		};// wait until timer0 overflow
		TIFR |= _BV(TOV0);                                         	// clear timer0 overflow flag
		TCNT0 = c_6ms;		                                        	// load Timer0
		while(bit_is_clear(TIFR, TOV0))                             // wait for MOSI low for minimum of 6ms
		{
			if (bit_is_set(PINB, b_MOSI)) TCNT0 = c_6ms;		    			// load Timer0  if MOSI is not low
		};// wait until timer overflow
		TCNT0 = c_Timer0_stop;		    															// stop timer0
	}
	else
	{
		flags.unilink_on_off = OFF;
		usart_puts("Unilink off.\n");
	}
#endif

#ifndef CANUSB_COMPATIBLE
	can_connect_macro ();
#endif

	// now we are going in end endless loop waiting for packets
	while(true)
	{
#if defined WATCHDOG
			if(flags.bus_sleep)
				wdt_reset();												// reset watchdog here only if bus is in sleep mode
#endif
		if(flags.unilink_rx_compl)
		{
			flags.unilink_rx_compl = false;				// clear RX complete flag, wait for new packet
			checksum_check();

	    	if ( flags.checksum_ok )								// do a parity check of received packet and proceed if OK
			{
				unilink_parse();
#ifdef BUS_LOGGING
				flags.unilink_tx_log = false;			// disable Tx packet logging
				bus_logging();                      // send valid Rx packet via UART
#endif
			}  // end of routine if checksum was ok
		}
		if(flags.usart_recieve_complete&&!flags.can_usart_recieve_complete)
		{
			usart_rx_proc();
			flags.usart_recieve_complete = false;
		}

#define mikecan
#ifdef mikecan
		if (CHECKBIT (CAN_flags, MSG_WAITING))
		{
    		// check frame format
			if (!CAN_rx_msg.format)
			{	// Standart Frame
				if (!CAN_rx_msg.rtr)
				{
					usart_putc (SEND_11BIT_ID);
				}		// send command tag
				else
				{
					usart_putc (SEND_R11BIT_ID);
				}
				// send high byte of ID
				if (((CAN_rx_msg.id >> 8) & 0x0F) < 10)
					usart_putc (((uint8_t) (CAN_rx_msg.id >> 8) & 0x0F) + 48);
				else
					usart_putc (((uint8_t) (CAN_rx_msg.id >> 8) & 0x0F) + 55);
				// send low byte of ID
				usart_byte2ascii ((uint8_t) CAN_rx_msg.id & 0xFF);
			}
			else
			{		// Extented Frame
				if (!CAN_rx_msg.rtr)
				{
					usart_putc (SEND_29BIT_ID);
				}		// send command tag
				else
				{
					usart_putc (SEND_R29BIT_ID);
				}
				// send ID bytes
				usart_byte2ascii ((uint8_t) (CAN_rx_msg.id >> 24) & 0xFF);
				usart_byte2ascii ((uint8_t) (CAN_rx_msg.id >> 16) & 0xFF);
				usart_byte2ascii ((uint8_t) (CAN_rx_msg.id >> 8) & 0xFF);
				usart_byte2ascii ((uint8_t) CAN_rx_msg.id & 0xFF);
			}
			// send data length code
			usart_putc (CAN_rx_msg.len + '0');
			if (!CAN_rx_msg.rtr)
			{	// send data only if no remote frame request
				// send data bytes
				for (i = 0; i < CAN_rx_msg.len; i++)
				usart_byte2ascii (CAN_rx_msg.data_bytes[i]);
			}
			// send time stamp if required
			if (ram_timestamp_status != 0)
			{
				usart_byte2ascii ((uint8_t) (timestamp >> 8));
				usart_byte2ascii ((uint8_t) timestamp);
			}
			// send end tag
			usart_putc (CR);
			CLEARBIT (CAN_flags, MSG_WAITING);
		}
#endif
#ifndef mikecan
		if (CHECKBIT (CAN_flags, MSG_WAITING))
		{
			switch(CAN_rx_msg.id)
			{
/*
Stereo
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
					switch(CAN_rx_msg.data_bytes[2])
					{
						case 0x00:
							switch(CAN_rx_msg.data_bytes[3])
							{
								case 0x05:
									usart_puts("cp");
									//pause
									break;
								case 0x03:
									usart_puts("cv+");
									//vol up
									break;
								case 0x04:
									usart_puts("cv-");
									//vol down
									break;
								case 0x01:
									usart_puts("cs+");
									//Sou Right
									break;
								case 0x02:
									usart_puts("cs-");
									flags.source_change = 1;
									//Sou Left
									break;
								case 0x0A:
									usart_puts("ce");
									flags.source_change = 1;
									//Enter
									break;
							}
							break;
						case 0x01:
							switch(CAN_rx_msg.data_bytes[3])
							{
								case 0x41:
									usart_puts("ct-");
									//Track back
									break;
								case 0x01:
									usart_puts("ct+");
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
					usart_puts("ID not found.");
					break;
			}
			usart_putc(CR);
			if(flags.source_change)
			{
				source_selected++;
				if(source_selected == number_of_sources)
					source_selected = 1;
				switch(source_selected)
				{
					case CD:
						usart_puts("CD");
						break;
					case IPOD:
						usart_puts("IPOD");
						break;
					case PC:
						usart_puts("PC");
						break;
				}
				usart_puts(" selected.");
				flags.source_change = 0;
			}
/*
			usart_putc (CAN_rx_msg.len);
			for (i = 0; i < CAN_rx_msg.len; i++)
				usart_byte2ascii (CAN_rx_msg.data_bytes[i]);
			usart_putc (CR);
*/
			CLEARBIT (CAN_flags, MSG_WAITING);
		}
#endif
		// read char from usart
		if (flags.can_usart_recieve_complete)
		{
//			usart_puts("Recieved chars - ");
			unsigned char temp_count;
#ifdef CANUSB_COMPATIBLE
			for(temp_count=1; temp_count<=(no_usart_rx_chars); temp_count++)
#else
			for(temp_count=1; temp_count<=(no_usart_rx_chars+1); temp_count++)
#endif
			{
    	        buf_ind = 0;	// point to start of command
#ifdef CANUSB_COMPATIBLE
				usart_rx_char = usart_rx_buffer[temp_count-1];
#else
				usart_rx_char = usart_rx_buffer[temp_count];
#endif
//				usart_putc(usart_rx_buffer[temp_count]);
				if (usart_rx_char == CR)	// check for end of command
				{
#ifdef CANUSB_COMPATIBLE
					if(usart_rx_buffer[0]==CR)
					{
						usart_putc(CR);
						no_usart_rx_chars = 0;
						flags.can_usart_recieve_complete = false;
					}
					else
					{
						//					usart_putc(usart_rx_buffer[temp_count]);
						cmd_buf[temp_count-1] = 0x00;
						// Execute USB command and return status to terminal
						usart_putc (exec_usart_cmd (cmd_buf));//while(1);
						no_usart_rx_chars = 0;
						flags.can_usart_recieve_complete = false;
					}
#else
//					usart_putc(usart_rx_buffer[temp_count]);
					cmd_buf[temp_count-1] = 0x00;
					// Execute USB command and return status to terminal
					usart_putc (exec_usart_cmd (cmd_buf));//while(1);
					no_usart_rx_chars = 0;
					flags.can_usart_recieve_complete = false;
#endif
				}
				else // store new char in buffer
				{
					cmd_buf[temp_count-1] = usart_rx_char;	// store char
					// check for buffer overflow
					if (buf_ind < sizeof (cmd_buf) - 1)
						buf_ind++;
				}
			}
		}
	}; // end of endless while loop
}; // end of main()


// Timer1 Overflow interrupt
SIGNAL(TIMER1_OVF_vect)
{
//	usart_putc('d');
	flags.timer1_ovf = true;							// set overflow flag
	TCNT1 = c_1s;           							// restart Timer1
#if defined(WATCHDOG)
	unilink_missed_pings++;								/* increment missed ping counter every second,
                                           the counter is reset on each ping request
														               from head unit */
	if(unilink_missed_pings < 10)
	{
		wdt_reset();
	}
#endif
};// end of TIMER1 overflow interrupt
