/*************************************************************************
**  Sony Unilink Interface (Main Header File)
**  by Michael Wolf
**
**  Released under GNU GENERAL PUBLIC LICENSE
**	See LICENSE.TXT for details
**
**  Revision History
**
**  when         what  who	why
**
**  2003-08-05   2.01  MIC  complete code revision
**  2003-08-06	 2.02  MIC  + added bus sleep flag
**  2003-08-21   2.02  MIC  + added unilink_appoint flag
**  2004-01-05   2.03  MIC  + added unilink_list array and
**														unilink_listcount for list support
**                          + added #defines for data bytes in Unilink RX
**														string
**													+ added playmode variables
**  2004-08-15	 2.04  MIC  - bitfrig() and respond_masterpoll() removed
**													+ added SPI config byte for new op-amp hardware
** ln -s /dev/ttyUSB0 ~/.wine/dosdevices/com1
**************************************************************************/

#define TRUE 1
#define FALSE 0
#define OFF 0
#define ON 1
// Check if target is supported
#if !defined(__AVR_AT90S8515__) && !defined(__AVR_ATmega8515__) && !defined(__AVR_ATmega8__)
//jc	#error Target not supported!
#endif

// Check if MCU frequency is supported
#if MCU_XTAL < 6000000
	#warning MCU clock frequency to low!
#elif MCU_XTAL > 16000000
	#warning MCU clock frequency to high!
#elif (MCU_XTAL > 8000000) && defined(__AVR_AT90S8515__)
	#warning MCU clock frequency to high for AT90S8515!
#endif

// Set correct timer prescaler dependent on MCU frequency
#if (MCU_XTAL >= 1000000) && (MCU_XTAL <= 2000000)
	#define PRESCALER 64
	#define c_Timer0_run 3              // run Timer0 with clk/64
	#define c_Timer1_run 3              // run Timer1 with clk/64
#elif (MCU_XTAL > 2000000) && (MCU_XTAL <= 8000000)
	#define PRESCALER 256
	#define c_Timer0_run 4              // run Timer0 with clk/256
	#define c_Timer1_run 4              // run Timer1 with clk/256
#elif (MCU_XTAL > 8000000) && (MCU_XTAL <= 16000000)
	#define PRESCALER 1024
	#define c_Timer0_run 5              // run Timer0 with clk/1024
	#define c_Timer1_run 5              // run Timer1 with clk/1024
#else
	#error MCU clock frequency not supported by Timer0 or Timer1!
#endif

// Check for maximum Baudrate
#if BAUD_RATE > 115200
	#error Selected Baudrate to high!
#endif

// Check for low Baudrate and Bus logging
#if defined(BUS_LOGGING) && (BAUD_RATE < 38400)
//jc	#error Baudrate to low for Bus logging!
#endif

// disable LED's and relais if target is not ATmega8
#if !defined(__AVR_ATmega8__)
	#undef LED_OUT
	#undef RELAIS_OUT
#endif

// disable Yampp3 control if bus logging is enabled and power control via interface is disabled
#if defined(BUS_LOGGING) || !defined(RELAIS_OUT)
	#undef YAMPP3
#endif

#define UART_BAUDRATE   (((MCU_XTAL / (BAUD_RATE * 16UL))) - 1) 	// calculate baud rate value for UBBR
#define RX_BUFFER_SIZE      50      // size of Rx buffer
#ifdef OPAMP_HW
	#define c_RUN_SPI					0xcc    // SPI config byte
#else
	#define c_RUN_SPI					0xc4		// SPI config byte
#endif
#define c_Timer0_stop 			0       // stop Timer0
#define c_Timer1_stop 			0       // stop Timer1

// Calculate delay (256 - TIME_ms * (CLOCK / PRESCALER) / 1000UL)
#define c_2ms ((unsigned char)(256 - 2 * (MCU_XTAL / PRESCALER) / 1000UL))
#define c_3ms ((unsigned char)(256 - 3 * (MCU_XTAL / PRESCALER) / 1000UL))
#define c_4ms ((unsigned char)(256 - 4 * (MCU_XTAL / PRESCALER) / 1000UL))
#define c_6ms ((unsigned char)(256 - 6 * (MCU_XTAL / PRESCALER) / 1000UL))
#define c_7ms ((unsigned char)(256 - 7 * (MCU_XTAL / PRESCALER) / 1000UL))
#define c_8ms ((unsigned char)(256 - 8 * (MCU_XTAL / PRESCALER) / 1000UL))
#define c_1s  ((unsigned int)(256 - 1000 * (MCU_XTAL / PRESCALER) / 1000UL))

#if defined(__AVR_ATmega8__)
// Pin definition for ATmega8
#define b_SS        2               // Pin SS
#define b_MOSI      3               // Pin MOSI
#define b_MISO      4               // Pin MISO
#define b_CLK       5               // Pin CLK
#define b_JUMPER    0               // Pin CD/MD mode jumper

#define d_RELAIS	7				// Power relais output
#define d_LED1		2				// LED 1
#define d_LED2		3				// LED 2
#define d_LED3		4				// LED 3
#endif

#if defined(__AVR_ATmega162__)
// Pin definition for ATmega162
// ATmega162
#define b_SS        4               // Pin SS
#define b_MOSI      5               // Pin MOSI
#define b_MISO      6               // Pin MISO
#define b_CLK       7               // Pin CLK
#define b_JUMPER    0               // Pin CD/MD mode jumper
#endif

#if defined(__AVR_AT90S8515__)
// Pin definition for AT90S8515 and ATmega8515
// ATmega162
#define b_SS        4               // Pin SS
#define b_MOSI      5               // Pin MOSI
#define b_MISO      6               // Pin MISO
#define b_CLK       7               // Pin CLK
#define b_JUMPER    0               // Pin CD/MD mode jumper
#endif

#define true		1
#define false		0

// byte in packet array locations
#define destaddr    0               // destination address
#define srcaddr     1               // source address
#define cmd1        2               // command 1
#define cmd2        3               // command 2
#define parity1     4               // parity 1
#define data        5               // start of data
#define d1			5				// data byte 1
#define d2			6				// data byte 2
#define d3			7				// data byte 3
#define d4			8				// data byte 4
#define d5			9				// data byte 5
#define d6			10				// data byte 6
#define d7			11				// data byte 7
#define d8			12				// data byte 8
#define d9			13				// data byte 9

//*** Global variables ***
volatile struct {                   // use a bit field as flag store
unsigned char checksum_ok :				1 ;	// set if Unilink packet is valid
unsigned char spi_tx:					1 ; // set when SPI is in Tx mode
unsigned char unilink_rx_compl :		1 ; // set when Unilink packet is complete
unsigned char unilink_tx_log :			1 ; // set if Unilink packet is transmitted, used in logging routine
unsigned char timepoll_req :			1 ; // indicates a requested time poll
unsigned char timer1_ovf :				1 ;	// timer 1 overflow occured
unsigned char interface_mode :			1 ;	// set if device is in MD mode, cleared if in CD mode
unsigned char bus_sleep :				1 ; // set if bus is in sleep mode
unsigned char unilink_anyone :			1 ; // set on respond to anyone
unsigned char usart_recieve_complete :	1 ; // signals a send of completed packet
unsigned char can_usart_recieve_complete:1;
unsigned char release_key_required :	1 ; // used to release a key
unsigned char pressed_key_required :	1 ; // used to press a key
unsigned char seeking :					1 ; // used to press a key
unsigned char my_input_selected :		1 ; // in MD/CD mode
unsigned char source_change :			1 ; // source changed
unsigned char unilink_on_off :			1 ; // defaults to can only if bus is off
unsigned char recieved_521 :			1 ; // ok from 521 recieved (screen)
unsigned char recieved_49A :			1 ; // ok from 49A recieved (remote)
} flags;                            // declare as flag byte

volatile struct {
unsigned char discname :			1 ;	// set if discname update is needed
unsigned char trackname :    		1 ;	// set if trackname update is needed
unsigned char time :         		1 ;	// set if time info update is needed
unsigned char playmode :        	1 ; // set if playmode info update is needed
} update_display;

volatile struct {                   // use a bit field to store play mode flags
unsigned char repeat  : 2 ;         /* xxxxxx00 Repeat off
																				 xxxxxx01 Repeat 1
																				 xxxxxx10 Repeat 2
																				 xxxxxx11 Repeat 3 */
unsigned char shuffle : 2 ;         /* xxxx00xx Shuffle off
																				 xxxx01xx Shuffle 1
																				 xxxx10xx Shuffle 2
																				 xxxx11xx Shuffle 3 */
unsigned char intro   : 1 ;         // xxx1xxxx Intro on/off
unsigned char dscan   : 1 ;         // xx1xxxxx D.Scan on/off
unsigned char bank    : 2 ;         /* 00xxxxxx Bank off
																				 01xxxxxx Bank on
																				 10xxxxxx Bank inverse */
} unilink_playmodes;

volatile struct {
unsigned char rx: 5;			    // counter for received bytes
unsigned char tx: 5;			    // counter for transmitted bytes
} unilink_bytecount;

volatile struct {
unsigned char rx: 5;			    // counter for received bytes
unsigned char tx: 5;			    // counter for transmitted bytes
} unilink_bytecount_rx;

struct {
unsigned char disc;						// holds actual field for disc name
unsigned char track;						// holds actual field for track name
} unilink_field;

unsigned char unilink_missed_pings = 0;		// count missed ping
unsigned char unilink_ownaddr = 0;			// holds the actual ID
unsigned char unilink_groupid = 0;			// holds current group ID
unsigned char unilink_rxsize =  0;			// receive packet size in bytes
unsigned char unilink_rxsize_rx =  0;			// receive packet size in bytes
unsigned char unilink_txsize =  0;			// transmit packet size in bytes
unsigned char unilink_timeinfo[8] = {0x70, 0x00, 0x90, 0x50, 0x01, 0x00, 0x00, 0x1E}; // init timeinfo string; Disc 1 Track 1 0:00
unsigned char unilink_status = 0x80;		// initial interface status (Idle)
unsigned char unilink_rxdata[21];			// holds the received packet
unsigned char unilink_rxdata_rx[21];			// holds the received packet
unsigned char unilink_txdata[21];			// holds the transmit packet
unsigned char unilink_slaveserial = 0;		// holds slave serial for poll request
unsigned char unilink_pm_d1 = 0;			// holds value for playmode command
unsigned char unilink_pm_d2 = 0;			// holds value for playmode command
unsigned char unilink_pm_d3 = 0;			// holds value for playmode command

unsigned char unilink_discname[48];			// holds disc name
unsigned char unilink_trackname[48];		// holds track name

unsigned char unilink_listcount = 0;		// used to send all 10 list items to HU
unsigned char unilink_list[10][8];			// holds the list items

unsigned char usart_rx_buffer[RX_BUFFER_SIZE+1];// UART Rx buffer
unsigned char usart_rx_index = 0;				// index of next char to be put into the buffer

unsigned char release_key_packet[4];		// buffer to store release key
unsigned char pressed_key_packet[4];		// buffer to store pressed key
volatile unsigned char no_usart_rx_chars;

//unsigned char disp_mode;							// display mode (changed with DISP button)

// time stamp counter 0-59999ms
static uint16_t timestamp;

// define EEPROM settings
/*
__attribute__ ((section (".eeprom")))
     uint8_t serial[] = SERIAL;	// store device serial
__attribute__ ((section (".eeprom")))
     uint8_t ee_timestamp_status = 0;	// store time stamp OFF
*/
// copy of ee_time_stamp_status in SRAM to prevent unnecessary EEPROM access
     uint8_t ram_timestamp_status;

// execute command received via USB
     uint8_t exec_usart_cmd (uint8_t * cmd_buf);

// convert 2 byte ASCII to one byte
     uint8_t ascii2byte (uint8_t * val);

#ifndef __MAIN_H__
#define __MAIN_H__

#define HW_VER        0x99		// hardware version
#define SW_VER        0x99		// software version
#define SW_VER_MAJOR  0x01    // software major version
#define SW_VER_MINOR  0x06    // software minor version
#define SERIAL        "0001"	// device serial number

#if !defined(CR)
#define CR            13	// command end tag (ASCII CR)
#endif

#if !defined(ERROR)
#define ERROR         7		// error tag (ASCII BEL)
#endif

#define SET_BITRATE     'S'	// set CAN bit rate
#define SET_BTR         's'	// set CAN bit rate via
#define OPEN_CAN_CHAN   'O'	// open CAN channel
#define CLOSE_CAN_CHAN  'C'	// close CAN channel
#define SEND_11BIT_ID   't'	// send CAN message with 11bit ID
#define SEND_29BIT_ID   'T'	// send CAN message with 29bit ID
#define SEND_R11BIT_ID  'r'	// send CAN remote message with 11bit ID
#define SEND_R29BIT_ID  'R'	// send CAN remote message with 29bit ID
#define READ_STATUS     'F'	// read status flag byte
#define SET_ACR         'M'	// set Acceptance Code Register
#define SET_AMR         'm'	// set Acceptance Mask Register
#define GET_VERSION     'V'	// get hardware and software version
#define GET_SW_VERSION  'v' // get software version only
#define GET_SERIAL      'N'	// get device serial number
#define TIME_STAMP      'Z'	// toggle time stamp setting
#define READ_ECR        'E'	// read Error Capture Register
#define READ_ALCR       'A'	// read Arbritation Lost Capture Register
#define READ_REG        'G'	// read register conten from SJA1000
#define WRITE_REG       'W'	// write register content to SJA1000
#define LISTEN_ONLY     'L'	// switch to listen only mode

#define TIME_STAMP_TICK 1000	// microseconds

/*
	define command receive buffer length
	minimum length is define as follow:
	1 byte	: command identifier
	8 byte	: CAN identifier (for both 11bit and 29bit ID)
	1 byte	: CAN data length (0-8 byte)
	2*8 byte: CAN data (one data byte is send as two ASCII chars)
	1 byte  : [CR] command end tag
	---
	27 byte
*/
#define CMD_BUFFER_LENGTH  30

// calculate timer0 overflow value
#define OCR_VALUE ((unsigned char)((unsigned long)(TIME_STAMP_TICK) / (1000000L / (float)((unsigned long)MCU_XTAL / 64L))))

#endif // __MAIN_H__

unsigned char source_selected;
#define CD 1
#define IPOD 2
#define PC 3
#define number_of_sources 4 // number of sources + 1

//*** Function prototype declaration ***
void usart_init(void);              // init UART
void reset_spi(void);               // reset SPI after interrupt
void checksum_check(void); 			// does a checksum verification on received packet
void bus_logging(void);             // send valid received packed via UART
void usart_putc(unsigned char byte);// send one byte via UART
void unilink_tx(unsigned char *msg);// send unilink packet
void usart_rx_proc(void);           // process received UART message
void slavebreak(void);              // do a slave break, force request poll
void unilink_parse(void);	        // Unilink command evaluation
void unilink_broadcast(void);       // do broadcast commands
void unilink_myid_cmd(void);        // do commands for my actual ID
void unilink_appoint(void);	        // do appoint respond
void repeat_mode(void);             // do play mode change
void shuffle_mode(void);            // do shuffle mode change
void intro_mode(void);              // do intro mode change
void dscan_mode(void);              // do d.scan mode change
void bank_mode(void);               // do bank mode change
void change_mode(unsigned char loc_d1, unsigned char loc_d2, unsigned char loc_d3); // send the changed play mode to display
void display_update(void);          // do a display update if necessary
unsigned char bcd2hex(unsigned char bcd); // convert 1 byte BCD to 1 byte HEX
unsigned char bin2ascii(unsigned char bin); // convert lower nibble of 1 byte binary to ASCII
void unilink_screenid_cmd(void);
//void usart_puts (uint8_t *tx_string);
void usart_puts (char *tx_string);
