

#ifndef CANLIB_H
#define CANLIB_H

#include "lpc17xx_can.h"


#define HW_VER        0x99		// hardware version
#define SW_VER        0x99		// software version
#define SW_VER_MAJOR  0x01    // software major version
#define SW_VER_MINOR  0x06    // software minor version
#define SERIAL        "0001"	// device serial number

#if !defined(CR)
#define CR            13	// command end tag (ASCII CR)
#endif

#if !defined(ERROR)
#define ERROR         'e'//7		// error tag (ASCII BEL)
#endif

#define false 0
#define true 1

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

#define COMMANDS \
"SET_BITRATE     S	 set CAN bit rate\n" \
"SET_BTR         s	 set CAN bit rate via\n" \
"OPEN_CAN_CHAN   O	 open CAN channel\n" \
"CLOSE_CAN_CHAN  C	 close CAN channel\n" \
"SEND_11BIT_ID   t	 send CAN message with 11bit ID\n" \
"SEND_29BIT_ID   T	 send CAN message with 29bit ID\n" \
"SEND_R11BIT_ID  r	 send CAN remote message with 11bit ID\n" \
"SEND_R29BIT_ID  R	 send CAN remote message with 29bit ID\n" \
"READ_STATUS     F	 read status flag byte\n" \
"SET_ACR         M	 set Acceptance Code Register\n" \
"SET_AMR         m	 set Acceptance Mask Register\n" \
"GET_VERSION     V	 get hardware and software version\n" \
"GET_SW_VERSION  v	 get software version only\n" \
"GET_SERIAL      N	 get device serial number\n" \
"TIME_STAMP      Z	 toggle time stamp setting\n" \
"READ_ECR        E	 read Error Capture Register\n" \
"READ_ALCR       A	 read Arbritation Lost Capture Register\n" \
"READ_REG        G	 read register conten from SJA1000\n" \
"WRITE_REG       W	 write register content to SJA1000\n" \
"LISTEN_ONLY     L	 switch to listen only mode\n"



// define bit macros
#define SETBIT(x,y) (x |= (y))	// Set bit y in byte x
#define CLEARBIT(x,y) (x &= (~y))	// Clear bit y in byte x
#define CHECKBIT(x,y) (x & (y))	// Check bit y in byte x

// define local CAN status flags
#define CAN_INIT          0x0001	// set if CAN controller is initalized
#define MSG_WAITING       0x0002	// set if Rx message is waiting

//register bits
#define BUS_ON_REG            (1<<4)	// set if CAN controller is in oper mode
#define TX_BUSY_REG           (1<<5)	// set if transmit is in progress

#define CANBUS_ON() (!(CAN_GetCTRLStatus(LPC_CAN2, CANCTRL_GLOBAL_STS)&(BUS_ON_REG))) //0 for Bus-On
#define TX_BUSY() (!(CAN_GetCTRLStatus(LPC_CAN2, CANCTRL_GLOBAL_STS)&(TX_BUSY_REG))) //0 for idle


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

volatile uint16_t CAN_flags;	// diverse CAN flags
volatile uint8_t CAN_MSG_RXED;


unsigned char source_selected;
#define CD 1
#define IPOD 2
#define PC 3
#define number_of_sources 4 // number of sources + 1

//*** Global variables ***
volatile struct {                   // use a bit field as flag store
unsigned char usart_recieve_complete :	1 ; // signals a send of completed packet
unsigned char can_usart_recieve_complete:1;
unsigned char release_key_required :	1 ; // used to release a key
unsigned char pressed_key_required :	1 ; // used to press a key
unsigned char source_change :			1 ; // source changed
unsigned char recieved_521 :			1 ; // ok from 521 recieved (screen)
unsigned char recieved_49A :			1 ; // ok from 49A recieved (remote)
} flags;                            // declare as flag byte





void CAN_init (void);
uint8_t exec_usart_cmd (uint8_t * cmd_buf);
uint8_t ascii2byte (uint8_t * val);
void CAN_loop(void);
void CAN_IRQHandler();
void PrintMessage(CAN_MSG_Type* CAN_Msg);// define EEPROM settings
/*
__attribute__ ((section (".eeprom")))
     uint8_t serial[] = SERIAL;	// store device serial
__attribute__ ((section (".eeprom")))
     uint8_t ee_timestamp_status = 0;	// store time stamp OFF
*/
// copy of ee_time_stamp_status in SRAM to prevent unnecessary EEPROM access
     uint8_t ram_timestamp_status;






#endif /* CANLIB_H */
