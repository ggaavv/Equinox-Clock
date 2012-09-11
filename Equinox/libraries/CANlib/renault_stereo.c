/*************************************************************************
**  Renault Stereo Interface
**  by Jamie Clarke
**
**  Released under GNU GENERAL PUBLIC LICENSE
**	See LICENSE.TXT for details
**
**  Revision History
**
**  when         what  who	why
**
**  2009-08-23   1.00  JC  Initial code revision
**  2009-09-05   1.01  JC  + slightly modified for CAN
**  2009-09-05   2.00  JC  Send now waits for reply packet
**
**************************************************************************/

#include "lpc17xx_can.h"
#include "renault_stereo.h"
#include "canlib.h"
#include "comm.h"
#include "sys_timer.h"

unsigned char source_selected;
uint8_t current_key_keyboard=0;
#define min_key_keyboard 0x31 // Beginning of special
//#define min_key_keyboard 0x41 // Upper Case start
//#define max_key_keyboard 0x7F // Lower Case End
#define max_key_keyboard 0x5A // Upper Case End

unsigned char CompleteLength;
unsigned char ScreenTextStart;
unsigned char last_key;

//unsigned char ScreenLitData[12];
//unsigned char ScreenAsciiData[34];
//unsigned char ScreenSendPackets[5][8];
unsigned char PrevScreenText[10]={'P','H','O','N','E'};
unsigned char Source;
volatile unsigned int ScreenTimeoutCounter;
volatile unsigned int OffTimeout;
volatile unsigned char ScreenTempData[36];
volatile unsigned char RecieveComplete;
volatile unsigned char RX_In_progress;
volatile unsigned char ButtomPressed;
volatile unsigned char source_change;
volatile unsigned char ButtonTempData[2];
volatile unsigned char WaitingForScreen;
volatile unsigned char FirstScreen;
volatile uint32_t ButtonTempFull;


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


extern volatile int32_t DISPLAY_TIMEOUT;
extern volatile uint32_t OFF_TIMEOUT;

extern CAN_MSG_Type TXMsg, RXMsg; // messages for test Bypass mode

void ScreenTimeout_init ( void){
	OFF_TIMEOUT = 0xFF;
	DISPLAY_TIMEOUT = 0xFF;
}
/*
**---------------------------------------------------------------------------
**
** Abstract: Ipod button send
**
**
** Parameters: * key
**
**
**---------------------------------------------------------------------------
*/
void ipod_button(char *key){
#if 0 //for ipod
	unsigned int a, checksum, length;
	//xprintf("\r\nButton Press - ");
	length = key[2]+3;

	//Command
	for(a=0, checksum=0x100;a<=length;a++){
		checksum = checksum - key[a];
		usart_putc_1(key[a]);
		//usart_byte2ascii(key[a]);
	}
	//Parameter
	//Not yet implemented
	
	//Checksum
	//usart_putc_1(checksum);
	//usart_byte2ascii(checksum);
	//xprintf("\r\n");
	
	//Send Release key
	//xprintf("\r\nButton Release - ");
	for(a=0;a<7;a++){
		usart_putc_1(IPOD_BUTTON_RELEASE[a]);
	}
#endif
}

/*
**---------------------------------------------------------------------------
**
** Abstract: Ipod button detect
**
**
** Parameters: * key
**
**
**---------------------------------------------------------------------------
*/
void ipod_control(unsigned char key){
#if 0
	if(key == BUTTON_PLAYPAUSE_TOGGLE){
		if(IpodStatus==IPOD_STATUS_PLAY){
			xprintf("Screen Pause");
			ipod_button(IPOD_PLAY_PAUSE);
			SendToScreen(0x121, "PAUSE", 0, TEMP_STRING);
			IpodStatus = IPOD_STATUS_STOP;
		}
		else if(IpodStatus==IPOD_STATUS_STOP){
			xprintf("Screen Play");
			ipod_button(IPOD_PLAY_PAUSE);
			SendToScreen( 0x121, "PLAY", 0, TEMP_STRING);
			//IpodStatus = IPOD_STATUS_PLAY;
		}
		return;
	}
	if(key == BUTTON_PREV){
		if(IpodStatus==IPOD_STATUS_PLAY){
			xprintf("Screen Track-");
			ipod_button(IPOD_PREV);
			SendToScreen( 0x121, "Track-", 0, TEMP_STRING);
		}
		return;
	}
	if(key == BUTTON_NEXT){
		if(IpodStatus==IPOD_STATUS_PLAY){
			xprintf("Screen Track+");
			ipod_button(IPOD_NEXT);
			SendToScreen( 0x121, "Track+", 0, TEMP_STRING);
		}
		return;
	}
	if(key == BUTTON_PLAY_PAUSE){
		if(IpodStatus==IPOD_STATUS_PLAY){
			//usart_byte2ascii(IpodStatus);
			return;
		}
		else if(IpodStatus==IPOD_STATUS_STOP){
			ipod_button(IPOD_PLAY_PAUSE);
			//SendToScreen( 0x121, "PLAY", 0, TEMP_STRING);
			IpodStatus = IPOD_STATUS_PLAY;
			//usart_byte2ascii(IpodStatus);
		}
		else if(IpodStatus==IPOD_STATUS_OFF){
			xprintf("Off");
			ipod_button(IPOD_ON);
			ipod_button(IPOD_SIMPLE_REMOTE);
			ipod_button(IPOD_PLAY);
			IpodStatus = IPOD_STATUS_PLAY;
			//xprintf("Pllay");
		}
	}
	if(key == BUTTON_STOP_PAUSE){
		if(IpodStatus==IPOD_STATUS_PLAY){
			ipod_button(IPOD_PLAY_PAUSE);
			//SendToScreent ( 0x121, "PAUSE", 0, TEMP_STRING);
			IpodStatus = IPOD_STATUS_STOP;
			return;
		}
		if(IpodStatus==IPOD_STATUS_STOP){
			return;
		}
	}
	if(key == BUTTON_PLAY_ONLY){
		if(IpodStatus==IPOD_STATUS_STOP){
			ipod_button(IPOD_PLAY);
			IpodStatus = IPOD_STATUS_PLAY;
			return;
		}
		if(IpodStatus==IPOD_STATUS_OFF){
			ipod_button(IPOD_ON);
			ipod_button(IPOD_SIMPLE_REMOTE);
			//_delay_ms(5);
			ipod_button(IPOD_PLAY);
			IpodStatus = IPOD_STATUS_PLAY;
			return;
		}
	}
	if(key == BUTTON_STOP_ONLY){
		if(IpodStatus!=IPOD_STATUS_OFF){
			ipod_button(IPOD_STOP);
			IpodStatus = IPOD_STATUS_STOP;
		}
		return;
	}
	if(key == BUTTON_ON){
		if(IpodStatus==IPOD_STATUS_OFF){
			ipod_button(IPOD_ON);
			ipod_button(IPOD_SIMPLE_REMOTE);
			IpodStatus = IPOD_STATUS_ON;
		}
		return;
	}
	if(key == BUTTON_OFF){
		if(IpodStatus!=IPOD_STATUS_OFF){
			ipod_button(IPOD_OFF);
			IpodStatus = IPOD_STATUS_OFF;
		}
		return;
	}
#endif
}

#if 0 //TODO
ISR(TIMER2_OVF_vect){
	if(OffTimeout==0){
		OffTimeout=0xFF;
		//xprintf_1(IPOD_OFF);
		ipod_control(BUTTON_OFF);
		//xprintf("TO/OFF\r\n");
	}
	else{
		OffTimeout--;
	}
	if(ScreenTimeoutCounter==0){
		//TCCR2 = 0;			// stop timer
		//TIMSK &= ~_BV (TOIE2);	// disable overflow interrupt
		ScreenTimeoutCounter=0xFF;
		ReturnScreen = 1;
//		SendToScreen(0x121, "", 0, RETURN_STRING);
		//xprintf("ScreenTimeoutCounter=0");
	}
	else if(ScreenTimeoutCounter<=BUTTON_TIMEOUT_HIGH){
		ScreenTimeoutCounter--;
		return;
	}
	
	else{
	}
}
#endif

/*
**---------------------------------------------------------------------------
**
** Abstract: Send string to Renault screen
**
**
** Parameters: * String , ScreenIconsON_OFF(1 or 0),
** tempstring:
**   		NEW_STRING
**   		TEMP_STRING,
**   		RETURN_STRING
**
** Returns: status of command execution
**          CR = OK
**          ERROR = Error
**
**---------------------------------------------------------------------------
*/
void Screen_init ( void ){
	CompleteLength = 0;
	ScreenTextStart = 0;
	last_key = 0;
	Source = 0;
	SourceTemp = 0;
	ScreenTimeoutCounter = 0;
	OffTimeout = 0;
	RecieveComplete = 0;
	RX_In_progress = 0;
	ButtomPressed = 0;
	source_change = 0;
	WaitingForScreen = 0;
	ScreenTimeout_init();
//	SendToScreen(0x121, "PHONE", 0, NEW_STRING);
}
/*
 **---------------------------------------------------------------------------
 **
 ** Abstract: Send string to Renault screen
 **
 **
 ** Parameters: * String , ScreenIconsON_OFF(1 or 0),
 ** tempstring:
 **   		NEW_STRING
 **   		TEMP_STRING,
 **   		RETURN_STRING
 **
 ** Returns: status of command execution
 **          CR = OK
 **          ERROR = Error
 **
 **---------------------------------------------------------------------------
 */
unsigned char SendToScreen ( unsigned int SendId,
 									char * String,
 									unsigned char ScreenIcons,
 									unsigned char Command){

	if ((SourceTemp==REMOTE_PAUSE)||(SourceTemp==REMOTE_TRAFFIC)){
		DISPLAY_TIMEOUT = -1;
		return ERROR;
	}

 	unsigned char a, b;
 	uint32_t Timeout;
 	Screen_tx.ID = SendId;
 	char temp_string[10];
 	temp_string[0]=0x00;

 	//Create string (all blanks are spaces)
 	strlcpy(temp_string, String, 9);
 	strlcat(temp_string, Spaces, 9);

 	//
 	// Copy relevant string to screen buffer
 	//
 	if(Command == NEW_STRING){
 //		xprintf("New String ");
 		xprintf(INFO "Sending - %s",temp_string);FFL_();
 		for( a=0; a<8; a++){
 			*(PrevScreenText+a) = *(temp_string+a);
 		}
 		for( a=Ascii3IconTextStart, b=0; b<8; a++, b++){
 			if(a==8)
 				a++;
 			*(ScreenText+a) = *(temp_string+b);
 		}
 		DISPLAY_TIMEOUT = -1;
 	}
 	else if(Command == TEMP_STRING){
 //		xprintf("Temp String ");
 		xprintf(INFO "Sending - %s",temp_string);FFL_();
 		for( a=Ascii3IconTextStart, b=0; b<8; a++, b++){
 			if(a==8)
 				a++;
 			*(ScreenText+a) = *(temp_string+b);
 		}
 		DISPLAY_TIMEOUT=BUTTON_TIMEOUT;
 	}
 	else if(Command == RETURN_STRING){
 //	else if(0){
 //		xprintf("Return String ");
 		xprintf(INFO "Sending - %s",PrevScreenText);FFL_();
 		for( a=Ascii3IconTextStart, b=0; b<8; a++, b++){
 			if(a==8)
 				a++;
 			*(ScreenText+a) = *(PrevScreenText+b);
 		}
 		DISPLAY_TIMEOUT = -1;
 	}
 	else{
 //		xprintf("ERROR");
 		DISPLAY_TIMEOUT = -1;
 	}
 //	xprintf("\r\n");
 /*
 	//
 	// If using extra icons use 5 packets
 	// else use 4
 	//
 	if (ScreenIcons)
 		Screen_tx.NoToSend = 5;
 	else
 */
 		Screen_tx.NoToSend = 4;


 	if(SendScreen()==CR)
 		return CR;
 	return ERROR;
 }
/*
**---------------------------------------------------------------------------
**
** Abstract: Send an 8 char string to Renault screen
**
**
** Returns: status of command execution
**          CR = OK
**          ERROR = Error
**
**---------------------------------------------------------------------------
*/
unsigned char SendScreen ( void ){
	uint8_t ScreenPacketNo;
	for(Screen_tx.NoSent=0;Screen_tx.NoSent<Screen_tx.NoToSend;Screen_tx.NoSent++){
		// check if CAN controller is in reset mode or busy

		TXMsg.id = Screen_tx.ID;
		TXMsg.type = DATA_FRAME;// no remote transmission request
		// store std. frame format
		TXMsg.format = STD_ID_FORMAT;
		TXMsg.len = 8;
		// copy ID and Data from buffer to SJA1000 transmit buffer
		ScreenPacketNo = Screen_tx.NoSent*8;
		TXMsg.dataA[0] = *(ScreenText+(0+ScreenPacketNo));
		TXMsg.dataA[1] = *(ScreenText+(1+ScreenPacketNo));
		TXMsg.dataA[2] = *(ScreenText+(2+ScreenPacketNo));
		TXMsg.dataA[3] = *(ScreenText+(3+ScreenPacketNo));
		TXMsg.dataB[0] = *(ScreenText+(4+ScreenPacketNo));
		TXMsg.dataB[1] = *(ScreenText+(5+ScreenPacketNo));
		TXMsg.dataB[2] = *(ScreenText+(6+ScreenPacketNo));
		TXMsg.dataB[3] = *(ScreenText+(7+ScreenPacketNo));
		// start normal transmission
		CheckForCANErr();
		CAN_SendMsg(LPC_CAN2, &TXMsg);
		delay_ms(1);
		WaitForID(0x521, 500);
	}
	return CR;
}
/*
**---------------------------------------------------------------------------
**
** Abstract: Send source change key to Renault screen
**
** Parameters: dir - can be either '+' or '-'
**
** Returns: status of command execution
**          CR = OK
**          ERROR = Error
**
**---------------------------------------------------------------------------
*/
uint8_t send_source_change(char dir){
	//t0A9803890001A2A2A2A2
	//t0A9803890002A2A2A2A2
	uint32_t Timeout, dest=0x0A9;
	TXMsg.id = dest;
	// check if CAN controller is in reset mode or busy
	CheckForCANErr();

	TXMsg.type = DATA_FRAME;// no remote transmission request
	// store std. frame format
	TXMsg.format = STD_ID_FORMAT;
	TXMsg.len = 8;
	TXMsg.id = dest;
	TXMsg.dataA[0] = 0x03;
	TXMsg.dataA[1] = 0x89;
	TXMsg.dataA[2] = 0x00;
	if(dir=='+')
		TXMsg.dataA[3] = 0x01;
	else if(dir=='-')
		TXMsg.dataA[3] = 0x02;
	TXMsg.dataB[0] = 0xA2;
	TXMsg.dataB[1] = 0xA2;
	TXMsg.dataB[2] = 0xA2;
	TXMsg.dataB[3] = 0xA2;
	// start normal transmission
	CAN_SendMsg(LPC_CAN2, &TXMsg);
	WaitForID(0x4A9, 500);
//	return ERROR;
	return CR;
}

/*
**---------------------------------------------------------------------------
**
** Abstract: Renault screen loop
**
** Parameters: dir - can be either '+' or '-'
**
** Returns: status of command execution
**          CR = OK
**          ERROR = Error
**
**---------------------------------------------------------------------------
*/
void Screen_loop( void ){
#if 0
	if(flags.source_change){
		source_selected++;
		if(source_selected == number_of_sources)
			source_selected = 1;
		switch(source_selected){
			case PHONE:
				xprintf("PHONE");
				break;
			case CD:
				xprintf("CD");
				break;
			case PC:
				xprintf("PC");
				break;
		}
		xprintf(" selected.\r\n");
		flags.source_change = 0;
	}
#endif
#ifndef IPOD_ENABLE
	unsigned char 	ScreenText[10],
					IBCopy = 0;

	if((DISPLAY_TIMEOUT==0)&&(!ButtomPressed)&&(!RecieveComplete))
		SendToScreen(0x121, "", 0, RETURN_STRING);
	if(RecieveComplete){
		for (IBCopy = 0; IBCopy <= 7*5; IBCopy++){
			//0x01 indicates this is the start of the screen ascii
			if (ScreenTempData[IBCopy] == 0x01){
				ScreenTextStart = IBCopy+1;
				//copy screen text into buffer
				for(uint32_t t=0; t<8; t++){
					ScreenText[t] = ScreenTempData[ScreenTextStart+t];
					ScreenText[t+1] = NULL;
				}
				break;
			}
		}
		for (IBCopy = 0; IBCopy <= 7*5; IBCopy++){
			//if 0x0081 indicates this is the end of transmission
			if ((ScreenTempData[IBCopy] == 0x00)&&(ScreenTempData[IBCopy+1]==0x81)){
				CompleteLength = IBCopy;
				break;
			}
		}

		//Check for Aux
		xprintf("ScreenText=|%s|\r\n");
		if( strncmp(ScreenText,"AUX",3) == 0){
			if(FirstScreen){
				SendToScreen(0x121, "PHONE", 0, NEW_STRING);
				xprintf("TASKER:PLAY\r\n");
				FirstScreen=0;
			}
			else
				SendToScreen( 0x121, "", 0, RETURN_STRING);
//			if(Source==REMOTE_AUX){

//			}
#if 0
			else {
				if (last_key=='-')
					source_selected = number_of_sources;
				else if (last_key=='+'){
					source_selected = 1;
				}
				// first use
				else {
					//_delay_ms(20);
					source_selected = 1;
				}
				source_change = 1;
			}
#endif
			if((SourceTemp==REMOTE_PAUSE)||(SourceTemp==REMOTE_TRAFFIC)){
				xprintf("TASKER:UNPAUSE\r\n");
			}
			Source = REMOTE_AUX;
			SourceTemp = REMOTE_AUX;
				//ipod_control(BUTTON_PLAY_ONLY);
//				ipod_control(BUTTON_PLAY_PAUSE);
		}
		else if( strncmp(ScreenText," PAUSE",6) == 0){
			SourceTemp= REMOTE_PAUSE;
			xprintf("TASKER:PAUSE\r\n");
//			ipod_control(BUTTON_STOP_PAUSE);
		}
		else if( strncmp(ScreenText,"VOL",3) == 0){
			SourceTemp = REMOTE_VOLUME;
//			xprintf("TASKER:PAUSE\r\n");
//			ipod_control(BUTTON_STOP_PAUSE);
		}
		else if( strncmp(ScreenText,"TRAFFIC",7) == 0){
			SourceTemp = REMOTE_TRAFFIC;
			xprintf("TASKER:PAUSE\r\n");
//			ipod_control(BUTTON_STOP_PAUSE);
		}
#if 0
		else if( strncmp(ScreenText,"CD ",3) == 0){
			Source = REMOTE_CD;
			xprintf("TASKER:STOP\r\n");
//			ipod_control(BUTTON_STOP_ONLY);
		}
		else if( strncmp(ScreenText,"FM ",3) == 0){
			Source = REMOTE_RADIO;
			xprintf("TASKER:STOP\r\n");
//			ipod_control(BUTTON_STOP_ONLY);
		}
		else if( strncmp(ScreenText,"MW ",3) == 0){
			Source = REMOTE_RADIO;
			xprintf("TASKER:STOP\r\n");
//			ipod_control(BUTTON_STOP_ONLY);
		}
#endif
		else{
			//Source = RADIO;
			//xprintf("R fd");
			if(!(SourceTemp==REMOTE_TRAFFIC)){
				xprintf("TASKER:STOP\r\n");
				Source = REMOTE_RADIO;
			}
//			ipod_control(BUTTON_STOP_ONLY);
		}
		renault_debug_print();
		RecieveComplete = 0;
		WaitingForScreen = 0;
	}



	if(ButtomPressed){
		// Remote pressed
		xprintf("Key Seen - ");
		switch(ButtonTempFull){
			case 0x0001:  //Source Right t0A9803890001A2A2A2A2
				xprintf("Source Next\r\n");
#if 0
				if ((Source==REMOTE_AUX)&&(SourceTemp!=REMOTE_TRAFFIC))
					source_change ='+';
				last_key='+';
#endif
				break;
			case 0x0002: //Source Left t0A9803890002A2A2A2A2
				xprintf("Source Prev\r\n");
#if 0
				if ((Source==REMOTE_AUX)&&(SourceTemp!=REMOTE_TRAFFIC))
					source_change='-';
				last_key='-';
#endif
				break;
			case 0x0003: //vol up t0A9803890003A2A2A2A2
				xprintf("Volume Up\r\n");
				break;
			case 0x04: //vol down t0A9803890004A2A2A2A2
				xprintf("Volume Down\r\n");
				break;
			case 0x05: //pause t0A9805890005A2A2A2A2
				xprintf("Pause\r\n");
#if 0
				if (Source==REMOTE_PAUSE){
					ipod_control(BUTTON_PLAY_PAUSE);
					Source=REMOTE_AUX;
				}
				usart_puts_1(IPOD_PLAY_PAUSE);
#endif
				break;
			case 0x0000: // t0A9803890000A2A2A2A2
			case 0x000A: // t0A980389000AA2A2A2A2 Enter (IPOD PLAY/PAUSE)
				xprintf("Enter\r\n");
				if (Source == REMOTE_AUX){
					//usart_byte2ascii(IpodStatus);
//					if((source_selected==PHONE)||(source_selected==PC)){
//						ipod_button(IPOD_PLAY_PAUSE);
//						xprintf("TASKER:PLAY/PAUSE\r\n");
						xprintf("TASKER:PAUSE\r\n");
						SendToScreen( 0x121, "=/>", 0, TEMP_STRING);
						//SendToScreen( 0x121, "Play/Pau", 0, TEMP_STRING);
						//delay_ms(500);
						//SendToScreen( 0x121, "ay/Pause", 0, TEMP_STRING);
						//delay_ms(100);
						//ipod_control(BUTTON_PLAYPAUSE_TOGGLE);
//					}
//					if(source_selected==KEYBOARD){
//						//usart_putc(current_key_keyboard);
//					}
//					else {
//					}
				}
				break;
			case 0x0101: //Track next t0A9803890101A2A2A2A2
				xprintf("Track Next\r\n");
				//usart_puts_1(IPOD_NEXT);
				if ((Source == REMOTE_AUX)&&((SourceTemp!=REMOTE_PAUSE)||(SourceTemp!=REMOTE_TRAFFIC))){
					xprintf("TASKER:NEXT\r\n");
					SendToScreen( 0x121, "Track +", 0, TEMP_STRING);
#if 0
					if(source_selected==PHONE){
						xprintf("TASKER:NEXT\r\n");
						SendToScreen( 0x121, "Track +", 0, TEMP_STRING);
						ipod_control(BUTTON_NEXT);
					}
							else if(source_selected==KEYBOARD){
//						keyboard_key('+');
					}
					else{
						xprintf("TASKER:NEXT\r\n");
						SendToScreen(0x121, "Track +", 0, TEMP_STRING);
					}
#endif
				}
				break;
			case 0x0141: //Track back t0A9803890141A2A2A2A2
				xprintf("Track Prev\r\n");
				//usart_puts_1(IPOD_PREV);
				if ((Source == REMOTE_AUX)&&((SourceTemp!=REMOTE_PAUSE)||(SourceTemp!=REMOTE_TRAFFIC))){
					xprintf("TASKER:PREV\r\n");
					SendToScreen(0x121, "Track -", 0, TEMP_STRING);
#if 0
					if(source_selected==PHONE){
						xprintf("TASKER:PREV\r\n");
						ipod_control(BUTTON_PREV);
						SendToScreen(0x121, "Track -", 0, TEMP_STRING);
					}
					else if(source_selected==KEYBOARD){
						keyboard_key('-');
					}
					else{
						xprintf("TASKER:PREV\r\n");
						SendToScreen(0x121, "Track -", 0, TEMP_STRING);
					}
#endif
				}
				break;
			default:
				xprintf("Unknown (0x%x) t0A980389****A2A2A2A2\r\n",ButtonTempFull);
				break;
		}
		ButtomPressed=0;
	}


#if 0 //old
	if(!WaitingForScreen&&ButtomPressed){
		// Remote pressed
		xprintf("Key Seen - ");
		switch(ButtonTempData[0]){
			case 0x00:
				switch(ButtonTempData[1]){
					case 0x01: //Source Right t0A9803890001A2A2A2A2
						xprintf("Source Next\r\n");
#if 0
						if ((Source==REMOTE_AUX)&&(SourceTemp!=REMOTE_TRAFFIC))
							source_change ='+';
						last_key='+';
#endif
						break;
					case 0x02: //Source Left t0A9803890002A2A2A2A2
						xprintf("Source Prev\r\n");
#if 0
						if ((Source==REMOTE_AUX)&&(SourceTemp!=REMOTE_TRAFFIC))
							source_change='-';
						last_key='-';
#endif
						break;
					case 0x03: //vol up t0A9803890003A2A2A2A2
						xprintf("Volume Up\r\n");
						break;
					case 0x04: //vol down t0A9803890004A2A2A2A2
						xprintf("Volume Down\r\n");
						break;
					case 0x05: //pause t0A9805890005A2A2A2A2
						xprintf("Pause\r\n");
#if 0
						if (Source==REMOTE_PAUSE){
							ipod_control(BUTTON_PLAY_PAUSE);
							Source=REMOTE_AUX;
						}
						usart_puts_1(IPOD_PLAY_PAUSE);
#endif
						break;
					case 0x00: // t0A9803890000A2A2A2A2
					case 0x0A: // t0A980389000AA2A2A2A2 Enter (IPOD PLAY/PAUSE)
//						xprintf("Play Pause\r\n");
						if (Source == REMOTE_AUX){
							//usart_byte2ascii(IpodStatus);
//							if((source_selected==PHONE)||(source_selected==PC)){
//								ipod_button(IPOD_PLAY_PAUSE);
								xprintf("ENTER\r\n");
//								xprintf("TASKER:PLAY/PAUSE\r\n");
								xprintf("TASKER:PAUSE\r\n");
								SendToScreen( 0x121, "=/>", 0, TEMP_STRING);
								//SendToScreen( 0x121, "Play/Pau", 0, TEMP_STRING);
								//delay_ms(500);
								//SendToScreen( 0x121, "ay/Pause", 0, TEMP_STRING);
								//delay_ms(100);
								//ipod_control(BUTTON_PLAYPAUSE_TOGGLE);
//							}
//							if(source_selected==KEYBOARD){
//								//usart_putc(current_key_keyboard);
//							}
//							else {
//							}
						}
						break;
					default:
						break;
				}
				break;
			case 0x01:
				switch(ButtonTempData[1]){
					case 0x01: //Track next t0A9803890101A2A2A2A2
						xprintf("Track Next\r\n");
						//usart_puts_1(IPOD_NEXT);
						if ((Source == REMOTE_AUX)&&((SourceTemp!=REMOTE_PAUSE)||(SourceTemp!=REMOTE_TRAFFIC))){
							xprintf("TASKER:NEXT\r\n");
							SendToScreen( 0x121, "Track +", 0, TEMP_STRING);
#if 0
							if(source_selected==PHONE){
								xprintf("TASKER:NEXT\r\n");
								SendToScreen( 0x121, "Track +", 0, TEMP_STRING);
								ipod_control(BUTTON_NEXT);
							}
							else if(source_selected==KEYBOARD){
//								keyboard_key('+');
							}
							else{
								xprintf("TASKER:NEXT\r\n");
								SendToScreen(0x121, "Track +", 0, TEMP_STRING);
							}
#endif
						}
						break;
					case 0x41: //Track back t0A9803890141A2A2A2A2
						xprintf("Track Prev\r\n");
						//usart_puts_1(IPOD_PREV);
						if ((Source == REMOTE_AUX)&&((SourceTemp!=REMOTE_PAUSE)||(SourceTemp!=REMOTE_TRAFFIC))){
							xprintf("TASKER:PREV\r\n");
							SendToScreen(0x121, "Track -", 0, TEMP_STRING);
#if 0
							if(source_selected==PHONE){
								xprintf("TASKER:PREV\r\n");
								ipod_control(BUTTON_PREV);
								SendToScreen(0x121, "Track -", 0, TEMP_STRING);
							}
							else if(source_selected==KEYBOARD){
								keyboard_key('-');
							}
							else{
								xprintf("TASKER:PREV\r\n");
								SendToScreen(0x121, "Track -", 0, TEMP_STRING);
							}
#endif
						}
						break;
					default:
						break;
				}
				break;
			default:
				break;
		}
		//ButtomPressed=0;//Must be done after passing screen data
	}
#endif //old

#endif
#if 0
	if(((source_change==1)||(source_change=='+')||(source_change=='-'))&&(Source==REMOTE_AUX)){
		if(source_change=='+'){
			source_selected++;
			if(source_selected==(number_of_sources+1)){
				source_selected=0;
				source_change=0;
			}
			else
				send_source_change('-');
		}
		else if(source_change=='-'){
			source_selected--;
			if(source_selected==0){
				source_change=0;
			}
			else
				send_source_change('+');
		}
		else{
		}
		if(source_change){
			switch(source_selected){
				case PHONE:
					SendToScreen(0x121, "PHONE", 0, NEW_STRING);
					break;
				case PC:
					SendToScreen(0x121, "PC", 0, NEW_STRING);
					break;
				case KEYBOARD:
					SendToScreen(0x121, "KB", 0, NEW_STRING);
					break;
				default:
					break;
			}
		}
		source_change = 0;
	}
#endif
}
/*
**---------------------------------------------------------------------------
**
** Abstract: Renault screen interrupt
**
** Parameters: dir - can be either '+' or '-'
**
** Returns: status of command execution
**          CR = OK
**          ERROR = Error
**
**---------------------------------------------------------------------------
*/
void Screen_Interrupt( void ){
	unsigned char rec_121, RxScreenPacketNo;
//	CAN_rx_msg.Last_id = CAN_rx_msg.id;
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

send
0x121 1019766001545220
0x121 2130362043441043
0x121 2244202020545220
0x121 2330362020008181
0x521 74A2A2A2A2A2A2A2
*/

	switch(RXMsg.id){
		case 0x0A9: // Button Pressed
//			ButtonTempData[0] = RXMsg.dataA[2];
//			ButtonTempData[1] = RXMsg.dataA[3];
			ButtonTempFull = ( RXMsg.dataA[2] << 8 ) || RXMsg.dataA[3];
//			if(!(ButtonTempFull==0x000A))//enter button not pressed
//				WaitingForScreen=1;
			ButtomPressed=1;
			break;
		case 0x121: // Screen data
//			Screen_RX_In_progress = 1;
			rec_121 = (RXMsg.dataA[0] & 0x0F);
			RxScreenPacketNo = rec_121*7;
			ScreenTempData[0+RxScreenPacketNo] = RXMsg.dataA[1];
			ScreenTempData[1+RxScreenPacketNo] = RXMsg.dataA[2];
			ScreenTempData[2+RxScreenPacketNo] = RXMsg.dataA[3];
			ScreenTempData[3+RxScreenPacketNo] = RXMsg.dataB[0];
			ScreenTempData[4+RxScreenPacketNo] = RXMsg.dataB[1];
			ScreenTempData[5+RxScreenPacketNo] = RXMsg.dataB[2];
			ScreenTempData[6+RxScreenPacketNo] = RXMsg.dataB[3];
//			if (rec_121==0){
//				RX_In_progress = 1;
//			}
			if ( ((rec_121 == 3)||(rec_121==4)) && (ScreenTempData[6+RxScreenPacketNo] == 0x81)){
//				Screen_RX_In_progress = 0;
				RecieveComplete=1;
			}
		case 0x3CF: // Ping recieved
		case 0x3DF: // Ping recieved
			//exec_usart_cmd("t3DF87900818181818181");
			//exec_usart_cmd("t3CF86900A2A2A2A2A2A2");
			OffTimeout=0xFF;
			break;
		//Power off timeout
		case 0x4A9: // Ok from remote recieved
//			flags.recieved_4A9 = true;
			break;
		case 0x521: // Acknowledged screen data
//			Screen_tx.NoAck++;
//			flags.recieved_521 = true;
			// send next screen data.
			break;
		case 0x5B1: // Ping recieved
		case 0x1B1: // Ping recieved
			//exec_usart_cmd("t3DF87900818181818181");
			//exec_usart_cmd("t3CF86900A2A2A2A2A2A2");
			break;
		case 0x5C1: // Ping recieved
		case 0x1C1: // Ping recieved
			//exec_usart_cmd("t3DF87900818181818181");
			//exec_usart_cmd("t3CF86900A2A2A2A2A2A2");
			break;
#if 0
		case 0x0A9: // Remote pressed
			switch(RXMsg.dataA[2]){
				case 0x00:
					switch(RXMsg.dataA[3]){
						case 0x05:
							xprintf("cp\r\n");
							//pause
							break;
						case 0x03:
							xprintf("cv+\r\n");
							//vol up
							break;
						case 0x04:
							xprintf("cv-\r\n");
							//vol down
							break;
						case 0x01:
							xprintf("cs+\r\n");
							//Sou Right
							break;
						case 0x02:
							xprintf("cs-\r\n");
							flags.source_change = 1;
							//Sou Left
							break;
						case 0x0A:
							xprintf("ce\r\n");
							flags.source_change = 1;
							//Enter
							break;
					}
					break;
				case 0x01:
					switch(RXMsg.dataA[3]){
						case 0x41:
							xprintf("ct-\r\n");
							//Track back
							break;
						case 0x01:
							xprintf("ct+\r\n");
							//Track next
							break;
					}
					break;
			}
			//exec_usart_cmd("t49Aetc");
			break;
#endif
		default:
			xprintf("ID (%x) not found.\r\n",RXMsg.id);
			break;
	}
}

/*
**---------------------------------------------------------------------------
**
** Abstract: Send debug data with screen info
**
** Parameters: dir - can be either '+' or '-'
**
**
**---------------------------------------------------------------------------
*/
void renault_debug_print( void ) {
	unsigned char  IBCopy = 0;
	if(SendScreenTxtStart){
		xprintf("Screen Txt Start - %x\n",ScreenTextStart);
	}
	if(SendDebugHex){
		xprintf("0x");
		for(IBCopy = StartOfMessage; IBCopy < ScreenTextStart-1; IBCopy++){
			//usart_byte2ascii(ScreenTempData[IBCopy]);
			xprintf("%8x",ScreenTempData[IBCopy]);
		}
		xprintf("\n0b");
		for(IBCopy = StartOfMessage; IBCopy < ScreenTextStart-1; IBCopy++){
			xprintf("%8b",ScreenTempData[IBCopy]);
		}
		xprintf("\n");
	}
	if(SendScreenAsciiData){
		xprintf("Screen Ascii Data - \n");
		for ( IBCopy = ScreenTextStart; IBCopy <= ScreenTextStart ; IBCopy++){
			xprintf("        ");
		}
		for ( IBCopy = ScreenTextStart; IBCopy  < (ScreenTextStart + AsciiLength -1) ; IBCopy++){
			//usart_byte2ascii(ScreenTempData[IBCopy]);
			xprintf("%8c",ScreenTempData[IBCopy]);
		}
//		xprintf("\r\n");
//		xprintf("ComLen - 0x");
//		usart_byte2ascii(CompleteLength);
		xprintf("\n");
	}
	if(SendScreenTempData){
		xprintf("Screen Temp Data - \n");
		for ( IBCopy = 0; IBCopy  < CompleteLength ; IBCopy++){
			//usart_byte2ascii(ScreenTempData[IBCopy]);
			xprintf("%8c",ScreenTempData[IBCopy]);
		}
		xprintf("\n");
	}
/*	//Set all ascii data to 5
	for (IBCopy = 0; IBCopy <= 7*5; IBCopy++){
		ScreenTempData[IBCopy]  = 0x55;
	}
*/
}

void keyboard_key (uint8_t dir){
	char temp_QuickScreenText[9];
	temp_QuickScreenText[0]=0x00; // causes string functions to think string is empty
	if(current_key_keyboard==0)
		current_key_keyboard=min_key_keyboard;
	if(dir=='+')
		current_key_keyboard++;
	else if(dir=='-')
		current_key_keyboard--;
	else{
	}
	//loop rount to end
	if(current_key_keyboard==(min_key_keyboard-1))
		current_key_keyboard=max_key_keyboard;
	//loop rount to beginning
	if(current_key_keyboard==(max_key_keyboard+1))
		current_key_keyboard=min_key_keyboard;
	//Other
	if((min_key_keyboard<=current_key_keyboard)&&(current_key_keyboard<=0x40)){
		switch (current_key_keyboard){
		    case (0x41-16):
				strlcat (temp_QuickScreenText, "TEST", 9); // 9th space is for NULL
				break;
		    case (0x41-15):
				strlcat (temp_QuickScreenText, "CAP LOCK", 9); // 9th space is for NULL
				break;
		    case (0x41-14):
				strlcat (temp_QuickScreenText, "-", 9); // 9th space is for NULL
				break;
		    case (0x41-13):
				strlcat (temp_QuickScreenText, "+", 9); // 9th space is for NULL
				break;
		    case (0x41-12):
				strlcat (temp_QuickScreenText, "PLAY/PAU", 9); // 9th space is for NULL
				break;
		    case (0x41-11):
				strlcat (temp_QuickScreenText, "Vol +", 9); // 9th space is for NULL
				break;
		    case (0x41-10):
				strlcat (temp_QuickScreenText, "Vol -", 9); // 9th space is for NULL
				break;
		    case (0x41-9):
				strlcat (temp_QuickScreenText, "UP", 9); // 9th space is for NULL
				break;
		    case (0x41-8):
				strlcat (temp_QuickScreenText, "DOWN", 9); // 9th space is for NULL
				break;
		    case (0x41-7):
				strlcat (temp_QuickScreenText, "LEFT", 9); // 9th space is for NULL
				break;
		    case (0x41-6):
				strlcat (temp_QuickScreenText, "RIGHT", 9); // 9th space is for NULL
				break;
		    case (0x41-5):
				strlcat (temp_QuickScreenText, "ALT+F4", 9); // 9th space is for NULL
				break;
		    case (0x41-4):
				strlcat (temp_QuickScreenText, "CT+ALT+D", 9); // 9th space is for NULL
				break;
		    case (0x41-3):
				strlcat (temp_QuickScreenText, "BACKSPAC", 9); // 9th space is for NULL
				break;
		    case (0x41-2):
				strlcat (temp_QuickScreenText, "SPACE", 9); // 9th space is for NULL
				break;
		    case (0x41-1):
				strlcat (temp_QuickScreenText, "CR", 9); // 9th space is for NULL
				break;
		    default:
				break;
		}
	}
	//Upper Case
	if((0x41<=current_key_keyboard)&&(current_key_keyboard<=0x5A)){
		temp_QuickScreenText[0]=current_key_keyboard;
		temp_QuickScreenText[1]=' ';
		temp_QuickScreenText[2]=0x00;
		strlcat (temp_QuickScreenText, "UPPER", 9); // 9th space is for NULL
	}
	//Lower Case
	else if((0x61<=current_key_keyboard)&&(current_key_keyboard<=0x7A)){
		temp_QuickScreenText[0]=current_key_keyboard-20;
		temp_QuickScreenText[1]=' ';
		temp_QuickScreenText[2]=0x00;
		strlcat(temp_QuickScreenText, "LOWER", 9); // 9th space is for NULL
	}
	else if(current_key_keyboard>0x7A){
	}
	else{

	}
//	strlcat(temp_QuickScreenText, spaces, 9); // fills empty space with ' ';
	SendToScreen(0x121, temp_QuickScreenText, 0, NEW_STRING);

	xprintf("Keyboard %s\r\n",temp_QuickScreenText);

	return;
}

