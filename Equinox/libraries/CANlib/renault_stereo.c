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

unsigned char source_selected;
uint8_t current_key_keyboard=0;
#define min_key_keyboard 0x31 // Beginning of special
//#define min_key_keyboard 0x41 // Upper Case start
//#define max_key_keyboard 0x7F // Lower Case End
#define max_key_keyboard 0x5A // Upper Case End

extern CAN_MSG_Type TXMsg, RXMsg; // messages for test Bypass mode

void ScreenTimeout_init ( void){
	OffTimeout = 0xFF;
	ScreenTimeoutCounter = 0xFF;
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
			InitScreen(0x121, "PAUSE", 0, TEMP_STRING);
			IpodStatus = IPOD_STATUS_STOP;
		}
		else if(IpodStatus==IPOD_STATUS_STOP){
			xprintf("Screen Play");
			ipod_button(IPOD_PLAY_PAUSE);
			InitScreen( 0x121, "PLAY", 0, TEMP_STRING);
			//IpodStatus = IPOD_STATUS_PLAY;
		}
		return;
	}
	if(key == BUTTON_PREV){
		if(IpodStatus==IPOD_STATUS_PLAY){
			xprintf("Screen Track-");
			ipod_button(IPOD_PREV);
			InitScreen( 0x121, "Track-", 0, TEMP_STRING);
		}
		return;
	}
	if(key == BUTTON_NEXT){
		if(IpodStatus==IPOD_STATUS_PLAY){
			xprintf("Screen Track+");
			ipod_button(IPOD_NEXT);
			InitScreen( 0x121, "Track+", 0, TEMP_STRING);
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
			//InitScreen( 0x121, "PLAY", 0, TEMP_STRING);
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
			//InitScreent ( 0x121, "PAUSE", 0, TEMP_STRING);
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
//		InitScreen(0x121, "", 0, RETURN_STRING);
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
** Abstract: Initiate Sending string to Renault screen
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
unsigned char InitScreen ( unsigned int SendId, 
									char * String, 
									unsigned char ScreenIcons, 
									unsigned char Command){
	unsigned char a, b;
	uint32_t Timeout;
	Screen_tx.ID = SendId;
	char temp_string[10];
	temp_string[0]=0x00;
	
	//Create string (all blanks are spaces)
	strlcpy(temp_string, String, 9);
	strlcat(temp_string, Spaces, 9);
	
	xprintf("Original String - %s\r\nSending String - %s\r\n",String,temp_string);

	//
	// Copy relevant string to screen buffer
	//
	if(Command == NEW_STRING){
		xprintf("New String ");
		for( a=0; a<8; a++){
			*(PrevScreenText+a) = *(temp_string+a);
		}
		for( a=Ascii3IconTextStart, b=0; b<8; a++, b++){
			if(a==8)
				a++;
			*(ScreenText+a) = *(temp_string+b);
		}
		ReturnScreen = 0;k
	}
	else if(Command == TEMP_STRING){
//		xprintf("Temp String ");
		for( a=Ascii3IconTextStart, b=0; b<8; a++, b++){
			if(a==8)
				a++;
			*(ScreenText+a) = *(temp_string+b);
		}
		ScreenTimeoutCounter=BUTTON_TIMEOUT_HIGH;
	}
	else if(Command == RETURN_STRING){
//	else if(0){
//		xprintf("Return String ");
		for( a=Ascii3IconTextStart, b=0; b<8; a++, b++){
			if(a==8)
				a++;
			*(ScreenText+a) = *(PrevScreenText+b);
		}
		ReturnScreen = 0;
	}
	else{
//		xprintf("ERROR");
		ReturnScreen = 0;
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
		
	for(Timeout=0xff;Timeout>0;Timeout--){
		if(SendScreen()==CR)
			return CR;
	}
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
	uint32_t Timeout;
	uint8_t ScreenPacketNo;
	for(Screen_tx.NoSent=0;Screen_tx.NoSent<=Screen_tx.NoToSend;Screen_tx.NoSent++){
		// check if CAN controller is in reset mode or busy
		if (!CANBUS_ON() || TX_BUSY()){
			xprintf("CANTX Busy or not On\r\n");
			return ERROR;
		}
		TXMsg.id = Screen_tx.ID;
		// if transmit buffer is unlocked
#if 0
		while(0){
//		for(Timeout=0x00ff;Timeout>0;Timeout--){
			if ((StatusReg & _BV (TBS_Bit)) != _BV (TBS_Bit)){
				Timeout=1;
//				break;
			}
			if(Timeout==2){
//				xprintf("TBL");
				xprintf("TX Buffer Locked\r\n");
				_delay_ms(100);
//				return ERROR;
			}
		}
		_delay_ms(1);
#endif
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
//		CommandReg = _BV (TR_Bit);
		CAN_SendMsg(LPC_CAN2, &TXMsg);

#if 0
		for(Timeout=0xfff;Timeout>0x0;Timeout--){
			if(wait_for_reply(0x521)==CR);
				break;
			if(Timeout==1)
				return ERROR;
		}
#endif
	}
//	xprintf(" TXD Done\r\n");
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
	if (!CANBUS_ON() || TX_BUSY()){
		xprintf("CANTX Busy or not On\r\n");
		return ERROR;
	}
	// if transmit buffer is unlocked
#if 0
	while(0){
//	for(Timeout=0x00ff;Timeout>0;Timeout--){
		if ((StatusReg & _BV (TBS_Bit)) != _BV (TBS_Bit)){
			Timeout=1;
//			break;
		}
		if(Timeout==2){
//			xprintf("TBL");
			xprintf("TX Buffer Locked\r\n");
			_delay_ms(100);
//			return ERROR;
		}
	}
	_delay_ms(1); //must be small
#endif

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
//	CommandReg = _BV (TR_Bit);
	CAN_SendMsg(LPC_CAN2, &TXMsg);

#ifdef WaitForReply
	for(Timeout=0xffff;Timeout>0x0;Timeout--){
		if(wait_for_reply(0x4A9)==CR)
			return CR;
	}
#else
	return CR;
#endif
	return ERROR;
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
	unsigned char  IBCopy = 0, CompleteLength = 0, ScreenTextStart = 0;
	if(SendScreenTxtStart){
		xprintf("Screen Txt Start - %x\r\n",ScreenTextStart);
	}
	if(SendDebugHex){
		xprintf("0x");
		for(IBCopy = StartOfMessage; IBCopy < ScreenTextStart-1; IBCopy++){
			//usart_byte2ascii(ScreenTempData[IBCopy]);
			xprintf("%c",ScreenTempData[IBCopy]);
			xprintf("              ");
		}
		xprintf("\r\n0b");
		for(IBCopy = StartOfMessage; IBCopy < ScreenTextStart-1; IBCopy++){
			xprintf("%b",ScreenTempData[IBCopy]);
		}
		xprintf("\r\n");
	}
	if(SendScreenAsciiData){
		xprintf("Screen Ascii Data - ");
		for ( IBCopy = ScreenTextStart; IBCopy <= ScreenTextStart ; IBCopy++){
			xprintf("  ");
		}
		for ( IBCopy = ScreenTextStart; IBCopy  < (ScreenTextStart + AsciiLength -1) ; IBCopy++){
			//usart_byte2ascii(ScreenTempData[IBCopy]);
			xprintf("%c ",ScreenTempData[IBCopy]);
		}
//		xprintf("\r\n");
//		xprintf("ComLen - 0x");
//		usart_byte2ascii(CompleteLength);
		xprintf("\r\n");
	}
	if(SendScreenTempData){
		xprintf("Screen Temp Data - ");
		for ( IBCopy = 0; IBCopy  < CompleteLength ; IBCopy++){
			//usart_byte2ascii(ScreenTempData[IBCopy]);
			xprintf("%c",ScreenTempData[IBCopy]);
		}
		xprintf("\r\n");
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
	InitScreen(0x121, temp_QuickScreenText, 0, NEW_STRING);

	xprintf("Keyboard %s\r\n",temp_QuickScreenText);

	return;
}

