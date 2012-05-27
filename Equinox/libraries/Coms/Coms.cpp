/*
 * coms.cpp
 *
 *  Created on: 31 Mar 2011
 *      Author: Jamie
 */

#include "coms.h"
#include <avr/pgmspace.h>
#include "../Ansi/Ansiterm.h"
#include "../RTClib/RTClib.h"

//Global vars
Ansiterm ANSI;
#define HELPCOUNT_MAX 10
int8_t noHelpCount=HELPCOUNT_MAX;
extern RTC_DS3231 RTC;
extern DST DST;
extern DateTime time_now;
extern boolean DEBUG;

/*
void PROGMEMprint(const prog_uchar str[], uint8_t length)
{
  char c;
  if(!str) return;
  while(length--) {
    (c = pgm_read_byte(str++));
    Serial.print(c,BYTE);
  }
}
*/

void PROGMEMprint(const prog_uchar str[], uint8_t length)
{
  char c;
  if(!str) return;
  while(length--) {
    (c = pgm_read_byte(str++));
    Serial.print(c,BYTE);
  }
}

void PROGMEMprint(const prog_uchar str[])
{
  char c;
  if(!str) return;
  while((c = pgm_read_byte(str++)))
    Serial.print(c,BYTE);
}


#define BUF_MAX 30
#define toSetTimeLength 60
const prog_uchar toSetTime[] PROGMEM = "to set time enter in format \"set yy:mm:dd:hh:mm:ss\"\r\n";
const prog_char command_0[] PROGMEM = "set ";
const prog_char command_1[] PROGMEM = "debug 0";
const prog_char command_2[] PROGMEM = "debug 1";
PROGMEM const char *commands[] = 	   // change "string_table" name to suit
{
  command_0,
  command_1,
  command_2 };

void checkSerial() {
	if(!Serial.available()) {
		if(!noHelpCount--) {
			PROGMEMprint(toSetTime);
			noHelpCount=HELPCOUNT_MAX;
		}
	}
	else {
//		echoDebug(__FILE__,__LINE__,"Serial.available()\r\n");
		int index=0;
		char buffer[BUF_MAX]="\0";
		while(1) {
			//Wait for new char with timeout
			uint32_t timeout=150000;
			while(!Serial.available()) {
				timeout--;
				delay(1);
				if(!timeout) {
					//Timeout so cancel
//					Serial.println("Timeout");
					return;
				}
			}
			//read char into buffer
			buffer[index]=Serial.read();
			Serial.print(buffer[index]);
			if(buffer[index]==0x08) {
				//Backspace
				Serial.print(' ');
				Serial.print(0x08,BYTE);
				buffer[--index]='\0';
				index--;
			}
			if(buffer[index]=='\r') {
				buffer[index]='\0';
//				buffer[index]='x';
//				Serial.println(buffer);
				for(uint8_t i=0;i<3;i++) {
					char buffer_f[BUF_MAX]="\0";
//					Serial.println(buffer);
					strcpy_P(buffer_f, (char*)pgm_read_word(&(commands[i])));
#ifdef NOTHING
//					Serial.print("from flash: ");
//					Serial.println(buffer_f);
//					Serial.print("from serial: ");
//					Serial.println(buffer);
//					Serial.println(strcmp(buffer_f,buffer),DEC);
					for(uint8_t j=0;j<3;j++) {
						strcpy_P(buffer_f, (char*)pgm_read_word(&(commands[j])));
						for(uint8_t k=0;k<strlen(buffer_f);k++) {
							Serial.print("j=");
							Serial.print(j,DEC);
							//input
							Serial.print(",k=");
							Serial.print(k,DEC);
							Serial.print(",buffer[k]=");
							Serial.print(buffer[k]);
							//flash
							Serial.print(",buffer_f[k]=");
							Serial.println(buffer_f[k]);
						}
						Serial.println();
					}
#endif
					if(strncmp(buffer_f,buffer,strlen(buffer_f))==0) {
#ifdef NOTHING
						Serial.print("match: ");
						Serial.println(i,DEC);
						for(uint8_t k=0;k<strlen(buffer_f);k++) {
							//input
							Serial.print("k=");
							Serial.print(k,DEC);
							Serial.print(",buffer[k]=");
							Serial.print(buffer[k]);
							//flash
							Serial.print(",buffer_f[k]=");
							Serial.println(buffer_f[k]);
						}
#endif
						uint8_t start, y, m, d, hh, mm, ss;
//						Serial.println(strlen(buffer),DEC);
						switch (i) {
						//set 11:01:02:03:04:05
						case 0:
							if(strlen(buffer)>21) {
								Serial.println("too long");
								return;
							}
							if(strlen(buffer)<21) {
								Serial.println("too short");
								return;
							}
							start=4;
							//year,month,day,hour,minute,second
							y=(buffer[4]-48)*10+(buffer[5]-48);
							m=(buffer[7]-48)*10+(buffer[8]-48);
							d=(buffer[10]-48)*10+(buffer[11]-48);
							hh=(buffer[13]-48)*10+(buffer[14]-48);
							mm=(buffer[16]-48)*10+(buffer[17]-48);
							ss=(buffer[19]-48)*10+buffer[20]-48;
//							DateTime time = DateTime(y, m, d, hh, mm, ss);
//							time.toString();
							RTC.adjust(DateTime(y, m, d, hh, mm, ss));
							break;
						//"debug 0"
						case 1:
							Serial.println("NO DEBUG");
							DEBUG=false;
							break;
						//"debug 1"
						case 2:
							Serial.println("DEBUG");
							DEBUG=true;
							break;
						default:
							break;
						}
						return;

					}
				}
//				Serial.println("no match");
				return;
//				for
//				switch
//				return;
			}
			index++;
			if(index>=BUF_MAX) {
//				Serial.println("full");
				return;
			}
		}
	}
}


//echoDebug(__FILE__,__LINE__," bst correction: ");
void echoDebug(const char *file, const unsigned long line, const char *msg) {
	if(DEBUG) {
		Serial.print("F:");
		Serial.print(file);
		Serial.print(" Ln:");
		Serial.print(line);
		Serial.print(" ");
		Serial.print(msg);
		delay(100);
	}
}
