// Code by JeeLabs http://news.jeelabs.org/code/
// Released to the public domain! Enjoy!

#include <avr/pgmspace.h>
#include "../Wire/Wire.h"
#include <WProgram.h>
#include "../RTClib/RTClib.h"
#include "../Ansi/Ansiterm.h"
#include "../Coms/Coms.h"


#define DSTEurope 1
#define DS3231_I2C_ADDRESS 0x68
#define SECONDS_PER_DAY 86400L
#define SECONDS_FROM_1970_TO_2000 946684800

extern boolean DEBUG;

// global clock objects
RTC_DS3231 RTC;
DST DST;
DateTime time_now;
DateTime time_prev;
extern Ansiterm ANSI;
extern boolean USE_ANSI;
extern boolean DEBUG;
extern boolean TWELVE_HOUR;

//DST daylight saving time
void DST::begin(void) {
  return;
}

#ifdef DSTEurope
//from: http://www.arduino.cc/cgi-bin/yabb2/YaBB.pl?num=1291637975
//see this link http://www.webexhibits.org/daylightsaving/i.html
//European Economic Community:
//Begin DST: Sunday March (31 - (5*y/4 + 4) mod 7) at 1h U.T.
//End DST: Sunday October (31 - (5*y/4 + 1) mod 7) at 1h U.T.
//Since 1996, valid through 2099
int8_t adjustForDST(DateTime time,boolean updateDST);
int8_t adjustForDST(uint32_t unixtime,boolean updateDST);

int8_t adjustForDST(uint32_t unixtime,boolean updateDST) {
//	echoDebug(__FILE__,__LINE__,"unixtime: ");
//	Serial.println(unixtime,DEC);
	DateTime time = DateTime(unixtime);
	if((DST.yearDST()==time.year())&&updateDST) {
		if(DST.bDST()<=unixtime && unixtime<=DST.eDST()) {
//			echoDebug(__FILE__,__LINE__,"DST offset: ");
//			Serial.println(DSTEurope,DEC);
			DST.setAdjDST(DSTEurope);
			return DSTEurope;
		}
		else {
			DST.setAdjDST(0);
			return 0;
		}
	}
//	Serial.print("Re-Calc DST for: ");
//	Serial.println(time.year());
//	delay(1000);
	int beginDSTMonth=3;
	int beginDSTHour=1;
	int endDSTMonth=10;
	int endDSTHour=1;
	// last sunday of march
	int beginDSTDay=  (31 - (5* time.year() /4 + 4) % 7);
	//last sunday of october
	int endDSTDay= (31 - (5 * time.year() /4 + 1) % 7);

	DateTime beginDST = DateTime(time.year(), beginDSTMonth, beginDSTDay, beginDSTHour, 0, 0);
	DateTime endDST = DateTime(time.year(), endDSTMonth, endDSTDay, endDSTHour, 0, 0);

	if(updateDST) {
		DST.setbDST(beginDST.unixtime());
		DST.seteDST(endDST.unixtime());
		DST.setyDST(time.year());
	}

	if(beginDST.unixtime()<=unixtime && unixtime<=endDST.unixtime()) {
//		echoDebug(__FILE__,__LINE__,"DST offset: ");
//		Serial.println(DSTEurope,DEC);
		DST.setAdjDST(DSTEurope);
		return DSTEurope;
	}
	else {
		DST.setAdjDST(0);
		return 0;
	}
}
int8_t adjustForDST(DateTime time,boolean updateDST) {
	return adjustForDST(time.unixtime(),updateDST);
}
#endif


static long time2long(uint16_t days, uint8_t h, uint8_t m, uint8_t s) {
    return ((days * 24L + h) * 60 + m) * 60 + s;
}

void DateTime::toString(void) const {

#ifdef USE_ANSI
		ANSI.eraseLine();
#endif
		if(TWELVE_HOUR) {
			if(hh>=12) {
				pm=true;
				if(hh<21)//if earlier than 9pm add a 0 to beginning
					Serial.print('0');
				Serial.print(hh-12, DEC);
			}
			else
				Serial.print(hh, DEC);
		}
		else {
			if(hh<10)
				Serial.print('0');
			Serial.print(hh, DEC);
		}
		Serial.print(':');
		if(mm<10)
			Serial.print('0');
		Serial.print(mm, DEC);
		Serial.print(':');
		if(ss<10)
			Serial.print('0');
		Serial.print(ss, DEC);
		if(TWELVE_HOUR) {
			if(pm)
				Serial.print(" pm");
			else
				Serial.print(" am");
		}
		if(!DST.adjDST())
			Serial.print(" GMT");
		Serial.println();
		delay(500);
	}
}

////////////////////////////////////////////////////////////////////////////////
// DST implementation


DST::DST(void) {
//    DSTbegin=0;
//    DSTbegin--;
//    DSTend=0;
//    DSTend--;
//    isDST=0;
//    yOffDST=0;
//    yOffDST--;
}

#ifdef NOTHING
//from: http://code.google.com/p/gfb/source/browse/arduino/DS3231/DS3231.pde?r=77

#include <Wire.h>

#define DS3231_I2C_ADDRESS 104

byte seconds, minutes, hours, day, date, month, year;
char weekDay[4];

float temp3231;
// Convert normal decimal numbers to binary coded decimal
byte decToBcd(byte val)
{
  return ( (val/10*16) + (val%10) );
}

#endif


