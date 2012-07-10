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

#ifdef DEBUG
//	echoDebug(__FILE__,__LINE__,"Setting DST:\r\n");
//	Serial.print("\tbeginDST.unixtime(): ");
	Serial.println(beginDST.unixtime(),DEC);
	delay(100);

//	Serial.print("\tunixtime: ");
	Serial.println(unixtime,DEC);
	delay(100);

//	Serial.print("\tendDST.unixtime(): ");
	Serial.println(endDST.unixtime(),DEC);
	delay(100);

//	Serial.print("\tbeginDST.year(): ");
	Serial.println(beginDST.year(),DEC);
	delay(100);
#endif

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

////////////////////////////////////////////////////////////////////////////////
// utility code, some of this could be exposed in the DateTime API if needed

static uint8_t daysInMonth [] PROGMEM = { 31,28,31,30,31,30,31,31,30,31,30,31 };

#define DOW_LEN 3
#define NUM_DAYS_OF_WEEK 7
#define DAYS_OF_WEEK_MAX_LEN 10
static unsigned char DayOfWeekName[NUM_DAYS_OF_WEEK][DAYS_OF_WEEK_MAX_LEN] PROGMEM = {
"Sunday",
"Monday",
"Tuesday",
"Wednesday",
"Thursday",
"Friday",
"Saturday"
};

// number of days since 2000/01/01, valid for 2001..2099
static uint16_t date2days(uint16_t y, uint8_t m, uint8_t d) {
    if (y >= 2000)
        y -= 2000;
    uint16_t days = d;
    for (uint8_t i = 1; i < m; ++i)
        days += pgm_read_byte(daysInMonth + i - 1);
    if (m > 2 && y % 4 == 0)
        ++days;
    return days + 365 * y + (y + 3) / 4 - 1;
}

static long time2long(uint16_t days, uint8_t h, uint8_t m, uint8_t s) {
    return ((days * 24L + h) * 60 + m) * 60 + s;
}

////////////////////////////////////////////////////////////////////////////////
// DateTime implementation - ignores time zones and DST changes
// NOTE: also ignores leap seconds, see http://en.wikipedia.org/wiki/Leap_second

DateTime::DateTime (uint32_t t) {
  t -= SECONDS_FROM_1970_TO_2000;    // bring to 2000 timestamp from 1970

    ss = t % 60;
    t /= 60;
    mm = t % 60;
    t /= 60;
    hh = t % 24;
    uint16_t days = t / 24;
    uint8_t leap;
    for (yOff = 0; ; ++yOff) {
        leap = yOff % 4 == 0;
        if (days < 365 + leap)
            break;
        days -= 365 + leap;
    }
    for (m = 1; ; ++m) {
        uint8_t daysPerMonth = pgm_read_byte(daysInMonth + m - 1);
        if (leap && m == 2)
            ++daysPerMonth;
        if (days < daysPerMonth)
            break;
        days -= daysPerMonth;
    }
    d = days + 1;
    dow = dayOfWeekManual();
}

//old
DateTime::DateTime (uint16_t year, uint8_t month, uint8_t day, uint8_t hour, uint8_t min, uint8_t sec) {
    if (year >= 2000)
        year -= 2000;
    yOff = year;
    m = month;
    d = day;
    hh = hour;
    mm = min;
    ss = sec;
    dow = dayOfWeekManual();
}


DateTime::DateTime (uint16_t year, uint8_t month, uint8_t dayOf, uint8_t day, uint8_t hour, uint8_t min, uint8_t sec) {
    if (year >= 2000)
        year -= 2000;
    yOff = year;
    m = month;
    d = day;
    dow = dayOf;
    hh = hour;
    mm = min;
    ss = sec;
}

static uint8_t conv2d(const char* p) {
    uint8_t v = 0;
    if ('0' <= *p && *p <= '9')
        v = *p - '0';
    return 10 * v + *++p - '0';
}

// A convenient constructor for using "the compiler's time":
//   DateTime now (__DATE__, __TIME__);
// NOTE: using PSTR would further reduce the RAM footprint
DateTime::DateTime (const char* date, const char* time) {
    // sample input: date = "Dec 26 2009", time = "12:34:56"
    yOff = conv2d(date + 9);
    // Jan Feb Mar Apr May Jun Jul Aug Sep Oct Nov Dec 
    switch (date[0]) {
        case 'J': m = date[1] == 'a' ? 1 : m = date[2] == 'n' ? 6 : 7; break;
        case 'F': m = 2; break;
        case 'A': m = date[2] == 'r' ? 4 : 8; break;
        case 'M': m = date[2] == 'r' ? 3 : 5; break;
        case 'S': m = 9; break;
        case 'O': m = 10; break;
        case 'N': m = 11; break;
        case 'D': m = 12; break;
    }
    d = conv2d(date + 4);
    hh = conv2d(time);
    mm = conv2d(time + 3);
    ss = conv2d(time + 6);
    dow = dayOfWeekManual();
}

//uint8_t DateTime::dayOfWeek() const {
uint8_t DateTime::dayOfWeekManual() const {
    uint16_t day = date2days(yOff, m, d);
    return (day + 6) % 7; // Jan 1, 2000 is a Saturday, i.e. returns 6
}

uint32_t DateTime::unixtime(void) const {
  uint32_t t;
  uint16_t days = date2days(yOff, m, d);
  t = time2long(days, hh, mm, ss);
  t += SECONDS_FROM_1970_TO_2000;  // seconds from 1970 to 2000
  return t;
}

void DateTime::toString(void) const {
	if(DEBUG) {
//		echoDebug(__FILE__,__LINE__,"toString:\r\n");
		boolean pm=false;
		uint8_t dows=dow;
#ifdef USE_ANSI
		ANSI.home();
		ANSI.eraseLine();
#endif
		if(dow==7)
			dows=0;
		PROGMEMprint(DayOfWeekName[dows], 3);
		Serial.print(' ');
#ifdef TimeShowMSBFirst
		Serial.print(yOff, DEC);
		Serial.print(':');
		if(m<10)
			Serial.print('0');
		Serial.print(m, DEC);
		Serial.print(':');
		if(d<10)
			Serial.print('0');
		Serial.print(d, DEC);
		Serial.print(':');
#else
		if(d<10)
			Serial.print('0');
		Serial.print(d, DEC);
		Serial.print('/');
		if(m<10)
			Serial.print('0');
		Serial.print(m, DEC);
		Serial.print('/');
//		Serial.println(yOff+2000, DEC);
		Serial.println(yOff, DEC);
#endif

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

////////////////////////////////////////////////////////////////////////////////
// RTC_DS3231 implementation

static uint8_t bcd2bin (uint8_t val) { return val - 6 * (val >> 4); }
static uint8_t bin2bcd (uint8_t val) { return val + 6 * (val / 10); }

uint8_t RTC_DS3231::begin(void) {
  return 1;
}

uint8_t RTC_DS3231::isrunning(void) {
  Wire.beginTransmission(DS3231_I2C_ADDRESS);
  Wire.send(0);
  Wire.endTransmission();

  Wire.requestFrom(DS3231_I2C_ADDRESS, 1);
  uint8_t ss = Wire.receive();
  return !(ss>>7);
}

void RTC_DS3231::adjust(const DateTime& dt) {
#ifdef DEBUG
//	echoDebug(__FILE__,__LINE__,"RTC_DS3231::adjust\r\n");
#endif
	int8_t DSTadjustment=adjustForDST(dt,true);
	uint32_t unixtime = dt.unixtime();
	DateTime new_dt;
	if(DSTadjustment!=0) {
		unixtime=unixtime-(DSTadjustment*3600);
		new_dt=DateTime(unixtime);
	}
	else {
		new_dt=dt;
	}
#ifdef DEBUG
//	echoDebug(__FILE__,__LINE__,"new time to RTC:\r\n");
//	new_dt.toString();
#endif
    Wire.beginTransmission(DS3231_I2C_ADDRESS);
    Wire.send(0);
    Wire.send(bin2bcd(new_dt.second()));
    Wire.send(bin2bcd(new_dt.minute()));
	Wire.send(bin2bcd(new_dt.hour()));
	Wire.send(bin2bcd(new_dt.dayOfWeek()));
    Wire.send(bin2bcd(new_dt.day()));
    Wire.send(bin2bcd(new_dt.month()));
    Wire.send(bin2bcd(new_dt.year() - 2000));
    Wire.send(0);
    Wire.endTransmission();
}

//DateTime RTC_DS3231::now() {
//	now(0);
//}
DateTime RTC_DS3231::now(int8_t dst_diff) {
	Wire.beginTransmission(DS3231_I2C_ADDRESS);
	Wire.send(0);
	Wire.endTransmission();

	Wire.requestFrom(DS3231_I2C_ADDRESS, 7);
	if(Wire.available()){
		uint8_t ss = bcd2bin(Wire.receive() & 0x7F);
		uint8_t mm = bcd2bin(Wire.receive());
		uint8_t hh = bcd2bin(Wire.receive());
		uint8_t dow = bcd2bin(Wire.receive());
		uint8_t d = bcd2bin(Wire.receive());
		uint8_t m = bcd2bin(Wire.receive());
		uint16_t y = bcd2bin(Wire.receive()) + 2000;
		DateTime time = DateTime(y, m, dow, d, hh, mm, ss);
		time = DateTime(time.unixtime()+dst_diff*3600);
		return time;
  }
  else {
#if(DEBUG)
	  Serial.println("!Wire.available()");
#endif
	  return 0;
  }

//#if(DEBUG)
//		echoDebug(__FILE__,__LINE__,"time from RTC:\r\n");
//		DateTime time = DateTime(y, m, dow, d, (hh + dst_diff), mm, ss);
//		time.toString();
//#endif
//DST adjusted every hour		return DateTime(time.unixtime()+adjustForDST(time.unixtime(),true)*3600);
//		return DateTime (y, m, dow, d, hh, mm, ss);
//		return DateTime(time.unixtime()+(dst_diff*3600));
//  return DateTime (y, m, d, hh, mm, ss);
}

float RTC_DS3231::getTemp()
{
	float temp;
	int8_t tMSB, tLSB;
  //temp registers (11h-12h) get updated automatically every 64s
  Wire.beginTransmission(DS3231_I2C_ADDRESS);
  Wire.send(0x11);
  Wire.endTransmission();
  Wire.requestFrom(DS3231_I2C_ADDRESS, 2);

  if(Wire.available()) {
    tMSB = Wire.receive(); //2's complement int portion
    tLSB = Wire.receive(); //fraction portion

    temp = (tMSB & B01111111); //do 2's math on Tmsb
    temp += ( (tLSB >> 6) * 0.25 ); //only care about bits 7 & 8
  }
  else {
    //oh noes, no data!
  }

  return temp;
}

////////////////////////////////////////////////////////////////////////////////
// RTC_Millis implementation

long RTC_Millis::offset = 0;

void RTC_Millis::adjust(const DateTime& dt) {
    offset = dt.unixtime() - millis() / 1000;
}

DateTime RTC_Millis::now() {
  return (uint32_t)(offset + millis() / 1000);
}

////////////////////////////////////////////////////////////////////////////////

#ifdef NOTHING
//from: http://code.google.com/p/gfb/source/browse/arduino/DS3231/DS3231.pde?r=77

#include <Wire.h>

#define DS3231_I2C_ADDRESS 104

byte seconds, minutes, hours, day, date, month, year;
char weekDay[4];

byte tMSB, tLSB;
float temp3231;

void setup()
{
  Wire.begin();
  Serial.begin(9600);
}

void loop()
{

  watchConsole();


  get3231Date();

  Serial.print(weekDay); Serial.print(", "); Serial.print(date, DEC); Serial.print("/"); Serial.print(month, DEC); Serial.print("/"); Serial.print(year, DEC); Serial.print(" - ");
  Serial.print(hours, DEC); Serial.print(":"); Serial.print(minutes, DEC); Serial.print(":"); Serial.print(seconds, DEC);

  Serial.print(" - Temp: "); Serial.println(get3231Temp());

  delay(1000);
}

// Convert normal decimal numbers to binary coded decimal
byte decToBcd(byte val)
{
  return ( (val/10*16) + (val%10) );
}

void watchConsole()
{
  if (Serial.available()) {      // Look for char in serial queue and process if found
    if (Serial.read() == 84) {      //If command = "T" Set Date
      set3231Date();
      get3231Date();
      Serial.println(" ");
    }
  }
}

void set3231Date()
{
//T(sec)(min)(hour)(dayOfWeek)(dayOfMonth)(month)(year)
//T(00-59)(00-59)(00-23)(1-7)(01-31)(01-12)(00-99)
//Example: 02-Feb-09 @ 19:57:11 for the 3rd day of the week -> T1157193020209

  seconds = (byte) ((Serial.read() - 48) * 10 + (Serial.read() - 48)); // Use of (byte) type casting and ascii math to achieve result.
  minutes = (byte) ((Serial.read() - 48) *10 +  (Serial.read() - 48));
  hours   = (byte) ((Serial.read() - 48) *10 +  (Serial.read() - 48));
  day     = (byte) (Serial.read() - 48);
  date    = (byte) ((Serial.read() - 48) *10 +  (Serial.read() - 48));
  month   = (byte) ((Serial.read() - 48) *10 +  (Serial.read() - 48));
  year    = (byte) ((Serial.read() - 48) *10 +  (Serial.read() - 48));
  Wire.beginTransmission(DS3231_I2C_ADDRESS);
  Wire.send(0x00);
  Wire.send(decToBcd(seconds));
  Wire.send(decToBcd(minutes));
  Wire.send(decToBcd(hours));
  Wire.send(decToBcd(day));
  Wire.send(decToBcd(date));
  Wire.send(decToBcd(month));
  Wire.send(decToBcd(year));
  Wire.endTransmission();
}


void get3231Date()
{
  // send request to receive data starting at register 0
  Wire.beginTransmission(DS3231_I2C_ADDRESS); // 104 is DS3231 device address
  Wire.send(0x00); // start at register 0
  Wire.endTransmission();
  Wire.requestFrom(DS3231_I2C_ADDRESS, 7); // request seven bytes

  if(Wire.available()) {
    seconds = Wire.receive(); // get seconds
    minutes = Wire.receive(); // get minutes
    hours   = Wire.receive();   // get hours
    day     = Wire.receive();
    date    = Wire.receive();
    month   = Wire.receive(); //temp month
    year    = Wire.receive();

    seconds = (((seconds & B11110000)>>4)*10 + (seconds & B00001111)); // convert BCD to decimal
    minutes = (((minutes & B11110000)>>4)*10 + (minutes & B00001111)); // convert BCD to decimal
    hours   = (((hours & B00110000)>>4)*10 + (hours & B00001111)); // convert BCD to decimal (assume 24 hour mode)
    day     = (day & B00000111); // 1-7
    date    = (((date & B00110000)>>4)*10 + (date & B00001111)); // 1-31
    month   = (((month & B00010000)>>4)*10 + (month & B00001111)); //msb7 is century overflow
    year    = (((year & B11110000)>>4)*10 + (year & B00001111));
  }
  else {
    //oh noes, no data!
  }

  switch (day) {
    case 1:
      strcpy(weekDay, "Sun");
      break;
    case 2:
      strcpy(weekDay, "Mon");
      break;
    case 3:
      strcpy(weekDay, "Tue");
      break;
    case 4:
      strcpy(weekDay, "Wed");
      break;
    case 5:
      strcpy(weekDay, "Thu");
      break;
    case 6:
      strcpy(weekDay, "Fri");
      break;
    case 7:
      strcpy(weekDay, "Sat");
      break;
  }
}

float get3231Temp()
{
  //temp registers (11h-12h) get updated automatically every 64s
  Wire.beginTransmission(DS3231_I2C_ADDRESS);
  Wire.send(0x11);
  Wire.endTransmission();
  Wire.requestFrom(DS3231_I2C_ADDRESS, 2);

  if(Wire.available()) {
    tMSB = Wire.receive(); //2's complement int portion
    tLSB = Wire.receive(); //fraction portion

    temp3231 = (tMSB & B01111111); //do 2's math on Tmsb
    temp3231 += ( (tLSB >> 6) * 0.25 ); //only care about bits 7 & 8
  }
  else {
    //oh noes, no data!
  }

  return temp3231;
}

#endif


