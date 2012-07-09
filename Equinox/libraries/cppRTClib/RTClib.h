// Code by JeeLabs http://news.jeelabs.org/code/
// Released to the public domain! Enjoy!

#ifndef RTClib_h
#define RTClib_h


// Simple general-purpose date/time class (no TZ / DST / leap second handling!)
class DateTime {
public:
    DateTime (uint32_t t =0);
    DateTime (uint16_t year, uint8_t month, uint8_t day,
                uint8_t hour, uint8_t min, uint8_t sec);
    DateTime (uint16_t year, uint8_t month, uint8_t dayOf, uint8_t day,
                uint8_t hour, uint8_t min, uint8_t sec);
    DateTime (const char* date, const char* time);
    uint16_t year() const       { return 2000 + yOff; }
    uint8_t month() const       { return m; }
    uint8_t day() const         { return d; }
    uint8_t hour() const        { return hh; }
    uint8_t hour12() const        { if(hh>12)return hh-12;else return hh; }
    uint8_t minute() const      { return mm; }
    uint8_t second() const      { return ss; }
//    uint8_t dayOfWeek() const;
    uint8_t dayOfWeekManual() const;
    uint8_t dayOfWeek() const	{ return dow; }
    void toString() const;

    // 32-bit times as seconds since 1/1/2000
    long secondstime() const;   
    // 32-bit times as seconds since 1/1/1970
    uint32_t unixtime(void) const;

protected:
    uint8_t yOff, m, d, hh, mm, ss, dow;
};

class DST {
public:
	DST(void);
	static void begin(void);
//For DST
    void setDST (uint8_t setDST) { isDST = setDST; }
    boolean isDSTset() const      { if(isDST) return true; else return false; }

    //start of DST
    void setbDST (uint32_t setbDST) { DSTbegin = setbDST; }
    uint32_t bDST() const      { return DSTbegin; }

    //end of DST
    void seteDST (uint32_t seteDST) { DSTend = seteDST; }
    uint32_t eDST() const      { return DSTend; }

    //year DST has been calculated for to save recalculating every RTC read.
    void setyDST (uint16_t setyOffDST) { yOffDST = setyOffDST - 2000; }
    uint16_t yearDST() const      { return 2000 + yOffDST; }

    //dst adjust ammount
    void setAdjDST (int8_t dstDiff) { dstAdj = dstDiff; }
    int8_t adjDST() const      { return dstAdj; }

protected:
    uint32_t DSTbegin, DSTend; //DST begin and end
    uint8_t isDST, yOffDST; //DST true/false, year off DST calc
    int8_t dstAdj;
};

// RTC based on the DS3231 chip connected via I2C and the Wire library
class RTC_DS3231 {
public:
  static uint8_t begin(void);
    static void adjust(const DateTime& dt);
    uint8_t isrunning(void);
//    static DateTime now();
    static DateTime now(int8_t dst_diff);
    float getTemp();
};

// RTC using the internal millis() clock, has to be initialized before use
// NOTE: this clock won't be correct once the millis() timer rolls over (>49d?)
class RTC_Millis {
public:
    static void begin(const DateTime& dt) { adjust(dt); }
    static void adjust(const DateTime& dt);
    static DateTime now();

protected:
    static long offset;
};

int8_t adjustForDST(DateTime time,boolean updateDST);
int8_t adjustForDST(uint32_t unixtime,boolean updateDST);

#endif
