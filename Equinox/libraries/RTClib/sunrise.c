

#include "sunrise.h"
#include "rtc.h"


int Sunrise_Compute(unsigned char month, unsigned char  day, int rs) {
	float y, decl, eqt, ha, lat, lon, rd, tz;
	unsigned char a, riseHour, riseMinute, setHour, setMinute, nHour, nMinute ;
	int doy, minutes;

	// Calculate constants
	rd = 57.295779513082322;
	lat=latitude/rd;
	lon=-longitude/rd;
	tz=timezone;

	// approximate hour;
	a=6;
	if(!READ_SUNSET) a=18;

	// approximate day of year
	doy=(month-1)*30.3+day-1;

	// compute fractional year
	y=1.721420632104e-02*(doy+a/24);

	// compute equation of time
	eqt=229.18 * (0.000075+0.001868*cos(y)  -0.032077*sin(y) -0.014615*cos(y*2) -0.040849*sin(y* 2) );

	// compute solar declination
	decl=0.006918-0.399912*cos(y)+0.070257*sin(y)-0.006758*cos(y*2)+0.000907*sin(y*2)-0.002697*cos(y*3)+0.00148*sin(y*3);

	//compute hour angle
	ha=(  cos(zenith) / (cos(lat)*cos(decl)) -tan(lat) * tan(decl)  );
	if(fabs(ha)>1){// we're in the (ant)arctic and there is no rise(or set) today!
		nHour=255;
		riseHour=255;
		setHour=255;
		return -1;
	}
	ha=acos(ha);

	if((rs==READ_SUNSET)||rs==READ_SUNRISE){
		if(rs==READ_SUNSET) ha=-ha;
		// computes minutes into the day of the event
		minutes=720+4*(lon-ha)*rd-eqt;

		// convert from UTC back to our timezone
		minutes+= (tz*60);
		if(minutes<0) minutes+=1440;
		minutes%=1440;

		// stuff hour into month, minute into day
		if(rs==READ_SUNSET) {
			setHour=minutes/60;
			setMinute=minutes-setHour*60;
		}
		if(rs==READ_SUNRISE) {
			riseHour=minutes/60;
			riseMinute=minutes-riseHour*60;
		}
		return minutes;

	}else if(rs==READ_NOON){
		// computes minutes into the day of the event
		minutes=720+4*lon*rd-eqt;

		// convert from UTC back to our timezone
		minutes+= (tz*60);
		if(minutes<0) minutes+=1440;
		minutes%=1440;

		// stuff hour into month, minute into day
		nHour=minutes/60;
		nMinute=minutes-nHour*60;
		return minutes;
	}
}


#ifdef NOTHING
	Serial.print("sunrise GMT: ");
	Serial.print(sunriseHour(), DEC);
	Serial.print(":");
	Serial.println(sunriseMinute(), DEC);
	Serial.print("sunset GMT: ");
	Serial.print(sunsetHour(), DEC);
	Serial.print(":");
	Serial.println(sunsetMinute(), DEC);
#endif


