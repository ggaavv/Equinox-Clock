

#include "sunrise.h"
#include "rtc.h"


//Northampton, England, Europe - Latitude/Longitude and Timezone
const float latitude = 52;
const float longitude = -0.9;
const float timezone = 0;


unsigned char sunsetDim,sunriseBrighten;
float lat,lon, zenith, rd, tz;
unsigned char riseHour,setHour,nHour,riseMinute,setMinute,nMinute;

//extern void ht1632_screenBrightness(unsigned char brightness);

Sunrise(float latitude, float longitude, float timezone){
	rd=57.295779513082322;
	lat=latitude/rd;
  	lon=-longitude/rd;
  	tz=timezone;
  	
  	zenith=1.579522973054868; // default to actual times
  	riseHour=255;
  	riseMinute=0;
  	setHour=255;
  	setMinute=0;
  	nHour=255;
  	nMinute=0;
}

void Sunrise_Actual(){
	zenith=1.579522973054868;
}
void Sunrise_Civil(){
  zenith=1.675516081914556;
}

void Sunrise_Nautical(){
  zenith=1.780235837034216;
}

void Sunrise_Astronomical(){
  zenith=1.884955592153876;
}

void Sunrise_Rise(unsigned char  month, unsigned char  day){
  Sunrise_Compute(month, day, READ_SUNRISE);
}

void Sunrise_Set(unsigned char  month, unsigned char  day){
  Sunrise_Compute(month, day, READ_SUNSET);
}

void Sunrise_Noon(unsigned char  month, unsigned char  day){
	Sunrise_Compute(month, day, READ_NOON);
}

int Sunrise_Compute(unsigned char month, unsigned char  day, int rs) {
  float y, decl, eqt, ha;
  unsigned char a;
  int doy, minutes;
  
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

// hours and minutes must be GMT time??
unsigned char Sunrise_isSunRisen(unsigned char hour, unsigned char minute) {
	unsigned int riseM, setM, timeM;

	//Converts time into minutes for easy comparing
	riseM=riseMinute+(riseHour*60);
	setM=setMinute+(setHour*60);
	timeM=minute+(hour*60);

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
	if( (riseM<=timeM) && (timeM<setM) ) {
//		ht1632_screenBrightness(SUNRISEN_BRIGHNESS);
		sunriseBrighten=TRUE;
		sunsetDim=FALSE;
		return TRUE;
	}
	else {
//		ht1632_screenBrightness(SUNSET_BRIGHNESS);
		sunsetDim=TRUE;
		sunriseBrighten=FALSE;
		return FALSE;
	}
}

