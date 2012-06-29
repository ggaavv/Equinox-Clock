#ifndef Sunrise_h
#define Sunrise_h


#include "debug_frmwrk.h"


#define READ_SUNSET 1
#define READ_SUNRISE 2
#define READ_NOON 3

#define SUNRISEN_BRIGHNESS 15
#define SUNSET_BRIGHNESS 2


  Sunrise(float, float, float);
  void Actual();
  void Civil();
  void Nautical();
  void Astronomical();
  void Rise(unsigned char ,unsigned char );
  void Set(unsigned char ,unsigned char );
  void Noon(unsigned char ,unsigned char );
  unsigned char sunsetHour();// const	{ return setHour; } ;
  unsigned char sunsetMinute();// const	{ return setMinute; } ;
  unsigned char sunriseHour();// const	{ return riseHour; } ;
  unsigned char sunriseMinute();// const	{ return riseMinute; } ;
  unsigned char noonHour();// const	{ return nHour; } ;
  unsigned char noonMinute();// const	{ return nMinute; } ;
  unsigned char isSunRisen(unsigned char hour, unsigned char minute); //accurate to 1 hour
  
  unsigned char sunsetDim,sunriseBrighten;

  int Sunrise_Compute(unsigned char month, unsigned char  day, int rs);
  float lat,lon, zenith, rd, tz;
  unsigned char riseHour,setHour,nHour,riseMinute,setMinute,nMinute;


#endif
