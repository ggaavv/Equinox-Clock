#ifndef Sunrise_h
#define Sunrise_h


#include "debug_frmwrk.h"

typedef enum {
	READ_SUNSET,
	READ_SUNRISE,
	READ_NOON
} Sunrise_Num;

//Northampton, England, Europe - Latitude/Longitude and Timezone
#define latitude 52
#define longitude -0.9
#define timezone 0

#define zenith 1.579522973054868	// Sunrise_Actual
//#define zenith 1.675516081914556	// Sunrise_Civil
//#define zenith 1.780235837034216	// Sunrise_Nautical
//#define zenith 1.884955592153876	//Sunrise_Astronomica

#define rd 57.295779513082322

// Sunrise returns minutes past 00:00:00 for the day
int Sunrise_Compute(unsigned char month, unsigned char  day, int rs);

#endif
