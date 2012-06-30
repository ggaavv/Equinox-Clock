#ifndef Sunrise_h
#define Sunrise_h


#include "debug_frmwrk.h"

typedef enum {
	READ_SUNSET = 0,
	READ_SUNRISE = 1,
	READ_NOON = 2
} Sunrise_Num;

//Northampton, England, Europe - Latitude/Longitude and Timezone
#define latitude 52
#define longitude -0.9
#define timezone 0

#define zenith 1.579522973054868	// Sunrise_Actual
//#define zenith 1.675516081914556	// Sunrise_Civil
//#define zenith 1.780235837034216	// Sunrise_Nautical
//#define zenith 1.884955592153876	//Sunrise_Astronomica

//#define SUNRISEN_BRIGHNESS 15
//#define SUNSET_BRIGHNESS 2

// Sunrise returns minutes
int Sunrise(unsigned char month, unsigned char  day, int rs);

#endif
