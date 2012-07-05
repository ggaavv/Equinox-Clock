/*
 * webpage.c
 *
 *  Created on: 3 Jul 2011
 *      Author: Gavin
 */

extern "C" {
	#include "libraries/RTClib/rtc.h"
	#include "debug_frmwrk.h"
}

#include "Webpage.h"
#include "WiServer.h"
#include <string.h>
#include <stdio.h>

const char body[] = "<body bgcolor=\"silver\"><br /><h1 align=\"center\"><font size=\"8\" align=\"center\"color=\"Red\">--==Equinox Clock==--</font></h1><br />";
const char time_format[] = "<p align=\"center\"><font size=\"6\" face=\"Comic Sans MS\" color=\"Red\">";
const char heading[] = "<html><head><title>--==Equinox Clock==-- by Gavin and Jamie Clarke</title></head>";
const char enter_date_format[] = "<p><font size=\"3\" color=\"Red\" face=\"Comic Sans MS\"> Enter Date: </font></p>";
const char submit_reset_buttons[] = "<input type=\"submit\" value=\"Send Date\" /><input type=\"reset\" value=\"Reset\" />";
const char form_open[] = "<form name=\"input\" action=\"\" method=\"get\" >";

// http://192.168.0.160/?dow=1&mm=0&yy=2012

bool home_page(char* URL){
	_DBG("[INFO]-home_page");_DBG(" (");_DBG(__FILE__);_DBG(":");_DBD16(__LINE__);_DBG(")\r\n");
	_DBG(URL);_DBG(" (");_DBG(__FILE__);_DBG(":");_DBD16(__LINE__);_DBG(")\r\n");

    // Check if the requested URL matches "/"
    if (strcmp(URL, "/") == 0) {
 //   	_DBG("[INFO]-strcmp(URL, /)");_DBG(" (");_DBG(__FILE__);_DBG(":");_DBD16(__LINE__);_DBG(")\r\n");
		uint16_t i,j;
		char DOW[DOW_LEN_MAX];
		strcpy(DOW,DayOfWeekName[GetDOW()]);
		uint8_t DOM = GetDOM();
		char DOM_s[2];
		sprintf(DOM_s,"%d",DOM);
//		_DBG(DOM);_DBG(" (");_DBG(__FILE__);_DBG(":");_DBD16(__LINE__);_DBG(")\r\n");
		char M[DOW_LEN_MAX];
		strcpy(M,Month_of_the_year[GetM()-1]);
		uint16_t Y = GetY();
		char Y_s[5];
		sprintf(Y_s,"%d",Y);
		uint8_t HH = GetHH();
		char HH_s[3];
		sprintf(HH_s,"%.2d",HH);
		uint8_t MM = GetMM();
		char MM_s[3];
		sprintf(MM_s,"%.2d",MM);
		uint8_t SS = GetSS();
		char SS_s[3];
		sprintf(SS_s,"%.2d",SS);
//		_DBG("[INFO]-strcmp(URL, /) Getting Variables finished");_DBG(" (");_DBG(__FILE__);_DBG(":");_DBD16(__LINE__);_DBG(")\r\n");

	// Use WiServer's print and println functions to write out the page content
		// Header
		WiServer.print(heading);
//		_DBG("[INFO]-strcmp(URL, /) Sending first line finished");_DBG(" (");_DBG(__FILE__);_DBG(":");_DBD16(__LINE__);_DBG(")\r\n");

		// Title
		WiServer.print(body);

		// Print Time
		WiServer.print(time_format);
		WiServer.print(DOW);		// Tuesday
		WiServer.print(", ");
		WiServer.print(DOM_s);		// 3
		WiServer.print(" ");
		WiServer.print(M);			// July
		WiServer.print(" ");
		WiServer.print(Y_s);		// 2012
		WiServer.print(", ");
		WiServer.print(HH_s);		// 8:xx:xx
		WiServer.print(":");
		WiServer.print(MM_s);		// xx:22:xx
		WiServer.print(":");
		WiServer.print(SS_s);		// xx:xx:55
		WiServer.print("</font></p><br /><br />");
//		_DBG("[INFO]-strcmp(URL, /) Dates printed");_DBG(" (");_DBG(__FILE__);_DBG(":");_DBD16(__LINE__);_DBG(")\r\n");

		// Date drop down selection box
		WiServer.print(form_open);
		WiServer.print(enter_date_format);
		WiServer.print("<select name=dow size=1>");
		// Day of the Month
		for (i = 1; i < 3; i++){
			WiServer.print("<option name=dom value=");
			WiServer.print(i);
			WiServer.print("> ");
			WiServer.print(i);
			WiServer.print(" </option>");
		}
//		_DBG("[INFO]-strcmp(URL, /) First dropdown box finished");_DBG(" (");_DBG(__FILE__);_DBG(":");_DBD16(__LINE__);_DBG(")\r\n");
		WiServer.print("</select>");
		// Month
		WiServer.print("<select name=mm size=1>");
		for (i = 0; i < NUM_MONTHS; i++){
//			_DBD(i);_DBG(" (");_DBG(__FILE__);_DBG(":");_DBD16(__LINE__);_DBG(")\r\n");
			WiServer.print("<option name=mm value=");
			WiServer.print(i);
			WiServer.print("> ");
			for (j = 0; j < 3; j++){
				WiServer.print(Month_of_the_year[i][j]);
			}
			WiServer.print(" </option>");
//			_DBG("[INFO]-strcmp(URL, /) Month sent");_DBG(" (");_DBG(__FILE__);_DBG(":");_DBD16(__LINE__);_DBG(")\r\n");
		}
		WiServer.print("</select>");
//		_DBG("[INFO]-strcmp(URL, /) Half way");_DBG(" (");_DBG(__FILE__);_DBG(":");_DBD16(__LINE__);_DBG(")\r\n");
		// Year
		WiServer.print("<select name=yy size=1>");
		for (i = Y; i < (Y+3); i++){
			WiServer.print("<option name=yy value=");
			// Calculate year
			sprintf(Y_s,"%d",i);
			WiServer.print(Y_s);
			WiServer.print("> ");
			WiServer.print(Y_s);
			WiServer.print(" </option>");
		}
		WiServer.print("</select><br />");
		// Save + Reset
		WiServer.print(submit_reset_buttons);
		WiServer.print("</form>");
//		_DBG("[INFO]-strcmp(URL, /) END");_DBG(" (");_DBG(__FILE__);_DBG(":");_DBD16(__LINE__);_DBG(")\r\n");




		WiServer.print("</body></html>");
		return true;
    }

    // Read input
    if (URL[2] = "/\?") {
    	_DBG("[INFO]-strcmp(URL, /?) END");_DBG(" (");_DBG(__FILE__);_DBG(":");_DBD16(__LINE__);_DBG(")\r\n");
    	// ?dom=01&mm=Jan&yy=2010
//    	for (i = 0; i < sizeof(URL); i++){
 //   		if( URL[i] == d && URL[i+1] == o && URL[i+2] == m) {
 //   			SetY(strtoul (URL[i+4]));
//    		}
 //   	}
        return true;
    }

	// URL not found
    _DBG("[INFO]-strcmp(URL, ?) URL not found");_DBG(" (");_DBG(__FILE__);_DBG(":");_DBD16(__LINE__);_DBG(")\r\n");
	return false;
};


