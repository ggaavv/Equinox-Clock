/*
 * webpage.c
 *
 *  Created on: 3 Jul 2011
 *      Author: Gavin
 */

extern "C" {
	#include "debug_frmwrk.h"
	#include "rtc.h"
}

#include "Webpage.h"
#include "WiServer.h"
#include <stdio.h>
#include <string.h>
#include <stdlib.h>

const char DOCTYPE[] = "<!DOCTYPE html PUBLIC \"-//W3C//DTD HTML 4.01//EN\" \"http://www.w3.org/TR/html4/DTD/html4-strict.dtd\">\n";
const char meta[] = "<meta http-equiv=\"Content-Type\" content=\"text/html; charset=iso-8859-1\">\n";
const char style_open[] = "<STYLE type=\"text/css\">\n";
const char style_close[] = "</STYLE>\n";
const char body_style[] = "BODY{font-family:'Comic Sans MS',cursive; color:Red; background-color:Silver; text-align:center; font-size:Large;}\n";
const char body_open[] = "<BODY>\n";
const char body_close[] = "</BODY>\n";
const char p_style[] = "P{font-family:'Comic Sans MS',cursive; color:Red; text-align:center; font-size:Large;}\n";
const char p_open[] = "<P>\n";
const char p_close[] = "</P>\n";
const char h1_style[] = "H1{font-family:'Comic Sans MS',cursive; color:Red; alink:Red; link:Red; vlink:Red; text-align:center; font-size:xx-large;}\n";
const char h1_open[] = "<H1>\n";
const char h1_close[] = "</H1>\n";
const char h2_style[] = "H2{font-family:'Comic Sans MS',cursive; color:Red; text-align:center; font-size:x-Large;}\n";
const char h2_open[] = "<H2>\n";
const char h2_close[] = "</H2>\n";

const char Heading1[] = "<a href=\"/\">--==Equinox Clock==--</a>";
const char line_break[] = "<br />\n";
const char html_open[] = "<HTML>\n";
const char html_close[] = "</HTML>\n";
const char head_open[] = "<HEAD>\n";
const char head_close[] = "</HEAD>\n";
const char title[] = "<title>--==Equinox Clock==-- by Gavin and Jamie Clarke</title>\n";
const char enter_date_format[] = " Enter Date: \n";
const char enter_time_format[] = " Enter Time: \n";
const char enter_DST_format[] = " Summertime (not yet implimented) \n";
const char enter_UTC_format[] = " Timezone   Summertime \n";
const char submit_reset_buttons[] = "<input type=\"submit\" value=\"Send Date\" />\n<input type=\"reset\" value=\"Reset\" />\n";
const char form_open[] = "<form name=\"input\" action=\"/date\" method=\"get\" >\n";
const char form_close[] = "\n</form>";

//bool home_page(char* URL_REQUESTED){
bool home_page(char* URL){
//	_DBG(URL);_DBG(" (");_DBG(__FILE__);_DBG(":");_DBD16(__LINE__);_DBG(")\r\n");

	// Check if the requested URL matches is enter date
	if (strncmp(URL,"/date?",6) == 0){
		uint16_t year = 0;
		uint8_t dom = 0, month = 0, hh = 0, mm = 0, ss = 0, st = 0;
		// /date?dom=01&mm=01&yy=2010
		// size set to 50 but should break before this point
		uint8_t dom_set = 0, m_set = 0, y_set = 0, hh_set = 0, mm_set = 0, ss_set = 0, st_set = 0;
		for (uint8_t i=6;i<100;i++){
			if (!dom_set && strncmp(URL+i,"dom",3)==0){
				dom = strtoul (URL+i+4,NULL,10);
				dom_set=1;
			} else if (!m_set && strncmp(URL+i,"month",5)==0){
				month = strtoul (URL+i+6,NULL,10);
				m_set=1;
			} else if (!year && strncmp(URL+i,"yy",2)==0){
				year = strtoul (URL+i+3,NULL,10);
				y_set=1;
			} else if (!hh_set && strncmp(URL+i,"hh",2)==0){
				hh = strtoul (URL+i+3,NULL,10);
				hh_set=1;
			}else if (!mm_set && strncmp(URL+i,"mm",2)==0){
				mm = strtoul (URL+i+3,NULL,10);
				mm_set=1;
			}else if (!ss_set && strncmp(URL+i,"ss",2)==0){
				ss = strtoul (URL+i+3,NULL,10);
				ss_set=1;
			}else if (!st_set && strncmp(URL+i,"st",2)==0){
				st = strtoul (URL+i+3,NULL,10);
				st_set=1;
			}
			if (dom_set&&m_set&&y_set&&hh_set&&mm_set&&ss_set&&st_set)break;
		}
     	RTC_time_SetTime(year, month, dom, hh, mm, ss, st);
	}

	// Check if the requested URL matches "/"
	if (strncmp(URL, "/",1) == 0) {
		// Get all Date/Times
		// Days of the Week
		uint8_t DOW = GetDOW();
		// Days of the Month
		uint8_t DOM = GetDOM();
		char DOM_s[2];
		sprintf(DOM_s,"%.2d",DOM);
		// Month
		uint8_t M = GetM();
		// Year
		uint16_t Y = GetY();
		char Y_s[5];
		sprintf(Y_s,"%.4d",Y);
		// Hours
		uint8_t hh = GetHH();
		char hh_s[2];
		sprintf(hh_s,"%.2d",hh);
		// Minutes
		uint8_t mm = GetMM();
		char mm_s[2];
		sprintf(mm_s,"%.2d",mm);
		// Seconds
		uint8_t ss = GetSS();
		char ss_s[2];
		sprintf(ss_s,"%.2d",ss);
		// Sunrise
		char Sunrise_s[9];
		GetSunRiseHH_MM_SS(&Sunrise_s[0]);
		// Noon
		char Noon_s[9];
		GetNoonHH_MM_SS(&Noon_s[0]);
		// Sunset
		char Sunset_s[9];
		GetSunSetHH_MM_SS(&Sunset_s[0]);
		// Summertime
		int8_t dst = GetDST_correction();
		char dst_s[4];
		sprintf(dst_s,"%.2d",dst);

	// Use WiServer's print and println functions to write out the page content
		// DOCTYPE
		WiServer.print(DOCTYPE);
		WiServer.print(html_open);
		WiServer.print(head_open);
		// Title
		WiServer.print(title);
		WiServer.print(style_open);
		WiServer.print(body_style);
		WiServer.print(p_style);
		WiServer.print(h1_style);
		WiServer.print(h2_style);
		WiServer.print(style_close);
		WiServer.print(head_close);
		// Header
		WiServer.print(h1_open);
		WiServer.print(Heading1);
		WiServer.print(h1_close);

		// Title
		WiServer.print(body_open);

		// Print Time
		WiServer.print(h2_open);
		WiServer.print(DayOfWeekName[DOW]);		// Tuesday
		WiServer.print(", ");
		WiServer.print(DOM_s);		// 3
		WiServer.print(" ");
		WiServer.print(Month_of_the_year[M-1]);			// July
		WiServer.print(" ");
		WiServer.print(Y_s);		// 2012
		WiServer.print(", ");
		WiServer.print(hh_s);		// 8:xx:xx
		WiServer.print(":");
		WiServer.print(mm_s);		// xx:22:xx
		WiServer.print(":");
		WiServer.print(ss_s);		// xx:xx:55
		WiServer.print(h2_close);

		// Print Sunrise+Noon+Sunset
		WiServer.print(p_open);
		WiServer.print("Sunrise ");
		WiServer.print(Sunrise_s);
		WiServer.print("   Noon ");
		WiServer.print(Noon_s);
		WiServer.print("   Sunset ");
		WiServer.print(Sunset_s);
		WiServer.print(p_close);

		// Date drop down selection box
		WiServer.print(form_open);
		WiServer.print(p_open);
		WiServer.print(enter_date_format);
		WiServer.print("<select name=dom>\n");
		// Day of the Month
		for (uint8_t i = 1; i < 32; i++){
			WiServer.print("<option");
			if (i==DOM){
				WiServer.print(" selected");
			}
			WiServer.print(">");
			char i_s[2];
			sprintf(i_s,"%.2d",i);
			WiServer.print(i_s);
			WiServer.print("</option>\n");
		}
//		_DBG("[INFO]-strcmp(URL, /) First dropdown box finished");_DBG(" (");_DBG(__FILE__);_DBG(":");_DBD16(__LINE__);_DBG(")\r\n");
		WiServer.print("</select>\n");
		// Month
		WiServer.print("<select name=month>\n");
		for (uint8_t i = 1; i <= NUM_MONTHS; i++){
			WiServer.print("<option value=");
			char i_s[2];
			sprintf(i_s,"%.2d",i);
			WiServer.print(i_s);
			if (i==M){
				WiServer.print(" selected");
			}
			WiServer.print("> ");
			for (uint16_t j = 0; j < 3; j++){
				WiServer.print(Month_of_the_year[i-1][j]);
			}
			WiServer.print(" </option>\n");
		}
		WiServer.print("</select>\n");
//		_DBG("[INFO]-strcmp(URL, /) Half way");_DBG(" (");_DBG(__FILE__);_DBG(":");_DBD16(__LINE__);_DBG(")\r\n");
		// Year
		WiServer.print("<select name=yy>\n");
		for (uint16_t i = Y; i < (Y+12); i++){
			WiServer.print("<option");
			if (i==Y){
				WiServer.print(" selected");
			}
			WiServer.print(">");
			// Calculate year
			sprintf(Y_s,"%.4d",i);
			WiServer.print(Y_s);
			WiServer.print("</option>\n");
		}
		WiServer.print("</select>\n");
		WiServer.print(line_break);
		WiServer.print(enter_time_format);
		// Hours
		WiServer.print("<select name=hh>\n");
		for (uint8_t i = 0; i < 24; i++){
			WiServer.print("<option");
			if (i==hh){
				WiServer.print(" selected");
			}
			WiServer.print(">");
			// Calculate year
			sprintf(hh_s,"%.2d",i);
			WiServer.print(hh_s);
			WiServer.print("</option>\n");
		}
		WiServer.print("</select>\n");
		// Minutes
		WiServer.print("<select name=mm>\n");
		for (uint8_t i = 0; i < 60; i++){
			WiServer.print("<option");
			if (i==mm){
				WiServer.print(" selected");
			}
			WiServer.print(">");
			// Calculate year
			sprintf(mm_s,"%.2d",i);
			WiServer.print(mm_s);
			WiServer.print("</option>\n");
		}
		WiServer.print("</select>\n");
		// Seconds
		WiServer.print("<select name=ss>\n");
		for (uint8_t i = 0; i < 60; i++){
			WiServer.print("<option");
			if (i==ss){
				WiServer.print(" selected");
			}
			WiServer.print(">");
			// Calculate year
			sprintf(ss_s,"%.2d",i);
			WiServer.print(ss_s);
			WiServer.print("</option>\n");
		}
		WiServer.print("</select>\n");
/*		// UTC
		WiServer.print(enter_UTC_format);
		WiServer.print("<select name=tz size=1>");
		for (int8_t i = -12; i < 15; i++){
			for (int8_t j = 0; j < 50; (j+15)){
				WiServer.print("<option name=tz value=");
				// Calculate year
				sprintf(ss_s,"%.2d",i);
				WiServer.print(ss_s);
				WiServer.print(":");
				sprintf(ss_s,"%.2d",j);
				WiServer.print(ss_s);
				if (i==ss){
					WiServer.print(" selected");
				}
				WiServer.print("> ");
				WiServer.print(ss_s);
				WiServer.print(" </option>");
			}
		}
		WiServer.print("</select>");*/
		// Summer time
//		_DBG("Summer time");_DBG(" (");_DBG(__FILE__);_DBG(":");_DBD16(__LINE__);_DBG(")\r\n");
		WiServer.print(p_open);
		WiServer.print(enter_DST_format);
		WiServer.print("<select name=st>\n");
		for (int8_t i = (0-60); i < 61; ){
			WiServer.print("<option");
			if (i==dst){
				WiServer.print(" selected");
			}
			WiServer.print(">");
			// Calculate year
			sprintf(dst_s,"%i",i);
			WiServer.print(dst_s);
			WiServer.print("</option>\n");
			i += 15;
		}
		WiServer.print("</select>\n");
		WiServer.print(line_break);
		// Save + Reset
		WiServer.print(submit_reset_buttons);
		WiServer.print(form_close);
//		_DBG("[INFO]-strcmp(URL, /) END");_DBG(" (");_DBG(__FILE__);_DBG(":");_DBD16(__LINE__);_DBG(")\r\n");
		WiServer.print(body_close);
		WiServer.print(html_close);
		return true;
	}

	// URL not found
    _DBG("[INFO]-strcmp(URL, ?) URL not found");_DBG(" (");_DBG(__FILE__);_DBG(":");_DBD16(__LINE__);_DBG(")\r\n");
	return false;
};


