/*
 * webpage.c
 *
 *  Created on: 3 Jul 2011
 *      Author: Gavin
 */

extern "C" {
	#include "libraries/RTClib/rtc.h"
}

#include "Webpage.h"
#include "WiServer.h"


bool send_home_page(char* URL){
	uint8_t i,j;
//	char DOW[DOW_LEN_MAX] = DayOfWeekName[GetDOW()][DOW_LEN_MAX];
	uint8_t DOM = GetDOM();
//	char M[DOW_LEN_MAX] = Month_of_the_year[GetM()][];
	uint16_t Y = GetY();

	uint8_t HH = GetHH();
	uint8_t MM = GetMM();
	uint8_t SS = GetSS();





// Use WiServer's print and println functions to write out the page content
	// Header
	WiServer.print("<html><head><title>--==Equinox Clock==-- by Gavin and Jamie Clarke</title></head>");

	// Title
	WiServer.print("<body bgcolor=\"silver\"><br /><h1 align=\"center\"><font size=\"8\" align=\"center\"color=\"Red\">--==Equinox Clock==--</font></h1><br />");



	// Date drop down selection box
	WiServer.print("<p><font size=\"3\" color=\"Red\" face=\"Comic Sans MS\"> Enter Date: </font></p>");
	WiServer.print("<select name=dow size=1>");
	// Day of the Month
	for (i = 0; i < NUM_DAYS_OF_WEEK; i++){
		WiServer.print("<option name=dom value=");
		WiServer.print(i);
		WiServer.print("> ");
		WiServer.print(i);
		WiServer.print(" </option>");
	}
	WiServer.print("</select>");
	// Month
	for (i = Y; i < Y + 20; i++){
		WiServer.print("<option name=dom value=");
		WiServer.print(i);
		WiServer.print("> ");
		for (j = 0; j < 3; j++){
			WiServer.print(Month_of_the_year[i][j]);
		}
		WiServer.print(" </option>");
	}
	WiServer.print("</select>");
	// Month
	for (i = 0; i < NUM_DAYS_OF_WEEK; i++){
		WiServer.print("<option name=dom value=");
		WiServer.print(i);
		WiServer.print("> ");
		WiServer.print(i);
		WiServer.print(" </option>");
	}
	WiServer.print("</select><br />");
	// Save + Reset
	WiServer.print("<input type=\"submit\" value=\"Send Date\" /><input type=\"reset\" value=\"Reset\" />");
	WiServer.print("</form>");






};


