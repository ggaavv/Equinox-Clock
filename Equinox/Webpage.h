/*
 * webpage.h
 *
 *  Created on: 3 Jul 2011
 *      Author: Gavin
 */

#ifndef WEBPAGE_H_
#define WEBPAGE_H_

#include "lpc_types.h"
#include "rtc.h"


void send_home_page(void){

// Use WiServer's print and println functions to write out the page content
	WiServer.print("HTTP/1.1 200 OK\r\nContent-Type:"
			"text/html\r\n\r\n"
			"<center>"
			"<h1>Hello World!! I am WiShield</h1>"
			"<form method=\"get\" action=\"0\">Toggle LED:<input type=\"submit\" name=\"0\" value=\"LED1\">"
			"</input>"
			"</form>"
			"</center>





			"<html>");
	WiServer.print("Hello World!");
	WiServer.print("</html>");

};

#endif /* WEBPAGE_H_ */

