
#include "Wifi.h"
#include "Webpage.h"
#include "WiServer.h"
#include <stdlib.h>
#include <string.h>

extern "C" {
	#include "debug_frmwrk.h"
	#include "pinout.h"
	#include "lpc17xx_pinsel.h"
	#include "lpc17xx_gpio.h"
	#include "lpc17xx_ssp.h"
	#include "lpc17xx_exti.h"
	#include "g2100.h"
	#include "comm.h"
	#include "sys_timer.h"
	void stack_init(void);
	void stack_process(void);
}


void WiFi_init(){

	// Configuring GPIO
	GPIO_SetDir(WF_CS_PORT, WF_CS_BIT, 1);
	GPIO_SetDir(WF_RESET_PORT, WF_RESET_BIT, 1);
	GPIO_SetDir(WF_HIBERNATE_PORT, WF_HIBERNATE_BIT, 1);

	GPIO_SetValue(WF_CS_PORT, WF_CS_BIT);//Active low
	GPIO_ClearValue(WF_HIBERNATE_PORT, WF_HIBERNATE_BIT);//Active high

	//reset wifi
	GPIO_ClearValue(WF_RESET_PORT, WF_RESET_BIT);
	delay_ms(100);
	GPIO_SetValue(WF_RESET_PORT, WF_RESET_BIT);//Active low
	delay_ms(500);

	// Initialize SPI pin connect
	PINSEL_CFG_Type PinCfg;
	/* SSEL1 */
	PinCfg.Funcnum   = PINSEL_FUNC_0;
	PinCfg.OpenDrain = PINSEL_PINMODE_NORMAL;
	PinCfg.Pinmode   = PINSEL_PINMODE_PULLUP;
	PinCfg.Pinnum    = WF_CS_PIN;
	PinCfg.Portnum   = WF_CS_PORT;
	PINSEL_ConfigPin(&PinCfg);
	/* SCK1 */
	PinCfg.Funcnum   = PINSEL_FUNC_2;
	PinCfg.OpenDrain = PINSEL_PINMODE_NORMAL;
	PinCfg.Pinmode   = PINSEL_PINMODE_PULLUP;
	PinCfg.Pinnum    = WF_SCK_PIN;
	PinCfg.Portnum   = WF_SCK_PORT;
	PINSEL_ConfigPin(&PinCfg);
	/* MISO1 */
	PinCfg.Funcnum   = PINSEL_FUNC_2;
	PinCfg.OpenDrain = PINSEL_PINMODE_NORMAL;
	PinCfg.Pinmode   = PINSEL_PINMODE_PULLUP;
	PinCfg.Pinnum    = WF_MISO_PIN;
	PinCfg.Portnum   = WF_MISO_PORT;
	PINSEL_ConfigPin(&PinCfg);
	/* MOSI1 */
	PinCfg.Funcnum   = PINSEL_FUNC_2;
	PinCfg.OpenDrain = PINSEL_PINMODE_NORMAL;
	PinCfg.Pinmode   = PINSEL_PINMODE_PULLUP;
	PinCfg.Pinnum    = WF_MOSI_PIN;
	PinCfg.Portnum   = WF_MOSI_PORT;
	PINSEL_ConfigPin(&PinCfg);
	/* EINT */
	PinCfg.Funcnum   = PINSEL_FUNC_1;
	PinCfg.OpenDrain = PINSEL_PINMODE_NORMAL;
	PinCfg.Pinmode   = PINSEL_PINMODE_PULLUP;
	PinCfg.Pinnum    = WF_EINT_PIN;
	PinCfg.Portnum   = WF_EINT_PORT;
	PINSEL_ConfigPin(&PinCfg);

	// Configuring Ext Int
	EXTI_InitTypeDef EXTICfg;
	EXTICfg.EXTI_Line 		= WF_EXTI_EINT;
	EXTICfg.EXTI_Mode 		= EXTI_MODE_EDGE_SENSITIVE;
	EXTICfg.EXTI_polarity 	= EXTI_POLARITY_LOW_ACTIVE_OR_FALLING_EDGE;
	EXTI_Config(&EXTICfg);

    NVIC_SetPriority(EINT_IRQn, 9); // set according to main.c
	NVIC_EnableIRQ(EINT_IRQn);

	/* initialize SSP configuration structure */
	SSP_CFG_Type SSP_ConfigStruct;
	SSP_ConfigStruct.CPHA = SSP_CPHA_SECOND;
	SSP_ConfigStruct.CPOL = SSP_CPOL_LO;
	SSP_ConfigStruct.ClockRate = 25000000; /* WF max frequency = 25mhz*/
	SSP_ConfigStruct.Databit = SSP_DATABIT_8;
	SSP_ConfigStruct.Mode = SSP_MASTER_MODE;
	SSP_ConfigStruct.FrameFormat = SSP_FRAME_SPI;
	SSP_Init(WF_SPI_CHN, &SSP_ConfigStruct);

	/* Enable SSP peripheral */
	SSP_Cmd(WF_SPI_CHN, ENABLE);

//	while(zg_get_conn_state() != 1) {
//		_DBG("BEFORE\n while(zg_get_conn_state() != 1) {");_DBG(" (");_DBG(__FILE__);_DBG(":");_DBD16(__LINE__);_DBG(")\r\n");
//		zg_drv_process();
//		_DBG("AFTER\nwhile(zg_get_conn_state() != 1) {");_DBG(" (");_DBG(__FILE__);_DBG(":");_DBD16(__LINE__);_DBG(")\r\n");
//	}
	//_DBG("[OK]-Wifi Connected :)");_DBG(" (");_DBG(__FILE__);_DBG(":");_DBD16(__LINE__);_DBG(")\r\n");
//	stack_init();
	WiServer.init(home_page);
}

/*bool sendMyPage(char* URL) {

    // Check if the requested URL matches "/"
    if (strcmp(URL, "/") == 0) {
        // Use WiServer's print and println functions to write out the page content
        WiServer.print("<html>");
        WiServer.print("Hello World!");
        WiServer.print("</html>");

        // URL was recognized
        return true;
    }
    // URL not found
    return false;
}*/

void WiFi_loop(){
//	_DBG("WiFi_loop()");_DBG(" (");_DBG(__FILE__);_DBG(":");_DBD16(__LINE__);_DBG(")\r\n");
//	stack_process();
//	zg_drv_process();
	WiServer.server_task();
}

// This is the webpage that is served up by the webserver
//const char webpage[] = {"HTTP/1.1 200 OK\r\nContent-Type: text/html\r\n\r\n<center><h1>Hello World!! I am WiShield</h1><form method=\"get\" action=\"0\">Toggle LED:<input type=\"submit\" name=\"0\" value=\"LED1\"></input></form></center>"};

///*
extern "C" {

   boolean connectAndSendTCP = false;

   // Process UDP UIP_APPCALL events
   void udpapp_appcall(void)
   {
      uip_dhcp_run();
   }

   // DHCP query complete callback
   void uip_dhcp_callback(const struct dhcp_state *s)
   {
      if(NULL != s) {
         // Set the received IP addr data into the uIP stack
         uip_sethostaddr(s->ipaddr);
         uip_setdraddr(s->default_router);
         uip_setnetmask(s->netmask);

         // Print the received data - its quick and dirty but informative
         xprintf(INFO "DHCP IP     : ");FFL_();
         xprintf(INFO "%d.%d.%d.%d",uip_ipaddr1(s->ipaddr),uip_ipaddr2(s->ipaddr),uip_ipaddr3(s->ipaddr),uip_ipaddr4(s->ipaddr));FFL_();

         xprintf(INFO "DHCP GATEWAY: ");FFL_();
         xprintf(INFO "%d.%d.%d.%d",uip_ipaddr1(s->default_router),uip_ipaddr2(s->default_router),uip_ipaddr3(s->default_router),uip_ipaddr4(s->default_router));FFL_();

         xprintf(INFO "DHCP NETMASK: ");FFL_();
         xprintf(INFO "%d.%d.%d.%d",uip_ipaddr1(s->netmask),uip_ipaddr2(s->netmask),uip_ipaddr3(s->netmask),uip_ipaddr4(s->netmask));FFL_();

         xprintf(INFO "DHCP DNS    : ");FFL_();
         xprintf(INFO "%d.%d.%d.%d",uip_ipaddr1(s->dnsaddr),uip_ipaddr2(s->dnsaddr),uip_ipaddr3(s->dnsaddr),uip_ipaddr4(s->dnsaddr));FFL_();

      }
      else {
    	  xprintf(INFO "DHCP NULL FALLBACK");FFL_();
      }

      // Shut down DHCP
      uip_dhcp_shutdown();

      connectAndSendTCP = true;
   }

/*
   char packet[] = "SocketAppDHCP";

   void socket_app_appcall(void)
   {
      if(uip_closed() || uip_timedout()) {
         Serial.println("SA: closed / timedout");
         uip_close();
         return;
      }
      if(uip_poll()) {
         Serial.println("SA: poll");
      }
      if(uip_aborted()) {
         Serial.println("SA: aborted");
      }
      if(uip_connected()) {
         Serial.println("SA: connected / send");
         uip_send(packet, strlen(packet));
      }
      if(uip_acked()) {
         Serial.println("SA: acked");
         uip_close();
      }
      if(uip_newdata()) {
         Serial.println("SA: newdata");
      }
      if(uip_rexmit()) {
         Serial.println("SA: rexmit");
         uip_send(packet, strlen(packet));
      }
   }
*/
   // These uIP callbacks are unused for the purposes of this simple DHCP example
   // but they must exist.
   void socket_app_init(void)
   {
   }

   void udpapp_init(void)
   {
   }

   void dummy_app_appcall(void)
   {
   }
}

//*/

