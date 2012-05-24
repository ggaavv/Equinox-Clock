/*******************************************************************************
  MRF24W Driver Customization

  Summary:
    Module for Microchip TCP/IP Stack
    
  Description:
    -Provides access to MRF24W WiFi controller
    -Reference: MRF24W Data sheet, IEEE 802.11 Standard
*******************************************************************************/

/*******************************************************************************
FileName:  wf_Config_mk.h 
Copyright © 2012 released Microchip Technology Inc.  All rights
reserved.

Microchip licenses to you the right to use, modify, copy and distribute
Software only when embedded on a Microchip microcontroller or digital signal
controller that is integrated into your product or third party product
(pursuant to the sublicense terms in the accompanying license agreement).

You should refer to the license agreement accompanying this Software for
additional information regarding your rights and obligations.

SOFTWARE AND DOCUMENTATION ARE PROVIDED “AS IS” WITHOUT WARRANTY OF ANY KIND,
EITHER EXPRESS OR IMPLIED, INCLUDING WITHOUT LIMITATION, ANY WARRANTY OF
MERCHANTABILITY, TITLE, NON-INFRINGEMENT AND FITNESS FOR A PARTICULAR PURPOSE.
IN NO EVENT SHALL MICROCHIP OR ITS LICENSORS BE LIABLE OR OBLIGATED UNDER
CONTRACT, NEGLIGENCE, STRICT LIABILITY, CONTRIBUTION, BREACH OF WARRANTY, OR
OTHER LEGAL EQUITABLE THEORY ANY DIRECT OR INDIRECT DAMAGES OR EXPENSES
INCLUDING BUT NOT LIMITED TO ANY INCIDENTAL, SPECIAL, INDIRECT, PUNITIVE OR
CONSEQUENTIAL DAMAGES, LOST PROFITS OR LOST DATA, COST OF PROCUREMENT OF
SUBSTITUTE GOODS, TECHNOLOGY, SERVICES, OR ANY CLAIMS BY THIRD PARTIES
(INCLUDING BUT NOT LIMITED TO ANY DEFENSE THEREOF), OR OTHER SIMILAR COSTS.
*******************************************************************************/

#ifndef __WF_CONFIG_H_
#define __WF_CONFIG_H_

#define WIFI_TCPIP_WEB_SERVER_DEMO

/*
*********************************************************************************************************
*                                           DEFINES                               
*********************************************************************************************************
*/



/*****************************************************************************/
/*****************************************************************************/
/*                             WIFI SECURITY COMPILE-TIME DEFAULTS           */
/*****************************************************************************/
/*****************************************************************************/
// Security modes available on WiFi network:
//   WF_SECURITY_OPEN                      : No security
//   WF_SECURITY_WEP_40                    : WEP Encryption using 40 bit keys
//   WF_SECURITY_WEP_104                   : WEP Encryption using 104 bit keys
//   WF_SECURITY_WPA_WITH_KEY              : WPA-PSK Personal where binary key is given to MRF24WB0M 
//   WF_SECURITY_WPA_WITH_PASS_PHRASE      : WPA-PSK Personal where passphrase is given to MRF24WB0M and it calculates the binary key
//   WF_SECURITY_WPA2_WITH_KEY             : WPA2-PSK Personal where binary key is given to MRF24WB0M 
//   WF_SECURITY_WPA2_WITH_PASS_PHRASE     : WPA2-PSK Personal where passphrase is given to MRF24WB0M and it calculates the binary key
//   WF_SECURITY_WPA_AUTO_WITH_KEY         : WPA-PSK Personal or WPA2-PSK Personal where binary key is given and MRF24WB0M will 
//                                             connect at highest level AP supports (WPA or WPA2)                                                
//   WF_SECURITY_WPA_AUTO_WITH_PASS_PHRASE : WPA-PSK Personal or WPA2-PSK Personal where passphrase is given to MRF24WB0M and it 
//                                             calculates the binary key and connects at highest level AP supports (WPA or WPA2)
//   WF_SECURITY_WPS_PUSH_BUTTON		   : WPS push button method
//   WF_SECURITY_WPS_PIN		   		   : WPS PIN method

/*--------------------------------------------*/
/* Default settings for Connection Management */
/*--------------------------------------------*/
#define WF_DEFAULT_DOMAIN                   WF_DOMAIN_FCC
#define WF_INFRASTRUCTURE_SSID              "MicrochipDemoAP"
#define WF_DEFAULT_WIFI_SECURITY_MODE        WF_SECURITY_OPEN

#define WF_DEFAULT_SCAN_TYPE                WF_ACTIVE_SCAN          /* WF_ACTIVE_SCAN or WF_PASSIVE_SCAN */
#define WF_DEFAULT_BEACON_TIMEOUT           (40)                    /* Number of missed beacon periods before losing connection */
#define WF_DEFAULT_SSID_NAME                WF_INFRASTRUCTURE_SSID
#define WF_DEFAULT_NETWORK_TYPE             WF_INFRASTRUCTURE       /* WF_INFRASTRUCTURE or WF_ADHOC     */
#define WF_DEFAULT_LIST_RETRY_COUNT         (WF_RETRY_FOREVER)
#define WF_DEFAULT_CHANNEL_LIST             {1,6,11}                /* Default channel list for FCC */


/* Select Infrastructure Power Save Mode */
#define WF_DEFAULT_PS_POLL                   WF_DISABLED         /* WF_DISABLED or WF_ENABLED */
#define AGGRESSIVE_PS                        WF_DISABLED



/* #define WF_AGGRESSIVE_PS */	/* WARNING !!! : This only can work with 1209 module FW version or later.
						* If you use the earlier version such as 1207 or 1205, then you should not define this.
						* Defining this will lead ASSERT problem with old module FW.
						*/



#define WF_DEFAULT_WIFI_SECURITY_WEP_KEYTYPE  WF_SECURITY_WEP_OPENKEY /* WF_SECURITY_WEP_OPENKEY (default) or	  */
																		 /*  WF_SECURITY_WEP_SHAREDKEY. 			 */ 

//-----------------------------------------------------------------------------------
// Default WEP keys used in WF_SECURITY_WEP_40  and WF_SECURITY_WEP_104 security mode
//-----------------------------------------------------------------------------------
#define WF_DEFAULT_WEP_PHRASE           "WEP Phrase"

// string 4 40-bit WEP keys -- corresponding to passphraseof "WEP Phrase"
#define WF_DEFAULT_WEP_KEYS_40 "\
\x5a\xfb\x6c\x8e\x77\
\xc1\x04\x49\xfd\x4e\
\x43\x18\x2b\x33\x88\
\xb0\x73\x69\xf4\x78"
// Do not indent above string as it will inject spaces

// string containing 4 104-bit WEP keys -- corresponding to passphraseof "WEP Phrase"
#define WF_DEFAULT_WEP_KEYS_104 "\
\x90\xe9\x67\x80\xc7\x39\x40\x9d\xa5\x00\x34\xfc\xaa\
\x77\x4a\x69\x45\xa4\x3d\x66\x63\xfe\x5b\x1d\xb9\xfd\
\x82\x29\x87\x4c\x9b\xdc\x6d\xdf\x87\xd1\xcf\x17\x41\
\xcc\xd7\x62\xde\x92\xad\xba\x3b\x62\x2f\x7f\xbe\xfb"
// Do not indent above string as it will inject spaces

/* Valid Key Index: 0, 1, 2, 3  */
#define WF_DEFAULT_WEP_KEY_INDEX        (0)

// Default pass phrase used for WF_SECURITY_WPA_WITH_PASS_PHRASE and 
// WF_SECURITY_WPA2_WITH_PASS_PHRASE security modes
#define WF_DEFAULT_PSK_PHRASE               "Microchip 802.11 Secret PSK Password"


// If using security mode of WF_SECURITY_WPA_WITH_KEY or WF_SECURITY_WPA2_WITH_KEY, then this section 
// must be set to  match the key for WF_DEFAULT_SSID_NAME and WF_DEFAULT_PSK_PHRASE
// combination.  The values below are derived from the SSID "MicrochipDemoAP" and the pass phrase
// "Microchip 802.11 Secret PSK Password".
// The tool at http://www.wireshark.org/tools/wpa-psk.html can be used to generate this field. 
#define WF_DEFAULT_PSK "\
\x86\xC5\x1D\x71\xD9\x1A\xAA\x49\
\x40\xC8\x88\xC6\xE9\x7A\x4A\xD5\
\xE5\x6D\xDA\x44\x8E\xFB\x9C\x0A\
\xE1\x47\x81\x52\x31\x1C\x13\x7C"
// Do not indent above string as it will inject spaces 

// The default interrupt priority to use for the TCPIP interrupts
#define	WIFI_EVENT_IPL	    	5
#define	WIFI_EVENT_SIPL 		1


/*** Selecting Event Notification Type ***/
#define WF_DEFAULT_EVENT_NOTIFICATION_LIST  (WF_NOTIFY_CONNECTION_ATTEMPT_SUCCESSFUL  |         \
                                             WF_NOTIFY_CONNECTION_ATTEMPT_FAILED      |         \
                                             WF_NOTIFY_CONNECTION_TEMPORARILY_LOST    |         \
                                             WF_NOTIFY_CONNECTION_PERMANENTLY_LOST    |         \
                                             WF_NOTIFY_CONNECTION_REESTABLISHED)


#endif /* __WF_CONFIG_H_ */


