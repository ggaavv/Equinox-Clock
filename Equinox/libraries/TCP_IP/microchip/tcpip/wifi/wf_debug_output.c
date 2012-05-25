/*******************************************************************************
  MRF24W Driver WiFi Console

  Summary:
    Module for Microchip TCP/IP Stack
    
  Description:
    -Provides access to MRF24W WiFi controller
    -Reference: MRF24W Data sheet, IEEE 802.11 Standard
*******************************************************************************/

/*******************************************************************************
FileName:   wf_debug_output.c
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

//============================================================================
// Includes
//============================================================================
#include "tcpip_private.h"

#if defined(TCPIP_IF_MRF24W)

#include "wf_debug_output.h"
#include "system_profile.h"
#include "wf_config.h"

#if defined(SYS_CONSOLE_ENABLE)
const char *connectionFailureStrings[] = {
                                        "NULL",                                  /* 0 - not used */
                                        "NULL",                                  /* 1 - not used */
                                        "WF_JOIN_FAILURE",                       /* 2            */
                                        "WF_AUTHENTICATION_FAILURE",             /* 3            */
                                        "WF_ASSOCIATION_FAILURE",                /* 4            */
                                        "WF_WEP_HANDSHAKE_FAILURE",              /* 5            */
                                        "WF_PSK_CALCULATION_FAILURE",            /* 6            */
                                        "WF_PSK_HANDSHAKE_FAILURE",              /* 7            */
                                        "WF_ADHOC_JOIN_FAILURE",                 /* 8            */
                                        "WF_SECURITY_MISMATCH_FAILURE",          /* 9            */
                                        "WF_NO_SUITABLE_AP_FOUND_FAILURE",       /* 10           */
                                        "WF_RETRY_FOREVER_NOT_SUPPORTED_FAILURE",/* 11           */
                                        "WF_LINK_LOST",                          /* 12           */
                                        "",                                      /* 13           */
                                        "WF_RSN_MIXED_MODE_NOT_SUPPORTED",       /* 14           */
                                        "WF_RECV_DEAUTH",                        /* 15           */
                                        "WF_RECV_DISASSOC",                      /* 16           */
                                        "WF_WPS_FAILURE",                        /* 17           */
                                        "WF_P2P_FAILURE",                        /* 18           */
                                        "WF_LINK_DOWN"                           /* 19           */
                                       };           
                                       
const char *connectionLostStrings[] = {
                                        "Association Failure",      /* 0 */
                                        "WF_BEACON_TIMEOUT",        /* 1 */
                                        "WF_DEAUTH_RECEIVED",       /* 2 */
                                        "WF_DISASSOCIATE_RECEIVED", /* 3 */
                                        "WF_TKIP_MIC_FAILURE",      /* 4 */                                            
                                        "WF_LINK_DOWN"              /* 5 */
                                    };  
#endif /* SYS_CONSOLE_ENABLE */


//============================================================================
// Local Function Prototypes
//============================================================================
#if defined(SYS_CONSOLE_ENABLE)
static void OutputDeviceInfo(void);
static void OutputDomain(void);
static void OutputMacAddress(void);
static void OutputSsid(void);
static void OutputNetworkType(void);
static void OutputScanType(void);
static void OutputChannelList(void);
static void OutputRetryCount(void);
static void OutputBeaconTimeout(void);
static void OutputSecurityMode(void);
static void OutputTxMode(void);
static void OutputPowerSaveMode(void);
static void OutputIpAddress(void);

static void OutputConnectionTempLostMsg(uint16_t eventInfo);
static void OutputConnectionFailedMsg(uint16_t eventInfo);
static void OutputConnectionPermLostMsg(uint16_t eventInfo);

#endif /* SYS_CONSOLE_ENABLE */

#if defined(SYS_CONSOLE_ENABLE)

/*****************************************************************************
  Function:
    void OutputDemoHeader(void);

  Summary:
	Outputs WiFi demo header to console.

  Description:
	Outputs the name of the demo, and various information items about how the 
	demo is configured.

  Precondition:
	None

  Parameters:
	None

  Returns:
	None

  Remarks:
	None
***************************************************************************/
void OutputDemoHeader(void)
{
    SYS_CONSOLE_MESSAGE("\r\n");
    #if defined(WIFI_TCPIP_CONSOLE_DEMO)
        SYS_CONSOLE_MESSAGE("=========================\r\n");
        SYS_CONSOLE_MESSAGE("*** WiFi Console Demo ***\r\n"); 
        SYS_CONSOLE_MESSAGE("=========================\r\n");        
    #elif defined(WIFI_TCPIP_WEB_SERVER_DEMO)
        SYS_CONSOLE_MESSAGE("===================================\r\n");
        SYS_CONSOLE_MESSAGE("*** WiFi TCP/IP Web Server Demo ***\r\n"); 
        SYS_CONSOLE_MESSAGE("===================================\r\n");        
    #elif defined(WF_EASY_CONFIG_DEMO)
        SYS_CONSOLE_MESSAGE("==========================\r\n");    
        SYS_CONSOLE_MESSAGE("*** WiFi EZConfig Demo ***\r\n"); 
        SYS_CONSOLE_MESSAGE("==========================\r\n");    
    #else
        SYS_CONSOLE_MESSAGE("*** Unknown Demo ***\r\n"); 
    #endif

    OutputDeviceInfo();
    OutputDomain();   
    OutputMacAddress();
    OutputSsid();   
    OutputNetworkType();
    OutputScanType();
    OutputChannelList();
    OutputRetryCount();
    OutputBeaconTimeout();
    OutputSecurityMode();
    OutputTxMode();
    OutputPowerSaveMode();
    OutputIpAddress();
}    


static void OutputDeviceInfo(void)
{
    char buf[20];
    tWFDeviceInfo deviceInfo;
    
    SYS_CONSOLE_MESSAGE("Device:          ");        
    
    WF_GetDeviceInfo(&deviceInfo);
    sprintf(buf, "MRF24WB (0x%02X%02X)\r\n", deviceInfo.romVersion, deviceInfo.patchVersion);
    SYS_CONSOLE_MESSAGE(buf);
}    

/*****************************************************************************
  Function:
    static void OutputDomain(void)

  Summary:
	Outputs WiFi domain string.

  Description:
	Called by OuputDomain()

  Precondition:
	None

  Parameters:
	None

  Returns:
	None

  Remarks:
	None
***************************************************************************/
static void OutputDomain(void)
{
    SYS_CONSOLE_MESSAGE("Domain:          ");
    #if (WF_DEFAULT_DOMAIN == WF_DOMAIN_FCC)
        SYS_CONSOLE_MESSAGE("FCC");
    #elif (WF_DEFAULT_DOMAIN == WF_DOMAIN_JAPAN)
        SYS_CONSOLE_MESSAGE("Japan");
    #elif (WF_DEFAULT_DOMAIN == WF_DOMAIN_ETSI)
        SYS_CONSOLE_MESSAGE("ETSI");
    #elif (WF_DEFAULT_DOMAIN == WF_DOMAIN_OTHER)
        SYS_CONSOLE_MESSAGE("Other");
    #else
        SYS_CONSOLE_MESSAGE("Unknown Regional Domain!!")
    #endif
    
    SYS_CONSOLE_MESSAGE("\r\n");
}    

/*****************************************************************************
  Function:
    static void OutputMacAddress(void)

  Summary:
	Outputs WiFi MAC address

  Description:
	Called by OuputDomain()

  Precondition:
	None

  Parameters:
	None

  Returns:
	None

  Remarks:
	None
***************************************************************************/
static void OutputMacAddress(void)
{
    uint8_t mac[6];
    int i;
    char buf[16];
    
    WF_GetMacAddress(mac);
    SYS_CONSOLE_MESSAGE("MAC:             ");
    for (i = 0; i < 6; ++i)
    {
        sprintf(buf, "%02X ", mac[i]);
        SYS_CONSOLE_MESSAGE(buf);
    }    
    SYS_CONSOLE_MESSAGE("\r\n");
}    

static void OutputSsid(void)
{
    NET_CONFIG* p_config;
    
    p_config = GetNetworkConfig();
    
    SYS_CONSOLE_MESSAGE("SSID:            ");
    if (p_config->MySSID[0] == '\0')
    {
        SYS_CONSOLE_MESSAGE("(none)");
    }    
    else
    {
        SYS_CONSOLE_MESSAGE((char *)p_config->MySSID);
    }    
    SYS_CONSOLE_MESSAGE("\r\n");
}

static void OutputNetworkType(void)
{
    SYS_CONSOLE_MESSAGE("Network Type:    ");
    #if (WF_DEFAULT_NETWORK_TYPE == WF_ADHOC)
        SYS_CONSOLE_MESSAGE("AdHoc");
	#elif (WF_DEFAULT_NETWORK_TYPE == WF_INFRASTRUCTURE)
        SYS_CONSOLE_MESSAGE("Infrastructure");
    #endif
    SYS_CONSOLE_MESSAGE("\r\n");
}    

static void OutputScanType(void)
{
    SYS_CONSOLE_MESSAGE("Scan Type:       ");
    #if (WF_DEFAULT_SCAN_TYPE == WF_PASSIVE_SCAN)
        SYS_CONSOLE_MESSAGE("Passive Scan\r\n");
    #else
        SYS_CONSOLE_MESSAGE("Active Scan\r\n");    
    #endif  
}

static void OutputChannelList(void)
{
    uint8_t channelList[] = WF_DEFAULT_CHANNEL_LIST;
    int i;
    char buf[64];
    
    SYS_CONSOLE_MESSAGE("Channel List:    ");
    for (i = 0; i < sizeof(channelList); ++i)
    {
        sprintf(buf, "%d", channelList[i]); 
        SYS_CONSOLE_MESSAGE(buf);   
        
        if (i != sizeof(channelList) - 1)
        {
            SYS_CONSOLE_MESSAGE(", ");
        }    
        else
        {
            SYS_CONSOLE_MESSAGE("\r\n");
        }    
    } 
}    

static void OutputRetryCount(void)
{
    #if WF_DEFAULT_LIST_RETRY_COUNT != WF_RETRY_FOREVER 
    char buf[16];
    #endif
    
    SYS_CONSOLE_MESSAGE("Retry Count:     ");
    #if (WF_DEFAULT_LIST_RETRY_COUNT == WF_RETRY_FOREVER)
        SYS_CONSOLE_MESSAGE("Retry Forever\r\n");
    #else
        sprintf(buf, "%d\r\n", WF_DEFAULT_LIST_RETRY_COUNT);
        SYS_CONSOLE_MESSAGE(buf);  
    #endif
}

static void OutputBeaconTimeout(void)
{
    char buf[16];
    
    SYS_CONSOLE_MESSAGE("Beacon Timeout:  ");
    #if (WF_DEFAULT_BEACON_TIMEOUT == 0)
        SYS_CONSOLE_MESSAGE("Ignored\r\n");
    #else
        sprintf(buf, "%d\r\n", WF_DEFAULT_BEACON_TIMEOUT);
        SYS_CONSOLE_MESSAGE(buf);
    #endif
}

static void OutputSecurityMode(void)
{
    SYS_CONSOLE_MESSAGE("Security:        ");
    /* Set Security */
    #if (WF_DEFAULT_WIFI_SECURITY_MODE == WF_SECURITY_OPEN)
        SYS_CONSOLE_MESSAGE("Open\r\n");
    #elif (WF_DEFAULT_WIFI_SECURITY_MODE == WF_SECURITY_WPA_WITH_KEY)
        SYS_CONSOLE_MESSAGE("WPA with key");
    #elif (WF_DEFAULT_WIFI_SECURITY_MODE == WF_SECURITY_WPA2_WITH_KEY)
        SYS_CONSOLE_MESSAGE("WPA2 with key");
    #elif (WF_DEFAULT_WIFI_SECURITY_MODE == WF_SECURITY_WPA_WITH_PASS_PHRASE)
        SYS_CONSOLE_MESSAGE("WPA with pass phrase");
    #elif (WF_DEFAULT_WIFI_SECURITY_MODE == WF_SECURITY_WPA2_WITH_PASS_PHRASE)
        SYS_CONSOLE_MESSAGE("WPA2 with pass phrase");    
    #elif (WF_DEFAULT_WIFI_SECURITY_MODE == WF_SECURITY_WPA_AUTO_WITH_KEY)
        SYS_CONSOLE_MESSAGE("WPA with key, auto-select");
    #elif (WF_DEFAULT_WIFI_SECURITY_MODE == WF_SECURITY_WPA_AUTO_WITH_PASS_PHRASE)
        SYS_CONSOLE_MESSAGE("WPA with pass phrase, auto-select");
	#elif (WF_DEFAULT_WIFI_SECURITY_MODE == WF_SECURITY_WPS_PUSH_BUTTON)
        SYS_CONSOLE_MESSAGE("WPS push button method");
	#elif (WF_DEFAULT_WIFI_SECURITY_MODE == WF_SECURITY_WPS_PIN)
        SYS_CONSOLE_MESSAGE("WPS PIN method");
	#elif (WF_DEFAULT_WIFI_SECURITY_MODE == WF_SECURITY_EAP)
        SYS_CONSOLE_MESSAGE("WPA Enterprise");
    #else
        SYS_CONSOLE_MESSAGE("Unknown security type");
    #endif 
    
    #if ((WF_DEFAULT_WIFI_SECURITY_MODE == WF_SECURITY_WEP_40)  ||   \
         (WF_DEFAULT_WIFI_SECURITY_MODE == WF_SECURITY_WEP_104))    
        SYS_CONSOLE_MESSAGE("\r\n");
        #if (WF_DEFAULT_WIFI_SECURITY_MODE == WF_SECURITY_WEP_40)
            SYS_CONSOLE_MESSAGE("WEP40, Open Key\r\n");
        #elif (WF_DEFAULT_WIFI_SECURITY_MODE == WF_SECURITY_WEP_104)
            SYS_CONSOLE_MESSAGE("WEP104, Open Key\r\n");
        #endif
        {
            char buf[4];
            NET_CONFIG* p_config;
            p_config = GetNetworkConfig();

            SYS_CONSOLE_MESSAGE("WEP Key Index:   ");
            sprintf(buf, "%d\r\n", p_config->wepKeyIndex);
            SYS_CONSOLE_MESSAGE(buf);
        }    
    #endif
}

static void OutputTxMode(void)
{    
    SYS_CONSOLE_MESSAGE("Tx Mode:         ");
    SYS_CONSOLE_MESSAGE("1 and 2 Mbps\r\n");
}

static void OutputPowerSaveMode(void)
{
   SYS_CONSOLE_MESSAGE("Power Save:      ");
    #if (WF_DEFAULT_PS_POLL == WF_ENABLED)
        SYS_CONSOLE_MESSAGE("Enabled\r\n");
    #else
        SYS_CONSOLE_MESSAGE("Disabled\r\n");    
    #endif    
}

static void OutputIpAddress(void)
{
    NET_CONFIG* p_config;
    #if defined(ENABLE_STATIC_IP)
    char buf[64];
    #endif
    
    p_config = GetNetworkConfig();

    SYS_CONSOLE_MESSAGE("IP Address:      ");
    #if defined(ENABLE_STATIC_IP)
        for (i = 0; i < 4; ++i)
        {
            sprintf(buf, "%d", (uint8_t)(p_config.MyIPAddr.Val >> (8 * i)));
            SYS_CONSOLE_MESSAGE(buf);
            if (i != 3)
            {
                SYS_CONSOLE_MESSAGE(".");
            }    
        }    
        SYS_CONSOLE_MESSAGE(" (static)\r\n");
    #else
        SYS_CONSOLE_MESSAGE("via DHCP\r\n");
    #endif
}





void WF_OutputConnectionDebugMsg(uint8_t event, uint16_t eventInfo)
{
    if (event == WF_EVENT_CONNECTION_TEMPORARILY_LOST)
    {
        OutputConnectionTempLostMsg(eventInfo);
    }
    else if (event == WF_EVENT_CONNECTION_FAILED)
    {
        OutputConnectionFailedMsg(eventInfo);
    }  
    else if (event == WF_EVENT_CONNECTION_PERMANENTLY_LOST)
    {
        OutputConnectionPermLostMsg(eventInfo);        
    }           
}

static void OutputConnectionTempLostMsg(uint16_t eventInfo)
{
    char buf[8];
    
    
    SYS_CONSOLE_MESSAGE("Event: Connection Temporarily Lost -- eventInfo = ");
    sprintf(buf, "%d, ", eventInfo);
    SYS_CONSOLE_MESSAGE(buf);
    SYS_CONSOLE_MESSAGE(connectionLostStrings[eventInfo]);

    SYS_CONSOLE_MESSAGE("\r\n");
}

static void OutputConnectionFailedMsg(uint16_t eventInfo)
{   

    char buf[8];
    
    SYS_CONSOLE_MESSAGE("Event: Connection Failed  -- eventInfo = ");
    sprintf(buf, "%d, ", eventInfo);
    SYS_CONSOLE_MESSAGE(buf);
    SYS_CONSOLE_MESSAGE(connectionFailureStrings[eventInfo]);
    SYS_CONSOLE_MESSAGE("\r\n");
}    
 
static void OutputConnectionPermLostMsg(uint16_t eventInfo)
{
    char buf[8];
    
    SYS_CONSOLE_MESSAGE("Event: Connection Permanently Lost -- eventInfo = ");
    sprintf(buf, "%d, ", eventInfo);
    SYS_CONSOLE_MESSAGE(buf);
    SYS_CONSOLE_MESSAGE(connectionLostStrings[eventInfo]);
    SYS_CONSOLE_MESSAGE("\r\n");
}    






#endif /* SYS_CONSOLE_ENABLE) */

void ValidateConfig(void)
{
}      
   

#endif /* #if defined(TCPIP_IF_MRF24W) */

