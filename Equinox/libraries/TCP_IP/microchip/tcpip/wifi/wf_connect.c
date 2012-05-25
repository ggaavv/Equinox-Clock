/*******************************************************************************
  MRF24W Connection Support

  Summary:
    
  Description:
*******************************************************************************/

/*******************************************************************************
FileName:  wf_connect.c 
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

/*
*********************************************************************************************************
*                                           INCLUDES                               
*********************************************************************************************************
*/

#include "tcpip_private.h"

// Compile only for PIC32MX MRF24W MAC interface
#if defined(TCPIP_IF_MRF24W)


#include "tcpip/wf_api.h"
#include "tcpip/dhcp.h"
#include "wf_debug_output.h"

extern void DhcpEventHandler(TCPIP_NET_HANDLE hNet, DHCP_EVENT_TYPE evType, const void* param);

/*
 * interface functions
 *
*/
/*****************************************************************************
 * FUNCTION: WF_Connect
 *
 * RETURNS:  None
 *
 * PARAMS:   None
 *
 *  NOTES:   Connects to an 802.11 network.
 *****************************************************************************/
void WF_Connect(TCPIP_MAC_HANDLE hMac)
{
    uint8_t ConnectionProfileID;
    uint8_t channelList[] = WF_DEFAULT_CHANNEL_LIST;
    NET_CONFIG* pConfig;

    pConfig = _TCPIPStackMacToNet(hMac);

    /* create a Connection Profile */
    WF_CPCreate(&ConnectionProfileID);
    
    WF_SetRegionalDomain(WF_DEFAULT_DOMAIN);  
    WF_CPSetSsid(ConnectionProfileID, 
                 pConfig->MySSID, 
                 pConfig->SsidLength);
    WF_CPSetNetworkType(ConnectionProfileID, WF_DEFAULT_NETWORK_TYPE);
    WF_CASetScanType(WF_DEFAULT_SCAN_TYPE);
    WF_CASetChannelList(channelList, sizeof(channelList));
    
    // The Retry Count parameter tells the WiFi Connection manager how many attempts to make when trying
    // to connect to an existing network.  In the Infrastructure case, the default is to retry forever so that
    // if the AP is turned off or out of range, the radio will continue to attempt a connection until the
    // AP is eventually back on or in range.  In the Adhoc case, the default is to retry 3 times since the 
    // purpose of attempting to establish a network in the Adhoc case is only to verify that one does not
    // initially exist.  If the retry count was set to WF_RETRY_FOREVER in the AdHoc mode, an AdHoc network
    // would never be established. 
    WF_CASetListRetryCount(WF_DEFAULT_LIST_RETRY_COUNT);
    WF_CASetEventNotificationAction(WF_DEFAULT_EVENT_NOTIFICATION_LIST);
    WF_CASetBeaconTimeout(40);

    if (isWiFiVer1209OrLater())
    {
        // If WEP security is used, set WEP Key Type.  The default WEP Key Type is Shared Key.
        if (pConfig->SecurityMode == WF_SECURITY_WEP_40 || pConfig->SecurityMode == WF_SECURITY_WEP_104)
        {
        	WF_CPSetWepKeyType(ConnectionProfileID, WF_DEFAULT_WIFI_SECURITY_WEP_KEYTYPE);
        }
    }    
     
    /* Set Security */
    WF_CPSetSecurity(ConnectionProfileID,
                     pConfig->SecurityMode,
                     pConfig->WepKeyIndex,   /* only used if WEP enabled */
                     pConfig->SecurityKey,
                     pConfig->SecurityKeyLength);

    #if WF_DEFAULT_PS_POLL == WF_ENABLED
        WF_PsPollEnable(true, AGGRESSIVE_PS);
        if (isWiFiVer1209OrLater())
        {
            WFEnableDeferredPowerSave();
        }    
    #else
        WF_PsPollDisable();
    #endif
    
    #if defined(WF_AGGRESSIVE_PS)
        if (isWiFiVer1209OrLater())
        {
            WFEnableAggressivePowerSave();
	    }
    #endif
	
	#if defined(SYS_CONSOLE_ENABLE)
 	OutputDemoHeader();
 	#endif
    
    SYS_CONSOLE_MESSAGE("\r\nStart WiFi Connect . . .\r\n");        
    
    DHCPRegisterHandler(TCPIP_STACK_NetHandle("MRF24W"), DhcpEventHandler, NULL);

    // start the WiFi connection process
    WF_CMConnect(ConnectionProfileID);
}   


#endif  // defined(TCPIP_IF_MRF24W)


