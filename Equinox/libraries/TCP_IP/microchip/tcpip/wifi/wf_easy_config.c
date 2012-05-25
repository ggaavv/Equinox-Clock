/*******************************************************************************
  MRF24W Driver

  Summary:
    Module for Microchip TCP/IP Stack
    
  Description:
    -Provides access to MRF24W WiFi controller
    -Reference: MRF24W Data sheet, IEEE 802.11 Standard
*******************************************************************************/

/*******************************************************************************
FileName:   wf_easy_config.c
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

#include "tcpip_private.h"

#if defined(TCPIP_IF_MRF24W)


#include "wf_easy_config.h"
#include "tcpip/wf_api.h"
#include "wf_console.h"

#if defined ( EZ_CONFIG_SCAN )
tWFScanCtx  g_ScanCtx;
#endif /* EZ_CONFIG_SCAN */

#if defined(TCPIP_STACK_USE_EZ_CONFIG)
/* Easy Config Globals */
extern uint8_t ConnectionProfileID;   //SCC2 ??? what to do with this...

tWFEasyConfigCtx g_easyConfigCtx;

/* Easy Config Private Functions */
static int WFEasyConfigProcess(void);

void WFEasyConfigInit(NET_CONFIG* pConfig)
{
    CFGCXT.ssid[0] = 0;
    CFGCXT.security = WF_SECURITY_OPEN;
    CFGCXT.key[0] = 0;
    CFGCXT.defaultWepKey = WF_WEP_KEY_INVALID;
    CFGCXT.type = WF_INFRASTRUCTURE;
    CFGCXT.cfg_state = cfg_stopped;
    CFGCXT.isWifiNeedToConfigure = 0;

    #if defined (EZ_CONFIG_STORE)
    CFGCXT.isWifiDoneConfigure = pConfig->dataValid;
    #endif
    return;
}

void WFEasyConfigMgr()
{
    if (CFGCXT.isWifiNeedToConfigure) {
        if (WFEasyConfigProcess()) {
            //Has been configured, clear flag
            CFGCXT.isWifiNeedToConfigure = 0;
            CFGCXT.isWifiDoneConfigure = 1;
        }
    }
    return;
}

static int WFEasyConfigProcess(void)
{
    uint8_t ConnectionProfileID;
    uint8_t ConnectionState;
    
    #if defined (EZ_CONFIG_STALL)
    if (CFGCXT.cfg_state == cfg_stopped)
	{
        /* State machine just started get current time stamp */
        CFGCXT.cfg_state = cfg_stalled;
        CFGCXT.timeStart = SYS_TICK_Get();
        return 0;
    }
    
    /* Wait for stall time to expire */
    if (CFGCXT.cfg_state == cfg_stalled)
	{
        SYS_TICK time = SYS_TICK_Get();
        if ((time - CFGCXT.timeStart) < (WF_STALL_TIME * SYS_TICK_TicksPerSecondGet()))
            return 0;
    }
    
    #endif //EZ_CONFIG_STALL
  
    /* We will re-use the current profile */
    WF_CMGetConnectionState(&ConnectionState, &ConnectionProfileID);

    /* Need to disconnect */
    WF_CMDisconnect();

    /* Delete profile */
    WF_CPDelete(ConnectionProfileID);

    /* Create and prepare new profile */
    WF_CPCreate(&ConnectionProfileID);

    /* Now set the various connection profile parameters */

    /* Set SSID... */
    if (CFGCXT.ssid)
        WF_CPSetSsid(ConnectionProfileID, 
            CFGCXT.ssid, 
            strlen((char*)CFGCXT.ssid));  

    /* Now deal with security... */
    switch ((uint8_t)CFGCXT.security) {
        case WF_SECURITY_OPEN: /* No security */
            WF_CPSetSecurity(ConnectionProfileID, WF_SECURITY_OPEN, 0, 0, 0);
            break; 

        case WF_SECURITY_WPA_AUTO_WITH_PASS_PHRASE:
            if (CFGCXT.key) {
                WF_CPSetSecurity(ConnectionProfileID, WF_SECURITY_WPA_AUTO_WITH_PASS_PHRASE, 0, CFGCXT.key, strlen((char *)CFGCXT.key));
            }
            break;

        case WF_SECURITY_WPA_AUTO_WITH_KEY:
            if (CFGCXT.key) {
                WF_CPSetSecurity(ConnectionProfileID, WF_SECURITY_WPA_AUTO_WITH_KEY, 0, CFGCXT.key, 32);
            }
            break;

        case WF_SECURITY_WEP_40:
            {
                uint8_t  keys[20];
                int   i;

                if (CFGCXT.key) {
                    /* Clear key */
                    for (i = 0; i < 20; i++)
                        keys[i] = 0;
                    memcpy(keys, CFGCXT.key, 20);
                    WF_CPSetSecurity(ConnectionProfileID, WF_SECURITY_WEP_40, CFGCXT.defaultWepKey, keys, 20);
                }
            }
            break;

        case WF_SECURITY_WEP_104:
            {
                uint8_t  keys[52];
                int   i;

                if (CFGCXT.key) {
                    /* Clear key */
                    for (i = 0; i < 52; i++)
                        keys[i] = 0;
                    memcpy(keys, CFGCXT.key, 52);
                    WF_CPSetSecurity(ConnectionProfileID, WF_SECURITY_WEP_104, CFGCXT.defaultWepKey, keys, 52);
                }
            }
            break;
    }
 
    #if defined (EZ_CONFIG_STORE)
    #if defined(EEPROM_CS_TRIS) || defined(SPIFLASH_CS_TRIS)
    TCPIP_STORAGE_HANDLE hS;
    hS = TCPIP_STORAGE_Open(0, 1);
    TCPIP_STORAGE_SaveIfConfig(hS, "MRF24W", true);
    TCPIP_STORAGE_Close(hS);
    #endif // defined(EEPROM_CS_TRIS) || defined(SPIFLASH_CS_TRIS)
    #endif // defined (EZ_CONFIG_STORE)

    /* Set wlan mode */
    WF_CPSetNetworkType(ConnectionProfileID, CFGCXT.type);

    /* Kick off connection now... */
    WF_CMConnect(ConnectionProfileID);
    /* Change state and return true to show we are done! */
    CFGCXT.cfg_state = cfg_stopped;

    return 1;
}
#endif /* TCPIP_STACK_USE_EZ_CONFIG */

#if defined ( EZ_CONFIG_SCAN )
void WFInitScan(void)
{
    SCANCXT.scanState = 0;
    SCANCXT.numScanResults = 0;
    SCANCXT.displayIdx = 0;

    return;
}

uint16_t WFStartScan(void)
{
   /* If scan already in progress bail out */
   if (IS_SCAN_IN_PROGRESS(SCANCXT.scanState))
       return WF_ERROR_OPERATION_CANCELLED;

   WF_Scan(WF_SCAN_ALL);

   SCAN_SET_IN_PROGRESS(SCANCXT.scanState);
   /* Should they be invalidated??? */
   //SCAN_CLEAR_VALID(SCANCXT.scanState);

   return WF_SUCCESS;
}

uint16_t WFRetrieveScanResult(uint8_t Idx, tWFScanResult *p_ScanResult)
{
    if (Idx >= SCANCXT.numScanResults)
        return WF_ERROR_INVALID_PARAM;

    WF_ScanGetResult(Idx, p_ScanResult);
    p_ScanResult->ssid[p_ScanResult->ssidLen] = 0; /* Terminate */

    return WF_SUCCESS;
}

void WFScanEventHandler(uint16_t scanResults)
{
    /* Cache number APs found in scan */
    SCANCXT.numScanResults = scanResults;

    /* Clear the scan in progress */
    SCAN_CLEAR_IN_PROGRESS(SCANCXT.scanState);
    SCAN_SET_VALID(SCANCXT.scanState);

    return;
}
#endif /* EZ_CONFIG_SCAN */

#if defined ( CMD_PARSER ) && defined ( EZ_CONFIG_SCAN ) 
extern void
WFDisplayScanMgr()
{
    tWFScanResult   bssDesc;
    char ssid[32];
	char rssiChan[48];

    if (SCANCXT.numScanResults == 0)
       return;
    if (!IS_SCAN_STATE_DISPLAY(SCANCXT.scanState))
       return;

    if (IS_SCAN_IN_PROGRESS(SCANCXT.scanState))
       return;

    if (!IS_SCAN_STATE_VALID(SCANCXT.scanState))
       return;

    WFRetrieveScanResult(SCANCXT.displayIdx, &bssDesc);

    /* Display SSID */
    sprintf(ssid, "%s\r\n", bssDesc.ssid);
    SYS_CONSOLE_MESSAGE(ssid);

	/* Display SSID  & Channel */
    /* RSSI_MAX : 200, RSSI_MIN : 106 */
    sprintf(rssiChan, "  => RSSI: %u, Channel: %u\r\n", bssDesc.rssi, bssDesc.channel);
    SYS_CONSOLE_MESSAGE(rssiChan);

    if (++SCANCXT.displayIdx == SCANCXT.numScanResults)  
    {
        SCAN_CLEAR_DISPLAY(SCANCXT.scanState);
        SCANCXT.displayIdx = 0;
        #if defined(CMD_PARSER)
        WFConsoleReleaseConsoleMsg();
        #endif
    }

    return;
}
#endif /* CMD_PARSER */

#endif /* TCPIP_IF_MRF24W */
