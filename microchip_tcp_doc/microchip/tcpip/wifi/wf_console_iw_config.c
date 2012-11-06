/*******************************************************************************
  MRF24W Driver iwconfig

  Summary:
    Module for Microchip TCP/IP Stack
    
  Description:
    -Provides access to MRF24W WiFi controller
    -Reference: MRF24W Data sheet, IEEE 802.11 Standard
*******************************************************************************/

/*******************************************************************************
FileName:   wf_console_iw_config.c
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
//                                  Includes
//============================================================================

#include "tcpip_private.h"

#include "wf_console.h"

#include <ctype.h>
#if defined( WF_CONSOLE_IFCFGUTIL )

#include "wf_console_iw_config.h"
#include "wf_console_msgs.h"
#include "wf_console_msg_handler.h"

#if defined ( EZ_CONFIG_SCAN )
#include "wf_easy_config.h"
#endif /* EZ_CONFIG_SCAN */
//============================================================================
//                                  Constants
//============================================================================

//============================================================================
//                                  Globals
//============================================================================

static bool		iwconfigCbInitialized = false;
tWFIwconfigCb	iwconfigCb;

//============================================================================
//                                  Local Function Prototypes
//============================================================================
static void iwconfigDisplayStatus(void);
static bool iwconfigSetSsid(void);
static bool iwconfigSetMode(void);
static bool iwconfigSetChannel(void);
static bool iwconfigSetPower(void);
static bool iwconfigSetDomain(void);
static bool iwconfigSetRTS(void);
static bool iwconfigSetTxRate(void);

uint8_t g_hibernate_state = WF_HB_NO_SLEEP;
uint8_t g_wakeup_notice = false;

/*****************************************************************************
 * FUNCTION: do_iwconfig_cmd
 *
 * RETURNS: None
 *
 * PARAMS:    None
 *
 * NOTES:   Responds to the user invoking iwconfig
 *****************************************************************************/
void do_iwconfig_cmd(void)
{
	if (!g_hibernate_state && !iwconfigSetCb() )
		return;

    // if user only typed in iwconfig with no other parameters
    if (ARGC == 1u)
    {
		if (!g_hibernate_state)
			iwconfigDisplayStatus();
		else
			WFConsolePrintStr("The Wi-Fi module is in hibernate mode - command failed.", true);
		return;
    }

	if ( (2u <= ARGC) && (strcmp((char*)ARGV[1], "wakeup") == 0) )
    {
		if (!g_wakeup_notice)
    		g_wakeup_notice = true;
		WFConsolePrintStr("The Wi-Fi module is awake.", true);	
		return;
	}

	if ( (2u <= ARGC) && (strcmp((char*)ARGV[1], "hibernate") == 0) )
    {
		if (!g_hibernate_state)
		{
    		g_hibernate_state = WF_HB_ENTER_SLEEP;
			g_wakeup_notice = false;
			WFConsolePrintStr("The Wi-Fi module is in hibernate mode.", true);
		}
		else
			WFConsolePrintStr("The Wi-Fi module is in hibernate mode.", true);
		return;
	}
	
	if (g_hibernate_state)
	{
		WFConsolePrintStr("The Wi-Fi module is in hibernate mode - command failed.", true);
		return;
	}

	if ( (2u <= ARGC) && (strcmp((char*)ARGV[1], "ssid") == 0) )
    {
    	if (!g_hibernate_state && !iwconfigSetSsid())
			return;
	}
    else if ( (2u <= ARGC) && (strcmp((char*)ARGV[1], "mode") == 0) )
    {
    	if (!g_hibernate_state && !iwconfigSetMode())
			return;
	}
    else if ( (2u <= ARGC) && (strcmp((char*)ARGV[1], "channel") == 0) )
    {
    	if (!g_hibernate_state && !iwconfigSetChannel())
			return;
	}
    else if ( (2u <= ARGC) && (strcmp((char*)ARGV[1], "power") == 0) )
    {
    	if (!g_hibernate_state && !iwconfigSetPower())
			return;
	}
    else if ( (2u <= ARGC) && (strcmp((char*)ARGV[1], "domain") == 0) )
    {
    	if (!g_hibernate_state && !iwconfigSetDomain())
			return;
	}
    else if ( (2u <= ARGC) && (strcmp((char*)ARGV[1], "rts") == 0) )
    {
    	if (!g_hibernate_state && !iwconfigSetRTS())
			return;
	}
    else if ( (2u <= ARGC) && (strcmp((char*)ARGV[1], "txrate") == 0) )
    {
    	if (!g_hibernate_state && !iwconfigSetTxRate())
			return;
	}

	#if defined ( EZ_CONFIG_SCAN )
	else if ( (2u <= ARGC) && (strcmp((char*)ARGV[1], "scan") == 0) )
    {
    	if (!g_hibernate_state) {
			WFConsolePrintStr("Scan...", true);
        	if (WFStartScan() == WF_SUCCESS)
	    	{
				WFConsolePrintStr("Scan completed.", true);
            	SCAN_SET_DISPLAY(SCANCXT.scanState);
            	SCANCXT.displayIdx = 0;
        	}
			else
				WFConsolePrintStr("Scan failed.", true);	
    	}
	    return;
	}
	#endif /* EZ_CONFIG_SCAN */

    else
    {
		WFConsolePrintStr("Unknown parameter", true);
		return;
	}
}


/*****************************************************************************
 * FUNCTION: iwconfigSetCb
 *
 * RETURNS: true or false
 *
 * PARAMS:  None
 *
 * NOTES:   Set the iwconfigCb structure
 *****************************************************************************/
bool iwconfigSetCb(void)
{
	uint8_t cpId, newCpId;

	if ( !iwconfigCbInitialized ) // first time call of iwconfigSetCb
	{
		memset(&iwconfigCb, 0, sizeof(iwconfigCb));
		iwconfigCbInitialized = true;
	}

	WF_GetPowerSaveState(&iwconfigCb.powerSaveState);
	if (iwconfigCb.powerSaveState == WF_PS_HIBERNATE)
	{
		WFConsolePrintStr("WF device hibernated", true);
		return false;
	}

	WF_CMGetConnectionState(&iwconfigCb.connState, &cpId);

	if ( iwconfigCb.cpId == WF_CURRENT_CPID_NONE )
	{
		if ( cpId == WF_CURRENT_CPID_NONE )
		{
			WF_CPCreate(&newCpId);
			iwconfigCb.cpId = newCpId;
		}
		else if ( cpId == WF_CURRENT_CPID_LIST )
		{
			WFConsolePrintStr("Conection profile list not supported", true);
			return false;
		}
		else
		{
			iwconfigCb.cpId = cpId; // use the application-created profile
		}
	}
	else // WF_MIN_NUM_CPID <= iwconfigCb.cpId && iwconfigCb.cpId <= WF_MAX_NUM_CPID
	{
		if ( cpId == WF_CURRENT_CPID_NONE )
		{
			// continue to use iwconfigCb.cpId
		}
		else if ( cpId == WF_CURRENT_CPID_LIST )
		{
			WFConsolePrintStr("Conection profile list not supported", true);

			WF_CPDelete(iwconfigCb.cpId);
			iwconfigCb.cpId = WF_CURRENT_CPID_NONE;

			return false;
		}
		else if ( cpId != iwconfigCb.cpId )
		{
			WF_CPDelete(iwconfigCb.cpId);
			iwconfigCb.cpId = cpId; // use the application-created profile
		}
		else // cpId == iwconfigCb.cpId
		{
			// contine to use iwconfigCb.cpId
		}
	}

    if ((iwconfigCb.connState == WF_CSTATE_NOT_CONNECTED) || (iwconfigCb.connState == WF_CSTATE_CONNECTION_PERMANENTLY_LOST))
    {
        iwconfigCb.isIdle = true;
    }    
    else
    {
        iwconfigCb.isIdle = false;
    }    

	return true;
}

/*****************************************************************************
 * FUNCTION: iwconfigDisplayStatus
 *
 * RETURNS:	None
 *
 * PARAMS:	None
 *
 * NOTES:	Responds to the user invoking iwconfig with no parameters
 *****************************************************************************/
static void iwconfigDisplayStatus(void)
{
	uint8_t *p;
	uint8_t tmp;
	uint8_t connectionState;
	uint8_t cpId;

	union
	{
		struct
		{
			uint8_t List[WF_CHANNEL_LIST_LENGTH];
			uint8_t Num;
		} Channel;

		uint8_t Domain;

		struct
		{
			uint8_t String[WF_MAX_SSID_LENGTH+1];
			uint8_t Len;
		} Ssid;

		struct
		{
			uint8_t NetworkType;
		} Mode;

		struct
		{
			uint16_t Threshold;
		} Rts;
	} ws; // workspace

	// cpId
	{
		WFConsolePrintStr("\tcpid:     ", false);
		WFConsolePrintInteger(iwconfigCb.cpId, 'd');
		WFConsolePrintStr("", true);
	}

	// channel
	{
		WF_CAGetChannelList(ws.Channel.List, &ws.Channel.Num);
		WFConsolePrintStr("\tchannel:  ", false);

		p = ws.Channel.List;
		tmp = ws.Channel.Num;

		while ( --tmp > 0u )
		{
			WFConsolePrintInteger(*p, 'd');
			WFConsolePrintStr(",", false);
			p++;
        }

		WFConsolePrintInteger(*p, 'd');
		WFConsolePrintStr("", true);
	}

	// domain
	{
		WF_GetRegionalDomain(&ws.Domain);

		WFConsolePrintStr("\tdomain:   ", false);

		if ( ws.Domain == WF_DOMAIN_FCC )
		{
			WFConsolePrintStr("fcc", true);
		}
		else if ( ws.Domain == WF_DOMAIN_IC )
		{
			WFConsolePrintStr("ic", true);
		}
		else if ( ws.Domain == WF_DOMAIN_ETSI )
		{
			WFConsolePrintStr("etsi", true);
		}
		else if ( ws.Domain == WF_DOMAIN_SPAIN )
		{
			WFConsolePrintStr("spain", true);
		}
		else if ( ws.Domain == WF_DOMAIN_FRANCE )
		{
			WFConsolePrintStr("france", true);
		}
		else if ( ws.Domain == WF_DOMAIN_JAPAN_A )
		{
			WFConsolePrintStr("japana", true);
		}
		else if ( ws.Domain == WF_DOMAIN_JAPAN_B )
		{
			WFConsolePrintStr("japanb", true);
		}
		else
		{
			WFConsolePrintStr("unknown", true);
		}
	}

	// rts
	{
		WF_GetRtsThreshold(&ws.Rts.Threshold);

		WFConsolePrintStr("\trts:      ", false);
		WFConsolePrintInteger(ws.Rts.Threshold, 'd');
		WFConsolePrintStr("", true);
	}

	// mode
	{

        WF_CMGetConnectionState(&connectionState, &cpId);
        WF_CPGetNetworkType(iwconfigCb.cpId, &ws.Mode.NetworkType);
        
		WFConsolePrintStr("\tmode:     ", false);

		if (iwconfigCb.isIdle)
		{
    		if (iwconfigCb.connState == WF_CSTATE_NOT_CONNECTED)
    		{
		    	WFConsolePrintStr("idle", true);
		    }
		    else if (iwconfigCb.connState == WF_CSTATE_CONNECTION_PERMANENTLY_LOST)
		    {
		    	WFConsolePrintStr("idle (connection permanently lost)", true);    		    
    		}   
    		else
    		{
        		WFConsolePrintStr("idle (?)", true);    		    
            }  		 	
		}
		else
		{
			WF_CPGetNetworkType(iwconfigCb.cpId, &ws.Mode.NetworkType);
			if (ws.Mode.NetworkType == WF_INFRASTRUCTURE)
			{
                if (iwconfigCb.connState == WF_CSTATE_CONNECTION_IN_PROGRESS)
                {
    				WFConsolePrintStr("managed (connection in progress)", true);
    		    }
    		    else if (iwconfigCb.connState == WF_CSTATE_CONNECTED_INFRASTRUCTURE)
    		    {
	                WFConsolePrintStr("managed", true);        		    
        		}    
        		else if (iwconfigCb.connState == WF_CSTATE_RECONNECTION_IN_PROGRESS)
        		{
                    WFConsolePrintStr("managed (reconnection in progress)", true);        		    
                }  		
                else 
                {
                    WFConsolePrintStr("managed (?)", true);        		                        
                }    
			}
			else if (ws.Mode.NetworkType == WF_ADHOC)
			{
                if (iwconfigCb.connState == WF_CSTATE_CONNECTION_IN_PROGRESS)
                {
    				WFConsolePrintStr("adhoc (connection in progress)", true);
    		    }
    		    else if (iwconfigCb.connState == WF_CSTATE_CONNECTED_ADHOC)
    		    {
	                WFConsolePrintStr("adhoc", true);        		    
        		}    
        		else if (iwconfigCb.connState == WF_CSTATE_RECONNECTION_IN_PROGRESS)
        		{
                    WFConsolePrintStr("adhoc (reconnection in progress)", true);        		    
                }  		
                else 
                {
                    WFConsolePrintStr("adhoc (?)", true);        		                        
                }    

				WFConsolePrintStr("adhoc", true);
			}
			else
			{
				WFConsolePrintStr("unknown", true);
			}
		}
	}

	// ssid
	{
		WF_CPGetSsid(iwconfigCb.cpId, ws.Ssid.String, &ws.Ssid.Len);
		ws.Ssid.String[ws.Ssid.Len] = '\0';

		WFConsolePrintStr("\tssid:     ", false);
		WFConsolePrintStr(ws.Ssid.String, true);
	}

	// power
	{
		switch (iwconfigCb.powerSaveState)
		{
		case WF_PS_PS_POLL_DTIM_ENABLED:
			WFConsolePrintStr("\tpwrsave:  enabled", true);
			WFConsolePrintStr("\tdtim rx:  enabled", true);
			break;
		case WF_PS_PS_POLL_DTIM_DISABLED:
			WFConsolePrintStr("\tpwrsave:  enabled", true);
			WFConsolePrintStr("\tdtim rx:  disabled", true);
			break;
		case WF_PS_OFF:
			WFConsolePrintStr("\tpwrsave:  disabled", true);
			break;
		default:
			WFConsolePrintStr("\tpwrsave:  unknown(", false);
			WFConsolePrintInteger(iwconfigCb.powerSaveState, 'd');
			WFConsolePrintStr(")", true);
			break;
		}
	}
}

static bool iwconfigSetSsid(void)
{
	if (ARGC < 3u)
	{
		WFConsolePrintStr("Missing value for last parameter", true);
		return false;
	}

	if (ARGC > 3u)
	{
		WFConsolePrintStr("SSID may not contain space for this demo", true);
		return false;
	}

	WF_CPSetSsid(iwconfigCb.cpId, (uint8_t *)ARGV[2], strlen((char*)ARGV[2]));

	return true;
}

static bool iwconfigSetMode(void)
{
	uint8_t networkType;

	WF_CPGetNetworkType(iwconfigCb.cpId, &networkType);

    if ( (3u <= ARGC) && (strcmp((char*)ARGV[2], "idle") == 0) )
    {
		if ( iwconfigCb.isIdle )
		{
			WFConsolePrintStr("Already in the idle mode", true);
		}
		else
		{
			WF_CMDisconnect();
		}
	}
    else if ( (3u <= ARGC) && (strcmp((char*)ARGV[2], "managed") == 0) )
    {
		if ( iwconfigCb.isIdle )
		{
			WF_CPSetNetworkType(iwconfigCb.cpId, WF_INFRASTRUCTURE);
			WF_CMConnect(iwconfigCb.cpId);
		}
		else
		{
			WF_CPGetNetworkType(iwconfigCb.cpId, &networkType);
			if (networkType == WF_INFRASTRUCTURE)
			{
				WFConsolePrintStr("Already in the managed mode", true);
			}
			else
			{
				WF_CMDisconnect();

				WF_CPSetNetworkType(iwconfigCb.cpId, WF_INFRASTRUCTURE);
				WF_CMConnect(iwconfigCb.cpId);
			}
		}
	}
    else if ( (3u <= ARGC) && (strcmp((char*)ARGV[2], "adhoc") == 0) )
    {
        if ( iwconfigCb.isIdle )

        {
            WF_CPSetNetworkType(iwconfigCb.cpId, WF_ADHOC);
            WF_CPSetAdHocBehavior(iwconfigCb.cpId, WF_ADHOC_CONNECT_THEN_START);
            WF_CMConnect(iwconfigCb.cpId);
        }
		else
		{
			WF_CPGetNetworkType(iwconfigCb.cpId, &networkType);
			if (networkType == WF_ADHOC)
			{
				WFConsolePrintStr("Already in the adhoc mode", true);
			}
			else
			{
				WF_CMDisconnect();

				WF_CPSetNetworkType(iwconfigCb.cpId, WF_ADHOC);
				WF_CMConnect(iwconfigCb.cpId);
			}
		}
	}
	else
	{
		WFConsolePrintStr("Unknown parameter", true);
		return false;
	}

	return true;
}

static bool iwconfigSetChannel(void)
{
	uint8_t *p1, *p2;
	uint8_t *p_channelList;
	uint8_t index = 0;
	uint16_t temp;

	if (ARGC < 3u)
	{
		WFConsolePrintStr("Missing value for last parameter", true);
		return false;
	}

	if ( !iwconfigCb.isIdle )
	{
		WFConsolePrintStr("Channel can only be set in idle mode", true);
		return false;
	}

	p_channelList = (uint8_t*) ARGV[2];
	p1 = p2 = p_channelList;

    if ( strlen( (char*) p_channelList) == 0u )
    	return false;

	if ( (3u <= ARGC) && (strcmp((char*)ARGV[2], "all") == 0) )
	{
		WF_CASetChannelList(p_channelList, 0); // reset to domain default channel list
		return true;
	}

    do
    {
       if ( (p2 = (uint8_t*) strchr( (const char *) p1, (int) ',')) != NULL )
       {
          *p2='\0';
          p2++;
       }

       if( !ConvertASCIIUnsignedDecimalToBinary((int8_t *)p1, &temp) )
          return  false;

       p1 = p2;
       p_channelList[index] = (uint8_t) temp;
       index++;

    } while (  p2 != NULL );

    WF_CASetChannelList(p_channelList, index);

	return true;
}

static bool iwconfigSetPower(void)
{
	if (ARGC < 3u)
	{
		WFConsolePrintStr("Missing value for last parameter", true);
		return false;
	}

    if ( (3u <= ARGC) && (strcmp((char*)ARGV[2], "reenable") == 0) )
    {	// reenable power saving
		WF_PsPollEnable(true, AGGRESSIVE_PS);
	}
    else if ( (3u <= ARGC) && (strcmp((char*)ARGV[2], "disable") == 0) )
    {	// disable power saving
		WF_PsPollDisable();
	}
    else if ( (3u <= ARGC) && (strcmp((char*)ARGV[2], "unicast") == 0) )
    {	// enable power saving but don't poll for DTIM
		WF_PsPollEnable(false, AGGRESSIVE_PS);
	}
	else if ( (3u <= ARGC) && (strcmp((char*)ARGV[2], "all") == 0) )
    {	// enable power saving and poll for DTIM
		WF_PsPollEnable(true, AGGRESSIVE_PS);
	}
	else
	{
		WFConsolePrintStr("Unknown parameter", true);
		return false;
	}

	return true;
}

static bool iwconfigSetDomain(void)
{
	uint8_t domain;

	if (ARGC < 3u)
	{
		WFConsolePrintStr("Missing value for last parameter", true);
		return false;
	}

	if ( !iwconfigCb.isIdle )
	{
		WFConsolePrintStr("Domain can only be set in idle mode", true);
		return false;
	}

    if ( (3u <= ARGC) && (strcmp((char*)ARGV[2], "fcc") == 0) )
    {
		domain = WF_DOMAIN_FCC;
	}
    else if ( (3u <= ARGC) && (strcmp((char*)ARGV[2], "ic") == 0) )
    {
		domain = WF_DOMAIN_IC;
	}
    else if ( (3u <= ARGC) && (strcmp((char*)ARGV[2], "etsi") == 0) )
    {
		domain = WF_DOMAIN_ETSI;
	}
	else if ( (3u <= ARGC) && (strcmp((char*)ARGV[2], "spain") == 0) )
    {
		domain = WF_DOMAIN_SPAIN;
	}
	else if ( (3u <= ARGC) && (strcmp((char*)ARGV[2], "france") == 0) )
    {
		domain = WF_DOMAIN_FRANCE;
	}
	else if ( (3u <= ARGC) && (strcmp((char*)ARGV[2], "japana") == 0) )
    {
		domain = WF_DOMAIN_JAPAN_A;
	}
	else if ( (3u <= ARGC) && (strcmp((char*)ARGV[2], "japanb") == 0) )
    {
		domain = WF_DOMAIN_JAPAN_B;
	}
	else
	{
		WFConsolePrintStr("Unknown domain", true);
		return false;
	}

	WF_SetRegionalDomain(domain);
	WF_CASetChannelList(NULL, 0); // reset to domain default channel list

	return true;
}

static bool iwconfigSetRTS(void)
{
	uint16_t rtsThreshold;

	if (ARGC < 3u)
	{
		WFConsolePrintStr("Missing value for last parameter", true);
		return false;
	}

    if( !ConvertASCIIUnsignedDecimalToBinary(ARGV[2], &rtsThreshold) )
        return  false;

	WF_SetRtsThreshold(rtsThreshold);

	return true;
}

static bool iwconfigSetTxRate(void)
{
	return false;
}
#endif  /* WF_CONSOLE_IFCFGUTIL */
