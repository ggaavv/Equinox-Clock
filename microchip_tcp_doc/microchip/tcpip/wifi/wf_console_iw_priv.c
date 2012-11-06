/*******************************************************************************
  MRF24W10C Driver iwpriv

  Summary:
    Module for Microchip TCP/IP Stack
    
  Description:
    - Provides access to MRF24W10C WiFi controller
    - Reference: MRF24W10C Data sheet, IEEE 802.11 Standard
*******************************************************************************/

/*******************************************************************************
FileName:   wf_console_iw_priv.c
Copyright � 2012 released Microchip Technology Inc.  All rights
reserved.

Microchip licenses to you the right to use, modify, copy and distribute
Software only when embedded on a Microchip microcontroller or digital signal
controller that is integrated into your product or third party product
(pursuant to the sublicense terms in the accompanying license agreement).

You should refer to the license agreement accompanying this Software for
additional information regarding your rights and obligations.

SOFTWARE AND DOCUMENTATION ARE PROVIDED �AS IS� WITHOUT WARRANTY OF ANY KIND,
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


#include <ctype.h>
#include "wf_console.h"

#if defined( WF_CONSOLE_IFCFGUTIL )

#include "wf_console_msgs.h"
#include "wf_console_msg_handler.h"
#include "wf_console_iw_config.h" // for using iwconfigCb
#include "wf_console_iw_priv.h"


//============================================================================
//                                  Constants
//============================================================================

#define IWPRIV_WEB_KEY_NUM          (4u)
#define IWPRIV_WEB_LONG_KEY_LEN		(13u) // in byte
#define IWPRIV_WEB_SHORT_KEY_LEN	(5u) // in byte

//============================================================================
//                                  Globals
//============================================================================

static bool		iwprivCbInitialized = false;
static struct
{
	// NOTE: cpId, ssid, and wepDefaultKeyId
    // are refreshed at each iwpriv

	uint8_t	cpId; // conn. profile ID
	uint8_t ssid[WF_MAX_SSID_LENGTH+1];
	uint8_t wepDefaultKeyId;

	// NOTE: securityType, securityKey and securityKeyLength are
    // not refreshed at each iwpriv

	uint8_t	securityType;
	uint8_t securityKey[WF_MAX_SECURITY_KEY_LENGTH+1];
	uint8_t securityKeyLength;
} iwprivCb;

extern uint8_t g_hibernate_state;
//============================================================================
//                                  Local Function Prototypes
//============================================================================

/*****************************************************************************
 * FUNCTION: iwprivSetCb
 *
 * RETURNS: true or false
 *
 * PARAMS:  None
 *
 * NOTES:   Set the iwprivCb structure
 *****************************************************************************/
static bool iwprivSetCb(void)
{
	tWFCPElements cprof;
	bool cpIdChanged = false;

	if ( !iwprivCbInitialized ) // first time call of iwprivSetCb
	{
		memset(&iwprivCb, 0, sizeof(iwprivCb));
		iwprivCbInitialized = true;
	}

	if (!g_hibernate_state && !iwconfigSetCb() ) // first set iwconfigCb
		return false;

    if ( iwprivCb.cpId != iwconfigCb.cpId)
    {
        iwprivCb.cpId = iwconfigCb.cpId;
        cpIdChanged = true;
    }    
        
    WF_CPGetElements(iwprivCb.cpId, &cprof);
        
	// set refreshable part of iwprivCb
	{
		memcpy((void*)iwprivCb.ssid, (const void*)cprof.ssid, cprof.ssidLength);
		iwprivCb.ssid[cprof.ssidLength] = '\0';

		iwprivCb.wepDefaultKeyId = cprof.wepDefaultKeyId;
	}
	
	// set non-refreshable part of iwprivCb only when cpId has changed
	if (cpIdChanged)
	{
 		iwprivCb.securityType = cprof.securityType;
 		if (iwprivCb.securityType == WF_SECURITY_WPA_WITH_KEY || iwprivCb.securityType == WF_SECURITY_WPA2_WITH_KEY)
 		{
     		iwprivCb.securityType = WF_SECURITY_WPA_AUTO_WITH_KEY;
        }
        else if (iwprivCb.securityType == WF_SECURITY_WPA_WITH_PASS_PHRASE || iwprivCb.securityType == WF_SECURITY_WPA2_WITH_PASS_PHRASE)
        {
     		iwprivCb.securityType = WF_SECURITY_WPA_AUTO_WITH_PASS_PHRASE;
        }
              		
 		iwprivCb.securityKeyLength = 0;
    }   	

	return true;
}

static void iwprivDisplayStatus(void)
{
	uint8_t i, j;
	uint8_t* p;

	// security type
	{
		WFConsolePrintStr("Encryption: ", false);

		switch (iwprivCb.securityType)
		{
		case WF_SECURITY_OPEN:
			WFConsolePrintStr("none", true);
			break;
		case WF_SECURITY_WEP_40:
		case WF_SECURITY_WEP_104:
			WFConsolePrintStr("wep", true);
			break;
		case WF_SECURITY_WPA_WITH_KEY:
		case WF_SECURITY_WPA2_WITH_KEY:
		case WF_SECURITY_WPA_AUTO_WITH_KEY:
			WFConsolePrintStr("wpa-psk", true);
			break;
		case WF_SECURITY_WPA_WITH_PASS_PHRASE:
		case WF_SECURITY_WPA2_WITH_PASS_PHRASE:
		case WF_SECURITY_WPA_AUTO_WITH_PASS_PHRASE:
			WFConsolePrintStr("wpa-phrase", true);
			break;
		default:
			WFConsolePrintStr("unknown", true);
			return;
		}
	}

	if ( iwprivCb.securityType == WF_SECURITY_WEP_40 || iwprivCb.securityType == WF_SECURITY_WEP_104 )
	{
		uint8_t webKeyLen;

		if ( iwprivCb.securityKeyLength == (IWPRIV_WEB_KEY_NUM * IWPRIV_WEB_LONG_KEY_LEN) )
		{
			webKeyLen = IWPRIV_WEB_LONG_KEY_LEN;
		}
		else if ( iwprivCb.securityKeyLength == (IWPRIV_WEB_KEY_NUM * IWPRIV_WEB_SHORT_KEY_LEN) )
		{
			webKeyLen = IWPRIV_WEB_SHORT_KEY_LEN;
		}
		else
		{
			WFConsolePrintStr("  Wep key: not yet set or unknown", true);
			return;
		}

		p = iwprivCb.securityKey;
        for( j=0; j < IWPRIV_WEB_KEY_NUM ; j++ )
        {
            if ( j == iwprivCb.wepDefaultKeyId )
                WFConsolePrintStr(" *", false);
            else
                WFConsolePrintStr("  ", false);

			WFConsolePrintStr("Wep key[", false);
			WFConsolePrintInteger(j+1, false);
			WFConsolePrintStr("]:  0x", false);

            for ( i=0; i < webKeyLen ; i++ )
            {
                sprintf( (char *) g_ConsoleContext.txBuf,
                    "%.2x", *p++);
                WFConsolePrintStr( (char *) g_ConsoleContext.txBuf, false);
            }

            WFConsolePrintStr("", true);
        }
	}
	else if ( iwprivCb.securityType == WF_SECURITY_WPA_AUTO_WITH_KEY )
	{
		if ( iwprivCb.securityKeyLength != WF_WPA_KEY_LENGTH )
		{
			WFConsolePrintStr("  PSK: not yet set or unknown", true);
			return;
		}

		SYS_CONSOLE_MESSAGE("  PSK: \"");

		p = iwprivCb.securityKey;
        for( j=0; j < WF_WPA_KEY_LENGTH ; j++ )
        {
			sprintf( (char *) g_ConsoleContext.txBuf,
				"%.2x", *p++);
			WFConsolePrintStr( (char *) g_ConsoleContext.txBuf , false );
		}

        WFConsolePrintStr("", true);
	}
	else if ( iwprivCb.securityType == WF_SECURITY_WPA_AUTO_WITH_PASS_PHRASE )
	{
		if ( iwprivCb.securityKeyLength == 0 )
		{
			WFConsolePrintStr("  Phrase: not yet set or unknown", true);
			return;
		}

		WFConsolePrintStr("  Phrase: \"", false);

		p = iwprivCb.securityKey;
        for( j=0; j < iwprivCb.securityKeyLength ; j++ )
        {
			sprintf( (char *) g_ConsoleContext.txBuf,
				"%c", *p++);
			WFConsolePrintStr( (char *) g_ConsoleContext.txBuf, false );
		}

		WFConsolePrintStr("\"", true);

		WFConsolePrintStr("  SSID: ", false);
		WFConsolePrintStr(iwprivCb.ssid, true);
	}
}

static bool iwprivSetEnc(void)
{
	uint8_t securityType;

	if (ARGC < 3u)
	{
		WFConsolePrintStr("Missing value for last parameter", true);
		return false;
	}

    if ( (3u <= ARGC) && (strcmp((char*)ARGV[2], "none") == 0) )
    {
		securityType = WF_SECURITY_OPEN;
	}
	else if ( (3u <= ARGC) && (strcmp((char*)ARGV[2], "wep") == 0) )
    {
		securityType = WF_SECURITY_WEP_40; // by default
	}
    else if ( (3u <= ARGC) && (strcmp((char*)ARGV[2], "wpa-psk") == 0) )
    {
		securityType = WF_SECURITY_WPA_AUTO_WITH_KEY;

	}
    else if ( (3u <= ARGC) && (strcmp((char*)ARGV[2], "wpa-phrase") == 0) )
    {
		securityType = WF_SECURITY_WPA_AUTO_WITH_PASS_PHRASE;
	}
	else
	{
		WFConsolePrintStr("Unknown parameter", true);
		return false;
	}

	if ( iwprivCb.securityType != securityType ) // security type changed
	{	// reset the security context
		memset(iwprivCb.securityKey, 0, sizeof(iwprivCb.securityKey));
		iwprivCb.securityKeyLength = 0;
	}
	
	iwprivCb.securityType = securityType; // save the security type
	
	if (iwprivCb.securityType == WF_SECURITY_OPEN)
	{
        WF_CPSetSecurity(iwprivCb.cpId, iwprivCb.securityType, 0, NULL, 0);
    }   	

	return true;
}

static bool iwprivSetKey(void)
{
	uint8_t webKey;

	if (iwprivCb.securityType != WF_SECURITY_WEP_40 && iwprivCb.securityType != WF_SECURITY_WEP_104)
	{
		WFConsolePrintStr("WEP encryption mode is not selected", true);
		return false;
	}

	if (ARGC < 3u)
	{
		WFConsolePrintStr("Missing value for last parameter", true);
		return false;
	}

    if ( (3u <= ARGC) && (strcmp((char*)ARGV[2], "[1]") == 0) )
    {
		webKey = 0u;
	}
	else if ( (3u <= ARGC) && (strcmp((char*)ARGV[2], "[2]") == 0) )
    {
		webKey = 1u;
	}
    else if ( (3u <= ARGC) && (strcmp((char*)ARGV[2], "[3]") == 0) )
    {
		webKey = 2u;

	}
    else if ( (3u <= ARGC) && (strcmp((char*)ARGV[2], "[4]") == 0) )
    {
		webKey = 3u;
	}
	else
	{
		WFConsolePrintStr("Invalid WEP key index", true);
		return false;
	}

	if (4u <= ARGC)
	{
		if ( convertAsciiToHexInPlace(ARGV[3], IWPRIV_WEB_LONG_KEY_LEN) ) // for long web key
		{
    		iwprivCb.securityType = WF_SECURITY_WEP_104;
    		
			memcpy((void*)(iwprivCb.securityKey + webKey * IWPRIV_WEB_LONG_KEY_LEN), (const void*)ARGV[3], IWPRIV_WEB_LONG_KEY_LEN);
			iwprivCb.securityKeyLength = IWPRIV_WEB_KEY_NUM * IWPRIV_WEB_LONG_KEY_LEN;
			
			WF_CPSetSecurity(iwprivCb.cpId, iwprivCb.securityType, webKey,
			    iwprivCb.securityKey + webKey * IWPRIV_WEB_LONG_KEY_LEN, IWPRIV_WEB_LONG_KEY_LEN);
		}
		else if ( convertAsciiToHexInPlace(ARGV[3], IWPRIV_WEB_SHORT_KEY_LEN) ) // for short web key
		{
    		iwprivCb.securityType = WF_SECURITY_WEP_40;
    		
			memcpy((void*)(iwprivCb.securityKey + webKey * IWPRIV_WEB_SHORT_KEY_LEN), (const void*)ARGV[3], IWPRIV_WEB_SHORT_KEY_LEN);
			iwprivCb.securityKeyLength = IWPRIV_WEB_KEY_NUM * IWPRIV_WEB_SHORT_KEY_LEN;
			
			WF_CPSetSecurity(iwprivCb.cpId, iwprivCb.securityType, webKey,
			    iwprivCb.securityKey + webKey * IWPRIV_WEB_SHORT_KEY_LEN, IWPRIV_WEB_SHORT_KEY_LEN);
		}
		else
		{
			WFConsolePrintStr("64/128bit WEP key format not valid", true);
			return false;
		}
	}
	else // ARGC == 3u
	{
		WF_CPSetDefaultWepKeyIndex(iwprivCb.cpId, webKey);
	}

	return true;
}

static bool iwprivSetPsk(void)
{
	if ( iwprivCb.securityType != WF_SECURITY_WPA_AUTO_WITH_KEY )
	{
		WFConsolePrintStr("WPA-PSK encryption mode is not selected", true);
		return false;
	}

	if (ARGC < 3u)
	{
		WFConsolePrintStr("Missing value for last parameter", true);
		return false;
	}

	if ( convertAsciiToHexInPlace(ARGV[2], WF_WPA_KEY_LENGTH) )
	{
		memcpy((void*)iwprivCb.securityKey, (const void*)ARGV[2], WF_WPA_KEY_LENGTH);
		iwprivCb.securityKeyLength = WF_WPA_KEY_LENGTH;
	}
	else
	{
		WFConsolePrintStr("WPA PSK must be exactly 32 bytes", true);
		return false;
	}

	WF_CPSetSecurity(iwprivCb.cpId, iwprivCb.securityType, 0, iwprivCb.securityKey, iwprivCb.securityKeyLength);

	return true;
}

static bool iwprivSetPhrase(void)
{
	uint8_t j;
	uint8_t securityType;
	uint8_t* phraseStart;
	uint8_t* phraseEnd;
	uint8_t phraseLen;

	if ( iwprivCb.securityType == WF_SECURITY_WPA_AUTO_WITH_KEY || iwprivCb.securityType == WF_SECURITY_WPA_AUTO_WITH_PASS_PHRASE )
	{
		securityType = WF_SECURITY_WPA_AUTO_WITH_PASS_PHRASE;
	}
	else
	{
		WFConsolePrintStr("WPA-PSK or WPA-PHRASE encryption mode is not selected", true);
		return false;
	}

	if (ARGC < 3u)
	{
		WFConsolePrintStr("Missing value for last parameter", true);
		return false;
	}

	phraseStart = (uint8_t*) ARGV[2];
	if (*phraseStart == '\"') // cancatenate remaining args into one string
	{
		for (j = 2; j < (ARGC-1); j++)
		{
			uint8_t argLen = strlen((char*)ARGV[j]);
			ARGV[j][argLen] = ' '; // replace '\0' with ' '
		}

		// searching for an ending quote
		phraseEnd = phraseStart + strlen((char *)phraseStart) - 1;
		while (*phraseEnd != '\"')
			phraseEnd--;

		// remove the double quotes
		phraseStart++;
		phraseEnd--;
	}
	else // a single word
	{
		phraseEnd = phraseStart + strlen((char *)phraseStart) - 1;
	}

	phraseLen = phraseEnd - phraseStart + 1;
	if (phraseLen < WF_MIN_WPA_PASS_PHRASE_LENGTH || WF_MAX_WPA_PASS_PHRASE_LENGTH < phraseLen)
	{
		WFConsolePrintStr("Phrase string must be at least 8 chars and no greater than 64", true);
		return false;
	}
	
	iwprivCb.securityType = securityType;

	memcpy((void*)iwprivCb.securityKey, (const void*)phraseStart, phraseLen);
	iwprivCb.securityKey[phraseLen] = '\0'; // just for easy printing on the console
	iwprivCb.securityKeyLength = phraseLen;

	WF_CPSetSecurity(iwprivCb.cpId, iwprivCb.securityType, 0,
	    iwprivCb.securityKey, iwprivCb.securityKeyLength);

	return true;
}

/*****************************************************************************
* FUNCTION: do_iwpriv_cmd
*
* RETURNS: None
*
* PARAMS:    None
*
* NOTES:   Responds to the user invoking ifconfig
*****************************************************************************/
void do_iwpriv_cmd(void)
{
	if (g_hibernate_state)
	{
		WFConsolePrintStr("The Wi-Fi module is in hibernate mode - command failed.", true);
		return;
	}

	if ( !iwprivSetCb() )
			return;

    // if user only typed in iwpriv with no other parameters
    if (ARGC == 1u)
    {
		iwprivDisplayStatus();
		return;
    }

	if ( !iwconfigCb.isIdle )
	{
		WFConsolePrintStr("Security context modification can be only done in the idle state", true);
		return;
	}

    if ( (2u <= ARGC) && (strcmp((char*)ARGV[1], "enc") == 0) )
    {
    	if ( !iwprivSetEnc() )
			return;
	}
    else if ( (2u <= ARGC) && (strcmp((char*)ARGV[1], "key") == 0) )
    {
    	if ( !iwprivSetKey() )
			return;
	}
    else if ( (2u <= ARGC) && (strcmp((char*)ARGV[1], "psk") == 0) )
    {
    	if ( !iwprivSetPsk() )
			return;
	}
    else if ( (2u <= ARGC) && (strcmp((char*)ARGV[1], "phrase") == 0) )
    {
    	if ( !iwprivSetPhrase() )
			return;
	}
    else
    {
		WFConsolePrintStr("Unknown parameter", true);
		return;
	}
}

#endif /* WF_CONSOLE_IFCFGUTIL */


