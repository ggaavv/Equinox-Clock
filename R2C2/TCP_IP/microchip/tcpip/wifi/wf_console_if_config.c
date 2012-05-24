/*******************************************************************************
  MRF24W Driver ifconfig

  Summary:
    Module for Microchip TCP/IP Stack
    
  Description:
    -Provides access to MRF24W WiFi controller
    -Reference: MRF24W Data sheet, IEEE 802.11 Standard
*******************************************************************************/

/*******************************************************************************
FileName:   wf_console_if_config.c
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

#include <ctype.h>

#include "wf_console.h"

#if defined( WF_CONSOLE_IFCFGUTIL )

#include "wf_console_if_config.h"
#include "wf_console_msgs.h"
#include "wf_console_msg_handler.h"

//============================================================================
//                                  Constants
//============================================================================

//============================================================================
//                                  Globals
//============================================================================
extern uint8_t g_hibernate_state;

//============================================================================
//                                  Local Function Prototypes
//============================================================================
static void IfconfigDisplayStatus(void);
static bool       isIPAddress(int8_t *p_string, uint8_t *p_Address);
static bool       isMacAddress(int8_t *p_string, uint8_t *p_Address);
#if defined(TCPIP_STACK_USE_DHCP_CLIENT)
static void setDHCPState(bool enable);
#endif
static void missingValue(void);
static void notHandledParam(uint8_t index);

void LCDDisplayIPValue(IP_ADDR IPVal)
{
    BYTE IPDigit[4];
    BYTE i;
	char message[16 + 1];

	strcpy(message, "");
    for(i = 0; i < sizeof(IP_ADDR); i++)
    {
        uitoa((uint16_t)IPVal.v[i], IPDigit);

		strcat(message, (char *)IPDigit);

        if(i == sizeof(IP_ADDR)-1)
            break;
        strcat(message, ".");
    }

	SYS_OUT_MESSAGE_LINE(message, 1);
}

/*****************************************************************************
 * FUNCTION: do_ifconfig_cmd
 *
 * RETURNS: None
 *
 * PARAMS:  None
 *
 * NOTES:   Responds to the user invoking ifconfig
 *****************************************************************************/
void do_ifconfig_cmd(void)
{
     uint8_t   address[6];
     uint8_t conState, cpId;
     NET_CONFIG *p_netConfig;
     
     p_netConfig = GetNetworkConfig();

	
    // if user only typed in ifconfig with no other parameters
    if (ARGC == 1u)
    {
        IfconfigDisplayStatus();
		return;
    }

	if (g_hibernate_state)
	{
		WFConsolePrintStr("The Wi-Fi module is in hibernate mode - command failed.", true);
		return;
	}

#if defined(WF_CM_DEBUG)
	else if ( (ARGC == 2u) && !strcmp((char *) ARGV[1], "info") )
	{
        uint8_t i;
		tWFCMInfoFSMStats cm_stats;

		WF_CMInfoGetFSMStats(&cm_stats);
		for (i = 0; i < 12; i++)
		{
    		sprintf( (char *) g_ConsoleContext.txBuf,
            		"[%02X]: %02X%02X %02X%02X",   
					i, 
					cm_stats.byte[i*4 + 0],
					cm_stats.byte[i*4 + 1],
					cm_stats.byte[i*4 + 2],
					cm_stats.byte[i*4 + 3]
					);
    		WFConsolePrintStr( (char *) g_ConsoleContext.txBuf , true);
		}
	}
	else if ( (ARGC == 2u) && !strcmp((char *) ARGV[1], "scan") )
	{
		WF_Scan(1); // scan, using CP 1
	}
	else if ( (ARGC == 2u) && !strcmp((char *) ARGV[1], "scanget") ) //"scangetresult"
	{
		tWFScanResult pScanResult[1];

		WF_ScanGetResult(0, pScanResult);
	}
	else if ( (ARGC == 2u) && !strcmp((char *) ARGV[1], "cpgete") ) //"cpgetelements"
	{
		tWFCPElements pCPElements[1];

		WF_CPGetElements(1, pCPElements);
	}
#endif
    // else if 2 arguments and the second arg is IP address
    else if ( (ARGC == 2u) && (isIPAddress(ARGV[1], address)) )
    {
        #if defined(TCPIP_STACK_USE_DHCP_CLIENT)
        if (DHCPIsEnabled(0))
        {
          WFConsolePrintStr(
                     "Static IP address should not be set with DHCP enabled", true);
          return;
        }
        #endif

        p_netConfig->MyIPAddr.v[0] = address[0];
        p_netConfig->MyIPAddr.v[1] = address[1];
        p_netConfig->MyIPAddr.v[2] = address[2];
        p_netConfig->MyIPAddr.v[3] = address[3];

        /* Microchip DHCP client clobbers static ip on every iteration of loop, even if dhcp is turned off*/
        p_netConfig->DefaultIPAddr.v[0] = address[0];
        p_netConfig->DefaultIPAddr.v[1] = address[1];
        p_netConfig->DefaultIPAddr.v[2] = address[2];
        p_netConfig->DefaultIPAddr.v[3] = address[3];

        LCDDisplayIPValue(p_netConfig->MyIPAddr);
    }
    // else if 2 args and second arg is MAC address
    else if ( (ARGC == 2u) && isMacAddress(ARGV[1], address))
    {
        /* Can only set MAC address in idle state */
        WF_CMGetConnectionState(&conState, &cpId);
        if ( conState != WF_CSTATE_NOT_CONNECTED )
        {
            WFConsolePrintStr("HW MAC address can only be set in idle mode", true);
            return;
        }

         WF_SetMacAddress( address );
    }
	else if ( (2u <= ARGC) && (strcmp((char *)ARGV[1], (const FAR char *)"netmask") == 0) )
	{
		if (ARGC != 3u)
		{
			missingValue();
			return;
		}

        #if defined(TCPIP_STACK_USE_DHCP_CLIENT)
        if ( DHCPIsEnabled(0) )
        {
            WFConsolePrintStr(
                "The Netmask should not be set with DHCP enabled", true);
            return;
        }
        #endif

        if ( !isIPAddress(ARGV[2], address) )
        {
            WFConsolePrintStr("Invalid netmask value", true);
            return;
        }

        p_netConfig->MyMask.v[0] = address[0];
        p_netConfig->MyMask.v[1] = address[1];
        p_netConfig->MyMask.v[2] = address[2];
        p_netConfig->MyMask.v[3] = address[3];

        /* Microchip DHCP client clobbers static netmask on every iteration of loop, even if dhcp is turned off*/
        p_netConfig->DefaultMask.v[0] = address[0];
        p_netConfig->DefaultMask.v[1] = address[1];
        p_netConfig->DefaultMask.v[2] = address[2];
        p_netConfig->DefaultMask.v[3] = address[3];  
	}
	else if ( (2u <= ARGC) && (strcmp((char *)ARGV[1], (const FAR char *)"gateway") == 0) )
	{
		if (ARGC != 3u)
		{
			missingValue();
			return;
		}

        if ( !isIPAddress(ARGV[2], address) )
        {
            WFConsolePrintStr("Invalid gateway value", true);
            return;
        }

        p_netConfig->MyGateway.v[0] = address[0];
        p_netConfig->MyGateway.v[1] = address[1];
        p_netConfig->MyGateway.v[2] = address[2];
        p_netConfig->MyGateway.v[3] = address[3];
	}
	else if ( (2u <= ARGC) && (strcmp((char*)ARGV[1], "auto-dhcp") == 0) )
	{
		if (ARGC != 3u)
		{
			missingValue();
			return;
		}

        #if defined(TCPIP_STACK_USE_DHCP_CLIENT)
		if (strcmp((char*)ARGV[2], "start") == 0)
		{
			setDHCPState(true);
		}
		else if (strcmp((char*)ARGV[2], "stop") == 0)
		{
			setDHCPState(false);
		}
		else
		#endif
		{
			WFConsolePrintStr("   Invalid dhcp param", true);
			return;
		}
	}
    else
    {
        notHandledParam(1);
	}
}

static void missingValue(void)
{
    WFConsolePrintStr(
        "Missing value after last parameter", true);
}

static void notHandledParam(uint8_t index)
{
    WFConsolePrintStr("Param ", false);
	WFConsolePrintInteger(index, 'd');
	WFConsolePrintStr(" not handled", true);
}

/*****************************************************************************
 * FUNCTION: isIPAddress
 *
 * RETURNS: True if valid IP address, else False
 *
 * PARAMS:    p_string  -- string to check
 *          p_Address -- Array where IP values will be written
 *
 * NOTES:   Determines if the input string is a valid dot-notation IP address.
 *          If it is, then returns an array of 4 bytes for each of the values.
 *          IP address
 *****************************************************************************/

static bool isIPAddress(int8_t *p_string, uint8_t *p_Address)
{
    uint8_t digIndex = 0;
    uint8_t bufIndex = 0;
    uint8_t dotCount = 0;
    int8_t buf[4];
    uint8_t  i;
    uint16_t tmp;

    memset(buf, 0x00, sizeof(buf));
    for (i = 0; i < strlen((char *)p_string); ++i)
    {
        // if gathering digits
        if (isdigit(p_string[i]))
        {
            // store digit in buf, fail if user has more than 3 digits
            buf[bufIndex++] = p_string[i];
            if (bufIndex > 3u)
            {
                return false;
            }
        }
        // else encountered a dot
        else if (p_string[i] == (int8_t)'.')
        {
            // keep track of dots and fail if we encounter too many of them
            ++dotCount;
            if (dotCount > 3u)
            {
                return false;
            }

            // convert the number we just pulled from the input string, fail if not a number
            if (!ConvertASCIIUnsignedDecimalToBinary(buf, &tmp))
            {
                return false;
            }
            // else a valid number
            else
            {
                // fail if greater than 255
                if ( tmp > 255u)
                {
                    return false;
                }

                 p_Address[digIndex] = (uint8_t) (tmp & 0xFF);

                // get ready for next number
                memset(buf, 0x00, sizeof(buf));
                bufIndex = 0;
                ++digIndex;
            }
        }
        // else got a character that is neither number nor dot
        else
        {
            return false;
        }

    }

    // fail if more than 3 dots
    if (dotCount != 3u)
    {
        return false;
    }

    // if made it here then make sure we have the last number
    if (buf[0] == 0)
    {
        return false;
    }

    // convert last number to binary, fail if we can't
    if (!ConvertASCIIUnsignedDecimalToBinary(buf, &tmp))
    {
        return false;
    }

    p_Address[digIndex] = (uint8_t) (tmp & 0xFF);

    // IP digits will be in p_Address[]
    return true;
}


/*****************************************************************************
 * FUNCTION: isMacAddress
 *
 * RETURNS: True if valid MAC address, else False
 *
 * PARAMS:    p_string  -- string to check
 *          p_Address -- Array where MAC values will be written
 *
 * NOTES:   Determines if the input string is a valid MAC address.
 *          If it is, then returns an array of 6 bytes for each of the values.
 *          MAC address must be in hex in the format xx:xx:xx:xx:xx:xx
 *****************************************************************************/
static bool isMacAddress(int8_t *p_string, uint8_t *p_Address)
{
    uint8_t i;
    uint16_t tmp;

    if (strlen((char *)p_string) != 17u)
    {
        return false;
    }

    // ensure the ':' is in the right place, and if so, set them to 0
    for (i = 2; i < 17u; i += 3)
    {
        if (p_string[i] == (int8_t)':')
        {
            p_string[i] = '\0';
        }
        else
        {
            return false;
        }
    }

    // now extract each hex number string
    for (i = 0; i < 6u;  ++i)
    {
        if (!ConvertASCIIHexToBinary(&p_string[i * 3], &tmp))
        {
            return false;
        }

        p_Address[i] = (uint8_t) (tmp & 0xFF);

    }

    return true;
}

/*****************************************************************************
 * FUNCTION: IfconfigDisplayStatus
 *
 * RETURNS: None
 *
 * PARAMS:    None
 *
 * NOTES:   Responds to the user invoking ifconfig with no parameters
 *****************************************************************************/
static void IfconfigDisplayStatus(void)
{
    NET_CONFIG *p_netConfig;
    
    p_netConfig = GetNetworkConfig();
    
    sprintf( (char *) g_ConsoleContext.txBuf,
              "\tIP addr:  %d.%d.%d.%d",   p_netConfig->MyIPAddr.v[0],
                                           p_netConfig->MyIPAddr.v[1],
                                           p_netConfig->MyIPAddr.v[2],
                                           p_netConfig->MyIPAddr.v[3] );
    WFConsolePrintStr( (char *) g_ConsoleContext.txBuf , true);


    sprintf( (char *) g_ConsoleContext.txBuf,
             "\tMAC addr: %02X:%02X:%02X:%02X:%02X:%02X",   p_netConfig->MyMACAddr.v[0],
                                                            p_netConfig->MyMACAddr.v[1],
                                                            p_netConfig->MyMACAddr.v[2],
                                                            p_netConfig->MyMACAddr.v[3],
                                                            p_netConfig->MyMACAddr.v[4],
                                                            p_netConfig->MyMACAddr.v[5]);
    WFConsolePrintStr( (char *) g_ConsoleContext.txBuf , true);



    sprintf( (char *) g_ConsoleContext.txBuf,
              "\tNetmask:  %d.%d.%d.%d",   p_netConfig->MyMask.v[0],
                                           p_netConfig->MyMask.v[1],
                                           p_netConfig->MyMask.v[2],
                                           p_netConfig->MyMask.v[3] );
    WFConsolePrintStr( (char *) g_ConsoleContext.txBuf , true);


    sprintf( (char *) g_ConsoleContext.txBuf,
              "\tGateway:  %d.%d.%d.%d",   p_netConfig->MyGateway.v[0],
                                           p_netConfig->MyGateway.v[1],
                                           p_netConfig->MyGateway.v[2],
                                           p_netConfig->MyGateway.v[3] );
    WFConsolePrintStr( (char *) g_ConsoleContext.txBuf , true);

    #if defined(TCPIP_STACK_USE_DHCP_CLIENT)
    if ( DHCPIsEnabled(0) )
       WFConsolePrintStr("\tDHCP:     Started", true);
    else
       WFConsolePrintStr("\tDHCP:     Stopped", true);
    #endif
}

#if defined(TCPIP_STACK_USE_DHCP_CLIENT)
/*****************************************************************************
 * FUNCTION: setDHCPState
 *
 * RETURNS: None
 *
 * PARAMS:  enable -- a boolean indicating whether to enable DHCP or not
 *
 * NOTES:   Enable or disable DHCP operation
 *****************************************************************************/
static void setDHCPState(bool enable)
{
    NET_CONFIG *p_netConfig;
    TCPIP_NET_HANDLE handle;
    
    handle = TCPIP_STACK_NetHandle("MRF24W");
    
    
    
    p_netConfig = GetNetworkConfig();    
    
    if ( enable )
    {
        p_netConfig->Flags.bIsDHCPEnabled = true;
        DHCPEnable(handle);
    }
    else
    {
        p_netConfig->Flags.bIsDHCPEnabled = false;
        DHCPDisable(handle, false);
    }
}
#endif

#endif /* WF_CONSOLE_IFCFGUTIL */


