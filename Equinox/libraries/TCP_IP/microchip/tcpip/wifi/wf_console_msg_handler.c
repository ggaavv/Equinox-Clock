/*******************************************************************************
  MRF24W Driver Console Msg Handler

  Summary:
    Module for Microchip TCP/IP Stack
    
  Description:
    -Provides access to MRF24W WiFi controller
    -Reference: MRF24W Data sheet, IEEE 802.11 Standard
*******************************************************************************/

/*******************************************************************************
FileName:   wf_console_msg_handler.h
Copyright ?2012 released Microchip Technology Inc.  All rights
reserved.

Microchip licenses to you the right to use, modify, copy and distribute
Software only when embedded on a Microchip microcontroller or digital signal
controller that is integrated into your product or third party product
(pursuant to the sublicense terms in the accompanying license agreement).

You should refer to the license agreement accompanying this Software for
additional information regarding your rights and obligations.

SOFTWARE AND DOCUMENTATION ARE PROVIDED “AS IS?WITHOUT WARRANTY OF ANY KIND,
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

//---------
// Includes
//---------

#include "tcpip_private.h"
#include "system/system_debug.h"
#include <ctype.h>
#include "wf_console.h"

#if defined ( CMD_PARSER )

#include "wifi/wf_console_msg_handler.h"
#if defined(WF_CONSOLE_IFCFGUTIL)
#include "wf_console_if_config.h"
#include "wf_console_iw_config.h"
#include "wf_console_iw_priv.h"
#endif

typedef struct dataStructDescriptor
{
    uint16_t  dataFormat;
    void *  p_validateFunc;
    void *  p_dest;
} tDataStructDescriptor;


#define kWFValidateWithU8               (0)
#define kWFValidateWithU16              (1)
#define kWFValidateWithS8               (2)
#define kWFValidateWithX8               (3)

#if defined(WF_CONSOLE_IFCFGUTIL)
extern uint8_t g_hibernate_state;
#endif
//============================================================================
// Function Prototypes
//============================================================================

static void    do_help_msg(void);
#if defined(TCPIP_IF_MRF24W)
static void    do_get_wfver_cmd(void);
#endif
static void    do_cls_cmd(void);

/*****************************************************************************
 * FUNCTION: process_cmd
 *
 * RETURNS: None
 *
 * PARAMS:  None
 *
 * NOTES:   Determines which command has been received and processes it.
 *****************************************************************************/
void process_cmd(void)
{
    bool new_arg;
    uint8_t i;


    g_ConsoleContext.argc = 0;
    new_arg = true;

    // Get pointers to each token in the command string
    TokenizeCmdLine(g_ConsoleContext.rxBuf);

    // if command line nothing but white kWFSpace or a linefeed
    if ( g_ConsoleContext.argc == 0u )
    {
        return;   // nothing to do
    }

    // change the command itself (token[0]) to lower case
    for (i = 0; i < strlen((char *)g_ConsoleContext.argv[0]); ++i)
    {
        g_ConsoleContext.argv[0][i] = tolower(g_ConsoleContext.argv[0][i]);
    }


    if ( IS_ECHO_ON() )
    {
        SYS_CONSOLE_MESSAGE("\n\r");
    }

    switch (GetCmdId())
    {

        case HELP_MSG:
            do_help_msg();
			WFConsoleSetMsgFlag();
            break;
			
#if defined(TCPIP_IF_MRF24W)  
        case GET_WF_VERSION_MSG:
            do_get_wfver_cmd();
            break;
#endif

        case RESET_HOST:
            Reset();
            break;

        case CLEAR_SCREEN_MSG:
            do_cls_cmd();
            break;

#if defined(WF_CONSOLE_IFCFGUTIL)
        case IFCONFIG_MSG:
            do_ifconfig_cmd();
            break;

        case IWCONFIG_MSG:
            do_iwconfig_cmd();
            break;

        case IWPRIV_MSG:
            do_iwpriv_cmd();
            break;
#endif // WF_CONSOLE_IFCFGUTIL

        default:
			WFConsoleSetMsgFlag();
            break;
    }
}

bool convertAsciiToHexInPlace( int8_t *p_string, uint8_t expectedHexBinSize )
{

    int8_t  ascii_buffer[3];
    uint8_t  hex_binary_index = 0;
    int8_t  *hex_string_start = p_string;
    uint16_t hex_buffer = 0;

    /* gobble up any hex prefix */
    if ( memcmp (hex_string_start, (const const FAR char*) "0x", 2) == 0 )
         hex_string_start+=2;

   if ( strlen( (char *) hex_string_start) != (expectedHexBinSize*2) )
      return false;

    while ( hex_binary_index < expectedHexBinSize )
    {

      memcpy ( ascii_buffer, (const char*) hex_string_start, 2 );
      ascii_buffer[2] = '\0';

      /* convert the hex string to a machine hex value */
      if ( !ConvertASCIIHexToBinary( ascii_buffer,&hex_buffer) )
        return false;

      p_string[hex_binary_index++] = (uint8_t) hex_buffer;

      hex_string_start +=2;

    }

    return true;

}

static void do_cls_cmd(void)
{
    Output_Monitor_Hdr();
}


static void do_help_msg(void)
{
    uint8_t i;

    SYS_CONSOLE_MESSAGE("\n\r");
    for (i = 0; i < g_numCmds; ++i)
    {
        SYS_CONSOLE_MESSAGE( (const FAR char *) g_consoleCmd[i].p_cmdName);
        SYS_CONSOLE_MESSAGE("\r\t\t");
        SYS_CONSOLE_MESSAGE( (const FAR char*) g_consoleCmd[i].p_cmdHelp);
        SYS_CONSOLE_MESSAGE("\n\r");
    }

}

/*****************************************************************************
 * FUNCTION: do_get_wfver_cmd
 *
 * RETURNS: None
 *
 * PARAMS:  None
 *
 * NOTES:   Processes get WF device information
 *****************************************************************************/
#if defined(TCPIP_IF_MRF24W)

static void do_get_wfver_cmd(void)
{
 	tWFDeviceInfo  deviceInfo;

	#if defined(WF_CONSOLE_IFCFGUTIL)
	if (g_hibernate_state)
	{
		WFConsolePrintStr("The Wi-Fi module is in hibernate mode - command failed.", true);
		return;
	}
	#endif
	
	WF_GetDeviceInfo(&deviceInfo);
	WFConsolePrintStr("Firmware version   0x", false);
	WFConsolePrintHex(deviceInfo.romVersion, 2);
	WFConsolePrintHex(deviceInfo.patchVersion, 2);
	WFConsolePrintStr("", true);  

	WFConsolePrintStr("Host Driver version        ", false);
	WFConsolePrintStr(WF_HOST_DRIVER_VERSION_NUMBER, true);
}
#endif

#endif /* CMD_PARSER */









