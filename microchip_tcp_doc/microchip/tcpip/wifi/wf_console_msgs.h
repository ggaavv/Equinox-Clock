/*******************************************************************************
  MRF24W Driver Console Messages

  Summary:
    Module for Microchip TCP/IP Stack
    
  Description:
    -Provides access to MRF24W WiFi controller
    -Reference: MRF24W Data sheet, IEEE 802.11 Standard
*******************************************************************************/

/*******************************************************************************
FileName:  wf_console_msgs.h 
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

#ifndef __WFCONSOLEMSGS_H
#define __WFCONSOLEMSGS_H

#ifdef __cplusplus
 extern "C" {
#endif

//----------------------------------------------------------------------------
//                                 Defines
//----------------------------------------------------------------------------

// !!! These defines MUST match the g_consoleCmd structure  !!!
enum validConsoleCmds
{
    HELP_MSG = 0,                   // only used by humans
    GET_WF_VERSION_MSG,
    RESET_HOST,
    CLEAR_SCREEN_MSG,
    IFCONFIG_MSG,
    IWCONFIG_MSG,
    IWPRIV_MSG,

    INVALID_CMD = 0xFF
};


#define kWFMaxTokensPerCmd          (16)  /* max tokens, including cmd and parameters */
#define kConsoleMaxMsgSize	        (80)
#define kConsoleCmdMaxLen           (16)  /* max string length of console commands (w/o arguments) */
#define kConsoleCmdMaxNum           (8)   /* max number of registered console commands */


//----------------------------------------------------------------------------
//                                 Typedefs
//----------------------------------------------------------------------------

// structure definition to define all ASCII messages
typedef struct msg_struct
{
    const int8_t   *p_cmdName;       // pointer to cmd name
    const int8_t   *p_cmdHelp;       // pointer to cmd help string
    uint8_t   maxTokens;            // max tokens for this command
} tWFCmd;

// parsed version numbers put in this structure
typedef struct version_struct
{
    uint8_t major;
    uint8_t minor;
    uint8_t revision;
} tWFVersion;


//----------------------------------------------------------------------------
//                                 External Globals
//----------------------------------------------------------------------------
extern  const tWFCmd    g_consoleCmd[];
extern  const uint8_t     g_numCmds;


//----------------------------------------------------------------------------
//                                 Function Prototypes
//----------------------------------------------------------------------------
void   TokenizeCmdLine(char *p_line);
uint8_t           GetCmdId(void);
void   Output_Monitor_Hdr(void);
bool         ConvertASCIIHexToBinary(int8_t *p_ascii, uint16_t *p_binary);
bool         ConvertASCIIUnsignedDecimalToBinary(int8_t *p_ascii, uint16_t *p_binary);
bool         ConvertASCIISignedDecimalToBinary(int8_t *p_ascii, int16_t *p_binary);
uint8_t           HexToBin(uint8_t hexChar);
void   WFConsoleSetMsgFlag(void);



#ifdef __cplusplus
 }
#endif


#endif /* __WFCONSOLEMSGS_H */
