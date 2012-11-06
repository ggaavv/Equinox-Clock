/*******************************************************************************
  Iperf Console

  Summary:
    Module for Microchip TCP/IP Stack
    
  Description:
*******************************************************************************/

/*******************************************************************************
FileName:  iperf_console.h 
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

#ifndef _IPEFF_CONSOLE_H_
#define _IPEFF_CONSOLE_H_

#include "tcpip/tcpip.h"

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
    RESET_HOST,
    CLEAR_SCREEN_MSG,

    INVALID_CMD = 0xFF
};


#define kIPERFMaxTokensPerCmd       (16)  /* max tokens, including cmd and parameters */
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
    uint8_t   maxTokens;             // max tokens for this command
} tIPERFCmd;

typedef struct
{
	char		rxBuf[kConsoleMaxMsgSize];  // buf for received characters
    char		txBuf[kConsoleMaxMsgSize];  // buf for transmitted characters
	uint8_t		rxState;                    // current state of Rx state machine
	uint8_t		cursorIndex;                // cursor index
    bool		firstChar;                  // false if waiting for very first character from user or PC
    bool		echoOn;                     // true if human input at console, false if PC communicating

	int8_t		p_cmdStrings[kConsoleCmdMaxNum][kConsoleCmdMaxLen];  // cmd string array
    uint8_t		numCmdStrings;              // number of cmd strings in p_cmdStrings

    uint8_t		appConsoleMsgRx;            // true if app received a console msg, else false

    char*		argv[kIPERFMaxTokensPerCmd];   // pointer to each token in the rxBuf

    uint8_t		argc;                       // number of tokens in rxBuf
	uint8_t		subState;
	bool		bStateMachineLoop;
	uint8_t		req;
} tConsoleContext;

//----------------------------------------------------------------------------
//                                 External Globals
//----------------------------------------------------------------------------
extern  const tIPERFCmd    g_consoleCmd[];
extern  const uint8_t     g_numCmds;

extern tConsoleContext g_ConsoleContext;
#define ARGC           g_ConsoleContext.argc
#define ARGV           g_ConsoleContext.argv

#define SET_ECHO_ON()       g_ConsoleContext.echoOn = true
#define SET_ECHO_OFF()      g_ConsoleContext.echoOn = false
#define IS_ECHO_ON()        g_ConsoleContext.echoOn

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
void   IperfConsoleSetMsgFlag(void);

bool convertAsciiToHexInPlace(int8_t *p_string, uint8_t expectedHexBinSize);
void process_cmd(void);

extern void IperfConsoleInit(void);
extern void IperfConsoleProcess(void);
extern void IperfConsoleProcessEpilogue(void);
extern void IperfConsoleReqClear(void);
extern bool IperfConsoleIsConsoleMsgReceived(void);
extern void IperfConsoleReleaseConsoleMsg(void);
extern char ** IperfConsoleGetCmdLineArgv(void);
extern uint8_t IperfConsoleGetCmdLineArgc(void);
extern void IperfConsoleSetMsgFlag(void);
extern bool IperfConsoleIsIperfAppKillRequested(void);


		
#ifdef __cplusplus
		 }
#endif

#endif /* _IPERF_CONSOLE_H_ */
