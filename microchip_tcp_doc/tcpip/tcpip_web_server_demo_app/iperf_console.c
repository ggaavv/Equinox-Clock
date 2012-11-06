/*******************************************************************************
  Iperf Console

  Summary:
    Module for Microchip TCP/IP Stack
    
  Description:
*******************************************************************************/

/*******************************************************************************
FileName:   iperf_console.c
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

//============================================================================
// Includes
//============================================================================
#include <stdarg.h>
#include <ctype.h>
#include "iperf_console.h"
#include "system/system_debug.h"


#if defined ( APP_USE_IPERF )

//============================================================================
// Constants
//============================================================================

// This define determines the number of command lines that are stored in the history
// buffer.  Setting this define to 0 will remove the command line history feature by
// eliminating history code and buffers, thereby saving memory.
#define  kNumHistoryEntries   (1)

// states for IperfConsoleProcess() state machine
enum
{
    kSTWaitForChar,
    kSTWaitForEscSeqSecondChar,
    kSTWaitForEscSeqThirdChar,
    kSTWaitForEscSeqFourthChar,
};

// ASCII control keys


#define kTab             ('\t')
#define kNewLine         ('\n')
#define kBackspace       ('\b')
#define kCR              ('\r')
#define kSpace           (' ')
#define kEscape          (0x1b)
#define kDelete          (0x7f)   /* not supported on hyperterminal unless in VT100 mode */
#define kEnter           (0x0d)
#define kCtrl_B          (0x02)
#define kCtrl_C          (0x03)
#define kCtrl_D          (0x04)
#define kCtrl_E          (0x05)   /* used to turns off console echo */
#define kCtrl_H          (0x08)
#define kCtrl_I          (0x09)
#define kCtrl_O          (0x0f)
#define kCtrl_X          (0x18)

// total number of commands to and from Host Bridge
#define kNumCmdsToHostBridge    (sizeof(MsgsToHB) / sizeof(MSGS))
#define kNumCmdsFromHostBridge  (sizeof(MsgsFromHB) / sizeof(MSGS))

#define  kNextHistory          (0)
#define  kPrevHistory          (1)

#define kMaxInputEscapeSequence  (5)

//============================================================================
// Macros
//============================================================================
#define SET_RX_STATE(state)     g_ConsoleContext.rxState = state
#define GET_RX_STATE()          g_ConsoleContext.rxState
#define SET_CURSOR(index)       g_ConsoleContext.cursorIndex = index
#define GET_CURSOR()            g_ConsoleContext.cursorIndex

#define GET_LEN_RX_CMD_STRING()        ( strlen( (char *) g_ConsoleContext.rxBuf))

//============================================================================
// TypeDefs
//============================================================================

#if (kNumHistoryEntries > 0)
typedef struct history_struct
{
    char   buf[kNumHistoryEntries][kConsoleMaxMsgSize + 1];  // N lines of history
    uint8_t   index;
    bool seeded;
    int8_t   recallIndex;
} tIperfHistory;
#endif

//============================================================================
// Local Globals
//============================================================================
#if (kNumHistoryEntries > 0)
static tIperfHistory history;
#endif


static const char gCmdLinePrompt[] = "> ";
static  uint8_t gCmdLinePromptLength = 2;
static  int8_t gTmpCmdLine[kConsoleMaxMsgSize];


//============================================================================
// Constant Local Globals
//============================================================================

// VT100 escapse sequences that are output to the terminal
static const char cursorLeftEscapeSequence[]           = {kEscape, '[', '1', 'D', 0x00};
static const char cursorRightEscapeSequence[]          = {kEscape, '[', '1', 'C', 0x00};
static const char cursorHomeEscapeSequence[]           = {kEscape, '[', 'f', 0x00};
static const char eraseToEndOfLineEscapeSequence[]     = {kEscape, '[', 'K', 0x00};
static const char saveCursorPositionEscapeSequence[]   = {kEscape, '7', 0x00};
static const char restoreCursorPositionSequence[]      = {kEscape, '8', 0x00};
static const char eraseEntireLineEscapeSequence[]      = {kEscape, '[', '2', 'K', 0x00};
static const char eraseEntireScreenEscapeSequence[]    = {kEscape, '[', '2', 'J', 0x00};
static const char underlineModeEscapeSequence[]        = {kEscape, '[', '4', 'm', 0x00};
static const char normalModeEscapeSequence[]           = {kEscape, '[', 'm', 0x00};
static const char highlightModeEscapeSequence[]        = {kEscape, '[', '1', 'm', 0x00};
static const char inverseVideoEscapeSequence[]         = {kEscape, '[', '7', 'm', 0x00};

// VT100 escape sequences that are input from the terminal
// (note, if we ever use a longer sequence, update kMaxInputEscapeSequence)
static const char upArrowEscapeSequence[]     = {kEscape, 0x5b, 'A', 0x00};
static const char downArrowEscapeSequence[]   = {kEscape, 0x5b, 'B', 0x00};
static const char rightArrowEscapeSequence[]  = {kEscape, 0x5b, 'C', 0x00};
static const char leftArrowEscapeSequence[]   = {kEscape, 0x5b, 'D', 0x00};
static const char homeKeyEscapeSequence[]     = {kEscape, 0x5b, '1', 0x00};
static const char endKeyEscapeSequence[]      = {kEscape, 0x5b, '4', 0x00};

//============================================================================
// Console Message related constants, type, variables
//============================================================================

//---------------------
// token parsing states
//---------------------
enum
{
    kIPERFWaitingForStartOfToken,
    kIPERFWaitingForEndOfToken
};

//----------------
// Command strings
//----------------
const int8_t helpCmd[]      = "help";
const int8_t helpHelp[]     = "Lists all commands";

const int8_t resetCmd[]     = "reset";
const int8_t resetHelp[]    = "Reset host MCU";

const int8_t clsCmd[]       = "cls";
const int8_t clsHelp[]      = "Clears screen";

const int8_t seeDocHelp[]      = "see documentation";


//----------------------
// Console Command Table
//-----------------------
const tIPERFCmd g_consoleCmd[] = {

    {helpCmd,                      // cmd name
     helpHelp,                     // cmd description
     2},                           // max tokens

    {resetCmd,                     // [2]
     resetHelp,
     1},

    {clsCmd,                       // [3]
     clsHelp,
     1},
};

const uint8_t g_numCmds   = sizeof(g_consoleCmd) / sizeof(tIPERFCmd);


//============================================================================
// Globals
//============================================================================

tConsoleContext g_ConsoleContext;


//============================================================================
// Local Function Prototypes
//============================================================================
#if 0   /* add back if needed */
static bool          isCtrlCharacter(int8_t c);
static void    NormalMode(void);
static void    UnderlineMode(void);
#endif

static bool          isPrintableCharacter(int8_t c);
static bool          isCmdLineOnlyWhitespace(void);
static void    InsertCharacter(int8_t c);
static void    Delete(void);
static void    Backspace(void);
static void    EchoCharacter(int8_t new_char);

#if 0 /* add back if you need this feature */
static void    EraseEntireScreen(void);
#endif

static void    EraseEntireLine(void);
static void    CursorRight(void);
static void    CursorRight_N(uint8_t n);
static void    CursorLeft(void);
static void    CursorLeft_N(uint8_t n);
static void    CursorHome(void);
static void    CursorEnd(void);


static void    Enter(void);
static void    ProcessEscapeSequence(int8_t *p_escape_sequence);
static void    OutputCommandPrompt(void);

#if (kNumHistoryEntries > 0)
static void    InitHistory(void);
static void    AddCmdToHistory(void);
static void    DisplayHistoryEntry(uint8_t action);
#endif

static void    do_help_msg(void);
static void    do_cls_cmd(void);

/*****************************************************************************
 * FUNCTION: IperfConsoleInit
 *
 * RETURNS: None
 *
 * PARAMS:
 *
 * NOTES:   Initialization for console thread
 *
 *****************************************************************************/
void IperfConsoleInit(void)
{

    SET_RX_STATE(kSTWaitForChar);
    SET_CURSOR(0);

    // zero out command line buffer
    memset(g_ConsoleContext.rxBuf, 0x00, sizeof(g_ConsoleContext.rxBuf));

    g_ConsoleContext.bStateMachineLoop = true;
    g_ConsoleContext.firstChar         = false;
    g_ConsoleContext.appConsoleMsgRx   = false;
    SET_ECHO_ON();
#if (kNumHistoryEntries > 0)
    InitHistory();
#endif

    g_ConsoleContext.echoOn = true;

}


/*****************************************************************************
 * FUNCTION: IperfConsoleProcess
 *
 * RETURNS: None
 *
 * PARAMS:  None
 *
 * NOTES:   State machine called from main loop of app.  Handles serial input.
 *
 *****************************************************************************/
void IperfConsoleProcess(void)
{
    //uint8_t *pStart = &(cmdline[0]);
    int8_t  c;
    static int8_t escape_sequence[kMaxInputEscapeSequence];
    static int8_t esc_seq_index;

    // if this state machine has been disabled
    if (g_ConsoleContext.bStateMachineLoop == false)
    {
        return;
    }


    // if a command was entered that is application=specific
    if (g_ConsoleContext.appConsoleMsgRx == true)
    {
        return;  // wait until app done before processing further characters
    }

    // if no character(s) received
    if (SYS_CONSOLE_DATA_RDY() == 0)
    {
        return;
    }

    // get the character
    c = (int8_t) SYS_CONSOLE_GETC();

    // if this is the very first character received by this state machine
    if (g_ConsoleContext.firstChar == false)
    {
        Output_Monitor_Hdr();
        g_ConsoleContext.firstChar = true;
    }

    switch( GET_RX_STATE() )
    {
        //------------------------------------------
        case kSTWaitForChar:
        //------------------------------------------
            // if a 'normal' printable character
            if (isPrintableCharacter(c))
            {
                InsertCharacter(c);
            }
            // else if Delete key
            else if (c == kDelete)
            {
                Delete();
            }
            // else if Backspace key
            else if (c == (int8_t)kBackspace)
            {
                Backspace();
            }
            // else if Enter key
            else if (c == kEnter)
            {
                Enter();
            }
            // else if Escape key
            else if (c == kEscape)
            {
                /* zero out escape buffer, init with ESC */
                memset(escape_sequence, 0x00, sizeof(escape_sequence));
                escape_sequence[0] = kEscape;
                esc_seq_index = 1;
                SET_RX_STATE(kSTWaitForEscSeqSecondChar);
            }
            // else if Ctrl C
            else if (c == kCtrl_C)
            {
               OutputCommandPrompt();
            }
            else {
                // Enter();
            }
            break;

        //------------------------------------------
        case kSTWaitForEscSeqSecondChar:
        //------------------------------------------
            /* if an arrow key, home, or end key (which is all that this state machine handles) */
            if (c == 0x5b)
            {
               escape_sequence[1] = c;
               SET_RX_STATE(kSTWaitForEscSeqThirdChar);
            }
            // else if user pressed escape followed by any printable character
            else if (isPrintableCharacter(c))
            {
                InsertCharacter(c);
                SET_RX_STATE(kSTWaitForChar);
            }
            // start this command line over
            else // anything else
            {
                OutputCommandPrompt();
                SET_RX_STATE(kSTWaitForChar);
            }
            break;

        //------------------------------------------
        case kSTWaitForEscSeqThirdChar:
        //------------------------------------------
            escape_sequence[2] = c;
            
            if ((c == 0x31) || (c == 0x34))   // home or end key
            {
                SET_RX_STATE(kSTWaitForEscSeqFourthChar);
            }
            else
            {    
                ProcessEscapeSequence(escape_sequence);
                SET_RX_STATE(kSTWaitForChar);
            }    
            break;
            
        //------------------------------------------            
        case kSTWaitForEscSeqFourthChar:
        //------------------------------------------
            ProcessEscapeSequence(escape_sequence);
            SET_RX_STATE(kSTWaitForChar);
            break;
            


    } // end switch
}

/*****************************************************************************
 * FUNCTION: IperfConsoleProcessEpilogue
 *
 * RETURNS: None
 *
 * PARAMS:  None
 *
 * NOTES:   Check if there is a left console msg, and release it if found.
 *
 *****************************************************************************/
void IperfConsoleProcessEpilogue(void)
{
    if (IperfConsoleIsConsoleMsgReceived())
	{
		if ( memcmp(ARGV[0], "help", 4) != 0 )
		{
			SYS_CONSOLE_MESSAGE("Unknown cmd: ");
			SYS_CONSOLE_MESSAGE((char *)ARGV[0]);
			SYS_CONSOLE_MESSAGE("\r\n");
		}

	    IperfConsoleReleaseConsoleMsg();
	}
}

char ** IperfConsoleGetCmdLineArgv(void)
{
    return g_ConsoleContext.argv;
}

uint8_t IperfConsoleGetCmdLineArgc(void)
{
    return g_ConsoleContext.argc;
}    

/*****************************************************************************
 * FUNCTION: ProcessEscapeSequence
 *
 * RETURNS: None
 *
 * PARAMS:  pEscapeSequence -- escape sequence string to be processed.
 *
 * NOTES:   Processes an escape sequence received by the state machine
 *
 *****************************************************************************/
static void ProcessEscapeSequence(int8_t *pEscapeSequence)
{

   /* if a Left Arrow Key */
   if (strcmp( (const char *) pEscapeSequence, (const char*) leftArrowEscapeSequence) == 0)
   {
      CursorLeft();
   }
   /* else if Right Arrow Key */
   else if (strcmp( (const char *) pEscapeSequence, (const char*) rightArrowEscapeSequence) == 0)
   {
      CursorRight();
   }
#if (kNumHistoryEntries > 0)
   /* else if Up Arrow Key */
   else if (strcmp( (const char *) pEscapeSequence, (const char*) upArrowEscapeSequence) == 0)
   {

      DisplayHistoryEntry(kPrevHistory);
   }
   /* else if Down Arrow Key */
   else if (strcmp( (const char *) pEscapeSequence, (const char*) downArrowEscapeSequence) == 0)
   {
      DisplayHistoryEntry(kNextHistory);
   }
#endif
   /* else if Home Key */
   else if (strcmp( (const char *) pEscapeSequence, (const char*) homeKeyEscapeSequence) == 0)
   {
      CursorHome();
   }
   /* else if End Key */
   else if (strcmp( (const char *) pEscapeSequence, (const char*) endKeyEscapeSequence) == 0)
   {
      CursorEnd();
   }
}

/*= Output_Monitor_Hdr =======================================================
Purpose: After clearing screen, outputs to terminal the header block of text.

Inputs:  None

Returns: None
============================================================================*/
void Output_Monitor_Hdr(void)
{
    // KS:
    // EraseEntireScreen();

    char    eraseStr[80];   // output 79 times '=' plus \n\r

    memset(eraseStr, '=', 79);
    eraseStr[79] = '\0';
    
    SYS_CONSOLE_MESSAGE("\n\r");
    SYS_CONSOLE_MESSAGE(eraseStr);
    SYS_CONSOLE_MESSAGE("\n\r");
    SYS_CONSOLE_MESSAGE("* Iperf Command Parser\n\r");
    SYS_CONSOLE_MESSAGE("* (c) 2012, 2013, 2014 -- Microchip Technology, Inc.\n\r");
    SYS_CONSOLE_MESSAGE("*\n\r* Type 'help' to get a list of commands.\n\r");
    SYS_CONSOLE_MESSAGE(eraseStr);
    SYS_CONSOLE_MESSAGE("\n\r");
    OutputCommandPrompt();

}


/*= is_Printable_Character ===================================================
Purpose: Determines if the input character can be output to the screen

Inputs:  c  -- char to test

Returns: True if printable, else False
============================================================================*/
static bool isPrintableCharacter(int8_t c)
{
   if ( ((isalpha(c))   ||
        (isdigit(c))    ||
        (isspace(c))    ||
        (ispunct(c)))

            &&

        (c != (int8_t)kEnter) && (c != (int8_t)kTab)

      )
   {
      return true;
   }
   else
   {
      return false;
   }
}

/*= InsertCharacter =========================================================
Purpose: Inserts and echoes an printable character into the command line at the
         cursor location.

Inputs:  c  -- char to insert

Returns: none
============================================================================*/
static void InsertCharacter(int8_t c)
{
   uint8_t len;

   uint8_t i;
   uint8_t orig_cursor_index = GET_CURSOR();
   uint8_t count;

   /* throw away characters if exceeded cmd line length */
   if (GET_LEN_RX_CMD_STRING() >= sizeof(g_ConsoleContext.rxBuf)-1)
   {
      return;
   }

   len = GET_LEN_RX_CMD_STRING() + gCmdLinePromptLength;

   /* if inserting a character at end of cmd line */
   if (GET_CURSOR() == len)
   {
      g_ConsoleContext.rxBuf[GET_CURSOR() - gCmdLinePromptLength] = c;
      SET_CURSOR(GET_CURSOR() + 1);
      EchoCharacter(c);
   }
   /* inserting a character somewhere before the end of command line */
   else
   {
      /* Null out tmp cmd line */
      memset(gTmpCmdLine, 0x00, sizeof(gTmpCmdLine));

      /* copy up to the point of insertion */
      strncpy( (char *) gTmpCmdLine, (const char *) g_ConsoleContext.rxBuf, GET_CURSOR() - gCmdLinePromptLength);

      /* insert the new character */
      gTmpCmdLine[GET_CURSOR() - gCmdLinePromptLength] = c;

      /* copy the chars after the new character */
      strncpy( (char *) &gTmpCmdLine[GET_CURSOR() - gCmdLinePromptLength + 1],
               (const char *) &g_ConsoleContext.rxBuf[GET_CURSOR() - gCmdLinePromptLength],
               len - GET_CURSOR());

      /* put the first part of new string in the cmd line buffer */
      strcpy( (char *) g_ConsoleContext.rxBuf, (const char *) gTmpCmdLine);

      /* erase entire line, put the cursor at index 0 */
      EraseEntireLine();

      /* output the prompt */
      SYS_CONSOLE_MESSAGE(gCmdLinePrompt);

      /* Output the updated command line */
      SYS_CONSOLE_MESSAGE((char *)&g_ConsoleContext.rxBuf[0]);

      /* move the cursor to the next insert location */
      count = (len + 1) - orig_cursor_index - 1;
      for (i = 0; i < count; ++i)
      {
         SYS_CONSOLE_MESSAGE(cursorLeftEscapeSequence);
      }

      SET_CURSOR(orig_cursor_index + 1);
   }
}

/*= Delete ==================================================================
Purpose: Deletes the character at the cursor index

Inputs:  none

Returns: none
============================================================================*/

static void Delete(void)
{
   unsigned int num_chars;
   unsigned int orig_index = GET_CURSOR();

   /* if cursor is not at the end of the line */
   if (GET_CURSOR() != GET_LEN_RX_CMD_STRING() + gCmdLinePromptLength)
   {
      /* Null out tmp cmd line */
      memset(gTmpCmdLine, 0x00, sizeof(gTmpCmdLine));

      /* get characters before the deleted key */
      num_chars = GET_CURSOR() - gCmdLinePromptLength;
      strncpy( (char *) gTmpCmdLine, (const char *) g_ConsoleContext.rxBuf, num_chars);

      /* append characters after the deleted char (if there are any) */
      if (strlen( (char *) g_ConsoleContext.rxBuf) - 1 > 0u)
      {
         strcpy( (char *) &gTmpCmdLine[num_chars], (const char *) &g_ConsoleContext.rxBuf[num_chars + 1]);
      }

      EraseEntireLine();               /* leaves g_ConsoleContext.cursorIndex at 0 */
      SYS_CONSOLE_MESSAGE(gCmdLinePrompt);

      strcpy( (char *) g_ConsoleContext.rxBuf, (const char *) gTmpCmdLine);


      SYS_CONSOLE_MESSAGE((char *)g_ConsoleContext.rxBuf );
      SET_CURSOR(gCmdLinePromptLength + GET_LEN_RX_CMD_STRING());
      CursorHome(); /* to first character after prompt */


      /* move cursor to point of delete */
      CursorRight_N(orig_index - gCmdLinePromptLength);
   }
}

/*= EchoCharacter ===========================================================
Purpose: Echoes a character to the terminal.

Inputs:  new_char -- character to echo

Returns: none
============================================================================*/
static void EchoCharacter(int8_t c)
{
    if (IS_ECHO_ON())
    {
       /* output cr then lf for lf */
       if (c == (int8_t)'\n')
       {
          SYS_CONSOLE_PUTC('\r');
       }

       SYS_CONSOLE_PUTC(c);
    }
}

/*= Enter ====================================================================
Purpose: Enter key processing

Inputs:  None

Returns: none
============================================================================*/
static void Enter(void)
{
   bool cmd_flag = false;


   if ( IS_ECHO_ON() )
   {
       /* if the command line has any characters in it and it is not just white space */
       if ( (GET_LEN_RX_CMD_STRING() > 0u)  &&  (!isCmdLineOnlyWhitespace() ) )
       {
#if (kNumHistoryEntries > 0)
          AddCmdToHistory();
#endif
          cmd_flag = true;
       }
   }
   // else talking to PC app, presume it only sends valid commands
   else
   {
       cmd_flag = true;
   }

   // Process command
   if (cmd_flag == true)
   {
       process_cmd();
   }

   // if we got an app-specific command,
   if (g_ConsoleContext.appConsoleMsgRx == false)
   {
       /* linefeed and output prompt */
       OutputCommandPrompt();
   }

   // don't output command prompt, which also clears rx buf, until app processes it's command

}

#if (kNumHistoryEntries > 0)

/*****************************************************************************
 * FUNCTION: InitHistory
 *
 * RETURNS: None
 *
 * PARAMS:  None
 *
 * NOTES:   Initialize command line history states.
 *
 *****************************************************************************/
static void InitHistory(void)
{
    history.index       = 0;
    history.seeded      = false;
    history.recallIndex = 0;
}

/*****************************************************************************
 * FUNCTION: AddCmdToHistory
 *
 * RETURNS: None
 *
 * PARAMS:  None
 *
 * NOTES:   Adds latest command to history buffer
 *
 *****************************************************************************/
static void AddCmdToHistory(void)
{
    // zero out current command at this location
    memset((void *)&history.buf[history.index], 0x00, sizeof(history.buf[history.index]));

    // copy new command to buffer
    memcpy( (void *) &history.buf[history.index], (void *) g_ConsoleContext.rxBuf, strlen( (char *) g_ConsoleContext.rxBuf));

    // bump index to next line in history buffer
    history.index = (history.index + 1) % kNumHistoryEntries;

    // put the recall index one command in advance of the command we just added
    history.recallIndex = history.index;

    // at least one entry in history buffer
    history.seeded = true;
}

/*****************************************************************************
 * FUNCTION: DisplayHistoryEntry
 *
 * RETURNS: None
 *
 * PARAMS:  action -- PREV_HISTORY or NEXT_HISTORY
 *
 * NOTES:   In response to the user pressing up or down arrow key, display
 *          corresponding entry in history buffer.
 *
 *****************************************************************************/
static void DisplayHistoryEntry(uint8_t action)
{

   bool foundEntry = false;

   // if nothing in history buffer
   if (history.seeded == false)
   {
      return;
   }

   if (action == (uint8_t)kPrevHistory)
   {
      --history.recallIndex;
      if (history.recallIndex < 0)
      {
         history.recallIndex = kNumHistoryEntries - 1;
      }

      /* search until found a history entry or searched all entries */
      while (foundEntry == false)
      {
         /* if found a history entry */
         if (history.buf[history.recallIndex][0] != 0)
         {
            foundEntry = true;
         }
         else
         {
            --history.recallIndex;
            if (history.recallIndex < 0)
            {
               history.recallIndex = kNumHistoryEntries  - 1;
            }
         }
      }
   }
   else /* action == kNextHistory */
   {
      history.recallIndex = (history.recallIndex + 1) % kNumHistoryEntries;

      /* search until found a history entry or searched all entries */
      while (foundEntry == false)
      {
         /* if found a history entry */
         if (history.buf[history.recallIndex][0] != 0)
         {
            foundEntry = true;
         }
         else
         {
            history.recallIndex = (history.recallIndex + 1) % kNumHistoryEntries;
         }
      }
   }

   if (foundEntry)
   {
      // erase line on screen and output command from history
      EraseEntireLine();          /* leaves Cursor_Index at 0 */
      SYS_CONSOLE_MESSAGE(gCmdLinePrompt );
      SYS_CONSOLE_MESSAGE(history.buf[history.recallIndex]);

      // copy history command to console buffer (so they match) and put cursor
      // at end of line
      memset(g_ConsoleContext.rxBuf, 0x00, GET_LEN_RX_CMD_STRING() );
      strcpy( (char *) g_ConsoleContext.rxBuf, (const char *) history.buf[history.recallIndex]);
      SET_CURSOR(gCmdLinePromptLength + strlen( (char *) history.buf[history.recallIndex]));
   }

}
#endif  /* (kNumHistoryEntries > 0) */


/*= Backspace ================================================================
Purpose: Performs a backspace operation on the command line

Inputs:  none

Returns: none
============================================================================*/
static void Backspace(void)
{
   uint8_t num_chars;
   uint8_t orig_index = GET_CURSOR();

   /* if cursor is not at the left-most position */
   if (GET_CURSOR() != gCmdLinePromptLength)
   {
      /* Null out tmp cmd line */
      memset(gTmpCmdLine, 0x00, sizeof(gTmpCmdLine));

      /* get characters before the backspace */
      num_chars = GET_CURSOR() - gCmdLinePromptLength - 1;
      strncpy( (char *) gTmpCmdLine, (const char *) g_ConsoleContext.rxBuf, num_chars);

      /* append characters after the deleted char (if there are any) */
      if ( (GET_LEN_RX_CMD_STRING() - 1) > 0u)
      {
         strcpy( (char *) &gTmpCmdLine[num_chars], (const char *) &g_ConsoleContext.rxBuf[num_chars + 1]);
      }

      EraseEntireLine();  /* leaves g_ConsoleContext.cursorIndex at 0 */

      strcpy( (char *) g_ConsoleContext.rxBuf, (const char *) gTmpCmdLine);

      SYS_CONSOLE_MESSAGE(gCmdLinePrompt);
      SYS_CONSOLE_MESSAGE((char *)g_ConsoleContext.rxBuf);
      SET_CURSOR(gCmdLinePromptLength + GET_LEN_RX_CMD_STRING());

      CursorHome(); /* to first character after prompt */


      /* move cursor to point of backspace */
      CursorRight_N(orig_index - 1 - gCmdLinePromptLength);
   }
}




static void EraseEntireLine()
{
   // int i;
   SYS_CONSOLE_MESSAGE(eraseEntireLineEscapeSequence);
   CursorLeft_N(GET_CURSOR());
   SET_CURSOR(0);
}

#if 0  /* add back if you want this feature */
static void EraseEntireScreen()
{
   SYS_CONSOLE_MESSAGE(eraseEntireScreenEscapeSequence);
}
#endif

static void OutputCommandPrompt(void)
{
    if ( IS_ECHO_ON() )
    {
     SYS_CONSOLE_MESSAGE("\n\r");
     SYS_CONSOLE_MESSAGE(gCmdLinePrompt);
    }
    SET_CURSOR(gCmdLinePromptLength);
    memset(g_ConsoleContext.rxBuf, 0x00, sizeof(g_ConsoleContext.rxBuf));

}

/*= CursorRight =============================================================
Purpose: Moves the cursor right by one character

Inputs:  none

Returns: none
============================================================================*/
void CursorRight(void)
{
   /* if cursor is not already at the right-most position */
   if (GET_CURSOR() < GET_LEN_RX_CMD_STRING() + gCmdLinePromptLength)
   {
      SET_CURSOR( GET_CURSOR() + 1);
      SYS_CONSOLE_MESSAGE(cursorRightEscapeSequence);
   }
}


/*= CursorRight_N ==============================================================
Purpose: Moves the cursor left N characters to the right

Inputs:  n -- number of characters to move the cursor to the left

         Note: This sequence only takes a single digit of length, so may need to
               do the move in steps


Returns: none
============================================================================*/
void CursorRight_N(uint8_t n)
{
   char sequence_string[sizeof(cursorRightEscapeSequence) + 2];  /* null and extra digit */

//   ASSERT(n <= (strlen(g_ConsoleContext.buf) + CMD_LINE_PROMPT_LENGTH));

   if (n > 0u)
   {
      SET_CURSOR( GET_CURSOR() + n );
      sequence_string[0] = cursorRightEscapeSequence[0]; /* ESC */
      sequence_string[1] = cursorRightEscapeSequence[1];  /* '[' */

      if (n < 10u)
      {
         sequence_string[2] = n + '0';  /* ascii digit */
         sequence_string[3] = cursorRightEscapeSequence[3];    /* 'C' */
         sequence_string[4] = '\0';
      }
      else
      {
         sequence_string[2] = (n / 10) + '0';  /* first ascii digit  */
         sequence_string[3] = (n % 10) + '0';  /* second ascii digit */
         sequence_string[4] = cursorRightEscapeSequence[3];    /* 'C' */
         sequence_string[5] = '\0';

      }

      SYS_CONSOLE_MESSAGE(sequence_string);
   }
}

/*= CursorLeft ==============================================================
Purpose: Moves the cursor left by one character

Inputs:  none

Returns: none
============================================================================*/
void CursorLeft(void)
{
   /* if cursor is not already at the left-most position */
   if (GET_CURSOR() > strlen( (const FAR char *) gCmdLinePrompt))
   {
      SET_CURSOR( GET_CURSOR() - 1);
      SYS_CONSOLE_MESSAGE(cursorLeftEscapeSequence);
   }
}


/*= CursorLeft_N ==============================================================
Purpose: Moves the cursor left N characters to the left

Inputs:  n -- number of characters to move the cursor to the left

         Note: This sequence only takes a single digit of length, so may need to
               do the move in steps


Returns: none
============================================================================*/
void CursorLeft_N(uint8_t n)
{
   char sequence_string[sizeof(cursorLeftEscapeSequence) + 2];  /* null and extra digit */

//   ASSERT(n <= g_ConsoleContext.cursorIndex + CMD_LINE_PROMPT_LENGTH);

   if (n > 0u)
   {
      SET_CURSOR( GET_CURSOR() - n );

      sequence_string[0] = cursorLeftEscapeSequence[0];  /* ESC */
      sequence_string[1] = cursorLeftEscapeSequence[1];  /* '[' */

      if (n < 10u)
      {
         sequence_string[2] = n + '0';  /* ascii digit */
         sequence_string[3] = cursorLeftEscapeSequence[3];    /* 'D' */
         sequence_string[4] = '\0';
      }
      else
      {
         sequence_string[2] = (n / 10) + '0';  /* first ascii digit  */
         sequence_string[3] = (n % 10) + '0';  /* second ascii digit */
         sequence_string[4] = cursorLeftEscapeSequence[3];    /* 'D' */
         sequence_string[5] = '\0';

      }

      SYS_CONSOLE_MESSAGE(sequence_string);
   }
}


/*= CursorHome ==============================================================
Purpose: Moves the cursor to the left-most position of the command line (just
         in front of the prompt).

Inputs:  none

Returns: none
============================================================================*/
static void CursorHome(void)
{
   /* if cursor not at start of command line */
   if (GET_CURSOR() != gCmdLinePromptLength)
   {
      /* move it to start of command line */
      CursorLeft_N(GET_CURSOR() - gCmdLinePromptLength);
   }
}

static void CursorEnd(void)
{
   uint8_t len;

   if ( (GET_LEN_RX_CMD_STRING() + gCmdLinePromptLength) != GET_CURSOR())
   {
      len = GET_LEN_RX_CMD_STRING() - GET_CURSOR() + gCmdLinePromptLength;
      CursorRight_N(len);
   }
}


static bool isCmdLineOnlyWhitespace(void)
{
   uint8_t i;
   uint8_t len = GET_LEN_RX_CMD_STRING();

   for (i = 0; i < len; ++i)
   {
      if ( !isspace(g_ConsoleContext.rxBuf[i]) )
      {
         return false;
      }
   }

   return true;
}

#if 0   /* Add back if you need this func */

static void UnderlineMode(void)
{
    SYS_CONSOLE_MESSAGE(inverseVideoEscapeSequence);
}

static void NormalMode(void)
{
    SYS_CONSOLE_MESSAGE(normalModeEscapeSequence);
}


/*****************************************************************************
 * FUNCTION: isCtrlCharacter
 *
 * RETURNS: true if input is a ctrl character, else false
 *          REG_OK_16_BIT_REG -- valid 16-bit register
 *          REG_UNKNOWN       -- unknown register ID
 *          REG_VAL_TOO_LARGE -- reg value to large for an 8-bit register
 *
 * PARAMS:  None
 *
 * NOTES:   Called by do_writereg_cmd and do_readreg_cmd to verify if accessing
 *          a legal register.  In the case of write, function verifies that
 *          write value doesn't overflow an 8-bit register.
 *
 *****************************************************************************/
static bool isCtrlCharacter(int8_t c)
{
    if (isprint(c))
    {
        return false;
    }
    else
    {
        return true;
    }
}
#endif

void IperfConsoleSetMsgFlag(void)
{
    g_ConsoleContext.appConsoleMsgRx = true;
}

void IperfConsoleReleaseConsoleMsg(void)
{
    /* linefeed and output prompt */
    OutputCommandPrompt();

    g_ConsoleContext.appConsoleMsgRx = false;
}

bool IperfConsoleIsConsoleMsgReceived(void)
{

    return g_ConsoleContext.appConsoleMsgRx;
}


//============================================================================
// Console Message functions
//============================================================================

/*****************************************************************************
 * FUNCTION: TokenizeCmdLine
 *
 * RETURNS: None
 *
 * PARAMS:  p_line -- pointer to the null terminated command line
 *
 * NOTES: Converts the input string into tokens separated by '\0'.
  *****************************************************************************/
void TokenizeCmdLine(char *p_line)
{
    uint8_t state = kIPERFWaitingForStartOfToken;
    uint8_t index = 0;

    ARGC = 0;

    //---------------------------
    // while not at end of string
    //---------------------------
    while (p_line[index] != (int8_t)'\0')
    {

        //----------------------------------------
        if (state == (uint8_t)kIPERFWaitingForStartOfToken)
        //----------------------------------------
        {
            // if hit non whitespace
            if (!isspace((int)p_line[index]))
            {
               // argument string starts here
               ARGV[ARGC++] = (char *)(&(p_line[index]));
               if (ARGC >= (uint8_t)kIPERFMaxTokensPerCmd)
               {
                   return;  // truncate because too many tokens
               }
               state = kIPERFWaitingForEndOfToken;
            }
            ++index;

        }
        //----------------------------------------
        else if (state == (uint8_t)kIPERFWaitingForEndOfToken)
        //----------------------------------------
        {
            // if white space, then end of token
            if (isspace((int)p_line[index]))
            {
                // string terminate the token
                p_line[index] = '\0';
                state = kIPERFWaitingForStartOfToken;
            }
            ++index;
        }
    }
}


/*****************************************************************************
 * FUNCTION: GetCmdId
 *
 * RETURNS: None
 *
 * PARAMS:  void
 *
 * NOTES: Determines index of cmd in CMD struct
  *****************************************************************************/
uint8_t GetCmdId(void)
{
    uint8_t i;
    const tIPERFCmd  *p_msgList;
    uint16_t  msgCount;

    p_msgList = g_consoleCmd;
    msgCount  = g_numCmds;

    for (i = 0; i < msgCount; ++i)
    {
        if ( strcmp( (FAR char *)ARGV[0], (FAR const char *) p_msgList[i].p_cmdName) == 0)
        {
            return i;
        }
    }

    return INVALID_CMD;
}



/*****************************************************************************
 * FUNCTION: ConvertASCIIHexToBinary
 *
 * RETURNS: true if conversion successful, else false
 *
 * PARAMS:  p_ascii   -- ascii string to be converted
 *          p_binary  -- binary value if conversion successful
 *
 * NOTES:   Converts an input ascii hex string to binary value (up to 32-bit value)
 *****************************************************************************/
bool ConvertASCIIHexToBinary(int8_t *p_ascii, uint16_t *p_binary)
{
    int8_t  i;
    uint32_t multiplier = 1;

    *p_binary = 0;

    // not allowed to have a string of more than 4 nibbles
    if (strlen((char*)p_ascii) > 8u)
    {
        return false;
    }

    // first, ensure all characters are a hex digit
    for (i = (uint8_t)strlen((char *)p_ascii) - 1; i >= 0 ; --i)
    {
        if (!isxdigit(p_ascii[i]))
        {
            return false;
        }
        *p_binary += multiplier * HexToBin(p_ascii[i]);
        multiplier *= 16;
    }

    return true;
}

/*****************************************************************************
 * FUNCTION: ConvertASCIIUnsignedDecimalToBinary
 *
 * RETURNS: true if conversion successful, else false
 *
 * PARAMS:  p_ascii   -- ascii string to be converted
 *          p_binary  -- binary value if conversion successful
 *
 * NOTES:   Converts an input ascii decimal string to binary value
 *****************************************************************************/
bool ConvertASCIIUnsignedDecimalToBinary(int8_t *p_ascii, uint16_t *p_binary)
{
    int8_t  i;
    uint32_t multiplier = 1;
    int8_t len;

    *p_binary = 0;
    len = (int8_t)strlen((char *)p_ascii);

    // should not be any numbers greater than 6 digits
    if ((len > 5) || (len == 0))
    {
        return false;
    }

    // first, ensure all characters are a decimal digit
    for (i = len - 1; i >= 0 ; --i)
    {
        if (!isdigit(p_ascii[i]))
        {
            return false;
        }
        *p_binary += multiplier * (p_ascii[i] - '0');
        multiplier *= 10;
    }

    return true;
}

/*****************************************************************************
 * FUNCTION: ConvertASCIISignedDecimalToBinary
 *
 * RETURNS: true if conversion successful, else false
 *
 * PARAMS:  p_ascii   -- ascii string to be converted
 *          p_binary  -- binary value if conversion successful
 *
 * NOTES:   Converts an input ascii signed decimal string to binary value
 *****************************************************************************/
bool ConvertASCIISignedDecimalToBinary(int8_t *p_ascii, int16_t *p_binary)
{
    int8_t   i;
    uint32_t  multiplier = 1;
    bool negFlag = false;
    int8_t   endIndex = 0;
    int8_t  len;

    *p_binary = 0;
    len = (int8_t)strlen((char *)p_ascii);

    // should not be any numbers greater than 5 digits (with -)
    if (len > 6)
    {
        return false;
    }

    if (p_ascii[0] == (int8_t)'-')
    {
        negFlag = true;
        endIndex = 1;
    }


    // first, ensure all characters are a decimal digit

    for (i = len - 1; i >= endIndex ; --i)
    {
        if (!isdigit(p_ascii[i]))
        {
            return false;
        }
        *p_binary += multiplier * (p_ascii[i] - '0');
        multiplier *= 10;
    }

    if (negFlag == true)
    {
        *p_binary *= -1;
    }

    return true;
}

/*****************************************************************************
 * FUNCTION: HexToBin
 *
 * RETURNS: binary value associated with ASCII hex input value
 *
 * PARAMS:  hexChar -- ASCII hex character
 *
 * NOTES:   Converts an input ascii hex character to its binary value.  Function
 *          does not error check; it assumes only hex characters are passed in.
 *****************************************************************************/
uint8_t HexToBin(uint8_t hexChar)
{
    if ((hexChar >= 'a') && (hexChar <= 'f'))
    {
        return (0x0a + (hexChar - 'a'));
    }
    else if ((hexChar >= 'A') && (hexChar <= 'F'))
    {
        return (0x0a + (hexChar - 'A'));
    }
    else //  ((hexChar >= '0') && (hexChar <= '9'))
    {
        return (0x00 + (hexChar - '0'));
    }

}

bool ExtractandValidateU16Range(int8_t *p_string, uint16_t *pValue, uint16_t minValue, uint16_t maxValue)
{
    /* extract next parameter as an unsigned short integer */
    if (!ConvertASCIIUnsignedDecimalToBinary(p_string, pValue))
    {
        /* IperfConsolePrintf("   Unable to parse paramter value"); */
        return false;
    }

    if ((*pValue < minValue) || (*pValue > maxValue))
    {
        /* IperfConsolePrintf("   parameter value out of range"); */
        return false;
    }

    return true;
}

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

    // if command line nothing but white kSpace or a linefeed
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
			IperfConsoleSetMsgFlag();
            break;
			
        case RESET_HOST:
            Reset();
            break;

        case CLEAR_SCREEN_MSG:
            do_cls_cmd();
            break;

        default:
			IperfConsoleSetMsgFlag();
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


#endif /* CMD_PARSER */
