/*******************************************************************************
  Iperf Application

  Summary:
    Runs iperf client and server demos
    
  Description:
    - Implements iperf application
*******************************************************************************/

/*******************************************************************************
FileName:  iperf_app.c 
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

#include <string.h> /* for memcpy */
#include "tcpip_config.h"
#include "tcpip/tcpip.h"
#include "tcpip/tcp.h"
#include "system/system_debug.h"

#if defined(APP_USE_IPERF)
#include "iperf_console.h" 
#include "iperf_config.h"

#if !defined(SYS_CONSOLE_ENABLE)
#error "Systeme console must be enabled for IPERF"
#endif

//****************************************************************************
// CONSTANTS (Defines and enums)
//****************************************************************************
/*--------------------*/
/* Endianness defines */
/*--------------------*/
#define IPERF_BIG_ENDIAN                   (0)
#define IPERF_LITTLE_ENDIAN                (1)
/* Indicate whether the Host CPU is big-endian or little-endian */
#define HOST_CPU_ENDIANNESS          IPERF_LITTLE_ENDIAN    /* BIG_ENDIAN or LITTLE_ENDIAN */
/*-------------------*/
/* Endianness Macros */
/*-------------------*/
/* if the Host CPU is Little Endian, which does not match the MRF24W */
#if (HOST_CPU_ENDIANNESS == IPERF_LITTLE_ENDIAN)    
    /* 32-bit data type conversion */
    #define HTOIPERFL(a) (((a & 0x000000ff) << 24) | ((a & 0x0000ff00) << 8) | ((a & 0x00ff0000) >> 8) | ((a & 0xff000000) >> 24))
    #define IPERFTOHL(a) HTOIPERFL(a)
    /* 16-bit data type conversion */
    #define HSTOIPERFS(a) (((a) << 8) | ((a) >> 8))
    #define IPERFSTOHS(a) HSTOIPERFS(a)    
/* else Host CPU is Big-Endian, which matches the MRF24W */
#else
    #define HTOIPERFL(a)   (a)
    #define IPERFTOHL(a)   (a)    
    #define HSTOIPERFS(a)  (a)
    #define IPERFSTOHS(a)  (a)
#endif

#define UDP_FIN_RETRANSMIT_COUNT		10u     // iperf retransmits 10 times the last UDP packet,
#define UDP_FIN_RETRANSMIT_PERIOD       10      // at 10ms apart.
#define TIMING_ERROR_MARGIN              2      // Account for msec tick uncertainty.

#define IPERF_GET_MSEC_TICK_COUNT() SYS_TICK_Get()

typedef enum {
    UDP_PROTOCOL = 1,
    TCP_PROTOCOL
} tIperfProto;

enum {
    IPERF_STANDBY_STATE=1,

    IPERF_RX_START_STATE,
    IPERF_UDP_RX_STATE,
    IPERF_UDP_RX_DRAIN_STATE,
    IPERF_UDP_RX_DONE_STATE,
    IPERF_TCP_RX_LISTEN_STATE,
    IPERF_TCP_RX_STATE,
    IPERF_TCP_RX_DONE_STATE,
    IPERF_RX_DONE_STATE,

    IPERF_TX_START_STATE,
    IPERF_TX_ARP_RESOLVE_STATE,

    IPERF_TCP_TX_OPEN_STATE,
    IPERF_TCP_TX_CONNECT_STATE,
    IPERF_TCP_TX_SEGMENT_STATE,
    IPERF_TCP_TX_DONE_STATE,

    IPERF_UDP_TX_OPEN_STATE,
    IPERF_UDP_TX_DATAGRAM_STATE,
    IPERF_UDP_TX_DONE_STATE
};

typedef enum
{
    INTERVAL_REPORT,
    SUBTOTAL_REPORT,
    SESSION_REPORT
} tIperfReport;


//****************************************************************************
// LOCAL DATA TYPES                                                             
//****************************************************************************

/* tAppState */
typedef struct
{
    uint32_t		mInterval;		// -i
    uint32_t		mAmount;		// -n
    uint32_t		mDuration;		// -t. Default = 10*TICK_SECOND msec
    uint32_t		mDatagramSize;	// -l
    uint16_t		mMSS;			// -M
    bool		mServerMode;    // -s or -c
    bool   		mBufLenSet;     // -l
    tIperfProto	mProtocol;		// -b or -u
    uint16_t		mServerPort;	// -p

    uint32_t		mTxRate;		// -b or
                                // -x: NONE-STANDARD IPERF OPTION. Max Tx bps rate for TCP.

    double			totalLen; // mTotalLen
    long 			pktId; 		// datagramID
    long			lastPktId; // lastDatagramID
    uint32_t		errorCount;
    uint32_t		outofOrder;

    TCP_SOCKET tcpServerSock;
    TCP_SOCKET tcpClientSock;

    UDP_SOCKET udpSock;

    TCP_SOCKET_INFO  remoteSide;
    uint16_t		 localPort;

    //struct sockaddr_in remoteAddr;
   // int 		remoteAddrlen;
    //	tSocketAddr remoteAddr;

    // Calculated packet period, in msec, to reflect the target bit rate.
    uint32_t		mPktPeriod;

    uint32_t		startTime;
    uint32_t		stopTime;
    uint32_t		nextTxTime;
    //uint32_t		remoteStartTime;
    //uint32_t		remoteStopTime;

    uint8_t			nAttempts;
    uint32_t		pktCount;
    bool			stopRequested;


    uint32_t		lastCheckPktCount;  // Check if pktCount changes within mInterval; or kIperfRxTimeOut.
    long			lastCheckPktId;
    uint32_t		lastCheckErrorCount;
    uint32_t		lastCheckTotalLen;
    uint32_t		lastCheckTime;
    uint8_t 		statusReported;

//	long		mPendingACK;		// number of outstanding TCP ACKs
//	uint8_t		mRetransmit;

    uint8_t 		state;

    uint32_t      timer;
    uint16_t      remainingTxData;
    uint16_t      availUDPTxSpace;

    bool     	isLastTransmit;


} tAppState;



//
// Data structure used by iperf protocol
//

#define HEADER_VERSION1 0x80000000

typedef struct
{
    long id;
    uint32_t tv_sec;
    uint32_t tv_usec;
} tIperfPktInfo; 	// In the original Iperf, this is the "UDP_datagram" structure.

// tUDP_datagram
typedef struct
{
    uint32_t flags;
    uint32_t total_len1;
    uint32_t total_len2;
    uint32_t stop_sec;
    uint32_t stop_usec;
    uint32_t error_cnt;
    uint32_t outorder_cnt;
    uint32_t datagrams;
    uint32_t jitter1;
    uint32_t jitter2;
} tServerHdr;

typedef struct
{
    uint32_t flags;
    uint32_t numThreads;
    uint32_t mPort;
    uint32_t bufferlen;
    uint32_t mWinBand;
    uint32_t mAmount;
} tClientHdr;

extern NODE_INFO remoteNode;
//****************************************************************************
// LOCAL GLOBALS                                                             
//****************************************************************************
static NET_CONFIG *pNetIf;

#define MAX_BUFFER   (sizeof(tIperfPktInfo) + sizeof(tServerHdr))
uint8_t  g_bfr[ MAX_BUFFER ];


static tAppState gAppState;

#define APPCTX (gAppState)

//
// Helper utilities
//

/* On the Microchip v4.51 stack the MSS is a compile time setting and not within the control */
/* of the application to change on a per socket basis, nor is it even a runtime configurable */
/* setting.  Search tcp.c for TCP_MAX_SEG_SIZE.                                              */
/* However, TCP_MAX_SEG_SIZE is not a public macro.                                          */
/* RFC 879 specifies the default MSS to be 536. Hence we assume this number here.            */
/* (IPERF_TCP_MSS == TCP_MAX_SEG_SIZE) is desirable but not required.                     */

#define IPERF_TCP_MSS  536

static void
ResetIperfCounters(void)
{
    // APPCTX.mAmount = 0;
    // APPCTX.mDuration = 10*1000; // -t: default 10 sec
    // APPCTX.mInterval = 1000; 	// -i: default 1 sec
    APPCTX.mMSS = IPERF_TCP_MSS;
    APPCTX.mDatagramSize = 1470; // -l: default 1470 bytes. UDP datagram size.
    APPCTX.totalLen = 0;
    APPCTX.pktId = 0;
    APPCTX.lastPktId = 0;
    APPCTX.errorCount = 0;
    APPCTX.outofOrder = 0;
    APPCTX.pktCount = 0;
    APPCTX.statusReported = 0;
    APPCTX.startTime = 0;
    APPCTX.stopTime = 0;

    APPCTX.lastCheckPktCount = 0;
    APPCTX.lastCheckPktId = 0;
    APPCTX.lastCheckErrorCount = 0;
    APPCTX.lastCheckTotalLen = 0;
    APPCTX.lastCheckTime = 0;

    APPCTX.isLastTransmit = false;

//	APPCTX.mPendingACK = 0;
//	APPCTX.mRetransmit = 0;

}

static void ascii_to_u32s(char *ptr, uint32_t *values, uint8_t count)
{
    uint8_t i;
    uint32_t tmp;

    // Convert "123.456_78_90k", with count set to 4,  to
    // unsigned 32-bit numbers 123, 456, 78 and 90000, and
    // store them in the values array.

    for (i = 0; i < count; i++)
    {
        tmp = 0;

        while ( (*ptr > (int8_t)'9') || (*ptr < (int8_t)'0') )
        {
            if ( (*ptr == (int8_t)' ') || (*ptr == (int8_t)0) ) return; // terminates at blank or NULL.

            ptr++;
        }

        while ( (*ptr <= (int8_t)'9') && (*ptr >= (int8_t)'0') )
        {
            tmp = tmp*10 + *ptr - '0';
            ptr++;
        }
        if ( (*ptr == (int8_t)'k') || (*ptr == (int8_t)'K') )
        {
            tmp = tmp * 1000;
            ptr++;
        }
        else if ( (*ptr == (int8_t)'m') || (*ptr == (int8_t)'M') )
        {
            tmp = tmp * 1000 * 1000;
            ptr++;
        }

        values[i] = tmp;
    }
}

//
// Report bandwith, jitter, and packet loss stastistics.
// Used by in both server and client modes.
//
// Todo: implement the jitter report.
//

static void ReportBW_Jitter_Loss(tIperfReport reportType)
{
    uint32_t nAttempted;
    uint32_t nDropped;
    double kbps;
    uint32_t currentTime;
    uint32_t sec;
	uint32_t msec;

    currentTime = IPERF_GET_MSEC_TICK_COUNT();

    switch ( reportType )
    {
        case INTERVAL_REPORT:

            nDropped = APPCTX.errorCount - APPCTX.lastCheckErrorCount;

            // bits-per-msec == Kbps

            sec = (currentTime- APPCTX.lastCheckTime)/SYS_TICK_TicksPerSecondGet();
			msec = ((double) (currentTime - APPCTX.lastCheckTime)) / (((double)(SYS_TICK_TicksPerSecondGet()))/1000);

            if ( APPCTX.state == (uint8_t)IPERF_UDP_TX_DONE_STATE )
            {
               nAttempted = (APPCTX.lastPktId - APPCTX.lastCheckPktId) + nDropped;
            }
            else
            {
                nAttempted = APPCTX.pktId - APPCTX.lastCheckPktId;
            }

			if ( msec == 0u )
            {
                kbps = 0;
            }
            else
            {
				kbps = ((APPCTX.totalLen - APPCTX.lastCheckTotalLen)*((double) 8)) / msec;
            }

            sec = (APPCTX.lastCheckTime - APPCTX.startTime)/SYS_TICK_TicksPerSecondGet();

            sprintf( (char *) g_ConsoleContext.txBuf,"    - [%2lu- %2lu sec] %3lu/ %3lu (%2lu%%)    %4lu Kbps\r\n",
                      (unsigned long)sec, 
                      (unsigned long)sec + ( (unsigned long) (APPCTX.mInterval/SYS_TICK_TicksPerSecondGet()) ),
                      (unsigned long)nDropped,
                      (unsigned long)nAttempted,
                      (unsigned long)nDropped*100/(unsigned long)nAttempted,
                      (unsigned long) (kbps + ((double) 0.5)));

            SYS_CONSOLE_MESSAGE( (char *) g_ConsoleContext.txBuf);

            break;


        case SUBTOTAL_REPORT:
            // intentional fall-through
        case SESSION_REPORT:

           nDropped = APPCTX.errorCount;

           if (APPCTX.state == (uint8_t)IPERF_UDP_TX_DONE_STATE)
            {
               nAttempted = APPCTX.lastPktId + nDropped;
            }
            else
            {
                nAttempted = APPCTX.lastPktId;
            }

			msec = ((double) (APPCTX.stopTime - APPCTX.startTime)) / (((double)(SYS_TICK_TicksPerSecondGet()))/1000);

			if ( msec == 0u )
            {
                kbps = 0;
            }
            else
            {
   				kbps = (APPCTX.totalLen * ((double) 8)) / msec;
            }

            sprintf( (char *) g_ConsoleContext.txBuf, "    - [0.0- %lu.%lu sec] %3lu/ %3lu (%2lu%%)    %4lu Kbps\r\n",
                             (unsigned long)(msec/1000),
                             (unsigned long)((msec%1000)/100),
                             (unsigned long)nDropped,
                             (unsigned long)nAttempted,
                             (nAttempted == 0u) ? 0 : ((unsigned long)nDropped*100/(unsigned long)nAttempted),
                             (unsigned long) (kbps + ((double) 0.5)));

            SYS_CONSOLE_MESSAGE( (char *) g_ConsoleContext.txBuf);

            break;
    }

    if ( reportType == 	SESSION_REPORT )
    {
      SYS_CONSOLE_MESSAGE("    Session completed ...");
    }

    APPCTX.lastCheckPktId = APPCTX.pktId;
    APPCTX.lastCheckErrorCount = APPCTX.errorCount;
    APPCTX.lastCheckPktCount = APPCTX.pktCount;
    APPCTX.lastCheckTime = currentTime;
    APPCTX.lastCheckTotalLen = APPCTX.totalLen;
}

bool IperfAppInit(const char* interface)
{
    pNetIf = (NET_CONFIG *)TCPIP_STACK_NetHandle(interface);

    if (pNetIf) {
        memset( &APPCTX, 0, sizeof(tAppState) );

        APPCTX.state = IPERF_STANDBY_STATE;
        APPCTX.stopRequested = false;

        APPCTX.tcpClientSock = INVALID_SOCKET;
        APPCTX.tcpServerSock = INVALID_SOCKET;
        APPCTX.udpSock = INVALID_SOCKET;
    } else {
        return false;
    }

    return true;
}

static bool IperfAppKillRequested(void)
{
    uint8_t argc;
    char **argv;

    // Did user ask to "kill iperf"?


    if ( IperfConsoleIsConsoleMsgReceived() == false ) return false;

    argv = IperfConsoleGetCmdLineArgv();
    argc = IperfConsoleGetCmdLineArgc();

    if (argc == 2u)
    {
        if ((memcmp(argv[0], "kill",  4) == 0) &&
           (memcmp(argv[1],  "iperf", 5) == 0))
        {
           //CliServicePerformed();
           IperfConsoleReleaseConsoleMsg();

           APPCTX.stopRequested = true;
           return true;
        }
    }
    SYS_CONSOLE_MESSAGE("Invalid commmand sequence\r\n");
    IperfConsoleReleaseConsoleMsg();
    return false;

}


static void StateMachineStandby(void)
{
    //tCliArguments *args;
    uint8_t argc;
    char **argv;
    uint8_t i;
    char *ptr;
    uint32_t values[4];
    uint16_t payloadSize;
    float pktRate;
    char buf[8];
    
    //if ( CliServiceRequested() == false ) return;
    //if ( !(args = CliGetArguments()) ) return;


    if ( IperfConsoleIsConsoleMsgReceived() == false ) return;
    
    argv = IperfConsoleGetCmdLineArgv();
    argc = IperfConsoleGetCmdLineArgc();   // needed because ConsoleGetCmdLineTokens() returns 0 for argc when in -Os 


    if ( (argc == 2u)                             &&
         (memcmp(argv[0], "kill", 4) == 0) &&
         (memcmp(argv[1], "iperf", 5) == 0) )
    {
       SYS_CONSOLE_MESSAGE("Iperf not started for the kill...\r\n");
       IperfConsoleReleaseConsoleMsg();
       return;
    }

    if ((memcmp(argv[0], "kill", 4) == 0) && (argc == 1u))
    {
        SYS_CONSOLE_MESSAGE("'kill iperf' is the only valid kill command\r\n");
        IperfConsoleReleaseConsoleMsg();
    }



    if ( (argc == 2u)                             &&
         (memcmp(argv[0], "kill", 4) == 0) &&
         (memcmp(argv[1], "iperf", 5) != 0) )
    {
       SYS_CONSOLE_MESSAGE("Invalid command sequence...\r\n");
       IperfConsoleReleaseConsoleMsg();
       return;
    }


    if ( memcmp(argv[0], "iperf", 5) ) return;

    // OK, we will handle the "iperf" command.

    APPCTX.mServerMode = false;
    APPCTX.mProtocol = TCP_PROTOCOL;   			// default is TCP mode.
    APPCTX.stopRequested = false;

    APPCTX.mServerPort = IPERF_APP_SERVER_PORT;		// -p. default: server port 5001



    APPCTX.mTxRate = ((uint32_t) 500)*((uint32_t) 1000);		// -b or -x. Target tx rate.
    // KS: default tx rate for iperf is actually 1Mbps. Here we set it to 500Kbps instead.

    APPCTX.mAmount = 0;			// -n: default 0.
    APPCTX.mDuration = ((uint32_t) 10)*((uint32_t) SYS_TICK_TicksPerSecondGet()); // -t: default 10 sec.
    APPCTX.mInterval =  SYS_TICK_TicksPerSecondGet(); 	// -i: default 1 sec.

    // Initialize statistics

    ResetIperfCounters();

    SYS_ASSERT(argc != 0, " ");
    for (i = 1; i < argc; i++)
    {
        if ((memcmp(argv[i], "-s", 2) == 0) ||
            (memcmp(argv[i], "--server", 5) == 0) )
        {
            // Function as an iperf server.

            APPCTX.mServerMode = true;
        }
        else if ((memcmp(argv[i], "-u", 2) == 0) ||
                 (memcmp(argv[i], "--udp", 5) == 0) )
        {
            // iperf UDP mode.
            APPCTX.mProtocol = UDP_PROTOCOL;
        }
        else if ((memcmp(argv[i], "-b", 2) == 0) ||
                 (memcmp(argv[i], "--bandwidth", 5) == 0) )
        {
            // iperf UDP mode.

            APPCTX.mProtocol = UDP_PROTOCOL;

            // Next argument should be the target rate, in bps.

            i++;
            ptr = argv[i];

            ascii_to_u32s(ptr, values, 1);

            APPCTX.mTxRate = values[0];
        }
        else if ((memcmp(argv[i], "-x", 2) == 0) ||
                 (memcmp(argv[i], "--xmitrate", 5) == 0) )
        {
            // NON-STANDARD IPERF OPTION. Set the max TCP tx rate.
            // Next argument should be the target rate, in bps.

            i++;
            ptr = argv[i];

            ascii_to_u32s(ptr, values, 1);

            APPCTX.mTxRate = values[0];
        }
        else if ((memcmp(argv[i], "-c", 2) == 0) ||
                 (memcmp(argv[i], "--client", 5) == 0) )
        {
            // Function as an iperf client.

            APPCTX.mServerMode = false;

            // Next argument should be the server IP, such as "192.168.1.100".

            i++;
            ptr = argv[i];

            ascii_to_u32s(ptr, values, 4);

            APPCTX.remoteSide.remoteIPaddress.v4Add.v[0] = values[0];
            APPCTX.remoteSide.remoteIPaddress.v4Add.v[1] = values[1];
            APPCTX.remoteSide.remoteIPaddress.v4Add.v[2] = values[2];
            APPCTX.remoteSide.remoteIPaddress.v4Add.v[3] = values[3];  

        }
        else if ((memcmp(argv[i], "-t", 2) == 0) ||
                 (memcmp(argv[i], "--time", 5) == 0) )
        {
            // Next argument should be the (client tx) duration, in seconds.

            i++;
            ptr = argv[i];

            ascii_to_u32s(ptr, values, 1);

            APPCTX.mDuration = values[0]*SYS_TICK_TicksPerSecondGet();
            APPCTX.mAmount = 0;
        }
        else if ((memcmp(argv[i], "-n", 2) == 0) ||
                 (memcmp(argv[i], "--num", 5) == 0) )
        {
            // Next argument should be the (client tx) size, in bytes.

            i++;
            ptr = argv[i];

            ascii_to_u32s(ptr, values, 1);

            APPCTX.mAmount = values[0];
            APPCTX.mDuration = 0;
        }

    /* On the Microchip v4.51 stack the MSS is a compile time setting and not within the control */
    /* of the application to change on a per socket basis, nor is it even a runtime configurable */
    /* setting.  Search tcp.c for TCP_MAX_SEG_SIZE  */

    //	else if ((memcmp(argv[i], "-M", 2) == 0) ||
    //			 (memcmp(argv[i], "--mss", 5) == 0) )
    //	{
    //		// Next argument should be the (client tcp tx) MSS size, in bytes.
    //
    //		i++;
    //		ptr = argv[i];
    //
    //		ascii_to_u32s(ptr, values, 1);
    //
    //		APPCTX.mMSS = values[0];
    //	}

        else if ((memcmp(argv[i], "-i", 2) == 0) ||
                 (memcmp(argv[i], "--interval", 5) == 0) )
        {
            // Next argument should be the report interval, in seconds.

            i++;
            ptr = argv[i];

            ascii_to_u32s(ptr, values, 1);

            APPCTX.mInterval = values[0]*SYS_TICK_TicksPerSecondGet(); // Convert to msec
        }
        else if ((memcmp(argv[i], "-l", 2) == 0) ||
                 (memcmp(argv[i], "--len", 5) == 0) )
        {
            // Next argument should be the buffer length, in bytes.
            // This is used as the UDP datagram size.

            i++;
            ptr = argv[i];

            ascii_to_u32s(ptr, values, 1);

            if ( values[0] <  MAX_BUFFER  )
            {
               sprintf( (char *) g_ConsoleContext.txBuf,"The minimum datagram size is %d\r\n", (int)MAX_BUFFER);
               SYS_CONSOLE_MESSAGE( (char *) g_ConsoleContext.txBuf);

               IperfConsoleReleaseConsoleMsg();
               return;
            }

            APPCTX.mDatagramSize = values[0];
        }
    }

    //CliServicePerformed();
    IperfConsoleReleaseConsoleMsg();

    switch (APPCTX.mServerMode)
    {
        case 0:
            // iperf client

            payloadSize = 	(APPCTX.mProtocol == UDP_PROTOCOL) ?
                            APPCTX.mDatagramSize : APPCTX.mMSS;

            pktRate =  (float) (APPCTX.mTxRate / 8) / (float) payloadSize;
            APPCTX.mPktPeriod =  (uint32_t) ( (float) SYS_TICK_TicksPerSecondGet() / pktRate );

            APPCTX.state = IPERF_TX_START_STATE;
            break;

        case 1:
            // iperf server

            SYS_CONSOLE_MESSAGE("---------------------------------------------------------\r\n");

            SYS_CONSOLE_MESSAGE("Server listening on ");
            if (APPCTX.mProtocol == UDP_PROTOCOL)
            {
                SYS_CONSOLE_MESSAGE((const char *)"UDP");
            }    
            else
            {
                SYS_CONSOLE_MESSAGE((const char *)"TCP");
            }    
            SYS_CONSOLE_MESSAGE(" port ");
            sprintf(buf, "%d\r\n", APPCTX.mServerPort);
            SYS_CONSOLE_MESSAGE(buf);
            APPCTX.state = IPERF_RX_START_STATE;
            break;
    }
}

static void StateMachineRxStart(void)
{
  if ( !APPCTX.mServerMode )
  {
     SYS_CONSOLE_MESSAGE("Unsupported Configuration\r\n");
     APPCTX.state = IPERF_STANDBY_STATE;
     return;
  }


 switch ( APPCTX.mProtocol )
 {
      case TCP_PROTOCOL:	// TCP


            /* TCP Server sockets are allocated for entire runtime duration, a call to disconnect does not free them */
            /* therefore a subsequent N+1 open will fail */
            if ( (APPCTX.tcpServerSock == INVALID_SOCKET) &&
                 (APPCTX.tcpServerSock = TCPOpenServer(IP_ADDRESS_TYPE_IPV4, APPCTX.mServerPort, 0)) == INVALID_SOCKET )
            {
               /* error case */
               SYS_CONSOLE_MESSAGE("Create TCP socket failed\r\n");
               APPCTX.state = IPERF_STANDBY_STATE;
               return;
            }

            TCPAdjustFIFOSize(APPCTX.tcpServerSock, 1, 0,TCP_ADJUST_PRESERVE_RX | TCP_ADJUST_GIVE_REST_TO_RX);

            APPCTX.state = IPERF_TCP_RX_LISTEN_STATE;
            break;

      case UDP_PROTOCOL:	// UDP

           if ( (APPCTX.udpSock = UDPOpenServer(IP_ADDRESS_TYPE_IPV4, APPCTX.mServerPort, 0)) == INVALID_UDP_SOCKET )
           {
               /* error case */
               SYS_CONSOLE_MESSAGE("Create UDP socket failed\r\n");
               APPCTX.state = IPERF_STANDBY_STATE;
               return;
           }

           UDPSocketSetNet(APPCTX.udpSock, pNetIf); // Bind test socket to dedicated interface for UDP Ack diagram
           APPCTX.state = IPERF_UDP_RX_STATE;
        break;

      default:
           SYS_CONSOLE_MESSAGE("Protocol error\r\n");
           APPCTX.state = IPERF_STANDBY_STATE;
           return;
  }

}


static void StateMachineTcpListen(void)
{

   if ( IperfAppKillRequested() == true )
   {
        APPCTX.state = IPERF_RX_DONE_STATE;
        return;
   }

   if( TCPIsConnected(APPCTX.tcpServerSock) )
   {
      TCP_SOCKET_INFO tcpSocketInfo;
	  TCPGetSocketInfo( APPCTX.tcpServerSock, &tcpSocketInfo);
      memcpy ( (void *) &APPCTX.remoteSide, &tcpSocketInfo, sizeof ( TCP_SOCKET_INFO) );
      APPCTX.state = IPERF_TCP_RX_STATE;

      /* clear the stack's reset flag */
      TCPWasReset(APPCTX.tcpServerSock);
   }
}


static void StateMachineTcpRx(void)
{
    uint16_t length;

    if( (length = TCPIsGetReady(APPCTX.tcpServerSock)) == 0 )
    {

      if ( TCPWasReset(APPCTX.tcpServerSock)  )
      {
          APPCTX.stopTime = IPERF_GET_MSEC_TICK_COUNT();
          APPCTX.state = IPERF_TCP_RX_DONE_STATE;
          return;
      }

    }
    else
    {
       if ( APPCTX.pktId == 0)
       {
          // This is the first rx pkt.
          SYS_CONSOLE_MESSAGE("\r\n    Session started ...\r\n");

          APPCTX.startTime = IPERF_GET_MSEC_TICK_COUNT();
          APPCTX.lastCheckTime = 	APPCTX.startTime;

          APPCTX.lastCheckPktId = APPCTX.pktId;

          sprintf((char *) g_ConsoleContext.txBuf, "    - Local  %u.%u.%u.%u port %u connected with\r\n",
                   pNetIf->MyIPAddr.v[0],
                   pNetIf->MyIPAddr.v[1],
                   pNetIf->MyIPAddr.v[2],
                   pNetIf->MyIPAddr.v[3],
                   APPCTX.mServerPort);

          SYS_CONSOLE_MESSAGE( (char *) g_ConsoleContext.txBuf);

          sprintf( (char *) g_ConsoleContext.txBuf,"    - Remote %u.%u.%u.%u port %u\r\n",
                   APPCTX.remoteSide.remoteIPaddress.v4Add.v[0],
                   APPCTX.remoteSide.remoteIPaddress.v4Add.v[1],
                   APPCTX.remoteSide.remoteIPaddress.v4Add.v[2],
                   APPCTX.remoteSide.remoteIPaddress.v4Add.v[3],
                   APPCTX.remoteSide.remotePort );

          SYS_CONSOLE_MESSAGE( (char *) g_ConsoleContext.txBuf);

       }

       APPCTX.pktId++;
       APPCTX.pktCount++;
       APPCTX.lastPktId = APPCTX.pktId;
       APPCTX.totalLen += length;

       /* read the remaining datagram payload */
       /* a UdpDiscard would be disingenuous, because it would not reflect the bandwidth at L7 */
       while ( length > 0 )
       {
          uint16_t chunk;

          if ( length <  (uint16_t)MAX_BUFFER )
            chunk = length;
          else
            chunk = MAX_BUFFER;

          TCPGetArray( APPCTX.tcpServerSock, (BYTE*)g_bfr, chunk);
          length -= chunk;
       }

    }

    if ((APPCTX.pktId != (long)0) &&
       (IPERF_GET_MSEC_TICK_COUNT() > (APPCTX.lastCheckTime + APPCTX.mInterval)) )
    {
         // Time to report statistics
         ReportBW_Jitter_Loss(INTERVAL_REPORT);
    }

    if ( IperfAppKillRequested() == true )
    {
       APPCTX.state = IPERF_TCP_RX_DONE_STATE;
       APPCTX.stopTime = IPERF_GET_MSEC_TICK_COUNT();

       return;
    }
}


static void StateMachineUdpRx(void)
{
    uint16_t length =0;
    tIperfPktInfo *pPktInfo;
	UDP_SOCKET_INFO UdpSkt;

    // Do nothing if no data is waiting
    if( (length = UDPIsGetReady(APPCTX.udpSock)) >= (uint16_t)(sizeof(tIperfPktInfo)) )
    {

       /* The GetArray should not fail... */
       if ( UDPGetArray(APPCTX.udpSock, (BYTE*)g_bfr, sizeof(tIperfPktInfo)) != sizeof(tIperfPktInfo) )
       {
          SYS_CONSOLE_MESSAGE("      UDP Get Array Failed\r\n");
          APPCTX.state = IPERF_UDP_RX_DRAIN_STATE;
          return;
       }

       pPktInfo = (tIperfPktInfo *) g_bfr;
       APPCTX.pktId = IPERFTOHL(pPktInfo->id);

       if ( (APPCTX.pktCount == (uint32_t)0) && (APPCTX.pktId < (long)0) )
       {
          // Ignore retransmits from previous session.
          UDPDiscard(APPCTX.udpSock);
          return;
       }

       APPCTX.pktCount++;
       if (APPCTX.pktCount == (uint32_t)1 )
       {
          // The first pkt is used to set up the server,
          // does not count as a data pkt.

          SYS_CONSOLE_MESSAGE("\r\n    Session started ...");

          if ( APPCTX.pktId != 0 )
          {
             // We have lost a few packets before the first pkt arrived.
             sprintf((char *) g_ConsoleContext.txBuf, "    - First pkt id = %ld (should be 0)\r\n",
                              APPCTX.pktId);

             SYS_CONSOLE_MESSAGE((char *) g_ConsoleContext.txBuf );


             // The first data pkt starts with id = 1.
             APPCTX.errorCount	+= 	APPCTX.pktId - 1;
          }

          APPCTX.lastPktId = APPCTX.pktId;

          sprintf((char *) g_ConsoleContext.txBuf, "    - Local  %u.%u.%u.%u port %u connected with\r\n",
                           pNetIf->MyIPAddr.v[0],
                           pNetIf->MyIPAddr.v[1],
                           pNetIf->MyIPAddr.v[2],
                           pNetIf->MyIPAddr.v[3],
                           APPCTX.mServerPort);

          SYS_CONSOLE_MESSAGE( (char *) g_ConsoleContext.txBuf);
		  
		  UDPGetSocketInfo(APPCTX.udpSock, &UdpSkt);		  
          sprintf((char *) g_ConsoleContext.txBuf,"    - Remote %u.%u.%u.%u port %u\r\n",      
                           remoteNode.IPAddr.v[0],
                           remoteNode.IPAddr.v[1],
                           remoteNode.IPAddr.v[2],
                           remoteNode.IPAddr.v[3],
                           UdpSkt.remotePort);

          SYS_CONSOLE_MESSAGE( (char *) g_ConsoleContext.txBuf);

          // Store the remote info so we can send the iperf "UDP-FIN-ACK" msg
          APPCTX.remoteSide.remoteIPaddress.v4Add.v[0] = remoteNode.IPAddr.v[0];
          APPCTX.remoteSide.remoteIPaddress.v4Add.v[1] = remoteNode.IPAddr.v[1];
          APPCTX.remoteSide.remoteIPaddress.v4Add.v[2] = remoteNode.IPAddr.v[2];
          APPCTX.remoteSide.remoteIPaddress.v4Add.v[3] = remoteNode.IPAddr.v[3];

          APPCTX.remoteSide.remotePort =  UdpSkt.remotePort;

          APPCTX.startTime = IPERF_GET_MSEC_TICK_COUNT();
          //APPCTX.remoteStartTime = IPERFTOHL(pPktInfo->tv_sec);

          APPCTX.lastCheckTime = 	APPCTX.startTime;

          APPCTX.lastCheckPktId = APPCTX.pktId;
          APPCTX.lastCheckPktCount = APPCTX.pktCount;
          APPCTX.lastCheckErrorCount = APPCTX.errorCount;

          UDPDiscard(APPCTX.udpSock);

          return;

      }

      APPCTX.totalLen += length;

      if ( APPCTX.pktId < 0 )
      {
         // this is the last datagram
         APPCTX.pktId = - APPCTX.pktId;

         APPCTX.stopTime = IPERF_GET_MSEC_TICK_COUNT();
        //APPCTX.remoteStopTime = IPERFTOHL(pPktInfo->tv_sec);

        APPCTX.nAttempts = 0;
        APPCTX.state = IPERF_UDP_RX_DRAIN_STATE;
      }

      if ( APPCTX.pktId != APPCTX.lastPktId+1 )
      {
         if ( APPCTX.pktId < APPCTX.lastPktId+1 )
         {
            APPCTX.outofOrder++;
         }
         else
         {
            APPCTX.errorCount += APPCTX.pktId - (APPCTX.lastPktId+1);
         }
      }

      // never decrease pktId (e.g. if we get an out-of-order packet)
      if ( APPCTX.pktId == APPCTX.lastPktId )
      {
         SYS_CONSOLE_MESSAGE("      Recv duplicated pkt\r\n");
      }

      if ( APPCTX.pktId > APPCTX.lastPktId )
      {
         APPCTX.lastPktId = APPCTX.pktId;
      }

      /* read the remaining datagram payload - the full payload */
      /* a UdpDiscard would be disingenuous, because it would not reflect the bandwidth at L7 */
      length -=  sizeof(tIperfPktInfo);
      while ( length > 0 )
      {
         uint16_t chunk;

         if ( length <  (uint16_t)MAX_BUFFER )
            chunk = length;
         else
            chunk = MAX_BUFFER;

         UDPGetArray(APPCTX.udpSock, (BYTE*)g_bfr, chunk);
         length -= chunk;
      }


    }  /* end got a datagram */

    if ( (APPCTX.pktCount != (uint32_t)0) &&
         (IPERF_GET_MSEC_TICK_COUNT() > (APPCTX.lastCheckTime + APPCTX.mInterval)) )
    {
        if ( APPCTX.pktCount == APPCTX.lastCheckPktCount )
        {
          // No events in APPCTX.mInterval msec, we timed out
          SYS_CONSOLE_MESSAGE("    Rx timed out\r\n");

          APPCTX.stopTime = IPERF_GET_MSEC_TICK_COUNT();

          APPCTX.nAttempts = 0;
          APPCTX.state = IPERF_UDP_RX_DRAIN_STATE;
        }
        else
        {
          ReportBW_Jitter_Loss(INTERVAL_REPORT);
        }
    }

    if ( IperfAppKillRequested() == true )
    {
        APPCTX.state = IPERF_UDP_RX_DRAIN_STATE;
        return;
    }

}

static void StateMachineUdpRxDrain(void)
{
    if( UDPIsGetReady(APPCTX.udpSock) > (uint8_t)0 )
    {
         UDPDiscard(APPCTX.udpSock);
         return;
    }

   /* If iperf kill was done, just jump to closing the socket */
   if ( APPCTX.stopRequested )
     APPCTX.state = IPERF_RX_DONE_STATE;
   else /* go ahead an generate reports, etc */
     APPCTX.state = IPERF_UDP_RX_DONE_STATE;

}


static void StateMachineUdpRxDone(void)
{
    tIperfPktInfo *pPktInfo;
    tServerHdr *pServer_hdr;
    uint16_t  tmp;
    float tmp2;


    if ( APPCTX.statusReported == 0u )
    {
        ReportBW_Jitter_Loss(SESSION_REPORT);
        APPCTX.statusReported = 1;
    }

    /* Drain any waiting pkts */
    if (  UDPIsGetReady(APPCTX.udpSock)  )
    {
        UDPDiscard(APPCTX.udpSock);
        return;
    }

    // Send the iperf UDP "FIN-ACK" 10 times.
    if ( APPCTX.nAttempts++ > 10u )
    {
        APPCTX.state = IPERF_RX_DONE_STATE;
        return;
    }

     /* Make sure space is available to TX the ACK packet of 128 bytes */
    if ( UDPIsPutReady(APPCTX.udpSock ) > 0u )
    {

      pPktInfo = (tIperfPktInfo *) g_bfr;
      pServer_hdr = (tServerHdr *) (pPktInfo +1);

      pPktInfo->id = HTOIPERFL( -APPCTX.lastPktId );
      pPktInfo->tv_sec = 0;
      pPktInfo->tv_usec = 0;

      pServer_hdr->flags = HTOIPERFL(HEADER_VERSION1);
      pServer_hdr->total_len1 = 0;
      pServer_hdr->total_len2 = HTOIPERFL( (uint32_t) APPCTX.totalLen);

      pServer_hdr->stop_sec =  HTOIPERFL( (uint32_t) (APPCTX.stopTime - APPCTX.startTime)/SYS_TICK_TicksPerSecondGet());

      /* get the remainder of the ticks using modulus */
      tmp2 = ((APPCTX.stopTime - APPCTX.startTime)%SYS_TICK_TicksPerSecondGet());

      /* normalize  to uSecs */
      tmp2 =  tmp2*1000/SYS_TICK_TicksPerSecondGet(); /* Convert to mSec */
      tmp2 *= 1000;   /* 1000 uSecs per mSec */


      pServer_hdr->stop_usec = HTOIPERFL( (uint32_t) tmp2 );
      pServer_hdr->error_cnt = HTOIPERFL( (uint32_t)  APPCTX.errorCount);;
      pServer_hdr->outorder_cnt = HTOIPERFL( (uint32_t) APPCTX.outofOrder);
      pServer_hdr->datagrams = HTOIPERFL( (uint32_t) APPCTX.lastPktId);
      pServer_hdr->jitter1 = 0;
      pServer_hdr->jitter2 = 0;

      UDPPutArray(APPCTX.udpSock, (BYTE*)g_bfr, MAX_BUFFER);

      for ( tmp=0; tmp < (128 - MAX_BUFFER); tmp++)
        UDPPut(APPCTX.udpSock, 0x00);

      UDPFlush(APPCTX.udpSock );

    }

}

static void StateMachineTcpRxDone(void)
{
    if ( APPCTX.statusReported == 0u )
    {
        ReportBW_Jitter_Loss(SESSION_REPORT);
        APPCTX.statusReported = 1;
    }

    APPCTX.state = IPERF_RX_DONE_STATE;
}

static void StateMachineRxDone(void)
{

   if ( APPCTX.mProtocol == UDP_PROTOCOL )
   {
     UDPClose(  APPCTX.udpSock );
   }
   else
   {

     /* 2 disconnects on purpose not mistake */
     TCPClose( APPCTX.tcpServerSock );
     //TCPDisconnect( APPCTX.tcpServerSock );
	 APPCTX.tcpServerSock = INVALID_SOCKET;
   }

    SYS_CONSOLE_MESSAGE("\r\n    Rx done. Socket closed.\r\n");

    // Clear statistics
    ResetIperfCounters();

    // In server mode, continue to accept new session requests ...

    if ((APPCTX.mServerMode == true) 	&&
        (APPCTX.stopRequested == false) )
    {
        SYS_CONSOLE_MESSAGE("    Ready for the next session.\r\n");

        APPCTX.state = IPERF_RX_START_STATE;
    }
    else
    {
        SYS_CONSOLE_MESSAGE("    Iperf completed.\r\n");
        APPCTX.state = IPERF_STANDBY_STATE;
    }

}




/******************************/
/* TX CLIENT CODE BEGINS HERE */
/******************************/


static void StateMachineTxStart(void)
{

   ARPResolve(pNetIf, &APPCTX.remoteSide.remoteIPaddress.v4Add);
   APPCTX.state = IPERF_TX_ARP_RESOLVE_STATE;
   APPCTX.timer = SYS_TICK_Get();

}

static void StateMachineTxArpResolve(void)
{

  if ( IperfAppKillRequested() == true )
  {
     SYS_CONSOLE_MESSAGE("Iperf client session closed.\r\n");
     APPCTX.state = IPERF_STANDBY_STATE;
     return;
  }

  if(!ARPIsResolved(pNetIf, &APPCTX.remoteSide.remoteIPaddress.v4Add, &APPCTX.remoteSide.remoteMACAddr))
  {
      /* every 3 seconds print a warning */
      if( SYS_TICK_Get() - APPCTX.timer > 5*SYS_TICK_TicksPerSecondGet() )
      {
         SYS_CONSOLE_MESSAGE("ARP unable to resolve the MAC address of remote side.\r\n");
         APPCTX.timer = SYS_TICK_Get();
      }
      return;
  }

  sprintf( (char *) g_ConsoleContext.txBuf,"    - RemoteNode MAC: %x %x %x %x %x %x\r\n",
           APPCTX.remoteSide.remoteMACAddr.v[0],
           APPCTX.remoteSide.remoteMACAddr.v[1],
           APPCTX.remoteSide.remoteMACAddr.v[2],
           APPCTX.remoteSide.remoteMACAddr.v[3],
           APPCTX.remoteSide.remoteMACAddr.v[4],
           APPCTX.remoteSide.remoteMACAddr.v[5]);

  SYS_CONSOLE_MESSAGE( (char *) g_ConsoleContext.txBuf);
  
  if ( APPCTX.mProtocol == UDP_PROTOCOL )
     APPCTX.state = IPERF_UDP_TX_OPEN_STATE;
  else
     APPCTX.state = IPERF_TCP_TX_OPEN_STATE;

}

static void StateMachineUDPTxOpen(void)
{	
	UDP_SOCKET_INFO UdpSkt;
	
    if ( (APPCTX.udpSock = UDPOpenClient(IP_ADDRESS_TYPE_IPV4, APPCTX.mServerPort, (IP_MULTI_ADDRESS*)&APPCTX.remoteSide.remoteIPaddress.v4Add)) == INVALID_UDP_SOCKET )
    {
        /* error case */
        SYS_CONSOLE_MESSAGE("Create UDP socket failed\r\n");
        APPCTX.state = IPERF_STANDBY_STATE;
        return;
    }

	
	UDPGetSocketInfo(APPCTX.udpSock, &UdpSkt);
    APPCTX.localPort = UdpSkt.localPort;

    SYS_CONSOLE_MESSAGE("---------------------------------------------------------\r\n");
    sprintf( (char *) g_ConsoleContext.txBuf,
             "Client connecting to %u.%u.%u.%u, UDP port %u\r\n",
              APPCTX.remoteSide.remoteIPaddress.v4Add.v[0],
              APPCTX.remoteSide.remoteIPaddress.v4Add.v[1],
              APPCTX.remoteSide.remoteIPaddress.v4Add.v[2],
              APPCTX.remoteSide.remoteIPaddress.v4Add.v[3],
              APPCTX.mServerPort );

    SYS_CONSOLE_MESSAGE( (char *) g_ConsoleContext.txBuf);

    APPCTX.state = IPERF_UDP_TX_DATAGRAM_STATE;

    APPCTX.startTime = IPERF_GET_MSEC_TICK_COUNT();

     // Wait for a few seconds before first TCP tx, so we can resolve ARP.
    APPCTX.nextTxTime = APPCTX.startTime + APPCTX.mPktPeriod;

}

static void StateMachineTCPTxOpen(void)
{

   if  ( (APPCTX.tcpClientSock = TCPOpenClient(IP_ADDRESS_TYPE_IPV4, APPCTX.mServerPort, &APPCTX.remoteSide.remoteIPaddress)) == INVALID_SOCKET )
   {
       /* error case */
        SYS_CONSOLE_MESSAGE("Create TCP socket failed\r\n");
        APPCTX.state = IPERF_STANDBY_STATE;
        return;
    }

    APPCTX.localPort = LOCAL_PORT_START_NUMBER;

    SYS_CONSOLE_MESSAGE("---------------------------------------------------------\r\n");
    sprintf( (char *) g_ConsoleContext.txBuf,
             "Client connecting to %u.%u.%u.%u, TCP port %u\r\n",
              APPCTX.remoteSide.remoteIPaddress.v4Add.v[0],
              APPCTX.remoteSide.remoteIPaddress.v4Add.v[1],
              APPCTX.remoteSide.remoteIPaddress.v4Add.v[2],
              APPCTX.remoteSide.remoteIPaddress.v4Add.v[3],
              APPCTX.mServerPort );

    SYS_CONSOLE_MESSAGE( (char *) g_ConsoleContext.txBuf);

    APPCTX.state =  IPERF_TCP_TX_CONNECT_STATE;

    TCPAdjustFIFOSize(APPCTX.tcpClientSock, 10, 1,TCP_ADJUST_GIVE_REST_TO_TX);

    APPCTX.timer = SYS_TICK_Get();
}


static void StateMachineTCPTxConnect(void)
{

 if ( IperfAppKillRequested() == true )
 {
     APPCTX.state = IPERF_TCP_TX_DONE_STATE;
     return;
 }

  if( !TCPIsConnected(APPCTX.tcpClientSock) )
  {

    // Time out if too much time is spent in this state
    if(SYS_TICK_Get()- APPCTX.timer > 5*SYS_TICK_TicksPerSecondGet())
    {
       TCPDisconnect(APPCTX.tcpClientSock);
       APPCTX.tcpClientSock = INVALID_SOCKET;
       SYS_CONSOLE_MESSAGE("TCP Client connection timeout\r\n");
       APPCTX.state = IPERF_TCP_TX_DONE_STATE;
    }

    return;
  }

  /* reset the reset flag so we don't get a false positive */
  TCPWasReset(APPCTX.tcpClientSock);

  APPCTX.startTime = IPERF_GET_MSEC_TICK_COUNT();
  APPCTX.nextTxTime = APPCTX.startTime + APPCTX.mPktPeriod;
  APPCTX.state = IPERF_TCP_TX_SEGMENT_STATE;
}



static void GenericTxHeaderPreparation(uint8_t *pData, bool isTheLastTransmit)
{
    tIperfPktInfo *pPktInfo = NULL;
    uint32_t currentTime;
    tClientHdr *pClientHdr = NULL;
    long tmp2;

    if ( APPCTX.pktId == 0 ) {
        // The first tx packet
    }

    switch ( APPCTX.mProtocol )
    {
        case TCP_PROTOCOL: // TCP

            // Required by iperf.
            pClientHdr = (tClientHdr *) pData;

            // We borrow the same tIperfPktInfo structure to embed
            // some useful (non-standard iperf) meta info.
            // Unfortunately, the order has to be reversed.

            pPktInfo = (tIperfPktInfo *) (pClientHdr + 1);

            break;

        case UDP_PROTOCOL: // UDP

            // Both are required by iperf.

            pPktInfo = (tIperfPktInfo *) pData;
            pClientHdr = (tClientHdr *) (pPktInfo + 1);

            break;
    }

    // Client header:
    // Needed for all UDP packets.
    // For TCP, only the first two segments need this info. However,
    // there seems to be no harm to put it to all segments though.

    pClientHdr->flags = HTOIPERFL( (uint32_t) 0);
    pClientHdr->numThreads = HTOIPERFL((uint32_t) 1);
    pClientHdr->mPort = HTOIPERFL((uint32_t) APPCTX.mServerPort);
    pClientHdr->bufferlen = HTOIPERFL( (uint32_t) 0);
    pClientHdr->mWinBand = HTOIPERFL(APPCTX.mTxRate);

    if ( APPCTX.mAmount != 0u )
    {
        pClientHdr->mAmount = HTOIPERFL(APPCTX.mAmount);
    }
    else
    {
        pClientHdr->mAmount = HTOIPERFL( - (long) (APPCTX.mDuration/10) );
    }

    // Additional info: needed for UDP only.
    // No harm to put it to all TCP segments though.

    if ( isTheLastTransmit == true )
    {
        // The last UDP tx packet. Some caveats:
        // 1. Iperf uses a negative Id for the last tx packet.
        // 2. Its id should not change during retransmit.

        pPktInfo->id = - ( (long) (APPCTX.pktId - APPCTX.nAttempts) );
    }
    else
    {
        pPktInfo->id = APPCTX.pktId;
    }

    pPktInfo->id = HTOIPERFL(pPktInfo->id);

    currentTime = IPERF_GET_MSEC_TICK_COUNT();

    pPktInfo->tv_sec = HTOIPERFL(currentTime / SYS_TICK_TicksPerSecondGet());

    /* get the remainder of the ticks using modulus */
    tmp2 = ((APPCTX.stopTime - APPCTX.startTime)%SYS_TICK_TicksPerSecondGet());

    /* normalize  to uSecs */
    tmp2 =  tmp2*1000/SYS_TICK_TicksPerSecondGet(); /* Convert to mSec */
    tmp2 *= 1000;   /* 1000 uSecs per mSec */


    pPktInfo->tv_usec = HTOIPERFL( tmp2 );

    return;
}



static bool GenericTxStart(void)
{
    uint32_t currentTime;
    bool iperfKilled;

    currentTime = IPERF_GET_MSEC_TICK_COUNT();

    if ( currentTime < (APPCTX.nextTxTime - TIMING_ERROR_MARGIN))
    {
        // Wait until we are scheduled to Tx.
        return false;
    }

    iperfKilled = IperfAppKillRequested();

    if ((iperfKilled == true) ||
        ((APPCTX.mDuration != 0u) &&
         (currentTime > (APPCTX.startTime + APPCTX.mDuration))) ||
        ((APPCTX.mAmount != 0u) &&
         (APPCTX.totalLen > APPCTX.mAmount)))
    {
        // Prepare to transmit the last packet.
        // Although the last packet needs to be retransmitted UDP_FIN_RETRANSMIT_COUNT times,
        // if we are in UDP mode.

         APPCTX.isLastTransmit = true;
    }

    if ( APPCTX.pktId == 0 )
    {
        // The first pkt is going out ...

        // Reset startTime, to get a more accurate report.

        APPCTX.startTime = currentTime;
        APPCTX.nextTxTime = APPCTX.startTime;

        APPCTX.lastCheckTime = 	APPCTX.startTime;

        APPCTX.lastCheckPktId = APPCTX.pktId;
        APPCTX.lastCheckPktCount = APPCTX.pktCount;
        APPCTX.lastCheckErrorCount = APPCTX.errorCount;
        APPCTX.nAttempts = 0;
    }

    if ( APPCTX.mProtocol == TCP_PROTOCOL )
    {
       /* Manage socket */
       if( TCPIsGetReady(APPCTX.tcpClientSock) > 0u )
       {
          TCPDiscard(APPCTX.tcpClientSock);
          return false;
       }

       if ( TCPWasReset(APPCTX.tcpClientSock) )
       {
         // We don't close the socket. We wait for user to "kill iperf" explicitly.
         SYS_CONSOLE_MESSAGE("\r\n    Warning, TCP server disconnect detected\r\n");
       }

       if  (( TCPIsPutReady(APPCTX.tcpClientSock) <= APPCTX.mMSS ) && (!iperfKilled))
          return false;

    }
    else
    {
       /* Manage socket */
       if( UDPIsGetReady(APPCTX.udpSock) > 0u )
       {
          UDPDiscard(APPCTX.udpSock);
          return false;
       }

       if ( UDPIsPutReady(APPCTX.udpSock) <= APPCTX.mDatagramSize )
       {
          return false;
       }

    }


    // One Tx per mPktPeriod msec.
    APPCTX.nextTxTime = currentTime + APPCTX.mPktPeriod;

    GenericTxHeaderPreparation(g_bfr, APPCTX.isLastTransmit);

    if ( APPCTX.mProtocol == UDP_PROTOCOL )
    {

        APPCTX.remainingTxData = (APPCTX.mDatagramSize - MAX_BUFFER);

        if ( UDPPutArray(APPCTX.udpSock, g_bfr, MAX_BUFFER) != MAX_BUFFER )
        {
            SYS_CONSOLE_MESSAGE("Socket send failed\r\n");
            APPCTX.errorCount++;
            return false;
        }

    }
    else
    {

        APPCTX.remainingTxData = (APPCTX.mMSS - MAX_BUFFER);

        if (( TCPPutArray(APPCTX.tcpClientSock, (BYTE*) g_bfr, MAX_BUFFER) != MAX_BUFFER ) && (!iperfKilled))
        {
            SYS_CONSOLE_MESSAGE("Socket send failed\r\n");
            APPCTX.errorCount++;
            return false;
        }

    }

     return true;

}


/* This routine does a piecewise send, because the entire RAM buffer may not be available for putArray */
static void TcpTxFillSegment(void)
{
  uint16_t chunk;

  /* Fill the buffer with ASCII char T */
  memset( g_bfr, 0x54, MAX_BUFFER);

  while( APPCTX.remainingTxData > 0u )
  {
    chunk = MAX_BUFFER;

    /* finish case where we get more than is needed */
    if ( APPCTX.remainingTxData < MAX_BUFFER )
      chunk = APPCTX.remainingTxData;

    APPCTX.remainingTxData -= chunk;

    if ( TCPPutArray( APPCTX.tcpClientSock, (BYTE *) g_bfr, chunk) != chunk )
      return;

  }

}

/* This routine does a piece wis send, because the entire RAM pkt buffer may not be available for putArray */
static void UdpTxFillDatagram(void)
{

  uint16_t chunk;

  /* Fill the buffer with ASCII char U */
  memset( g_bfr, 0x55, MAX_BUFFER);

  while( APPCTX.remainingTxData > 0u )
  {
    chunk = MAX_BUFFER;

    /* finish case where we get more than is needed */
    if ( APPCTX.remainingTxData < MAX_BUFFER )
      chunk = APPCTX.remainingTxData;

    APPCTX.remainingTxData -= chunk;

    if (  UDPPutArray(APPCTX.udpSock, (BYTE *) g_bfr, chunk) != chunk )
     return;

  }

}



static void GenericTxEnd(void)
{

  if(  APPCTX.remainingTxData  > 0u )
  {
     /* unhandled error */
     SYS_CONSOLE_MESSAGE("Socket send failed\r\n");
     APPCTX.errorCount++;
  }
  else
  {
     // send successful.

     if ( APPCTX.pktCount == 0u )
     {
        // first tx pkt

        SYS_CONSOLE_MESSAGE("\n\r    Session started ...\r\n");


        sprintf( (char *) g_ConsoleContext.txBuf,
                 "    - Local  %u.%u.%u.%u port %u connected with\r\n",
                 pNetIf->MyIPAddr.v[0],
                 pNetIf->MyIPAddr.v[1],
                 pNetIf->MyIPAddr.v[2],
                 pNetIf->MyIPAddr.v[3],
                 APPCTX.localPort);

        SYS_CONSOLE_MESSAGE( (char *) g_ConsoleContext.txBuf);

        sprintf( (char *) g_ConsoleContext.txBuf,
                 "    - Remote %u.%u.%u.%u port %u\r\n",
                 APPCTX.remoteSide.remoteIPaddress.v4Add.v[0],
                 APPCTX.remoteSide.remoteIPaddress.v4Add.v[1],
                 APPCTX.remoteSide.remoteIPaddress.v4Add.v[2],
                 APPCTX.remoteSide.remoteIPaddress.v4Add.v[3],
                 APPCTX.mServerPort );

        SYS_CONSOLE_MESSAGE( (char *) g_ConsoleContext.txBuf);

        sprintf( (char *) g_ConsoleContext.txBuf, "    - Target rate = %ld bps, period = %ld ms\r\n",
                                        (unsigned long)APPCTX.mTxRate, 
                                        (unsigned long)(APPCTX.mPktPeriod*1000/SYS_TICK_TicksPerSecondGet()) );

        SYS_CONSOLE_MESSAGE( (char *) g_ConsoleContext.txBuf);

     }

     APPCTX.pktId++;
     APPCTX.pktCount++;

     if ( APPCTX.mProtocol == UDP_PROTOCOL )
     {
        APPCTX.totalLen += APPCTX.mDatagramSize;
     }
     else
     {

        APPCTX.totalLen += APPCTX.mMSS;
     }


  }

  APPCTX.lastPktId = APPCTX.pktId - 1;



  if ( IPERF_GET_MSEC_TICK_COUNT() > (APPCTX.lastCheckTime + APPCTX.mInterval - TIMING_ERROR_MARGIN) )
  {
        // Time to report statistics
        ReportBW_Jitter_Loss(INTERVAL_REPORT);

        //APPCTX.lastCheckPktCount = APPCTX.pktCount;
  }


  if ( APPCTX.isLastTransmit == true )
  {
      if ((APPCTX.mProtocol == (tIperfProto)UDP_PROTOCOL) &&
          (++APPCTX.nAttempts < UDP_FIN_RETRANSMIT_COUNT) ) {

          if ( APPCTX.nAttempts == 1u )
          {
              // So the normal pkt statistics is not mixed with the retransmited last pkt.
              APPCTX.stopTime = IPERF_GET_MSEC_TICK_COUNT();

              ReportBW_Jitter_Loss(SUBTOTAL_REPORT);
              SYS_CONSOLE_MESSAGE("    -----------------------------------------\r\n"); 
          }

          // Don't follow the same transmision rate during retransmit.
          APPCTX.mPktPeriod = UDP_FIN_RETRANSMIT_PERIOD;
      }
      else
      {
          APPCTX.state = (APPCTX.mProtocol == UDP_PROTOCOL) ? IPERF_UDP_TX_DONE_STATE : IPERF_TCP_TX_DONE_STATE;
          APPCTX.stopTime = IPERF_GET_MSEC_TICK_COUNT();
      }
  }

}

static void StateMachineTcpTxSegment(void)
{

    if ( GenericTxStart() == true )
    {
       TcpTxFillSegment();
       TCPFlush(APPCTX.tcpClientSock);
       GenericTxEnd();
    }
}

static void StateMachineUdpTxDatagram(void)
{

    if ( GenericTxStart() == true )
    {
       UdpTxFillDatagram();
       UDPFlush(APPCTX.udpSock);
       GenericTxEnd();
    }
}


static void
GenericTxDone(void)
{
    if ( APPCTX.statusReported == 0u )
    {
        ReportBW_Jitter_Loss(SESSION_REPORT);
        APPCTX.statusReported = 1;
    }

    APPCTX.state = IPERF_STANDBY_STATE;

    SYS_CONSOLE_MESSAGE("    Tx done. Socket closed.\r\n");

    // Clear statistics
    ResetIperfCounters();

    //SYS_CONSOLE_MESSAGE("    Back to standby mode.\r\n");
    SYS_CONSOLE_MESSAGE("    Iperf completed.\r\n");

}


static void StateMachineTcpTxDone(void)
{
    GenericTxDone();

    //No calling this API twice is NOT redundant...  The 2nd time forces a RST per Microchip's own doc for v4.51
    TCPDisconnect(APPCTX.tcpClientSock);
    TCPDisconnect(APPCTX.tcpClientSock);
    APPCTX.tcpClientSock = INVALID_SOCKET;
}

static void
StateMachineUdpTxDone(void)
{

    GenericTxDone();

    UDPClose(APPCTX.udpSock );
}


void IperfAppCall(void)
{
    uint8_t argc;
    char **argv;

	if ( IperfConsoleIsConsoleMsgReceived() == true )
	{
       	argv = IperfConsoleGetCmdLineArgv();
       	argc = IperfConsoleGetCmdLineArgc();

		if ( memcmp(argv[0], "help", 4) == 0 )
		{
            SYS_CONSOLE_MESSAGE("iperf\t\tsee documentation\r\n");
            SYS_CONSOLE_MESSAGE("kill iperf\tstop the running iperf session\r\n");
		}
	    /* This check is for duplicate iperf app calls */
    	/* the only legitimate state that is ready for */
   		/* iperf server or client is standby           */
		else if ( APPCTX.state != (uint8_t)IPERF_STANDBY_STATE )
		{
        	if ( memcmp(argv[0], "iperf", 5) == 0 )
        	{
            	SYS_CONSOLE_MESSAGE("\r\nError, Iperf session already started\r\n");
            	IperfConsoleReleaseConsoleMsg();
        	}
		}
	}

    switch ( APPCTX.state )
    {
        case IPERF_STANDBY_STATE:

            StateMachineStandby();

            break;

        /********************/
        /* RX Client States */
        /********************/

        case IPERF_RX_START_STATE:

            StateMachineRxStart();

            break;

        case IPERF_UDP_RX_STATE:

            StateMachineUdpRx();

            break;

        case IPERF_UDP_RX_DRAIN_STATE:

            StateMachineUdpRxDrain();

            break;


        case IPERF_UDP_RX_DONE_STATE:

            StateMachineUdpRxDone();

            break;


        case IPERF_TCP_RX_LISTEN_STATE:

            StateMachineTcpListen();

            break;

        case IPERF_TCP_RX_STATE:

            StateMachineTcpRx();

            break;

        case IPERF_TCP_RX_DONE_STATE:

            StateMachineTcpRxDone();

            break;

        case IPERF_RX_DONE_STATE:

            StateMachineRxDone();

            break;


       /********************/
       /* TX Client states */
       /********************/


        case IPERF_TX_START_STATE:

            StateMachineTxStart();

            break;


        case IPERF_TX_ARP_RESOLVE_STATE:

           StateMachineTxArpResolve();

           break;

        case IPERF_UDP_TX_OPEN_STATE:

            StateMachineUDPTxOpen();

            break;

        case IPERF_TCP_TX_OPEN_STATE:

            StateMachineTCPTxOpen();

            break;

        case IPERF_TCP_TX_CONNECT_STATE:

            StateMachineTCPTxConnect();

            break;

        case IPERF_TCP_TX_SEGMENT_STATE:

            StateMachineTcpTxSegment();

            break;

        case IPERF_UDP_TX_DATAGRAM_STATE:

            StateMachineUdpTxDatagram();

            break;

        case IPERF_TCP_TX_DONE_STATE:

            StateMachineTcpTxDone();

            break;


        case IPERF_UDP_TX_DONE_STATE:

            StateMachineUdpTxDone();

            break;

        }
}

#endif /* CMD_PARSER */
