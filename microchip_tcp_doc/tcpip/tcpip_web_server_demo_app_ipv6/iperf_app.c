/******************************************************************************

 MRF24W Driver iperf
 Module for Microchip TCP/IP Stack
  -Provides access to MRF24W WiFi controller
  -Reference: MRF24W Data sheet, IEEE 802.11 Standard

*******************************************************************************
 FileName:		IperfApp.c
 Dependencies:	TCP/IP Stack header files
 Processor:		PIC24F, PIC24H, dsPIC30F, dsPIC33F, PIC32
 Compiler:		Microchip C32 v1.10b or higher
				Microchip C30 v3.22 or higher
 Company:		Microchip Technology, Inc.

 Software License Agreement

 Copyright (C) 2002-2010 Microchip Technology Inc.  All rights reserved.

 Microchip licenses to you the right to use, modify, copy, and distribute:
 (i)  the Software when embedded on a Microchip microcontroller or digital 
      signal controller product ("Device") which is integrated into 
      Licensee's product; or
 (ii) ONLY the Software driver source files ENC28J60.c, ENC28J60.h,
      ENCX24J600.c and ENCX24J600.h ported to a non-Microchip device used in 
	  conjunction with a Microchip ethernet controller for the sole purpose 
	  of interfacing with the ethernet controller.

 You should refer to the license agreement accompanying this Software for 
 additional information regarding your rights and obligations.

 THE SOFTWARE AND DOCUMENTATION ARE PROVIDED "AS IS" WITHOUT WARRANTY OF ANY
 KIND, EITHER EXPRESS OR IMPLIED, INCLUDING WITHOUT LIMITATION, ANY WARRANTY
 OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE, TITLE AND
 NON-INFRINGEMENT. IN NO EVENT SHALL MICROCHIP BE LIABLE FOR ANY INCIDENTAL,
 SPECIAL, INDIRECT OR CONSEQUENTIAL DAMAGES, LOST PROFITS OR LOST DATA, COST
 OF PROCUREMENT OF SUBSTITUTE GOODS, TECHNOLOGY OR SERVICES, ANY CLAIMS BY
 THIRD PARTIES (INCLUDING BUT NOT LIMITED TO ANY DEFENSE THEREOF), ANY CLAIMS
 FOR INDEMNITY OR CONTRIBUTION, OR OTHER SIMILAR COSTS, WHETHER ASSERTED ON
 THE BASIS OF CONTRACT, TORT (INCLUDING NEGLIGENCE), BREACH OF WARRANTY, OR
 OTHERWISE.


 Author				Date		Comment
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
 KH                 27 Jan 2010 Updated for MRF24W
******************************************************************************/

#include <string.h> /* for memcpy */
#include <assert.h>
#include "tcpip_config.h"
#include "tcpip/tcpip.h"
#include "tcpip/tcp.h"

#if defined (CMD_CONSOLE) && defined(TCPIP_STACK_USE_IPERF)
#include "tcpip/wf_console.h"

/*****************************************************************************/
/* Constants used internally by code in this file.                           */
/*****************************************************************************/
/*--------------------*/
/* Endianness defines */
/*--------------------*/
#define BIG_ENDIAN                   (0)
#define LITTLE_ENDIAN                (1)
/* Indicate whether the Host CPU is big-endian or little-endian */
#define HOST_CPU_ENDIANNESS          LITTLE_ENDIAN    /* BIG_ENDIAN or LITTLE_ENDIAN */
/*-------------------*/
/* Endianness Macros */
/*-------------------*/
/* if the Host CPU is Little Endian, which does not match the MRF24W */
#if (HOST_CPU_ENDIANNESS == LITTLE_ENDIAN)    
    /* 32-bit data type conversion */
    #define HTOWFL(a) (((a & 0x000000ff) << 24) | ((a & 0x0000ff00) << 8) | ((a & 0x00ff0000) >> 8) | ((a & 0xff000000) >> 24))
    #define WFTOHL(a) HTOWFL(a)
    /* 16-bit data type conversion */
    #define HSTOWFS(a) (((a) << 8) | ((a) >> 8))
    #define WFSTOHS(a) HSTOWFS(a)    
/* else Host CPU is Big-Endian, which matches the MRF24W */
#else
    #define HTOWFL(a)   (a)
    #define WFTOHL(a)   (a)    
    #define HSTOWFS(a)  (a)
    #define WFSTOHS(a)  (a)
#endif

//#define kIperfRxTimeOut 				1000    // 1 second.

#define kIperfUDPFinRetransmitCount		10u		// iperf retransmits 10 times the last UDP packet,
#define kIperfUDPFinRetransmitPeriod    10      // at 10ms apart.
#define kIperfTimingErrorMargin 		2		// Account for msec tick uncertainty.

#define IPERF_GET_MSEC_TICK_COUNT() SYS_TICK_Get()
#define IPERF_ASSERT(expr) assert(expr)

enum {
    kStateStandby=1,

    kStateRxStart,
    kStateUdpRx,
    kStateUdpRxDrain,
    kStateUdpRxDone,
    kStateTcpRxListen,
    kStateTcpRx,
    kStateTcpRxDone,
    kStateRxDone,

    kStateTxStart,
    kStateTxArpResolve,

    kStateTcpTxOpen,
    kStateTcpTxConnect,
    kStateTcpTxSegment,
    kStateTcpTxDone,

    kStateUdpTxOpen,
    kStateUdpTxDatagram,
    kStateUdpTxDone

};

/*****************************************************************************/
/* Data types used internally by code in this file.                          */
/*****************************************************************************/

typedef enum {
    kIperfProtoUDP = 1,
    kIperfProtoTCP
} tIperfProto;

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
    //	tWFSocketAddr remoteAddr;

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

typedef enum
{
    kIntervalReport,
    kSubtotalReport,
    kSessionReport
} tIperfReport;

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
/*****************************************************************************/
/* Global variables used internally by code in this file.                    */
/*****************************************************************************/
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

static void
ascii_to_u32s(int8_t *ptr, uint32_t *values, uint8_t count)
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

static void
ReportBW_Jitter_Loss(tIperfReport reportType)
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
        case kIntervalReport:

            nDropped = APPCTX.errorCount - APPCTX.lastCheckErrorCount;

            // bits-per-msec == Kbps

            sec = (currentTime- APPCTX.lastCheckTime)/SYS_TICK_TicksPerSecondGet();
			msec = ((double) (currentTime - APPCTX.lastCheckTime)) / (((double)(SYS_TICK_TicksPerSecondGet()))/1000);

            if ( APPCTX.state == (uint8_t)kStateUdpTxDone )
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

            sprintf( (char *) g_ConsoleContext.txBuf,"    - [%2lu- %2lu sec] %3lu/ %3lu (%2lu%%)    %4lu Kbps",
                      (unsigned long)sec, 
                      (unsigned long)sec + ( (unsigned long) (APPCTX.mInterval/SYS_TICK_TicksPerSecondGet()) ),
                      (unsigned long)nDropped,
                      (unsigned long)nAttempted,
                      (unsigned long)nDropped*100/(unsigned long)nAttempted,
                      (unsigned long) (kbps + ((double) 0.5)));

            WFConsolePrintStr( (char *) g_ConsoleContext.txBuf , true);

            break;


        case kSubtotalReport:
            // intentional fall-through
        case kSessionReport:

           nDropped = APPCTX.errorCount;

           if (APPCTX.state == (uint8_t)kStateUdpTxDone)
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

            sprintf( (char *) g_ConsoleContext.txBuf, "    - [0.0- %lu.%lu sec] %3lu/ %3lu (%2lu%%)    %4lu Kbps",
                             (unsigned long)(msec/1000),
                             (unsigned long)((msec%1000)/100),
                             (unsigned long)nDropped,
                             (unsigned long)nAttempted,
                             (nAttempted == 0u) ? 0 : ((unsigned long)nDropped*100/(unsigned long)nAttempted),
                             (unsigned long) (kbps + ((double) 0.5)));

            WFConsolePrintStr( (char *) g_ConsoleContext.txBuf , true);

            break;
    }

    if ( reportType == 	kSessionReport )
    {
      WFConsolePrintStr("    Session completed ...", true);
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

        APPCTX.state = kStateStandby;
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
    int8_t **argv;

    // Did user ask to "kill iperf"?


    if ( WFConsoleIsConsoleMsgReceived() == false ) return false;

    argv = WFConsoleGetCmdLineArgv();
    argc = WFConsoleGetCmdLineArgc();

    if (argc == 2u)
    {
        if ((memcmp(argv[0], "kill",  4) == 0) &&
           (memcmp(argv[1],  "iperf", 5) == 0))
        {
           //WFCliServicePerformed();
           WFConsoleReleaseConsoleMsg();

           APPCTX.stopRequested = true;
           return true;
        }
    }
    WFConsolePrintStr("Invalid commmand sequence", true);
    WFConsoleReleaseConsoleMsg();
    return false;

}


static void StateMachineStandby(void)
{
    //tWFCliArguments *args;
    uint8_t argc;
    int8_t **argv;
    uint8_t i;
    int8_t *ptr;
    uint32_t values[4];
    uint16_t payloadSize;
    float pktRate;
    
    //if ( WFCliServiceRequested() == false ) return;
    //if ( !(args = WFCliGetArguments()) ) return;


    if ( WFConsoleIsConsoleMsgReceived() == false ) return;
    
    argv = WFConsoleGetCmdLineArgv();
    argc = WFConsoleGetCmdLineArgc();   // needed because WFConsoleGetCmdLineTokens() returns 0 for argc when in -Os 


    if ( (argc == 2u)                             &&
         (memcmp(argv[0], "kill", 4) == 0) &&
         (memcmp(argv[1], "iperf", 5) == 0) )
    {
       WFConsolePrintStr("Iperf not started for the kill...", true);
       WFConsoleReleaseConsoleMsg();
       return;
    }

    if ((memcmp(argv[0], "kill", 4) == 0) && (argc == 1u))
    {
        WFConsolePrintStr("'kill iperf' is the only valid kill command", true);
        WFConsoleReleaseConsoleMsg();
    }



    if ( (argc == 2u)                             &&
         (memcmp(argv[0], "kill", 4) == 0) &&
         (memcmp(argv[1], "iperf", 5) != 0) )
    {
       WFConsolePrintStr("Invalid command sequence...", true);
       WFConsoleReleaseConsoleMsg();
       return;
    }


    if ( memcmp(argv[0], "iperf", 5) ) return;

    // OK, we will handle the "iperf" command.

    APPCTX.mServerMode = false;
    APPCTX.mProtocol = kIperfProtoTCP;   			// default is TCP mode.
    APPCTX.stopRequested = false;

    APPCTX.mServerPort = IPERF_APP_SERVER_PORT;		// -p. default: server port 5001



    APPCTX.mTxRate = ((uint32_t) 500)*((uint32_t) 1000);		// -b or -x. Target tx rate.
    // KS: default tx rate for iperf is actually 1Mbps. Here we set it to 500Kbps instead.

    APPCTX.mAmount = 0;			// -n: default 0.
    APPCTX.mDuration = ((uint32_t) 10)*((uint32_t) SYS_TICK_TicksPerSecondGet()); // -t: default 10 sec.
    APPCTX.mInterval =  SYS_TICK_TicksPerSecondGet(); 	// -i: default 1 sec.

    // Initialize statistics

    ResetIperfCounters();

    IPERF_ASSERT(argc != 0);
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
            APPCTX.mProtocol = kIperfProtoUDP;
        }
        else if ((memcmp(argv[i], "-b", 2) == 0) ||
                 (memcmp(argv[i], "--bandwidth", 5) == 0) )
        {
            // iperf UDP mode.

            APPCTX.mProtocol = kIperfProtoUDP;

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

            APPCTX.remoteSide.remoteIP.remoteIPv4.v[0] = values[0];
            APPCTX.remoteSide.remoteIP.remoteIPv4.v[1] = values[1];
            APPCTX.remoteSide.remoteIP.remoteIPv4.v[2] = values[2];
            APPCTX.remoteSide.remoteIP.remoteIPv4.v[3] = values[3];  

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
               sprintf( (char *) g_ConsoleContext.txBuf,"The minimum datagram size is %d", (int)MAX_BUFFER);
               WFConsolePrintStr( (char *) g_ConsoleContext.txBuf, true);

               WFConsoleReleaseConsoleMsg();
               return;
            }

            APPCTX.mDatagramSize = values[0];
        }
    }

    //WFCliServicePerformed();
    WFConsoleReleaseConsoleMsg();

    switch (APPCTX.mServerMode)
    {
        case 0:
            // iperf client

            payloadSize = 	(APPCTX.mProtocol == kIperfProtoUDP) ?
                            APPCTX.mDatagramSize : APPCTX.mMSS;

            pktRate =  (float) (APPCTX.mTxRate / 8) / (float) payloadSize;
            APPCTX.mPktPeriod =  (uint32_t) ( (float) SYS_TICK_TicksPerSecondGet() / pktRate );

            APPCTX.state = kStateTxStart;
            break;

        case 1:
            // iperf server

            WFConsolePrintStr("---------------------------------------------------------", true);

            WFConsolePrintStr("Server listening on ", false);
            if (APPCTX.mProtocol == kIperfProtoUDP)
            {
                WFConsolePrintStr((const char *)"UDP", false);
            }    
            else
            {
                WFConsolePrintStr((const char *)"TCP", false);
            }    
            WFConsolePrintStr(" port ", false);
            WFConsolePrintInteger(APPCTX.mServerPort, 'd');
            WFConsolePrintStr("", true);
            APPCTX.state = kStateRxStart;
            break;
    }
}

static void
StateMachineRxStart(void)
{
  if ( !APPCTX.mServerMode )
  {
     SYS_MESSAGE("Unsupported Configuration\n\r");
     APPCTX.state = kStateStandby;
     return;
  }


 switch ( APPCTX.mProtocol )
 {
      case kIperfProtoTCP:	// TCP


            /* TCP Server sockets are allocated for entire runtime duration, a call to disconnect does not free them */
            /* therefore a subsequent N+1 open will fail */
            if ( (APPCTX.tcpServerSock == INVALID_SOCKET) &&
                 (APPCTX.tcpServerSock = TCPOpenServer(IP_ADDRESS_TYPE_IPV4, APPCTX.mServerPort, 0)) == INVALID_SOCKET )
            {
               /* error case */
               WFConsolePrintStr("Create TCP socket failed", true);
               APPCTX.state = kStateStandby;
               return;
            }

            TCPAdjustFIFOSize(APPCTX.tcpServerSock, 1, 0,TCP_ADJUST_PRESERVE_RX | TCP_ADJUST_GIVE_REST_TO_RX);

            APPCTX.state = kStateTcpRxListen;
            break;

      case kIperfProtoUDP:	// UDP

           if ( (APPCTX.udpSock = UDPOpenServer(IP_ADDRESS_TYPE_IPV4, APPCTX.mServerPort, 0)) == INVALID_UDP_SOCKET )
           {
               /* error case */
               WFConsolePrintStr("Create UDP socket failed", true);
               APPCTX.state = kStateStandby;
               return;
           }

           UDPSocketSetNet(APPCTX.udpSock, pNetIf); // Bind test socket to dedicated interface for UDP Ack diagram
           APPCTX.state = kStateUdpRx;
        break;

      default:
           WFConsolePrintStr("Protocol error", true);
           APPCTX.state = kStateStandby;
           return;
  }

}


static void
StateMachineTcpListen(void)
{

   if ( IperfAppKillRequested() == true )
   {
        APPCTX.state = kStateRxDone;
        return;
   }

   if( TCPIsConnected(APPCTX.tcpServerSock) )
   {
      TCP_SOCKET_INFO socketInfo;
      TCPGetSocketInfo( APPCTX.tcpServerSock, &socketInfo );
      memcpy ( (void *) &APPCTX.remoteSide, (void *) &socketInfo, sizeof ( TCP_SOCKET_INFO) );
      APPCTX.state = kStateTcpRx;

      /* clear the stack's reset flag */
      TCPWasReset(APPCTX.tcpServerSock);
   }
}


static void
StateMachineTcpRx(void)
{
    uint16_t length;

    if( (length = TCPIsGetReady(APPCTX.tcpServerSock)) == 0 )
    {

      if ( TCPWasReset(APPCTX.tcpServerSock)  )
      {
          APPCTX.stopTime = IPERF_GET_MSEC_TICK_COUNT();
          APPCTX.state = kStateTcpRxDone;
          return;
      }

    }
    else
    {
       if ( APPCTX.pktId == 0)
       {
          // This is the first rx pkt.
          SYS_MESSAGE("\n\r    Session started ...\n\r");

          APPCTX.startTime = IPERF_GET_MSEC_TICK_COUNT();
          APPCTX.lastCheckTime = 	APPCTX.startTime;

          APPCTX.lastCheckPktId = APPCTX.pktId;

          sprintf((char *) g_ConsoleContext.txBuf, "    - Local  %u.%u.%u.%u port %u connected with",
                   pNetIf->MyIPAddr.v[0],
                   pNetIf->MyIPAddr.v[1],
                   pNetIf->MyIPAddr.v[2],
                   pNetIf->MyIPAddr.v[3],
                   APPCTX.mServerPort);

          WFConsolePrintStr( (char *) g_ConsoleContext.txBuf , true);

          sprintf( (char *) g_ConsoleContext.txBuf,"    - Remote %u.%u.%u.%u port %u",
                   APPCTX.remoteSide.remoteIP.remoteIPv4.v[0],
                   APPCTX.remoteSide.remoteIP.remoteIPv4.v[1],
                   APPCTX.remoteSide.remoteIP.remoteIPv4.v[2],
                   APPCTX.remoteSide.remoteIP.remoteIPv4.v[3],
                   APPCTX.remoteSide.remotePort.Val );

          WFConsolePrintStr( (char *) g_ConsoleContext.txBuf , true);

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
         ReportBW_Jitter_Loss(kIntervalReport);
    }

    if ( IperfAppKillRequested() == true )
    {
       APPCTX.state = kStateTcpRxDone;
       APPCTX.stopTime = IPERF_GET_MSEC_TICK_COUNT();

       return;
    }
}


static void
StateMachineUdpRx(void)
{
    uint16_t length =0;
    tIperfPktInfo *pPktInfo;
    UDP_SOCKET_INFO sktInfo;

    // Do nothing if no data is waiting
    if( (length = UDPIsGetReady(APPCTX.udpSock)) >= (uint16_t)(sizeof(tIperfPktInfo)) )
    {

       /* The GetArray should not fail... */
       if ( UDPGetArray(APPCTX.udpSock, (BYTE*)g_bfr, sizeof(tIperfPktInfo)) != sizeof(tIperfPktInfo) )
       {
          SYS_MESSAGE("      UDP Get Array Failed\n\r");
          APPCTX.state = kStateUdpRxDrain;
          return;
       }

       pPktInfo = (tIperfPktInfo *) g_bfr;
       APPCTX.pktId = WFTOHL(pPktInfo->id);

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

          WFConsolePrintStr("", true);
          WFConsolePrintStr("    Session started ...", true);

          if ( APPCTX.pktId != 0 )
          {
             // We have lost a few packets before the first pkt arrived.
             sprintf((char *) g_ConsoleContext.txBuf, "    - First pkt id = %ld (should be 0)",
                              APPCTX.pktId);

             WFConsolePrintStr((char *) g_ConsoleContext.txBuf, true );


             // The first data pkt starts with id = 1.
             APPCTX.errorCount	+= 	APPCTX.pktId - 1;
          }

          APPCTX.lastPktId = APPCTX.pktId;

          sprintf((char *) g_ConsoleContext.txBuf, "    - Local  %u.%u.%u.%u port %u connected with ",
                           pNetIf->MyIPAddr.v[0],
                           pNetIf->MyIPAddr.v[1],
                           pNetIf->MyIPAddr.v[2],
                           pNetIf->MyIPAddr.v[3],
                           APPCTX.mServerPort);

          WFConsolePrintStr( (char *) g_ConsoleContext.txBuf , true);

          UDPGetSocketInfo(APPCTX.udpSock, &sktInfo);
          sprintf((char *) g_ConsoleContext.txBuf,"    - Remote %u.%u.%u.%u port %u",      
                           remoteNode.IPAddr.v[0],
                           remoteNode.IPAddr.v[1],
                           remoteNode.IPAddr.v[2],
                           remoteNode.IPAddr.v[3],
                           sktInfo.remotePort );

          WFConsolePrintStr( (char *) g_ConsoleContext.txBuf , true);

          // Store the remote info so we can send the iperf "UDP-FIN-ACK" msg
          APPCTX.remoteSide.remoteIP.remoteIPv4.v[0] = remoteNode.IPAddr.v[0];
          APPCTX.remoteSide.remoteIP.remoteIPv4.v[1] = remoteNode.IPAddr.v[1];
          APPCTX.remoteSide.remoteIP.remoteIPv4.v[2] = remoteNode.IPAddr.v[2];
          APPCTX.remoteSide.remoteIP.remoteIPv4.v[3] = remoteNode.IPAddr.v[3];

          APPCTX.remoteSide.remotePort.Val =  sktInfo.remotePort;

          APPCTX.startTime = IPERF_GET_MSEC_TICK_COUNT();
          //APPCTX.remoteStartTime = WFTOHL(pPktInfo->tv_sec);

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
        //APPCTX.remoteStopTime = WFTOHL(pPktInfo->tv_sec);

        APPCTX.nAttempts = 0;
        APPCTX.state = kStateUdpRxDrain;
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
         WFConsolePrintStr("      Recv duplicated pkt", true);
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
          WFConsolePrintStr("    Rx timed out", true);

          APPCTX.stopTime = IPERF_GET_MSEC_TICK_COUNT();

          APPCTX.nAttempts = 0;
          APPCTX.state = kStateUdpRxDrain;
        }
        else
        {
          ReportBW_Jitter_Loss(kIntervalReport);
        }
    }

    if ( IperfAppKillRequested() == true )
    {
        APPCTX.state = kStateUdpRxDrain;
        return;
    }

}

static void
StateMachineUdpRxDrain(void)
{
    if( UDPIsGetReady(APPCTX.udpSock) > (uint8_t)0 )
    {
         UDPDiscard(APPCTX.udpSock);
         return;
    }

   /* If iperf kill was done, just jump to closing the socket */
   if ( APPCTX.stopRequested )
     APPCTX.state = kStateRxDone;
   else /* go ahead an generate reports, etc */
     APPCTX.state = kStateUdpRxDone;

}


static void
StateMachineUdpRxDone(void)
{
    tIperfPktInfo *pPktInfo;
    tServerHdr *pServer_hdr;
    uint16_t  tmp;
    float tmp2;


    if ( APPCTX.statusReported == 0u )
    {
        ReportBW_Jitter_Loss(kSessionReport);
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
        APPCTX.state = kStateRxDone;
        return;
    }

     /* Make sure space is available to TX the ACK packet of 128 bytes */
    if ( UDPIsPutReady(APPCTX.udpSock ) > 0u )
    {

      pPktInfo = (tIperfPktInfo *) g_bfr;
      pServer_hdr = (tServerHdr *) (pPktInfo +1);

      pPktInfo->id = HTOWFL( -APPCTX.lastPktId );
      pPktInfo->tv_sec = 0;
      pPktInfo->tv_usec = 0;

      pServer_hdr->flags = HTOWFL(HEADER_VERSION1);
      pServer_hdr->total_len1 = 0;
      pServer_hdr->total_len2 = HTOWFL( (uint32_t) APPCTX.totalLen);

      pServer_hdr->stop_sec =  HTOWFL( (uint32_t) (APPCTX.stopTime - APPCTX.startTime)/SYS_TICK_TicksPerSecondGet());

      /* get the remainder of the ticks using modulus */
      tmp2 = ((APPCTX.stopTime - APPCTX.startTime)%SYS_TICK_TicksPerSecondGet());

      /* normalize  to uSecs */
      tmp2 =  tmp2*1000/SYS_TICK_TicksPerSecondGet(); /* Convert to mSec */
      tmp2 *= 1000;   /* 1000 uSecs per mSec */


      pServer_hdr->stop_usec = HTOWFL( (uint32_t) tmp2 );
      pServer_hdr->error_cnt = HTOWFL( (uint32_t)  APPCTX.errorCount);;
      pServer_hdr->outorder_cnt = HTOWFL( (uint32_t) APPCTX.outofOrder);
      pServer_hdr->datagrams = HTOWFL( (uint32_t) APPCTX.lastPktId);
      pServer_hdr->jitter1 = 0;
      pServer_hdr->jitter2 = 0;

      UDPPutArray(APPCTX.udpSock, (BYTE*)g_bfr, MAX_BUFFER);

      for ( tmp=0; tmp < (128 - MAX_BUFFER); tmp++)
        UDPPut(APPCTX.udpSock, 0x00);

      UDPFlush(APPCTX.udpSock );

    }

}

static void
StateMachineTcpRxDone(void)
{
    if ( APPCTX.statusReported == 0u )
    {
        ReportBW_Jitter_Loss(kSessionReport);
        APPCTX.statusReported = 1;
    }

    APPCTX.state = kStateRxDone;
}

static void
StateMachineRxDone(void)
{

   if ( APPCTX.mProtocol == kIperfProtoUDP )
   {
     UDPClose(  APPCTX.udpSock );
   }
   else
   {

     /* 2 disconnects on purpose not mistake */
     TCPDisconnect( APPCTX.tcpServerSock );
     TCPDisconnect( APPCTX.tcpServerSock );
   }

    WFConsolePrintStr("", true);
    WFConsolePrintStr("    Rx done. Socket closed.", true);

    // Clear statistics
    ResetIperfCounters();

    // In server mode, continue to accept new session requests ...

    if ((APPCTX.mServerMode == true) 	&&
        (APPCTX.stopRequested == false) )
    {
        WFConsolePrintStr("    Ready for the next session.", true);

        APPCTX.state = kStateRxStart;
    }
    else
    {
        WFConsolePrintStr("    Iperf completed.", true);
        APPCTX.state = kStateStandby;
    }

}




/******************************/
/* TX CLIENT CODE BEGINS HERE */
/******************************/


static void
StateMachineTxStart(void)
{

   ARPResolve(pNetIf, &APPCTX.remoteSide.remoteIP.remoteIPv4);
   APPCTX.state = kStateTxArpResolve;
   APPCTX.timer = SYS_TICK_Get();

}

static void
StateMachineTxArpResolve(void)
{

  if ( IperfAppKillRequested() == true )
  {
     WFConsolePrintStr("Iperf client session closed.", true);
     APPCTX.state = kStateStandby;
     return;
  }

  if(!ARPIsResolved(pNetIf, &APPCTX.remoteSide.remoteIP.remoteIPv4, &APPCTX.remoteSide.remoteMACAddr))
  {
      /* every 3 seconds print a warning */
      if( SYS_TICK_Get() - APPCTX.timer > 5*SYS_TICK_TicksPerSecondGet() )
      {
         WFConsolePrintStr("ARP unable to resolve the MAC address of remote side.", true);
         APPCTX.timer = SYS_TICK_Get();
      }
      return;
  }

  sprintf( (char *) g_ConsoleContext.txBuf,"    - RemoteNode MAC: %x %x %x %x %x %x",
           APPCTX.remoteSide.remoteMACAddr.v[0],
           APPCTX.remoteSide.remoteMACAddr.v[1],
           APPCTX.remoteSide.remoteMACAddr.v[2],
           APPCTX.remoteSide.remoteMACAddr.v[3],
           APPCTX.remoteSide.remoteMACAddr.v[4],
           APPCTX.remoteSide.remoteMACAddr.v[5]);

  WFConsolePrintStr( (char *) g_ConsoleContext.txBuf , true);
  
  if ( APPCTX.mProtocol == kIperfProtoUDP )
     APPCTX.state = kStateUdpTxOpen;
  else
     APPCTX.state = kStateTcpTxOpen;

}

static void
StateMachineUDPTxOpen(void)
{
    UDP_SOCKET_INFO sktInfo;
    
    if ( (APPCTX.udpSock = UDPOpenClient(IP_ADDRESS_TYPE_IPV4, APPCTX.mServerPort, (IP_MULTI_ADDRESS*)&APPCTX.remoteSide.remoteIP.remoteIPv4)) == INVALID_UDP_SOCKET )
    {
        /* error case */
        WFConsolePrintStr("Create UDP socket failed", true);
        APPCTX.state = kStateStandby;
        return;
    }

    UDPGetSocketInfo(APPCTX.udpSock, &sktInfo);
    APPCTX.localPort = sktInfo.localPort;

    WFConsolePrintStr("---------------------------------------------------------", true);
    sprintf( (char *) g_ConsoleContext.txBuf,
             "Client connecting to %u.%u.%u.%u, UDP port %u",
              APPCTX.remoteSide.remoteIP.remoteIPv4.v[0],
              APPCTX.remoteSide.remoteIP.remoteIPv4.v[1],
              APPCTX.remoteSide.remoteIP.remoteIPv4.v[2],
              APPCTX.remoteSide.remoteIP.remoteIPv4.v[3],
              APPCTX.mServerPort );

    WFConsolePrintStr( (char *) g_ConsoleContext.txBuf , true);

    APPCTX.state = kStateUdpTxDatagram;

    APPCTX.startTime = IPERF_GET_MSEC_TICK_COUNT();

     // Wait for a few seconds before first TCP tx, so we can resolve ARP.
    APPCTX.nextTxTime = APPCTX.startTime + APPCTX.mPktPeriod;

}

static void
StateMachineTCPTxOpen(void)
{

   if  ( (APPCTX.tcpClientSock = TCPOpenClient(IP_ADDRESS_TYPE_IPV4, APPCTX.mServerPort, &APPCTX.remoteSide.remoteIPaddress)) == INVALID_SOCKET )
   {
       /* error case */
        WFConsolePrintStr("Create TCP socket failed", true);
        APPCTX.state = kStateStandby;
        return;
    }

    APPCTX.localPort = LOCAL_PORT_START_NUMBER;

    WFConsolePrintStr("---------------------------------------------------------", true);
    sprintf( (char *) g_ConsoleContext.txBuf,
             "Client connecting to %u.%u.%u.%u, TCP port %u",
              APPCTX.remoteSide.remoteIP.remoteIPv4.v[0],
              APPCTX.remoteSide.remoteIP.remoteIPv4.v[1],
              APPCTX.remoteSide.remoteIP.remoteIPv4.v[2],
              APPCTX.remoteSide.remoteIP.remoteIPv4.v[3],
              APPCTX.mServerPort );

    WFConsolePrintStr( (char *) g_ConsoleContext.txBuf , true);

    APPCTX.state =  kStateTcpTxConnect;

    TCPAdjustFIFOSize(APPCTX.tcpClientSock, 10, 1,TCP_ADJUST_GIVE_REST_TO_TX);

    APPCTX.timer = SYS_TICK_Get();
}


static void
StateMachineTCPTxConnect(void)
{

 if ( IperfAppKillRequested() == true )
 {
     APPCTX.state = kStateTcpTxDone;
     return;
 }

  if( !TCPIsConnected(APPCTX.tcpClientSock) )
  {

    // Time out if too much time is spent in this state
    if(SYS_TICK_Get()- APPCTX.timer > 5*SYS_TICK_TicksPerSecondGet())
    {
       TCPDisconnect(APPCTX.tcpClientSock);
       APPCTX.tcpClientSock = INVALID_SOCKET;
       WFConsolePrintStr("TCP Client connection timeout", true);
       APPCTX.state = kStateTcpTxDone;
    }

    return;
  }

  /* reset the reset flag so we don't get a false positive */
  TCPWasReset(APPCTX.tcpClientSock);

  APPCTX.startTime = IPERF_GET_MSEC_TICK_COUNT();
  APPCTX.nextTxTime = APPCTX.startTime + APPCTX.mPktPeriod;
  APPCTX.state = kStateTcpTxSegment;
}



static void
GenericTxHeaderPreparation(uint8_t *pData, bool isTheLastTransmit)
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
        case kIperfProtoTCP: // TCP

            // Required by iperf.
            pClientHdr = (tClientHdr *) pData;

            // We borrow the same tIperfPktInfo structure to embed
            // some useful (non-standard iperf) meta info.
            // Unfortunately, the order has to be reversed.

            pPktInfo = (tIperfPktInfo *) (pClientHdr + 1);

            break;

        case kIperfProtoUDP: // UDP

            // Both are required by iperf.

            pPktInfo = (tIperfPktInfo *) pData;
            pClientHdr = (tClientHdr *) (pPktInfo + 1);

            break;
    }

    // Client header:
    // Needed for all UDP packets.
    // For TCP, only the first two segments need this info. However,
    // there seems to be no harm to put it to all segments though.

    pClientHdr->flags = HTOWFL( (uint32_t) 0);
    pClientHdr->numThreads = HTOWFL((uint32_t) 1);
    pClientHdr->mPort = HTOWFL((uint32_t) APPCTX.mServerPort);
    pClientHdr->bufferlen = HTOWFL( (uint32_t) 0);
    pClientHdr->mWinBand = HTOWFL(APPCTX.mTxRate);

    if ( APPCTX.mAmount != 0u )
    {
        pClientHdr->mAmount = HTOWFL(APPCTX.mAmount);
    }
    else
    {
        pClientHdr->mAmount = HTOWFL( - (long) (APPCTX.mDuration/10) );
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

    pPktInfo->id = HTOWFL(pPktInfo->id);

    currentTime = IPERF_GET_MSEC_TICK_COUNT();

    pPktInfo->tv_sec = HTOWFL(currentTime / SYS_TICK_TicksPerSecondGet());

    /* get the remainder of the ticks using modulus */
    tmp2 = ((APPCTX.stopTime - APPCTX.startTime)%SYS_TICK_TicksPerSecondGet());

    /* normalize  to uSecs */
    tmp2 =  tmp2*1000/SYS_TICK_TicksPerSecondGet(); /* Convert to mSec */
    tmp2 *= 1000;   /* 1000 uSecs per mSec */


    pPktInfo->tv_usec = HTOWFL( tmp2 );

    return;
}



static bool
GenericTxStart(void)
{
    uint32_t currentTime;
    bool iperfKilled;

    currentTime = IPERF_GET_MSEC_TICK_COUNT();

    if ( currentTime < (APPCTX.nextTxTime - kIperfTimingErrorMargin))
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
        // Although the last packet needs to be retransmitted kIperfUDPFinRetransmitCount times,
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

    if ( APPCTX.mProtocol == kIperfProtoTCP )
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
         WFConsolePrintStr("", true);
         WFConsolePrintStr("    Warning, TCP server disconnect detected", true);
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

    if ( APPCTX.mProtocol == kIperfProtoUDP )
    {

        APPCTX.remainingTxData = (APPCTX.mDatagramSize - MAX_BUFFER);

        if ( UDPPutArray(APPCTX.udpSock, g_bfr, MAX_BUFFER) != MAX_BUFFER )
        {
            WFConsolePrintStr("Socket send failed", true);
            APPCTX.errorCount++;
            return false;
        }

    }
    else
    {

        APPCTX.remainingTxData = (APPCTX.mMSS - MAX_BUFFER);

        if (( TCPPutArray(APPCTX.tcpClientSock, (BYTE*) g_bfr, MAX_BUFFER) != MAX_BUFFER ) && (!iperfKilled))
        {
            WFConsolePrintStr("Socket send failed", true);
            APPCTX.errorCount++;
            return false;
        }

    }

     return true;

}


/* This routine does a piecewise send, because the entire RAM buffer may not be available for putArray */
static void
TcpTxFillSegment(void)
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
static void
UdpTxFillDatagram(void)
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



static void
GenericTxEnd(void)
{

  if(  APPCTX.remainingTxData  > 0u )
  {
     /* unhandled error */
     WFConsolePrintStr("Socket send failed", true);
     APPCTX.errorCount++;
  }
  else
  {
     // send successful.

     if ( APPCTX.pktCount == 0u )
     {
        // first tx pkt

        SYS_MESSAGE("\n\r    Session started ...\n\r");


        sprintf( (char *) g_ConsoleContext.txBuf,
                 "    - Local  %u.%u.%u.%u port %u connected with",
                 pNetIf->MyIPAddr.v[0],
                 pNetIf->MyIPAddr.v[1],
                 pNetIf->MyIPAddr.v[2],
                 pNetIf->MyIPAddr.v[3],
                 APPCTX.localPort);

        WFConsolePrintStr( (char *) g_ConsoleContext.txBuf , true );

        sprintf( (char *) g_ConsoleContext.txBuf,
                 "    - Remote %u.%u.%u.%u port %u",
                 APPCTX.remoteSide.remoteIP.remoteIPv4.v[0],
                 APPCTX.remoteSide.remoteIP.remoteIPv4.v[1],
                 APPCTX.remoteSide.remoteIP.remoteIPv4.v[2],
                 APPCTX.remoteSide.remoteIP.remoteIPv4.v[3],
                 APPCTX.mServerPort );

        WFConsolePrintStr( (char *) g_ConsoleContext.txBuf , true );

        sprintf( (char *) g_ConsoleContext.txBuf, "    - Target rate = %ld bps, period = %ld ms",
                                        (unsigned long)APPCTX.mTxRate, 
                                        (unsigned long)(APPCTX.mPktPeriod*1000/SYS_TICK_TicksPerSecondGet()) );

        WFConsolePrintStr( (char *) g_ConsoleContext.txBuf , true);

     }

     APPCTX.pktId++;
     APPCTX.pktCount++;

     if ( APPCTX.mProtocol == kIperfProtoUDP )
     {
        APPCTX.totalLen += APPCTX.mDatagramSize;
     }
     else
     {

        APPCTX.totalLen += APPCTX.mMSS;
     }


  }

  APPCTX.lastPktId = APPCTX.pktId - 1;



  if ( IPERF_GET_MSEC_TICK_COUNT() > (APPCTX.lastCheckTime + APPCTX.mInterval - kIperfTimingErrorMargin) )
  {
        // Time to report statistics
        ReportBW_Jitter_Loss(kIntervalReport);

        //APPCTX.lastCheckPktCount = APPCTX.pktCount;
  }


  if ( APPCTX.isLastTransmit == true )
  {
      if ((APPCTX.mProtocol == (tIperfProto)kIperfProtoUDP) &&
          (++APPCTX.nAttempts < kIperfUDPFinRetransmitCount) ) {

          if ( APPCTX.nAttempts == 1u )
          {
              // So the normal pkt statistics is not mixed with the retransmited last pkt.
              APPCTX.stopTime = IPERF_GET_MSEC_TICK_COUNT();

              ReportBW_Jitter_Loss(kSubtotalReport);
              WFConsolePrintStr("    -----------------------------------------", true);
          }


          // Don't follow the same transmision rate during retransmit.
          APPCTX.mPktPeriod = kIperfUDPFinRetransmitPeriod;
      }
      else
      {
          APPCTX.state = (APPCTX.mProtocol == kIperfProtoUDP) ? kStateUdpTxDone : kStateTcpTxDone;
          APPCTX.stopTime = IPERF_GET_MSEC_TICK_COUNT();
      }
  }

}

static void
StateMachineTcpTxSegment(void)
{

    if ( GenericTxStart() == true )
    {
       TcpTxFillSegment();
       TCPFlush(APPCTX.tcpClientSock);
       GenericTxEnd();
    }
}

static void
StateMachineUdpTxDatagram(void)
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
        ReportBW_Jitter_Loss(kSessionReport);
        APPCTX.statusReported = 1;
    }

    APPCTX.state = kStateStandby;

    WFConsolePrintStr("    Tx done. Socket closed.", true);

    // Clear statistics
    ResetIperfCounters();

    //WFConsolePrintStr("    Back to standby mode.", true);
    WFConsolePrintStr("    Iperf completed.", true);

}


static void
StateMachineTcpTxDone(void)
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
    int8_t **argv;

	if ( WFConsoleIsConsoleMsgReceived() == true )
	{
       	argv = WFConsoleGetCmdLineArgv();
       	argc = WFConsoleGetCmdLineArgc();

		if ( memcmp(argv[0], "help", 4) == 0 )
		{
            WFConsolePrintStr("iperf\t\tsee documentation", true);
            WFConsolePrintStr("kill iperf\tstop the running iperf session", true);
		}
	    /* This check is for duplicate iperf app calls */
    	/* the only legitimate state that is ready for */
   		/* iperf server or client is standby           */
		else if ( APPCTX.state != (uint8_t)kStateStandby )
		{
        	if ( memcmp(argv[0], "iperf", 5) == 0 )
        	{
            	WFConsolePrintStr("", true);
            	WFConsolePrintStr("Error, Iperf session already started", true);
            	WFConsoleReleaseConsoleMsg();
        	}
		}
	}

    switch ( APPCTX.state )
    {
        case kStateStandby:

            StateMachineStandby();

            break;

        /********************/
        /* RX Client States */
        /********************/

        case kStateRxStart:

            StateMachineRxStart();

            break;

        case kStateUdpRx:

            StateMachineUdpRx();

            break;

        case kStateUdpRxDrain:

            StateMachineUdpRxDrain();

            break;


        case kStateUdpRxDone:

            StateMachineUdpRxDone();

            break;


        case kStateTcpRxListen:

            StateMachineTcpListen();

            break;

        case kStateTcpRx:

            StateMachineTcpRx();

            break;

        case kStateTcpRxDone:

            StateMachineTcpRxDone();

            break;

        case kStateRxDone:

            StateMachineRxDone();

            break;


       /********************/
       /* TX Client states */
       /********************/


        case kStateTxStart:

            StateMachineTxStart();

            break;


        case kStateTxArpResolve:

           StateMachineTxArpResolve();

           break;

        case kStateUdpTxOpen:

            StateMachineUDPTxOpen();

            break;

        case kStateTcpTxOpen:

            StateMachineTCPTxOpen();

            break;

        case kStateTcpTxConnect:

            StateMachineTCPTxConnect();

            break;

        case kStateTcpTxSegment:

            StateMachineTcpTxSegment();

            break;

        case kStateUdpTxDatagram:

            StateMachineUdpTxDatagram();

            break;

        case kStateTcpTxDone:

            StateMachineTcpTxDone();

            break;


        case kStateUdpTxDone:

            StateMachineUdpTxDone();

            break;

        }
}

#endif /* CMD_CONSOLE */
