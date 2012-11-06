/*******************************************************************************
  TCP Module private defs for Microchip TCP/IP Stack

  Summary:
    
  Description:
*******************************************************************************/

/*******************************************************************************
FileName:  tcp_private.h
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

#ifndef _TCP_PRIVATE_H_
#define _TCP_PRIVATE_H_


/****************************************************************************
  Section:
	Type Definitions
  ***************************************************************************/


#define TCP_CHECKSUM_OFFSET (16u)

/****************************************************************************
  Section:
	State Machine Variables
  ***************************************************************************/

// TCP States as defined by RFC 793
typedef enum
{
#if defined (TCPIP_STACK_USE_IPV6)
	TCP_GET_DNS_MODULE_IPV6,    // Special state for TCP client mode sockets
#endif
	TCP_GET_DNS_MODULE_IPV4,    // Special state for TCP client mode sockets
#if defined (TCPIP_STACK_USE_IPV6)
	TCP_DNS_RESOLVE_IPV6,	    // Special state for TCP client mode sockets
#endif
	TCP_DNS_RESOLVE_IPV4,	    // Special state for TCP client mode sockets
	TCP_GATEWAY_SEND_ARP,	    // Special state for TCP client mode sockets
	TCP_GATEWAY_GET_ARP,	    // Special state for TCP client mode sockets

    TCP_LISTEN,				    // Socket is listening for connections
    TCP_SYN_SENT,			    // A SYN has been sent, awaiting an SYN+ACK
    TCP_SYN_RECEIVED,		    // A SYN has been received, awaiting an ACK
    TCP_ESTABLISHED,		    // Socket is connected and connection is established
    TCP_FIN_WAIT_1,			    // FIN WAIT state 1
    TCP_FIN_WAIT_2,			    // FIN WAIT state 2
    TCP_CLOSING,			    // Socket is closing
//	TCP_TIME_WAIT, state is not implemented
	TCP_CLOSE_WAIT,			    // Waiting to close the socket
    TCP_LAST_ACK,			    // The final ACK has been sent
    TCP_CLOSED,			    	// Socket is idle and unallocated

    TCP_CLOSED_BUT_RESERVED	// Special state for TCP client mode sockets.  Socket is idle, but still allocated pending application closure of the handle.
} TCP_STATE;

typedef enum
{
	SSL_NONE = 0,			// No security is enabled
	SSL_HANDSHAKING,		// Handshake is progressing (no application data allowed)
	SSL_ESTABLISHED,		// Connection is established and secured
	SSL_CLOSED				// Connection has been closed (no applicaiton data is allowed)
} SSL_STATE;

/****************************************************************************
  Section:
	TCB Definitions
  ***************************************************************************/

// TCP Control Block (TCB) stub data storage.  Stubs are stored in local PIC RAM for speed.
typedef struct
{
	PTR_BASE bufferTxStart;		// First byte of TX buffer
	PTR_BASE bufferRxStart;		// First byte of RX buffer.  TX buffer ends 1 byte prior
	PTR_BASE bufferEnd;			// Last byte of RX buffer
	PTR_BASE txHead;			// Head pointer for TX
	PTR_BASE txTail;			// Tail pointer for TX
	PTR_BASE rxHead;			// Head pointer for RX
	PTR_BASE rxTail;			// Tail pointer for RX
    SYS_TICK eventTime;			// Packet retransmissions, state changes
	SYS_TICK eventTime2;		// Window updates, automatic transmission
	union
	{
		SYS_TICK delayedACKTime;	// Delayed Acknowledgement timer
		SYS_TICK closeWaitTime;		// TCP_CLOSE_WAIT timeout timer
	} OverlappedTimers;
    TCP_STATE smState;			// State of this socket
    struct
    {
	    unsigned char vUnackedKeepalives : 3;		// Count of how many keepalives have been sent with no response
        unsigned char bServer : 1;					// Socket should return to listening state when closed
		unsigned char bTimerEnabled	: 1;			// Timer is enabled
		unsigned char bTimer2Enabled : 1;			// Second timer is enabled
		unsigned char bDelayedACKTimerEnabled : 1;	// DelayedACK timer is enabled
		unsigned char bOneSegmentReceived : 1;		// A segment has been received
		unsigned char bHalfFullFlush : 1;			// Flush is for being half full
		unsigned char bTXASAP : 1;					// Transmit as soon as possible (for Flush)
		unsigned char bTXASAPWithoutTimerReset : 1;	// Transmit as soon as possible (for Flush), but do not reset retransmission timers
		unsigned char bTXFIN : 1;					// FIN needs to be transmitted
		unsigned char bSocketReset : 1;				// Socket has been reset (self-clearing semaphore)
		unsigned char bSSLHandshaking : 1;			// Socket is in an SSL handshake
		unsigned char bInitialized : 1;				// Future expansion
		unsigned char filler : 2;					// Future expansion
    } Flags;
	TCPIP_UINT16_VAL remoteHash;	// Consists of remoteIP, remotePort, localPort for connected sockets.  It is a localPort number only for listening server sockets.

    PTR_BASE sslTxHead;		// Position of data being written in next SSL application record
    						//   Also serves as cache of localSSLPort when smState = TCP_LISTENING
    PTR_BASE sslRxHead;		// Position of incoming data not yet handled by SSL
    uint8_t sslStubID;			// Which sslStub is associated with this connection
    uint8_t sslReqMessage;		// Currently requested SSL message
    //#endif

	uint8_t vMemoryMedium;		// Which memory medium the TCB is actually stored

    NET_CONFIG* pSktNet;   // which interface this socket is bound to
    IP_PACKET * pTxPkt;  // Transmit packet state
} TCB_STUB;

// Remainder of TCP Control Block data.
// The rest of the TCB is stored in Ethernet buffer RAM or elsewhere as defined by vMemoryMedium.
typedef struct
{
	SYS_TICK    retryInterval;			// How long to wait before retrying transmission
	uint32_t		MySEQ;					// Local sequence number
	uint32_t		RemoteSEQ;				// Remote sequence number
	PTR_BASE	txUnackedTail;			// TX tail pointer for data that is not yet acked
    TCPIP_UINT16_VAL	remotePort;				// Remote port number
    TCPIP_UINT16_VAL	localPort;				// Local port number
	uint16_t		remoteWindow;			// Remote window size
	uint16_t		wFutureDataSize;		// How much out-of-order data has been received
    uint32_t       remoteHost;
	int16_t		sHoleSize;				// Size of the hole, or -1 for none exists.  (0 indicates hole has just been filled)
    struct
    {
        unsigned char bFINSent : 1;		// A FIN has been sent
		unsigned char bSYNSent : 1;		// A SYN has been sent
		unsigned char bRXNoneACKed1 : 1;	// A duplicate ACK was likely received
		unsigned char bRXNoneACKed2 : 1;	// A second duplicate ACK was likely received
		unsigned char filler : 4;		// future use
    } flags;
	uint16_t		wRemoteMSS;				// Maximum Segment Size option advirtised by the remote node during initial handshaking
    TCPIP_UINT16_VAL	localSSLPort;			// Local SSL port number (for listening sockets)
    //#endif
	uint8_t		retryCount;				// Counter for transmission retries
	uint8_t		vSocketPurpose;			// Purpose of socket (as defined in tcpip_config.h)
} TCB;






#endif  // _TCP_PRIVATE_H_
