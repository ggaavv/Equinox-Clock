/*******************************************************************************
  Transmission Control Protocol (TCP) Communications Layer

  Summary:
    Module for Microchip TCP/IP Stack
    
  Description:
    -Provides reliable, handshaked transport of application stream 
     oriented data with flow control
    -Reference: RFC 793
*******************************************************************************/

/*******************************************************************************
FileName:   TCP.c
Copyright 2012 released Microchip Technology Inc.  All rights
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


#include "tcpip_private.h"
#include "tcp_private.h"

#if defined(TCPIP_STACK_USE_TCP)

/****************************************************************************
  Section:
	TCP Header Data Types
  ***************************************************************************/

#define FIN     (0x01)		// FIN Flag as defined in RFC
#define SYN     (0x02)		// SYN Flag as defined in RFC
#define RST     (0x04)		// Reset Flag as defined in RFC
#define PSH     (0x08)		// Push Flag as defined in RFC
#define ACK     (0x10)		// Acknowledge Flag as defined in RFC
#define URG     (0x20)		// Urgent Flag as defined in RFC

// TCP Header Data Structure
typedef struct
{
	uint16_t    SourcePort;		// Local port number
	uint16_t    DestPort;		// Remote port number
	uint32_t   SeqNumber;		// Local sequence number
	uint32_t   AckNumber;		// Acknowledging remote sequence number

	struct
	{
		unsigned char Reserved3      : 4;
		unsigned char Val            : 4;
	} DataOffset;			// Data offset flags nibble

	union
	{
		struct
		{
			unsigned char flagFIN    : 1;
			unsigned char flagSYN    : 1;
			unsigned char flagRST    : 1;
			unsigned char flagPSH    : 1;
			unsigned char flagACK    : 1;
			unsigned char flagURG    : 1;
			unsigned char Reserved2  : 2;
		} bits;
		uint8_t byte;
	} Flags;				// TCP Flags as defined in RFC

	uint16_t    Window;			// Local free RX buffer window
	uint16_t    Checksum;		// Data payload checksum
	uint16_t    UrgentPointer;	// Urgent pointer
} TCP_HEADER;

#define TCP_OPTIONS_END_OF_LIST     (0x00u)		// End of List TCP Option Flag
#define TCP_OPTIONS_NO_OP           (0x01u)		// No Op TCP Option
#define TCP_OPTIONS_MAX_SEG_SIZE    (0x02u)		// Maximum segment size TCP flag
typedef struct
{
	uint8_t        Kind;							// Type of option
	uint8_t        Length;							// Length
	TCPIP_UINT16_VAL    MaxSegSize;						// Maximum segment size
} TCP_OPTIONS;									// TCP Options data structure							

// Structure containing all the important elements of an incomming 
// SYN packet in order to establish a connection at a future time 
// if all sockets on the listening port are already connected to 
// someone
typedef struct 
{
    union
    {
        IP_ADDR sourceIPv4;
        IPV6_ADDR sourceIPv6;
    } sourceAddress;
    IPV6_ADDR   destAddr;
    MAC_ADDR    remoteMACAddr;     // Remote MAC address
	uint16_t		wSourcePort;	// Remote TCP port number that the response SYN needs to be sent to
	uint32_t		dwSourceSEQ;	// Remote TCP SEQuence number that must be ACKnowledged when we send our response SYN
	uint16_t		wDestPort;		// Local TCP port which the original SYN was destined for
	SYS_TICK    wTimestamp;		// Timer to expire old SYN packets that can't be serviced at all
    NET_CONFIG* pSynIf;         // interface for the incoming SYN packet
    IP_ADDRESS_TYPE addressType;
} TCP_SYN_QUEUE;


/****************************************************************************
  Section:
	TCB Definitions
  ***************************************************************************/


// use dynamic allocation instead of arrays
static TCB_STUB* TCBStubs = 0;
#if TCP_SYN_QUEUEING
    static TCP_SYN_QUEUE* SYNQueue = 0; // Array of saved incoming SYN requests that need to be serviced later
#endif

static TCB MyTCB;									// Currently loaded TCB
static TCP_SOCKET hCurrentTCP = INVALID_SOCKET;		// Current TCP socket
static int        tcpInitCount = 0;                 // initialization counter

static const void*  tcpHeapH = 0;                    // memory allocation handle
static unsigned int TcpSockets;                      // number of sockets in the current TCP configuration
static TCP_SOCKET hLastTCB = INVALID_SOCKET;

#if TCP_SYN_QUEUEING
    static int  TcpSynQueues = TCP_SYN_QUEUE_MAX_ENTRIES;           // number of syn queues in current TCP configuration
#endif

static int                  tcpTickPending = 0;        // TCP processing tick
static SystemTickHandle     tcpTimerHandle = 0;

	
	#if TCP_PIC_RAM_SIZE > 0u
		static uint8_t __attribute__((unused)) TCPBufferInPIC[TCP_PIC_RAM_SIZE] __attribute__((far));
		// The base address for TCP data in PIC RAM
        	#define TCP_PIC_RAM_BASE_ADDRESS			((PTR_BASE)&TCPBufferInPIC[0])
	#endif
    
/****************************************************************************
  Section:
	Function Prototypes
  ***************************************************************************/

static void TCPRAMCopy(NET_CONFIG* pNet, PTR_BASE wDest, uint8_t vDestType, PTR_BASE wSource, uint8_t vSourceType, uint16_t wLength);


static bool SendTCP(uint8_t vTCPFlags, uint8_t vSendFlags);
static void HandleTCPSeg(TCP_HEADER* h, uint16_t len);
static bool FindMatchingSocket(NET_CONFIG* pPktIf, TCP_HEADER* h, void * localIP, void * remoteIP, MAC_ADDR * remoteMACAddr, IP_ADDRESS_TYPE addressType);
static void SwapTCPHeader(TCP_HEADER* header);
static void CloseSocket(void);
static void SyncTCB(void);

static void TCPTmoHandler(SYS_TICK currSysTick);

static void TCPCleanup(void);



#if defined(TCPIP_STACK_CLIENT_MODE)
static TCP_PORT     TcpAllocateEphemeralPort(void);
static bool         TCPIsAvailablePort(TCP_PORT port);
#endif  // defined(TCPIP_STACK_CLIENT_MODE)

// Indicates if this packet is a retransmission (no reset) or a new packet (reset required)
#define SENDTCP_RESET_TIMERS	0x01
// Instead of transmitting normal data, a garbage octet is transmitted according to RFC 1122 section 4.2.3.6
#define SENDTCP_KEEP_ALIVE		0x02


/****************************************************************************
  Section:
	TCB Optimization Configuration
  ***************************************************************************/

#if defined(TCP_OPTIMIZE_FOR_SIZE)
	static TCB_STUB MyTCBStub;
	
	// Flushes MyTCBStub cache and loads up the specified TCB_STUB.
	// Does nothing on cache hit.
	static void SyncTCBStub(TCP_SOCKET hTCP)
	{
		if(hCurrentTCP == hTCP)
		{
			if(TCBStubs[hTCP].pSktNet != 0)
				return;
		}
	
		if(hCurrentTCP != INVALID_SOCKET)
		{
			// Save the current TCB stub
			memcpy((void*)&TCBStubs[hCurrentTCP], (void*)&MyTCBStub, sizeof(MyTCBStub));
		}
	
		hCurrentTCP = hTCP;
	
		if(hTCP == INVALID_SOCKET)
			return;
	
		// Load up the new TCB stub
		memcpy((void*)&MyTCBStub, (void*)&TCBStubs[hTCP], sizeof(MyTCBStub));
	}
#else
	// Flushes MyTCBStub cache and loads up the specified TCB_STUB.
	// Does nothing on cache hit.
	#define SyncTCBStub(a)	hCurrentTCP = (a)
	// Alias to current TCP stub.
	#define MyTCBStub		TCBStubs[hCurrentTCP]
#endif



// Flushes MyTCB cache and loads up the specified TCB.
// Does nothing on cache hit.
// NOTE: the TCPRAMCopy copy works for UNBOUND sockets ONLY for vMemoryMedium == TCP_PIC_RAM or TCP_PIC_DRAM when no hMac is needed! 
static void SyncTCB(void)
{
	if(hLastTCB == hCurrentTCP)
		return;

	if( hLastTCB != INVALID_SOCKET )
	{
		// Save the current TCB
		TCPRAMCopy(TCBStubs[hLastTCB].pSktNet, TCBStubs[hLastTCB].bufferTxStart - sizeof(MyTCB), TCBStubs[hLastTCB].vMemoryMedium, (PTR_BASE)&MyTCB, TCP_PIC_RAM, sizeof(MyTCB));
	}

	// Load up the new TCB
	hLastTCB = hCurrentTCP;
    TCPRAMCopy(MyTCBStub.pSktNet, (PTR_BASE)&MyTCB, TCP_PIC_RAM, MyTCBStub.bufferTxStart - sizeof(MyTCB), MyTCBStub.vMemoryMedium, sizeof(MyTCB));
}

/*static __inline__*/static  void /*__attribute__((always_inline))*/ _TcpSocketBind(TCB_STUB* pSkt, NET_CONFIG* pNet)
{
    pSkt->pSktNet = pNet;
    pSkt->pTxPkt->netIf = pNet;
}

/*static __inline__*/static  bool /*__attribute__((always_inline))*/ _TcpSocketChk(TCP_SOCKET hTCP)
{
    return (hTCP >= 0 && hTCP < TcpSockets);
}

/*****************************************************************************
  Function:
	bool TCPInit(const TCPIP_STACK_MODULE_CTRL* const stackInit, TCP_MODULE_CONFIG* pTcpInit)

  Summary:
	Initializes the TCP module.

  Description:
	Initializes the TCP module.  This function sets up the TCP buffers
	in memory and initializes each socket to the CLOSED state.  If
	insufficient memory was allocated for the TCP sockets, the function
	will call the TCPIPError() error function that can be captured by the debugger
    and will return false.

  Precondition:
	None

  Parameters:
    stackInit   - pointer to stack initialization data; contains heap, interfaces, etc

    pTcpInit    - pointer to a TCP initialization structure containing:
                    - nSockets:     number of sockets to be created

                    - pTcpSktInit:  pointer to an initialization structure
                                    if !NULL, it's an array of nSockets initialization structures 


    NOTE: if pTcpSktInit != 0, then it must contain at least nSockets initialization structures
          Excess initialization structures are ingored.

          If static configuration is enabled (TCP_DYNAMIC_CONFIGURATION is NOT defined)
          then pTcpSktInit is a pointer to a TCPSocketInitializer[] array.

          If pTcpSktInit == 0, then default values will be used
    

  Returns:
  	true if initialization succeeded
    false otherwise
  	
  Remarks:
    If pTcpInit->pTcpSktInit != 0 then each socket is initialized as requested by each entry
    in the SocketInitializer.
    Else the default TCP_SOCKET_DEFAULT_XXX parameters  are used. 
    
  ***************************************************************************/
bool TCPInit(const TCPIP_STACK_MODULE_CTRL* const stackInit, const TCP_MODULE_CONFIG* const pTcpInit)
{
	int      sktIx;
	uint16_t wTXSize, wRXSize;
	uint8_t  vMedium, vPurpose;
    int      nSockets;
	PTR_BASE ptrBaseAddress = 0;
#if TCP_ETH_RAM_SIZE > 0
	uint16_t wCurrentETHAddress = TCP_ETH_RAM_BASE_ADDRESS;
#endif
#if TCP_PIC_RAM_SIZE > 0
	PTR_BASE ptrCurrentPICAddress = TCP_PIC_RAM_BASE_ADDRESS;
#endif
#if TCP_SPI_RAM_SIZE > 0
	uint16_t wCurrentSPIAddress = TCP_SPI_RAM_BASE_ADDRESS;
#endif
    bool initFail = false;

    
    if(stackInit->stackAction == TCPIP_STACK_ACTION_IF_UP)
    {   // interface start up
        return true;    // do not store per interface data
    }

    // stack start up
    if(tcpInitCount != 0)
    {   // initialize just once
        tcpInitCount++;
        return true;
    }
    
    if(stackInit->memH == 0)
    {
        SYS_ERROR(SYS_ERROR_ERROR, "TCP NULL dynamic allocation handle");
        return false;
    }
    tcpHeapH = stackInit->memH;

    nSockets = pTcpInit->nSockets?pTcpInit->nSockets:TCP_MAX_SOCKETS;
    TCBStubs = (TCB_STUB*)TCPIP_HEAP_Malloc(tcpHeapH, sizeof(TCB_STUB) * nSockets);
    if(TCBStubs == 0)
    {
        SYS_ERROR(SYS_ERROR_ERROR, " TCP Dynamic allocation failed");
        return false;
    }
    

	// Mark all SYN Queue entries as invalid by zeroing the memory
#if TCP_SYN_QUEUEING
    SYNQueue = 0;
    if(TcpSynQueues)
    {
        SYNQueue = (TCP_SYN_QUEUE*)TCPIP_HEAP_Malloc(tcpHeapH, sizeof(*SYNQueue) * TcpSynQueues);
        if(SYNQueue == 0)
        {
            initFail = true;
        }
        else
        {
            memset((void*)SYNQueue, 0x00, sizeof(*SYNQueue) * TcpSynQueues);
        }
    }
        
#endif  // TCP_SYN_QUEUEING
	
	// Allocate all socket FIFO addresses
	for(sktIx = 0; sktIx < nSockets && initFail == false; sktIx++)
	{
		// Generate all needed sockets of each type (TCP_PURPOSE_*)
		SyncTCBStub(sktIx);
        
        // default initialization
        vPurpose = TCP_SOCKET_DEFAULT_PURPOSE;
        vMedium = TCP_SOCKET_DEFAULT_MEMORY_MEDIUM;
        wTXSize = TCP_SOCKET_DEFAULT_TX_SIZE;
        wRXSize = TCP_SOCKET_DEFAULT_RX_SIZE;
        
        // static mode override
#if !defined(TCP_DYNAMIC_CONFIGURATION)
        if(pTcpInit->pTcpSktInit != 0)
        {
            TCP_SOCKET_CONF* SocketInitializer;
            SocketInitializer = (TCP_SOCKET_CONF*)pTcpInit->pTcpSktInit;

            vPurpose = SocketInitializer[sktIx].vSocketPurpose;
            vMedium = SocketInitializer[sktIx].vMemoryMedium;
            wTXSize = SocketInitializer[sktIx].wTXBufferSize;
            wRXSize = SocketInitializer[sktIx].wRXBufferSize;
        }
#endif  // !defined(TCP_DYNAMIC_CONFIGURATION)

		switch(vMedium)
		{
			#if TCP_ETH_RAM_SIZE > 0
			case TCP_ETH_RAM:
				ptrBaseAddress = wCurrentETHAddress;
				wCurrentETHAddress += sizeof(TCB) + wTXSize+1 + wRXSize+1;
				// Do a sanity check to ensure that we aren't going to use memory that hasn't been allocated to us.
				// If your code locks up right here, it means you've incorrectly allocated your TCP socket buffers in tcpip_config.h.  See the TCP memory allocation section.  More RAM needs to be allocated to the base memory mediums, or the individual sockets TX and RX FIFOS and socket quantiy needs to be shrunken.
				if(wCurrentETHAddress > TCP_ETH_RAM_BASE_ADDRESS + TCP_ETH_RAM_SIZE)
                {
                    SYS_ERROR(SYS_ERROR_ERROR, "ETH RAM allocation failed");
                    initFail = 1;
                }
				break;
			#endif
				
			#if TCP_PIC_RAM_SIZE > 0
			case TCP_PIC_RAM:
				ptrBaseAddress = ptrCurrentPICAddress;
				ptrCurrentPICAddress += sizeof(TCB) + wTXSize+1 + wRXSize+1;
				// Do a sanity check to ensure that we aren't going to use memory that hasn't been allocated to us.
				// If your code locks up right here, it means you've incorrectly allocated your TCP socket buffers in tcpip_config.h.  See the TCP memory allocation section.  More RAM needs to be allocated to the base memory mediums, or the individual sockets TX and RX FIFOS and socket quantiy needs to be shrunken.
				if(ptrCurrentPICAddress > TCP_PIC_RAM_BASE_ADDRESS + TCP_PIC_RAM_SIZE)
                {
                    SYS_ERROR(SYS_ERROR_ERROR, "PIC RAM allocation failed");
                    initFail = 1;
                }
				break;
			#endif
				
			#if TCP_SPI_RAM_SIZE > 0
			case TCP_SPI_RAM:
				ptrBaseAddress = wCurrentSPIAddress;
				wCurrentSPIAddress += sizeof(TCB) + wTXSize+1 + wRXSize+1;
				// Do a sanity check to ensure that we aren't going to use memory that hasn't been allocated to us.
				// If your code locks up right here, it means you've incorrectly allocated your TCP socket buffers in tcpip_config.h.  See the TCP memory allocation section.  More RAM needs to be allocated to the base memory mediums, or the individual sockets TX and RX FIFOS and socket quantiy needs to be shrunken.
				if(wCurrentSPIAddress > TCP_SPI_RAM_BASE_ADDRESS + TCP_SPI_RAM_SIZE)
                {
                    SYS_ERROR(SYS_ERROR_ERROR, "SPI RAM allocation failed");
                    initFail = 1;
                }
				break;
			#endif
			
			case TCP_PIC_DRAM:
				ptrBaseAddress = (PTR_BASE)TCPIP_HEAP_Malloc(tcpHeapH, sizeof(TCB) + wTXSize+1 + wRXSize+1);
                if(ptrBaseAddress == 0)
                {
                    SYS_ERROR(SYS_ERROR_ERROR, "Dynamic allocation failed");
                    initFail = 1;
                }
				break;
                
			default:
                SYS_ERROR(SYS_ERROR_ERROR, "Undefined allocation medium");
                initFail = 1; // Go fix your tcpip_config.h TCP memory allocations.
				break;
		}

        if(!initFail)
        {
            MyTCBStub.vMemoryMedium = vMedium;
            MyTCBStub.bufferTxStart	= ptrBaseAddress + sizeof(TCB);
            MyTCBStub.bufferRxStart	= MyTCBStub.bufferTxStart + wTXSize + 1;
            MyTCBStub.bufferEnd		= MyTCBStub.bufferRxStart + wRXSize;
            MyTCBStub.smState		= TCP_CLOSED;
            MyTCBStub.Flags.bServer	= false;
            MyTCBStub.pTxPkt        = NULL;
#if defined(TCPIP_STACK_USE_SSL)
            MyTCBStub.sslStubID = SSL_INVALID_ID;
#endif		
            MyTCBStub.pSktNet = 0;

            SyncTCB();
            MyTCB.vSocketPurpose = vPurpose;
            CloseSocket();
        }
        else
        {   // do not advance the socket number if error
            break;
        }
	}

    TcpSockets = sktIx;
    if(!initFail)
    {
        tcpTimerHandle = SYS_TICK_TimerCreate(TCPTmoHandler);
        if(tcpTimerHandle)
        {
            tcpTickPending = 0;
            SYS_TICK_TimerSetRate(tcpTimerHandle, (SYS_TICK_ResolutionGet()+TCP_TICKS_PER_SECOND-1)/TCP_TICKS_PER_SECOND);
        }
        else
        {   // cannot create the TCP timer
            initFail = 1;
        }

    }
    
    if(initFail)
    {
        TCPCleanup();
        return false;
    }
    
    tcpInitCount++;

    return true;
}

/*****************************************************************************
  Function:
	void TCPDeInit(const TCPIP_STACK_MODULE_CTRL* const stackInit)

  Summary:
	De-Initializes the TCP module.

  Description:
	De-Initializes the TCP module.
    This function initializes each socket to the CLOSED state.
    If dynamic memory was allocated for the TCP sockets, the function
	will deallocate it.

  Precondition:
	TCPInit() should have been called

  Parameters:
    stackInit   - pointer to stack initialization data; contains heap, interfaces, etc
                  and interface that's going down

  Returns:
    None
    
  Remarks:
  ***************************************************************************/
void TCPDeInit(const TCPIP_STACK_MODULE_CTRL* const stackInit)
{
    int ix;


    // interface is going down
    for(ix = 0; ix < TcpSockets; ix++)
    {
        SyncTCBStub(ix);

        if(MyTCBStub.pSktNet == stackInit->pNetIf)  
        {
            TCPClose(ix);
        }
    }


    if(stackInit->stackAction == TCPIP_STACK_ACTION_DEINIT)
    {   // stack shut down
        if(tcpInitCount > 0)
        {   // we're up and running

            if(--tcpInitCount == 0)
            {   // all closed
                // release resources
                TCPCleanup();
            }
        }
    }
	
}

// performs the clean-up of resources
static void TCPCleanup(void)
{
    int ix;
    
	for(ix = 0; ix < TcpSockets; ix++)
    {
        SyncTCBStub(ix);

        if(MyTCBStub.vMemoryMedium == TCP_PIC_DRAM )
        {
            TCPIP_HEAP_Free(tcpHeapH, (void*)(MyTCBStub.bufferTxStart	- sizeof(TCB)));
        }
    }
    

#if TCP_SYN_QUEUEING
    if(SYNQueue)
    {
        TCPIP_HEAP_Free(tcpHeapH, SYNQueue);
        SYNQueue = 0;
    }
#endif  // TCP_SYN_QUEUEING

    if(TCBStubs)
    {
        TCPIP_HEAP_Free(tcpHeapH, TCBStubs);
        TCBStubs = 0;
    }

    TcpSockets = 0;
		
	hLastTCB = INVALID_SOCKET;

	if(tcpTimerHandle)
    {
        SYS_TICK_TimerDelete(tcpTimerHandle);
        tcpTimerHandle = 0;
		tcpTickPending = 0;
    }

}

static void TCPTmoHandler(SYS_TICK currSysTick)
{
    tcpTickPending++;
}
    

// returns true if service needed
// called by the stack manager
bool TCPTaskPending(void)
{
    return tcpTickPending != 0;
}

/*****************************************************************************
  Function:
	 TCP_SOCKET TCPOpenServer(IP_ADDRESS_TYPE addType, TCP_PORT localPort,  IP_MULTI_ADDRESS* localAddress)

  Summary:
	Opens a TCP socket as a server.
	
  Description:
	Provides a unified method for opening TCP server sockets. 

	Sockets are statically/dynamically allocated on boot, and can be claimed with this
    \function and freed using TCPClose.

  Precondition:
    TCP is initialized.

  Parameters:
    IP_ADDRESS_TYPE addType			-	The type of address being used. Example: IP_ADDRESS_TYPE_IPV4.
    TCP_PORT localPort				-	TCP port to listen on or connect to:
										* Server sockets &#45; the local TCP port on which to
										  isten for connections.
    IP_MULTI_ADDRESS* localAddress	-	Local address to use.
	
  Returns:
 	INVALID_SOCKET -  No sockets of the specified type were available to be
                      opened.
    Otherwise -       A TCP_SOCKET handle. Save this handle and use it when
                      calling all other TCP APIs. 
 ***************************************************************************/
TCP_SOCKET TCPOpenServer(IP_ADDRESS_TYPE addType, TCP_PORT localPort,  IP_MULTI_ADDRESS* localAddress)
{
    TCP_SOCKET  skt;
    NET_CONFIG* pDefIf;

    if(addType == IP_ADDRESS_TYPE_IPV4 && localAddress != 0 && localAddress->v4Add.Val != 0)
    {
        pDefIf = _TCPIPStackIpAddToNet(&localAddress->v4Add, false);
        if(pDefIf == 0)
        {    // no such interface
            return INVALID_UDP_SOCKET;
        }
    }
    else
    {
        pDefIf = (NET_CONFIG*)TCPIP_STACK_GetDefaultNet();
    }


    skt = TCPOpen(0, TCP_OPEN_SERVER, localPort, TCP_SOCKET_DEFAULT_PURPOSE);
    if(skt != INVALID_SOCKET)
    {
        SyncTCBStub(skt);
        _TcpSocketBind(&MyTCBStub, pDefIf);
    }

    return skt;

}

/*****************************************************************************
  Function:
	 TCP_SOCKET TCPOpenClient(IP_ADDRESS_TYPE addType, TCP_PORT remotePort, IP_MULTI_ADDRESS* remoteAddress)

  Summary:
	Opens a TCP socket as a client.
	
  Description:
	Provides a unified method for opening TCP client sockets. 

	Sockets are statically/dynamically allocated on boot, and can be claimed with this
    \function and freed using TCPDisconnect or TCPClose.

  Precondition:
    TCP is initialized.

  Parameters:
    IP_ADDRESS_TYPE addType			-	The type of address being used. Example: IP_ADDRESS_TYPE_IPV4.
    TCP_PORT remotePort				-	TCP port to listen on or connect to:
										* Client sockets &#45; the remote TCP port to which a
                         				  connection should be made. The local port for client
                         				  sockets will be automatically picked by the TCP
                         				  module.
    IP_MULTI_ADDRESS* remoteAddress	-	The remote address to be used.
	
  Returns:
 	INVALID_SOCKET -  No sockets of the specified type were available to be
                      opened.
    Otherwise -       A TCP_SOCKET handle. Save this handle and use it when
                      calling all other TCP APIs. 
 ***************************************************************************/
TCP_SOCKET TCPOpenClient(IP_ADDRESS_TYPE addType, TCP_PORT remotePort, IP_MULTI_ADDRESS* remoteAddress)
{
#if defined (TCPIP_STACK_USE_IPV6)
    if(addType == IP_ADDRESS_TYPE_IPV6)
    {
        return TCPOpen((uint32_t)&remoteAddress->v6Add, TCP_OPEN_IPV6_ADDRESS, remotePort, TCP_SOCKET_DEFAULT_PURPOSE);
    }
#endif

    IPV4_ADDR v4Add;
    v4Add.Val = remoteAddress?remoteAddress->v4Add.Val:0;
    return TCPOpen(v4Add.Val, TCP_OPEN_IP_ADDRESS, remotePort, TCP_SOCKET_DEFAULT_PURPOSE); 

}


/****************************************************************************
  Section:
	Connection Management Functions
  ***************************************************************************/


/*****************************************************************************
  Function:
	TCP_SOCKET TCPOpen(uint32_t dwRemoteHost, uint8_t vRemoteHostType, uint16_t wPort, uint8_t vSocketPurpose)
    
  Summary:
    Opens a TCP socket for listening or as a client.

  Description:
    Provides a unified method for opening TCP sockets. This function can
    open both client and server sockets. For client sockets, it can accept
    a host name string to query in DNS, an IP address as a string, an IP
    address in binary form, or a previously resolved NODE_INFO structure
    containing the remote IP address and associated MAC address. When a
    host name or IP address only is provided, the TCP module will
    internally perform the necessary DNS and/or ARP resolution steps before
    reporting that the TCP socket is connected (via a call to
    TCPISConnected returning true). Server sockets ignore this destination
    parameter and listen only on the indicated port.
    
    The vSocketPurpose field allows sockets to be opened with varying
    buffer size parameters and memory storage mediums. This field
    corresponds to the optional pre-defined sockets allocated in the
    TCPSocketInitializer[] array in tcpip_config.h. The tcpip_config.h file
    can be edited using the TCP/IP Configuration Wizard.
    
    Sockets are statically/dynamically allocated on boot, and can be claimed with this
    \function and freed using TCPDisconnect or TCPClose (for client
    sockets). Server sockets can be freed using TCPClose only (calls to
    TCPDisconnect will return server sockets to the listening state,
    allowing reuse).

  Conditions:
    TCP is initialized.

  Input:
    dwRemoteHost -     For client sockets only. Provide a pointer to a
                       null\-terminated string of the remote host name (ex\:
                       "www.microchip.com" or "192.168.1.123"), a literal
                       destination IP address (ex\: 0x7B01A8C0 or an IP_ADDR
                       data type), or a pointer to a NODE_INFO structure
                       with the remote IP address and remote node or gateway
                       MAC address specified. If a string is provided, note
                       that it must be statically allocated in memory and
                       cannot be modified or deallocated until
                       TCPIsConnected returns true.<p />This parameter is
                       ignored for server sockets.
    vRemoteHostType -  Any one of the following flags to identify the
                       meaning of the dwRemoteHost parameter\:
                       * TCP_OPEN_SERVER &#45; Open a server socket and
                         ignore the dwRemoteHost parameter.
                       * TCP_OPEN_RAM_HOST &#45; Open a client socket and
                         connect it to a remote host who's name is stored as a
                         null terminated string in a RAM array. Ex\:
                         "www.microchip.com" or "192.168.0.123" (uint8_t&#42;
                         type)
                       * TCP_OPEN_ROM_HOST &#45; Open a client socket and
                         connect it to a remote host who's name is stored as a
                         null terminated string in a literal string or const
                         array. Ex\: "www.microchip.com" or "192.168.0.123"
                         (const uint8_t&#42; type)
                       * TCP_OPEN_IP_ADDRESS &#45; Open a client socket and
                         connect it to a remote IP address. Ex\: 0x7B01A8C0
                         for 192.168.1.123 (uint32_t type). Note that the byte
                         ordering is big endian.
                       * TCP_OPEN_NODE_INFO &#45; Open a client socket and
                         connect it to a remote IP and MAC addresses pair
                         stored in a NODE_INFO structure. dwRemoteHost must be
                         a pointer to the NODE_INFO structure. This option is
                         provided for backwards compatibility with
                         applications built against prior stack versions that
                         only implemented the TCPConnect() function. It can
                         also be used to skip DNS and ARP resolution steps if
                         connecting to a remote node which you've already
                         connected to and have cached addresses for.
    wPort -            TCP port to listen on or connect to\:
                       * Client sockets &#45; the remote TCP port to which a
                         connection should be made. The local port for client
                         sockets will be automatically picked by the TCP
                         module.
                       * Server sockets &#45; the local TCP port on which to
                         listen for connections.
    vSocketPurpose -   Any of the TCP_PURPOSE_* constants defined in
                       tcpip_config.h or the TCPIPConfig utility (see the optional
                       TCPSocketInitializer[] array).

  Return Values:
    INVALID_SOCKET -  No sockets of the specified type were available to be
                      opened.
    Otherwise -       A TCP_SOCKET handle. Save this handle and use it when
                      calling all other TCP APIs.

  Remarks:
    This function replaces the old TCPConnect and TCPListen functions.
    
    If TCP_OPEN_RAM_HOST or TCP_OPEN_ROM_HOST are used for the destination
    type, the DNS client module must also be enabled (TCPIP_STACK_USE_DNS must be
    defined in tcpip_config.h).

  Example:
    \ \ 
    <code>
    // Open a server socket
    skt = TCPOpen(NULL, TCP_OPEN_SERVER, HTTP_PORT, TCP_PURPOSE_HTTP_SERVER);
    
    // Open a client socket to www.microchip.com
    // The double cast here prevents compiler warnings
    skt = TCPOpen((uint32_t)(PTR_BASE)"www.microchip.com",
                    TCP_OPEN_ROM_HOST, 80, TCP_PURPOSE_DEFAULT);
    
    // Reopen a client socket without repeating DNS or ARP
    TCP_SOCKET_INFO cache = TCPGetSocketInfo(skt);  // Call with the old socket
    skt = TCPOpen((uint32_t)(PTR_BASE)&amp;cache.remote, TCP_OPEN_NODE_INFO,
                    cache.remotePort.Val, TCP_PURPOSE_DEFAULT);
    </code>                                                    
  *****************************************************************************/
TCP_SOCKET TCPOpen(uint32_t dwRemoteHost, uint8_t vRemoteHostType, uint16_t wPort, uint8_t vSocketPurpose)
{
	TCP_SOCKET hTCP;

	// Find an available socket that matches the specified socket type
	for(hTCP = 0; hTCP < TcpSockets; hTCP++)
	{
		SyncTCBStub(hTCP);

		// Sockets that are in use will be in a non-closed state
		if(MyTCBStub.smState != TCP_CLOSED)
			continue;

		SyncTCB();

		// See if this socket matches the desired type
		if(MyTCB.vSocketPurpose != vSocketPurpose && MyTCB.vSocketPurpose != TCP_PURPOSE_ANY)
			continue;

		// Start out assuming worst case Maximum Segment Size (changes when MSS 
		// option is received from remote node)
		MyTCB.wRemoteMSS = 536;

		// See if this is a server socket
		if(vRemoteHostType == TCP_OPEN_SERVER)
		{
            // unbound the socket
            // the server should accept a request on any incoming
            // interface unless is BOUND! 
            MyTCBStub.pSktNet = 0;

            MyTCBStub.pTxPkt = TCPAllocateTxPacketStruct (MyTCBStub.pSktNet, IP_ADDRESS_TYPE_IPV4);
    
            if (MyTCBStub.pTxPkt == NULL)
                return INVALID_SOCKET;
        
			MyTCB.localPort.Val = wPort;
			MyTCBStub.Flags.bServer = true;
			MyTCBStub.smState = TCP_LISTEN;
			MyTCBStub.remoteHash.Val = wPort;
			#if defined(TCPIP_STACK_USE_SSL_SERVER)
			MyTCB.localSSLPort.Val = 0;
			#endif
		}
		// Handle all the client mode socket types
		else
		{
			#if defined(TCPIP_STACK_CLIENT_MODE)
			{
                TCP_PORT clientPort;     
				clientPort = TcpAllocateEphemeralPort();
				if(clientPort  == 0)
                {
                    return INVALID_SOCKET;
                }
                    
				// Restore hCurrentTCP by using SyncTCBStub, because TcpAllocateEphemeralPort modified it 
				SyncTCBStub(hTCP);
                SyncTCB();
                // bind the socket on a default interface
                MyTCBStub.pSktNet = (NET_CONFIG*)TCPIP_STACK_GetDefaultNet();

                if (vRemoteHostType == TCP_OPEN_IPV6_ADDRESS)
                    MyTCBStub.pTxPkt = TCPAllocateTxPacketStruct (MyTCBStub.pSktNet, IP_ADDRESS_TYPE_IPV6);
                else
                    MyTCBStub.pTxPkt = TCPAllocateTxPacketStruct (MyTCBStub.pSktNet, IP_ADDRESS_TYPE_IPV4);
        
                if (MyTCBStub.pTxPkt == NULL)
                    return INVALID_SOCKET;
        
				// Set the non-zero TCB fields
				MyTCB.localPort.Val = clientPort;
				MyTCB.remotePort.Val = wPort;
	
				// Flag to start the DNS, ARP, SYN processes
				MyTCBStub.eventTime = SYS_TICK_Get();
				MyTCBStub.Flags.bTimerEnabled = 1;
	
				switch(vRemoteHostType)
				{
					#if defined(TCPIP_STACK_USE_DNS)
					case TCP_OPEN_RAM_HOST:
					case TCP_OPEN_ROM_HOST:
						MyTCB.remoteHost = dwRemoteHost;
						MyTCBStub.smState = TCP_GET_DNS_MODULE_IPV4;
						break;
					#endif
		
					case TCP_OPEN_IP_ADDRESS:
						// dwRemoteHost is a literal IP address.  This 
						// doesn't need DNS and can skip directly to the 
						// Gateway ARPing step.
						MyTCBStub.remoteHash.Val = (((TCPIP_UINT32_VAL)dwRemoteHost).w[1]+((TCPIP_UINT32_VAL)dwRemoteHost).w[0] + wPort) ^ MyTCB.localPort.Val;
						TCPIP_IPV4_SetDestAddress(MyTCBStub.pTxPkt, dwRemoteHost);
                        TCPIP_IP_SetPacketIPProtocol (MyTCBStub.pTxPkt, IP_ADDRESS_TYPE_IPV4);
						MyTCB.retryCount = 0;
						MyTCB.retryInterval = (SYS_TICK_TicksPerSecondGet()/4);
						MyTCBStub.smState = TCP_GATEWAY_SEND_ARP;
						break;
#if defined (TCPIP_STACK_USE_IPV6)
					case TCP_OPEN_IPV6_ADDRESS:
						// dwRemoteHost is a pointer to an IPv6 address.  This 
						// doesn't need DNS.
						MyTCBStub.remoteHash.Val = TCPIP_IPV6_GetHash((IPV6_ADDR *)dwRemoteHost,wPort,MyTCB.localPort.Val);
  						TCPIP_IPV6_SetDestAddress(MyTCBStub.pTxPkt, (IPV6_ADDR *)dwRemoteHost);
                        TCPIP_IP_SetPacketIPProtocol (MyTCBStub.pTxPkt, IP_ADDRESS_TYPE_IPV6);
						MyTCB.retryCount = 0;
						MyTCB.retryInterval = (SYS_TICK_TicksPerSecondGet()/4);
						MyTCBStub.smState = TCP_SYN_SENT;
						SendTCP(SYN, SENDTCP_RESET_TIMERS);
						break;
#endif		
					case TCP_OPEN_NODE_INFO:
						MyTCBStub.remoteHash.Val = (((NODE_INFO*)(PTR_BASE)dwRemoteHost)->IPAddr.w[1]+((NODE_INFO*)(PTR_BASE)dwRemoteHost)->IPAddr.w[0] + wPort) ^ MyTCB.localPort.Val;
						TCPIP_IPV4_SetDestAddress(MyTCBStub.pTxPkt, ((NODE_INFO *)(PTR_BASE)dwRemoteHost)->IPAddr.Val);
                        TCPIP_IP_SetPacketIPProtocol (MyTCBStub.pTxPkt, IP_ADDRESS_TYPE_IPV4);
						memcpy((void*)(uint8_t*)&MyTCBStub.pTxPkt->remoteMACAddr, (void *)&(((NODE_INFO *)(PTR_BASE)dwRemoteHost)->MACAddr), sizeof(MAC_ADDR));
						MyTCBStub.smState = TCP_SYN_SENT;
						SendTCP(SYN, SENDTCP_RESET_TIMERS);
						break;
				}
			}		
			#else
			{
				return INVALID_SOCKET;
			}	
			#endif
		}

        if (MyTCBStub.pTxPkt == NULL)
        {
            MyTCBStub.smState = TCP_CLOSED;
            return INVALID_SOCKET;
        }

		return hTCP;		
	}

	// If there is no socket available, return error.
	return INVALID_SOCKET;
}

IP_PACKET * TCPAllocateTxPacketStruct (NET_CONFIG * pConfig, IP_ADDRESS_TYPE addressType)
{
    IP_PACKET * pkt;

    pkt = TCPIP_IP_AllocateTxPacket (MyTCBStub.pSktNet, addressType, 0, 0);

    if (pkt == NULL)
        return NULL;

    if (!TCPIP_IP_PutUpperLayerHeader (pkt, NULL, sizeof (TCP_HEADER), IP_PROT_TCP, TCP_CHECKSUM_OFFSET))
    {
        TCPIP_IP_FreePacket (pkt);
        return NULL;
    }

    return pkt;
}

/*****************************************************************************
  Function:
	bool TCPWasReset(TCP_SOCKET hTCP)

  Summary:
	Self-clearing semaphore inidicating socket reset.

  Description:
	This function is a self-clearing semaphore indicating whether or not
	a socket has been disconnected since the previous call.  This function
	works for all possible disconnections: a call to TCPDisconnect, a FIN 
	from the remote node, or an acknowledgement timeout caused by the loss
	of a network link.  It also returns true after the first call to TCPInit.
	Applications should use this function to reset their state machines.
	
	This function was added due to the possibility of an error when relying
	on TCPIsConnected returing false to check for a condition requiring a
	state machine reset.  If a socket is closed (due to a FIN ACK) and then
	immediately reopened (due to a the arrival of a new SYN) in the same
	cycle of the stack, calls to TCPIsConnected by the application will 
	never return false even though the socket has been disconnected.  This 
	can cause errors for protocols such as HTTP in which a client will 
	immediately open a new connection upon closing of a prior one.  Relying
	on this function instead allows applications to trap those conditions 
	and properly reset their internal state for the new connection.

  Precondition:
	TCP is initialized.

  Parameters:
	hTCP - The socket to check.

  Return Values:
  	true - The socket has been disconnected since the previous call.
  	false - The socket has not been disconnected since the previous call.
  ***************************************************************************/
bool TCPWasReset(TCP_SOCKET hTCP)
{
	if(!_TcpSocketChk(hTCP))
    {
        return false;
    }

	SyncTCBStub(hTCP);
	
	if(MyTCBStub.Flags.bSocketReset)
	{
		MyTCBStub.Flags.bSocketReset = 0;
		return true;
	}	
	
	return false;
}


/*****************************************************************************
  Function:
	bool TCPIsConnected(TCP_SOCKET hTCP)

  Summary:
	Determines if a socket has an established connection.

  Description:
	This function determines if a socket has an established connection to 
	a remote node.  Call this function after calling TCPOpen to determine 
	when the connection is set up and ready for use.  This function was 
	historically used to check for disconnections, but TCPWasReset is now a
	more appropriate solution. 

  Precondition:
	TCP is initialized.

  Parameters:
	hTCP - The socket to check.

  Return Values:
  	true - The socket has an established connection to a remote node.
  	false - The socket is not currently connected.

  Remarks:
	A socket is said to be connected only if it is in the TCP_ESTABLISHED
	state.  Sockets in the process of opening or closing will return false.
  ***************************************************************************/
bool TCPIsConnected(TCP_SOCKET hTCP)
{
	if(!_TcpSocketChk(hTCP))
    {
        return false;
    }

	SyncTCBStub(hTCP);
	return (MyTCBStub.smState == TCP_ESTABLISHED);
}


/*****************************************************************************
  Function:
	void TCPDisconnect(TCP_SOCKET hTCP)

  Summary:
	Disconnects an open socket.

  Description:
	This function closes a connection to a remote node by sending a FIN (if 
	currently connected).
	
	The function can be called a second time to force a socket closed by 
	sending a RST packet.  This is useful when the application knows that 
	the remote node will not send an ACK (if it has crashed or lost its link),
	or when the application needs to reuse the socket immediately regardless
	of whether or not the remote node would like to transmit more data before
	closing.
	
	For client mode sockets, upon return, the hTCP handle is relinquished to 
	the TCP/IP stack and must no longer be used by the application (except for 
	an immediate subsequent call to TCPDisconnect() to force a RST 
	transmission, if needed).  
	
	For server mode sockets, upon return, the hTCP handle is NOT relinquished 
	to the TCP/IP stack.  After closing, the socket returns to the listening 
	state allowing future connection requests to be serviced.  This leaves the 
	hTCP handle in a valid state and must be retained for future operations on 
	the socket.  If you want to close the server and relinquish the socket back 
	to the TCP/IP stack, call the TCPClose() API instead of TCPDisconnect().

  Precondition:
	None

  Parameters:
	hTCP - Handle of the socket to disconnect.

  Returns:
	None

  Remarks:
	If the socket is using SSL, a CLOSE_NOTIFY record will be transmitted
	first to allow the SSL session to be resumed at a later time.
  ***************************************************************************/
void TCPDisconnect(TCP_SOCKET hTCP)
{
	if(!_TcpSocketChk(hTCP))
    {
        return;
    }

	SyncTCBStub(hTCP);
	
	// If the TSBStubs table has been set to null, this will not work. Return in that case.
	if (TCBStubs == NULL)
	{
		return;
	}

	// Delete all data in the RX FIFO
	// In this stack's API, the application TCP handle is 
	// immediately invalid after calling this function, so there 
	// is no longer any way to receive data from the TCP RX FIFO, 
	// even though the data is still there.  Leaving the data there 
	// could interfere with the remote node sending us a FIN if our
	// RX window is zero
	MyTCBStub.rxTail = MyTCBStub.rxHead;

	switch(MyTCBStub.smState)
	{
		#if defined(TCPIP_STACK_CLIENT_MODE) && defined(TCPIP_STACK_USE_DNS)
#if defined (TCPIP_STACK_USE_IPV6)
		case TCP_DNS_RESOLVE_IPV6:
#endif
		case TCP_DNS_RESOLVE_IPV4:
			DNSEndUsage(0);	// Release the DNS module, since the user is aborting
			CloseSocket();
			break;
		#endif

#if defined (TCPIP_STACK_USE_IPV6)
		case TCP_GET_DNS_MODULE_IPV6:
#endif
		case TCP_GET_DNS_MODULE_IPV4:
		case TCP_GATEWAY_SEND_ARP:
		case TCP_GATEWAY_GET_ARP:
		case TCP_SYN_SENT:
			CloseSocket();
			break;

		case TCP_SYN_RECEIVED:
		case TCP_ESTABLISHED:
			#if defined(TCPIP_STACK_USE_SSL)
			// When disconnecting SSL sockets, send a close_notify so we can resume later
			if(MyTCBStub.sslStubID != SSL_INVALID_ID)
			{
				// Flush pending data and send close_notify
				SSLTxRecord(hTCP, MyTCBStub.sslStubID, SSL_APPLICATION);
				SSLTxMessage(hTCP, MyTCBStub.sslStubID, SSL_ALERT_CLOSE_NOTIFY);
			}
			#endif

			// Send the FIN.  This is done in a loop to ensure that if we have 
			// more data wating in the TX FIFO than can be sent in a single 
			// packet (due to the remote Max Segment Size packet size limit), 
			// we will keep generating more packets until either all data gets 
			// transmitted or the remote node's receive window fills up.
			do
			{
				SendTCP(FIN | ACK, SENDTCP_RESET_TIMERS);
				if(MyTCB.remoteWindow == 0u)
					break;
			} while(MyTCBStub.txHead != MyTCB.txUnackedTail);
			
			MyTCBStub.smState = TCP_FIN_WAIT_1;
			break;

		case TCP_CLOSE_WAIT:
			// Send the FIN.  This is done in a loop to ensure that if we have 
			// more data wating in the TX FIFO than can be sent in a single 
			// packet (due to the remote Max Segment Size packet size limit), 
			// we will keep generating more packets until either all data gets 
			// transmitted or the remote node's receive window fills up.
			do
			{
				SendTCP(FIN | ACK, SENDTCP_RESET_TIMERS);
				if(MyTCB.remoteWindow == 0u)
					break;
			} while(MyTCBStub.txHead != MyTCB.txUnackedTail);

			MyTCBStub.smState = TCP_LAST_ACK;
			break;
			
		// These states are all already closed or don't need explicit disconnecting -- they will disconnect by themselves after a while
        case TCP_CLOSED:    
		//case TCP_LISTEN:
		//case TCP_CLOSING:
		//case TCP_TIME_WAIT:
			return;

		case TCP_CLOSED_BUT_RESERVED:
			MyTCBStub.smState = TCP_CLOSED;
			break;

		// These states will close themselves after some delay, however, 
		// this is handled so that the user can call TCPDisconnect() 
		// twice to immediately close a socket (using an RST) without 
		// having to get an ACK back from the remote node.  This is 
		// great for instance when the application determines that 
		// the remote node has been physically disconnected and 
		// already knows that no ACK will be returned.  Alternatively, 
		// if the application needs to immediately reuse the socket 
		// regardless of what the other node's state is in (half open).
		case TCP_FIN_WAIT_1:
		case TCP_FIN_WAIT_2:
		case TCP_LAST_ACK:
		default:
			SendTCP(RST | ACK, 0);
			CloseSocket();
			break;
	}
}


/*****************************************************************************
  Function:
	void TCPClose(TCP_SOCKET hTCP)

  Summary:
	Disconnects an open socket and destroys the socket handle, including server 
	mode socket handles.

  Description:
	Disconnects an open socket and destroys the socket handle, including server 
	mode socket handles.  This function performs identically to the 
	TCPDisconnect() function, except that both client and server mode socket 
	handles are relinquished to the TCP/IP stack upon return.

  Precondition:
	None

  Parameters:
	hTCP - Handle to the socket to disconnect and close.

  Returns:
	None
  ***************************************************************************/
void TCPClose(TCP_SOCKET hTCP)
{
	if(!_TcpSocketChk(hTCP))
    {
        return;
    }

	SyncTCBStub(hTCP);

	if(MyTCBStub.smState != TCP_CLOSED)
	{
		if (MyTCBStub.pTxPkt != NULL)
		{
			MyTCBStub.Flags.bServer = false;
			TCPDisconnect(hTCP);
			TCPDisconnect(hTCP);
			TCPIP_IP_FreePacket (MyTCBStub.pTxPkt);
			MyTCBStub.pTxPkt = NULL;
		}
	}
}


/*****************************************************************************
  Function:
	bool TCPGetSocketInfo(TCP_SOCKET hTCP, TCP_SOCKET_INFO* remoteInfo)

  Summary:
	Obtains information about a currently open socket.

  Description:
	Fills the provided TCP_SOCKET_INFO structure associated with this socket.  This 
	contains the NODE_INFO structure with IP and MAC address (or gateway
	MAC) and the remote port.

  Precondition:
	TCP is initialized and the socket is connected.

  Parameters:
	hTCP - The socket to check.

  Returns:
    true if the call succeeded
    false if no such socket or the socket is not opened
  ***************************************************************************/
bool TCPGetSocketInfo(TCP_SOCKET hTCP, TCP_SOCKET_INFO* remoteInfo)
{

	if(!_TcpSocketChk(hTCP))
    {
        return false;
    }

	SyncTCBStub(hTCP);
	SyncTCB();

	if(MyTCBStub.smState == TCP_CLOSED)
    {
        return false;
    }

#if defined (TCPIP_STACK_USE_IPV6)
    if (MyTCBStub.pTxPkt->flags.addressType == IP_ADDRESS_TYPE_IPV6)
    {
    	memcpy((void*)&remoteInfo->remoteIPaddress.v6Add.v, (void*)TCPIP_IPV6_GetDestAddress(MyTCBStub.pTxPkt), sizeof(IPV6_ADDR));
        remoteInfo->addressType = IP_ADDRESS_TYPE_IPV6;
    }
    else
#endif
    {
        remoteInfo->remoteIPaddress.v4Add = TCPIP_IPV4_GetDestAddress(MyTCBStub.pTxPkt);
        remoteInfo->addressType = IP_ADDRESS_TYPE_IPV4;
    }

  	memcpy((void*)&remoteInfo->remoteMACAddr, (void*)&MyTCBStub.pTxPkt->remoteMACAddr, sizeof(MAC_ADDR));
	remoteInfo->remotePort = MyTCB.remotePort.Val;

	return true;
}



/****************************************************************************
  Section:
	Transmit Functions
  ***************************************************************************/

/*****************************************************************************
  Function:
	void TCPFlush(TCP_SOCKET hTCP)

  Summary:
	Immediately transmits all pending TX data.

  Description:
	This function immediately transmits all pending TX data with a PSH 
	flag.  If this function is not called, data will automatically be sent
	when either a) the TX buffer is half full or b) the 
	TCP_AUTO_TRANSMIT_TIMEOUT_VAL (default: 40ms) has elapsed.

  Precondition:
	TCP is initialized and the socket is connected.

  Parameters:
	hTCP - The socket whose data is to be transmitted.

  Returns:
	None

  Remarks:
	SSL application data is automatically flushed, so this function has 
	no effect for SSL sockets.
  ***************************************************************************/
void TCPFlush(TCP_SOCKET hTCP)
{
	if(!_TcpSocketChk(hTCP))
    {
        return;
    }

	SyncTCBStub(hTCP);
	SyncTCB();

	// NOTE: Pending SSL data will NOT be transferred here

	if(MyTCBStub.txHead != MyTCB.txUnackedTail)
	{
		// Send the TCP segment with all unacked bytes
		SendTCP(ACK, SENDTCP_RESET_TIMERS);
	}
}


/*****************************************************************************
  Function:
	uint16_t TCPIsPutReady(TCP_SOCKET hTCP)

  Summary:
	Determines how much free space is available in the TCP TX buffer.

  Description:
	Call this function to determine how many bytes can be written to the 
	TCP TX buffer.  If this function returns zero, the application must 
	return to the main stack loop before continuing in order to transmit
	more data.

  Precondition:
	TCP is initialized.

  Parameters:
	hTCP - The socket to check.

  Returns:
	The number of bytes available to be written in the TCP TX buffer.
  ***************************************************************************/
uint16_t TCPIsPutReady(TCP_SOCKET hTCP)
{
	uint8_t i;
    IP_PACKET * pkt;
	
	if(!_TcpSocketChk(hTCP))
    {
        return 0;
    }

	SyncTCBStub(hTCP);

    pkt = MyTCBStub.pTxPkt;

    if (pkt != NULL)
    {
        if (pkt->flags.queued)
        {
            // Try to allocate a new transmit packet
            IP_PACKET * tempPtr = TCPAllocateTxPacketStruct (MyTCBStub.pSktNet, pkt->flags.addressType);
            if (tempPtr != NULL)
            {
                if (!TCPIP_IP_CopyTxPacketStruct (tempPtr, pkt))
                {
                    TCPIP_IP_FreePacket (tempPtr);
                    return 0;
                }
                pkt->flags.packetBound = false;
                MyTCBStub.pTxPkt = tempPtr;
            }
            else
            {
                // We couldn't allocate a new packet.  Return 0 until we can 
                // or until a queued packet can be returned to this node.
                return 0;
            }
        }
    }
    else
    {
        // This should only happen if the user has made an inappropriate call to an 
        // unopened socket.
        return 0;
    }

	i = MyTCBStub.smState;

	// Unconnected sockets shouldn't be transmitting anything.
	if(!( (i == (uint8_t)TCP_ESTABLISHED) || (i == (uint8_t)TCP_CLOSE_WAIT) ))
		return 0;

	// Calculate the free space in this socket's TX FIFO
	#if defined(TCPIP_STACK_USE_SSL)
	if(MyTCBStub.sslStubID != SSL_INVALID_ID)
	{// Use sslTxHead as the head pointer when SSL is active
		uint16_t rem;
		
		// Find out raw free space
		if(MyTCBStub.sslTxHead >= MyTCBStub.txTail)
			rem = (MyTCBStub.bufferRxStart - MyTCBStub.bufferTxStart - 1) - (MyTCBStub.sslTxHead - MyTCBStub.txTail);
		else
			rem = MyTCBStub.txTail - MyTCBStub.sslTxHead - 1;
			
		// Reserve space for a new MAC and header
		if(rem > 22u)
			return rem - 22;
		else
			return 0;
	}
	#endif
	
	if(MyTCBStub.txHead >= MyTCBStub.txTail)
		return (MyTCBStub.bufferRxStart - MyTCBStub.bufferTxStart - 1) - (MyTCBStub.txHead - MyTCBStub.txTail);
	else
		return MyTCBStub.txTail - MyTCBStub.txHead - 1;
}


/*****************************************************************************
  Function:
	bool TCPPut(TCP_SOCKET hTCP, uint8_t byte)

  Description:
	Writes a single byte to a TCP socket.

  Precondition:
	TCP is initialized.

  Parameters:
	hTCP - The socket to which data is to be written.
	byte - The byte to write.

  Return Values:
	true - The byte was written to the transmit buffer.
	false - The transmit buffer was full, or the socket is not connected.
  ***************************************************************************/
bool TCPPut(TCP_SOCKET hTCP, uint8_t byte)
{
	uint16_t wFreeTXSpace;

	if(!_TcpSocketChk(hTCP))
    {
        return false;
    }

	SyncTCBStub(hTCP);

	wFreeTXSpace = TCPIsPutReady(hTCP);
	if(wFreeTXSpace == 0u)
		return false;
	else if(wFreeTXSpace == 1u) // About to run out of space, lets transmit so the remote node might send an ACK back faster
		TCPFlush(hTCP);	

	// Send all current bytes if we are crossing half full
	// This is required to improve performance with the delayed 
	// acknowledgement algorithm
	if((!MyTCBStub.Flags.bHalfFullFlush) && (wFreeTXSpace <= ((MyTCBStub.bufferRxStart-MyTCBStub.bufferTxStart)>>1)))
	{
		TCPFlush(hTCP);	
		MyTCBStub.Flags.bHalfFullFlush = true;
	}

	#if defined(TCPIP_STACK_USE_SSL)
	if(MyTCBStub.sslStubID != SSL_INVALID_ID)
	{
		TCPRAMCopy(MyTCBStub.pSktNet, MyTCBStub.sslTxHead, MyTCBStub.vMemoryMedium, (PTR_BASE)&byte, TCP_PIC_RAM, sizeof(byte));
		if(++MyTCBStub.sslTxHead >= MyTCBStub.bufferRxStart)
			MyTCBStub.sslTxHead = MyTCBStub.bufferTxStart;
	}
	else
	{
		TCPRAMCopy(MyTCBStub.pSktNet, MyTCBStub.txHead, MyTCBStub.vMemoryMedium, (PTR_BASE)&byte, TCP_PIC_RAM, sizeof(byte));
		if(++MyTCBStub.txHead >= MyTCBStub.bufferRxStart)
			MyTCBStub.txHead = MyTCBStub.bufferTxStart;
	}
	#else
	TCPRAMCopy(MyTCBStub.pSktNet, MyTCBStub.txHead, MyTCBStub.vMemoryMedium, (PTR_BASE)&byte, TCP_PIC_RAM, sizeof(byte));
	if(++MyTCBStub.txHead >= MyTCBStub.bufferRxStart)
		MyTCBStub.txHead = MyTCBStub.bufferTxStart;
	#endif
	

	// Send the last byte as a separate packet (likely will make the remote node send back ACK faster)
	if(wFreeTXSpace == 1u)
	{
		TCPFlush(hTCP);
	}
	// If not already enabled, start a timer so this data will 
	// eventually get sent even if the application doens't call
	// TCPFlush()
	else if(!MyTCBStub.Flags.bTimer2Enabled)
	{
		MyTCBStub.Flags.bTimer2Enabled = true;
		MyTCBStub.eventTime2 = SYS_TICK_Get() + (TCP_AUTO_TRANSMIT_TIMEOUT_VAL * SYS_TICK_TicksPerSecondGet())/1000;
	}

	return true;
}

/*****************************************************************************
  Function:
	uint16_t TCPPutArray(TCP_SOCKET hTCP, const uint8_t* data, uint16_t len)

  Description:
	Writes an array from RAM to a TCP socket.

  Precondition:
	TCP is initialized.

  Parameters:
	hTCP - The socket to which data is to be written.
	data - Pointer to the array to be written.
	len  - Number of bytes to be written.

  Returns:
	The number of bytes written to the socket.  If less than len, the
	buffer became full or the socket is not conected.
  ***************************************************************************/
uint16_t TCPPutArray(TCP_SOCKET hTCP, const uint8_t* data, uint16_t len)
{
	uint16_t wActualLen;
	uint16_t wFreeTXSpace;
	uint16_t wRightLen = 0;

	if(!_TcpSocketChk(hTCP))
    {
        return 0;
    }

	SyncTCBStub(hTCP);

	wFreeTXSpace = TCPIsPutReady(hTCP);
	if(wFreeTXSpace == 0u)
	{
		TCPFlush(hTCP);
		return 0;
	}

	wActualLen = wFreeTXSpace;
	if(wFreeTXSpace > len)
		wActualLen = len;

	// Send all current bytes if we are crossing half full
	// This is required to improve performance with the delayed 
	// acknowledgement algorithm
	if((!MyTCBStub.Flags.bHalfFullFlush) && (wFreeTXSpace <= ((MyTCBStub.bufferRxStart-MyTCBStub.bufferTxStart)>>1)))
	{
		TCPFlush(hTCP);	
		MyTCBStub.Flags.bHalfFullFlush = true;
	}
	
	#if defined(TCPIP_STACK_USE_SSL)
	if(MyTCBStub.sslStubID != SSL_INVALID_ID)
	{
		// See if we need a two part put
		if(MyTCBStub.sslTxHead + wActualLen >= MyTCBStub.bufferRxStart)
		{
			wRightLen = MyTCBStub.bufferRxStart-MyTCBStub.sslTxHead;
			TCPRAMCopy(MyTCBStub.pSktNet, MyTCBStub.sslTxHead, MyTCBStub.vMemoryMedium, (PTR_BASE)data, TCP_PIC_RAM, wRightLen);
			data += wRightLen;
			wActualLen -= wRightLen;
			MyTCBStub.sslTxHead = MyTCBStub.bufferTxStart;
		}
	
		TCPRAMCopy(MyTCBStub.pSktNet, MyTCBStub.sslTxHead, MyTCBStub.vMemoryMedium, (PTR_BASE)data, TCP_PIC_RAM, wActualLen);
		MyTCBStub.sslTxHead += wActualLen;
	}
	else
	{
		// See if we need a two part put
		if(MyTCBStub.txHead + wActualLen >= MyTCBStub.bufferRxStart)
		{
			wRightLen = MyTCBStub.bufferRxStart-MyTCBStub.txHead;
			TCPRAMCopy(MyTCBStub.pSktNet, MyTCBStub.txHead, MyTCBStub.vMemoryMedium, (PTR_BASE)data, TCP_PIC_RAM, wRightLen);
			data += wRightLen;
			wActualLen -= wRightLen;
			MyTCBStub.txHead = MyTCBStub.bufferTxStart;
		}
	
		TCPRAMCopy(MyTCBStub.pSktNet, MyTCBStub.txHead, MyTCBStub.vMemoryMedium, (PTR_BASE)data, TCP_PIC_RAM, wActualLen);
		MyTCBStub.txHead += wActualLen;
	}
	#else
	// See if we need a two part put
	if(MyTCBStub.txHead + wActualLen >= MyTCBStub.bufferRxStart)
	{
		wRightLen = MyTCBStub.bufferRxStart-MyTCBStub.txHead;
		TCPRAMCopy(MyTCBStub.pSktNet, MyTCBStub.txHead, MyTCBStub.vMemoryMedium, (PTR_BASE)data, TCP_PIC_RAM, wRightLen);
		data += wRightLen;
		wActualLen -= wRightLen;
		MyTCBStub.txHead = MyTCBStub.bufferTxStart;
	}

	TCPRAMCopy(MyTCBStub.pSktNet, MyTCBStub.txHead, MyTCBStub.vMemoryMedium, (PTR_BASE)data, TCP_PIC_RAM, wActualLen);
	MyTCBStub.txHead += wActualLen;
	#endif

	// Send these bytes right now if we are out of TX buffer space
	if(wFreeTXSpace <= len)
	{
		TCPFlush(hTCP);
	}
	// If not already enabled, start a timer so this data will 
	// eventually get sent even if the application doens't call
	// TCPFlush()
	else if(!MyTCBStub.Flags.bTimer2Enabled)
	{
		MyTCBStub.Flags.bTimer2Enabled = true;
		MyTCBStub.eventTime2 = SYS_TICK_Get() + (TCP_AUTO_TRANSMIT_TIMEOUT_VAL * SYS_TICK_TicksPerSecondGet())/1000;
	}

	return wActualLen + wRightLen;
}

/*****************************************************************************
  Function:
	const uint8_t* TCPPutString(TCP_SOCKET hTCP, const uint8_t* data)

  Description:
	Writes a null-terminated string from RAM to a TCP socket.  The 
	null-terminator is not copied to the socket.

  Precondition:
	TCP is initialized.

  Parameters:
	hTCP - The socket to which data is to be written.
	data - Pointer to the string to be written.

  Returns:
	Pointer to the byte following the last byte written to the socket.  If
	this pointer does not dereference to a NUL byte, the buffer became full
	or the socket is not connected.

  Remarks:
	The return value of this function differs from that of TCPPutArray.  To
	write long strings in a single state, initialize the *data pointer to the
	first byte, then call this function repeatedly (breaking to the main 
	stack loop after each call) until the return value dereferences to a NUL
	byte.  Save the return value as the new starting *data pointer otherwise.
  ***************************************************************************/
const uint8_t* TCPPutString(TCP_SOCKET hTCP, const uint8_t* data)
{
	return data + TCPPutArray(hTCP, data, strlen((char*)data));
}

/*****************************************************************************
  Function:
	uint16_t TCPGetTxFIFOFull(TCP_SOCKET hTCP)

  Description:
	Determines how many bytes are pending in the TCP TX FIFO.

  Precondition:
	TCP is initialized.

  Parameters:
	hTCP - The socket to check.

  Returns:
	Number of bytes pending to be flushed in the TCP TX FIFO.
  ***************************************************************************/
uint16_t TCPGetTxFIFOFull(TCP_SOCKET hTCP)
{
	uint16_t wDataLen;
	uint16_t wFIFOSize;

	if(!_TcpSocketChk(hTCP))
    {
        return 0;
    }

	SyncTCBStub(hTCP);

	// Calculate total usable FIFO size
	wFIFOSize = MyTCBStub.bufferRxStart - MyTCBStub.bufferTxStart - 1;

	// Find out how many data bytes are free in the TX FIFO
	wDataLen = TCPIsPutReady(hTCP);

	return wFIFOSize - wDataLen;
}



/****************************************************************************
  Section:
	Receive Functions
  ***************************************************************************/

/*****************************************************************************
  Function:
	void TCPDiscard(TCP_SOCKET hTCP)

  Description:
	Discards any pending data in the TCP RX FIFO.

  Precondition:
	TCP is initialized.

  Parameters:
	hTCP - The socket whose RX FIFO is to be cleared.

  Returns:
	None
  ***************************************************************************/
void TCPDiscard(TCP_SOCKET hTCP)
{
	if(TCPIsGetReady(hTCP))
	{
		SyncTCBStub(hTCP);
	
		// Delete all data in the RX buffer
		MyTCBStub.rxTail = MyTCBStub.rxHead;
	
		// Send a Window update message to the remote node
		SendTCP(ACK, SENDTCP_RESET_TIMERS);
	}
}


/*****************************************************************************
  Function:
	void uint16_t TCPIsGetReady(TCP_SOCKET hTCP)

  Summary:
	Determines how many bytes can be read from the TCP RX buffer.

  Description:
	Call this function to determine how many bytes can be read from the 
	TCP RX buffer.  If this function returns zero, the application must 
	return to the main stack loop before continuing in order to wait for
	more data to arrive.

  Precondition:
	TCP is initialized.

  Parameters:
	hTCP - The socket to check.

  Returns:
	The number of bytes available to be read from the TCP RX buffer.
  ***************************************************************************/
uint16_t TCPIsGetReady(TCP_SOCKET hTCP)
{
	if(!_TcpSocketChk(hTCP))
    {
        return 0;
    }

	SyncTCBStub(hTCP);
		
	if(MyTCBStub.rxHead >= MyTCBStub.rxTail)
		return MyTCBStub.rxHead - MyTCBStub.rxTail;
	else
		return (MyTCBStub.bufferEnd - MyTCBStub.rxTail + 1) + (MyTCBStub.rxHead - MyTCBStub.bufferRxStart);
}


/*****************************************************************************
  Function:
	bool TCPGet(TCP_SOCKET hTCP, uint8_t* byte)

  Description:
	Retrieves a single byte to a TCP socket.

  Precondition:
	TCP is initialized.

  Parameters:
	hTCP - The socket from which to read.
	byte - Pointer to location in which the read byte should be stored.

  Return Values:
	true - A byte was read from the buffer.
	false - The buffer was empty, or the socket is not connected.
  ***************************************************************************/
bool TCPGet(TCP_SOCKET hTCP, uint8_t* byte)
{
	uint16_t wGetReadyCount;

	// See if there is any data which can be read
	wGetReadyCount = TCPIsGetReady(hTCP);
	if(wGetReadyCount == 0u)
		return false;

	SyncTCBStub(hTCP);
	
	if(byte)
		TCPRAMCopy(MyTCBStub.pSktNet, (PTR_BASE)byte, TCP_PIC_RAM, MyTCBStub.rxTail, MyTCBStub.vMemoryMedium, 1);
	if(++MyTCBStub.rxTail > MyTCBStub.bufferEnd)
		MyTCBStub.rxTail = MyTCBStub.bufferRxStart;

	// Send a window update if we've run out of data
	if(wGetReadyCount == 1u)
	{
		MyTCBStub.Flags.bTXASAPWithoutTimerReset = 1;
	}
	// If not already enabled, start a timer so a window 
	// update will get sent to the remote node at some point
	else if(!MyTCBStub.Flags.bTimer2Enabled)
	{
		MyTCBStub.Flags.bTimer2Enabled = true;
		MyTCBStub.eventTime2 = SYS_TICK_Get() + (TCP_WINDOW_UPDATE_TIMEOUT_VAL * SYS_TICK_TicksPerSecondGet())/1000;
	}


	return true;
}


/*****************************************************************************
  Function:
	uint16_t TCPGetArray(TCP_SOCKET hTCP, uint8_t* buffer, uint16_t len)

  Description:
	Reads an array of data bytes from a TCP socket's receive FIFO.  The data 
	is removed from the FIFO in the process.

  Precondition:
	TCP is initialized.

  Parameters:
	hTCP - The socket from which data is to be read.
	buffer - Pointer to the array to store data that was read.
	len  - Number of bytes to be read.

  Returns:
	The number of bytes read from the socket.  If less than len, the
	RX FIFO buffer became empty or the socket is not conected.
  ***************************************************************************/
uint16_t TCPGetArray(TCP_SOCKET hTCP, uint8_t* buffer, uint16_t len)
{
	uint16_t wGetReadyCount;
	uint16_t RightLen = 0;

	// See if there is any data which can be read
	wGetReadyCount = TCPIsGetReady(hTCP);
	if(wGetReadyCount == 0u)
		return 0x0000u;

	SyncTCBStub(hTCP);

	// Make sure we don't try to read more data than is available
	if(len > wGetReadyCount)
		len = wGetReadyCount;

	// See if we need a two part get
	if(MyTCBStub.rxTail + len > MyTCBStub.bufferEnd)
	{
		RightLen = MyTCBStub.bufferEnd - MyTCBStub.rxTail + 1;
		if(buffer)
		{
			TCPRAMCopy(MyTCBStub.pSktNet, (PTR_BASE)buffer, TCP_PIC_RAM, MyTCBStub.rxTail, MyTCBStub.vMemoryMedium, RightLen);
			buffer += RightLen;
		}
		len -= RightLen;
		MyTCBStub.rxTail = MyTCBStub.bufferRxStart;
	}

	if(buffer)
		TCPRAMCopy(MyTCBStub.pSktNet, (PTR_BASE)buffer, TCP_PIC_RAM, MyTCBStub.rxTail, MyTCBStub.vMemoryMedium, len);
	MyTCBStub.rxTail += len;
	len += RightLen;

	// Send a window update if we've run low on data
	if(wGetReadyCount - len <= len)
	{
		MyTCBStub.Flags.bTXASAPWithoutTimerReset = 1;
	}
	else if(!MyTCBStub.Flags.bTimer2Enabled)
	// If not already enabled, start a timer so a window 
	// update will get sent to the remote node at some point
	{
		MyTCBStub.Flags.bTimer2Enabled = true;
		MyTCBStub.eventTime2 = SYS_TICK_Get() + (TCP_WINDOW_UPDATE_TIMEOUT_VAL * SYS_TICK_TicksPerSecondGet())/1000;
	}

	return len;
}


/*****************************************************************************
  Function:
	uint16_t TCPGetRxFIFOFree(TCP_SOCKET hTCP)

  Description:
	Determines how many bytes are free in the RX FIFO.

  Precondition:
	TCP is initialized.

  Parameters:
	hTCP - The socket to check.

  Returns:
	The number of bytes free in the TCP RX FIFO.  If zero, no additional 
	data can be received until the application removes some data using one
	of the TCPGet family functions.
  ***************************************************************************/
uint16_t TCPGetRxFIFOFree(TCP_SOCKET hTCP)
{
	uint16_t wDataLen;
	uint16_t wFIFOSize;
	
	if(!_TcpSocketChk(hTCP))
    {
        return 0;
    }

	SyncTCBStub(hTCP);
	
	// Calculate total usable FIFO size
	wFIFOSize = MyTCBStub.bufferEnd - MyTCBStub.bufferRxStart;

	#if defined(TCPIP_STACK_USE_SSL)
	{
		PTR_BASE SSLtemp = MyTCBStub.rxHead;

		// Move SSL pointer to determine full buffer size
		if(MyTCBStub.sslStubID != SSL_INVALID_ID)
			MyTCBStub.rxHead = MyTCBStub.sslRxHead;

		// Find out how many data bytes are actually in the RX FIFO
		wDataLen = TCPIsGetReady(hTCP);
		
		// Move SSL pointer back to proper location (if we changed it)
		MyTCBStub.rxHead = SSLtemp;
	}
	#else
	{
		// Find out how many data bytes are actually in the RX FIFO
		wDataLen = TCPIsGetReady(hTCP);
	}
	#endif
	
	// Perform the calculation	
	return wFIFOSize - wDataLen;
}

/*****************************************************************************
  Function:
	uint16_t TCPPeekArray(TCP_SOCKET hTCP, uint8_t *vBuffer, uint16_t wLen, uint16_t wStart)

  Summary:
  	Reads a specified number of data bytes from the TCP RX FIFO without 
  	removing them from the buffer.

  Description:
	Reads a specified number of data bytes from the TCP RX FIFO without 
  	removing them from the buffer.  No TCP control actions are taken as a 
  	result of this function (ex: no window update is sent to the remote node).
  	
  Precondition:
	TCP is initialized.

  Parameters:
	hTCP - The socket to peak from (read without removing from stream).
	vBuffer - Destination to write the peeked data bytes.
	wLen - Length of bytes to peak from the RX FIFO and copy to vBuffer.
	wStart - Zero-indexed starting position within the FIFO to start peeking 
		from.

  Return Values:
	Number of bytes actually peeked from the stream and copied to vBuffer.  
	This value can be less than wLen if wStart + wLen is greater than the 
	deepest possible character in the RX FIFO.

  Remarks:
  	None
  ***************************************************************************/
uint16_t TCPPeekArray(TCP_SOCKET hTCP, uint8_t *vBuffer, uint16_t wLen, uint16_t wStart)
{
	PTR_BASE ptrRead;
	uint16_t w;
	uint16_t wBytesUntilWrap;

	if(!_TcpSocketChk(hTCP) || wLen == 0)
    {
        return 0;
    }


	SyncTCBStub(hTCP);

	// Find out how many bytes are in the RX FIFO and decrease read length 
	// if the start offset + read length is beyond the end of the FIFO
	w = TCPIsGetReady(hTCP);
	if(wStart + wLen > w)
		wLen = w - wStart;

	// Find the read start location
	ptrRead = MyTCBStub.rxTail + wStart;
	if(ptrRead > MyTCBStub.bufferEnd)
		ptrRead -= MyTCBStub.bufferEnd - MyTCBStub.bufferRxStart + 1;

	// Calculate how many bytes can be read in a single go
	wBytesUntilWrap = MyTCBStub.bufferEnd - ptrRead + 1;
	if(wLen <= wBytesUntilWrap)
	{
		// Read all at once
		TCPRAMCopy(MyTCBStub.pSktNet, (PTR_BASE)vBuffer, TCP_PIC_RAM, ptrRead, MyTCBStub.vMemoryMedium, wLen);
	}
	else
	{
		// Read all bytes up to the wrap position and then read remaining bytes 
		// at the start of the buffer
		TCPRAMCopy(MyTCBStub.pSktNet, (PTR_BASE)vBuffer, TCP_PIC_RAM, ptrRead, MyTCBStub.vMemoryMedium, wBytesUntilWrap);
		TCPRAMCopy(MyTCBStub.pSktNet, (PTR_BASE)vBuffer+wBytesUntilWrap, TCP_PIC_RAM, MyTCBStub.bufferRxStart, MyTCBStub.vMemoryMedium, wLen - wBytesUntilWrap);
	}
	
	return wLen;
}

/*****************************************************************************
  Function:
	uint8_t TCPPeek(TCP_SOCKET hTCP, uint16_t wStart)

  Summary:
  	Peaks at one byte in the TCP RX FIFO without removing it from the buffer.

  Description:
	Peaks at one byte in the TCP RX FIFO without removing it from the buffer.
  	
  Precondition:
	TCP is initialized.

  Parameters:
	hTCP - The socket to peak from (read without removing from stream).
	wStart - Zero-indexed starting position within the FIFO to peek from.

  Return Values:
	Byte peeked from the RX FIFO.  If there is no data in the buffer or an 
	illegal wStart starting offset is given, then an indeterminate value is 
	returned.  The caller must ensure that valid parameters are passed to avoid 
	(i.e ensure that TCPIsGetReady() returns a number that is less than wStart 
	before calling TCPPeek()).

  Remarks:
  	Use the TCPPeekArray() function to read more than one byte.  It will 
  	perform better than calling TCPPeek() in a loop.
  ***************************************************************************/
uint8_t TCPPeek(TCP_SOCKET hTCP, uint16_t wStart)
{
	uint8_t i;
	
	TCPPeekArray(hTCP, &i, 1, wStart);
	return i;
}


/****************************************************************************
  Section:
	Search Functions
  ***************************************************************************/

/*****************************************************************************
  Function:
	uint16_t TCPFindArray(TCP_SOCKET hTCP, uint8_t* cFindArray, uint16_t wLen, 
						uint16_t wStart, uint16_t wSearchLen, bool bTextCompare)

  Summary:
  	Searches for a string in the TCP RX buffer.

  Description:
	This function finds the first occurrance of an array of bytes in the
	TCP RX buffer.  It can be used by an application to abstract searches 
	out of their own application code.  For increased efficiency, the 
	function is capable of limiting the scope of search to a specific
	range of bytes.  It can also perform a case-insensitive search if
	required.
	
	For example, if the buffer contains "I love PIC MCUs!" and the search
	array is "love" with a length of 4, a value of 2 will be returned.

  Precondition:
	TCP is initialized.

  Parameters:
	hTCP - The socket to search within.
	cFindArray - The array of bytes to find in the buffer.
	wLen - Length of cFindArray.
	wStart - Zero-indexed starting position within the buffer.
	wSearchLen - Length from wStart to search in the buffer.
	bTextCompare - true for case-insensitive text search, false for binary search

  Return Values:
	0xFFFF - Search array not found
	Otherwise - Zero-indexed position of the first occurrance

  Remarks:
	Since this function usually must transfer data from external storage
	to internal RAM for comparison, its performance degrades significantly
	when the buffer is full and the array is not found.  For better 
	performance, try to search for characters that are expected to exist or
	limit the scope of the search as much as possible.  The HTTP2 module, 
	for example, uses this function to parse headers.  However, it searches 
	for newlines, then the separating colon, then reads the header name to 
	RAM for final comparison.  This has proven to be significantly faster  
	than searching for full header name strings outright.
  ***************************************************************************/
uint16_t TCPFindArray(TCP_SOCKET hTCP, const uint8_t* cFindArray, uint16_t wLen, uint16_t wStart, uint16_t wSearchLen, bool bTextCompare)
{
	PTR_BASE ptrRead;
	uint16_t wDataLen;
	uint16_t wBytesUntilWrap;
	PTR_BASE ptrLocation;
	uint16_t wLenStart;
	const uint8_t *cFindArrayStart;
	uint8_t i, j, k;
	bool isFinding;
	uint8_t buffer[32];

	if(!_TcpSocketChk(hTCP) || wLen == 0)
    {
        return 0;
    }


	SyncTCBStub(hTCP);

	// Find out how many bytes are in the RX FIFO and return 
	// immediately if we won't possibly find a match
	wDataLen = TCPIsGetReady(hTCP) - wStart;
	if(wDataLen < wLen)
		return 0xFFFFu;
	if(wSearchLen && (wDataLen > wSearchLen))
		wDataLen = wSearchLen;

	ptrLocation = MyTCBStub.rxTail + wStart;
	if(ptrLocation > MyTCBStub.bufferEnd)
		ptrLocation -= MyTCBStub.bufferEnd - MyTCBStub.bufferRxStart + 1;
	ptrRead = ptrLocation;
	wBytesUntilWrap = MyTCBStub.bufferEnd - ptrLocation + 1;
	ptrLocation = wStart;
	wLenStart = wLen;
	cFindArrayStart = cFindArray;
	j = *cFindArray++;
	isFinding = false;
	if(bTextCompare)
	{
		if(j >= 'a' && j <= 'z')
			j += 'A'-'a';
	}

	// Search for the array
	while(1)
	{
		// Figure out how big of a chunk to read
		k = sizeof(buffer);
		if(k > wBytesUntilWrap)
			k = wBytesUntilWrap;
		if((uint16_t)k > wDataLen)
			k = wDataLen;

		// Read a chunk of data into the buffer
		TCPRAMCopy(MyTCBStub.pSktNet, (PTR_BASE)buffer, TCP_PIC_RAM, ptrRead, MyTCBStub.vMemoryMedium, (uint16_t)k);
		ptrRead += k;
		wBytesUntilWrap -= k;

		if(wBytesUntilWrap == 0u)
		{
			ptrRead = MyTCBStub.bufferRxStart;
			wBytesUntilWrap = 0xFFFFu;
		}

		// Convert everything to uppercase
		if(bTextCompare)
		{
			for(i = 0; i < k; i++)
			{
				if(buffer[i] >= 'a' && buffer[i] <= 'z')
					buffer[i] += 'A'-'a';

				if(j == buffer[i])
				{
					if(--wLen == 0u)
						return ptrLocation-wLenStart + i + 1;
					j = *cFindArray++;
					isFinding = true;
					if(j >= 'a' && j <= 'z')
						j += 'A'-'a';
				}
				else
				{
					wLen = wLenStart;
					if(isFinding)
					{
						cFindArray = cFindArrayStart;
						j = *cFindArray++;
						if(j >= 'a' && j <= 'z')
							j += 'A'-'a';
						isFinding = false;
					}
				}
			}
		}
		else	// Compare as is
		{
			for(i = 0; i < k; i++)
			{
				if(j == buffer[i])
				{
					if(--wLen == 0u)
						return ptrLocation-wLenStart + i + 1;
					j = *cFindArray++;
					isFinding = true;
				}
				else
				{
					wLen = wLenStart;
					if(isFinding)
					{
						cFindArray = cFindArrayStart;
						j = *cFindArray++;
						isFinding = false;
					}
				}
			}
		}

		// Check to see if it is impossible to find a match
		wDataLen -= k;
		if(wDataLen < wLen)
			return 0xFFFFu;

		ptrLocation += k;
	}
}

/*****************************************************************************
  Function:
	uint16_t TCPFind(TCP_SOCKET hTCP, uint8_t cFind,
						uint16_t wStart, uint16_t wSearchLen, bool bTextCompare)

  Summary:
  	Searches for a byte in the TCP RX buffer.

  Description:
	This function finds the first occurrance of a byte in the TCP RX
	buffer.  It can be used by an application to abstract searches 
	out of their own application code.  For increased efficiency, the 
	function is capable of limiting the scope of search to a specific
	range of bytes.  It can also perform a case-insensitive search if
	required.
	
	For example, if the buffer contains "I love PIC MCUs!" and the cFind
	byte is ' ', a value of 1 will be returned.

  Precondition:
	TCP is initialized.

  Parameters:
	hTCP - The socket to search within.
	cFind - The byte to find in the buffer.
	wStart - Zero-indexed starting position within the buffer.
	wSearchLen - Length from wStart to search in the buffer.
	bTextCompare - true for case-insensitive text search, false for binary search

  Return Values:
	0xFFFF - Search array not found
	Otherwise - Zero-indexed position of the first occurrance

  Remarks:
	Since this function usually must transfer data from external storage
	to internal RAM for comparison, its performance degrades significantly
	when the buffer is full and the array is not found.  For better 
	performance, try to search for characters that are expected to exist or
	limit the scope of the search as much as possible.  The HTTP2 module, 
	for example, uses this function to parse headers.  However, it searches 
	for newlines, then the separating colon, then reads the header name to 
	RAM for final comparison.  This has proven to be significantly faster  
	than searching for full header name strings outright.
  ***************************************************************************/
uint16_t TCPFind(TCP_SOCKET hTCP, uint8_t cFind, uint16_t wStart, uint16_t wSearchLen, bool bTextCompare)
{
	return TCPFindArray(hTCP, &cFind, sizeof(cFind), wStart, wSearchLen, bTextCompare);
}



/****************************************************************************
  Section:
	Data Processing Functions
  ***************************************************************************/

/*****************************************************************************
  Function:
	void TCPTick(void)

  Summary:
  	Performs periodic TCP tasks.

  Description:
	This function performs any required periodic TCP tasks.  Each 
	socket's state machine is checked, and any elapsed timeout periods
	are handled.

  Precondition:
	TCP is initialized.

  Parameters:
	None

  Returns:
	None
  ***************************************************************************/
void TCPTick(void)
{
	TCP_SOCKET hTCP;
	bool bRetransmit;
	bool bCloseSocket;
	uint8_t vFlags;
	uint16_t w;
    IP_PACKET * pkt;

	// Periodically all "not closed" sockets must perform timed operations
	for(hTCP = 0; hTCP < TcpSockets; hTCP++)
	{
		SyncTCBStub(hTCP);
		
        pkt = MyTCBStub.pTxPkt;


		// Handle any SSL Processing and Message Transmission
		#if defined(TCPIP_STACK_USE_SSL)
		if(MyTCBStub.sslStubID != SSL_INVALID_ID)
		{
			// Handle any periodic tasks, such as RSA operations
			SSLPeriodic(hTCP, MyTCBStub.sslStubID);
			
			// If unsent data is waiting, transmit it as an application record
			if(MyTCBStub.sslTxHead != MyTCBStub.txHead && TCPSSLGetPendingTxSize(hTCP) != 0u)
				SSLTxRecord(hTCP, MyTCBStub.sslStubID, SSL_APPLICATION);
			
			// If an SSL message is requested, send it now
			if(MyTCBStub.sslReqMessage != SSL_NO_MESSAGE)
				SSLTxMessage(hTCP, MyTCBStub.sslStubID, MyTCBStub.sslReqMessage);
		}
		#endif

		vFlags = 0x00;
		bRetransmit = false;
		bCloseSocket = false;

		// Transmit ASAP data if the medium is available
		if(MyTCBStub.Flags.bTXASAP || MyTCBStub.Flags.bTXASAPWithoutTimerReset)
		{
			if(TCPIP_IP_IsTxReady(MyTCBStub.pSktNet))
			{
				vFlags = ACK;
				bRetransmit = MyTCBStub.Flags.bTXASAPWithoutTimerReset;
			}
		}

		// Perform any needed window updates and data transmissions
		if(MyTCBStub.Flags.bTimer2Enabled)
		{
			// See if the timeout has occured, and we need to send a new window update and pending data
			if(SYS_TICK_Get() >= MyTCBStub.eventTime2)
				vFlags = ACK;
		}

		// Process Delayed ACKnowledgement timer
		if(MyTCBStub.Flags.bDelayedACKTimerEnabled)
		{
			// See if the timeout has occured and delayed ACK needs to be sent
			if(SYS_TICK_Get() >= MyTCBStub.OverlappedTimers.delayedACKTime)
				vFlags = ACK;
		}
		
		// Process TCP_CLOSE_WAIT timer
		if(MyTCBStub.smState == TCP_CLOSE_WAIT)
		{
			// Automatically close the socket on our end if the application 
			// fails to call TCPDisconnect() is a reasonable amount of time.
			if(SYS_TICK_Get() >= MyTCBStub.OverlappedTimers.closeWaitTime)
			{
				vFlags = FIN | ACK;
				MyTCBStub.smState = TCP_LAST_ACK;
			}
		}

		// Process listening server sockets that might have a SYN waiting in the SYNQueue[]
		#if TCP_SYN_QUEUEING
			if(MyTCBStub.smState == TCP_LISTEN)
			{
				for(w = 0; w < TcpSynQueues; w++)
				{
                    if(_TCPIPStackIsNetUp(SYNQueue[w].pSynIf))
                    {   // ignore interfaces that are down
                        bool isSslSkt, isRegSkt, isForceBound;
                        // Abort search if there are no more valid records
                        if(SYNQueue[w].wDestPort == 0u)
                            break;

                        // Stop searching if this SYN queue entry can be used by this socket
#if defined(TCPIP_STACK_USE_SSL_SERVER)
                        isSslSkt = (SYNQueue[w].wDestPort == MyTCBStub.sslTxHead);
#else
                        isSslSkt = 0;
#endif

                        isRegSkt = (SYNQueue[w].wDestPort == MyTCBStub.remoteHash.Val);

                        if( isSslSkt || isRegSkt)
                        {
                            if(MyTCBStub.pSktNet == 0)
                            {   // unbound socket; bind it to this interface
                                _TcpSocketBind(&MyTCBStub, SYNQueue[w].pSynIf);
                                isForceBound = 1;    // if the SSL connection fails we'll have to unbound!
                            }
                            else if (MyTCBStub.pSktNet != SYNQueue[w].pSynIf)
                            {   // make sure is bound to the correct interface
                                continue;   // find another one
                            }
                            else
                            {
                                isForceBound = 0;
                            }

#if defined(TCPIP_STACK_USE_SSL_SERVER)
                            // If this matches the SSL port, make sure that can be configured
                            // before continuing.  If not, break and leave this in the queue
                            if(isSslSkt)
                            {
                                if(!TCPStartSSLServer(hTCP))
                                {
                                    if(isForceBound)
                                    {
                                        _TcpSocketBind(&MyTCBStub, 0);  // remove the binding
                                    }
                                    break;
                                }
                            }
#endif

                            // Set up our socket and generate a reponse SYN+ACK packet
                            SyncTCB();
                            MyTCB.remotePort.Val = SYNQueue[w].wSourcePort;
#if defined (TCPIP_STACK_USE_IPV6)
                            if (SYNQueue[w].addressType == IP_ADDRESS_TYPE_IPV6)
                            {
                                TCPIP_IPV6_SetDestAddress (MyTCBStub.pTxPkt, &SYNQueue[w].sourceAddress.sourceIPv6);
                                TCPIP_IPV6_SetSourceAddress (MyTCBStub.pTxPkt, &SYNQueue[w].destAddr);
                                MyTCBStub.remoteHash.Val = TCPIP_IPV6_GetHash(TCPIP_IPV6_GetDestAddress(MyTCBStub.pTxPkt), MyTCB.remotePort.Val, MyTCB.localPort.Val);
                            }
                            else
#endif
                            {
                                TCPIP_IPV4_SetDestAddress (MyTCBStub.pTxPkt, SYNQueue[w].sourceAddress.sourceIPv4.Val);
                                memcpy((void*)&MyTCBStub.pTxPkt->remoteMACAddr, (void*)&SYNQueue[w].remoteMACAddr, sizeof(MAC_ADDR));
                                MyTCBStub.remoteHash.Val = (TCPIP_IPV4_GetDestAddress(MyTCBStub.pTxPkt).w[1] + TCPIP_IPV4_GetDestAddress(MyTCBStub.pTxPkt).w[0] + MyTCB.remotePort.Val) ^ MyTCB.localPort.Val;
                            }

#if defined (TCPIP_STACK_USE_IPV6)
                            TCPIP_IP_SetPacketIPProtocol (MyTCBStub.pTxPkt, SYNQueue[w].addressType);
#endif
                            MyTCB.RemoteSEQ = SYNQueue[w].dwSourceSEQ + 1;
                            vFlags = SYN | ACK;
                            MyTCBStub.smState = TCP_SYN_RECEIVED;

                            // Delete this SYN from the SYNQueue and compact the SYNQueue[] array
                            TCPRAMCopy(0, (PTR_BASE)&SYNQueue[w], TCP_PIC_RAM, (PTR_BASE)&SYNQueue[w+1], TCP_PIC_RAM, (TcpSynQueues-1u-w)*sizeof(TCP_SYN_QUEUE));
                            SYNQueue[TcpSynQueues-1].wDestPort = 0u;

                            break;
                        }
                    }
                }
			}
		#endif

		if(vFlags)
        {
			SendTCP(vFlags, bRetransmit ? 0 : SENDTCP_RESET_TIMERS);
        }

		// The TCP_CLOSED, TCP_LISTEN, and sometimes the TCP_ESTABLISHED 
		// state don't need any timeout events, so see if the timer is enabled
		if(!MyTCBStub.Flags.bTimerEnabled)
		{
			#if defined(TCP_KEEP_ALIVE_TIMEOUT)
				// Only the established state has any use for keep-alives
				if(MyTCBStub.smState == TCP_ESTABLISHED)
				{
					// If timeout has not occured, do not do anything.
					if(SYS_TICK_Get() < MyTCBStub.eventTime)
						continue;

					// If timeout has occured and the connection appears to be dead (no 
					// responses from remote node at all), close the connection so the 
					// application doesn't sit around indefinitely with a useless socket 
					// that it thinks is still open
					if(MyTCBStub.Flags.vUnackedKeepalives == TCP_MAX_UNACKED_KEEP_ALIVES)
					{
						vFlags = MyTCBStub.Flags.bServer;

						// Force an immediate FIN and RST transmission
						// Double calling TCPDisconnect() will also place us 
						// back in the listening state immediately if a server socket.
						TCPDisconnect(hTCP);
						TCPDisconnect(hTCP);

						// Prevent client mode sockets from getting reused by other applications.  
						// The application must call TCPDisconnect() with the handle to free this 
						// socket (and the handle associated with it)
						if(!vFlags)
							MyTCBStub.smState = TCP_CLOSED_BUT_RESERVED;

						continue;
					}

					// Otherwise, if a timeout occured, simply send a keep-alive packet
					SyncTCB();
					SendTCP(ACK, SENDTCP_KEEP_ALIVE);
					MyTCBStub.eventTime = SYS_TICK_Get() + (TCP_KEEP_ALIVE_TIMEOUT * SYS_TICK_TicksPerSecondGet())/1000;
				}
			#endif
			continue;
		}

		// If timeout has not occured, do not do anything.
		if(SYS_TICK_Get() < MyTCBStub.eventTime)
			continue;

		// Load up extended TCB information
		SyncTCB();
    
		// A timeout has occured.  Respond to this timeout condition
		// depending on what state this socket is in.
		switch(MyTCBStub.smState)
		{
			#if defined(TCPIP_STACK_CLIENT_MODE)
			#if defined(TCPIP_STACK_USE_DNS)
#if defined (TCPIP_STACK_USE_IPV6)
			case TCP_GET_DNS_MODULE_IPV6:
				if(DNSBeginUsage(0) == DNS_RES_OK)
				{
					MyTCBStub.smState = TCP_DNS_RESOLVE_IPV6;
                    DNSResolve((const char *)(PTR_BASE)MyTCB.remoteHost, DNS_TYPE_AAAA);
				}
				break;
		    case TCP_DNS_RESOLVE_IPV6:
    			{
    				IPV6_ADDR ipResolvedDNSIP;
                    DNS_RESULT dnsRes;

                    // See if the DNS resolution has finished
                    dnsRes = DNSIsResolved ((const char *)(PTR_BASE)MyTCB.remoteHost, &ipResolvedDNSIP);
                    if (dnsRes == DNS_RES_PENDING)
                    {
                        break;
                    }

                    DNSEndUsage(0);
                    if (dnsRes < 0)
                    {
                        // Some error has occured
                        MyTCBStub.eventTime = SYS_TICK_Get() + 10 * SYS_TICK_TicksPerSecondGet();
                        MyTCBStub.smState = TCP_GET_DNS_MODULE_IPV6;
                    }
                    else
                    {   // DNS_RES_OK
                            TCPIP_IPV6_SetDestAddress (MyTCBStub.pTxPkt, &ipResolvedDNSIP);
                            TCPIP_IP_SetPacketIPProtocol (MyTCBStub.pTxPkt, IP_ADDRESS_TYPE_IPV6);
    						MyTCBStub.remoteHash.Val = TCPIP_IPV6_GetHash(TCPIP_IPV6_GetDestAddress(MyTCBStub.pTxPkt), MyTCB.remotePort.Val, MyTCB.localPort.Val);
    						MyTCB.retryCount = 0;
    						MyTCB.retryInterval = (SYS_TICK_TicksPerSecondGet()/4);
                    }

    				// Send out SYN connection request to remote node
    				// This automatically disables the Timer from 
    				// continuously firing for this socket
    				vFlags = SYN;
    				bRetransmit = false;
					MyTCBStub.smState = TCP_SYN_SENT;
    				break;
    			}
                break;
#endif
			case TCP_GET_DNS_MODULE_IPV4:
				if(DNSBeginUsage(0) == DNS_RES_OK)
				{
					MyTCBStub.smState = TCP_DNS_RESOLVE_IPV4;
                    DNSResolve((char*)(PTR_BASE)MyTCB.remoteHost, DNS_TYPE_A);
				}
				break;
			case TCP_DNS_RESOLVE_IPV4:
			{
				IP_ADDR ipResolvedDNSIP;
                DNS_RESULT  dnsRes;

				// See if DNS resolution has finished.
				dnsRes = DNSIsResolved((const char*)(PTR_BASE)MyTCB.remoteHost, &ipResolvedDNSIP);
                
				if(dnsRes == DNS_RES_PENDING)
				{   // still waiting
                    break;
                }

                // DNS done
                DNSEndUsage(0);
				if(dnsRes < 0)
                {   // some error has occurred
                    MyTCBStub.eventTime = SYS_TICK_Get() + 10 * SYS_TICK_TicksPerSecondGet();
                    MyTCBStub.smState = TCP_GET_DNS_MODULE_IPV4;
                }
                else
                {   // DNS_RES_OK
                    TCPIP_IPV4_SetDestAddress (MyTCBStub.pTxPkt, ipResolvedDNSIP.Val);
                    TCPIP_IP_SetPacketIPProtocol (MyTCBStub.pTxPkt, IP_ADDRESS_TYPE_IPV4);
					MyTCBStub.smState = TCP_GATEWAY_SEND_ARP;
					MyTCBStub.remoteHash.Val = (TCPIP_IPV4_GetDestAddress(MyTCBStub.pTxPkt).w[1]+ TCPIP_IPV4_GetDestAddress(MyTCBStub.pTxPkt).w[0] + MyTCB.remotePort.Val) ^ MyTCB.localPort.Val;
					MyTCB.retryCount = 0;
					MyTCB.retryInterval = (SYS_TICK_TicksPerSecondGet()/4);
                }
				break;
			}
			#endif // #if defined(TCPIP_STACK_USE_DNS)
				
			case TCP_GATEWAY_SEND_ARP:
				// Obtain the MAC address associated with the server's IP address (either direct MAC address on same subnet, or the MAC address of the Gateway machine)
				MyTCBStub.eventTime2 = SYS_TICK_Get();
				ARPResolve(MyTCBStub.pSktNet, (IP_ADDR *)&(MyTCBStub.pTxPkt->ipHeader.ipv4Header.DestAddress));
				MyTCBStub.smState = TCP_GATEWAY_GET_ARP;
				break;

			case TCP_GATEWAY_GET_ARP:
				// Wait for the MAC address to finish being obtained
				if(!ARPIsResolved(MyTCBStub.pSktNet, (IP_ADDR *)&(MyTCBStub.pTxPkt->ipHeader.ipv4Header.DestAddress), &MyTCBStub.pTxPkt->remoteMACAddr))
				{
					// Time out if too much time is spent in this state
					// Note that this will continuously send out ARP 
					// requests for an infinite time if the Gateway 
					// never responds
					if(SYS_TICK_Get() - MyTCBStub.eventTime2 > MyTCB.retryInterval)
					{
						// Exponentially increase timeout until we reach 6 attempts then stay constant
						if(MyTCB.retryCount < 6u)
						{
							MyTCB.retryCount++;
							MyTCB.retryInterval <<= 1;
						}

						// Retransmit ARP request
						MyTCBStub.smState = TCP_GATEWAY_SEND_ARP;
					}
					break;
				}
				
				// Send out SYN connection request to remote node
				// This automatically disables the Timer from 
				// continuously firing for this socket
				vFlags = SYN;
				bRetransmit = false;
				MyTCBStub.smState = TCP_SYN_SENT;
				break;
			#endif // #if defined(TCPIP_STACK_CLIENT_MODE)
			
			case TCP_SYN_SENT:
				// Keep sending SYN until we hear from remote node.
				// This may be for infinite time, in that case
				// caller must detect it and do something.
				vFlags = SYN;
				bRetransmit = true;

// NOTE : This if statement was only in 5.36
				// Exponentially increase timeout until we reach TCP_MAX_RETRIES attempts then stay constant
				if(MyTCB.retryCount >= (TCP_MAX_RETRIES - 1))
				{
					MyTCB.retryCount = TCP_MAX_RETRIES - 1;
					MyTCB.retryInterval = ((TCP_START_TIMEOUT_VAL * SYS_TICK_TicksPerSecondGet())/1000) << (TCP_MAX_RETRIES-1);
				}
				break;
	
			case TCP_SYN_RECEIVED:
				// We must receive ACK before timeout expires.
				// If not, resend SYN+ACK.
				// Abort, if maximum attempts counts are reached.
				if(MyTCB.retryCount < TCP_MAX_SYN_RETRIES)
				{
					vFlags = SYN | ACK;
					bRetransmit = true;
				}
				else
				{
					if(MyTCBStub.Flags.bServer)
					{
						vFlags = RST | ACK;
						bCloseSocket = true;
					}
					else
					{
						vFlags = SYN;
					}
				}
				break;
	
			case TCP_ESTABLISHED:
			case TCP_CLOSE_WAIT:
				// Retransmit any unacknowledged data
				if(MyTCB.retryCount < TCP_MAX_RETRIES)
				{
					vFlags = ACK;
					bRetransmit = true;
				}
				else
				{
					// No response back for too long, close connection
					// This could happen, for instance, if the communication 
					// medium was lost
					MyTCBStub.smState = TCP_FIN_WAIT_1;
					vFlags = FIN | ACK;
				}
				break;
	
			case TCP_FIN_WAIT_1:
				if(MyTCB.retryCount < TCP_MAX_RETRIES)
				{
					// Send another FIN
					vFlags = FIN | ACK;
					bRetransmit = true;
				}
				else
				{
					// Close on our own, we can't seem to communicate 
					// with the remote node anymore
					vFlags = RST | ACK;
					bCloseSocket = true;
				}
				break;
	
			case TCP_FIN_WAIT_2:
				// Close on our own, we can't seem to communicate 
				// with the remote node anymore
				vFlags = RST | ACK;
				bCloseSocket = true;
				break;

			case TCP_CLOSING:
				if(MyTCB.retryCount < TCP_MAX_RETRIES)
				{
					// Send another ACK+FIN (the FIN is retransmitted 
					// automatically since it hasn't been acknowledged by 
					// the remote node yet)
					vFlags = ACK;
					bRetransmit = true;
				}
				else
				{
					// Close on our own, we can't seem to communicate 
					// with the remote node anymore
					vFlags = RST | ACK;
					bCloseSocket = true;
				}
				break;
	
//			case TCP_TIME_WAIT:
//				// Wait around for a while (2MSL) and then goto closed state
//				bCloseSocket = true;
//				break;
//			

			case TCP_LAST_ACK:
				// Send some more FINs or close anyway
				if(MyTCB.retryCount < TCP_MAX_RETRIES)
				{
					vFlags = FIN | ACK;
					bRetransmit = true;
				}
				else
				{
					vFlags = RST | ACK;
					bCloseSocket = true;
				}
				break;
			
			default:
				break;
		}

		if(vFlags)
		{
			// Transmit all unacknowledged data over again
			if(bRetransmit)
			{
				// Set the appropriate retry time
				MyTCB.retryCount++;
				MyTCB.retryInterval <<= 1;
		
				// Calculate how many bytes we have to roll back and retransmit
				w = MyTCB.txUnackedTail - MyTCBStub.txTail;
				if(MyTCB.txUnackedTail < MyTCBStub.txTail)
					w += MyTCBStub.bufferRxStart - MyTCBStub.bufferTxStart;
				
				// Perform roll back of local SEQuence counter, remote window 
				// adjustment, and cause all unacknowledged data to be 
				// retransmitted by moving the unacked tail pointer.
				MyTCB.MySEQ -= w;
				MyTCB.remoteWindow += w;
				MyTCB.txUnackedTail = MyTCBStub.txTail;		
				SendTCP(vFlags, 0);
			}
			else
            {
				SendTCP(vFlags, SENDTCP_RESET_TIMERS);
            }

		}
		
		if(bCloseSocket)
			CloseSocket();
	}


	#if TCP_SYN_QUEUEING
		// Process SYN Queue entry timeouts
		for(w = 0; w < TcpSynQueues; w++)
		{
			// Abort search if there are no more valid records
			if(SYNQueue[w].wDestPort == 0u)
				break;

			// See if this SYN has timed out
			if(SYS_TICK_Get() - SYNQueue[w].wTimestamp > (TCP_SYN_QUEUE_TIMEOUT * SYS_TICK_TicksPerSecondGet())/1000)
			{
				// Delete this SYN from the SYNQueue and compact the SYNQueue[] array
				TCPRAMCopy(0, (PTR_BASE)&SYNQueue[w], TCP_PIC_RAM, (PTR_BASE)&SYNQueue[w+1], TCP_PIC_RAM, (TcpSynQueues-1u-w)*sizeof(TCP_SYN_QUEUE));
				SYNQueue[TcpSynQueues-1].wDestPort = 0u;

				// Since we deleted an entry, we need to roll back one 
				// index so next loop will process the correct record
				w--;	
			}
		}
	#endif
        
    tcpTickPending = 0;
}


/*****************************************************************************
  Function:
	bool TCPProcess(NET_CONFIG* pPktIf, NODE_INFO* remote, IP_ADDR* localIP, uint16_t len)

  Summary:
  	Handles incoming TCP segments.

  Description:
	This function handles incoming TCP segments.  When a segment arrives, it
	is compared to open sockets using a hash of the remote port and IP.  
	On a match, the data is passed to HandleTCPSeg for further processing.

  Precondition:
	TCP is initialized and a TCP segment is ready in the MAC buffer.

  Parameters:
    pPktIf -  interface that has packets
    remote - Remote NODE_INFO structure
	localIP - This stack's IP address (for header checking)
	len - Total length of the waiting TCP segment

  Return Values:
	true - the segment was properly handled.
	false - otherwise
  ***************************************************************************/
bool TCPProcess(NET_CONFIG* pPktIf, NODE_INFO* remote, IP_ADDR* localIP, uint16_t len)
{
	TCP_HEADER      TCPHeader;
	PSEUDO_HEADER   pseudoHeader;
	TCPIP_UINT16_VAL        checksum1;
	TCPIP_UINT16_VAL        checksum2;
	uint8_t            optionsSize;
    TCPIP_MAC_HANDLE hMac;

	// Calculate IP pseudoheader checksum.
	pseudoHeader.SourceAddress      = remote->IPAddr;
	pseudoHeader.DestAddress        = *localIP;
	pseudoHeader.Zero               = 0x0;
	pseudoHeader.Protocol           = IP_PROT_TCP;
	pseudoHeader.Length          	= len;

	TCPIP_IPV4_SwapPseudoHeader(pseudoHeader);

	checksum1.Val = ~CalcIPChecksum((uint8_t*)&pseudoHeader,
		sizeof(pseudoHeader));

    hMac = _TCPIPStackNetToMac(pPktIf);
	// Now calculate TCP packet checksum in NIC RAM - should match
	// pesudo header checksum
	checksum2.Val = MACCalcIPBufferChecksum(hMac, len);

	// Compare checksums.
	if(checksum1.Val != checksum2.Val)
	{
		TCPIP_IP_DiscardRx(hMac);
		return true;
	}

#if defined(DEBUG_GENERATE_RX_LOSS)
	// Throw RX packets away randomly
	if(LFSRRand() > DEBUG_GENERATE_RX_LOSS)
	{
		TCPIP_IP_DiscardRx(hMac);
		return true;
	}
#endif

	// Retrieve TCP header.
	TCPIP_IPV4_SetRxBuffer(pPktIf, 0);
	TCPIP_IP_GetArray(hMac, (uint8_t*)&TCPHeader, sizeof(TCPHeader));
	SwapTCPHeader(&TCPHeader);


	// Skip over options to retrieve data bytes
	optionsSize = (uint8_t)((TCPHeader.DataOffset.Val << 2)-
		sizeof(TCPHeader));
	len = len - optionsSize - sizeof(TCPHeader);

	// Find matching socket.
	if(FindMatchingSocket(pPktIf, &TCPHeader, (void *)&(remote->IPAddr), localIP, &remote->MACAddr, IP_ADDRESS_TYPE_IPV4))
	{
		#if defined(TCPIP_STACK_USE_SSL)
		PTR_BASE prevRxHead;
		// For SSL connections, show HandleTCPSeg() the full data buffer
		prevRxHead = MyTCBStub.rxHead;
		if(MyTCBStub.sslStubID != SSL_INVALID_ID)
			MyTCBStub.rxHead = MyTCBStub.sslRxHead;
		#endif
		
		HandleTCPSeg(&TCPHeader, len);
		
		#if defined(TCPIP_STACK_USE_SSL)
		if(MyTCBStub.sslStubID != SSL_INVALID_ID)
		{
			// Restore the buffer state
			MyTCBStub.sslRxHead = MyTCBStub.rxHead;
			MyTCBStub.rxHead = prevRxHead;

			// Process the new SSL data, using the currently loaded stub
			TCPSSLHandleIncoming(hCurrentTCP);
		}
		#endif
	}
//	else
//	{
//		// NOTE: RFC 793 specifies that if the socket is closed and a segment 
//		// arrives, we should send back a RST if the RST bit in the incoming 
//		// packet is not set.  Instead, we will just silently ignore such a 
//		// packet since this is what firewalls do on purpose to enhance 
//		// security.
//		//if(!TCPHeader.Flags.bits.flagRST)
//		//	SendTCP(RST, SENDTCP_RESET_TIMERS);
//	}

	// Finished with this packet, discard it and free the Ethernet RAM for new packets
	TCPIP_IP_DiscardRx(hMac);

	return true;
}

#if defined (TCPIP_STACK_USE_IPV6)
bool TCPProcessIPv6(NET_CONFIG* pPktIf, IPV6_ADDR * localIP, IPV6_ADDR * remoteIP, uint16_t dataLen, uint16_t headerLen)
{
	TCP_HEADER      TCPHeader;
	IPV6_PSEUDO_HEADER   pseudoHeader;
	TCPIP_UINT16_VAL        checksum1;
	TCPIP_UINT16_VAL        checksum2;
	uint8_t            optionsSize;
    TCPIP_MAC_HANDLE hMac;

	// Calculate IP pseudoheader checksum.
    memcpy (&pseudoHeader.SourceAddress, remoteIP, sizeof (IPV6_ADDR));
    memcpy (&pseudoHeader.DestAddress, localIP, sizeof (IPV6_ADDR));
    // Total payload length is the length of data + extension headers
    pseudoHeader.PacketLength = swaps(dataLen);
    pseudoHeader.zero1 = 0;
    pseudoHeader.zero2 = 0;
    pseudoHeader.NextHeader = IP_PROT_TCP;

	checksum1.Val = ~CalcIPChecksum((uint8_t*)&pseudoHeader,
		sizeof(pseudoHeader));

    hMac = _TCPIPStackNetToMac(pPktIf);
    // Reset the RX buffer so the checksum will encompass the necessary headers
    TCPIP_IPV6_SetRxBuffer (pPktIf, 0);
	// Now calculate TCP packet checksum in NIC RAM - should match
	// pesudo header checksum
	checksum2.Val = MACCalcIPBufferChecksum(hMac, dataLen);

	// Compare checksums.
	if(checksum1.Val != checksum2.Val)
	{
		TCPIP_IP_DiscardRx(hMac);
		return true;
	}

#if defined(DEBUG_GENERATE_RX_LOSS)
	// Throw RX packets away randomly
	if(LFSRRand() > DEBUG_GENERATE_RX_LOSS)
	{
		TCPIP_IP_DiscardRx(hMac);
		return true;
	}
#endif

	// Retrieve TCP header.
	TCPIP_IPV6_SetRxBuffer(pPktIf, headerLen);
	TCPIP_IP_GetArray(hMac, (uint8_t*)&TCPHeader, sizeof(TCPHeader));
	SwapTCPHeader(&TCPHeader);

	// Skip over options to retrieve data bytes
	optionsSize = (uint8_t)((TCPHeader.DataOffset.Val << 2)-
		sizeof(TCPHeader));
	dataLen = dataLen - optionsSize - sizeof(TCPHeader);
	// Find matching socket.
	if(FindMatchingSocket(pPktIf, &TCPHeader, remoteIP, localIP, NULL, IP_ADDRESS_TYPE_IPV6))
	{
		#if defined(TCPIP_STACK_USE_SSL)
		PTR_BASE prevRxHead;
		// For SSL connections, show HandleTCPSeg() the full data buffer
		prevRxHead = MyTCBStub.rxHead;
		if(MyTCBStub.sslStubID != SSL_INVALID_ID)
			MyTCBStub.rxHead = MyTCBStub.sslRxHead;
		#endif
		
		HandleTCPSeg(&TCPHeader, dataLen);
		
		#if defined(TCPIP_STACK_USE_SSL)
		if(MyTCBStub.sslStubID != SSL_INVALID_ID)
		{
			// Restore the buffer state
			MyTCBStub.sslRxHead = MyTCBStub.rxHead;
			MyTCBStub.rxHead = prevRxHead;

			// Process the new SSL data, using the currently loaded stub
			TCPSSLHandleIncoming(hCurrentTCP);
		}
		#endif
	}
	else
	{
        // Send ICMP Destination Unreachable Code 4 (Port unreachable) and discard packet
        TCPIP_IPV6_SendError (pPktIf, localIP, remoteIP, ICMPV6_ERR_DU_PORT_UNREACHABLE, ICMPV6_ERROR_DEST_UNREACHABLE, 0x00000000, dataLen + headerLen + sizeof (IPV6_HEADER));
	}

	// Finished with this packet, discard it and free the Ethernet RAM for new packets
	TCPIP_IP_DiscardRx(hMac);

	return true;
}
#endif

/*****************************************************************************
  Function:
	static bool SendTCP(uint8_t vTCPFlags, uint8_t vSendFlags)

  Summary:
	Transmits a TPC segment.

  Description:
	This function assembles and transmits a TCP segment, including any 
	pending data.  It also supports retransmissions, keep-alives, and 
	other packet types.

  Precondition:
	TCP is initialized.

  Parameters:
	vTCPFlags - Additional TCP flags to include
	vSendFlags - Any combinations of SENDTCP_* constants to modify the
				 transmit behavior or contents.

  Returns:
	None
  ***************************************************************************/
static bool SendTCP(uint8_t vTCPFlags, uint8_t vSendFlags)
{
	TCPIP_UINT16_VAL        wVal;
	TCP_HEADER *    header;
	TCP_OPTIONS     options;
	uint16_t 			len, len2;
    TCPIP_MAC_HANDLE hMac;
    IP_PACKET * tempPkt;
	
	SyncTCB();

	//  Make sure that we can write to the MAC transmit area
    if(MyTCBStub.pSktNet == 0 || !TCPIP_IP_IsTxReady(MyTCBStub.pSktNet))
    {
        return false;
    }

	// FINs must be handled specially
	if(vTCPFlags & FIN)
	{
		MyTCBStub.Flags.bTXFIN = 1;
		vTCPFlags &= ~FIN;
	}

	// Status will now be synched, disable automatic future 
	// status transmissions
	MyTCBStub.Flags.bTimer2Enabled = 0;
	MyTCBStub.Flags.bDelayedACKTimerEnabled = 0;
	MyTCBStub.Flags.bOneSegmentReceived = 0;
	MyTCBStub.Flags.bTXASAP = 0;
	MyTCBStub.Flags.bTXASAPWithoutTimerReset = 0;
	MyTCBStub.Flags.bHalfFullFlush = 0;

    hMac = _TCPIPStackNetToMac(MyTCBStub.pSktNet);

    header = TCPIP_IP_GetUpperLayerHeaderPtr(MyTCBStub.pTxPkt);
	
	if (header != NULL) // Something was wrong...
	{

    header->DataOffset.Val = 0;

	// Put all socket application data in the TX space
	if(vTCPFlags & (SYN | RST))
	{
		// Don't put any data in SYN and RST messages
		len = 0;

    	// Insert the MSS (Maximum Segment Size) TCP option if this is SYN packet
    	if(vTCPFlags & SYN)
    	{
            if (TCPIP_IP_IsTxPutReady (MyTCBStub.pTxPkt, sizeof (options)) < sizeof (options))
            {
                return false;
            }
    		options.Kind = TCP_OPTIONS_MAX_SEG_SIZE;
    		options.Length = 0x04;
    
    		// Load MSS and swap to big endian
    		options.MaxSegSize.Val = (((TCP_MAX_SEG_SIZE_RX)&0x00FF)<<8) | (((TCP_MAX_SEG_SIZE_RX)&0xFF00)>>8);
    
    		header->DataOffset.Val   += sizeof(options) >> 2;
    
    		TCPIP_IP_PutArray(MyTCBStub.pTxPkt, (uint8_t*)&options, sizeof(options));
    	}
	}
	else
	{
		// Begin copying any application data over to the TX space
		if(MyTCBStub.txHead == MyTCB.txUnackedTail)
		{
			// All caught up on data TX, no real data for this packet
			len = 0;
		}
		else if(MyTCBStub.txHead > MyTCB.txUnackedTail)
		{
			len = MyTCBStub.txHead - MyTCB.txUnackedTail;
			if(len > MyTCB.remoteWindow)
            {
				len = MyTCB.remoteWindow;
            }

			if(len > MyTCB.wRemoteMSS)
			{
				len = MyTCB.wRemoteMSS;
				MyTCBStub.Flags.bTXASAPWithoutTimerReset = 1;
			}

            TCPIP_IP_SetPayload (MyTCBStub.pTxPkt, MyTCB.txUnackedTail, len);
			// Copy application data into the raw TX buffer
			MyTCB.txUnackedTail += len;
		}
		else
		{
			len2 = MyTCBStub.bufferRxStart - MyTCB.txUnackedTail;
			len = len2 + MyTCBStub.txHead - MyTCBStub.bufferTxStart;

			if(len > MyTCB.remoteWindow)
				len = MyTCB.remoteWindow;

			if(len > MyTCB.wRemoteMSS)
			{
				len = MyTCB.wRemoteMSS;
				MyTCBStub.Flags.bTXASAPWithoutTimerReset = 1;
			}

            if (len2 > len)
                len2 = len;

            TCPIP_IP_SetPayload (MyTCBStub.pTxPkt, MyTCB.txUnackedTail, len2);

			// Copy application data into the raw TX buffer
			len2 = len - len2;
	
			// Copy any left over chunks of application data over
			if(len2)
			{
                TCPIP_IP_SetPayload (MyTCBStub.pTxPkt, MyTCBStub.bufferTxStart, len2);
			}

			MyTCB.txUnackedTail += len;
			if(MyTCB.txUnackedTail >= MyTCBStub.bufferRxStart)
				MyTCB.txUnackedTail -= MyTCBStub.bufferRxStart-MyTCBStub.bufferTxStart;
		}

		// If we are to transmit a FIN, make sure we can put one in this packet
		if(MyTCBStub.Flags.bTXFIN)
		{
			if((len != MyTCB.remoteWindow) && (len != MyTCB.wRemoteMSS))
				vTCPFlags |= FIN;
		}
	}

	// Ensure that all packets with data of some kind are 
	// retransmitted by TCPTick() until acknowledged
	// Pure ACK packets with no data are not ACKed back in TCP
	if(len || (vTCPFlags & (SYN | FIN)))
	{
		// Transmitting data, update remote window variable to reflect smaller 
		// window.
		MyTCB.remoteWindow -= len;

		// Push (PSH) all data for enhanced responsiveness on 
		// the remote end, especially with GUIs
		if(len)
			vTCPFlags |= PSH;

		if(vSendFlags & SENDTCP_RESET_TIMERS)
		{
			MyTCB.retryCount = 0;
			MyTCB.retryInterval = (TCP_START_TIMEOUT_VAL * SYS_TICK_TicksPerSecondGet())/1000;
		}	

		MyTCBStub.eventTime = SYS_TICK_Get() + MyTCB.retryInterval;
		MyTCBStub.Flags.bTimerEnabled = 1;
	}
	else if(vSendFlags & SENDTCP_KEEP_ALIVE)
	{
		// Increment Keep Alive TX counter to handle disconnection if not response is returned
		MyTCBStub.Flags.vUnackedKeepalives++;
		
		// Generate a dummy byte
		MyTCB.MySEQ -= 1;
		len = 1;
	}
	else if(MyTCBStub.Flags.bTimerEnabled) 
	{
		// If we have data to transmit, but the remote RX window is zero, 
		// so we aren't transmitting any right now then make sure to not 
		// extend the retry counter or timer.  This will stall our TX 
		// with a periodic ACK sent to the remote node.
		if(!(vSendFlags & SENDTCP_RESET_TIMERS))
		{
			// Roll back retry counters since we can't send anything, 
			// but only if we incremented it in the first place
			if(MyTCB.retryCount)
			{
				MyTCB.retryCount--;
				MyTCB.retryInterval >>= 1;
			}
		}
	
		MyTCBStub.eventTime = SYS_TICK_Get() + MyTCB.retryInterval;
	}

	header->SourcePort			= MyTCB.localPort.Val;
	header->DestPort				= MyTCB.remotePort.Val;
	header->SeqNumber			= MyTCB.MySEQ;
	header->AckNumber			= MyTCB.RemoteSEQ;
	header->Flags.bits.Reserved2	= 0;
	header->DataOffset.Reserved3	= 0;
	header->Flags.byte			= vTCPFlags;
	header->UrgentPointer        = 0;

	// Update our send sequence number and ensure retransmissions 
	// of SYNs and FINs use the right sequence number
  	MyTCB.MySEQ += (uint32_t)len;
	if(vTCPFlags & SYN)
	{
        len += sizeof(options);

		// SEG.ACK needs to be zero for the first SYN packet for compatibility 
		// with certain paranoid TCP/IP stacks, even though the ACK flag isn't 
		// set (indicating that the AckNumber field is unused).
		if(!(vTCPFlags & ACK))
			header->AckNumber = 0x00000000;

		if(MyTCB.flags.bSYNSent)
			header->SeqNumber--;
		else
		{
			MyTCB.MySEQ++;
			MyTCB.flags.bSYNSent = 1;
		}
	}

	if(vTCPFlags & FIN)
	{
		if(MyTCB.flags.bFINSent)
			header->SeqNumber--;
		else
		{
			MyTCB.MySEQ++;
			MyTCB.flags.bFINSent = 1;
		}
	}

	// Calculate the amount of free space in the RX buffer area of this socket
	if(MyTCBStub.rxHead >= MyTCBStub.rxTail)
    {
		header->Window = (MyTCBStub.bufferEnd - MyTCBStub.bufferRxStart) - (MyTCBStub.rxHead - MyTCBStub.rxTail);
    }
	else
    {
		header->Window = MyTCBStub.rxTail - MyTCBStub.rxHead - 1;
    }

	// Calculate the amount of free space in the MAC RX buffer area and adjust window if needed
	wVal.Val = MACGetFreeRxSize(hMac);
	if(wVal.Val < 64)
    {
		wVal.Val = 0;
    }
    else
    {
		wVal.Val -= 64;
    }
	// Force the remote node to throttle back if we are running low on general RX buffer space
	if(header->Window > wVal.Val)
		header->Window = wVal.Val;

	SwapTCPHeader(header);


	len += sizeof(TCP_HEADER);
	header->DataOffset.Val   += sizeof(TCP_HEADER) >> 2;

	// Write IP header
	TCPIP_IP_PutHeader(MyTCBStub.pTxPkt, IP_PROT_TCP);

	// Update the TCP checksum
	// Calculate IP pseudoheader checksum.
    //header->Checksum = ~TCPIP_IP_GetPseudoHeaderChecksum (MyTCBStub.pTxPkt);
    //header->Checksum = TCPIP_IP_CalculatePayloadChecksum (MyTCBStub.pTxPkt);

#if defined(DEBUG_GENERATE_TX_LOSS)
	// Damage TCP checksums on TX packets randomly
	if(LFSRRand() > DEBUG_GENERATE_TX_LOSS)
	{
		header->Checksum++;
	}
#endif
        
	// Physically start the packet transmission over the network
    TCPIP_IP_Flush (MyTCBStub.pTxPkt, NULL);

    if (TCPIP_IP_IsPacketQueued(MyTCBStub.pTxPkt))
    {
        tempPkt = TCPAllocateTxPacketStruct (MyTCBStub.pSktNet, MyTCBStub.pTxPkt->flags.addressType);
        if (!TCPIP_IP_CopyTxPacketStruct (tempPkt, MyTCBStub.pTxPkt))
        {
            TCPIP_IP_FreePacket (tempPkt);
        }
        else
        {
            MyTCBStub.pTxPkt->flags.packetBound = false;
            MyTCBStub.pTxPkt = tempPkt;
        }
    }
    else
    {
        TCPIP_IP_ResetTransmitPacketState (MyTCBStub.pTxPkt);
    }

    return true;

	}

    return false;
}

/*****************************************************************************
  Function:
	static bool FindMatchingSocket(NET_CONFIG* pPktIf, TCP_HEADER* h, NODE_INFO* remote)

  Summary:
	Finds a suitable socket for a TCP segment.

  Description:
	This function searches through the sockets and attempts to match one with
	a given TCP header and NODE_INFO structure.  If a socket is found, its 
	index is saved in hCurrentTCP and the associated MyTCBStub and MyTCB are
	loaded. Otherwise, INVALID_SOCKET is placed in hCurrentTCP.
	
  Precondition:
	TCP is initialized.

  Parameters:
    pPktIf - interface the TCP packet/header belongs to  
	h - TCP header to be matched against
	remote - The remote node who sent this header

  Return Values:
	true - A match was found and is loaded in hCurrentTCP
	false - No suitable socket was found and hCurrentTCP is INVALID_SOCKET
  ***************************************************************************/
static bool FindMatchingSocket(NET_CONFIG* pPktIf, TCP_HEADER* h, void * remoteIP, void * localIP, MAC_ADDR * remoteMACAddr, IP_ADDRESS_TYPE addressType)
{
	TCP_SOCKET hTCP;
	TCP_SOCKET partialMatch;
	uint16_t hash;
    int  isSslSkt, isRegSkt, isForceBound;

	// Prevent connections on invalid port 0
	if(h->DestPort == 0u)
		return false;

	partialMatch = INVALID_SOCKET;
#if defined (TCPIP_STACK_USE_IPV6)
    if (addressType == IP_ADDRESS_TYPE_IPV6)
        hash = TCPIP_IPV6_GetHash(remoteIP,h->SourcePort,h->DestPort);
    else
#endif
    	hash = (((IP_ADDR *)remoteIP)->w[1] + ((IP_ADDR *)remoteIP)->w[0] + h->SourcePort) ^ h->DestPort;

	// Loop through all sockets looking for a socket that is expecting this 
	// packet or can handle it.
	for(hTCP = 0; hTCP < TcpSockets; hTCP++ )
	{
		SyncTCBStub(hTCP);

		if(MyTCBStub.smState == TCP_CLOSED)
		{
			continue;
		}
		else if(MyTCBStub.smState == TCP_LISTEN)
		{
            // For listening ports, check if this is the correct port
			if(MyTCBStub.remoteHash.Val == h->DestPort)
            {
				partialMatch = hTCP;
            }
			
			#if defined(TCPIP_STACK_USE_SSL_SERVER)
			// Check the SSL port as well for SSL Servers
			// 0 is defined as an invalid port number
			if(MyTCBStub.sslTxHead == h->DestPort)
				partialMatch = hTCP;
			#endif
			
			continue;
		}
		else if(MyTCBStub.remoteHash.Val != hash)
		{// Ignore if the hash doesn't match
			continue;
		}

		SyncTCB();
		if(	h->DestPort == MyTCB.localPort.Val &&
			h->SourcePort == MyTCB.remotePort.Val)
		{
#if defined (TCPIP_STACK_USE_IPV6)
            if (addressType == IP_ADDRESS_TYPE_IPV6)
            {
                if (MyTCBStub.pTxPkt->flags.addressType != IP_ADDRESS_TYPE_IPV6)
                {
                    continue;
                }
                if (memcmp (TCPIP_IPV6_GetDestAddress(MyTCBStub.pTxPkt), remoteIP, sizeof (IPV6_ADDR)))
                {
                    continue;
                }
            }
            else
#endif
            {
#if defined (TCPIP_STACK_USE_IPV6)
                if (MyTCBStub.pTxPkt->flags.addressType != IP_ADDRESS_TYPE_IPV4)
                {
                    continue;
                }
#endif
                if (TCPIP_IPV4_GetDestAddress(MyTCBStub.pTxPkt).Val != ((IP_ADDR *)remoteIP)->Val)
                {
                    continue;
                }
            }

            // got a client socket match
            if(MyTCBStub.pSktNet == 0)
            {   // unbound socket; bind it to this interface
                _TcpSocketBind(&MyTCBStub, pPktIf);
                TCPIP_IP_SetPacketIPProtocol (MyTCBStub.pTxPkt, addressType);
            }

            if (MyTCBStub.pSktNet == pPktIf)
            {
                return true;    // bound to the correct interface
            }
            // else find another
		}
	}


	// If there is a partial match, then a listening socket is currently 
	// available.  Set up the extended TCB with the info needed 
	// to establish a connection and return this socket to the 
	// caller.
	if(partialMatch != INVALID_SOCKET)
	{
        for(hTCP = 0; hTCP < TcpSockets; hTCP++ )
        {
            SyncTCBStub(hTCP);

            if(MyTCBStub.smState != TCP_LISTEN)
            {
                continue;
            }
            isSslSkt = isRegSkt = 0;
            #if defined(TCPIP_STACK_USE_SSL_SERVER)
            // Check the SSL port as well for SSL Servers
            // 0 is defined as an invalid port number
            if(MyTCBStub.sslTxHead == h->DestPort)
            {
                isSslSkt = 1;   // OK, it's a SSL socket
            }
            #endif
            
            if(isSslSkt == 0)
            {   // try a regular socket
                if(MyTCBStub.remoteHash.Val == h->DestPort)
                {
                    isRegSkt = 1;
                }
            }

            if(isSslSkt==0 && isRegSkt == 0)
            {
                continue;   // couldn't find a match, try another socket
            }
            
            if(MyTCBStub.pSktNet == 0)
            {   // unbound socket; bind it to this interface
                _TcpSocketBind(&MyTCBStub, pPktIf);
                isForceBound = 1;    // if the SSL connection fails we'll have to unbound!
            }
            else if (MyTCBStub.pSktNet != pPktIf)
            {   // make sure is bound to the correct interface
                continue;   // find another one
            }
            else
            {
                isForceBound = 0;
            }

            #if defined(TCPIP_STACK_USE_SSL_SERVER)
            // Check the SSL port as well for SSL Servers
            // 0 is defined as an invalid port number
            if(isSslSkt)
            {
                // Try to start an SSL session.  If no stubs are available,
                // we can't service this request right now, so ignore it.

                if(!TCPStartSSLServer(hTCP))
                {
                    if(isForceBound)
                    {
                        _TcpSocketBind(&MyTCBStub, 0);  // remove the binding
                    }
                    continue;
                }
            }
            #endif
			
            // OK, found a server socket that can handle this packet
        
            SyncTCB();
	            
            MyTCBStub.remoteHash.Val = hash;

            TCPIP_IP_SetPacketIPProtocol (MyTCBStub.pTxPkt, addressType);

#if defined (TCPIP_STACK_USE_IPV6)
            if (addressType == IP_ADDRESS_TYPE_IPV6)
            {
                TCPIP_IPV6_SetDestAddress (MyTCBStub.pTxPkt, remoteIP);
                TCPIP_IPV6_SetSourceAddress(MyTCBStub.pTxPkt, &(((IPV6_ADDR_STRUCT *)localIP)->address));
            }
            else
#endif
            {
                TCPIP_IPV4_SetDestAddress (MyTCBStub.pTxPkt, ((IP_ADDR *)remoteIP)->Val);
                memcpy((void*)&MyTCBStub.pTxPkt->remoteMACAddr, (void*)remoteMACAddr, sizeof(MAC_ADDR));
            }
            MyTCB.remotePort.Val = h->SourcePort;
            MyTCB.localPort.Val = h->DestPort;
            MyTCB.txUnackedTail	= MyTCBStub.bufferTxStart;

            // All done, and we have a match
            return true;
		}
	}

	// No available sockets are listening on this port.  (Or, for
	// SSL requests, perhaps no SSL sessions were available.  However,
	// there may be a server socket which is currently busy but 
	// could handle this packet, so we should check.
	#if TCP_SYN_QUEUEING
	{
		uint16_t wQueueInsertPos;
		
		// See if this is a SYN packet
		if(!h->Flags.bits.flagSYN)
			return false;

		// See if there is space in our SYN queue
		if(SYNQueue[TcpSynQueues-1].wDestPort)
			return false;
		
		// See if we have this SYN already in our SYN queue.
		// If not already in the queue, find out where we 
		// should insert this SYN to the queue
		for(wQueueInsertPos = 0; wQueueInsertPos < TcpSynQueues; wQueueInsertPos++)
		{
			// Exit loop if we found a free record
			if(SYNQueue[wQueueInsertPos].wDestPort == 0u)
				break;

			// Check if this SYN packet is already in the SYN queue
			if(SYNQueue[wQueueInsertPos].wDestPort != h->DestPort)
				continue;
			if(SYNQueue[wQueueInsertPos].wSourcePort != h->SourcePort)
				continue;
            if (addressType == IP_ADDRESS_TYPE_IPV4)
            {
                if (SYNQueue[wQueueInsertPos].addressType == IP_ADDRESS_TYPE_IPV4)
                {
        			if(SYNQueue[wQueueInsertPos].sourceAddress.sourceIPv4.Val != ((IP_ADDR *)remoteIP)->Val)
    	    			continue;
                }
            }
            else
            {
                if (SYNQueue[wQueueInsertPos].addressType == IP_ADDRESS_TYPE_IPV6)
                {
                    if (memcmp (&SYNQueue[wQueueInsertPos].sourceAddress.sourceIPv6, remoteIP, sizeof (IPV6_ADDR)))
    	    			continue;
                }
            }

			// SYN matches SYN queue entry.  Update timestamp and do nothing.
			SYNQueue[wQueueInsertPos].wTimestamp = SYS_TICK_Get();
			return false;
		}
		
		// Check to see if we have any server sockets which 
		// are currently connected, but could handle this SYN 
		// request at a later time if the client disconnects.
		for(hTCP = 0; hTCP < TcpSockets; hTCP++)
		{
			SyncTCBStub(hTCP);
			if(!MyTCBStub.Flags.bServer)
				continue;

			SyncTCB();
			#if defined(TCPIP_STACK_USE_SSL_SERVER)
			if((MyTCB.localPort.Val != h->DestPort) && (MyTCB.localSSLPort.Val != h->DestPort))
			#else
			if(MyTCB.localPort.Val != h->DestPort)
			#endif
				continue;

			// Generate the SYN queue entry
            SYNQueue[wQueueInsertPos].addressType = addressType;
            if (addressType == IP_ADDRESS_TYPE_IPV4)
            {
                SYNQueue[wQueueInsertPos].sourceAddress.sourceIPv4.Val = ((IP_ADDR *)remoteIP)->Val;
    			memcpy((void*)&SYNQueue[wQueueInsertPos].remoteMACAddr, (void*)remoteMACAddr, sizeof(MAC_ADDR));            
            }
            else
            {
                memcpy (&SYNQueue[wQueueInsertPos].sourceAddress.sourceIPv6, remoteIP, sizeof (IPV6_ADDR));
                memcpy (&SYNQueue[wQueueInsertPos].destAddr, localIP, sizeof (IPV6_ADDR));
            }
			SYNQueue[wQueueInsertPos].wSourcePort = h->SourcePort;
			SYNQueue[wQueueInsertPos].dwSourceSEQ = h->SeqNumber;
			SYNQueue[wQueueInsertPos].wDestPort = h->DestPort;
			SYNQueue[wQueueInsertPos].wTimestamp = SYS_TICK_Get();
			SYNQueue[wQueueInsertPos].pSynIf = pPktIf;

			return false;
		}
	}
	#endif
		
	return false;

}



/*****************************************************************************
  Function:
	static void SwapTCPHeader(TCP_HEADER* header)

  Summary:
	Swaps endian-ness of a TCP header.

  Description:
	This function swaps the endian-ness of a given TCP header for comparison.

  Precondition:
	None

  Parameters:
	header - The TCP header that is to be swapped

  Returns:
	None
  ***************************************************************************/
static void SwapTCPHeader(TCP_HEADER* header)
{
	header->SourcePort      = swaps(header->SourcePort);
	header->DestPort        = swaps(header->DestPort);
	header->SeqNumber       = swapl(header->SeqNumber);
	header->AckNumber       = swapl(header->AckNumber);
	header->Window          = swaps(header->Window);
	header->Checksum        = swaps(header->Checksum);
	header->UrgentPointer   = swaps(header->UrgentPointer);
}



/*****************************************************************************
  Function:
	static void CloseSocket(void)

  Summary:
	Closes a TCP socket.

  Description:
	This function closes a TCP socket.  All socket state information is 
	reset, and any buffered bytes are discarded.  The socket is no longer
	accessible by the application after this point.

  Precondition:
	The TCPStub corresponding to the socket to be closed is synced.

  Parameters:
	None

  Returns:
	None
  ***************************************************************************/
static void CloseSocket(void)
{
	SyncTCB();

	MyTCBStub.remoteHash.Val = MyTCB.localPort.Val;
	MyTCBStub.txHead = MyTCBStub.bufferTxStart;
	MyTCBStub.txTail = MyTCBStub.bufferTxStart;
	MyTCBStub.rxHead = MyTCBStub.bufferRxStart;
	MyTCBStub.rxTail = MyTCBStub.bufferRxStart;
	MyTCBStub.Flags.vUnackedKeepalives = 0;
	MyTCBStub.Flags.bTimerEnabled = 0;
	MyTCBStub.Flags.bTimer2Enabled = 0;
	MyTCBStub.Flags.bDelayedACKTimerEnabled = 0;
	MyTCBStub.Flags.bOneSegmentReceived = 0;
	MyTCBStub.Flags.bHalfFullFlush = 0;
	MyTCBStub.Flags.bTXASAP = 0;
	MyTCBStub.Flags.bTXASAPWithoutTimerReset = 0;
	MyTCBStub.Flags.bTXFIN = 0;
	MyTCBStub.Flags.bSocketReset = 1;

	#if defined(TCPIP_STACK_USE_SSL)
	// If SSL is active, then we need to close it
	if(MyTCBStub.sslStubID != SSL_INVALID_ID)
	{
		SSLTerminate(hCurrentTCP, MyTCBStub.sslStubID);
		MyTCBStub.sslStubID = SSL_INVALID_ID;

		// Swap the SSL port and local port back to proper values
		MyTCBStub.remoteHash.Val = MyTCB.localSSLPort.Val;
		MyTCB.localSSLPort.Val = MyTCB.localPort.Val;
		MyTCB.localPort.Val = MyTCBStub.remoteHash.Val;
	}

	// Reset the SSL buffer pointers
	MyTCBStub.sslRxHead = MyTCBStub.bufferRxStart;
	MyTCBStub.sslTxHead = MyTCBStub.bufferTxStart;
	#endif
	
	#if defined(TCPIP_STACK_USE_SSL_SERVER)
	MyTCBStub.sslTxHead = MyTCB.localSSLPort.Val;
	#endif

	MyTCB.flags.bFINSent = 0;
	MyTCB.flags.bSYNSent = 0;
	MyTCB.flags.bRXNoneACKed1 = 0;
	MyTCB.flags.bRXNoneACKed2 = 0;
	MyTCB.txUnackedTail = MyTCBStub.bufferTxStart;
	((TCPIP_UINT32_VAL*)(&MyTCB.MySEQ))->w[0] = LFSRRand();
	((TCPIP_UINT32_VAL*)(&MyTCB.MySEQ))->w[1] = LFSRRand();
	MyTCB.sHoleSize = -1;
	MyTCB.remoteWindow = 1;

    MyTCBStub.pSktNet = 0;
    if (MyTCBStub.pTxPkt != NULL)
    {
        if (!TCPIP_IP_IsPacketQueued(MyTCBStub.pTxPkt))
        {
            TCPIP_IP_ResetTransmitPacketState (MyTCBStub.pTxPkt);
        }
    }
	MyTCBStub.smState = MyTCBStub.Flags.bServer ? TCP_LISTEN : TCP_CLOSED;
}


/*****************************************************************************
  Function:
	static uint16_t GetMaxSegSizeOption(NET_CONFIG* pNet)

  Summary:
	Obtains the Maximum Segment Size (MSS) TCP Option out of the TCP header 
	for the current socket.

  Description:
	Parses the current TCP packet header and extracts the Maximum Segment Size 
	option.  

  Precondition:
	Must be called while a TCP packet is present and being processed via 
	HandleTCPSeg() and only if the the TCP SYN flag is set.

  Parameters:
	hMac   - MAC interface for multi-homed hosts

  Returns:
	Maximum segment size option value.  If illegal or not present, a failsafe 
	value of 536 is returned.  If the option is larger than the 
	TCP_MAX_SEG_SIZE_TX upper limit, then TCP_MAX_SEG_SIZE_TX is returned.

  Remarks:
	The internal MAC Read Pointer is moved but not restored.
  ***************************************************************************/
static uint16_t GetMaxSegSizeOption(NET_CONFIG* pNet)
{
	uint8_t vOptionsBytes;
	uint8_t vOption;
	uint16_t wMSS;
	TCPIP_MAC_HANDLE hMac;

    hMac = _TCPIPStackNetToMac(pNet);
    // Find out how many options bytes are in this packet.
	TCPIP_IPV4_SetRxBuffer(pNet, 2+2+4+4);	// Seek to data offset field, skipping Source port (2), Destination port (2), Sequence number (4), and Acknowledgement number (4)
	vOptionsBytes = MACGet(hMac);
	vOptionsBytes = ((vOptionsBytes&0xF0)>>2) - sizeof(TCP_HEADER);

	// Return minimum Maximum Segment Size value of 536 bytes if none are 
	// present
	if(vOptionsBytes == 0u)
		return 536;
		
	// Seek to beginning of options
	MACGetArray(hMac, NULL, 7);

	// Search for the Maximum Segment Size option	
	while(vOptionsBytes--)
	{
		vOption = MACGet(hMac);
		
		if(vOption == 0u)	// End of Options list
			break;
		
		if(vOption == 1u)	// NOP option
			continue;
			
		if(vOption == 2u)	// Maximum Segment Size option
		{
			if(vOptionsBytes < 3u)
				break;

			wMSS = 0;
				
			// Get option length
			vOption = MACGet(hMac);
			if(vOption == 4u)
			{// Retrieve MSS and swap value to little endian
				((uint8_t*)&wMSS)[1] = MACGet(hMac);
				((uint8_t*)&wMSS)[0] = MACGet(hMac);
			}
			
			if(wMSS < 536u)
				break;
			if(wMSS > TCP_MAX_SEG_SIZE_TX)
				return TCP_MAX_SEG_SIZE_TX;
			else 
				return wMSS;
		}
		else
		{ // Assume this is a multi byte option and throw it way
			if(vOptionsBytes < 2u)
				break;
			vOption = MACGet(hMac);
			if(vOptionsBytes < vOption)
				break;
			MACGetArray(hMac, NULL, vOption);
			vOptionsBytes -= vOption;
		}
		
	}
	
	// Did not find MSS option, return worst case default
	return 536;
}

/*****************************************************************************
  Function:
	static void HandleTCPSeg(TCP_HEADER* h, uint16_t len)

  Summary:
	Processes an incoming TCP segment.

  Description:
	Once an incoming segment has been matched to a socket, this function
	performs the necessary processing with the data.  Depending on the 
	segment and the state, this may include copying data to the TCP buffer,
	re-assembling out-of order packets, continuing an initialization or 
	closing handshake, or closing the socket altogether.

  Precondition:
	TCP is initialized and the current TCP stub is already synced.

  Parameters:
	h - The TCP header for this packet
	len - The total buffer length of this segment

  Returns:
	None
  ***************************************************************************/
static void HandleTCPSeg(TCP_HEADER* h, uint16_t len)
{
	uint32_t dwTemp;
	PTR_BASE wTemp;
	int32_t lMissingBytes;
	uint16_t wMissingBytes;
	uint16_t wFreeSpace;
	uint8_t localHeaderFlags;
	uint32_t localAckNumber;
	uint32_t localSeqNumber;
	uint16_t wSegmentLength;
	bool bSegmentAcceptable;
	uint16_t wNewWindow;


	// Cache a few variables in local RAM.  
	// PIC18s take a fair amount of code and execution time to 
	// dereference pointers frequently.
	localHeaderFlags = h->Flags.byte;
	localAckNumber = h->AckNumber;
	localSeqNumber = h->SeqNumber;

	// We received a packet, reset the keep alive timer and count
	#if defined(TCP_KEEP_ALIVE_TIMEOUT)
		MyTCBStub.Flags.vUnackedKeepalives = 0;
		if(!MyTCBStub.Flags.bTimerEnabled)
			MyTCBStub.eventTime = SYS_TICK_Get() + (TCP_KEEP_ALIVE_TIMEOUT * SYS_TICK_TicksPerSecondGet())/1000;
	#endif

	// Handle TCP_LISTEN and TCP_SYN_SENT states
	// Both of these states will return, so code following this 
	// state machine need not check explicitly for these two 
	// states.
	switch(MyTCBStub.smState)
	{
		case TCP_LISTEN:
			// First: check RST flag
			if(localHeaderFlags & RST)
			{
				CloseSocket();	// Unbind remote IP address/port info
				return;
			}

			// Second: check ACK flag, which would be invalid
			if(localHeaderFlags & ACK)
			{
				// Use a believable sequence number and reset the remote node
				MyTCB.MySEQ = localAckNumber;
				SendTCP(RST, 0);
				CloseSocket();	// Unbind remote IP address/port info
				return;
			}

			// Third: check for SYN flag, which is what we're looking for
			if(localHeaderFlags & SYN)
			{
				// We now have a sequence number for the remote node
				MyTCB.RemoteSEQ = localSeqNumber + 1;

				// Get MSS option
				MyTCB.wRemoteMSS = GetMaxSegSizeOption(MyTCBStub.pSktNet);

				// Set Initial Send Sequence (ISS) number
				// Nothing to do on this step... ISS already set in CloseSocket()
				
				// Respond with SYN + ACK
				SendTCP(SYN | ACK, SENDTCP_RESET_TIMERS);
				MyTCBStub.smState = TCP_SYN_RECEIVED;
			}
			else
			{
				CloseSocket();	// Unbind remote IP address/port info
			}

			// Fourth: check for other text and control
			// Nothing to do since we don't support this
			return;

		case TCP_SYN_SENT:
			// Second: check the RST bit
			// This is out of order because this stack has no API for 
			// notifying the application that the connection seems to 
			// be failing.  Instead, the application must time out and 
			// the stack will just keep trying in the mean time.
			if(localHeaderFlags & RST)
				return;

			// First: check ACK bit
			if(localHeaderFlags & ACK)
			{
				if(localAckNumber != MyTCB.MySEQ)
				{
					// Send a RST packet with SEQ = SEG.ACK, but retain our SEQ 
					// number for arivial of any other SYN+ACK packets
					localSeqNumber = MyTCB.MySEQ;	// Save our original SEQ number
					MyTCB.MySEQ = localAckNumber;	// Set SEQ = SEG.ACK
					SendTCP(RST, SENDTCP_RESET_TIMERS);		// Send the RST
					MyTCB.MySEQ = localSeqNumber;	// Restore original SEQ number
					return;
				}
			}

#if defined (TCPIP_STACK_USE_IPV6)
            if (MyTCBStub.pTxPkt->flags.addressType == IP_ADDRESS_TYPE_IPV6)
            {
                TCPIP_NDP_NeighborConfirmReachability (MyTCBStub.pSktNet, TCPIP_IPV6_GetDestAddress(MyTCBStub.pTxPkt));
            }
#endif

			// Third: check the security and precedence
			// No such feature in this stack.  We want to accept all connections.

			// Fourth: check the SYN bit
			if(localHeaderFlags & SYN)
			{
				// We now have an initial sequence number and window size
				MyTCB.RemoteSEQ = localSeqNumber + 1;
				MyTCB.remoteWindow = h->Window;

				// Get MSS option
				MyTCB.wRemoteMSS = GetMaxSegSizeOption(MyTCBStub.pSktNet);

				if(localHeaderFlags & ACK)
				{
					SendTCP(ACK, SENDTCP_RESET_TIMERS);
					MyTCBStub.smState = TCP_ESTABLISHED;
					// Set up keep-alive timer
					#if defined(TCP_KEEP_ALIVE_TIMEOUT)
						MyTCBStub.eventTime = SYS_TICK_Get() + (TCP_KEEP_ALIVE_TIMEOUT * SYS_TICK_TicksPerSecondGet())/1000;
					#endif
					MyTCBStub.Flags.bTimerEnabled = 0;
				}
				else
				{
					SendTCP(SYN | ACK, SENDTCP_RESET_TIMERS);
					MyTCBStub.smState = TCP_SYN_RECEIVED;
				}
			}

			// Fifth: drop the segment if neither SYN or RST is set
			return;

		default:
			break;
	}

	//
	// First: check the sequence number
	//
	wSegmentLength = len;
	if(localHeaderFlags & FIN)
		wSegmentLength++;
	if(localHeaderFlags & SYN)
		wSegmentLength++;

	// Calculate the RX FIFO space
	if(MyTCBStub.rxHead >= MyTCBStub.rxTail)
		wFreeSpace = (MyTCBStub.bufferEnd - MyTCBStub.bufferRxStart) - (MyTCBStub.rxHead - MyTCBStub.rxTail);
	else
		wFreeSpace = MyTCBStub.rxTail - MyTCBStub.rxHead - 1;

	// Calculate the number of bytes ahead of our head pointer this segment skips
	lMissingBytes = localSeqNumber - MyTCB.RemoteSEQ;
	wMissingBytes = (uint16_t)lMissingBytes;
	
	// Run TCP acceptability tests to verify that this packet has a valid sequence number
	bSegmentAcceptable = false;
	if(wSegmentLength)
	{
		// Check to see if we have free space, and if so, if any of the data falls within the freespace
		if(wFreeSpace)
		{
			// RCV.NXT =< SEG.SEQ < RCV.NXT+RCV.WND
			if((lMissingBytes >= (int32_t)0) && (wFreeSpace > (uint32_t)lMissingBytes))
				bSegmentAcceptable = true;
			else
			{
				// RCV.NXT =< SEG.SEQ+SEG.LEN-1 < RCV.NXT+RCV.WND
				if((lMissingBytes + (int32_t)wSegmentLength > (int32_t)0) && (lMissingBytes <= (int32_t)(int16_t)(wFreeSpace - wSegmentLength)))
					bSegmentAcceptable = true;
			}
			
			if((lMissingBytes < (int32_t)wFreeSpace) && ((int16_t)wMissingBytes + (int16_t)wSegmentLength > (int16_t)0))
				bSegmentAcceptable = true;
		}
		// Segments with data are not acceptable if we have no free space
	}
	else
	{
		// Zero length packets are acceptable if they fall within our free space window
		// SEG.SEQ = RCV.NXT
		if(lMissingBytes == 0)
		{
			bSegmentAcceptable = true;
		}
		else
		{
			// RCV.NXT =< SEG.SEQ < RCV.NXT+RCV.WND
			if((lMissingBytes >= (int32_t)0) && (wFreeSpace > (uint32_t)lMissingBytes))
				bSegmentAcceptable = true;
		}
	}
	
	if(!bSegmentAcceptable)
	{
		// Unacceptable segment, drop it and respond appropriately
		if(!(localHeaderFlags & RST)) 
			SendTCP(ACK, SENDTCP_RESET_TIMERS);
		return;
	}


	//
	// Second: check the RST bit
	//
	//
	// Fourth: check the SYN bit
	//
	// Note, that since the third step is not implemented, we can 
	// combine this second and fourth step into a single operation.
	if(localHeaderFlags & (RST | SYN))
	{
		CloseSocket();
		return;
	}

	//
	// Third: check the security and precedence
	//
	// Feature not supported.  Let's process this segment.

	//
	// Fifth: check the ACK bit
	//
	if(!(localHeaderFlags & ACK))
		return;

#if defined (TCPIP_STACK_USE_IPV6)
    // If we've received an ACK, update neighbor reachability
    if (MyTCBStub.pTxPkt->flags.addressType == IP_ADDRESS_TYPE_IPV6)
    {
        TCPIP_NDP_NeighborConfirmReachability (MyTCBStub.pSktNet, TCPIP_IPV6_GetDestAddress(MyTCBStub.pTxPkt));
    }
#endif

	switch(MyTCBStub.smState)
	{
		case TCP_SYN_RECEIVED:
			if(localAckNumber != MyTCB.MySEQ)
			{
				// Send a RST packet with SEQ = SEG.ACK, but retain our SEQ 
				// number for arivial of any other correct packets
				localSeqNumber = MyTCB.MySEQ;	// Save our original SEQ number
				MyTCB.MySEQ = localAckNumber;	// Set SEQ = SEG.ACK
				SendTCP(RST, SENDTCP_RESET_TIMERS);		// Send the RST
				MyTCB.MySEQ = localSeqNumber;	// Restore original SEQ number
				return;
			}
			MyTCBStub.smState = TCP_ESTABLISHED;
			// No break

		case TCP_ESTABLISHED:
		case TCP_FIN_WAIT_1:
		case TCP_FIN_WAIT_2:
		case TCP_CLOSE_WAIT:
		case TCP_CLOSING:
			// Calculate what the highest possible SEQ number in our TX FIFO is
			wTemp = MyTCBStub.txHead - MyTCB.txUnackedTail;
			if((int16_t)wTemp < (int16_t)0)
				wTemp += MyTCBStub.bufferRxStart - MyTCBStub.bufferTxStart;
			dwTemp = MyTCB.MySEQ + (uint32_t)wTemp;

			// Drop the packet if it ACKs something we haven't sent
			if((int32_t)(dwTemp - localAckNumber) < (int32_t)0)
			{
				SendTCP(ACK, 0);
				return;
			}

			// Throw away all ACKnowledged TX data:
			// Calculate what the last acknowledged sequence number was (ignoring any FINs we sent)
			dwTemp = MyTCB.MySEQ - (int32_t)(int16_t)(MyTCB.txUnackedTail - MyTCBStub.txTail);
			if(MyTCB.txUnackedTail < MyTCBStub.txTail)
				dwTemp -= MyTCBStub.bufferRxStart - MyTCBStub.bufferTxStart;
	
			// Calcluate how many bytes were ACKed with this packet
			dwTemp = localAckNumber - dwTemp;
			if(((int32_t)(dwTemp) > (int32_t)0) && (dwTemp <= MyTCBStub.bufferRxStart - MyTCBStub.bufferTxStart))
			{
				MyTCB.flags.bRXNoneACKed1 = 0;
				MyTCB.flags.bRXNoneACKed2 = 0;
				MyTCBStub.Flags.bHalfFullFlush = false;
	
				// Bytes ACKed, free up the TX FIFO space
				wTemp = MyTCBStub.txTail;
				MyTCBStub.txTail += dwTemp;
				if(MyTCB.txUnackedTail >= wTemp)
				{
					if(MyTCB.txUnackedTail < MyTCBStub.txTail)
					{
						MyTCB.MySEQ += MyTCBStub.txTail - MyTCB.txUnackedTail;
						MyTCB.txUnackedTail = MyTCBStub.txTail;
					}
				}
				else
				{
					wTemp = MyTCB.txUnackedTail + (MyTCBStub.bufferRxStart - MyTCBStub.bufferTxStart);
					if(wTemp < MyTCBStub.txTail)
					{
						MyTCB.MySEQ += MyTCBStub.txTail - wTemp;
						MyTCB.txUnackedTail = MyTCBStub.txTail;
					}
				}
				if(MyTCBStub.txTail >= MyTCBStub.bufferRxStart)
					MyTCBStub.txTail -= MyTCBStub.bufferRxStart - MyTCBStub.bufferTxStart;
				if(MyTCB.txUnackedTail >= MyTCBStub.bufferRxStart)
					MyTCB.txUnackedTail -= MyTCBStub.bufferRxStart - MyTCBStub.bufferTxStart;
			}
			else
			{
				// See if we have outstanding TX data that is waiting for an ACK
				if(MyTCBStub.txTail != MyTCB.txUnackedTail)
				{
					if(MyTCB.flags.bRXNoneACKed1)
					{
						if(MyTCB.flags.bRXNoneACKed2)
						{
							// Set up to perform a fast retransmission
							// Roll back unacknowledged TX tail pointer to cause retransmit to occur
							MyTCB.MySEQ -= (int32_t)(int16_t)(MyTCB.txUnackedTail - MyTCBStub.txTail);
							if(MyTCB.txUnackedTail < MyTCBStub.txTail)
								MyTCB.MySEQ -= (int32_t)(int16_t)(MyTCBStub.bufferRxStart - MyTCBStub.bufferTxStart);
							MyTCB.txUnackedTail = MyTCBStub.txTail;
							MyTCBStub.Flags.bTXASAPWithoutTimerReset = 1;
						}
						MyTCB.flags.bRXNoneACKed2 = 1;
					}
					MyTCB.flags.bRXNoneACKed1 = 1;
				}
			}

			// No need to keep our retransmit timer going if we have nothing that needs ACKing anymore
			if(MyTCBStub.txTail == MyTCBStub.txHead)
			{
				// Make sure there isn't a "FIN byte in our TX FIFO"
				if(MyTCBStub.Flags.bTXFIN == 0u)
				{
					// Convert retransmission timer to keep-alive timer
					#if defined(TCP_KEEP_ALIVE_TIMEOUT)
						MyTCBStub.eventTime = SYS_TICK_Get() + (TCP_KEEP_ALIVE_TIMEOUT * SYS_TICK_TicksPerSecondGet())/1000;
					#endif
					MyTCBStub.Flags.bTimerEnabled = 0;
				}
				else
				{
					// "Throw away" FIN byte from our TX FIFO if it has been ACKed
					if((MyTCB.MySEQ == localAckNumber) && MyTCB.flags.bFINSent)
					{
						MyTCBStub.Flags.bTimerEnabled = 0;
						MyTCBStub.Flags.bTXFIN = 0;
					}
				}
			}

			// The window size advirtised in this packet is adjusted to account 
			// for any bytes that we have transmitted but haven't been ACKed yet 
			// by this segment.
			wNewWindow = h->Window - ((uint16_t)(MyTCB.MySEQ - localAckNumber));

			// Update the local stored copy of the RemoteWindow.
			// If previously we had a zero window, and now we don't, then 
			// immediately send whatever was pending.
			if((MyTCB.remoteWindow == 0u) && wNewWindow)
				MyTCBStub.Flags.bTXASAP = 1;
			MyTCB.remoteWindow = wNewWindow;

			// A couple of states must do all of the TCP_ESTABLISHED stuff, but also a little more
			if(MyTCBStub.smState == TCP_FIN_WAIT_1)
			{
				// Check to see if our FIN has been ACKnowledged
				if((MyTCB.MySEQ == localAckNumber) && MyTCB.flags.bFINSent)
				{
					// Reset our timer for forced closure if the remote node 
					// doesn't send us a FIN in a timely manner.
					MyTCBStub.eventTime = SYS_TICK_Get() + (TCP_FIN_WAIT_2_TIMEOUT * SYS_TICK_TicksPerSecondGet())/1000;
					MyTCBStub.Flags.bTimerEnabled = 1;
					MyTCBStub.smState = TCP_FIN_WAIT_2;
				}
			}
			else if(MyTCBStub.smState == TCP_FIN_WAIT_2)
			{
				// RFC noncompliance:
				// The remote node should not keep sending us data 
				// indefinitely after we send a FIN to it.  
				// However, some bad stacks may still keep sending 
				// us data indefinitely after ACKing our FIN.  To 
				// prevent this from locking up our socket, let's 
				// send a RST right now and close forcefully on 
				// our side.
				if(!(localHeaderFlags & FIN))
				{
					MyTCB.MySEQ = localAckNumber;	// Set SEQ = SEG.ACK
					SendTCP(RST | ACK, 0);
					CloseSocket();
					return;
				}
			}
			else if(MyTCBStub.smState == TCP_CLOSING)
			{
				// Check to see if our FIN has been ACKnowledged
				if(MyTCB.MySEQ == localAckNumber)
				{
					// RFC not recommended: We should be going to 
					// the TCP_TIME_WAIT state right here and 
					// starting a 2MSL timer, but since we have so 
					// few precious sockets, we can't afford to 
					// leave a socket waiting around doing nothing 
					// for a long time.  If the remote node does 
					// not recieve this ACK, it'll have to figure 
					// out on it's own that the connection is now 
					// closed.
					CloseSocket();
				}

				return;
			}

			break;

		case TCP_LAST_ACK:
			// Check to see if our FIN has been ACKnowledged
			if(MyTCB.MySEQ == localAckNumber)
				CloseSocket();
			return;

//		case TCP_TIME_WAIT:
//			// Nothing is supposed to arrive here.  If it does, reset the quiet timer.
//			SendTCP(ACK, SENDTCP_RESET_TIMERS);
//			return;

		default:
			break;
	}

	//
	// Sixth: Check the URG bit
	//
	// Urgent packets are not supported in this stack, so we
	// will throw them away instead
	if(localHeaderFlags & URG)
		return;

	//
	// Seventh: Process the segment text
	//
	// Throw data away if in a state that doesn't accept data
	if(MyTCBStub.smState == TCP_CLOSE_WAIT)
		return;
	if(MyTCBStub.smState == TCP_CLOSING)
		return;
	if(MyTCBStub.smState == TCP_LAST_ACK)
		return;
//	if(MyTCBStub.smState == TCP_TIME_WAIT)
//		return;

	// Copy any valid segment data into our RX FIFO, if any
	if(len)
	{
		// See if there are bytes we must skip
		if((int16_t)wMissingBytes <= 0)
		{
			// Position packet read pointer to start of useful data area.
#if defined (TCPIP_STACK_USE_IPV6)
            if (MyTCBStub.pTxPkt->flags.addressType == IP_ADDRESS_TYPE_IPV6)
    			TCPIP_IPV6_SetRxBuffer(MyTCBStub.pSktNet, (h->DataOffset.Val << 2) - wMissingBytes);
            else
#endif
    			TCPIP_IPV4_SetRxBuffer(MyTCBStub.pSktNet, (h->DataOffset.Val << 2) - wMissingBytes);
			len += wMissingBytes;		
	
			// Truncate packets that would overflow our TCP RX FIFO
			// and request a retransmit by sending a duplicate ACK
			if(len > wFreeSpace)
				len = wFreeSpace;
	
			MyTCB.RemoteSEQ += (uint32_t)len;
		
			// Copy the application data from the packet into the socket RX FIFO
			// See if we need a two part copy (spans bufferEnd->bufferRxStart)
			if(MyTCBStub.rxHead + len > MyTCBStub.bufferEnd)
			{
				wTemp = MyTCBStub.bufferEnd - MyTCBStub.rxHead + 1;
				TCPRAMCopy(MyTCBStub.pSktNet, MyTCBStub.rxHead, MyTCBStub.vMemoryMedium, (PTR_BASE)-1, TCP_ETH_RAM, wTemp);
				TCPRAMCopy(MyTCBStub.pSktNet, MyTCBStub.bufferRxStart, MyTCBStub.vMemoryMedium, (PTR_BASE)-1, TCP_ETH_RAM, len - wTemp);
				MyTCBStub.rxHead = MyTCBStub.bufferRxStart + (len - wTemp);
			}
			else
			{
				TCPRAMCopy(MyTCBStub.pSktNet, MyTCBStub.rxHead, MyTCBStub.vMemoryMedium, (PTR_BASE)-1, TCP_ETH_RAM, len);
				MyTCBStub.rxHead += len;
			}
		
			// See if we have a hole and other data waiting already in the RX FIFO
			if(MyTCB.sHoleSize != -1)
			{
				MyTCB.sHoleSize -= len;
				wTemp = MyTCB.wFutureDataSize + MyTCB.sHoleSize;
		
				// See if we just closed up a hole, and if so, advance head pointer
				if((int16_t)wTemp < (int16_t)0)
				{
					MyTCB.sHoleSize = -1;
				}
				else if(MyTCB.sHoleSize <= 0)
				{
					MyTCB.RemoteSEQ += wTemp;
					MyTCBStub.rxHead += wTemp;
					if(MyTCBStub.rxHead > MyTCBStub.bufferEnd)
						MyTCBStub.rxHead -= MyTCBStub.bufferEnd - MyTCBStub.bufferRxStart + 1;							
					MyTCB.sHoleSize = -1;
				}
			}
		} // This packet is out of order or we lost a packet, see if we can generate a hole to accomodate it
		else if((int16_t)wMissingBytes > 0)
		{
			// Truncate packets that would overflow our TCP RX FIFO
			if(len + wMissingBytes > wFreeSpace)
				len = wFreeSpace - wMissingBytes;

			// Position packet read pointer to start of useful data area.
#if defined (TCPIP_STACK_USE_IPV6)
            if (MyTCBStub.pTxPkt->flags.addressType == IP_ADDRESS_TYPE_IPV6)
    			TCPIP_IPV6_SetRxBuffer(MyTCBStub.pSktNet, h->DataOffset.Val << 2);
            else
#endif
    			TCPIP_IPV4_SetRxBuffer(MyTCBStub.pSktNet, h->DataOffset.Val << 2);
	
			// See if we need a two part copy (spans bufferEnd->bufferRxStart)
			if(MyTCBStub.rxHead + wMissingBytes + len > MyTCBStub.bufferEnd)
			{
				// Calculate number of data bytes to copy before wraparound
				wTemp = MyTCBStub.bufferEnd - MyTCBStub.rxHead + 1 - wMissingBytes;
				if((int16_t)wTemp >= 0)
				{
					TCPRAMCopy(MyTCBStub.pSktNet, MyTCBStub.rxHead + wMissingBytes, MyTCBStub.vMemoryMedium, (PTR_BASE)-1, TCP_ETH_RAM, wTemp);
					TCPRAMCopy(MyTCBStub.pSktNet, MyTCBStub.bufferRxStart, MyTCBStub.vMemoryMedium, (PTR_BASE)-1, TCP_ETH_RAM, len - wTemp);
				}
				else
				{
					TCPRAMCopy(MyTCBStub.pSktNet, MyTCBStub.rxHead + wMissingBytes - (MyTCBStub.bufferEnd - MyTCBStub.bufferRxStart + 1), MyTCBStub.vMemoryMedium, (PTR_BASE)-1, TCP_ETH_RAM, len);
				}
			}
			else
			{
				TCPRAMCopy(MyTCBStub.pSktNet, MyTCBStub.rxHead + wMissingBytes, MyTCBStub.vMemoryMedium, (PTR_BASE)-1, TCP_ETH_RAM, len);
			}

			// Record the hole is here
			if(MyTCB.sHoleSize == -1)
			{
				MyTCB.sHoleSize = wMissingBytes;
				MyTCB.wFutureDataSize = len;
			}
			else
			{
				// We already have a hole, see if we can shrink the hole 
				// or extend the future data size
				if(wMissingBytes < (uint16_t)MyTCB.sHoleSize)
				{
					if((wMissingBytes + len > (uint16_t)MyTCB.sHoleSize + MyTCB.wFutureDataSize) || (wMissingBytes + len < (uint16_t)MyTCB.sHoleSize))
						MyTCB.wFutureDataSize = len;
					else
						MyTCB.wFutureDataSize = (uint16_t)MyTCB.sHoleSize + MyTCB.wFutureDataSize - wMissingBytes;
					MyTCB.sHoleSize = wMissingBytes;
				}
				else if(wMissingBytes + len > (uint16_t)MyTCB.sHoleSize + MyTCB.wFutureDataSize)
				{
					// Make sure that there isn't a second hole between 
					// our future data and this TCP segment's future data
					if(wMissingBytes <= (uint16_t)MyTCB.sHoleSize + MyTCB.wFutureDataSize)
						MyTCB.wFutureDataSize += wMissingBytes + len - (uint16_t)MyTCB.sHoleSize - MyTCB.wFutureDataSize;
				}
				
			}
		}
	}

	// Send back an ACK of the data (+SYN | FIN) we just received, 
	// if any.  To minimize bandwidth waste, we are implementing 
	// the delayed acknowledgement algorithm here, only sending 
	// back an immediate ACK if this is the second segment received.  
	// Otherwise, a 200ms timer will cause the ACK to be transmitted.
	if(wSegmentLength)
	{
		// For non-established sockets, let's delete all data in 
		// the RX buffer immediately after receiving it.  This is 
		// not really how TCP was intended to operate since a 
		// socket cannot receive any response after it sends a FIN,
		// but our TCP application API doesn't readily accomodate
		// receiving data after calling TCPDisconnect(), which 
		// invalidates the application TCP handle.  By deleting all 
		// data, we'll ensure that the RX window is nonzero and 
		// the remote node will be able to send us a FIN response, 
		// which needs an RX window of at least 1.
		if(MyTCBStub.smState != TCP_ESTABLISHED)
			MyTCBStub.rxTail = MyTCBStub.rxHead;

		if(MyTCBStub.Flags.bOneSegmentReceived)
		{
			SendTCP(ACK, SENDTCP_RESET_TIMERS);
			SyncTCB();
			// bOneSegmentReceived is cleared in SendTCP(), so no need here
		}
		else
		{
			MyTCBStub.Flags.bOneSegmentReceived = true;	
		
			// Do not send an ACK immediately back.  Instead, we will 
			// perform delayed acknowledgements.  To do this, we will 
			// just start a timer
			if(!MyTCBStub.Flags.bDelayedACKTimerEnabled)
			{
				MyTCBStub.Flags.bDelayedACKTimerEnabled = 1;
				MyTCBStub.OverlappedTimers.delayedACKTime = SYS_TICK_Get() + (TCP_DELAYED_ACK_TIMEOUT * SYS_TICK_TicksPerSecondGet())/1000;
			}
		}
	}

	//
	// Eighth: check the FIN bit
	//
	if(localHeaderFlags & FIN)
	{
		// Note: Since we don't have a good means of storing "FIN bytes" 
		// in our TCP RX FIFO, we must ensure that FINs are processed 
		// in-order.
		if(MyTCB.RemoteSEQ + 1 == localSeqNumber + (uint32_t)wSegmentLength)
		{
			// FINs are treated as one byte of data for ACK sequencing
			MyTCB.RemoteSEQ++;
			
			switch(MyTCBStub.smState)
			{
				case TCP_SYN_RECEIVED:
					// RFC in exact: Our API has no need for the user 
					// to explicitly close a socket that never really 
					// got opened fully in the first place, so just 
					// transmit a FIN automatically and jump to 
					// TCP_LAST_ACK
					MyTCBStub.smState = TCP_LAST_ACK;
					SendTCP(FIN | ACK, SENDTCP_RESET_TIMERS);
					return;

				case TCP_ESTABLISHED:
					// Go to TCP_CLOSE_WAIT state
					MyTCBStub.smState = TCP_CLOSE_WAIT;
					
					// For legacy applications that don't call 
					// TCPDisconnect() as needed and expect the TCP/IP 
					// Stack to automatically close sockets when the 
					// remote node sends a FIN, let's start a timer so 
					// that we will eventually close the socket automatically
					MyTCBStub.OverlappedTimers.closeWaitTime = SYS_TICK_Get() + (TCP_CLOSE_WAIT_TIMEOUT * SYS_TICK_TicksPerSecondGet())/1000;
					break;
	
				case TCP_FIN_WAIT_1:
					if(MyTCB.MySEQ == localAckNumber)
					{
						// RFC not recommended: We should be going to 
						// the TCP_TIME_WAIT state right here and 
						// starting a 2MSL timer, but since we have so 
						// few precious sockets, we can't afford to 
						// leave a socket waiting around doing nothing 
						// for a long time.  If the remote node does 
						// not recieve this ACK, it'll have to figure 
						// out on it's own that the connection is now 
						// closed.
						SendTCP(ACK, 0);
						CloseSocket();
						return;
					}
					else
					{
						MyTCBStub.smState = TCP_CLOSING;
					}
					break;
	
				case TCP_FIN_WAIT_2:
					// RFC not recommended: We should be going to 
					// the TCP_TIME_WAIT state right here and 
					// starting a 2MSL timer, but since we have so 
					// few precious sockets, we can't afford to 
					// leave a socket waiting around doing nothing 
					// for a long time.  If the remote node does 
					// not recieve this ACK, it'll have to figure 
					// out on it's own that the connection is now 
					// closed.
					SendTCP(ACK, 0);
					CloseSocket();
					return;

				default:
					break;
			}

			// Acknowledge receipt of FIN
			SendTCP(ACK, SENDTCP_RESET_TIMERS);
		}
	}
}

/****************************************************************************
  Section:
	Buffer Management Functions
  ***************************************************************************/

/*****************************************************************************
  Function:
	bool TCPAdjustFIFOSize(TCP_SOCKET hTCP, uint16_t wMinRXSize, 
							uint16_t wMinTXSize, TCP_ADJUST_FLAGS vFlags)

  Summary:
	Adjusts the relative sizes of the RX and TX buffers.

  Description:
	This function can be used to adjust the relative sizes of the RX and
	TX FIFO depending on the immediate needs of an application.  Since a 
	larger FIFO can allow more data to be sent in a given packet, adjusting 
	the relative sizes on the fly can allow for optimal transmission speed 
	for one-sided application protocols.  For example, HTTP typically 
	begins by receiving large amounts of data from the client, then switches
	to serving large amounts of data back.  Adjusting the FIFO at these 
	points can increase performance substantially.  Once the FIFO is
	adjusted, a window update is sent.
	
	If neither or both of TCP_ADJUST_GIVE_REST_TO_TX and 
	TCP_ADJUST_GIVE_REST_TO_RX are set, the function distributes the
	remaining space equally.
	
	Received data can be preserved as long as the buffer is expanding and 
	has not wrapped.

  Precondition:
	TCP is initialized.

  Parameters:
	hTCP		- The socket to be adjusted
	wMinRXSize	- Minimum number of byte for the RX FIFO
	wMinTXSize 	- Minimum number of bytes for the RX FIFO
	vFlags		- Any combination of TCP_ADJUST_GIVE_REST_TO_RX, 
				  TCP_ADJUST_GIVE_REST_TO_TX, TCP_ADJUST_PRESERVE_RX.
				  TCP_ADJUST_PRESERVE_TX is not currently supported.

  Return Values:
	true - The FIFOs were adjusted successfully
	false - Minimum RX, Minimum TX, or flags couldn't be accommodated and
			therefore the socket was left unchanged.

  Side Effects:
	Any unacknowledged or untransmitted data in the TX FIFO is always
	deleted.

  Remarks:
	At least one byte must always be allocated to the RX buffer so that
	a FIN can be received.  The function automatically corrects for this.
  ***************************************************************************/
bool TCPAdjustFIFOSize(TCP_SOCKET hTCP, uint16_t wMinRXSize, uint16_t wMinTXSize, TCP_ADJUST_FLAGS vFlags)
{
	PTR_BASE ptrTemp, ptrHead;
	uint16_t wTXAllocation;
	
	if(!_TcpSocketChk(hTCP))
    {
        return false;
    }

	// Load up info on this socket
	SyncTCBStub(hTCP);

	// RX has to be at least 1 byte to receive SYN and FIN bytes 
	// from the remote node, even if they aren't stored in the RX FIFO
	if(wMinRXSize == 0u)
		wMinRXSize = 1;
		
	// SSL connections need to be able to send or receive at least 
	// a full Alert record, MAC, and FIN
	#if defined(TCPIP_STACK_USE_SSL)
	if(TCPIsSSL(hTCP) && wMinRXSize < 25u)
		wMinRXSize = 25;
	if(TCPIsSSL(hTCP) && wMinTXSize < 25u)
		wMinTXSize = 25;
	#endif
	
	// Make sure space is available for minimums
	ptrTemp = MyTCBStub.bufferEnd - MyTCBStub.bufferTxStart - 1;
	if(wMinRXSize + wMinTXSize > ptrTemp)
		return false;

	SyncTCB();

	// Set both allocation flags if none set
	if(!(vFlags & (TCP_ADJUST_GIVE_REST_TO_TX | TCP_ADJUST_GIVE_REST_TO_RX)))
		vFlags |= TCP_ADJUST_GIVE_REST_TO_TX | TCP_ADJUST_GIVE_REST_TO_RX;
		

	// Allocate minimums
	wTXAllocation = wMinTXSize;
	ptrTemp -= wMinRXSize + wMinTXSize;

	// Allocate extra
	if(vFlags & TCP_ADJUST_GIVE_REST_TO_TX)
	{
		if(vFlags & TCP_ADJUST_GIVE_REST_TO_RX)
		{
			// Do a 50%/50% split with any odd byte always going to the RX FIFO
			wTXAllocation += ptrTemp>>1;
		}
		else
		{
			wTXAllocation += ptrTemp;
		}
	}

	// Calculate new bufferRxStart pointer
	ptrTemp = MyTCBStub.bufferTxStart + wTXAllocation + 1;

	// Find the head pointer to use
	ptrHead = MyTCBStub.rxHead;
	#if defined(TCPIP_STACK_USE_SSL)
	if(TCPIsSSL(hTCP))
		ptrHead = MyTCBStub.sslRxHead;
	#endif
	
	// If there's out-of-order data pending, adjust the head pointer to compensate
	if(MyTCB.sHoleSize != -1)
	{
		ptrHead += MyTCB.sHoleSize + MyTCB.wFutureDataSize;
		if(ptrHead > MyTCBStub.bufferEnd)
			ptrHead -= MyTCBStub.bufferEnd - MyTCBStub.bufferRxStart + 1;
	}

	// Determine if resizing will lose any RX data
	if(MyTCBStub.rxTail < ptrHead)
	{
		if(ptrTemp > MyTCBStub.rxTail)
		{
			if(vFlags & TCP_ADJUST_PRESERVE_RX)
				return false;
			else
			{
				MyTCBStub.rxTail = ptrTemp;
				MyTCBStub.rxHead = ptrTemp;

				#if defined(TCPIP_STACK_USE_SSL)
				MyTCBStub.sslRxHead = ptrTemp;
				#endif
			}
		}
	}
	else if(MyTCBStub.rxTail > ptrHead)
	{
		if(ptrTemp > MyTCBStub.bufferRxStart)
		{
			if(vFlags & TCP_ADJUST_PRESERVE_RX)
				return false;
			else
			{
				MyTCBStub.rxTail = ptrTemp;
				MyTCBStub.rxHead = ptrTemp;
				
				#if defined(TCPIP_STACK_USE_SSL)
				MyTCBStub.sslRxHead = ptrTemp;
				#endif
			}
		}
	}
	else
	{
		// No data to preserve, but we may need to move 
		// the pointers to stay in the RX space
		MyTCBStub.rxTail = ptrTemp;
		MyTCBStub.rxHead = ptrTemp;
		
		#if defined(TCPIP_STACK_USE_SSL)
		MyTCBStub.sslRxHead = ptrTemp;
		#endif
	}
	
	// If we need to preserve data that wrapped in the ring, we must copy
	if(ptrHead < MyTCBStub.rxTail && (vFlags & TCP_ADJUST_PRESERVE_RX))
	{
		TCPRAMCopy(MyTCBStub.pSktNet, ptrTemp, MyTCBStub.vMemoryMedium, 
			MyTCBStub.bufferRxStart, MyTCBStub.vMemoryMedium,
			ptrHead - MyTCBStub.bufferRxStart);

		// Move the pointers if they were in front of the tail
		#if defined(TCPIP_STACK_USE_SSL)
		if(TCPIsSSL(hTCP) && MyTCBStub.sslRxHead < MyTCBStub.rxTail)
			MyTCBStub.sslRxHead -= MyTCBStub.bufferRxStart - ptrTemp;
		#endif
		if(MyTCBStub.rxHead < MyTCBStub.rxTail)
			MyTCBStub.rxHead -= MyTCBStub.bufferRxStart - ptrTemp;
	}
	
	// Move the RX buffer pointer - it's the one that divides the two
	MyTCBStub.bufferRxStart = ptrTemp;

	// Empty the TX buffer
	MyTCB.txUnackedTail = MyTCBStub.bufferTxStart;
	MyTCBStub.txTail = MyTCBStub.bufferTxStart;
	MyTCBStub.txHead = MyTCBStub.bufferTxStart;
	
	#if defined(TCPIP_STACK_USE_SSL)
	if(TCPIsSSL(hTCP))
		MyTCBStub.sslTxHead = MyTCBStub.txHead + 5;
	#endif
	
	// Send a window update to notify remote node of change
	if(MyTCBStub.smState == TCP_ESTABLISHED)
		SendTCP(ACK, SENDTCP_RESET_TIMERS);

	return true;

}

/*****************************************************************************
  Function:
	static void TCPRAMCopy(NET_CONFIG* pNet, PTR_BASE ptrDest, uint8_t vDestType, PTR_BASE ptrSource, 
							uint8_t vSourceType, uint16_t wLength)

  Summary:
	Copies data to/from various memory mediums.

  Description:
	This function copies data between memory mediums (PIC RAM, SPI
	RAM, and Ethernet buffer RAM).

  Precondition:
	TCP is initialized.

  Parameters:
    pNet        - interface to use 
    ptrDest		- Address to write to
	vDestType	- Destination meidum (TCP_PIC_RAM, TCP_ETH_RAM, TCP_SPI_RAM)
	ptrSource	- Address to copy from
	vSourceType - Source medium (TCP_PIC_RAM, TCP_ETH_RAM, or TCP_SPI_RAM)
	wLength		- Number of bytes to copy

  Returns:
	None

  Remarks:
	Copying to a destination region that overlaps with the source address 
	is supported only if the destination start address is at a lower memory 
	address (closer to 0x0000) than the source pointer.  However, if they do 
	overlap there must be at least 4 bytes of non-overlap to ensure correct 
	results due to hardware DMA requirements.
  ***************************************************************************/
// NOTE: TCPRAMCopy() is called even from TCPInit() at a moment when the socket is NOT BOUND
// (i.e. interface is un-known). 
static void TCPRAMCopy(NET_CONFIG* pNet, PTR_BASE ptrDest, uint8_t vDestType, PTR_BASE ptrSource, uint8_t vSourceType, uint16_t wLength)
{
	#if defined(SPIRAM_CS_TRIS)
	uint8_t vBuffer[16];
	uint16_t w;
	#endif
    TCPIP_MAC_HANDLE    hMac;

    hMac = _TCPIPStackNetToMac(pNet);
    
	switch(vSourceType)
	{
		case TCP_PIC_RAM:
		case TCP_PIC_DRAM:
			switch(vDestType)
			{
				case TCP_PIC_RAM:
				case TCP_PIC_DRAM:
					memcpy((void*)(uint8_t*)ptrDest, (void*)(uint8_t*)ptrSource, wLength);
					break;
	
				case TCP_ETH_RAM:
                    if(hMac ==0 )
                    {
                        SYS_ERROR(SYS_ERROR_ERROR,  "TCPRAMCopy: ETH RAM copy required with no valid MAC!");
                    }

					if(ptrDest!=(PTR_BASE)-1)
						MACSetWritePtr(hMac, ptrDest);
					MACPutArray(hMac, (uint8_t*)ptrSource, wLength);
					break;
	
				#if defined(SPIRAM_CS_TRIS)
				case TCP_SPI_RAM:
					SPIRAMPutArray(ptrDest, (uint8_t*)ptrSource, wLength);
					break;
				#endif
			}
			break;
	
		case TCP_ETH_RAM:
            if(hMac ==0 )
            {
                SYS_ERROR(SYS_ERROR_ERROR, "TCPRAMCopy: ETH RAM copy required with no valid MAC!");
            }
            
			switch(vDestType)
			{
				case TCP_PIC_RAM:
				case TCP_PIC_DRAM:
					if(ptrSource!=(PTR_BASE)-1)
						MACSetReadPtr(hMac, ptrSource);
					MACGetArray(hMac, (uint8_t*)ptrDest, wLength);
					break;
	
				case TCP_ETH_RAM:
					MACMemCopyAsync(hMac, ptrDest, ptrSource, wLength);
					while(!MACIsMemCopyDone(hMac));
					break;
	
				#if defined(SPIRAM_CS_TRIS)
				case TCP_SPI_RAM:
					if(ptrSource!=(PTR_BASE)-1)
						MACSetReadPtr(hMac, ptrSource);
					w = sizeof(vBuffer);
					while(wLength)
					{
						if(w > wLength)
							w = wLength;
						
						// Read and write a chunk	
						MACGetArray(hMac, vBuffer, w);
						SPIRAMPutArray(ptrDest, vBuffer, w);
						ptrDest += w;
						wLength -= w;
					}
					break;
				#endif
			}
			break;
	
		#if defined(SPIRAM_CS_TRIS)
		case TCP_SPI_RAM:
			switch(vDestType)
			{
				case TCP_PIC_RAM:
				case TCP_PIC_DRAM:
					SPIRAMGetArray(ptrSource, (uint8_t*)ptrDest, wLength);
					break;
	
				case TCP_ETH_RAM:
                    if(hMac ==0 )
                    {
                        SYS_ERROR(SYS_ERROR_ERROR, "TCPRAMCopy: ETH RAM copy required with no valid MAC!");
                    }

					if(ptrDest!=(PTR_BASE)-1)
						MACSetWritePtr(hMac, ptrDest);
					w = sizeof(vBuffer);
					while(wLength)
					{
						if(w > wLength)
							w = wLength;
						
						// Read and write a chunk	
						SPIRAMGetArray(ptrSource, vBuffer, w);
						ptrSource += w;
						MACPutArray(hMac, vBuffer, w);
						wLength -= w;
					}
					break;
	
				case TCP_SPI_RAM:
					// Copy all of the data over in chunks
					w = sizeof(vBuffer);
					while(wLength)
					{
						if(w > wLength)
							w = wLength;
							
						SPIRAMGetArray(ptrSource, vBuffer, w);
						SPIRAMPutArray(ptrDest, vBuffer, w);
						ptrSource += w;
						ptrDest += w;
						wLength -= w;
					}
					break;
			}
			break;
		#endif			
	}
}


/****************************************************************************
  Section:
	SSL Functions
  ***************************************************************************/

/*****************************************************************************
  Function:
	bool TCPStartSSLClient(TCP_SOCKET hTCP, uint8_t* host)

  Summary:
	Begins an SSL client session.

  Description:
	This function escalates the current connection to an SSL secured 
	connection by initiating an SSL client handshake.

  Precondition:
	TCP is initialized and hTCP is already connected.

  Parameters:
	hTCP		- TCP connection to secure
	host		- Expected host name on certificate (currently ignored)

  Return Values:
	true 		- an SSL connection was initiated
	false 		- Insufficient SSL resources (stubs) were available

  Remarks:
	The host parameter is currently ignored and is not validated.
  ***************************************************************************/
#if defined(TCPIP_STACK_USE_SSL_CLIENT)
bool TCPStartSSLClient(TCP_SOCKET hTCP, uint8_t* host)
{
	uint8_t i;
	
	if(!_TcpSocketChk(hTCP))
    {
        return false;
    }

	SyncTCBStub(hTCP);
	
	// Make sure SSL is not established already
	if(MyTCBStub.sslStubID != SSL_INVALID_ID)
		return false;
	
	// Try to start the session
    if(MyTCBStub.pSktNet == 0)
    {   // unbound socket; bind it to the default interface
        _TcpSocketBind(&MyTCBStub, (NET_CONFIG*)TCPIP_STACK_GetDefaultNet());
    }
    MyTCBStub.sslStubID = SSLStartSession(hTCP, NULL, 0);
	
	// Make sure a session stub was obtained
	if(MyTCBStub.sslStubID == SSL_INVALID_ID)
		return false;

	// Mark connection as handshaking and return
	MyTCBStub.sslReqMessage = SSL_CLIENT_HELLO;
	MyTCBStub.sslRxHead = MyTCBStub.rxHead;
	MyTCBStub.sslTxHead = MyTCBStub.txHead;
	MyTCBStub.Flags.bSSLHandshaking = 1;
	for(i = 0; i < 5u; i++)
	{// Skip first 5 bytes in TX for the record header
		if(++MyTCBStub.sslTxHead >= MyTCBStub.bufferRxStart)
			MyTCBStub.sslTxHead = MyTCBStub.bufferTxStart;
	}
	return true;
}
#endif // SSL Client

/*****************************************************************************
  Function:
	bool TCPStartSSLClientEx(TCP_SOCKET hTCP, uint8_t* host, uint8_t * buffer, uint8_t suppDataType)

  Summary:
	Begins an SSL client session.

  Description:
	This function escalates the current connection to an SSL secured 
	connection by initiating an SSL client handshake.

  Precondition:
	TCP is initialized and hTCP is already connected.

  Parameters:
	hTCP			- TCP connection to secure
	host			- Expected host name on certificate (currently ignored)
	buffer      	- Buffer for supplementary data return
	suppDataType 	- Type of supplementary data to copy

  Return Values:
	true 		- an SSL connection was initiated
	false 		- Insufficient SSL resources (stubs) were available

  Remarks:
	The host parameter is currently ignored and is not validated.
  ***************************************************************************/
#if defined(TCPIP_STACK_USE_SSL_CLIENT)
bool TCPStartSSLClientEx(TCP_SOCKET hTCP, uint8_t* host, void * buffer, uint8_t suppDataType)
{
	uint8_t i;
	
	if(!_TcpSocketChk(hTCP))
    {
        return false;
    }

	SyncTCBStub(hTCP);
	
	// Make sure SSL is not established already
	if(MyTCBStub.sslStubID != SSL_INVALID_ID)
		return false;
	
	// Try to start the session
    if(MyTCBStub.pSktNet == 0)
    {   // unbound socket; bind it to the default interface
        _TcpSocketBind(&MyTCBStub, (NET_CONFIG*)TCPIP_STACK_GetDefaultNet());
    }
	MyTCBStub.sslStubID = SSLStartSession(hTCP, buffer, suppDataType);
	
	// Make sure a session stub was obtained
	if(MyTCBStub.sslStubID == SSL_INVALID_ID)
		return false;

	// Mark connection as handshaking and return
	MyTCBStub.sslReqMessage = SSL_CLIENT_HELLO;
	MyTCBStub.sslRxHead = MyTCBStub.rxHead;
	MyTCBStub.sslTxHead = MyTCBStub.txHead;
	MyTCBStub.Flags.bSSLHandshaking = 1;
	for(i = 0; i < 5u; i++)
	{// Skip first 5 bytes in TX for the record header
		if(++MyTCBStub.sslTxHead >= MyTCBStub.bufferRxStart)
			MyTCBStub.sslTxHead = MyTCBStub.bufferTxStart;
	}
	return true;
}
#endif // SSL Client

/*****************************************************************************
  Function:
	bool TCPStartSSLServer(TCP_SOCKET hTCP)

  Summary:
	Begins an SSL server session.

  Description:
	This function sets up an SSL server session when a new connection is
	established on an SSL port.

  Precondition:
	TCP is initialized and hTCP is already connected.

  Parameters:
	hTCP		- TCP connection to secure

  Return Values:
	true		- an SSL connection was initiated
	false		- Insufficient SSL resources (stubs) were available
  ***************************************************************************/
#if defined(TCPIP_STACK_USE_SSL_SERVER)
bool TCPStartSSLServer(TCP_SOCKET hTCP)
{
	uint8_t i;
	
	if(!_TcpSocketChk(hTCP))
    {
        return false;
    }

	SyncTCBStub(hTCP);
	SyncTCB();
	
	// Make sure SSL is not established already
	if(MyTCBStub.sslStubID != SSL_INVALID_ID)
		return true;
	
	// Try to start the session
	MyTCBStub.sslStubID = SSLStartSession(hTCP, NULL, 0);
	
	// Make sure a session stub was obtained
	if(MyTCBStub.sslStubID == SSL_INVALID_ID)
		return false;

	// Swap the localPort and localSSLPort
	MyTCBStub.remoteHash.Val = MyTCB.localPort.Val;
	MyTCB.localPort.Val = MyTCB.localSSLPort.Val;
	MyTCB.localSSLPort.Val = MyTCBStub.remoteHash.Val;	

	// Mark connection as handshaking and return
	MyTCBStub.sslReqMessage = SSL_NO_MESSAGE;
	MyTCBStub.sslRxHead = MyTCBStub.rxHead;
	MyTCBStub.sslTxHead = MyTCBStub.txHead;
	MyTCBStub.Flags.bSSLHandshaking = 1;
	for(i = 0; i < 5u; i++)
	{// Skip first 5 bytes in TX for the record header
		if(++MyTCBStub.sslTxHead >= MyTCBStub.bufferRxStart)
			MyTCBStub.sslTxHead = MyTCBStub.bufferTxStart;
	}
	return true;
}
#endif // SSL Client

/*****************************************************************************
  Function:
	bool TCPAddSSLListener(TCP_SOCKET hTCP, uint16_t port)

  Summary:
	Listens for SSL connection on a specific port.

  Description:
	This function adds an additional listening port to a TCP connection.  
	Connections made on this alternate port will be secured via SSL.

  Precondition:
	TCP is initialized and hTCP is listening.

  Parameters:
	hTCP		- TCP connection to secure
	port		- SSL port to listen on

  Return Values:
	true		- SSL port was added.
	false		- The socket was not a listening socket.
  ***************************************************************************/
#if defined(TCPIP_STACK_USE_SSL_SERVER)
bool TCPAddSSLListener(TCP_SOCKET hTCP, uint16_t port)
{
	if(!_TcpSocketChk(hTCP))
    {
        return false;
    }

	SyncTCBStub(hTCP);
	
	if(MyTCBStub.smState != TCP_LISTEN)
		return false;
	
	SyncTCB();
	
	MyTCB.localSSLPort.Val = port;
	MyTCBStub.sslTxHead = port;

	return true;
}
#endif // SSL Server

/*****************************************************************************
  Function:
	bool TCPRequestSSLMessage(TCP_SOCKET hTCP, uint8_t msg)

  Summary:
	Requests an SSL message to be transmitted.

  Description:
	This function is called to request that a specific SSL message be
	transmitted.  This message should only be called by the SSL module.
	
  Precondition:
	TCP is initialized.

  Parameters:
	hTCP		- TCP connection to use
	msg			- One of the SSL_MESSAGE types to transmit.

  Return Values:
	true		- The message was requested.
	false		- Another message is already pending transmission.
  ***************************************************************************/
#if defined(TCPIP_STACK_USE_SSL)
bool TCPRequestSSLMessage(TCP_SOCKET hTCP, uint8_t msg)
{
	if(!_TcpSocketChk(hTCP))
    {
        return false;
    }

	SyncTCBStub(hTCP);
	
	if(msg == SSL_NO_MESSAGE || MyTCBStub.sslReqMessage == SSL_NO_MESSAGE)
	{
		MyTCBStub.sslReqMessage = msg;
		return true;
	}
	
	return false;
}
#endif // SSL

/*****************************************************************************
  Function:
	bool TCPSSLIsHandshaking(TCP_SOCKET hTCP)

  Summary:
	Determines if an SSL session is still handshaking.

  Description:
	Call this function after calling TCPStartSSLClient until false is
	returned.  Then your application may continue with its normal data
	transfer (which is now secured).
	
  Precondition:
	TCP is initialized and hTCP is connected.

  Parameters:
	hTCP		- TCP connection to check

  Return Values:
	true		- SSL handshake is still progressing
	false		- SSL handshake has completed
  ***************************************************************************/
#if defined(TCPIP_STACK_USE_SSL)
bool TCPSSLIsHandshaking(TCP_SOCKET hTCP)
{
	if(!_TcpSocketChk(hTCP))
    {
        return false;
    }

	SyncTCBStub(hTCP);
	return MyTCBStub.Flags.bSSLHandshaking;	
}
#endif // SSL

/*****************************************************************************
  Function:
	bool TCPIsSSL(TCP_SOCKET hTCP)

  Summary:
	Determines if a TCP connection is secured with SSL.

  Description:
	Call this function to determine whether or not a TCP connection is 
	secured with SSL.
	
  Precondition:
	TCP is initialized and hTCP is connected.

  Parameters:
	hTCP		- TCP connection to check

  Return Values:
	true		- Connection is secured via SSL
	false		- Connection is not secured
  ***************************************************************************/
#if defined(TCPIP_STACK_USE_SSL)
bool TCPIsSSL(TCP_SOCKET hTCP)
{
	if(!_TcpSocketChk(hTCP))
    {
        return false;
    }

	SyncTCBStub(hTCP);
	
	if(MyTCBStub.sslStubID == SSL_INVALID_ID)
		return false;
	
	return true;
}
#endif // SSL

/*****************************************************************************
  Function:
	void TCPSSLHandshakeComplete(TCP_SOCKET hTCP)

  Summary:
	Clears the SSL handshake flag.

  Description:
	This function clears the flag indicating that an SSL handshake is
	complete.
	
  Precondition:
	TCP is initialized and hTCP is connected.

  Parameters:
	hTCP		- TCP connection to set

  Returns:
	None

  Remarks:
	This function should never be called by an application.  It is used 
	only by the SSL module itself.
  ***************************************************************************/
#if defined(TCPIP_STACK_USE_SSL)
void TCPSSLHandshakeComplete(TCP_SOCKET hTCP)
{

	if(!_TcpSocketChk(hTCP))
    {
        return;
    }
	SyncTCBStub(hTCP);
	MyTCBStub.Flags.bSSLHandshaking = 0;
}
#endif // SSL

/*****************************************************************************
  Function:
	void TCPSSLDecryptMAC(TCP_SOCKET hTCP, ARCFOUR_CTX* ctx, uint16_t len)

  Summary:
	Decrypts and MACs data arriving via SSL.

  Description:
	This function decrypts data in the TCP buffer and calculates the MAC over
	the data.  All data is left in the exact same location in the TCP buffer.
	It is called to help process incoming SSL records.
	
  Precondition:
	TCP is initialized, hTCP is connected, and ctx's Sbox is loaded.

  Parameters:
	hTCP		- TCP connection to decrypt in
	ctx			- ARCFOUR encryption context to use
	len 		- Number of bytes to crypt
	inPlace		- true to write back in place, false to write at end of
					currently visible data.

  Returns:
	None

  Remarks:
	This function should never be called by an application.  It is used 
	only by the SSL module itself.
  ***************************************************************************/
#if defined(TCPIP_STACK_USE_SSL)
void TCPSSLDecryptMAC(TCP_SOCKET hTCP, ARCFOUR_CTX* ctx, uint16_t len)
{
	PTR_BASE wSrc, wDest, wBlockLen, wTemp;
	uint8_t buffer[32];
	
	if(!_TcpSocketChk(hTCP))
    {
        return;
    }

	// Set up the pointers
	SyncTCBStub(hTCP);
	wSrc = MyTCBStub.rxTail;
	wDest = wSrc;
	
	// Handle 32 bytes at a time
	while(len)
	{
		// Determine how many bytes we can read
		wBlockLen = sizeof(buffer);
		if(wBlockLen > len) // Don't do more than we should
			wBlockLen = len;
		
		// Read those bytes to a buffer
		if(wSrc + wBlockLen > MyTCBStub.bufferEnd)
		{// Two part read
			wTemp = MyTCBStub.bufferEnd - wSrc + 1;
			TCPRAMCopy(MyTCBStub.pSktNet, (PTR_BASE)buffer, TCP_PIC_RAM, wSrc, MyTCBStub.vMemoryMedium, wTemp);
			TCPRAMCopy(MyTCBStub.pSktNet, (PTR_BASE)buffer+wTemp, TCP_PIC_RAM, MyTCBStub.bufferRxStart, MyTCBStub.vMemoryMedium, wBlockLen - wTemp);
			wSrc = MyTCBStub.bufferRxStart + wBlockLen - wTemp;
		}
		else
		{
			TCPRAMCopy(MyTCBStub.pSktNet, (PTR_BASE)buffer, TCP_PIC_RAM, wSrc, MyTCBStub.vMemoryMedium, wBlockLen);
			wSrc += wBlockLen;
		}
		
		// Decrypt and hash
		ARCFOURCrypt(ctx, buffer, wBlockLen);
		SSLMACAdd(buffer, wBlockLen);
		
		// Write decrypted bytes back
		if(wDest + wBlockLen > MyTCBStub.bufferEnd)
		{// Two part write
			wTemp = MyTCBStub.bufferEnd - wDest + 1;
			TCPRAMCopy(MyTCBStub.pSktNet, wDest, MyTCBStub.vMemoryMedium, (PTR_BASE)buffer, TCP_PIC_RAM, wTemp);
			TCPRAMCopy(MyTCBStub.pSktNet, MyTCBStub.bufferRxStart, MyTCBStub.vMemoryMedium, (PTR_BASE)buffer+wTemp, TCP_PIC_RAM, wBlockLen - wTemp);
			wDest = MyTCBStub.bufferRxStart + wBlockLen - wTemp;
		}
		else
		{
			TCPRAMCopy(MyTCBStub.pSktNet, wDest, MyTCBStub.vMemoryMedium, (PTR_BASE)buffer, TCP_PIC_RAM, wBlockLen);
			wDest += wBlockLen;
		}
		
		// Update the length remaining
		len -= wBlockLen;
	}
}	
#endif // SSL

/*****************************************************************************
  Function:
	void TCPSSLInPlaceMACEncrypt(TCP_SOCKET hTCP, ARCFOUR_CTX* ctx, 
									uint8_t* MACSecret, uint16_t len)

  Summary:
	Encrypts and MACs data in place in the TCP TX buffer.

  Description:
	This function encrypts data in the TCP buffer while calcuating a MAC.  
	When encryption is finished, the MAC is appended to the buffer and 
	the record will be ready to transmit.
	
  Precondition:
	TCP is initialized, hTCP is connected, and ctx's Sbox is loaded.

  Parameters:
	hTCP		- TCP connection to encrypt in
	ctx			- ARCFOUR encryption context to use
	MACSecret	- MAC encryption secret to use
	len 		- Number of bytes to crypt

  Returns:
	None

  Remarks:
	This function should never be called by an application.  It is used 
	only by the SSL module itself.
  ***************************************************************************/
#if defined(TCPIP_STACK_USE_SSL)
void TCPSSLInPlaceMACEncrypt(TCP_SOCKET hTCP, ARCFOUR_CTX* ctx, uint8_t* MACSecret, uint16_t len)
{
	PTR_BASE pos;
	uint16_t blockLen;
	uint8_t buffer[32];
	
	if(!_TcpSocketChk(hTCP))
    {
        return;
    }

	// Set up the pointers
	SyncTCBStub(hTCP);
	pos = MyTCBStub.txHead;
	for(blockLen = 0; blockLen < 5u; blockLen++)
	{// Skips first 5 bytes for the header
		if(++pos >= MyTCBStub.bufferRxStart)
			pos = MyTCBStub.bufferTxStart;
	}
	
	// Handle 32 bytes at a time
	while(len)
	{
		// Determine how many bytes we can read
		blockLen = sizeof(buffer);
		if(blockLen > len) // Don't do more than we should
			blockLen = len;
		if(blockLen > MyTCBStub.bufferRxStart - pos) // Don't pass the end
			blockLen = MyTCBStub.bufferRxStart - pos;
		
		// Read those bytes to a buffer
		TCPRAMCopy(MyTCBStub.pSktNet, (PTR_BASE)buffer, TCP_PIC_RAM, pos, MyTCBStub.vMemoryMedium, blockLen);
		
		// Hash and encrypt
		SSLMACAdd(buffer, blockLen);
		ARCFOURCrypt(ctx, buffer, blockLen);
		
		// Put them back
		TCPRAMCopy(MyTCBStub.pSktNet, pos, MyTCBStub.vMemoryMedium, (PTR_BASE)buffer, TCP_PIC_RAM, blockLen);
		
		// Update the pointers
		pos += blockLen;
		len -= blockLen;
		if(pos >= MyTCBStub.bufferRxStart)
			pos = MyTCBStub.bufferTxStart;
	}
	
	// Calculate and add the MAC
	SSLMACCalc(MACSecret, buffer);
	ARCFOURCrypt(ctx, buffer, 16);

	// Write the MAC to the TX FIFO
	// Can't use TCPPutArray here because TCPIsPutReady() saves 16 bytes for the MAC
	// TCPPut* functions use this to prevent writing too much data.  Therefore, the
	// functionality is duplicated here.
	
	len = 16;
	blockLen = 0;
	// See if we need a two part put
	if(MyTCBStub.sslTxHead + len >= MyTCBStub.bufferRxStart)
	{
		blockLen = MyTCBStub.bufferRxStart-MyTCBStub.sslTxHead;
		TCPRAMCopy(MyTCBStub.pSktNet, MyTCBStub.sslTxHead, MyTCBStub.vMemoryMedium, (PTR_BASE)buffer, TCP_PIC_RAM, blockLen);
		len -= blockLen;
		MyTCBStub.sslTxHead = MyTCBStub.bufferTxStart;
	}
	
	TCPRAMCopy(MyTCBStub.pSktNet, MyTCBStub.sslTxHead, MyTCBStub.vMemoryMedium, (PTR_BASE)&buffer[blockLen], TCP_PIC_RAM, len);
	MyTCBStub.sslTxHead += len;

}	
#endif // SSL

/*****************************************************************************
  Function:
	void TCPSSLPutRecordHeader(TCP_SOCKET hTCP, uint8_t* hdr, bool recDone)

  Summary:
	Writes an SSL record header and sends an SSL record.

  Description:
	This function writes an SSL record header to the pending TCP SSL data, 
	then indicates that the data is ready to be sent by moving the txHead
	pointer.
	
	If the record is complete, set recDone to true.  The sslTxHead 
	pointer will be moved forward 5 bytes to leave space for a future 
	record header.  If the record is only partially sent, use false and
	to leave the pointer where it is so that more data can be added
	to the record.  Partial records can only be used for the 
	SERVER_CERTIFICATE handshake message.
	
  Precondition:
	TCP is initialized, and hTCP is connected with an active SSL session.

  Parameters:
	hTCP		- TCP connection to write the header and transmit with
	hdr			- Record header (5 bytes) to send or NULL to just 
				  move the pointerctx
	recDone		- true if the record is done, false otherwise

  Returns:
	None

  Remarks:
	This function should never be called by an application.  It is used 
	only by the SSL module itself.
  ***************************************************************************/
#if defined(TCPIP_STACK_USE_SSL)
void TCPSSLPutRecordHeader(TCP_SOCKET hTCP, uint8_t* hdr, bool recDone)
{
	uint8_t i;
	
	// Set up the pointers
	if(!_TcpSocketChk(hTCP))
    {
        return;
    }

	SyncTCBStub(hTCP);
	
	// Write the header if needed
	if(hdr)
	{// This is a new record, so insert the header
		for(i = 0; i < 5u; i++)
		{
			TCPRAMCopy(MyTCBStub.pSktNet, MyTCBStub.txHead, MyTCBStub.vMemoryMedium, (PTR_BASE)hdr+i, TCP_PIC_RAM, sizeof(uint8_t));
			if(++MyTCBStub.txHead >= MyTCBStub.bufferRxStart)
				MyTCBStub.txHead = MyTCBStub.bufferTxStart;
		}
	}
	
	// Move the txHead pointer to indicate what data is ready
	// Also, flush just the header, then all the data.  This shotguns two 
	// packets down the line, therefore causing immediate ACKs by the 
	// remote node.  Reconnect handshakes are as much as 60% faster now.
	TCPFlush(hTCP);
	MyTCBStub.txHead = MyTCBStub.sslTxHead;
	TCPFlush(hTCP);
	
	// If this record is done, move the sslTxHead forward
	// to accomodate the next record header
	if(recDone)
	{
		for(i = 0; i < 5u; i++)
		{// Skip first 5 bytes in TX for the record header
			if(++MyTCBStub.sslTxHead >= MyTCBStub.bufferRxStart)
				MyTCBStub.sslTxHead = MyTCBStub.bufferTxStart;
		}
	}
}	
#endif // SSL

/*****************************************************************************
  Function:
	uint16_t TCPSSLGetPendingTxSize(TCP_SOCKET hTCP)

  Summary:
	Determines how many bytes are pending for a future SSL record.

  Description:
	This function determines how many bytes are pending for a future SSL
	record.
	
  Precondition:
	TCP is initialized, and hTCP is connected with an active SSL connection.

  Parameters:
	hTCP		- TCP connection to check

  Returns:
	None
  ***************************************************************************/
#if defined(TCPIP_STACK_USE_SSL)
uint16_t TCPSSLGetPendingTxSize(TCP_SOCKET hTCP)
{
	if(!_TcpSocketChk(hTCP))
    {
        return 0;
    }

	SyncTCBStub(hTCP);

	// Non-SSL connections have no pending SSL data
	//if(MyTCBStub.sslStubID == SSL_INVALID_ID)
	//	return 0;
			
	// Determine how many bytes are waiting to be written in this record
	if(MyTCBStub.sslTxHead > MyTCBStub.txHead)
		return MyTCBStub.sslTxHead - MyTCBStub.txHead - 5;
	else
		return (MyTCBStub.bufferRxStart - MyTCBStub.bufferTxStart - 1) - (MyTCBStub.txHead - MyTCBStub.sslTxHead - 1) - 5;
}
#endif


/*****************************************************************************
  Function:
	void TCPSSLHandleIncoming(TCP_SOCKET hTCP)

  Summary:
	Hands newly arrive TCP data to the SSL module for processing.

  Description:
	This function processes incoming TCP data as an SSL record and 
	performs any necessary repositioning and decrypting.
	
  Precondition:
	TCP is initialized, and hTCP is connected with an active SSL session.

  Parameters:
	hTCP		- TCP connection to handle incoming data on

  Returns:
	None

  Remarks:
	This function should never be called by an application.  It is used 
	only by the SSL module itself.
  ***************************************************************************/
#if defined(TCPIP_STACK_USE_SSL)
void TCPSSLHandleIncoming(TCP_SOCKET hTCP)
{
	PTR_BASE prevRxTail, nextRxHead, startRxTail, wSrc, wDest;
	uint16_t wToMove, wLen, wSSLBytesThatPoofed, wDecryptedBytes;
	
	if(!_TcpSocketChk(hTCP))
    {
        return;
    }

	// Sync the stub
	SyncTCBStub(hTCP);

	// If new data is waiting
	if(MyTCBStub.sslRxHead != MyTCBStub.rxHead)
	{
		// Reconfigure pointers for SSL use
		prevRxTail = MyTCBStub.rxTail;
		nextRxHead = MyTCBStub.rxHead;
		MyTCBStub.rxTail = MyTCBStub.rxHead;
		MyTCBStub.rxHead = MyTCBStub.sslRxHead;
		
		do
		{
			startRxTail = MyTCBStub.rxTail;

			// Handle incoming data.  This function performs deframing of the 
			// SSL records, decryption, and MAC verification.
			wSSLBytesThatPoofed = TCPIsGetReady(hTCP);
			wDecryptedBytes = SSLRxRecord(hTCP, MyTCBStub.sslStubID);
			wSSLBytesThatPoofed -= TCPIsGetReady(hTCP);

			// Now need to move data to fill the SSL header/MAC/padding hole, 
			// if there is one
			if(wSSLBytesThatPoofed)
			{	
				// Sync the TCP so we can see if there is a TCP hole
				SyncTCB();

				// Calculate how big the SSL hole is
				if(MyTCB.sHoleSize == -1)
				{// Just need to move pending SSL data
					wToMove = TCPIsGetReady(hTCP);
				}
				else
				{// A TCP hole exists, so move all data
					wToMove = TCPIsGetReady(hTCP) + MyTCB.sHoleSize + MyTCB.wFutureDataSize;
				}
				
				// Start with the destination as the startRxTail and source as current rxTail
				wDest = startRxTail;
				wSrc = MyTCBStub.rxTail;
				
				// If data exists between the end of the buffer and 
				// the destination, then move it forward
				if(wSrc > wDest)
				{
					wLen = MyTCBStub.bufferEnd - wSrc + 1;
					if(wLen > wToMove)
						wLen = wToMove;
					TCPRAMCopy(MyTCBStub.pSktNet, wDest, MyTCBStub.vMemoryMedium, 
							   wSrc, MyTCBStub.vMemoryMedium, wLen);
					wDest += wLen;
					wSrc = MyTCBStub.bufferRxStart;
					wToMove -= wLen;
				}
				
				// If data remains to be moved, fill in to end of buffer
				if(wToMove)
				{
					wLen = MyTCBStub.bufferEnd - wDest + 1;
					if(wLen > wToMove)
						wLen = wToMove;
					TCPRAMCopy(MyTCBStub.pSktNet, wDest, MyTCBStub.vMemoryMedium, 
							   wSrc, MyTCBStub.vMemoryMedium, wLen);
					wDest = MyTCBStub.bufferRxStart;
					wSrc += wLen;
					wToMove -= wLen;
				}
				
				// If data still remains, copy from from front + len to front
				if(wToMove)
				{
					TCPRAMCopy(MyTCBStub.pSktNet, wDest, MyTCBStub.vMemoryMedium,
							   wSrc, MyTCBStub.vMemoryMedium, wToMove);
				}

				// Since bytes poofed, we need to move the head pointers 
				// backwards by an equal amount.
				MyTCBStub.rxHead -= wSSLBytesThatPoofed;
				if(MyTCBStub.rxHead < MyTCBStub.bufferRxStart)
					MyTCBStub.rxHead += MyTCBStub.bufferEnd - MyTCBStub.bufferRxStart + 1;
				MyTCBStub.sslRxHead = MyTCBStub.rxHead;
			}
				
			// Move tail pointer forward by the number of decrypted bytes ready 
			// for the application (but not poofed bytes)
			MyTCBStub.rxTail = startRxTail + wDecryptedBytes;
			if(MyTCBStub.rxTail > MyTCBStub.bufferEnd)
				MyTCBStub.rxTail -= MyTCBStub.bufferEnd - MyTCBStub.bufferRxStart + 1;
			nextRxHead += wDecryptedBytes;
			
			// Loop until SSLRxRecord() runs out of data and stops doing 
			// anything
		} while(wSSLBytesThatPoofed || (startRxTail != MyTCBStub.rxTail));

		// Restore TCP buffer pointers to point to the decrypted application data 
		// only
		if(nextRxHead > MyTCBStub.bufferEnd)
			nextRxHead -= MyTCBStub.bufferEnd - MyTCBStub.bufferRxStart + 1;
		MyTCBStub.rxTail = prevRxTail;
		MyTCBStub.rxHead = nextRxHead;
	}
}	
#endif

/*****************************************************************************
  Function:
	void TCPSocketSetNet(TCP_SOCKET hTCP, TCPIP_NET_HANDLE hNet)

  Summary:
	Sets the interface for an TCP socket
	
  Description:
	This function sets the network interface for an TCP socket

  Precondition:
	TCP socket should have been opened with TCPOpen().
    hTCP - valid socket

  Parameters:
	hTCP - The TCP socket
   	hNet - interface handle.
	
  Returns:
    None.

  Note: 
    None.
  ***************************************************************************/
void TCPSocketSetNet(TCP_SOCKET hTCP, TCPIP_NET_HANDLE hNet)
{
    NET_CONFIG* pNetIf = _TCPIPStackHandleToNet(hNet);
    if(pNetIf)
    {
        _TcpSocketBind(TCBStubs + hTCP, pNetIf);
    }
}

/*****************************************************************************
  Function:
	NET_CONFIG* TCPSocketGetNet(TCP_SOCKET hTCP)

  Summary:
	Gets the MAC interface of an TCP socket
	
  Description:
	This function returns the MAC interface id of an TCP socket

  Precondition:
	TCP socket should have been opened with TCPOpen().
    hTCP - valid socket

  Parameters:
	hTCP - The TCP socket
	
  Returns:
    None.
  ***************************************************************************/
TCPIP_NET_HANDLE  TCPSocketGetNet(TCP_SOCKET hTCP)
{
	if(!_TcpSocketChk(hTCP))
    {
        return 0;
    }
    
    SyncTCBStub(hTCP);
    if(MyTCBStub.smState == TCP_CLOSED)
    {
        return 0;
    }

		// Sockets that are in use will be in a non-closed state
    
    return MyTCBStub.pSktNet;
}

// allocates a new ephemeral port number
// returns 0 on error
#if defined(TCPIP_STACK_CLIENT_MODE)
static TCP_PORT TcpAllocateEphemeralPort(void)
{
    int      num_ephemeral;
    int      count;
    TCP_PORT next_ephemeral;


    count = num_ephemeral = LOCAL_TCP_PORT_END_NUMBER - LOCAL_TCP_PORT_START_NUMBER + 1;

    next_ephemeral = LOCAL_TCP_PORT_END_NUMBER + (rand() % num_ephemeral);

    while(count--)
    {
        if(TCPIsAvailablePort(next_ephemeral))
        {
            return next_ephemeral;
        }

        if (next_ephemeral == LOCAL_TCP_PORT_END_NUMBER)
        {
            next_ephemeral = LOCAL_TCP_PORT_START_NUMBER;
        }
        else
        {
            next_ephemeral++;
        }
    }

    return 0;   // not found

}

static bool TCPIsAvailablePort(TCP_PORT port)
{
    TCP_SOCKET hTCP;

    // Find an available socket that matches the specified socket type
    for(hTCP = 0; hTCP < TcpSockets; hTCP++)
    {
        SyncTCBStub(hTCP);

        // Sockets that are in use will be in a non-closed state
        if(MyTCBStub.smState != TCP_CLOSED)
        {

            SyncTCB();

            if( MyTCB.localPort.Val == port)
            {
                return false;
            }
        }
    }

    return true;
}
#endif  // defined(TCPIP_STACK_CLIENT_MODE)

/*****************************************************************************
  Function:
	TCPBind(TCP_SOCKET s, IP_ADDRESS_TYPE addType, TCP_PORT localPort,  IP_MULTI_ADDRESS* localAddress)

  Summary:
    Bind a socket to a local address
    This function is meant for client sockets.
    It is similar to TCPSocketSetNet() that assigns a specific source interface for a socket.
    If localPort is 0 the stack will assign a unique local port

  Description:
    Sockets don't need specific binding, it is done automatically by the stack
    However, specific binding can be requested using these functions.
    Works for both client and server sockets.
    TBD: the call should fail if the socket is already bound to an interface
    (a server socket is conencted or a client socket already sent the data on an interface).
    Implementation pending
  ***************************************************************************/
bool TCPBind(TCP_SOCKET hTCP, IP_ADDRESS_TYPE addType, TCP_PORT localPort,  IP_MULTI_ADDRESS* localAddress)
{
    NET_CONFIG* pSktIf;

	if(!_TcpSocketChk(hTCP))
    {
        return false;
    }

    if(localPort == 0)
    {
        localPort = TcpAllocateEphemeralPort();
        if(localPort == 0)
        {
            return false;
        }
    }

    SyncTCBStub(hTCP);
    SyncTCB();

    if(MyTCBStub.smState == TCP_CLOSED)
    {
        return false;
    }

    if(localAddress && localAddress->v4Add.Val != 0)
    {
        pSktIf = _TCPIPStackIpAddToNet(&localAddress->v4Add, false);
        if(pSktIf == 0)
        {    // no such interface
            return false;
        }
    }
    else
    {
        pSktIf = (NET_CONFIG*)TCPIP_STACK_GetDefaultNet();
    }

    _TcpSocketBind(&MyTCBStub, pSktIf);
    MyTCB.localPort.Val = localPort;
    return true;
}

/*****************************************************************************
  Function:
	bool TCPRemoteBind(TCP_SOCKET hTCP, IP_ADDRESS_TYPE addType, TCP_PORT remotePort,  IP_MULTI_ADDRESS* remoteAddress)

  Summary:
    Bind a socket to a remote address
    This function is meant for server sockets.

  Description:
    Sockets don't need specific remote binding, they should accept connections on any incoming interface.
    Thus the binding is done automatically by the stack.
    However, specific remote binding can be requested using these functions.
    For a server socket it can be used to restrict accepting connections from  a specific remote host.
    For a client socket it will just change the default binding done when the socket was opened.
    TBD: the call should fail if the socket is already bound to an interface
    (a server socket is conencted or a client socket already sent the data on an interface).
    Implementation pending
  ***************************************************************************/
bool TCPRemoteBind(TCP_SOCKET hTCP, IP_ADDRESS_TYPE addType, TCP_PORT remotePort,  IP_MULTI_ADDRESS* remoteAddress)
{
    
    if(TCPSetDestinationIPAddress(hTCP, addType, remoteAddress))
    {
        SyncTCBStub(hTCP);
        SyncTCB();
        MyTCB.remotePort.Val = remotePort;
        return true;
    }

    return false;
}

// Allows setting options to a socket like Rx/Tx buffer size, etc
bool TCPSetOptions(TCP_SOCKET s, int option, int param)
{
    return false;
}

bool TCPSetDestinationIPAddress(TCP_SOCKET hTCP, IP_ADDRESS_TYPE addType, IP_MULTI_ADDRESS* remoteAddress)
{

	if(!_TcpSocketChk(hTCP))
    {
        return false;
    }

	SyncTCBStub(hTCP);
	SyncTCB();

	if(MyTCBStub.smState == TCP_CLOSED)
    {
        return false;
    }
    
#if defined (TCPIP_STACK_USE_IPV6)
    if (MyTCBStub.pTxPkt->flags.addressType == IP_ADDRESS_TYPE_IPV6)
    {
        TCPIP_IPV6_SetDestAddress (MyTCBStub.pTxPkt, remoteAddress?&remoteAddress->v6Add:0);
    }
    else
#endif
    {
        TCPIP_IPV4_SetDestAddress (MyTCBStub.pTxPkt, remoteAddress?remoteAddress->v4Add.Val:0);
    }
    return true;
}


// sets the source IP address of a packet
bool TCPSetSourceIPAddress(TCP_SOCKET hTCP, IP_ADDRESS_TYPE addType, IP_MULTI_ADDRESS* localAddress)
{

	if(!_TcpSocketChk(hTCP))
    {
        return false;
    }

	SyncTCBStub(hTCP);
	SyncTCB();

	if(MyTCBStub.smState == TCP_CLOSED)
    {
        return false;
    }
    
#if defined (TCPIP_STACK_USE_IPV6)
    if (MyTCBStub.pTxPkt->flags.addressType == IP_ADDRESS_TYPE_IPV6)
        TCPIP_IPV6_SetSourceAddress(MyTCBStub.pTxPkt, localAddress?&localAddress->v6Add:0);
    else
#endif
    {
        TCPIP_IPV4_SetSourceAddress(MyTCBStub.pTxPkt, localAddress?localAddress->v4Add.Val:0);
    }
    return true;

}

#endif //#if defined(TCPIP_STACK_USE_TCP)
