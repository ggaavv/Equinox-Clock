/*******************************************************************************
  Transmission Control Protocol (TCP) Configuration file

  Summary:
    TCP configuration file
    
  Description:
    This file contains the TCP module configuration options
    
*******************************************************************************/

/*******************************************************************************
FileName:   tcp_config.h
Copyright © 2011 released Microchip Technology Inc.  All rights 
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

#ifndef _TCP_CONFIG_H_
#define _TCP_CONFIG_H_

// First port number for randomized local port number selection
// Use the dynamic port range defined by IANA consists of the 49152-65535 range
// and is meant for the selection of ephemeral ports (RFC 6056).
// Adjust to your needs but stay within the IANA range 
#define LOCAL_TCP_PORT_START_NUMBER (49152)

// Last port number for randomized local port number selection
#define LOCAL_TCP_PORT_END_NUMBER   (65535)

// TCP Maximum Segment Size for TX.  The TX maximum segment size is actually 
// govered by the remote node's MSS option advirtised during connection 
// establishment.  However, if the remote node specifies an unhandlably large 
// MSS (ex: > Ethernet MTU), this define sets a hard limit so that we don't 
// cause any TX buffer overflows.  If the remote node does not advirtise a MSS 
// option, all TX segments are fixed at 536 bytes maximum.
#define TCP_MAX_SEG_SIZE_TX			(1460u)

// TCP Maximum Segment Size for RX.  This value is advirtised during connection 
// establishment and the remote node should obey it.  This should be set to 536 
// to avoid IP layer fragmentation from causing packet loss.  However, raising 
// its value can enhance performance at the (small) risk of introducing 
// incompatibility with certain special remote nodes (ex: ones connected via a 
// slow dial up modem).
#define TCP_MAX_SEG_SIZE_RX			(536u)

// For debugging only.  Normal applications should never enable these
//#define DEBUG_GENERATE_TX_LOSS		62257
//#define DEBUG_GENERATE_RX_LOSS		64225

// TCP Timeout and retransmit numbers
// All timeouts in milliseconds
// Timeout to retransmit unacked data, ms
#define TCP_START_TIMEOUT_VAL   	(1000ul)

// Timeout for delayed-acknowledgement algorithm, ms
#define TCP_DELAYED_ACK_TIMEOUT		(100ul)

// Timeout for FIN WAIT 2 state, ms
#define TCP_FIN_WAIT_2_TIMEOUT		(5000ul)

// Timeout for keep-alive messages when no traffic is sent, ms
#define TCP_KEEP_ALIVE_TIMEOUT		(10000ul)

// Timeout for the CLOSE_WAIT state, ms
#define TCP_CLOSE_WAIT_TIMEOUT		(200ul)

// Maximum number of retransmission attempts
#define TCP_MAX_RETRIES			(5u)

// Maximum number of keep-alive messages that can be sent 
// without receiving a response before automatically closing 
// the connection
#define TCP_MAX_UNACKED_KEEP_ALIVES	(6u)

// Smaller than all other retries to reduce SYN flood DoS duration
#define TCP_MAX_SYN_RETRIES		(2u)

// Timeout before automatically transmitting unflushed data, ms; Default 40 ms
#define TCP_AUTO_TRANSMIT_TIMEOUT_VAL	(40ul)

// Timeout before automatically transmitting a window update due to a TCPGet() or TCPGetArray() function call, ms
#define TCP_WINDOW_UPDATE_TIMEOUT_VAL	(200ul)

// Use queueing for TCP RX SYN packets that cannot be serviced immediately
#define TCP_SYN_QUEUEING        	1

// Default Number of TCP RX SYN packets to save if they cannot be serviced immediately
#define TCP_SYN_QUEUE_MAX_ENTRIES	(3u)

// Timeout for when SYN queue entries are deleted if unserviceable, ms
#define TCP_SYN_QUEUE_TIMEOUT		(3000ul)

// A lot of pointer dereference code can be removed if you 
// locally copy TCBStubs to an absolute memory location.
// If you define TCP_OPTIMIZE_FOR_SIZE, local caching will 
// occur and will substantially decrease the entire TCP const 
// footprint (up to 35%).  If you leave TCP_OPTIMIZE_FOR_SIZE 
// undefined, the local caching will be disabled.
#define TCP_OPTIMIZE_FOR_SIZE

// For smallest size and best throughput, TCP_OPTIMIZE_FOR_SIZE 
// should always be enabled on PIC24/dsPIC products.  On PIC32 
// products there is very little difference and depnds on compiler 
// optimization level
#if defined(__C30__) && !defined(TCP_OPTIMIZE_FOR_SIZE)
	#define TCP_OPTIMIZE_FOR_SIZE
#elif defined(__C32__) && defined(TCP_OPTIMIZE_FOR_SIZE)
	#undef TCP_OPTIMIZE_FOR_SIZE
#endif

// The number of ticks per second to generate an TCP tick.
// Used by the TCP state machine
// Note: the System Tick resolution in system_profile.h (SYS_TICKS_PER_SECOND) has to 
// be fine enough to allow for this TCP tick granularity.  
#define TCP_TICKS_PER_SECOND        (200)        // 5 ms default tick 


// Memory Configuration
//   The following section sets up the memory types for use by
//   this application.

// Represents data stored in Ethernet buffer RAM
#define TCP_ETH_RAM	0u

// The base address for TCP data in Ethernet RAM
#define TCP_ETH_RAM_BASE_ADDRESS			(BASE_TCB_ADDR)

// Represents data stored in local PIC RAM
#define TCP_PIC_RAM	1u

// Represents data stored in external SPI RAM
#define TCP_SPI_RAM	2u

// The base address for TCP data in SPI RAM
#define TCP_SPI_RAM_BASE_ADDRESS			(0x00)

// Represents data stored in dynamic PIC RAM
#define TCP_PIC_DRAM	3u

//END Memory Config


// List of MAX_PROTO_CONNECTIONS:
// Number of simultaneous connections to support for each TCP protocol
// If !defined(TCP_MAX_SOCKETS) then,
// based on these selections (when the corresponding service is enabled)
// the stack will determinate the corresponding number of TCP sockets to create
// 
// NOTES: - in dynamic configuration (defined(TCP_DYNAMIC_CONFIGURATION)) which is the default/preferred one
//          the number of connections is dictated solely by the MAX_PROTO_CONNECTIONS parameter.
//        
//        - in static configuration (!defined(TCP_DYNAMIC_CONFIGURATION)) the number of actual
//          opened sockets (connections) is dictated by the minimum between:
//              - the number of the entries in TCPSocketInitializer[] having the corresponding
//                TCP_PURPOSE_ type.
//              - MAX_PROTO_CONNECTIONS defined below
 
//        - when calculating the number of TCP sockets the number of interfaces is not taken into account.
//         
// Adjust your MAX_PROTO_CONNECTIONS values accordingly depending on the application.

// Transport Layer Configuration
// 	The following low level modules are automatically enabled
// 	based on module selections above.  If your custom module
// 	requires them otherwise, enable them here.
//#define TCPIP_STACK_USE_TCP
//#define TCPIP_STACK_USE_UDP

// TCP Socket Memory Allocation
//   TCP needs memory to buffer incoming and outgoing data.  The
//   amount and medium of storage can be allocated on a per-socket
//   basis using the example below as a guide.

// Allocate how much total RAM (in bytes) you want to allocate
// for use by your TCP TCBs, RX FIFOs, and TX FIFOs.

#define TCP_ETH_RAM_SIZE				(0ul)
//#define TCP_PIC_RAM_SIZE				(0ul)
//#define TCP_SPI_RAM_SIZE				(0ul)

	
#define TCP_DYNAMIC_CONFIGURATION
// Default Dynamic TCP configuration
// You also have the option to use static configuration
// by commenting out TCP_DYNAMIC_CONFIGURATION (see below)
	
#if defined (TCP_DYNAMIC_CONFIGURATION)

	// default socket purpose
	#define TCP_SOCKET_DEFAULT_PURPOSE		TCP_PURPOSE_ANY		

	// default socket medium
	#define TCP_SOCKET_DEFAULT_MEMORY_MEDIUM	TCP_PIC_DRAM		

	// default socket Tx buffer size
	#define TCP_SOCKET_DEFAULT_TX_SIZE		256//512			

	// default socket Rx buffer size
	#define TCP_SOCKET_DEFAULT_RX_SIZE		256//512			
		
	// global number of TCP sockets created dynamically
	// this number of TCP sockets will be created
	// each having TCP_SOCKET_DEFAULT_PURPOSE,
	// TCP_SOCKET_DEFAULT_TX_SIZE and TCP_SOCKET_DEFAULT_RX_SIZE
	// NOTES:
	//	- When defining TCP_MAX_SOCKETS 
	//	  MATCH them accordingly with the MAX_PROTO_CONNECTIONS
	//	  defined in the Application configuration section!
	//	- When defining TCP_MAX_SOCKETS take into account
	//	  the NUMBER OF INTERFACES the stack is supporting 
	//	- COMMENT OUT this define and the number of needed TCP sockets
	//	  will be calculated using the MAX_PROTO_CONNECTIONS
	//	  in the Application configuration section!
	#define TCP_MAX_SOCKETS 			(3u)//(10u)

#else
	// Statically defined TCP configuration
    // NOTE: This is for backwards compatibility purposes only!
    // New projects should not need to pre-configure the socket allocation
    // The stack allocates sockets when needed based on the available resources.
    //
    //
    // 
	// Define what types of sockets are needed, how many of
	// each to include, where their TCB, TX FIFO, and RX FIFO
	// should be stored, and how big the RX and TX FIFOs should
	// be.	Making this initializer bigger or smaller defines
	// how many total TCP sockets are available.

	// Each socket requires up to 56 bytes of PIC RAM and
	// 48+(TX FIFO size)+(RX FIFO size) bytes of TCP_*_RAM each.

	// Note: The RX FIFO must be at least 1 byte in order to
	// receive SYN and FIN messages required by TCP.  The TX
	// FIFO can be zero if desired.
		
	// Structure for TCP socket configuration 
	typedef struct
	{
		BYTE vSocketPurpose;
		BYTE vMemoryMedium;
		WORD wTXBufferSize;
		WORD wRXBufferSize;
	}TCP_SOCKET_CONF;

	static ROM TCP_SOCKET_CONF TCPSocketInitializer[] =
	{
//		{TCP_PURPOSE_GENERIC_TCP_CLIENT, TCP_PIC_DRAM, 25, 120},
//		{TCP_PURPOSE_GENERIC_TCP_SERVER, TCP_PIC_DRAM, 20, 20},
//		{TCP_PURPOSE_TELNET, TCP_PIC_DRAM, 200, 150},
//		{TCP_PURPOSE_TELNET, TCP_PIC_DRAM, 200, 150},
		//{TCP_PURPOSE_TELNET, TCP_PIC_DRAM, 200, 150},
		//{TCP_PURPOSE_FTP_COMMAND, TCP_PIC_DRAM, 100, 40},
		//{TCP_PURPOSE_FTP_DATA, TCP_PIC_DRAM, 0, 128},
//		{TCP_PURPOSE_TCP_PERFORMANCE_TX, TCP_PIC_DRAM, 2000, 1},
		//{TCP_PURPOSE_TCP_PERFORMANCE_RX, TCP_PIC_DRAM, 40, 2000},
//		{TCP_PURPOSE_UART_2_TCP_BRIDGE, TCP_PIC_DRAM, 256, 256},
		{TCP_PURPOSE_HTTP_SERVER, TCP_PIC_DRAM, 100, 100},
		{TCP_PURPOSE_HTTP_SERVER, TCP_PIC_DRAM, 100, 100},
		{TCP_PURPOSE_DEFAULT, TCP_PIC_DRAM, 100, 100},
//		{TCP_PURPOSE_BERKELEY_SERVER, TCP_PIC_DRAM, 25, 20},
	//	{TCP_PURPOSE_BERKELEY_SERVER, TCP_PIC_DRAM, 25, 20},
		//{TCP_PURPOSE_BERKELEY_SERVER, TCP_PIC_DRAM, 25, 20},
//		{TCP_PURPOSE_BERKELEY_CLIENT, TCP_PIC_DRAM, 125, 100},
	};
#endif	// defined(TCP_DYNAMIC_CONFIGURATION)

// TCP layer configuration/initialization
typedef struct
{
    int             nSockets;   // number of sockets to be created
    const void*     pTcpSktInit;// pointer to an initialization structure
                                // if !NULL, it's an array of nSockets initialization structures 
}TCP_MODULE_CONFIG;

static const TCP_MODULE_CONFIG  tcpConfigData = 
{
#if defined(TCP_MAX_SOCKETS)
    TCP_MAX_SOCKETS,
#else
    0,
#endif

#if defined(TCP_DYNAMIC_CONFIGURATION)
    0
#else
    (const void*)TCPSocketInitializer
#endif  // defined(TCP_DYNAMIC_CONFIGURATION)
};

#endif  // _TCP_CONFIG_H_
