/*******************************************************************************
  TCP Module Defs for Microchip TCP/IP Stack

  Summary:
    
  Description:
*******************************************************************************/

/*******************************************************************************
FileName:  tcp.h 
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

#ifndef __TCP_H__
#define __TCP_H__


/****************************************************************************
  Section:
	Type Definitions
  ***************************************************************************/

// TCP Port Number identifier
typedef uint16_t TCP_PORT;

// A TCP_SOCKET is stored as a single uint8_t
typedef int16_t TCP_SOCKET;

#define INVALID_SOCKET      (-1)	// The socket is invalid or could not be opened


// Information about a socket
typedef struct
{
    IP_MULTI_ADDRESS    remoteIPaddress;
    IP_ADDRESS_TYPE     addressType;
    MAC_ADDR remoteMACAddr; 
    TCP_PORT remotePort;	// Port number associated with remote node
} TCP_SOCKET_INFO;


// adjust socket buffer sizes
typedef enum
{
    TCP_ADJUST_GIVE_REST_TO_RX  = 0x01, 	// Resize flag: extra bytes go to RX 
    TCP_ADJUST_GIVE_REST_TO_TX  = 0x02, 	// Resize flag: extra bytes go to TX
    TCP_ADJUST_PRESERVE_RX	    = 0x04,     // Resize flag: attempt to preserve RX buffer
    TCP_ADJUST_PRESERVE_TX		= 0x08,     // Resize flag: attempt to preserve TX buffer
}TCP_ADJUST_FLAGS;

/****************************************************************************
  Section:
	Function Declarations
  ***************************************************************************/

/*****************************************************************************
  Function:
	 TCP_SOCKET TCPOpenServer(IP_ADDRESS_TYPE addType, TCP_PORT localPort,  IP_MULTI_ADDRESS* localAddress)

  Summary:
	Opens a TCP socket as a server.
	
  Description:
	Provides a unified method for opening TCP server sockets. 

	Sockets are statically/dynamically allocated on boot, and can be claimed with this
    function and freed using TCPClose.

  Precondition:
    TCP is initialized.

  Parameters:
    IP_ADDRESS_TYPE addType			-	The type of address being used. Example: IP_ADDRESS_TYPE_IPV4.
    
    TCP_PORT localPort				-	The local TCP port on which to listen for connections. 
    
    IP_MULTI_ADDRESS* localAddress	-	Local address to use.
                                        Can be NULL for multi-homed hosts when any incoming interface is acceptable
	
  Returns:
 	INVALID_SOCKET -  No sockets of the specified type were available to be
                      opened.

    Otherwise -       A TCP_SOCKET handle. Save this handle and use it when
                      calling all other TCP APIs. 
 ***************************************************************************/
TCP_SOCKET          TCPOpenServer(IP_ADDRESS_TYPE addType, TCP_PORT localPort,  IP_MULTI_ADDRESS* localAddress);

/*****************************************************************************
  Function:
	 TCP_SOCKET TCPOpenClient(IP_ADDRESS_TYPE addType, TCP_PORT remotePort, IP_MULTI_ADDRESS* remoteAddress)

  Summary:
	Opens a TCP socket as a client.
	
  Description:
	Provides a unified method for opening TCP client sockets. 

	Sockets are statically/dynamically allocated on boot, and can be claimed with this
    function and freed using TCPDisconnect or TCPClose.

  Precondition:
    TCP is initialized.

  Parameters:
    IP_ADDRESS_TYPE addType			-	The type of address being used. Example: IP_ADDRESS_TYPE_IPV4.
    
    TCP_PORT remotePort				-	TCP remote port to which a connection should be made.
                                        The local port for client sockets will be automatically picked
                                        by the TCP module.
                                        
    IP_MULTI_ADDRESS* remoteAddress	-	The remote address to connect to.
	
  Returns:
 	INVALID_SOCKET -  No sockets of the specified type were available to be
                      opened.

    Otherwise -       A TCP_SOCKET handle. Save this handle and use it when
                      calling all other TCP APIs. 
 ***************************************************************************/
TCP_SOCKET          TCPOpenClient(IP_ADDRESS_TYPE addType, TCP_PORT remotePort, IP_MULTI_ADDRESS* remoteAddress);

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

  Precondition:
    TCP is initialized.

  Parameters:
    s				-	Socket to bind
    addType			-	The type of address being used. Example: IP_ADDRESS_TYPE_IPV4.
    localPort		-	Local port to use
    localAddress	-	The local address to bind to.

  Returns:
 	True of success
    false otherwise	
 ***************************************************************************/
bool                TCPBind(TCP_SOCKET s, IP_ADDRESS_TYPE addType, TCP_PORT localPort,  IP_MULTI_ADDRESS* localAddress);

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

  Precondition:
    TCP is initialized.

  Parameters:
    hTCP			-	Socket to bind
    addType			-	The type of address being used. Example: IP_ADDRESS_TYPE_IPV4.
    remotePort		-	remote port to use
    remoteAddress	-	The remote address to bind to.

  Returns:
 	True of success
    false otherwise
  ***************************************************************************/
bool                TCPRemoteBind(TCP_SOCKET s, IP_ADDRESS_TYPE addType, TCP_PORT remotePort,  IP_MULTI_ADDRESS* remoteAddress);


/*****************************************************************************
  Function:
	bool TCPRemoteBind(TCP_SOCKET hTCP, IP_ADDRESS_TYPE addType, TCP_PORT remotePort,  IP_MULTI_ADDRESS* remoteAddress)

  Summary:
    Allows setting options to a socket like adjust Rx/Tx buffer size, etc

  Description:

  Precondition:

  Parameters:
    hTCP			-	
	addType			-	
	remotePort		-	
	remoteAddress	-	

  Returns:
 	True of success
    false otherwise

  Remarks:
    Not implemented yet!
  ***************************************************************************/
bool                TCPSetOptions(TCP_SOCKET s, int option, int param);

/*****************************************************************************
  Function:
	bool TCPIsConnected(TCP_SOCKET hTCP)

  Summary:
	Determines if a socket has an established connection.

  Description:
	This function determines if a socket has an established connection to 
	a remote node.  Call this function after calling TCPOpenServer()/TCPOpenClient()
    to determine when the connection is set up and ready for use.  This function was 
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
bool                TCPIsConnected(TCP_SOCKET hTCP);

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
bool                TCPWasReset(TCP_SOCKET hTCP);

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
void                TCPDisconnect(TCP_SOCKET hTCP);

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
void                TCPClose(TCP_SOCKET hTCP);

/*****************************************************************************
  Function:
	bool TCPGetSocketInfo(TCP_SOCKET hTCP, TCP_SOCKET_INFO* pInfo)

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
bool                TCPGetSocketInfo(TCP_SOCKET hTCP, TCP_SOCKET_INFO* pInfo);

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
uint16_t            TCPIsPutReady(TCP_SOCKET hTCP);

/*****************************************************************************
  Function:
	uint16_t TCPPutArray(TCP_SOCKET hTCP, const uint8_t* Data, uint16_t Len)

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
uint16_t            TCPPutArray(TCP_SOCKET hTCP, const uint8_t* Data, uint16_t Len);

/*****************************************************************************
  Function:
	const uint8_t* TCPPutString(TCP_SOCKET hTCP, const uint8_t* Data)

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
const uint8_t*      TCPPutString(TCP_SOCKET hTCP, const uint8_t* Data);

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
void                TCPFlush(TCP_SOCKET hTCP);

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
uint16_t            TCPGetTxFIFOFull(TCP_SOCKET hTCP);


// Alias to TCPIsPutReady provided for API completeness
#define             TCPGetTxFIFOFree(a) 				TCPIsPutReady(a)

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

  Remarks:
    Note that the following function is inefficient.
    A buffered approach (TCPPutArray) should be preferred
  ***************************************************************************/
bool                TCPPut(TCP_SOCKET hTCP, uint8_t byte);

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
uint16_t            TCPIsGetReady(TCP_SOCKET hTCP);

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
uint16_t            TCPGetArray(TCP_SOCKET hTCP, uint8_t* buffer, uint16_t count);

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
uint8_t             TCPPeek(TCP_SOCKET hTCP, uint16_t wStart);

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
uint16_t            TCPPeekArray(TCP_SOCKET hTCP, uint8_t *vBuffer, uint16_t wLen, uint16_t wStart);

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
uint16_t            TCPGetRxFIFOFree(TCP_SOCKET hTCP);


// Alias to TCPIsGetReady provided for API completeness
#define             TCPGetRxFIFOFull(a)					TCPIsGetReady(a)

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
void                TCPDiscard(TCP_SOCKET hTCP);

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

  Remarks:
    Note that the following function is inefficient.
    A buffered approach (TCPGetArray) should be preferred
  ***************************************************************************/
bool                TCPGet(TCP_SOCKET hTCP, uint8_t* byte);

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
uint16_t            TCPFind(TCP_SOCKET hTCP, uint8_t cFind, uint16_t wStart, uint16_t wSearchLen, bool bTextCompare);

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
uint16_t            TCPFindArray(TCP_SOCKET hTCP, const uint8_t* cFindArray, uint16_t wLen, uint16_t wStart, uint16_t wSearchLen, bool bTextCompare);

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
bool                TCPAdjustFIFOSize(TCP_SOCKET hTCP, uint16_t wMinRXSize, uint16_t wMinTXSize, TCP_ADJUST_FLAGS vFlags);

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
bool                TCPStartSSLClient(TCP_SOCKET hTCP, uint8_t* host);

/*****************************************************************************
  Function:
	bool TCPStartSSLClientEx(TCP_SOCKET hTCP, uint8_t* host, void * buffer, uint8_t suppDataType)

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
bool                TCPStartSSLClientEx(TCP_SOCKET hTCP, uint8_t* host, void * buffer, uint8_t suppDataType);

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
bool                TCPStartSSLServer(TCP_SOCKET hTCP);

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
bool                TCPAddSSLListener(TCP_SOCKET hTCP, uint16_t port);

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
bool                TCPRequestSSLMessage(TCP_SOCKET hTCP, uint8_t msg);

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
bool                TCPSSLIsHandshaking(TCP_SOCKET hTCP);

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
bool                TCPIsSSL(TCP_SOCKET hTCP);

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
void                TCPSSLHandshakeComplete(TCP_SOCKET hTCP);

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
void                TCPSSLDecryptMAC(TCP_SOCKET hTCP, ARCFOUR_CTX* ctx, uint16_t len);

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
void                TCPSSLInPlaceMACEncrypt(TCP_SOCKET hTCP, ARCFOUR_CTX* ctx, uint8_t* MACSecret, uint16_t len);

/*****************************************************************************
  Function:
	void TCPSocketSetNet(TCP_SOCKET hTCP, TCPIP_NET_HANDLE hNet)

  Summary:
	Sets the interface for an TCP socket
	
  Description:
	This function sets the network interface for an TCP socket

  Precondition:
	TCP socket should have been opened with TCPOpenServer()/TCPOpenClient()().
    hTCP - valid socket

  Parameters:
	hTCP - The TCP socket
   	hNet - interface handle.
	
  Returns:
    None.

  Note: 
    None
  ***************************************************************************/
void                TCPSocketSetNet(TCP_SOCKET hTCP, TCPIP_NET_HANDLE hNet);

/*****************************************************************************
  Function:
	NET_CONFIG* TCPSocketGetNet(TCP_SOCKET hTCP)

  Summary:
	Gets the MAC interface of an TCP socket
	
  Description:
	This function returns the MAC interface id of an TCP socket

  Precondition:
	TCP socket should have been opened with TCPOpenServer()/TCPOpenClient()().
    hTCP - valid socket

  Parameters:
	hTCP - The TCP socket
	
  Returns:
    None.
  ***************************************************************************/
TCPIP_NET_HANDLE    TCPSocketGetNet(TCP_SOCKET hTCP);

/*  Obsolete TCP calls that have been remove form the API.
 *  No longer supported!
*/


// TCPOpen() is obsolete and no longer supported.
// use TCPOpenServer()/TCPOpenClient() instead
// TCP_SOCKET TCPOpen(uint32_t dwRemoteHost, uint8_t vRemoteHostType, uint16_t wPort, uint8_t vSocketPurpose);


#endif  // __TCP_H__
