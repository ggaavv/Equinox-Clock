/*******************************************************************************
  UDP Module Defs for Microchip TCP/IP Stack

  Summary:
    
  Description:
*******************************************************************************/

/*******************************************************************************
FileName:  udp.h 
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

#ifndef __UDP_H_
#define __UDP_H_


// Stores a UDP Port Number
typedef uint16_t UDP_PORT;

// Provides a handle to a UDP Socket
typedef int16_t UDP_SOCKET;


#define INVALID_UDP_SOCKET      (-1)		// Indicates a UDP socket that is not valid
#define INVALID_UDP_PORT        (0ul)		// Indicates a UDP port that is not valid

// Information about a socket
typedef struct
{
    IP_MULTI_ADDRESS    remoteIPaddress;
    IP_MULTI_ADDRESS    localIPaddress;
    IP_ADDRESS_TYPE     addressType;
	UDP_PORT            remotePort;	// Port number associated with remote node
    UDP_PORT            localPort;  // local port number
} UDP_SOCKET_INFO;



/****************************************************************************
  Section:
	Function Prototypes
  ***************************************************************************/

/*****************************************************************************
  Function:
	 UDP_SOCKET UDPOpenServer(IP_ADDRESS_TYPE addType, UDP_PORT localPort,  IP_MULTI_ADDRESS* localAddress)

  Summary:
	Opens a UDP socket as a server.
	
  Description:
	Provides a unified method for opening UDP server sockets. 

  Precondition:
    UDP is initialized.

  Parameters:
    IP_ADDRESS_TYPE addType			-	The type of address being used. Example: IP_ADDRESS_TYPE_IPV4.
    
    UDP_PORT localPort				-	UDP port on which to listen for connections
    
    IP_MULTI_ADDRESS* localAddress	-	Local address to use.
                                        Can be NULL if any incoming interface will do.
	
  Returns:
 	INVALID_SOCKET -  No sockets of the specified type were available to be
                      opened.

    Otherwise -       A UDP_SOCKET handle. Save this handle and use it when
                      calling all other UDP APIs. 
 ***************************************************************************/
UDP_SOCKET          UDPOpenServer(IP_ADDRESS_TYPE addType, UDP_PORT localPort,  IP_MULTI_ADDRESS* localAddress);

/*****************************************************************************
  Function:
	 UDPOpenClient(IP_ADDRESS_TYPE addType, UDP_PORT remotePort, IP_MULTI_ADDRESS* remoteAddress)

  Summary:
	Opens a UDP socket as a client.
	
  Description:
	Provides a unified method for opening UDP client sockets. 

  Precondition:
    UDP is initialized.

  Parameters:
    IP_ADDRESS_TYPE addType			-	The type of address being used. Example: IP_ADDRESS_TYPE_IPV4.

    UDP_PORT remotePort				-   The remote UDP port to which a connection should be made.
                                        The local port for client sockets will be automatically picked
                                        by the UDP module.
                                        
    IP_MULTI_ADDRESS* remoteAddress	-	The remote address to connect to.
	
  Returns:
 	INVALID_SOCKET -  No sockets of the specified type were available to be
                      opened.

    Otherwise -       A UDP_SOCKET handle. Save this handle and use it when
                      calling all other UDP APIs. 
 ***************************************************************************/
UDP_SOCKET          UDPOpenClient(IP_ADDRESS_TYPE addType, UDP_PORT remotePort, IP_MULTI_ADDRESS* remoteAddress);

/*****************************************************************************
  Function:
    bool UDPBind(UDP_SOCKET s, IP_ADDRESS_TYPE addType, UDP_PORT localPort,  IP_MULTI_ADDRESS* localAddress);

  Summary:
    Bind a socket to a local address
    This function is meant for client sockets.
    It is similar to UDPSocketSetNet() that assigns a specific source interface for a socket.
    If localPort is 0 the stack will assign a unique local port

  Description:
    Sockets don't need specific binding, it is done automatically by the stack
    However, specific binding can be requested using these functions.
    Works for both client and server sockets.
    TBD: the call should fail if the socket is already bound to an interface
    (a server socket is conencted or a client socket already sent the data on an interface).
    Implementation pending

  Precondition:
	UDP is initialized.

  Parameters:
	s				-	The socket to bind.
	addType			-	The type of address being used. Example: IP_ADDRESS_TYPE_IPV4.
	localPort		-	The local port to bind to.
	localAddress	-   Local address to use.
	
  Returns:
	True if success
	False otherwise

  ***************************************************************************/
bool                UDPBind(UDP_SOCKET s, IP_ADDRESS_TYPE addType, UDP_PORT localPort,  IP_MULTI_ADDRESS* localAddress);

/*****************************************************************************
  Function:
    bool UDPBind(UDP_SOCKET s, IP_ADDRESS_TYPE addType, UDP_PORT localPort,  IP_MULTI_ADDRESS* localAddress);

  Summary:
    Bind a socket to a remote address
    This function is meant for server sockets.

  Description:
    Sockets don't need specific binding, it is done automatically by the stack
    However, specific binding can be requested using these functions.
    Works for both client and server sockets.
    TBD: the call should fail if the socket is already bound to an interface
    (a server socket is conencted or a client socket already sent the data on an interface).
    Implementation pending

  Precondition:
	UDP is initialized.

  Parameters:
	s				-	The socket to bind.
	addType			-	The type of address being used. Example: IP_ADDRESS_TYPE_IPV4.
	localPort		-	The local port to bind to.
	localAddress	-   Local address to use.
	
  Returns:
	True if success
	False otherwise

  ***************************************************************************/
bool                UDPRemoteBind(UDP_SOCKET s, IP_ADDRESS_TYPE addType, UDP_PORT remotePort,  IP_MULTI_ADDRESS* remoteAddress);


// Allows setting options to a socket like enable broadcast, Rx/Tx buffer size, etc
// TBD: Not implemented yet!
bool                UDPSetOptions(UDP_SOCKET s, int option, int param);

/******************************************************************************
  Function:
	  bool UDPISOpened(UDP_SOCKET socket)
  
 Summary:
	  Determines if a socket has an established connection.

 Description:
	This function determines if a socket has an established connection to a remote node .  
	Call this function after calling UDPOpenServer()/UDPOpenClient()
    to determine when the connection is set up and ready for use.  

 Precondition:
	UDP is initialized.

 Parameters:
	s - The socket to check.

 Return Values:
	true - The socket has been opened and ARP has been resolved.
	false - The socket is not currently connected.

 Remarks:
	None
 *****************************************************************************/
bool                UDPIsOpened(UDP_SOCKET socket);

/*****************************************************************************
  Function:
	void UDPClose(UDP_SOCKET s)

  Summary:
	Closes a UDP socket and frees the handle.
	
  Description:
	Closes a UDP socket and frees the handle.  Call this function to release
	a socket and return it to the pool for use by future communications.

  Precondition:
	UDPInit() must have been previously called.

  Parameters:
	s - The socket handle to be released.  If an illegal handle value is 
		provided, the function safely does nothing.

  Returns:
  	None
  	
  Remarks:
	This function does not affect the previously designated active socket.
  ***************************************************************************/
void                UDPClose(UDP_SOCKET s);

/*****************************************************************************
  Function:
	bool UDPGetSocketInfo(UDP_SOCKET s, UDP_SOCKET_INFO* pInfo)

  Summary:
	Points handle at socket unfo for socket s
	
  Description:

  Precondition:

  Parameters:
  	UDP_SOCKET s - Socket to obtain info for.
	UDP_SOCKET_INFO* pInfo - pointer to reference info location.

  Returns:
  ***************************************************************************/
bool                UDPGetSocketInfo(UDP_SOCKET s, UDP_SOCKET_INFO* pInfo);

/*****************************************************************************
  Function:
	bool UDPSetTxOffset(UDP_SOCKET s, uint16_t wOffset)

  Summary:
	Moves the pointer within the TX buffer.
	
  Description:
	This function allows the write location within the TX buffer to be 
	specified.  Future calls to UDPPut, UDPPutArray, UDPPutString, etc will
	write data from the indicated location.

  Precondition:
	UDPInit() must have been previously called and a socket is currently 
	active.

  Parameters:
    s       - UDP socket handle 
	wOffset - Offset from beginning of UDP packet data payload to place the
		write pointer.

  Returns:
  	true if the offset is a valid one
    false otherwise
  ***************************************************************************/
bool                UDPSetTxOffset(UDP_SOCKET s, uint16_t wOffset);

/*****************************************************************************
  Function:
	uint16_t UDPIsPutReady(UDP_SOCKET s)

  Summary:
	Determines how many bytes can be written to the UDP socket.
	
  Description:
	This function determines if bytes can be written to the specified UDP
	socket.  It also prepares the UDP module for writing by setting the 
	indicated socket as the currently active connection.

  Precondition:
	UDPInit() must have been previously called.

  Parameters:
	s - The socket to be made active

  Returns:
  	The number of bytes that can be written to this socket.
  ***************************************************************************/
uint16_t            UDPIsPutReady(UDP_SOCKET s);

/*****************************************************************************
  Function:
	uint16_t UDPIsTxPutReady(UDP_SOCKET s)

  Summary:
	Determines how many bytes can be written to the UDP socket.
	
  Description:
	This function determines if bytes can be written to the specified UDP
	socket.  It also prepares the UDP module for writing by setting the 
	indicated socket as the currently active connection.

  Precondition:
	UDPInit() must have been previously called.

  Parameters:
	s - The socket to be made active
    count - Number of bytes to allocate

  Returns:
  	The number of bytes that can be written to this socket.
  ***************************************************************************/
uint16_t            UDPIsTxPutReady(UDP_SOCKET s, unsigned short count);

/*****************************************************************************
  Function:
	uint16_t UDPPutArray(UDP_SOCKET s, const uint8_t *cData, uint16_t wDataLen)

  Summary:
	Writes an array of bytes to the currently active socket.
	
  Description:
	This function writes an array of bytes to the currently active UDP socket, 
	while incrementing the buffer length.  UDPIsPutReady should be used 
	before calling this function to specify the currently active socket.

  Precondition:
	UDPIsPutReady() was previously called to specify the current socket.

  Parameters:
	cData - The array to write to the socket.
	wDateLen - Number of bytes from cData to be written.
	
  Returns:
  	The number of bytes successfully placed in the UDP transmit buffer.  If
  	this value is less than wDataLen, then the buffer became full and the
  	input was truncated.
  ***************************************************************************/
uint16_t            UDPPutArray(UDP_SOCKET s, const uint8_t *cData, uint16_t wDataLen);

/*****************************************************************************
  Function:
	uint8_t* UDPPutString(UDP_SOCKET s, const uint8_t *strData)

  Summary:
	Writes null-terminated string to the currently active socket.
	
  Description:
	This function writes a null-terminated string to the currently active 
	UDP socket, while incrementing the buffer length.  UDPIsPutReady should 
	be used before calling this function to specify the currently active
	socket.

  Precondition:
	UDPIsPutReady() was previously called to specify the current socket.

  Parameters:
    s     - UDP socket handle
	cData - Pointer to the string to be written to the socket.
	
  Returns:
  	A pointer to the byte following the last byte written.  Note that this
  	is different than the UDPPutArray functions.  If this pointer does not
  	dereference to a NULL byte, then the buffer became full and the input
  	data was truncated.
  ***************************************************************************/
const uint8_t*      UDPPutString(UDP_SOCKET s, const uint8_t *strData);

/*****************************************************************************
  Function:
	uint16_t UDPGetTxCount(UDP_SOCKET s)

  Summary:
	Returns the amount of bytes written into the active UDP socket.
	
  Description:
	This function returns the amount of bytes written into the active UDP socket, 

  Precondition:
	UDPIsPutReady() was previously called to specify the current socket.

  Parameters:
	s   - UDP socket handle

  Return Values:
  	number of bytes in the socket
  ***************************************************************************/
uint16_t            UDPGetTxCount(UDP_SOCKET s);

/*****************************************************************************
  Function:
	uint16_t UDPFlush(UDP_SOCKET s)

  Summary:
	Transmits all pending data in a UDP socket.
	
  Description:
	This function builds a UDP packet with the pending TX data and marks it 
	for transmission over the network interface.  Since UDP is a frame-based
	protocol, this function must be called before returning to the main
	stack loop whenever any data is written.

  Precondition:
	UDPIsPutReady() was previously called to specify the current socket, and
	data has been written to the socket using the UDPPut family of functions.

  Parameters:
	None
	
  Returns:
  	The number of bytes that currently were in the socket TX buffer
    and have been flushed.

  Remarks:
	Note that unlike TCPFlush, UDPFlush must be called before returning to 
	the main stack loop.  There is no auto transmit for UDP segments.
  ***************************************************************************/
uint16_t            UDPFlush(UDP_SOCKET s);

/*****************************************************************************
  Function:
	bool UDPPut(UDP_SOCKET s, uint8_t v)

  Summary:
	Writes a byte to the currently active socket.
	
  Description:
	This function writes a single byte to the currently active UDP socket, 
	while incrementing the buffer length.  UDPIsPutReady should be used 
	before calling this function to specify the currently active socket.

  Precondition:
	UDPIsPutReady() was previously called to specify the current socket.

  Parameters:
    s   - UDP socket handle
	v - The byte to be loaded into the transmit buffer.

  Return Values:
  	true - The byte was successfully written to the socket.
  	false - The transmit buffer is already full and so the write failed.

  Remarks:
    The following function is very inefficient.
    A buffered approach (UDPPutArray) should be preferred
  ***************************************************************************/
bool                UDPPut(UDP_SOCKET s, uint8_t v);

/*****************************************************************************
  Function:
	uint16_t UDPIsGetReady(UDP_SOCKET s)

  Summary:
	Determines how many bytes can be read from the UDP socket.
	
  Description:
	This function determines if bytes can be read from the specified UDP
	socket.  It also prepares the UDP module for reading by setting the 
	indicated socket as the currently active connection.

  Precondition:
	UDPInit() must have been previously called.

  Parameters:
	s - The socket to be made active (which has already been opened or is
		listening)

  Returns:
  	The number of bytes that can be read from this socket.
  ***************************************************************************/
uint16_t            UDPIsGetReady(UDP_SOCKET s);

/*****************************************************************************
  Function:
	void UDPSetRxOffset(UDP_SOCKET s, uint16_t wOffset)

  Summary:
	Moves the pointer within the RX buffer.
	
  Description:
	This function allows the read location within the RX buffer to be 
	specified.  Future calls to UDPGet and UDPGetArray will read data from
	the indicated location forward.

  Precondition:
	UDPInit() must have been previously called and a socket is currently 
	active.

  Parameters:
    s       - UDP socket handle
	wOffset - Offset from beginning of UDP packet data payload to place the
		read pointer.

  Returns:
  	None
  ***************************************************************************/
void                UDPSetRxOffset(UDP_SOCKET s, uint16_t rOffset);

/*****************************************************************************
  Function:
	uint16_t UDPGetArray(UDP_SOCKET s, uint8_t *cData, uint16_t wDataLen)

  Summary:
	Reads an array of bytes from the currently active socket.
	
  Description:
	This function reads an array of bytes from the currently active UDP socket, 
	while decrementing the remaining bytes available. UDPIsGetReady should be 
	used before calling this function to specify the currently active socket.

  Precondition:
	UDPIsGetReady() was previously called to specify the current socket.

  Parameters:
    s     - UDP socket handle
	cData - The buffer to receive the bytes being read.  If NULL, the bytes are 
			simply discarded without being written anywhere (effectively skips 
			over the bytes in the RX buffer, although if you need to skip a lot 
			of data, seeking using the UDPSetRxOffset() will be more efficient).
	wDateLen - Number of bytes to be read from the socket.
	
  Returns:
  	The number of bytes successfully read from the UDP buffer.  If this
  	value is less than wDataLen, then the buffer was emptied and no more 
  	data is available.
  ***************************************************************************/
uint16_t            UDPGetArray(UDP_SOCKET s, uint8_t *cData, uint16_t wDataLen);

/*****************************************************************************
  Function:
	void UDPDiscard(UDP_SOCKET s)

  Summary:
	Discards any remaining RX data from a UDP socket.
	
  Description:
	This function discards any remaining received data in the currently 
	active UDP socket.

  Precondition:
	UDPIsGetReady() was previously called to select the currently active
	socket.

  Parameters:
    s   - socket handle
	
  Returns:
  	None

  Remarks:
	It is safe to call this function more than is necessary.  If no data is
	available, this function does nothing.
  ***************************************************************************/
void                UDPDiscard(UDP_SOCKET s);

/*****************************************************************************
  Function:
	bool UDPGet(UDP_SOCKET s, uint8_t *v)

  Summary:
	Reads a byte from the currently active socket.
	
  Description:
	This function reads a single byte from the currently active UDP socket, 
	while decrementing the remaining buffer length.  UDPIsGetReady should be 
	used before calling this function to specify the currently active socket.

  Precondition:
	UDPIsGetReady() was previously called to specify the current socket.

  Parameters:
    s   - socket handle
	v - The buffer to receive the data being read.

  Return Values:
  	true - A byte was successfully read
  	false - No data remained in the read buffer

  Remarks:
    The following function is very inefficient.
    A buffered approach (UDPGetArray) should be preferred
 ***************************************************************************/
bool                UDPGet(UDP_SOCKET s, uint8_t *v);

/*****************************************************************************
  Function:
	void UDPSocketSetNet(UDP_SOCKET s, TCPIP_NET_HANDLE hNet)

  Summary:
	Sets the network interface for an UDP socket
	
  Description:
	This function sets the network interface for an UDP socket

  Precondition:
	UDP socket should have been opened with UDPOpenServer()/UDPOpenClient()().
    s - valid socket

  Parameters:
	s - The UDP socket
   	pNet - interface .
	
  Returns:
    None.
  ***************************************************************************/
void                UDPSocketSetNet(UDP_SOCKET s, TCPIP_NET_HANDLE hNet);

/*****************************************************************************
  Function:
	TCPIP_MAC_HANDLE UDPSocketGetNet(UDP_SOCKET s)

  Summary:
	Gets the MAC interface of an UDP socket
	
  Description:
	This function returns the MAC interface id of an UDP socket

  Precondition:
	UDP socket should have been opened with UDPOpenServer()/UDPOpenClient()().
    s - valid socket

  Parameters:
	s - The UDP socket
	
  Returns:
    IHandle of the interface that socket currently uses.
  ***************************************************************************/
TCPIP_NET_HANDLE    UDPSocketGetNet(UDP_SOCKET s);

/*****************************************************************************
  Function:
	bool UDPSetBcastIPAddress(UDP_SOCKET s, IP_ADDRESS_TYPE addType, IP_MULTI_ADDRESS* bcastAddress)

  Summary:
	Sets the broadcast IP address of a socket
	Allows an UDP socket to send broadcasts.
	
  Description:
      - for now it checks for broadcast address
        and properly sets the MAC address to 
        ff:ff:ff:ff:ff:ff is this is a valid broadcast
        (this should be decided by the IP layer)
      - To be consistent with the current stack, if
        the destination is not a broadcast, an ARP 
        should be initiated.
        IT iS NOT!
        Use just for broadcast, for now!

  Precondition:
	UDP initialized

  Parameters:
	s				-	the UDP socket
	addType			-	Type of address
	bcastAddress	- 	bcast pointer to store value
	
  Returns:
    True if success
	False otherwisw
  ***************************************************************************/
bool                UDPSetBcastIPAddress(UDP_SOCKET s, IP_ADDRESS_TYPE addType, IP_MULTI_ADDRESS* bcastAddress);


/*  Obsolete UDP calls that have been remove form the API.
 *  No longer supported!
*/

// UDPOpen() is obsolete and no longer supported.
// use UDPOpenServer()/UDPOpenClient() instead
// UDP_SOCKET          UDPOpen(uint32_t remoteHost, uint8_t remoteHostType, UDP_PORT localPort,UDP_PORT remotePort);
  
// UDPSetTxBuffer() is obsolete and no longer supported.
// use UDPSetTxOffset() instead
//

// UDPSetTxCount() is obsolete and no longer supported.
// use UDPSetTxOffset() instead
//

// UDPSetRxBuffer() is obsolete and no longer supported.
// use UDPSetRxOffset() instead
//

#endif  // __UDP_H_


