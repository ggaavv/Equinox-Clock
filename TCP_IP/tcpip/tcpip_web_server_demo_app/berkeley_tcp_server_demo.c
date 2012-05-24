/*******************************************************************************
  Berekely TCP server demo application.

  Summary:
    
  Description:
    This application uses the BSD socket APIs and starts a server 
    listening on TCP port 9764.  All data sent to a connection on 
    this port will be echoed back to the sender.  By default, this 
    demo supports 3 simultaneous connections.
*******************************************************************************/

/*******************************************************************************
FileName:  BerkeleyTCPServerDemo.c 
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

#include "tcpip_config.h" 

#if defined(TCPIP_STACK_USE_BERKELEY_API)

#include "tcpip/tcpip.h"

#include <system/errno.h>

#define PORTNUM 9764
#define MAX_CLIENT (3) // Maximum number of simultanous connections accepted by the server.


/*********************************************************************
 * Function:        void BerkeleyTCPServerDemo(void)
 *
 * PreCondition:    Stack is initialized
 *
 * Input:           None
 *
 * Output:          None
 *
 * Side Effects:    None
 *
 * Overview:        None
 *
 * Note:            None
 ********************************************************************/
void BerkeleyTCPServerDemo(void)
{
    static SOCKET bsdServerSocket;   
    static SOCKET ClientSock[MAX_CLIENT];
    struct sockaddr_in addr;
    struct sockaddr_in addRemote;
    int addrlen = sizeof(struct sockaddr_in);
    char bfr[15];
    int length;
    int tcpSkt, i;
    static enum
    {
	    BSD_INIT = 0,
        BSD_CREATE_SOCKET,
        BSD_BIND,
        BSD_LISTEN,
        BSD_OPERATION
    } BSDServerState = BSD_INIT;

    switch(BSDServerState)
    {
	    case BSD_INIT:
        	// Initialize all client socket handles so that we don't process 
        	// them in the BSD_OPERATION state
        	for(i = 0; i < MAX_CLIENT; i++)
        		ClientSock[i] = INVALID_SOCKET;
        		
        	BSDServerState = BSD_CREATE_SOCKET;
        	// No break needed
	    
        case BSD_CREATE_SOCKET:
            // Create a socket for this server to listen and accept connections on
            tcpSkt = socket(AF_INET, SOCK_STREAM, IPPROTO_TCP);
            if(tcpSkt == INVALID_SOCKET)
                return;
            bsdServerSocket = (SOCKET)tcpSkt;
            
            BSDServerState = BSD_BIND;
            // No break needed

        case BSD_BIND:
            // Bind socket to a local port
            addr.sin_port = PORTNUM;
            addr.sin_addr.S_un.S_addr = IP_ADDR_ANY;
            if( bind( bsdServerSocket, (struct sockaddr*)&addr, addrlen ) == SOCKET_ERROR )
                return;
            
            BSDServerState = BSD_LISTEN;
            // No break needed
         
      case BSD_LISTEN:
            if(listen(bsdServerSocket, MAX_CLIENT) == 0)
	            BSDServerState = BSD_OPERATION;

			// No break.  If listen() returns SOCKET_ERROR it could be because 
			// MAX_CLIENT is set to too large of a backlog than can be handled 
			// by the underlying TCP socket count (TCP_PURPOSE_BERKELEY_SERVER 
			// type sockets in tcpip_config.h).  However, in this case, it is 
			// possible that some of the backlog is still handleable, in which 
			// case we should try to accept() connections anyway and proceed 
			// with normal operation.
         
      case BSD_OPERATION:
            for(i=0; i<MAX_CLIENT; i++)
            {
	            // Accept any pending connection requests, assuming we have a place to store the socket descriptor
                if(ClientSock[i] == INVALID_SOCKET)
                    ClientSock[i] = accept(bsdServerSocket, (struct sockaddr*)&addRemote, &addrlen);
                
                // If this socket is not connected then no need to process anything
                if(ClientSock[i] == INVALID_SOCKET)
                	continue;

	            // For all connected sockets, receive and send back the data
                length = recv( ClientSock[i], bfr, sizeof(bfr), 0);
         
                if( length > 0 )
                {
                    bfr[length] = '\0';
                    send(ClientSock[i], bfr, strlen(bfr), 0);
                }
                else if( length == 0 || errno != EWOULDBLOCK )
                {
                    closesocket( ClientSock[i] );
                    ClientSock[i] = INVALID_SOCKET;
                }
                // else just wait for some more data
            }
            break;
         
        default:
            return;
    }
    return;
}

#endif //#if defined(TCPIP_STACK_USE_BERKELEY_API)

