/*******************************************************************************
  Berekely TCP lient demo application.
  This application uses the BSD socket APIs and start a client

  Summary:
    
  Description:
*******************************************************************************/

/*******************************************************************************
FileName:   BerkeleyTCPClientDemo.c
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

#include "tcpip_config.h" 

#if defined(TCPIP_STACK_USE_BERKELEY_API)

#include "tcpip/tcpip.h"

#include "system/errno.h"

#define PORTNUM 80
static const uint8_t ServerName[] =  "www.google.com";
// This is specific to this HTTP Client example
static uint8_t sendRequest[] = "GET /search?as_q=Microchip&as_sitesearch=microchip.com HTTP/1.0\r\nHost: www.google.com\r\nConnection: close\r\n\r\n";

/*********************************************************************
 * Function:        void BerkeleyTCPClientDemo(void)
 *
 * PreCondition:    Stack is initialized()
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
void BerkeleyTCPClientDemo(void)
{
	#if defined(TCPIP_STACK_USE_DNS)
    static SOCKET bsdClientSocket;
    static struct sockaddr_in addr;
    char recvBuffer[9];
    int tcpSkt, i;
    int addrlen;
    DNS_RESULT  dnsRes;
   
    static enum
    {
	    DNS_START_RESOLUTION = 0,
	    DNS_GET_RESULT,
        BSD_START,
        BSD_CONNECT,
        BSD_SEND,
        BSD_OPERATION,
        BSD_CLOSE,
        BSD_DONE
    } BSDClientState = BSD_DONE;

    switch(BSDClientState)
    {
	    case DNS_START_RESOLUTION:
            // use the default DNS interface
            if(DNSBeginUsage(0) == DNS_RES_OK)
	    	{
		    	DNSResolve((const char*)ServerName, DNS_TYPE_A);
		    	BSDClientState = DNS_GET_RESULT;
		    }
	    	break;
	    
	    case DNS_GET_RESULT:
	    	dnsRes = DNSIsResolved((const char*)ServerName, (IP_ADDR*)&addr.sin_addr.S_un.S_addr);
            if(dnsRes == DNS_RES_PENDING)
            {   // still waiting
	    		break;
            }

            // release the DNS module
	    	DNSEndUsage(0);
            
            if(dnsRes < 0)
            {   // some error
                SYS_ERROR(SYS_ERROR_ERROR, "Could not resolve ServerName[] to IP address.\r\n");
                BSDClientState = BSD_DONE;
                break;
            }
            
            // DNS_RES_OK
	    	BSDClientState = BSD_START;			
	    	// No break; here.
	    
        case BSD_START:
            // Create a socket for this client to connect with 
            if((tcpSkt = socket(AF_INET, SOCK_STREAM, IPPROTO_TCP)) == SOCKET_ERROR )
            {
                return;
            }
            else
            {
                bsdClientSocket = (SOCKET)tcpSkt;
            } 
         
			SYS_ERROR(SYS_ERROR_INFO, "\r\n\r\nConnecting using Berkeley Sockets TCP API...\r\n");
			SYS_ERROR(SYS_ERROR_INFO, "   Note: this demo will do nothing if an underlying TCP_PURPOSE_BERKELEY_CLIENT type \r\n"
								 "   socket is unavailable, as declared by the TCPSocketInitializer[] array in \r\n"
								 "   tcpip_config.h.\r\n\r\n");
         	         
            BSDClientState = BSD_CONNECT;
            break;

        case BSD_CONNECT:
            // addr.sin_addr.S_un.S_addr destination IP address was set earlier in DNS step
            addr.sin_port = PORTNUM;
            addrlen = sizeof(struct sockaddr);
            if(connect( bsdClientSocket, (struct sockaddr*)&addr, addrlen) < 0)
            	return;

            BSDClientState = BSD_SEND;
            // No break needed

        case BSD_SEND:
            //send TCP data
            send(bsdClientSocket, (const char*)sendRequest, strlen((char*)sendRequest), 0);  
            BSDClientState = BSD_OPERATION;
            break;
        
        case BSD_OPERATION:
            // Obtian and print the server reply
            while(1)
            {
				i = recv(bsdClientSocket, recvBuffer, sizeof(recvBuffer)-1, 0); //get the data from the recv queue

                if(i <= 0) // close or error condition
                {
                    if(i == 0 || errno != EWOULDBLOCK)
                    {
                        BSDClientState = BSD_CLOSE; // true error/close condition 
                    }
                    // else just wait for some more data
                    
                    break;
                }
                
                recvBuffer[i] = '\0';	// Null terminate data
                SYS_ERROR(SYS_ERROR_INFO, recvBuffer);

                if(BSDClientState == BSD_OPERATION)
                    break;
            }
            break;
         
        case BSD_CLOSE:
            closesocket(bsdClientSocket);
            BSDClientState = BSD_DONE;
            // No break needed
            
        case BSD_DONE:
            if(BUTTON2_IO == 0u)
            {
            	BSDClientState = DNS_START_RESOLUTION;
            }
            break;
         
        default:
            return;
    }


	//#if defined(TCPIP_STACK_USE_DNS)
	#else
		#warning You must define TCPIP_STACK_USE_DNS for BerkeleyTCPClientDemo to work
	#endif
}

#endif //#if defined(TCPIP_STACK_USE_BERKELEY_API)

