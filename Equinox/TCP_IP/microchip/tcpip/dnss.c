/*******************************************************************************
  Domain Name System (DNS) Server dummy

  Summary:
    Module for Microchip TCP/IP Stack
    
  Description:
    - Acts as a DNS server, but gives out the local IP address for all 
      queries to force web browsers to access the board.
    - Reference: RFC 1034 and RFC 1035
*******************************************************************************/

/*******************************************************************************
FileName:   DNSs.c
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
#define DNSS_C

#include "tcpip_private.h"
#include "tcpip_config_private.h"

#if defined(TCPIP_STACK_USE_DNS_SERVER)

static void DNSCopyRXNameToTX(UDP_SOCKET s, NET_CONFIG* pNet);

typedef enum
{
    DNSS_STATE_START             = 0,
    DNSS_STATE_WAIT_REQUEST,
    DNSS_STATE_PUT_REQUEST,
    DNSS_STATE_DONE,
}DNSS_STATE;


static DNSS_STATE   dnssState = DNSS_STATE_START;

/*********************************************************************
 * Function:        void DNSServerTask(NET_CONFIG* pNet)
 *
 * PreCondition:    None
 *
 * Input:           None
 *
 * Output:          None
 *
 * Side Effects:    None
 *
 * Overview:        Sends dummy responses that point to ourself for DNS requests
 *
 * Note:            None
 ********************************************************************/
void DNSServerTask(NET_CONFIG* pNet)
{
	static UDP_SOCKET	MySocket = INVALID_UDP_SOCKET;
	static struct
	{
		uint16_t wTransactionID;
		uint16_t wFlags;
		uint16_t wQuestions;
		uint16_t wAnswerRRs;
		uint16_t wAuthorityRRs;
		uint16_t wAdditionalRRs;
	} DNSHeader;

    switch(dnssState)
    {
        case DNSS_STATE_START:

            // Create a socket to listen on if this is the first time calling this function
            if(MySocket == INVALID_UDP_SOCKET)
            {
                MySocket = UDPOpen(0,UDP_OPEN_SERVER,DNS_PORT,0);
                if(MySocket == INVALID_UDP_SOCKET)
                    break;
            }
            
            UDPSocketSetNet(MySocket, pNet);
            dnssState = DNSS_STATE_WAIT_REQUEST;
            break;

        case DNSS_STATE_WAIT_REQUEST:

            // See if a DNS query packet has arrived
            if(UDPIsGetReady(MySocket) < sizeof(DNSHeader))
                break;

            // Read DNS header
            UDPGetArray(MySocket, (uint8_t*)&DNSHeader, sizeof(DNSHeader));

            // Ignore this packet if it isn't a query
            if((DNSHeader.wFlags & 0x8000) == 0x8000u)
                break;

            // Ignore this packet if there are no questions in it
            if(DNSHeader.wQuestions == 0u)
                break;

            dnssState = DNSS_STATE_PUT_REQUEST;
            break;

        case DNSS_STATE_PUT_REQUEST:

            // check that we can transmit a DNS response packet
            if(!UDPIsPutReady(MySocket))
            {
                break;
            }

            // Write DNS response packet
            UDPPutArray(MySocket, (uint8_t*)&DNSHeader.wTransactionID, 2);	// 2 byte Transaction ID
            if(DNSHeader.wFlags & 0x0100)
                UDPPut(MySocket, 0x81);	// Message is a response with recursion desired
            else
                UDPPut(MySocket, 0x80);	// Message is a response without recursion desired flag set
            
            UDPPut(MySocket, 0x80);	// Recursion available
            UDPPut(MySocket, 0x00);	// 0x0000 Questions
            UDPPut(MySocket, 0x00);
            UDPPut(MySocket, 0x00);	// 0x0001 Answers RRs
            UDPPut(MySocket, 0x01);
            UDPPut(MySocket, 0x00);	// 0x0000 Authority RRs
            UDPPut(MySocket, 0x00);
            UDPPut(MySocket, 0x00);	// 0x0000 Additional RRs
            UDPPut(MySocket, 0x00);
            DNSCopyRXNameToTX(MySocket, pNet);	// Copy hostname of first question over to TX packet
            UDPPut(MySocket, 0x00);	// Type A Host address
            UDPPut(MySocket, 0x01);
            UDPPut(MySocket, 0x00);	// Class INternet
            UDPPut(MySocket, 0x01);
            UDPPut(MySocket, 0x00);	// Time to Live 10 seconds
            UDPPut(MySocket, 0x00);
            UDPPut(MySocket, 0x00);
            UDPPut(MySocket, 0x0A);
            UDPPut(MySocket, 0x00);	// Data Length 4 bytes
            UDPPut(MySocket, 0x04);
            UDPPutArray(MySocket, (uint8_t*)&pNet->MyIPAddr.Val, 4);	// Our IP address

            UDPFlush(MySocket);

            dnssState = DNSS_STATE_DONE;
            break;
            
         case DNSS_STATE_DONE:
            break;
    }
            
           
}



/*****************************************************************************
  Function:
	static void DNSCopyRXNameToTX(UDP_SOCKET s, NET_CONFIG* pNet)

  Summary:
	Copies a DNS hostname, possibly including name compression, from the RX 
	packet to the TX packet (without name compression in TX case).
	
  Description:
	None

  Precondition:
	RX pointer is set to currently point to the DNS name to copy

  Parameters:
	None

  Returns:
  	None
  ***************************************************************************/
static void DNSCopyRXNameToTX(UDP_SOCKET s, NET_CONFIG* pNet)
{
	uint16_t w;
	uint8_t i;
	uint8_t len;

	while(1)
	{
		// Get first byte which will tell us if this is a 16-bit pointer or the 
		// length of the first of a series of labels
		if(!UDPGet(s, &i))
			return;
		
		// Check if this is a pointer, if so, get the reminaing 8 bits and seek to the pointer value
		if((i & 0xC0u) == 0xC0u)
		{
			((uint8_t*)&w)[1] = i & 0x3F;
			UDPGet(s, (uint8_t*)&w);
			IPSetRxBuffer(pNet, sizeof(UDP_HEADER) + w);
			continue;
		}

		// Write the length byte
		len = i;
		UDPPut(s, len);
		
		// Exit if we've reached a zero length label
		if(len == 0u)
			return;
		
		// Copy all of the bytes in this label	
		while(len--)
		{
			UDPGet(s, &i);
			UDPPut(s, i);
		}
	}
}

#endif //#if defined(TCPIP_STACK_USE_DNS_SERVER)
