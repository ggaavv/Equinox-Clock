/*******************************************************************************
  Simple Network Time Protocol (SNTP) Client Version 3

  Summary:
    Module for Microchip TCP/IP Stack - implemented with Berkeley API
    
  Description:
    -Locates an NTP Server from public site using DNS
    -Requests UTC time using SNTP and updates SNTPTime structure
     periodically, according to NTP_QUERY_INTERVAL value
    -Reference: RFC 1305
*******************************************************************************/

/*******************************************************************************
FileName:  BerkeleyUDPClientDemo.c 
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

#define __SNTP_C

#include "tcpip_config.h"

#if defined(TCPIP_STACK_USE_BERKELEY_API)

#include "tcpip/tcpip.h"


// Defines the structure of an NTP packet
typedef struct
{
	struct
	{
		uint8_t mode			: 3;	// NTP mode
		uint8_t versionNumber 	: 3;	// SNTP version number
		uint8_t leapIndicator	: 2;	// Leap second indicator
	} flags;						// Flags for the packet

	uint8_t stratum;					// Stratum level of local clock
	int8_t poll;						// Poll interval
	int8_t precision;					// Precision (seconds to nearest power of 2)
	uint32_t root_delay;				// Root delay between local machine and server
	uint32_t root_dispersion;			// Root dispersion (maximum error)
	uint32_t ref_identifier;			// Reference clock identifier
	uint32_t ref_ts_secs;				// Reference timestamp (in seconds)
	uint32_t ref_ts_fraq;				// Reference timestamp (fractions)
	uint32_t orig_ts_secs;				// Origination timestamp (in seconds)
	uint32_t orig_ts_fraq;				// Origination timestamp (fractions)
	uint32_t recv_ts_secs;				// Time at which request arrived at sender (seconds)
	uint32_t recv_ts_fraq;				// Time at which request arrived at sender (fractions)
	uint32_t tx_ts_secs;				// Time at which request left sender (seconds)
	uint32_t tx_ts_fraq;				// Time at which request left sender (fractions)
} NTP_PACKET;

// Seconds value obtained by last update
static uint32_t dwSNTPSeconds = 0;

// Tick count of last update
static SYS_TICK dwLastUpdateTick = 0;


/*****************************************************************************
  Function:
	void BerkeleyUDPClientDemo(uint32_t localIP)

  Summary:
	Periodically checks the current time from a pool of servers.

  Description:
	This function periodically checks a pool of time servers to obtain the
	current date/time.

  Precondition:
	UDP is initialized.

  Parameters:
	localIP   - the local interface to use

  Returns:
  	None
  	
  Remarks:
	This function requires once available UDP socket while processing, but
	frees that socket when the SNTP module is idle.
  ***************************************************************************/
void BerkeleyUDPClientDemo(uint32_t localIP)
{
	#if defined(TCPIP_STACK_USE_DNS)
	NTP_PACKET			pkt;
	int			 		i;
	static uint32_t		dwServerIP;
	static SYS_TICK		udpTick;
    static SOCKET		bsdUdpClient;
    int                 udpSkt;
    int 				addrlen = sizeof(struct sockaddr_in);
    static struct sockaddr_in	udpaddr;
    DNS_RESULT  dnsRes;
    
	static enum
	{
		SM_HOME = 0,
		SM_NAME_RESOLVE,
		SM_CREATE_SOCKET,
		SM_BIND,	
		SM_UDP_SEND,
		SM_UDP_RECV,
		SM_SHORT_WAIT,
		SM_WAIT
	} SNTPState = SM_HOME;

	switch(SNTPState)
	{
		case SM_HOME:
			// Obtain ownership of the DNS resolution module
            if(DNSBeginUsage(0) != DNS_RES_OK)
            {
				break;
            }

			// Obtain the IP address associated with the server name
			DNSResolve(NTP_SERVER, DNS_TYPE_A);
			udpTick = SYS_TICK_Get();
			SNTPState = SM_NAME_RESOLVE;
			break;

		case SM_NAME_RESOLVE:
			// Wait for DNS resolution to complete
			dnsRes = DNSIsResolved(NTP_SERVER, (IP_ADDR*)&dwServerIP);
            if(dnsRes == DNS_RES_PENDING)
            {   // still waiting
                break;
            }
            
            // release the DNS module
	    	DNSEndUsage(0);

            if(dnsRes < 0)
            {   // some error
                udpTick = SYS_TICK_Get();
                SNTPState = SM_SHORT_WAIT;
                break;
            }
			
            // DNS_RES_OK
			SNTPState = SM_CREATE_SOCKET;
			// No break needed

		case SM_CREATE_SOCKET:
			udpSkt = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
			if(udpSkt == SOCKET_ERROR)
            {
				return;
            }
            else
            {
                bsdUdpClient = (SOCKET)udpSkt;
            }

// Usually no explicit binding is necessary since we are going to be the first one 
// sending a UDP packet.  sendto() will do implicit binding.  Explicit binding 
// is necessary only when creating UDP servers that will be receiving the first 
// packet.
// However, when using this example with multi-homed hosts we want to use
// a specified network connection specified by the localIP parameter
			SNTPState = SM_BIND;
			// No break needed
			
		case SM_BIND:
            udpaddr.sin_port = 0;
            udpaddr.sin_addr.S_un.S_addr = localIP;
            if( bind(bsdUdpClient, (struct sockaddr*)&udpaddr, addrlen) == SOCKET_ERROR )
                break;

            SNTPState = SM_UDP_SEND;
            // No break needed
            
		case SM_UDP_SEND:
            // Transmit a time request packet
			memset(&pkt, 0, sizeof(pkt));
			pkt.flags.versionNumber = 3;	// NTP Version 3
			pkt.flags.mode = 3;				// NTP Client
			pkt.orig_ts_secs = swapl(NTP_EPOCH);
            udpaddr.sin_port = NTP_SERVER_PORT;
            udpaddr.sin_addr.S_un.S_addr = dwServerIP;
			if(sendto(bsdUdpClient, (const char*)&pkt, sizeof(pkt), 0, (struct sockaddr*)&udpaddr, addrlen)>0)
            {
				udpTick = SYS_TICK_Get();
                SNTPState = SM_UDP_RECV;
            }
			break;

		case SM_UDP_RECV:
			// Look for a response time packet
			i = recvfrom(bsdUdpClient, (char*)&pkt, sizeof(pkt), 0, (struct sockaddr*)&udpaddr, &addrlen);
            if(i < (int)sizeof(pkt))
			{
				if((SYS_TICK_Get()) - udpTick > NTP_REPLY_TIMEOUT * SYS_TICK_TicksPerSecondGet())
				{
					// Abort the request and wait until the next timeout period
					closesocket(bsdUdpClient);
					udpTick = SYS_TICK_Get();
					SNTPState = SM_SHORT_WAIT;
					break;
				}
				break;
			}
			closesocket(bsdUdpClient);
			udpTick = SYS_TICK_Get();
			SNTPState = SM_WAIT;

			// Set out local time to match the returned time
			dwLastUpdateTick = SYS_TICK_Get();
			dwSNTPSeconds = swapl(pkt.tx_ts_secs) - NTP_EPOCH;
			// Do rounding.  If the partial seconds is > 0.5 then add 1 to the seconds count.
			if(((uint8_t*)&pkt.tx_ts_fraq)[0] & 0x80)
				dwSNTPSeconds++;

			break;

		case SM_SHORT_WAIT:
			// Attempt to requery the NTP server after a specified NTP_FAST_QUERY_INTERVAL time (ex: 8 seconds) has elapsed.
			if(SYS_TICK_Get() - udpTick > (NTP_FAST_QUERY_INTERVAL * SYS_TICK_TicksPerSecondGet() ))
				SNTPState = SM_HOME;	
			break;

		case SM_WAIT:
			// Requery the NTP server after a specified NTP_QUERY_INTERVAL time (ex: 10 minutes) has elapsed.
			if(SYS_TICK_Get() - udpTick > (NTP_QUERY_INTERVAL * SYS_TICK_TicksPerSecondGet()))
				SNTPState = SM_HOME;	

			break;
	}

	//#if defined(TCPIP_STACK_USE_DNS)
	#else
		#warning You must define TCPIP_STACK_USE_DNS for BerkeleyUDPClientDemo to work
	#endif
}


/*****************************************************************************
  Function:
	uint32_t BerkeleySNTPGetUTCSeconds(void)

  Summary:
	Obtains the current time from the SNTP module.

  Description:
	This function obtains the current time as reported by the SNTP module.  
	Use this value for absolute time stamping.  The value returned is (by
	default) the number of seconds since 01-Jan-1970 00:00:00.

  Precondition:
	None

  Parameters:
	None

  Returns:
  	The number of seconds since the Epoch.  (Default 01-Jan-1970 00:00:00)
  	
  Remarks:
	Do not use this function for time difference measurements.  The Tick
	module is more appropriate for those requirements.
  ***************************************************************************/
uint32_t BerkeleySNTPGetUTCSeconds(void)
{
	SYS_TICK dwTickDelta;
	SYS_TICK dwTick;

	// Update the dwSNTPSeconds variable with the number of seconds 
	// that has elapsed
	dwTick = SYS_TICK_Get();
	dwTickDelta = dwTick - dwLastUpdateTick;
	while(dwTickDelta > SYS_TICK_TicksPerSecondGet())
	{
		dwSNTPSeconds++;
		dwTickDelta -= SYS_TICK_TicksPerSecondGet();
	}
	
	// Save the tick and residual fractional seconds for the next call
	dwLastUpdateTick = dwTick - dwTickDelta;

	return dwSNTPSeconds;
}

#endif  //if defined(TCPIP_STACK_USE_BERKELEY_API)

