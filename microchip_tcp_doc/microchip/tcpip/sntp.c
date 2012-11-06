/*******************************************************************************
  Simple Network Time Protocol (SNTP) Client Version 3

  Summary:
    Module for Microchip TCP/IP Stack
    
  Description:
    -Locates an NTP Server from public site using DNS
    -Requests UTC time using SNTP and updates SNTPTime structure
     periodically, according to NTP_QUERY_INTERVAL value
    -Reference: RFC 1305
*******************************************************************************/

/*******************************************************************************
FileName:   SNTP.c
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

#include "tcpip_private.h"
#include "tcpip_config_private.h"

#if defined(TCPIP_STACK_USE_SNTP_CLIENT)

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

static NET_CONFIG*  pSntpIf = 0;       // we use only one interface for SNTP (for now at least)

static UDP_SOCKET   sntpSocket = INVALID_UDP_SOCKET;    // UDP socket we use
static enum
{
    SM_HOME = 0,
    SM_WAIT_DNS,
    SM_DNS_RESOLVED,
    SM_UDP_IS_OPENED,
    SM_UDP_SEND,
    SM_UDP_RECV,
    SM_SHORT_WAIT,
    SM_WAIT
} sntpState = SM_HOME;

// the server address
static IPV4_ADDR  serverIP;

static void  _SntpInit(NET_CONFIG* pNewIf)
{
    if(pSntpIf != 0)
    {
        if(pNewIf == pSntpIf)
        {   // this interface is going away/re-initialized, etc
            if(sntpSocket != INVALID_UDP_SOCKET)
            {
                UDPClose(sntpSocket);
                sntpSocket = INVALID_UDP_SOCKET;
            }
            pSntpIf = 0;
            sntpState = SM_HOME;
        }
    }


}

/*****************************************************************************
  Function:
    bool SNTPInit(const TCPIP_STACK_MODULE_CTRL* const stackCtrl, const SNTP_MODULE_CONFIG* pSNTPConfig);

  Summary:
	Resets the SNTP client module.

  Description:
	Initialization of the SNTP module

  Precondition:
	None

  Parameters:
	stackCtrl - pointer to stack structure specifying the interface to initialize

  Returns:
	None

  Remarks:
	None 
***************************************************************************/
bool SNTPInit(const TCPIP_STACK_MODULE_CTRL* const stackCtrl, const SNTP_MODULE_CONFIG* pSNTPConfig)
{
	
    if(stackCtrl->stackAction == TCPIP_STACK_ACTION_IF_UP)
    {   // interface restart
        return true;
    }

    // stack init
    _SntpInit(stackCtrl->pNetIf);

    sntpState = SM_HOME;
    return true;
}

/*****************************************************************************
  Function:
    bool SNTPDeInit(const TCPIP_STACK_MODULE_CTRL* const stackInit);

  Summary:
	Turns off the SNTP module for the specified interface.

  Description:
	Deinitialization of the SNTP module.

  Precondition:
	None

  Parameters:
	stackData - pointer to stack structure specifying the interface to deinitialize

  Returns:
	None

  Remarks:
***************************************************************************/
void SNTPDeInit(const TCPIP_STACK_MODULE_CTRL* const stackCtrl)
{

    // if(stackCtrl->stackAction == TCPIP_STACK_ACTION_IF_DOWN)
    // if(stackCtrl->stackAction == TCPIP_STACK_ACTION_DEINIT) // stack shut down
    
    // interface is going down one way or another
    _SntpInit(stackCtrl->pNetIf);
}


/*****************************************************************************
  Function:
	void SNTPClient(NET_CONFIG* pConfig)

  Summary:
	Periodically checks the current time from a pool of servers.

  Description:
	This function periodically checks a pool of time servers to obtain the
	current date/time.

  Precondition:
	UDP is initialized.

  Parameters:
	pConfig   - interface 

  Returns:
  	None
  	
  Remarks:
	This function requires once available UDP socket while processing, but
	frees that socket when the SNTP module is idle.
  ***************************************************************************/
void SNTPClient(NET_CONFIG* pConfig)
{
	NTP_PACKET			pkt;
	uint16_t		    w;
    DNS_RESULT          dnsRes;
	static SYS_TICK		SNTPTimer;
    static TCPIP_NET_HANDLE    netH;

    if(pSntpIf != 0 && pConfig != pSntpIf)
    {   // not our job
        return;
    }
    
	switch(sntpState)
	{
		case SM_HOME:
			sntpSocket = INVALID_UDP_SOCKET;
            netH = TCPIP_STACK_GetDefaultNet();
            pSntpIf = (NET_CONFIG*)TCPIP_STACK_NetHandle(MY_DEFAULT_SNTP_IF);
            if(!TCPIP_STACK_IsNetUp(pSntpIf))
            {
                pSntpIf = (NET_CONFIG*)netH;
            }
            
            if(DNSBeginUsage(netH) != DNS_RES_OK)
            {
                break;
            }
            DNSResolve(NTP_SERVER, DNS_TYPE_A);
			sntpState++;
			break;

        case SM_WAIT_DNS:
            
            dnsRes = DNSIsResolved(NTP_SERVER, &serverIP);
            if(dnsRes == DNS_RES_PENDING)
            {   // ongoing operation;
                break;
            }
            else if(dnsRes < 0)
            {   // some DNS error occurred; retry
				sntpState = SM_HOME;
            }
            else
            {
                sntpState++;
            }
            DNSEndUsage(netH);
            break;

		case SM_DNS_RESOLVED:

            sntpSocket = UDPOpenClient(IP_ADDRESS_TYPE_IPV4, NTP_SERVER_PORT, (IP_MULTI_ADDRESS*)&serverIP);
            if(sntpSocket != INVALID_UDP_SOCKET)
            {
                UDPSocketSetNet(sntpSocket, pSntpIf);
                sntpState++;
                SNTPTimer = SYS_TICK_Get();
            }
            break;
			
		case SM_UDP_IS_OPENED:
			if(UDPIsOpened(sntpSocket) == true)
			{
                SNTPTimer = SYS_TICK_Get();
				sntpState = SM_UDP_SEND;
			}
            else if((SYS_TICK_Get() - SNTPTimer > 1*SYS_TICK_TicksPerSecondGet())) 
			{   // failed to open
				UDPClose(sntpSocket);
				sntpState = SM_HOME;
				sntpSocket = INVALID_UDP_SOCKET;
			}
			break;

		case SM_UDP_SEND:
			// Open up the sending UDP socket
			// Make certain the socket can be written to
			if(!UDPIsTxPutReady(sntpSocket, sizeof(pkt)))
            {   // Wait no more than 1 sec
                if((SYS_TICK_Get() - SNTPTimer > 1*SYS_TICK_TicksPerSecondGet()))
                {
                    UDPClose(sntpSocket);
                    sntpState = SM_HOME;
                    sntpSocket = INVALID_UDP_SOCKET;
                    break;
                }
            }

            // Success
			// Transmit a time request packet
			memset(&pkt, 0, sizeof(pkt));
			pkt.flags.versionNumber = 3;	// NTP Version 3
			pkt.flags.mode = 3;				// NTP Client
			pkt.orig_ts_secs = swapl(NTP_EPOCH);
			UDPPutArray(sntpSocket, (uint8_t*) &pkt, sizeof(pkt));	
			UDPFlush(sntpSocket);	
			
			SNTPTimer = SYS_TICK_Get();
			sntpState = SM_UDP_RECV;		
			break;

		case SM_UDP_RECV:
			// Look for a response time packet
			if(!UDPIsGetReady(sntpSocket)) 
			{
				if((SYS_TICK_Get()) - SNTPTimer > NTP_REPLY_TIMEOUT * SYS_TICK_TicksPerSecondGet())
				{
					// Abort the request and resume
					UDPClose(sntpSocket);
					//SNTPTimer = SYS_TICK_Get();
					//sntpState = SM_SHORT_WAIT;
					sntpState = SM_HOME;
					sntpSocket = INVALID_UDP_SOCKET;
				}
				break;
			}
			
			// Get the response time packet
			w = UDPGetArray(sntpSocket, (uint8_t*) &pkt, sizeof(pkt));
			UDPClose(sntpSocket);
			SNTPTimer = SYS_TICK_Get();
			sntpState = SM_WAIT;
			sntpSocket = INVALID_UDP_SOCKET;
			
			// Validate packet size
			if(w != sizeof(pkt)) 
			{
				break;	
			}
			
			// Set out local time to match the returned time
			dwLastUpdateTick = SYS_TICK_Get();
			dwSNTPSeconds = swapl(pkt.tx_ts_secs) - NTP_EPOCH;
			// Do rounding.  If the partial seconds is > 0.5 then add 1 to the seconds count.
			if(((uint8_t*)&pkt.tx_ts_fraq)[0] & 0x80)
				dwSNTPSeconds++;

			break;

		case SM_SHORT_WAIT:
			// Attempt to requery the NTP server after a specified NTP_FAST_QUERY_INTERVAL time (ex: 8 seconds) has elapsed.
			if(SYS_TICK_Get() - SNTPTimer > (NTP_FAST_QUERY_INTERVAL * SYS_TICK_TicksPerSecondGet()))
			{
				sntpState = SM_HOME;
				sntpSocket = INVALID_UDP_SOCKET;
			}
			break;

		case SM_WAIT:
			// Requery the NTP server after a specified NTP_QUERY_INTERVAL time (ex: 10 minutes) has elapsed.
			if(SYS_TICK_Get() - SNTPTimer > (NTP_QUERY_INTERVAL * SYS_TICK_TicksPerSecondGet()))
			{
				sntpState = SM_HOME;
				sntpSocket = INVALID_UDP_SOCKET;
			}

			break;
	}
}


/*****************************************************************************
  Function:
	uint32_t SNTPGetUTCSeconds(void)

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
uint32_t SNTPGetUTCSeconds(void)
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

#endif  //if defined(TCPIP_STACK_USE_SNTP_CLIENT)
