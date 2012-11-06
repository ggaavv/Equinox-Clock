/*******************************************************************************
  ICMP Client Demo (Ping)

  Summary:
    
  Description:
*******************************************************************************/

/*******************************************************************************
FileName:  PingDemo.c 
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

#define __PINGDEMO_C

#include "tcpip_config.h"

#if defined(TCPIP_STACK_USE_ICMP_CLIENT) || defined(TCPIP_STACK_USE_ICMP_SERVER) || defined(TCPIP_STACK_USE_IPV6)

#include "tcpip/tcpip.h"
#include "main_demo.h"

#if defined (TCPIP_STACK_USE_IPV6)
    IPV6_ADDR targetAddressIPv6;
#endif
IP_ADDR targetAddressIPv4;
IP_ADDR firstHopAddress;
MAC_ADDR targetMACAddr;
char * targetHostName;

static unsigned short wICMPSequenceNumber = 0;
SYS_TICK pingTimer;
uint32_t miscData = 0x11FF22EE;
uint8_t pingCount;
NET_CONFIG * pNetIf;

static enum
{
    STATE_IDLE = 0,
    STATE_DNS_SEND_QUERY_IPV4,
    STATE_DNS_GET_RESPONSE_IPV4,
    STATE_RESOLVE_ARP,
    STATE_ARP_RESOLVED,
    STATE_SEND_ECHO_REQUEST_IPV4,
    STATE_GET_RESPONSE_IPV4,
#if defined (TCPIP_STACK_USE_IPV6)
    STATE_DNS_SEND_QUERY_IPV6,
    STATE_DNS_GET_RESPONSE_IPV6,
    STATE_SEND_ECHO_REQUEST_IPV6,
    STATE_GET_RESPONSE_IPV6
#endif
} pingState;

/*****************************************************************************
  Function:
	bool Ping6(int8_t * target)

  Summary:
	Sends an ICMPv6 Echo Request to the target.
	
  Description:
	This function begins the state machine for transmitting ICMPv6 Echo 
    Requests and processing the responses from them.
	
	This function can be used as a model for applications requiring Ping6 
	capabilities to check if a host is reachable.

  Precondition:
	None.

  Parameters:
	target   - the target IPV6_ADDR or host name to ping

  Returns:
  	None
  ***************************************************************************/
#if defined (TCPIP_STACK_USE_IPV6)
bool Ping6 (char * target)
{
    if (pingState != STATE_IDLE)
        return false;

    pNetIf = (NET_CONFIG*)TCPIP_STACK_GetDefaultNet();

    pingCount = 0;

    if (TCPIP_HELPER_StringToIPv6Address ((uint8_t *)target, &targetAddressIPv6))
    {
        // The user passed in a valid IPv6 address
        pingState = STATE_SEND_ECHO_REQUEST_IPV6;
    }
    else
    {
        // The user passed in a host name
        targetHostName = target;
        pingState = STATE_DNS_SEND_QUERY_IPV6;
    }

    return true;
}
#endif

/*****************************************************************************
  Function:
	bool Ping4(int8_t * target)

  Summary:
	Sends an ICMP Echo Request to the target.
	
  Description:
	This function begins the state machine for transmitting ICMP Echo 
    Requests and processing the responses from them.
	
	This function can be used as a model for applications requiring Ping6 
	capabilities to check if a host is reachable.

  Precondition:
	None.

  Parameters:
	target   - the target IP_ADDR or host name to ping

  Returns:
  	None
  ***************************************************************************/
bool Ping4 (char * target)
{
    if (pingState != STATE_IDLE)
        return false;

    pNetIf = (NET_CONFIG*)TCPIP_STACK_GetDefaultNet();

    pingCount = 0;

    if (TCPIP_HELPER_StringToIPAddress ((const char *)target, &targetAddressIPv4))
    {
        // The user passed in a valid IPv4 address
        pingState = STATE_RESOLVE_ARP;
    }
    else
    {
        // The user passed in a host name
        targetHostName = target;
        pingState = STATE_DNS_SEND_QUERY_IPV4;
    }

    return true;
}

/*****************************************************************************
  Function:
	void PingDemoTask (void)

  Summary:
	Handles state machine for ping demo processes.
	
  Description:
	This function performs state processing for the ping demo.
	
	This function can be used as a model for applications requiring Ping6 
	capabilities to check if a host is reachable.

  Precondition:
	None.

  Parameters:
	None

  Returns:
  	None
  ***************************************************************************/
void PingDemoTask (void)
{
    switch (pingState)
    {
#if defined (TCPIP_STACK_USE_ICMP_CLIENT)
        case STATE_DNS_SEND_QUERY_IPV4:
            if (!DNSBeginUsage(pNetIf))
                return;

            if (DNSResolve((const char *)targetHostName, DNS_TYPE_A) != DNS_RES_OK)
                return;

            pingTimer = SYS_TICK_Get() + (SYS_TICK_ResolutionGet() * TCPIP_PING_DNS_TIMEOUT);
            pingState = STATE_DNS_GET_RESPONSE_IPV4;
            break;
        case STATE_DNS_GET_RESPONSE_IPV4:
            {
                DNS_RESULT res;
                if ((long)(SYS_TICK_Get() - pingTimer) > 0)
                {
                    SYS_OUT_MESSAGE_LINE("Couldn't resolve", 2);
                    DNSEndUsage(pNetIf);
                    pingState = STATE_IDLE;
                    return;
                } 
    
                res = DNSIsResolved((const char *)targetHostName, &targetAddressIPv4);

                switch (res)
                {
                    case DNS_RES_OK:
                        DNSEndUsage(pNetIf);
                        pingState = STATE_RESOLVE_ARP;
                        break;
                    case DNS_RES_PENDING:
                        
                        break;
                    default:
                        SYS_OUT_MESSAGE_LINE ("Couldn't resolve", 1);
                        DNSEndUsage(pNetIf);
                        pingState = STATE_IDLE;
                        break;
                }
            }
            break;
		case STATE_RESOLVE_ARP:
            if ((targetAddressIPv4.Val & pNetIf->MyMask.Val) == pNetIf->MyMask.Val)
                firstHopAddress.Val = targetAddressIPv4.Val;
            else
                firstHopAddress.Val = pNetIf->MyGateway.Val;
  			ARPResolve(pNetIf, &firstHopAddress);
			pingTimer = SYS_TICK_Get();
			pingState = STATE_ARP_RESOLVED;
			break;

		case STATE_ARP_RESOLVED:
			if(!ARPIsResolved(pNetIf, &firstHopAddress, &targetMACAddr))
			{
				if(SYS_TICK_Get() - pingTimer > (TCPIP_PING_DNS_TIMEOUT * SYS_TICK_TicksPerSecondGet()))
                {
                    SYS_OUT_MESSAGE_LINE ("Couldn't ARP", 1);
					pingState = STATE_IDLE;
                }
				break;
			}

			pingState = STATE_SEND_ECHO_REQUEST_IPV4;
            break;
#endif
#if defined (TCPIP_STACK_USE_IPV6)
        case STATE_DNS_SEND_QUERY_IPV6:
            if (!DNSBeginUsage(pNetIf))
                return;

            if (DNSResolve((const char *)targetHostName, DNS_TYPE_AAAA) != DNS_RES_OK)
                return;

            pingTimer = SYS_TICK_Get() + (SYS_TICK_ResolutionGet() * TCPIP_PING_DNS_TIMEOUT);
            pingState = STATE_DNS_GET_RESPONSE_IPV6;
            break;
        case STATE_DNS_GET_RESPONSE_IPV6:
            {
                DNS_RESULT res;
                if ((long)(SYS_TICK_Get() - pingTimer) > 0)
                {
                    SYS_OUT_MESSAGE_LINE ("Couldn't resolve", 1);
                    DNSEndUsage(pNetIf);
                    pingState = STATE_IDLE;
                    return;
                } 
    
                res = DNSIsResolved((const char *)targetHostName, &targetAddressIPv6);

                switch (res)
                {
                    case DNS_RES_OK:
                        DNSEndUsage(pNetIf);
                        pingState = STATE_SEND_ECHO_REQUEST_IPV6;
                        break;
                    case DNS_RES_PENDING:
                        
                        break;
                    default:
                        SYS_OUT_MESSAGE_LINE ("Couldn't resolve", 1);
                        DNSEndUsage(pNetIf);
                        pingState = STATE_IDLE;
                        break;
                }
            }
            break;
#endif
#if defined(TCPIP_STACK_USE_ICMP_CLIENT)
        case STATE_SEND_ECHO_REQUEST_IPV4:
            {
                NODE_INFO info;

                info.IPAddr = targetAddressIPv4;
                memcpy (&info.MACAddr, &targetMACAddr, sizeof (MAC_ADDR));
                ICMPSendEchoRequest (&info, ++wICMPSequenceNumber, 0xBEEF);

    			// Record the current time.  This will be used as a basis for 
    			// finding the echo response time, which exludes the ARP and DNS 
    			// steps
   			    pingTimer = SYS_TICK_Get();
    
                pingCount++;

                SYS_OUT_MESSAGE_LINE ("Pinging...", 1);

    			// Echo sent, advance state
    			pingState = STATE_GET_RESPONSE_IPV4;
            }
            break;
#endif
#if defined (TCPIP_STACK_USE_IPV6)
        case STATE_SEND_ECHO_REQUEST_IPV6:
            {
                IP_PACKET * pkt;
                IPV6_ADDR_STRUCT * localAddress;

                localAddress = TCPIP_IPV6_DAS_SelectSourceAddress (pNetIf, &targetAddressIPv6, NULL);

                if (localAddress == NULL)
                {
                    SYS_OUT_MESSAGE_LINE ("No local addr!", 1);
                    pingState = STATE_IDLE;
                    break;
                }
    
                pkt = TCPIP_ICMPV6_PutHeaderEchoRequest (pNetIf, &localAddress->address, &targetAddressIPv6, ICMPV6_INFO_ECHO_REQUEST, 
                                                    0xEFBE, ++wICMPSequenceNumber);
    
                if (TCPIP_IP_IsTxPutReady(pkt, 4) < 4)
                {
                    TCPIP_IP_FreePacket (pkt);
                    return;
                }
    
                TCPIP_IP_PutArray (pkt, (uint8_t *)&miscData, sizeof (uint32_t));
    
                // Just let the IPv6 module figure out the next hop neighbor and its MAC address
                TCPIP_ICMPV6_Flush (pkt);
    		
    			// Record the current time.  This will be used as a basis for 
    			// finding the echo response time, which exludes the ARP and DNS 
    			// steps
   			    pingTimer = SYS_TICK_Get();
    
                pingCount++;

                SYS_OUT_MESSAGE_LINE ("Pinging...", 1);

    			// Echo sent, advance state
    			pingState = STATE_GET_RESPONSE_IPV6;
            }
            break;
#endif
#if defined (TCPIP_STACK_USE_ICMP_CLIENT)
        case STATE_GET_RESPONSE_IPV4:
            if ((long)(SYS_TICK_Get() - pingTimer) > (SYS_TICK_ResolutionGet() * TCPIP_PING_TIMEOUT))
            {
                SYS_OUT_MESSAGE_LINE("Ping timeout", 1);
                pingState = STATE_IDLE;
            }            
            break;
#endif
#if defined (TCPIP_STACK_USE_IPV6)
        case STATE_GET_RESPONSE_IPV6:
            if ((long)(SYS_TICK_Get() - pingTimer) > (SYS_TICK_ResolutionGet() * TCPIP_PING_TIMEOUT))
            {
                SYS_OUT_MESSAGE_LINE ("Ping timeout", 1);
                pingState = STATE_IDLE;
            }
            break;
#endif
        default:
        case STATE_IDLE:
            break;
        
    }
}

/*****************************************************************************
  Function:
	bool PingProcessIPv6 (ICMPV6_HEADER_ECHO * header, IPV6_ADDR * remoteIP, IPV6_ADDR * localIP)

  Summary:
	Processes incoming IPv6 Echo Replies
	
  Description:
	This function processes incoming IPv6 Echo Replies.
	
	This function can be used as a model for applications requiring Ping6 
	capabilities to check if a host is reachable.

  Precondition:
	None.

  Parameters:
	header - The ICMPv6 header of the incoming echo reply
    remoteIP - The address of the node that sent the echo reply
    localIP - The destination address of the echo reply

  Returns:
  	bool - True if the ping demo was expecting an echo reply from the remote 
            node, otherwise false
  ***************************************************************************/
#if defined (TCPIP_STACK_USE_IPV6)
bool PingProcessIPv6 (ICMPV6_HEADER_ECHO * header, IPV6_ADDR * remoteIP, IPV6_ADDR * localIP)
{
    char message[16];
    char time[5];

    if (pingState != STATE_GET_RESPONSE_IPV6)
        return false;

    if (header->sequenceNumber != wICMPSequenceNumber)
        return false;

    if (header->identifier != 0xEFBE)
        return false;

    if (memcmp (remoteIP, &targetAddressIPv6, sizeof (IPV6_ADDR)))
        return false;

    pingTimer = SYS_TICK_Get() - pingTimer;

    uitoa (pingTimer << 1, (uint8_t *)time);

    strcpy(message, "Reply: ");
    strcat(message, time);
    strcat(message, "ms");
    SYS_OUT_MESSAGE_LINE(message, 1);

    if (pingCount < TCPIP_PING_COUNT_DEFAULT)
    {
        pingState = STATE_SEND_ECHO_REQUEST_IPV6;
    }
    else
    {
        pingState = STATE_IDLE;
    }

    return true;
}
#endif

/*****************************************************************************
  Function:
	bool PingProcessIPv4 (IP_ADDR * remoteIP, TCPIP_UINT32_VAL * data)

  Summary:
	Processes incoming IPv4 Echo Replies
	
  Description:
	This function processes incoming IPv4 Echo Replies.
	
	This function can be used as a model for applications requiring Ping6 
	capabilities to check if a host is reachable.

  Precondition:
	None.

  Parameters:
    remoteIP - The address of the node that sent the echo reply
    data - The identifier and sequence number

  Returns:
  	bool - True if the ping demo was expecting an echo reply from the remote 
            node, otherwise false
  ***************************************************************************/
void PingProcessIPv4 (IP_ADDR * remoteIP, void * data)
{
    char message[16];
    char time[5];


    TCPIP_UINT32_VAL packetInformation = *((TCPIP_UINT32_VAL *)data);

    if (pingState != STATE_GET_RESPONSE_IPV4)
        return;

    if (swaps(packetInformation.w[1]) != wICMPSequenceNumber)
        return;

    if (packetInformation.w[0] != 0xEFBE)
        return;

    if (remoteIP->Val != targetAddressIPv4.Val)
        return;

    pingTimer = SYS_TICK_Get() - pingTimer;

    uitoa (pingTimer << 1, (uint8_t *)time);

    strcpy(message, "Reply: ");
    strcat(message, time);
    strcat(message, "ms");
    SYS_OUT_MESSAGE_LINE(message, 1);

    if (pingCount < TCPIP_PING_COUNT_DEFAULT)
    {
        pingState = STATE_SEND_ECHO_REQUEST_IPV4;
    }
    else
    {
        pingState = STATE_IDLE;
    }

    return;
}

#endif // TCPIP_STACK_USE_ICMP*, TCPIP_STACK_USE_IPV6

