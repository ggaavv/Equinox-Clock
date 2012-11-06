/*******************************************************************************
  Dynamic Host Configuration Protocol (DHCP) Server

  Summary:
    Module for Microchip TCP/IP Stack
    
  Description:
    - Provides automatic IP address, subnet mask, gateway address, 
      DNS server address, and other configuration parameters on DHCP 
      enabled networks.
    - Reference: RFC 2131, 2132
*******************************************************************************/

/*******************************************************************************
FileName:   DHCPs.c
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
#define __DHCPS_C

#include "tcpip_private.h"
#include "tcpip_config_private.h"

#if defined(TCPIP_STACK_USE_DHCP_SERVER)

// DHCP Control Block.  Lease IP address is derived from index into DCB array.
typedef struct
{
	uint32_t 		LeaseExpires;	// Expiration time for this lease
	MAC_ADDR	ClientMAC;		// Client's MAC address.  Multicase bit is used to determine if a lease is given out or not
	enum 
	{
		LEASE_UNUSED = 0,
		LEASE_REQUESTED,
		LEASE_GRANTED
	} smLease;					// Status of this lease
} DHCP_CONTROL_BLOCK;


// DHCP is running on all interfaces
static UDP_SOCKET			MySocket[TCPIP_NETWORK_INTERFACES];		// Socket used by DHCP Server
static IP_ADDR				DHCPNextLease[TCPIP_NETWORK_INTERFACES];	// IP Address to provide for next lease
bool 						bDHCPServerEnabled[TCPIP_NETWORK_INTERFACES] = {true};	// Whether or not the DHCP server is enabled
static enum
{
    DHCP_OPEN_SOCKET,
        DHCP_LISTEN
} smDHCPServer[TCPIP_NETWORK_INTERFACES] = {DHCP_OPEN_SOCKET};

#if defined(TCPIP_STACK_USE_ZEROCONF_LINK_LOCAL)
static bool bLeaseAvailable[TCPIP_NETWORK_INTERFACES] = {true};
#endif

/// Ignore: static DHCP_CONTROL_BLOCK	DCB[DHCP_MAX_LEASES];	// Not Implmented

static void DHCPReplyToDiscovery(BOOTP_HEADER *Header, int netIx);
static void DHCPReplyToRequest(BOOTP_HEADER *Header, bool bAccept, int netIx);


/*****************************************************************************
  Function:
	void DHCPServerTask(NET_CONFIG* pConfig)

  Summary:
	Performs periodic DHCP server tasks.

  Description:
	This function performs any periodic tasks requied by the DHCP server 
	module, such as processing DHCP requests and distributing IP addresses.

  Precondition:
	None

  Parameters:
	pConfig   - interface

  Returns:
  	None
  ***************************************************************************/
void DHCPServerTask(NET_CONFIG* pConfig)
{
	uint8_t 				i;
	uint8_t				Option, Len;
	BOOTP_HEADER		BOOTPHeader;
	uint32_t				dw;
	bool				bAccept;
    int                 netIx;
    UDP_SOCKET          s;
    

#if defined(TCPIP_STACK_USE_DHCP_CLIENT)
	// Make sure we don't clobber anyone else's DHCP server
	if(DHCPIsServerDetected(pConfig))
		return;
#endif

    netIx = _TCPIPStackNetIx(pConfig);
	if(!bDHCPServerEnabled[netIx])
		return;

    s = MySocket[netIx];

	switch(smDHCPServer[netIx])
	{
		case DHCP_OPEN_SOCKET:
			// Obtain a UDP socket to listen/transmit on
			MySocket[netIx] = UDPOpen(0,UDP_OPEN_SERVER,DHCP_SERVER_PORT, DHCP_CLIENT_PORT);
			if(MySocket[netIx] == INVALID_UDP_SOCKET)
				break;

            UDPSocketSetNet(MySocket[netIx], pConfig);
			// Decide which address to lease out
			// Note that this needs to be changed if we are to 
			// support more than one lease
			DHCPNextLease[netIx].Val = (pConfig->MyIPAddr.Val & pConfig->MyMask.Val) + 0x02000000;
			if(DHCPNextLease[netIx].v[3] == 255u)
				DHCPNextLease[netIx].v[3] += 0x03;
			if(DHCPNextLease[netIx].v[3] == 0u)
				DHCPNextLease[netIx].v[3] += 0x02;

			smDHCPServer[netIx]++;

		case DHCP_LISTEN:
			// Check to see if a valid DHCP packet has arrived
			if(UDPIsGetReady(s) < 241u)
				break;

			// Retrieve the BOOTP header
			UDPGetArray(s, (uint8_t*)&BOOTPHeader, sizeof(BOOTPHeader));

			bAccept = (BOOTPHeader.ClientIP.Val == DHCPNextLease[netIx].Val) || (BOOTPHeader.ClientIP.Val == 0x00000000u);

			// Validate first three fields
			if(BOOTPHeader.MessageType != 1u)
				break;
			if(BOOTPHeader.HardwareType != 1u)
				break;
			if(BOOTPHeader.HardwareLen != 6u)
				break;

			// Throw away 10 unused bytes of hardware address,
			// server host name, and boot file name -- unsupported/not needed.
			for(i = 0; i < 64+128+(16-sizeof(MAC_ADDR)); i++)
				UDPGet(s, &Option);

			// Obtain Magic Cookie and verify
			UDPGetArray(s, (uint8_t*)&dw, sizeof(uint32_t));
			if(dw != 0x63538263ul)
				break;

			// Obtain options
			while(1)
			{
				// Get option type
				if(!UDPGet(s, &Option))
					break;
				if(Option == DHCP_END_OPTION)
					break;

				// Get option length
				UDPGet(s, &Len);
	
				// Process option
				switch(Option)
				{
					case DHCP_MESSAGE_TYPE:
						UDPGet(s, &i);
						switch(i)
						{
							case DHCP_DISCOVER_MESSAGE:
								DHCPReplyToDiscovery(&BOOTPHeader, netIx);
								break;

							case DHCP_REQUEST_MESSAGE:
							// NOTE : This #if section was missing from 5.36
                                #if defined(TCPIP_STACK_USE_ZEROCONF_LINK_LOCAL)
								if ( (BOOTPHeader.ClientIP.Val == 0x00000000u) &&
									 (bLeaseAvailable[netIx] == false) )
								{
									// Lease available only to the current lease holder
									break;
								}
								#endif

								DHCPReplyToRequest(&BOOTPHeader, bAccept, netIx);

							// NOTE : This #if section was missing from 5.36
                                #if defined(TCPIP_STACK_USE_ZEROCONF_LINK_LOCAL)
								bLeaseAvailable[netIx] = false;
								#endif

								break;

							// Need to handle these if supporting more than one DHCP lease
							case DHCP_RELEASE_MESSAGE:
							case DHCP_DECLINE_MESSAGE:
								break;
						}
						break;

					case DHCP_PARAM_REQUEST_IP_ADDRESS:
						if(Len == 4u)
						{
							// Get the requested IP address and see if it is the one we have on offer.
							UDPGetArray(s, (uint8_t*)&dw, 4);
							Len -= 4;
							bAccept = (dw == DHCPNextLease[netIx].Val);
						}
						break;

					case DHCP_END_OPTION:
						UDPDiscard(s);
						return;
				}

				// Remove any unprocessed bytes that we don't care about
				while(Len--)
				{
					UDPGet(s, &i);
				}
			}			

			UDPDiscard(s);
			break;
	}
}


/*****************************************************************************
  Function:
	static void DHCPReplyToDiscovery(BOOTP_HEADER *Header, int netIx)

  Summary:
	Replies to a DHCP Discover message.

  Description:
	This function replies to a DHCP Discover message by sending out a 
	DHCP Offer message.

  Precondition:
	None

  Parameters:
	Header - the BootP header this is in response to.
    netIx   - interface index

  Returns:
  	None
  ***************************************************************************/
static void DHCPReplyToDiscovery(BOOTP_HEADER *Header, int netIx)
{
	uint8_t         i;
    NET_CONFIG*     pConfig;
    UDP_SOCKET      s;
    
	// Set the correct socket to active and ensure that 
	// enough space is available to generate the DHCP response
    s = MySocket[netIx];
    if(UDPIsPutReady(s) < 300u)
		return;

    pConfig = UDPSocketGetNet(s);

	// Begin putting the BOOTP Header and DHCP options
	UDPPut(s, BOOT_REPLY);			// Message Type: 2 (BOOTP Reply)
	// Reply with the same Hardware Type, Hardware Address Length, Hops, and Transaction ID fields
	UDPPutArray(s, (uint8_t*)&(Header->HardwareType), 7);
	UDPPut(s, 0x00);				// Seconds Elapsed: 0 (Not used)
	UDPPut(s, 0x00);				// Seconds Elapsed: 0 (Not used)
	UDPPutArray(s, (uint8_t*)&(Header->BootpFlags), sizeof(Header->BootpFlags));
	UDPPut(s, 0x00);				// Your (client) IP Address: 0.0.0.0 (none yet assigned)
	UDPPut(s, 0x00);				// Your (client) IP Address: 0.0.0.0 (none yet assigned)
	UDPPut(s, 0x00);				// Your (client) IP Address: 0.0.0.0 (none yet assigned)
	UDPPut(s, 0x00);				// Your (client) IP Address: 0.0.0.0 (none yet assigned)
	UDPPutArray(s, (uint8_t*)&DHCPNextLease[netIx], sizeof(IP_ADDR));	// Lease IP address to give out
	UDPPut(s, 0x00);				// Next Server IP Address: 0.0.0.0 (not used)
	UDPPut(s, 0x00);				// Next Server IP Address: 0.0.0.0 (not used)
	UDPPut(s, 0x00);				// Next Server IP Address: 0.0.0.0 (not used)
	UDPPut(s, 0x00);				// Next Server IP Address: 0.0.0.0 (not used)
	UDPPut(s, 0x00);				// Relay Agent IP Address: 0.0.0.0 (not used)
	UDPPut(s, 0x00);				// Relay Agent IP Address: 0.0.0.0 (not used)
	UDPPut(s, 0x00);				// Relay Agent IP Address: 0.0.0.0 (not used)
	UDPPut(s, 0x00);				// Relay Agent IP Address: 0.0.0.0 (not used)
	UDPPutArray(s, (uint8_t*)&(Header->ClientMAC), sizeof(MAC_ADDR));	// Client MAC address: Same as given by client
	for(i = 0; i < 64+128+(16-sizeof(MAC_ADDR)); i++)	// Remaining 10 bytes of client hardware address, server host name: Null string (not used)
		UDPPut(s, 0x00);									// Boot filename: Null string (not used)
	UDPPut(s, 0x63);				// Magic Cookie: 0x63538263
	UDPPut(s, 0x82);				// Magic Cookie: 0x63538263
	UDPPut(s, 0x53);				// Magic Cookie: 0x63538263
	UDPPut(s, 0x63);				// Magic Cookie: 0x63538263
	
	// Options: DHCP Offer
	UDPPut(s, DHCP_MESSAGE_TYPE);	
	UDPPut(s, 1);
	UDPPut(s, DHCP_OFFER_MESSAGE);

	// Option: Subnet Mask
	UDPPut(s, DHCP_SUBNET_MASK);
	UDPPut(s, sizeof(IP_ADDR));
	UDPPutArray(s, (uint8_t*)&pConfig->MyMask, sizeof(IP_ADDR));

	// Option: Lease duration
	UDPPut(s, DHCP_IP_LEASE_TIME);
	UDPPut(s, 4);
	UDPPut(s, (DHCP_LEASE_DURATION>>24) & 0xFF);
	UDPPut(s, (DHCP_LEASE_DURATION>>16) & 0xFF);
	UDPPut(s, (DHCP_LEASE_DURATION>>8) & 0xFF);
	UDPPut(s, (DHCP_LEASE_DURATION) & 0xFF);

	// Option: Server identifier
	UDPPut(s, DHCP_SERVER_IDENTIFIER);	
	UDPPut(s, sizeof(IP_ADDR));
	UDPPutArray(s, (uint8_t*)&pConfig->MyIPAddr, sizeof(IP_ADDR));

	// Option: Router/Gateway address
	UDPPut(s, DHCP_ROUTER);		
	UDPPut(s, sizeof(IP_ADDR));
	UDPPutArray(s, (uint8_t*)&pConfig->MyIPAddr, sizeof(IP_ADDR));

	// Option: DNS server address
	UDPPut(s, DHCP_DNS);
	UDPPut(s, sizeof(IP_ADDR));
	UDPPutArray(s, (uint8_t*)&pConfig->MyIPAddr, sizeof(IP_ADDR));

	// No more options, mark ending
	UDPPut(s, DHCP_END_OPTION);

	// Add zero padding to ensure compatibility with old BOOTP relays that discard small packets (<300 UDP octets)
	while(UDPGetTxCount(s) < 300u)
		UDPPut(s, 0); 

	// Force remote destination address to be the broadcast address, regardless 
	// of what the node's source IP address was (to ensure we don't try to 
	// unicast to 0.0.0.0).
	memset((void*)&UDPSocketDcpt[s].remoteNode, 0xFF, sizeof(NODE_INFO));

	// Transmit the packet
	UDPFlush(s);
}


/*****************************************************************************
  Function:
	static void DHCPReplyToRequest(BOOTP_HEADER *Header, bool bAccept, int netIx)

  Summary:
	Replies to a DHCP Request message.

  Description:
	This function replies to a DHCP Request message by sending out a 
	DHCP Acknowledge message.

  Precondition:
	None

  Parameters:
	Header - the BootP header this is in response to.
	bAccept - whether or not we've accepted this request
    netIx   - interface index

  Returns:
  	None
  
  Internal:
	Needs to support more than one simultaneous lease in the future.
  ***************************************************************************/
static void DHCPReplyToRequest(BOOTP_HEADER *Header, bool bAccept, int netIx)
{
	uint8_t         i;
    NET_CONFIG*     pConfig;
    UDP_SOCKET      s;
    
	// Set the correct socket to active and ensure that 
	// enough space is available to generate the DHCP response
    s = MySocket[netIx];
    if(UDPIsPutReady(s) < 300u)
		return;
    
    pConfig = UDPSocketGetNet(s);

	// Search through all remaining options and look for the Requested IP address field
	// Obtain options
	while(UDPIsGetReady(s))
	{
		uint8_t Option, Len;
		uint32_t dw;

		// Get option type
		if(!UDPGet(s, &Option))
			break;
		if(Option == DHCP_END_OPTION)
			break;

		// Get option length
		UDPGet(s, &Len);

		// Process option
		if((Option == DHCP_PARAM_REQUEST_IP_ADDRESS) && (Len == 4u))
		{
			// Get the requested IP address and see if it is the one we have on offer.  If not, we should send back a NAK, but since there could be some other DHCP server offering this address, we'll just silently ignore this request.
			UDPGetArray(s, (uint8_t*)&dw, 4);
			Len -= 4;
			if(dw != DHCPNextLease[netIx].Val)
			{
				bAccept = false;
			}
			break;
		}

		// Remove the unprocessed bytes that we don't care about
		while(Len--)
		{
			UDPGet(s, &i);
		}
	}			

	// Begin putting the BOOTP Header and DHCP options
	UDPPut(s, BOOT_REPLY);			// Message Type: 2 (BOOTP Reply)
	// Reply with the same Hardware Type, Hardware Address Length, Hops, and Transaction ID fields
	UDPPutArray(s, (uint8_t*)&(Header->HardwareType), 7);
	UDPPut(s, 0x00);				// Seconds Elapsed: 0 (Not used)
	UDPPut(s, 0x00);				// Seconds Elapsed: 0 (Not used)
	UDPPutArray(s, (uint8_t*)&(Header->BootpFlags), sizeof(Header->BootpFlags));
	UDPPutArray(s, (uint8_t*)&(Header->ClientIP), sizeof(IP_ADDR));// Your (client) IP Address:
	UDPPutArray(s, (uint8_t*)&DHCPNextLease[netIx], sizeof(IP_ADDR));	// Lease IP address to give out
	UDPPut(s, 0x00);				// Next Server IP Address: 0.0.0.0 (not used)
	UDPPut(s, 0x00);				// Next Server IP Address: 0.0.0.0 (not used)
	UDPPut(s, 0x00);				// Next Server IP Address: 0.0.0.0 (not used)
	UDPPut(s, 0x00);				// Next Server IP Address: 0.0.0.0 (not used)
	UDPPut(s, 0x00);				// Relay Agent IP Address: 0.0.0.0 (not used)
	UDPPut(s, 0x00);				// Relay Agent IP Address: 0.0.0.0 (not used)
	UDPPut(s, 0x00);				// Relay Agent IP Address: 0.0.0.0 (not used)
	UDPPut(s, 0x00);				// Relay Agent IP Address: 0.0.0.0 (not used)
	UDPPutArray(s, (uint8_t*)&(Header->ClientMAC), sizeof(MAC_ADDR));	// Client MAC address: Same as given by client
	for(i = 0; i < 64+128+(16-sizeof(MAC_ADDR)); i++)	// Remaining 10 bytes of client hardware address, server host name: Null string (not used)
		UDPPut(s, 0x00);									// Boot filename: Null string (not used)
	UDPPut(s, 0x63);				// Magic Cookie: 0x63538263
	UDPPut(s, 0x82);				// Magic Cookie: 0x63538263
	UDPPut(s, 0x53);				// Magic Cookie: 0x63538263
	UDPPut(s, 0x63);				// Magic Cookie: 0x63538263
	
	// Options: DHCP lease ACKnowledge
	if(bAccept)
	{
		UDPPut(s, DHCP_OPTION_ACK_MESSAGE);	
		UDPPut(s, 1);
		UDPPut(s, DHCP_ACK_MESSAGE);
	}
	else	// Send a NACK
	{
		UDPPut(s, DHCP_OPTION_ACK_MESSAGE);	
		UDPPut(s, 1);
		UDPPut(s, DHCP_NAK_MESSAGE);
	}

	// Option: Lease duration
	UDPPut(s, DHCP_IP_LEASE_TIME);
	UDPPut(s, 4);
	UDPPut(s, (DHCP_LEASE_DURATION>>24) & 0xFF);
	UDPPut(s, (DHCP_LEASE_DURATION>>16) & 0xFF);
	UDPPut(s, (DHCP_LEASE_DURATION>>8) & 0xFF);
	UDPPut(s, (DHCP_LEASE_DURATION) & 0xFF);

	// Option: Server identifier
	UDPPut(s, DHCP_SERVER_IDENTIFIER);	
	UDPPut(s, sizeof(IP_ADDR));
	UDPPutArray(s, (uint8_t*)&pConfig->MyIPAddr, sizeof(IP_ADDR));

	// Option: Subnet Mask
	UDPPut(s, DHCP_SUBNET_MASK);
	UDPPut(s, sizeof(IP_ADDR));
	UDPPutArray(s, (uint8_t*)&pConfig->MyMask, sizeof(IP_ADDR));

	// Option: Router/Gateway address
	UDPPut(s, DHCP_ROUTER);		
	UDPPut(s, sizeof(IP_ADDR));
	UDPPutArray(s, (uint8_t*)&pConfig->MyIPAddr, sizeof(IP_ADDR));

	// Option: DNS server address
	UDPPut(s, DHCP_DNS);
	UDPPut(s, sizeof(IP_ADDR));
	UDPPutArray(s, (uint8_t*)&pConfig->MyIPAddr, sizeof(IP_ADDR));

	// No more options, mark ending
	UDPPut(s, DHCP_END_OPTION);

	// Add zero padding to ensure compatibility with old BOOTP relays that discard small packets (<300 UDP octets)
	while(UDPGetTxCount(s) < 300u)
		UDPPut(s, 0); 

	// Force remote destination address to be the broadcast address, regardless 
	// of what the node's source IP address was (to ensure we don't try to 
	// unicast to 0.0.0.0).
	memset((void*)&UDPSocketDcpt[s].remoteNode, 0xFF, sizeof(NODE_INFO));

	// Transmit the packet
	UDPFlush(s);
}

#endif //#if defined(TCPIP_STACK_USE_DHCP_SERVER)
