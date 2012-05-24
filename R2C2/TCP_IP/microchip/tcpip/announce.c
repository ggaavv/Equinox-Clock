/*******************************************************************************
  Announce Client and Server

  Summary:
    Module for Microchip TCP/IP Stack
    
  Description:
    - Provides device hostname and IP address discovery on a local 
      Ethernet subnet (same broadcast domain)
    - Reference: None.  Hopefully AN833 in the future.
*******************************************************************************/

/*******************************************************************************
FileName:   Announce.c
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

#define __ANNOUNCE_C

#include "tcpip_private.h"
#include "tcpip_config_private.h"

#if defined(TCPIP_STACK_USE_ANNOUNCE)

#define ANNOUNCE_MAX_PAYLOAD        (1500 - sizeof (IP_HEADER) - sizeof (UDP_HEADER))

typedef enum
{
    ANNOUNCE_FIELD_TRUNCATED = 0x01,
    ANNOUNCE_FIELD_MAC_ADDR,
    ANNOUNCE_FIELD_MAC_TYPE,
    ANNOUNCE_FIELD_HOST_NAME,
    ANNOUNCE_FIELD_IPV4_ADDRESS,
    ANNOUNCE_FIELD_IPV6_UNICAST,
    ANNOUNCE_FIELD_IPV6_MULTICAST,
} ANNOUNCE_FIELD_PAYLOAD;

static const uint8_t announceFieldTerminator[] = "\r\n";

#ifdef TCPIP_STACK_USE_IPV6
    extern IPV6_INTERFACE_CONFIG ipv6Config[];
#endif

typedef struct _TAG_ANNOUNCE_LIST_NODE
{
	struct _TAG_ANNOUNCE_LIST_NODE*		next;		// next node in list
    TCPIP_NET_HANDLE handle;
    uint8_t event;
} ANNOUNCE_LIST_NODE;

extern NODE_INFO remoteNode;

static DHCP_HANDLE announceDHCPHandler = NULL;
#if defined (TCPIP_STACK_USE_IPV6)
static IPV6_HANDLE announceIPV6Handler = NULL;
#endif

static bool announceEventPending = false;

static const void*          announceMemH = 0;        // memory handle

static SINGLE_LIST          announceEvents = { 0 };

bool ANNOUNCE_Init(const TCPIP_STACK_MODULE_CTRL* const stackData,
             const ANNOUNCE_MODULE_CONFIG* const announceData)
{
    if (announceDHCPHandler == NULL)
    {
        announceDHCPHandler = DHCPRegisterHandler(0, (DHCP_EVENT_HANDLER)ANNOUNCE_Notify, NULL);
    }

    if (announceDHCPHandler == NULL)
    {
        return false;
    }

#if defined (TCPIP_STACK_USE_IPV6)
    if (announceIPV6Handler == NULL)
    {
        announceIPV6Handler = TCPIP_IPV6_RegisterHandler(0, (IPV6_EVENT_HANDLER)ANNOUNCE_Notify, NULL);
    }

    if (announceIPV6Handler == NULL)
    {
        DHCPDeRegisterHandler(announceDHCPHandler);
        announceDHCPHandler = NULL;
        return false;
    }
#endif

    announceMemH = stackData->memH;

    announceEventPending = false;

    return true;
}

void ANNOUNCE_DeInit(const TCPIP_STACK_MODULE_CTRL* const stackData)
{
    SGL_LIST_NODE*  node;

    announceEventPending = false;

    if (announceDHCPHandler != NULL)
    {
        DHCPDeRegisterHandler(announceDHCPHandler);
        announceDHCPHandler = NULL;
    }

#if defined (TCPIP_STACK_USE_IPV6)
    if (announceIPV6Handler != NULL)
    {
        TCPIP_IPV6_DeRegisterHandler(announceIPV6Handler);
        announceIPV6Handler = NULL;
    }
#endif

    // Free all announce events
    while ((node = SingleListRemoveHead(&announceEvents)) != NULL)
    {
        TCPIP_HEAP_Free (announceMemH, node);
    }

    announceMemH = 0;
}

void ANNOUNCE_Notify(TCPIP_NET_HANDLE hNet, uint8_t evType, const void * param)
{
    if(announceMemH)
    {
        ANNOUNCE_LIST_NODE* newNode = TCPIP_HEAP_Malloc(announceMemH, sizeof(*newNode));

        if(newNode)
        {
            newNode->handle = hNet;
            newNode->event = evType;
            SingleListAddTail(&announceEvents, (SGL_LIST_NODE*)newNode);
            announceEventPending = true;
            return;
        }
    }

    return;
}

void ANNOUNCE_Send(void)
{
    UDP_SOCKET  announceSocket;
    int         netIx;
    uint16_t    dataLen;
    uint16_t    minimumDataLen;
    uint16_t    txLen;
    bool truncated;
    NET_CONFIG * pNetIf;
    ANNOUNCE_LIST_NODE *  node = (ANNOUNCE_LIST_NODE *)announceEvents.head;
    ANNOUNCE_FIELD_PAYLOAD payloadType;
    uint16_t terminatorLen = strlen ((const char *)announceFieldTerminator);

#if defined (TCPIP_STACK_USE_IPV6)
    IPV6_ADDR_STRUCT * addressPointer;
#endif

    while (node != NULL)
    {    
        pNetIf = (NET_CONFIG *)node->handle;

        netIx = TCPIP_STACK_NetIx (pNetIf);

        truncated = false;

        dataLen = ((terminatorLen + 1) * 4) + sizeof (IPV4_ADDR) + sizeof (MAC_ADDR);

        dataLen += strlen(TCPIP_HOSTS_CONFIGURATION[netIx].interface);
        dataLen += strlen((char *)pNetIf->NetBIOSName);
    
        minimumDataLen = dataLen + 1 + terminatorLen;
    
        if(!MACIsLinked(_TCPIPStackNetToMac(pNetIf)))  // Check for link before blindly opening and transmitting (similar to DHCP case)
        {
            return;
        }
    
        announceSocket = UDPOpenClient(IP_ADDRESS_TYPE_IPV4, ANNOUNCE_PORT, 0);
    
        if (announceSocket == INVALID_UDP_SOCKET)
        {
            return;
        }
    
        UDPSocketSetNet (announceSocket, pNetIf);
    
    #if defined (TCPIP_STACK_USE_IPV6)
        addressPointer = (IPV6_ADDR_STRUCT *)ipv6Config[netIx].listIpv6UnicastAddresses.head;
    
        while(addressPointer != NULL)
        {
            dataLen += sizeof (IPV6_ADDR) + 1 + terminatorLen;
            addressPointer = addressPointer->next;
        }
    
        addressPointer = (IPV6_ADDR_STRUCT *)ipv6Config[netIx].listIpv6MulticastAddresses.head;
    
        while(addressPointer != NULL)
        {
            dataLen += sizeof (IPV6_ADDR) + 1 + terminatorLen;
            addressPointer = addressPointer->next;
        }
    #endif
    
        if (dataLen > ANNOUNCE_MAX_PAYLOAD)
        {
            dataLen = ANNOUNCE_MAX_PAYLOAD;
        }
    
        if ((txLen = UDPIsTxPutReady(announceSocket, dataLen)) < dataLen)
        {
            truncated = true;
            if ((txLen = UDPIsTxPutReady(announceSocket, minimumDataLen)) < minimumDataLen)
            {
                UDPClose (announceSocket);
                return;
            }
        }
    
        // Put Mac Address
        payloadType = ANNOUNCE_FIELD_MAC_ADDR;
        UDPPut (announceSocket, payloadType);
        UDPPutArray(announceSocket, (const uint8_t *)&pNetIf->MyMACAddr, sizeof (MAC_ADDR));
        UDPPutArray (announceSocket, announceFieldTerminator, terminatorLen);

        if (truncated)
        {
            payloadType = ANNOUNCE_FIELD_TRUNCATED;
            UDPPut (announceSocket, payloadType);
            UDPPutArray (announceSocket, announceFieldTerminator, terminatorLen);
        }

        // Put Mac Type
        payloadType = ANNOUNCE_FIELD_MAC_TYPE;
        UDPPut (announceSocket, payloadType);
        UDPPutArray(announceSocket, (const uint8_t *)TCPIP_HOSTS_CONFIGURATION[netIx].interface, strlen ((const char *)TCPIP_HOSTS_CONFIGURATION[netIx].interface));
        UDPPutArray (announceSocket, announceFieldTerminator, terminatorLen);

        // Put Host Name
        payloadType = ANNOUNCE_FIELD_HOST_NAME;
        UDPPut (announceSocket, payloadType);
        UDPPutArray(announceSocket, (const uint8_t *)&pNetIf->NetBIOSName, strlen((char*)pNetIf->NetBIOSName));
        UDPPutArray (announceSocket, announceFieldTerminator, terminatorLen);

        // Put IPv4 Address
        payloadType = ANNOUNCE_FIELD_IPV4_ADDRESS;
        UDPPut (announceSocket, payloadType);
        UDPPutArray(announceSocket, (const uint8_t *)&pNetIf->MyIPAddr, sizeof (IP_ADDR));
        UDPPutArray (announceSocket, announceFieldTerminator, terminatorLen);
    
#if defined (TCPIP_STACK_USE_IPV6)
    
        // Put IPv6 unicast addresses
        minimumDataLen = sizeof (IPV6_ADDR) + 1 + terminatorLen;
    
        addressPointer = (IPV6_ADDR_STRUCT *)ipv6Config[netIx].listIpv6UnicastAddresses.head;
    
        payloadType = ANNOUNCE_FIELD_IPV6_UNICAST;

        while(addressPointer != NULL && (UDPIsTxPutReady(announceSocket, minimumDataLen) >= minimumDataLen))
        {
            UDPPut (announceSocket, payloadType);
            UDPPutArray(announceSocket, (const uint8_t *)&addressPointer->address, sizeof (IPV6_ADDR));
            UDPPutArray (announceSocket, announceFieldTerminator, terminatorLen);
            addressPointer = addressPointer->next;
        }
    
        // Put IPv6 multicast listeners    
        addressPointer = (IPV6_ADDR_STRUCT *)ipv6Config[netIx].listIpv6MulticastAddresses.head;
    
        payloadType = ANNOUNCE_FIELD_IPV6_MULTICAST;

        while(addressPointer != NULL && (UDPIsTxPutReady(announceSocket, minimumDataLen) >= minimumDataLen))
        {
            UDPPut (announceSocket, payloadType);
            UDPPutArray(announceSocket, (const uint8_t *)&addressPointer->address, sizeof (IPV6_ADDR));
            UDPPutArray (announceSocket, announceFieldTerminator, terminatorLen);
            addressPointer = addressPointer->next;
        }
#endif
    
        UDPFlush (announceSocket);
    
        UDPClose (announceSocket);

        SingleListRemoveHead(&announceEvents);

        TCPIP_HEAP_Free (announceMemH, node);

        node = (ANNOUNCE_LIST_NODE *)announceEvents.head;

        if (node == NULL)
        {
            announceEventPending = false;
        }
    }
}

bool ANNOUNCE_TaskPending (void)
{
    return announceEventPending;
}

/*********************************************************************
 * Function:        void ANNOUNCE_Task(void)
 *
 * Summary:         Announce callback task.
 *
 * PreCondition:    Stack is initialized()
 *
 * Input:           None
 *
 * Output:          None
 *
 * Side Effects:    None
 *
 * Overview:        Recurring task used to listen for Discovery
 *                  messages on the specified ANNOUNCE_PORT.  These
 *                  messages can be sent using the TCP/IP
 *                  Discoverer tool. If one is received, this
 *                  function will transmit a reply.
 *
 * Note:            A UDP socket must be available before this 
 *					function is called.  It is freed at the end of 
 *					the function.  UDP_MAX_SOCKETS may need to be 
 *					increased if other modules use UDP sockets.
 ********************************************************************/
void ANNOUNCE_Task(NET_CONFIG * pNetIf)
{
	static enum {
		DISCOVERY_HOME = 0,
		DISCOVERY_LISTEN,
		DISCOVERY_REQUEST_RECEIVED,
		DISCOVERY_DISABLED
	} DiscoverySM[TCPIP_NETWORK_INTERFACES] = {DISCOVERY_HOME};

	static UDP_SOCKET	MySocket[TCPIP_NETWORK_INTERFACES];
	uint8_t 				i;
    int                 netIx;
    UDP_SOCKET          s;

    if(!pNetIf)
    {
        return;
    }
    else
    {
        netIx = _TCPIPStackNetIx(pNetIf);
    }
    
    s = MySocket[netIx];

	switch(DiscoverySM[netIx])
	{
		case DISCOVERY_HOME:
			// Open a UDP socket for inbound and outbound transmission
			// Since we expect to only receive broadcast packets and 
			// only send unicast packets directly to the node we last 
			// received from, the remote NodeInfo parameter can be anything
			MySocket[netIx] = UDPOpen(0,UDP_OPEN_SERVER,ANNOUNCE_PORT, ANNOUNCE_PORT);

			if(MySocket[netIx] == INVALID_UDP_SOCKET)
            {
				return;
            }
			else
            {
                DiscoverySM[netIx]++;
                UDPSocketSetNet(MySocket[netIx], pNetIf);
            }
			break;

		case DISCOVERY_LISTEN:
			// Do nothing if no data is waiting
			if(!UDPIsGetReady(s))
				return;
			
			// See if this is a discovery query or reply
			UDPGet(s, &i);
			UDPDiscard(s);
			if(i != 'D')
				return;

			// We received a discovery request, reply when we can
			DiscoverySM[netIx]++;

			// Change the destination to the unicast address of the last received packet
            TCPIP_IPV4_SetDestAddress(UDPSocketDcpt[s].pTxPkt,remoteNode.IPAddr.Val);
        	memcpy((void*)&UDPSocketDcpt[s].pTxPkt->remoteMACAddr, (const void*)&remoteNode.MACAddr, sizeof(MAC_ADDR));
			
			// No break needed.  If we get down here, we are now ready for the DISCOVERY_REQUEST_RECEIVED state

		case DISCOVERY_REQUEST_RECEIVED:
            ANNOUNCE_Notify (pNetIf, 0, NULL);		
			// Listen for other discovery requests
			DiscoverySM[netIx] = DISCOVERY_LISTEN;
			break;

		case DISCOVERY_DISABLED:
			break;
	}	

}

#endif //#if defined(TCPIP_STACK_USE_ANNOUNCE)
