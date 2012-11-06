/*******************************************************************************
  Internet Control Message Protocol (ICMP) Server

  Summary:
    Module for Microchip TCP/IP Stack
    
  Description:
    - Provides "ping" diagnostics
    - Reference: RFC 792
*******************************************************************************/

/*******************************************************************************
FileName:   ICMP.c
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

#define __ICMP_C

#include "tcpip_private.h"
#include "tcpip_config_private.h"


#if defined(TCPIP_STACK_USE_ICMP_SERVER) || defined(TCPIP_STACK_USE_ICMP_CLIENT)

// ICMP Header Structure
typedef struct
{
    uint8_t vType;
    uint8_t vCode;
    uint16_t wChecksum;
    uint16_t wIdentifier;
    uint16_t wSequenceNumber;
} ICMP_HEADER;

void (*icmpCallback) (IP_ADDR * remoteIP, void * data);

static IP_PACKET*    icmpTxPkt[TCPIP_NETWORK_INTERFACES] = { 0 };
static const void*      icmpMemH = 0;                       // memory heap handle

#if defined(TCPIP_STACK_USE_ICMP_CLIENT)

static bool bICMPInUse;

// ICMP Packet Structure
typedef struct
{
	uint8_t vType;
	uint8_t vCode;
	uint16_t wChecksum;
	uint16_t wIdentifier;
	uint16_t wSequenceNumber;
	uint32_t wData;
} ICMP_PACKET;

#endif

bool ICMPInitialize(const TCPIP_STACK_MODULE_CTRL* const pStackInit, const ICMP_MODULE_GONFIG* const pIcmpInit)
{
    int          netIx;     
    
    if(pStackInit->stackAction == TCPIP_STACK_ACTION_IF_UP)
    {   // interface restart
        return true;
    }

    // stack start up
    netIx = pStackInit->netIx;
    
    if(icmpTxPkt[netIx] == 0)
    {   // memory not allocated yet
        icmpMemH = pStackInit->memH;

        icmpTxPkt[netIx] = ICMPAllocateTxPacketStruct (pStackInit->pNetIf);

        if (icmpTxPkt[netIx] == NULL)
            return false;
    }

    icmpCallback = NULL;

    return true;
}

void ICMPRegisterCallback (void (*callback)(IP_ADDR * remoteIP, void * data))
{
    icmpCallback = callback;
}

void ICMPDeinitialize(const TCPIP_STACK_MODULE_CTRL* const pStackInit)
{
    
    if(pStackInit->stackAction == TCPIP_STACK_ACTION_DEINIT)
    {   // stack shut down
        int netIx = pStackInit->netIx;

        if(icmpTxPkt[netIx] != 0)
        {   // memory allocated 
            TCPIP_IP_ResetTransmitPacketState (icmpTxPkt[netIx]);
            icmpTxPkt[netIx] = 0;
        }
    }

}

IP_PACKET * ICMPAllocateTxPacketStruct (NET_CONFIG * pNetIf)
{
    IP_PACKET * ptrPacket;

    ptrPacket = TCPIP_IP_AllocateTxPacket (pNetIf, IP_ADDRESS_TYPE_IPV4, 0, 0);

    if (ptrPacket == NULL)
    {
        return NULL;
    }

    if (!TCPIP_IP_PutUpperLayerHeader (ptrPacket, NULL, sizeof (ICMP_HEADER), IP_PROT_ICMP, IP_NO_UPPER_LAYER_CHECKSUM))
    {
        TCPIP_IP_FreePacket (ptrPacket);
        return NULL;
    }

    return ptrPacket;
}

#if defined (TCPIP_STACK_USE_ICMP_CLIENT)
void ICMPSendEchoRequest (NODE_INFO * remoteNode, uint16_t sequenceNumber, uint16_t identifier)
{
    uint32_t data = 0x44332211;
    ICMP_HEADER * pICMPHeader;
    int netIx;
    NET_CONFIG * pNetIf = TCPIP_STACK_GetDefaultNet();
    IP_PACKET * pTxPkt;

    netIx = _TCPIPStackNetIx (pNetIf);
    pTxPkt = icmpTxPkt[netIx];

    if(!TCPIP_IP_IsTxReady(pNetIf))
    {
        return;
    }

    if (TCPIP_IP_IsTxPutReady(pTxPkt, 4) < 4)
        return;

    pICMPHeader = TCPIP_IP_GetUpperLayerHeaderPtr (pTxPkt);

    pICMPHeader->vType = 0x08;  // 0x08: Echo (ping) request
    pICMPHeader->vCode = 0x00;
    pICMPHeader->wChecksum = 0x0000;
    pICMPHeader->wIdentifier = swaps(identifier);
    pICMPHeader->wSequenceNumber = swaps(sequenceNumber);

    TCPIP_IP_PutArray(pTxPkt, (uint8_t *)&data, 4);

    TCPIP_IPV4_SetDestAddress (pTxPkt, remoteNode->IPAddr.Val);

    pICMPHeader->wChecksum = TCPIP_IP_CalculatePayloadChecksum (pTxPkt);

	TCPIP_IP_PutHeader(pTxPkt, IP_PROT_ICMP);

	TCPIP_IP_Flush(pTxPkt, &remoteNode->MACAddr);

    if (TCPIP_IP_IsPacketQueued(pTxPkt))
    {
        icmpTxPkt[netIx] = ICMPAllocateTxPacketStruct (pNetIf);
    }
    else
        TCPIP_IP_ResetTransmitPacketState (pTxPkt);
}

#endif

/*********************************************************************
 * Function:        void ICMPProcess(NET_CONFIG* pNetIf)
 *
 * PreCondition:    MAC buffer contains ICMP type packet.
 *
 * Input:           hMac: interfaceon which the request was received
 *                  *remote: Pointer to a NODE_INFO structure of the 
 *					ping requester
 *					len: Count of how many bytes the ping header and 
 *					payload are in this IP packet
 *
 * Output:          Generates an echo reply, if requested
 *					Validates and sets ICMPFlags.bReplyValid if a 
 *					correct ping response to one of ours is received.
 *
 * Side Effects:    None
 *
 * Overview:        None
 *
 * Note:            None
 ********************************************************************/
void ICMPProcess(NET_CONFIG* pNetIf, NODE_INFO *remote, uint16_t len)
{
	TCPIP_UINT32_VAL dwVal;
    TCPIP_MAC_HANDLE hMac;
    ICMP_HEADER * headerPtr;
    int          netIx;     
    IP_PACKET *pTxPkt;
    
    hMac = _TCPIPStackNetToMac(pNetIf);
    netIx = _TCPIPStackNetIx(pNetIf);
    
    // Obtain the ICMP header Type, Code, and Checksum fields
    TCPIP_IP_GetArray(hMac, (uint8_t*)&dwVal, sizeof(dwVal));
	
	// See if this is an ICMP echo (ping) request
	if(dwVal.w[0] == 0x0008u)
	{
		// Validate the checksum using the Microchip MAC's DMA module
		// The checksum data includes the precomputed checksum in the 
		// header, so a valid packet will always have a checksum of 
		// 0x0000 if the packet is not disturbed.
		if(TCPIP_IP_CalcRxChecksum(hMac, 0+sizeof(IP_HEADER), len))
			return;

        pTxPkt = icmpTxPkt[netIx];

		if (pTxPkt == NULL)
		{
			TCPIP_IP_DiscardRx(hMac);
			return;
		}


        headerPtr = (ICMP_HEADER *)TCPIP_IP_GetUpperLayerHeaderPtr(pTxPkt);
	
		// Calculate new Type, Code, and Checksum values
		headerPtr->vType = 0x00;	// Type: 0 (ICMP echo/ping reply)
        headerPtr->vCode = dwVal.v[1];
		dwVal.v[2] += 8;	// Subtract 0x0800 from the checksum
		if(dwVal.v[2] < 8u)
		{
			dwVal.v[3]++;
			if(dwVal.v[3] == 0u)
				dwVal.v[2]++;
		}

        headerPtr->wChecksum = dwVal.w[1];

        TCPIP_IP_GetArray (hMac, (uint8_t *)&headerPtr->wIdentifier, 4);
	
	    // Wait for TX hardware to become available (finish transmitting 
	    // any previous packet)
	    if(!TCPIP_IP_IsTxReady(pNetIf))
        {
            return;
        }
        	
		// Create IP header in TX memory
		TCPIP_IP_PutHeader(pTxPkt, IP_PROT_ICMP);

        TCPIP_IPV4_SetDestAddress (pTxPkt, remote->IPAddr.Val);

        if (TCPIP_IP_IsTxPutReady(pTxPkt, len - sizeof(ICMP_HEADER)) < len - sizeof(ICMP_HEADER))
        {
            TCPIP_IP_DiscardRx (hMac);
            TCPIP_IP_ResetTransmitPacketState (pTxPkt);
            return;
        }

		// Copy ICMP response into the TX memory
        TCPIP_IP_PutRxData (hMac, pTxPkt, len - 8);
	
		// Transmit the echo reply packet
	    TCPIP_IP_Flush(pTxPkt, &remote->MACAddr);

        if (TCPIP_IP_IsPacketQueued(pTxPkt))
        {
            //pTxPkt = ICMPInit();
        }
        else
            TCPIP_IP_ResetTransmitPacketState (pTxPkt);
	}
#if defined(TCPIP_STACK_USE_ICMP_CLIENT)
	else if(dwVal.w[0] == 0x0000u)	// See if this an ICMP Echo reply to our request
	{
		// Get the sequence number and identifier fields
		TCPIP_IP_GetArray(hMac, (uint8_t*)&dwVal, sizeof(dwVal));

		// Validate the ICMP checksum field
	    TCPIP_IPV4_SetRxBuffer(pNetIf, 0);
		if(MACCalcIPBufferChecksum(hMac, sizeof(ICMP_PACKET)))	// Two bytes of payload were sent in the echo request
			return;

		// Send a message to the application-level Ping driver that we've received an Echo Reply
        if (icmpCallback != NULL)
            (*icmpCallback) (&remote->IPAddr, (void *)&dwVal);
	}
#endif
}

#if defined(TCPIP_STACK_USE_ICMP_CLIENT)
/*********************************************************************
 * Function:        bool ICMPBeginUsage(void)
 *
 * PreCondition:    None
 *
 * Input:           pNet - the netwoork to use, on multi-homed hosts
 *
 * Output:          true: You have successfully gained ownership of 
 *						  the ICMP client module and can now use the 
 *						  ICMPSendPing() and ICMPGetReply() functions.
 *					false: Some other application is using the ICMP 
 *						   client module.  Calling ICMPSendPing() 
 *						   will corrupt the other application's ping 
 *						   result.
 *
 * Side Effects:    None
 *
 * Overview:        Claims ownership of the ICMP module.
 *
 * Note:            None
 ********************************************************************/
bool ICMPBeginUsage(NET_CONFIG* pNet)
{
    int netIx;

	if(bICMPInUse)
		return false;

    netIx = _TCPIPStackNetIx (pNet);

    if (icmpTxPkt[netIx] == NULL)
    {
        if ((icmpTxPkt[netIx] = ICMPAllocateTxPacketStruct(pNet)) == NULL)
            return false;
    }

    icmpTxPkt[netIx]->netIf = pNet;

	bICMPInUse = true;

	return true;
}


/*********************************************************************
 * Function:        void ICMPEndUsage(void)
 *
 * PreCondition:    ICMPBeginUsage() was called by you and it 
 *					returned true.
 *
 * Input:           None
 *
 * Output:          Your ownership of the ICMP module is released.  
 *					You can no longer use ICMPSendPing().
 *
 * Side Effects:    None
 *
 * Overview:        Gives up ownership of the ICMP module.
 *
 * Note:            None
 ********************************************************************/
void ICMPEndUsage(void)
{
	bICMPInUse = false;
}

#endif //#if defined(TCPIP_STACK_USE_ICMP_CLIENT)

#endif //#if defined(TCPIP_STACK_USE_ICMP_SERVER) || defined(TCPIP_STACK_USE_ICMP_CLIENT)
