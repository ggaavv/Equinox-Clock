/*******************************************************************************
  NetBIOS Name Service (NBNS) Server

  Summary:
    Module for Microchip TCP/IP Stack
    
  Description:
    -Responds to NBNS name requests to allow human name assignment 
     to the board.  i.e. allows nodes on the same IP subnet to use a 
     hostname to access the board instead of an IP address.
    -Reference: RFC 1002
*******************************************************************************/

/*******************************************************************************
FileName:   NBNS.c
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

#define __NBNS_C

#include "tcpip_private.h"
#include "tcpip_config_private.h"

#if defined(TCPIP_STACK_USE_NBNS)

// NBNS Header structure
typedef struct _NBNS_HEADER
{
	TCPIP_UINT16_VAL TransactionID;
	TCPIP_UINT16_VAL Flags;
	TCPIP_UINT16_VAL Questions;
	TCPIP_UINT16_VAL Answers;
	TCPIP_UINT16_VAL AuthoritativeRecords;
	TCPIP_UINT16_VAL AdditionalRecords;
} NBNS_HEADER;

static void NBNSPutName(UDP_SOCKET s, uint8_t *String);
static void NBNSGetName(UDP_SOCKET s, uint8_t *String);

extern NODE_INFO remoteNode;


	static UDP_SOCKET	MySocket[TCPIP_NETWORK_INTERFACES];
	static enum
	{
		NBNS_HOME = 0,
		NBNS_OPEN_SOCKET,
		NBNS_LISTEN
	} smNBNS[TCPIP_NETWORK_INTERFACES] = {NBNS_HOME};

/*********************************************************************
 * Function:        void NBNSInit(const TCPIP_STACK_MODULE_CTRL* const stackInit, const NBNS_MODULE_GONFIG* pNbnsInit)
 *
 * PreCondition:    None
 *
 * Input:           stackInit - Interface and stack module data.
 *                  pNbnsInit - Module-specific information for NBNS.
 *
 * Output:          None
 *
 * Side Effects:    None
 *
 * Overview:        Initializes state machine
 *
 * Note:            None
 ********************************************************************/
bool NBNSInit(const TCPIP_STACK_MODULE_CTRL* const stackInit, const NBNS_MODULE_GONFIG* pNbnsInit)
{
    // if(stackInit->stackAction == TCPIP_STACK_ACTION_IF_UP)  // interface restart
    // if(stackInit->stackAction == TCPIP_STACK_ACTION_INIT)   // stack init

	smNBNS[stackInit->netIx] = NBNS_HOME;
	
	return true;
}

/*********************************************************************
 * Function:        void NBNSDeInit(const TCPIP_STACK_MODULE_CTRL* const stackInit)
 *
 * PreCondition:    None
 *
 * Input:           stackInit - Interface and stack module data.
 *
 * Output:          None
 *
 * Side Effects:    None
 *
 * Overview:        DeInitializes state machine
 *
 * Note:            None
 ********************************************************************/
void NBNSDeInit(const TCPIP_STACK_MODULE_CTRL* const stackInit)
{
    // if(stackInit->stackAction == TCPIP_STACK_ACTION_DEINIT) // stack shut down
    // if(stackInit->stackAction == TCPIP_STACK_ACTION_IF_DOWN) // interface down


	smNBNS[stackInit->netIx] = NBNS_HOME;
}

/*********************************************************************
 * Function:        void NBNSTask(NET_CONFIG* pConfig)
 *
 * PreCondition:    None
 *
 * Input:           pConfig   - interface 
 *
 * Output:          None
 *
 * Side Effects:    None
 *
 * Overview:        Sends responses to NetBIOS name requests
 *
 * Note:            None
 ********************************************************************/
void NBNSTask(NET_CONFIG* pConfig)
{
	uint8_t 			i;
	TCPIP_UINT16_VAL    Type, Class;
	NBNS_HEADER			NBNSHeader;
	uint8_t				NameString[16];
    int                 netIx;
    UDP_SOCKET          s;
   
    netIx = _TCPIPStackNetIx(pConfig);
    s = MySocket[netIx];
    
	switch(smNBNS[netIx])
	{
		case NBNS_HOME:
			smNBNS[netIx]++;
			break;

		case NBNS_OPEN_SOCKET:
			MySocket[netIx] = UDPOpen(0,UDP_OPEN_SERVER,NBNS_PORT,NBNS_PORT);
			if(MySocket[netIx] == INVALID_UDP_SOCKET)
				break;

            UDPSocketSetNet(MySocket[netIx], pConfig);
			smNBNS[netIx]++;

		case NBNS_LISTEN:
			if(!UDPIsGetReady(s))
            {
				break;
            }


			// Respond only to name requests sent to us from nodes on the same subnet
			// This prevents us from sending out the wrong IP address information if 
			// we haven't gotten a DHCP lease yet.
        	if((remoteNode.IPAddr.Val & pConfig->MyMask.Val) != (pConfig->MyIPAddr.Val & pConfig->MyMask.Val))
			{
				UDPDiscard(s);
				break;
			}

			// Retrieve the NBNS header and de-big-endian it
			UDPGet(s, &NBNSHeader.TransactionID.v[1]);
			UDPGet(s, &NBNSHeader.TransactionID.v[0]);
			UDPGet(s, &NBNSHeader.Flags.v[1]);
			UDPGet(s, &NBNSHeader.Flags.v[0]);
			UDPGet(s, &NBNSHeader.Questions.v[1]);
			UDPGet(s, &NBNSHeader.Questions.v[0]);
			UDPGet(s, &NBNSHeader.Answers.v[1]);
			UDPGet(s, &NBNSHeader.Answers.v[0]);
			UDPGet(s, &NBNSHeader.AuthoritativeRecords.v[1]);
			UDPGet(s, &NBNSHeader.AuthoritativeRecords.v[0]);
			UDPGet(s, &NBNSHeader.AdditionalRecords.v[1]);
			UDPGet(s, &NBNSHeader.AdditionalRecords.v[0]);

			// Remove all questions
			while(NBNSHeader.Questions.Val--)
			{
				NBNSGetName(s, NameString);
				UDPGet(s, &i);				// <??> Trailing character on string
				UDPGet(s, &Type.v[1]);		// Question type
				UDPGet(s, &Type.v[0]);
				UDPGet(s, &Class.v[1]);	// Question class
				UDPGet(s, &Class.v[0]);
				
				if(Type.Val == 0x0020u && Class.Val == 0x0001u && memcmp((void*)NameString, (void*)pConfig->NetBIOSName, sizeof(pConfig->NetBIOSName)) == 0)
				{
					if(UDPIsTxPutReady(s, 64))
                    {   
                        NBNSHeader.Flags.Val = 0x8400;

                        UDPPut(s, NBNSHeader.TransactionID.v[1]);
                        UDPPut(s, NBNSHeader.TransactionID.v[0]);
                        UDPPut(s, NBNSHeader.Flags.v[1]);
                        UDPPut(s, NBNSHeader.Flags.v[0]);
                        UDPPut(s, 0x00);	// 0x0000 Questions
                        UDPPut(s, 0x00);
                        UDPPut(s, 0x00);	// 0x0001 Answers
                        UDPPut(s, 0x01);
                        UDPPut(s, 0x00);	// 0x0000 Athoritative records
                        UDPPut(s, 0x00);
                        UDPPut(s, 0x00);	// 0x0000 Additional records
                        UDPPut(s, 0x00);

                        NBNSPutName(s, pConfig->NetBIOSName);
                        UDPPut(s, 0x00);	// 0x0020 Type: NetBIOS
                        UDPPut(s, 0x20);
                        UDPPut(s, 0x00);	// 0x0001 Class: Internet
                        UDPPut(s, 0x01);
                        UDPPut(s, 0x00);	// 0x00000000 Time To Live
                        UDPPut(s, 0x00);
                        UDPPut(s, 0x00);
                        UDPPut(s, 0x00);

                        UDPPut(s, 0x00);	// 0x0006 Data length
                        UDPPut(s, 0x06);	
                        UDPPut(s, 0x60);	// 0x6000 Flags: H-node, Unique
                        UDPPut(s, 0x00);
                        UDPPut(s, pConfig->MyIPAddr.v[0]);	// Put out IP address
                        UDPPut(s, pConfig->MyIPAddr.v[1]);
                        UDPPut(s, pConfig->MyIPAddr.v[2]);
                        UDPPut(s, pConfig->MyIPAddr.v[3]);

                        // Change the destination address to the unicast address of the last received packet
                        TCPIP_IPV4_SetDestAddress(UDPSocketDcpt[s].pTxPkt,remoteNode.IPAddr.Val);
                        memcpy((void*)&UDPSocketDcpt[s].pTxPkt->remoteMACAddr, (const void*)&remoteNode.MACAddr, sizeof(remoteNode.MACAddr));
                        UDPFlush(s);				
                    }
				}
			}
			
			UDPDiscard(s);

			break;
	}
}

/*********************************************************************
 * Function:        static void NBNSPutName (UDP_SOCKET s, uint8_t *String)
 *
 * PreCondition:    None
 *
 * Input:           String: The name to transmit
 *
 * Output:          None
 *
 * Side Effects:    None
 *
 * Overview:        Transmits the NetBIOS name across an open UDP
 *                  socket.
 *
 * Note:            None
 ********************************************************************/
static void NBNSPutName(UDP_SOCKET s, uint8_t *String)
{
	uint8_t i, j;

	UDPPut(s, 32);	// NetBIOS names are always 32 bytes long (16 decoded bytes)
	for(i = 0; i < 16u; i++)
	{
		j = *String++;
		UDPPut(s, (j>>4) + 'A');
		UDPPut(s, (j & 0x0F) + 'A');
	}
	
	UDPPut(s, 0x00);
}

/*********************************************************************
 * Function:        static void NBNSGetName (UDP_SOCKET s, uint8_t *String)
 *
 * PreCondition:    None
 *
 * Input:           String: Pointer to an array into which
 *                  a received NetBIOS name should be copied.
 *
 * Output:          None
 *
 * Side Effects:    None
 *
 * Overview:        Reads the NetBIOS name from a UDP socket and
 *                  copies it into a user-specified buffer.
 *
 * Note:            None
 ********************************************************************/
static void NBNSGetName(UDP_SOCKET s, uint8_t *String)
{
	uint8_t i, j, k;

	if(String == NULL)
	{
		UDPGet(s, &i);
		while(i--)
		{
			UDPGet(s, &j);
		}
	}
	else
	{
		UDPGet(s, &i);
		if(i != 32u)
		{
			*String = 0;
			return;
		}
		while(i--)
		{
			UDPGet(s, &j);
			j -= 'A';
			k = j<<4;
			i--;
			UDPGet(s, &j);
			j -= 'A';
			*String++ = k | j;
		}
	}
}


#endif //#if defined(TCPIP_STACK_USE_NBNS)
