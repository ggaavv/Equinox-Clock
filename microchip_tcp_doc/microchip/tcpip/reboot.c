/*******************************************************************************
  Reboot Module

  Summary:
    Module for Microchip TCP/IP Stack
    
  Description:
    -Remotely resets the PIC
    -Reference: Internet Bootloader documentation
*******************************************************************************/

/*******************************************************************************
FileName:   Reboot.c
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

#define __REBOOT_C

#include "tcpip_private.h"
#include "tcpip_config_private.h"

#if defined(TCPIP_STACK_USE_REBOOT_SERVER)

extern NODE_INFO remoteNode;


static UDP_SOCKET	MySocket[TCPIP_NETWORK_INTERFACES] = {INVALID_UDP_SOCKET};


/*********************************************************************
 * Function:        void RebootTask(NET_CONFIG* pConfig)
 *
 * PreCondition:    Stack is initialized()
 *
 * Input:           pConfig   - interface 
 *
 * Output:          None
 *
 * Side Effects:    None
 *
 * Overview:        Checks for incomming traffic on port 69.  
 *					Resets the PIC if a 'R' is received.
 *
 * Note:            This module is primarily for use with the 
 *					Ethernet bootloader.  By resetting, the Ethernet 
 *					bootloader can take control for a second and let
 *					a firmware upgrade take place.
 ********************************************************************/
void RebootTask(NET_CONFIG* pConfig)
{
	struct
	{
		uint8_t vMACAddress[6];
		uint32_t dwIPAddress;
		uint16_t wChecksum;
	} BootloaderAddress;
    int netIx;

    netIx = _TCPIPStackNetIx(pConfig);

	
	if(MySocket[netIx] == INVALID_UDP_SOCKET)
    {
        MySocket[netIx] = UDPOpen(0,UDP_OPEN_SERVER,REBOOT_PORT,INVALID_UDP_PORT);

        if(MySocket[netIx] == INVALID_UDP_SOCKET)
        {
            return;
        }
        UDPSocketSetNet(MySocket[netIx], pConfig);
    }

	// Do nothing if no data is waiting
	if(!UDPIsGetReady(MySocket[netIx]))
		return;
    
	#if defined(REBOOT_SAME_SUBNET_ONLY)
		// Respond only to name requests sent to us from nodes on the same subnet
     	if((remoteNode.IPAddr.Val & pConfig->MyMask.Val) != (pConfig->MyIPAddr.Val & pConfig->MyMask.Val))
		{
			UDPDiscard(pConfig);
			return;
		}
	#endif

	// Get our MAC address, IP address, and compute a checksum of them 
	memcpy((void*)&BootloaderAddress.vMACAddress[0], (void*)&pConfig->MyMACAddr.v[0], sizeof(pConfig->MyMACAddr));
	BootloaderAddress.dwIPAddress = pConfig->MyIPAddr.Val;
	BootloaderAddress.wChecksum = CalcIPChecksum((uint8_t*)&BootloaderAddress, sizeof(BootloaderAddress) - sizeof(BootloaderAddress.wChecksum));
	
	// To enter the bootloader, we reset the system
	SYS_OUT_MESSAGE("Bootloader Reset");

	SYS_Reboot();
}

#endif //#if defined(TCPIP_STACK_USE_REBOOT_SERVER)
