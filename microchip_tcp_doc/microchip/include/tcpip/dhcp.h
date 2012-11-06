/*******************************************************************************
  DHCP API for Microchip TCP/IP Stack

  Summary:
    
  Description:
*******************************************************************************/

/*******************************************************************************
FileName:  dhcp.h 
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


#ifndef __DHCP_H
#define __DHCP_H


typedef enum
{
    DHCP_EVENT_DISCOVER = 1,       // DHCP cycle started
    DHCP_EVENT_BOUND,              // DHCP lease obtained
    DHCP_EVENT_LEASE_EXPIRED,      // lease expired
}DHCP_EVENT_TYPE;

// prototype of a DHCP event handler
// clients can register a handler with the DHCP service
// Once an DHCP event occurs the DHCP service will called the registered handler
// The handler has to be short and fast.
// It is meant for setting an event flag, not for lengthy processing!
typedef void    (*DHCP_EVENT_HANDLER)(TCPIP_NET_HANDLE hNet, DHCP_EVENT_TYPE evType, const void* param);

// a handle that a client can use
// after the event handler has been registered
typedef const void* DHCP_HANDLE;

bool DHCPEnable(TCPIP_NET_HANDLE hNet);
bool DHCPDisable(TCPIP_NET_HANDLE hNet, bool keepLease);
bool DHCPIsEnabled(TCPIP_NET_HANDLE hNet);
bool DHCPIsBound(TCPIP_NET_HANDLE hNet);
bool DHCPIsServerDetected(TCPIP_NET_HANDLE hNet);


// Register an DHCP event handler
// Use hNet == 0 to register on all interfaces available
// Returns a valid handle if the call succeeds,
// or a null handle if the call failed.
// Function has to be called after the DHCP is initialized
// The hParam is passed by the client and will be used by the DHCP when the notification is made.
// It is used for per-thread content or if more modules, for example, share the same handler
// and need a way to differentiate the callback.
DHCP_HANDLE      DHCPRegisterHandler(TCPIP_NET_HANDLE hNet, DHCP_EVENT_HANDLER handler, const void* hParam);

// deregister the event handler
// returns true or false if no such handler registered
bool             DHCPDeRegisterHandler(DHCP_HANDLE hDhcp);

#endif  // __DHCP_H


