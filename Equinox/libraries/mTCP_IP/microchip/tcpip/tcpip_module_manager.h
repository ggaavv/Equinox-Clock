/*******************************************************************************
  TCPIP modules manager file

  Summary:
    Internal TCPIP stack module manager file 
    
  Description:
    This header file contains the function prototypes and definitions of the 
    TCPIP stack manager services
*******************************************************************************/

/*******************************************************************************
FileName:   tcpip_module_manager.h
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

#ifndef __TCPIP_MODULE_MANAGER_H_
#define __TCPIP_MODULE_MANAGER_H_

#include <stdlib.h>
#include <stdint.h>
#include <stdbool.h>


// definitions
// 


// list of the supported modules
// 



// function that is called by the stack manager to detect 
// if the corresponding module has some asynchronous event
// pending. It should return true if an asynchronous
// event is pending, false otherwise.
// This could be a timer event created by the module
// or other specific module events
typedef bool    (*tcpipModuleAsyncPending)(void);

// asynchronous event handler
// the stack calls it when there's an event pending
// it should clear the pending status
typedef void    (*tcpipModuleAsyncHandler)(void);

// initialization function
// if the module has initialization to do, this function
// will be called. It should return a result to indicate
// if the initialization was successful. If not, the
// interface will not be completed.
typedef bool	(*tcpipModuleInitFunc)(
		const TCPIP_STACK_MODULE_CTRL* const, 
		const void* const);
		
// de-initialization function
// if the module needs to clean up when the module is
// brought down, this function will be called. It should
// return a result to indicate that everything has been
// cleaned up.
typedef void	(*tcpipModuleDeInitFunc)(const TCPIP_STACK_MODULE_CTRL * const);

// descriptor of an TCPIP stack entry (module that's part of the stack)
// 
typedef struct
{
    TCPIP_STACK_MODULE          moduleId;           // module identification
    tcpipModuleInitFunc		    initFunc;           // initialization function
    tcpipModuleDeInitFunc       deInitFunc;         // de-initialization function
    tcpipModuleAsyncPending     asyncPending;       // returns true if attention needed
    tcpipModuleAsyncHandler     asyncHandler;       // attention handler
}TCPIP_STACK_MODULE_ENTRY;


// table with TCPIP stack modules
// We use functions pointers rather than variable pointers so that we can create this table in const
// Also, the name of the functions is fixed rather than providing registration functions
//



static const TCPIP_STACK_MODULE_ENTRY  TCPIP_STACK_MODULE_ENTRY_TBL [] =
{
    //ModuleID                  //InitFunc                                      //DeInitFunc                // asyncPending       // asyncHandler
    {TCPIP_MODULE_IP,           (tcpipModuleInitFunc)TCPIP_IP_Initialize,       TCPIP_IP_DeInitialize,		0,                          0},                     //         TCPIP_MODULE_IP,
#if defined(TCPIP_STACK_USE_ICMP_CLIENT) || defined(TCPIP_STACK_USE_ICMP_SERVER)		
    {TCPIP_MODULE_ICMP,         (tcpipModuleInitFunc)ICMPInitialize,            ICMPDeinitialize,           0,                          0},                     //         TCPIP_MODULE_ICMP,      
#endif
    {TCPIP_MODULE_ARP,          (tcpipModuleInitFunc)ARPInitialize,             ARPDeinitialize,            ARPTaskPending,             ARPTask},               //         TCPIP_MODULE_ARP,
#if defined(TCPIP_STACK_USE_IPV6)
	{TCPIP_MODULE_IPV6,			(tcpipModuleInitFunc)TCPIP_IPV6_Initialize,     TCPIP_IPV6_Deinitialize,    TCPIP_IPV6_InitTaskPending, TCPIP_IPV6_InitializeTask},
	{TCPIP_MODULE_IPV6,			0,                                              0,                          TCPIP_IP_TaskPending,       TCPIP_IP_Task},
    {TCPIP_MODULE_ICMPV6,		(tcpipModuleInitFunc)TCPIP_ICMPV6_Initialize,   TCPIP_ICMPV6_Deinitialize,  0,                          0},
    {TCPIP_MODULE_NDP,          (tcpipModuleInitFunc)TCPIP_NDP_Initialize,      TCPIP_NDP_Deinitialize,     0,                          0},
#endif
#if defined(TCPIP_STACK_USE_UDP)
    {TCPIP_MODULE_UDP,           (tcpipModuleInitFunc)UDPInit,                  UDPDeInit,                  0,                          0},                     //         TCPIP_MODULE_UDP,
#endif
#if defined(TCPIP_STACK_USE_TCP)
    {TCPIP_MODULE_TCP,           (tcpipModuleInitFunc)TCPInit,                  TCPDeInit,                  TCPTaskPending,             TCPTick},               //         TCPIP_MODULE_TCP,
#endif    
#if defined(TCPIP_STACK_USE_DHCP_CLIENT)
    {TCPIP_MODULE_DHCP_CLIENT,   (tcpipModuleInitFunc)DHCPInit,                 DHCPDeInit,                 0,                          0},                     //         TCPIP_MODULE_DHCP_CLIENT,
#endif
#if defined(TCPIP_STACK_USE_DHCP_SERVER)
    {TCPIP_MODULE_DHCP_SERVER,   0,                                             0,                          0,                          0},                     //         TCPIP_MODULE_DHCP_SERVER,
#if defined(TCPIP_STACK_USE_AUTOIP)
	{TCPIP_MODULE_AUTOIP,		  0,                            			    0,                          0,                          0},
#endif
#endif
#if defined(TCPIP_STACK_USE_ANNOUNCE)
    {TCPIP_MODULE_ANNOUNCE,     (tcpipModuleInitFunc)ANNOUNCE_Init,             ANNOUNCE_DeInit,            ANNOUNCE_TaskPending,       ANNOUNCE_Send},         //         TCPIP_MODULE_ANNOUNCE,
#endif
#if defined(TCPIP_STACK_USE_DNS)
#if DNS_CLIENT_VERSION_NO >= 2
    {TCPIP_MODULE_DNS_CLIENT,    (tcpipModuleInitFunc)DNSClientInit,            DNSClientDeInit,            DNSClientTaskPending,       DNSClientTask},         //         TCPIP_MODULE_DNS_CLIENT,
#else
    {TCPIP_MODULE_DNS_CLIENT,    (tcpipModuleInitFunc)DNSClientInit,            DNSClientDeInit,            0,                          0},
#endif // DNS_CLIENT_VERSION_NO >= 2
#if defined(TCPIP_STACK_USE_NBNS)
	{TCPIP_MODULE_NBNS,			 (tcpipModuleInitFunc)NBNSInit,			        NBNSDeInit,	                0,                          0},                     // TCPIP_MODULE_NBNS
#endif
#endif
#if defined(TCPIP_STACK_USE_SNTP_CLIENT)
    {TCPIP_MODULE_SNTP,      (tcpipModuleInitFunc)SNTPInit,                     SNTPDeInit,                 0,                          0},                     //         TCPIP_MODULE_SNTP,
#endif
#if defined(TCPIP_STACK_USE_BERKELEY_API)
    {TCPIP_MODULE_BERKELEY,      (tcpipModuleInitFunc)BerkeleySocketInit,       BerkeleySocketDeInit,       0,                          0},                     //         TCPIP_MODULE_BERKELEY,
#endif
#if defined(TCPIP_STACK_USE_HTTP2_SERVER)
    {TCPIP_MODULE_HTTP_SERVER,   (tcpipModuleInitFunc)HTTPInit,                 HTTPDeInit,                 0,                          0},                     //         TCPIP_MODULE_HTTP_SERVER,
#endif
#if defined(TCPIP_STACK_USE_RSA)
    {TCPIP_MODULE_RSA,           (tcpipModuleInitFunc)RSAInit,                  RSADeInit,                  0,                          0},                     //         TCPIP_MODULE_RSA,
#endif
#if defined(TCPIP_STACK_USE_SSL_SERVER) || defined(TCPIP_STACK_USE_SSL_CLIENT)
    {TCPIP_MODULE_SSL,          (tcpipModuleInitFunc)SSLInit,                   SSLDeInit,                  0,                          0},                     //         TCPIP_MODULE_SSL_SERVER,
#endif
#if defined(TCPIP_STACK_USE_FTP_SERVER) && defined(TCPIP_STACK_USE_MPFS2)
    {TCPIP_MODULE_FTP_SERVER,    (tcpipModuleInitFunc)FTPInit,                  FTPDeInit,                  0,                          0},                     //         TCPIP_MODULE_FTP_SERVER,
#endif
#if defined(TCPIP_STACK_USE_SNMP_SERVER)
    {TCPIP_MODULE_SNMP_SERVER,   (tcpipModuleInitFunc)SNMPInit,                 SNMPDeInit,                 0,                          0},                     //         TCPIP_MODULE_SNMP_SERVER,
#endif
#if defined(TCPIP_STACK_USE_DYNAMICDNS_CLIENT)
    {TCPIP_MODULE_DYNDNS_CLIENT, (tcpipModuleInitFunc)DDNSInit,                 DDNSDeInit,                 0,                          0},                     //         TCPIP_MODULE_DYNDNS_CLIENT,
#endif
        // Add other needed services here
     
};



// Connection event handler definition.
// The stack calls the handler when a new connection event occurs.
// Note that this call will carry only connection events!
typedef void    (*tcpipModuleConnHandler)(NET_CONFIG* pNetIf, TCPIP_EVENT connEvent);



// Since the modules that need connection notification is
// known to the stack manager no dynamic approach is taken.
// But simply a call table is maintained.
static const tcpipModuleConnHandler  TCPIP_STACK_CONN_EVENT_TBL [] =
{
#if defined(TCPIP_STACK_USE_DHCP_CLIENT)
    DHCPConnectionHandler,
#endif // defined(TCPIP_STACK_USE_DHCP_CLIENT)

    // add other needed handlers here
};



#endif //  __TCPIP_MODULE_MANAGER_H_








