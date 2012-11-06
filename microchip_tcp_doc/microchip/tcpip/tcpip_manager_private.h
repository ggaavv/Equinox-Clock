/*******************************************************************************
  Microchip TCP/IP Stack Definitions

  Summary:
    
  Description:
*******************************************************************************/

/*******************************************************************************
FileName:  tcpip_manager_private.h 
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

#ifndef _TCPIP_MANAGER_PRIVATE_H_
#define _TCPIP_MANAGER_PRIVATE_H_

#include "tcpip/tcpip_mac.h"



#if defined (TCPIP_IF_MRF24W)
    #include "wf_config.h" // pull in additional defines from wireless settings
#endif

// supported network interface power mode state for init/re-init functions
typedef enum
{
    TCPIP_STACK_POWER_NONE,     // unknown power mode; 
    TCPIP_STACK_POWER_FULL,     // up and running; valid for init/re-init
    TCPIP_STACK_POWER_LOW,      // low power mode; valid for init/re-init
    TCPIP_STACK_POWER_DOWN,     // interface is down; 
}TCPIP_STACK_POWER_MODE;



#include "tcpip/tcpip_helpers.h"
#include "tcpip_helpers_private.h"

// Application-dependent structure used to contain address information
typedef struct __attribute__((__packed__)) 
{
	IP_ADDR		MyIPAddr;               // IP address; currently only one IP add per interface
	IP_ADDR		MyMask;                 // Subnet mask
	IP_ADDR		MyGateway;              // Default Gateway
	IP_ADDR		PrimaryDNSServer;       // Primary DNS Server
	IP_ADDR		SecondaryDNSServer;     // Secondary DNS Server
	IP_ADDR		DefaultIPAddr;          // Default IP address
	IP_ADDR		DefaultMask;            // Default subnet mask
	uint8_t		NetBIOSName[16];        // NetBIOS name
	union
    {
        struct
        {
            unsigned char bInterfaceEnabled : 1; // 0 when TCPIP_STACK_POWER_DOWN/TCPIP_STACK_POWER_LOW 
            unsigned char : 2;
            unsigned char bIPv6InConfig : 1;
            unsigned char bIPv6Enabled : 1;
            unsigned char bWFEasyConfig : 1;  // NOTE : This was not present in 5.36
            unsigned char bIsDHCPEnabled : 1;
            unsigned char bInConfigMode : 1;
        };
        unsigned char v;
    }
    Flags;                            // Flag structure
	MAC_ADDR	MyMACAddr;            // Application MAC address

#if defined(TCPIP_IF_MRF24W)
	uint8_t		   MySSID[32];             // Wireless SSID (if using MRF24W)
	uint8_t        SsidLength;             // number of bytes in SSID
	uint8_t        SecurityMode;           // WF_SECURITY_OPEN or one of the other security modes
	uint8_t        SecurityKey[64];        // WiFi Security key, or passphrase.   
	uint8_t        SecurityKeyLength;      // number of bytes in security key (can be 0)
	uint8_t        WepKeyIndex;            // WEP key index (only valid for WEP)
    #if defined(EZ_CONFIG_STORE) // WLAN configuration data stored to NVM
    uint8_t        dataValid;
    uint8_t        networkType;
    uint8_t        saveSecurityInfo;       // Save 32-byte PSK
    #endif
#endif
	
#if defined(TCPIP_STACK_USE_SNMP_SERVER) || defined(TCPIP_STACK_USE_SNMPV3_SERVER)
	// SNMPv2C Read community names
	// SNMP_COMMUNITY_MAX_LEN (8) + 1 null termination byte
	uint8_t readCommunity[SNMP_MAX_COMMUNITY_SUPPORT][SNMP_COMMUNITY_MAX_LEN+1]; 

	// SNMPv2C Write community names
	// SNMP_COMMUNITY_MAX_LEN (8) + 1 null termination byte
	uint8_t writeCommunity[SNMP_MAX_COMMUNITY_SUPPORT][SNMP_COMMUNITY_MAX_LEN+1];

	uint32_t SnmpEngineBootRcrd;
#endif
    TCPIP_MAC_ID      macId;        // corresponding MAC id
    // starting here is dynamic stuff, that can change at run time
    // this info is NOT saved or restored to/from external configuration storage!
    uint16_t          netIfIx;      // index of this entry in the NetConfig table.
                                    // netIfIx = (this - &NetConfig[0])/sizeof(NET_CONFIG)
                                    // stored to avoid the run time division
    uint8_t           powerMode;    // current interface power mode; a TCPIP_STACK_POWER_MODE value;  
    uint8_t           linkPrev;     // previous status of the link
    TCPIP_MAC_HANDLE __attribute__((__aligned__))    hIfMac;          // quick reference to which MAC this interface belongs to

} NET_CONFIG;


// network interface action for initialization/de-initialization
typedef enum
{
    TCPIP_STACK_ACTION_INIT,         // stack is initialized 
    TCPIP_STACK_ACTION_REINIT,       // stack is re-initialized
    TCPIP_STACK_ACTION_DEINIT,       // stack is de-initialized
    TCPIP_STACK_ACTION_IF_UP,        // interface is brought up 
    TCPIP_STACK_ACTION_IF_DOWN,      // interface is brought down
}TCPIP_STACK_ACTION;


// data that's passed as reference to each other module init/deinit, etc
typedef struct _TCPIP_STACK_MODULE_CTRL
{
    // permanent data; this data is maintained by the stack for one full session
    // i.e. accross StackInit() -> StackDeInit() calls
    // 
    //
    // number of the interfaces supported in this seession
    int     nIfs;           
    // allocation parameters
    const void* memH;                                                   // handle to be used in the calls
    void* (*mallocCallback)( const void* h, size_t size);               // malloc style function
    void* (*callocCallback)( const void* h, size_t nitems, size_t size);//calloc style function
    void (*freeCallback)( const void* h, void* ptr);                    // free function

    // transient data; contains info for a specific call
    //
    //
    // pointer to the current interface
    NET_CONFIG* pNetIf;
    // index of the current interface addressed
    int     netIx;
    // current action for the stack
    TCPIP_STACK_ACTION      stackAction;
    // the power mode for this interface to go to
    // valid only if stackAction == init/re-init; ignored for de-init
    TCPIP_STACK_POWER_MODE  powerMode;
    
}TCPIP_STACK_MODULE_CTRL;




NET_CONFIG*         _TCPIPStackIpAddToNet(IP_ADDR* pIpAddress, bool useDefault);

NET_CONFIG*         _TCPIPStackIpAddFromNet(IP_ADDR* pIpAddress);

// clear the dynamic part of the NET_CONFIG
void  _TCPIPStackClearDynNetSettings(NET_CONFIG* pNetIf);

int  _TCPIPStackNetIx(NET_CONFIG* pNetIf);


uint32_t  _TCPIPStackNetAddress(NET_CONFIG* pNetIf);

void  _TCPIPStackSetNetAddress(NET_CONFIG* pNetIf, IP_ADDR* ipAddress);
void  _TCPIPStackSetDefaultAddress(NET_CONFIG* pNetIf);



uint32_t  _TCPIPStackNetMask(NET_CONFIG* pNetIf);

void  _TCPIPStackSetNetMask(NET_CONFIG* pNetIf, IP_ADDR* mask);

void  _TCPIPStackSetGatewayAddress(NET_CONFIG* pNetIf, IP_ADDR* ipAddress);
void  _TCPIPStackSetPriDNSAddress(NET_CONFIG* pNetIf, IP_ADDR* ipAddress);

void  _TCPIPStackSetSecondDNSAddress(NET_CONFIG* pNetIf, IP_ADDR* ipAddress);

NET_CONFIG*         _TCPIPStackNetByAddress(IP_ADDR* pIpAddress);


bool  _TCPIPStackIsAddressOfNet( NET_CONFIG* pNetIf, IP_ADDR* pIpAdd);

// detects net-directed bcast
bool  _TCPIPStackIsNetBcastAddress(NET_CONFIG* pNetIf, IP_ADDR* pIpAdd);

// detects limited or net-directed bcast
bool  _TCPIPStackIsBcastAddress(NET_CONFIG* pNetIf, IP_ADDR* pIpAdd);

bool  _TCPIPStackIsNetUp(NET_CONFIG* pNetIf);


NET_CONFIG*         _TCPIPStackMacIdToNet(TCPIP_MAC_ID macId);

TCPIP_MAC_ID  _TCPIPStackNetMacId(NET_CONFIG* pNetIf);

NET_CONFIG*     _TCPIPStackMacToNet(TCPIP_MAC_HANDLE hMac);

TCPIP_MAC_HANDLE  _TCPIPStackNetToMac(NET_CONFIG* pNetIf);

int         _TCPIPStackMacToNetIx(TCPIP_MAC_HANDLE hMac);



const uint8_t*  _TCPIPStackNetMacAddress(NET_CONFIG* pNetIf);

NET_CONFIG*  _TCPIPStackHandleToNet(TCPIP_NET_HANDLE hNet);


#endif  // _TCPIP_MANAGER_PRIVATE_H_


