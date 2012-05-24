/*******************************************************************************
  Microchip TCP/IP Stack Definitions

  Summary:
    
  Description:
*******************************************************************************/

/*******************************************************************************
FileName:  tcpip_manager.h 
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

#ifndef __TCPIP_MANAGER_H_
#define __TCPIP_MANAGER_H_

//#include "tcpip/tcpip_mac.h"

// a network interface handle
typedef const void* TCPIP_NET_HANDLE;

/*********************************************************************
 * Function:        bool TCPIP_STACK_Init(const TCPIP_NETWORK_CONFIG* pNetConf, int nNets, TCPIP_STACK_MODULE_CONFIG* pModConfig, int nModules)
 *
 * PreCondition:    None 
 *
 * Input:           pNetConf  	- pointer to an array of TCPIP_NETWORK_CONFIG to support
 *                  nNets       - number of network configurations in the array
 *                  pModConfig  - pointer to an array of TCPIP_STACK_MODULE_CONFIG
 *                  nModules    - number of modules to initialize 
 *
 * Output:          true if Stack and its componets are initialized
 *                  false otherwise
 *
 * Overview:        The function initializes the stack.
 *                  If an error occurs, the SYS_ERROR() is called
 *                  and the function de-initialize itself and will return false.
 *                  
 * Side Effects:    None
 *
 * Note:            This function must be called before any of the
 *                  stack or its component routines are used.
 *
 *                  New TCPIP_NETWORK_CONFIG types should be added/removed at run time for implementations that support
 *                  dynamic network interface creation.
 ********************************************************************/
bool                TCPIP_STACK_Init(const TCPIP_NETWORK_CONFIG* pNetConf, int nNets, const TCPIP_STACK_MODULE_CONFIG* pModConfig, int nModules);

/*********************************************************************
 * Function:        void TCPIP_STACK_DeInit(void)
 *
 * PreCondition:    None
 *
 * Input:           None
 *
 * Output:          None
 *
 * Side Effects:    None
 *
 * Overview:        This function performs the de-initialization of the TCPIP stack
 *
 * Note:            None
 ********************************************************************/
void                TCPIP_STACK_DeInit(void);

/*********************************************************************
 * Function:        void TCPIP_STACK_Task(void)
 *
 * PreCondition:    TCPIP_STACK_Init() is already called.
 *
 * Input:           None
 *
 * Output:          Stack Finite-state Machine (FSM) is executed.
 *
 * Side Effects:    None
 *
 * Note:            This FSM checks for new incoming packets,
 *                  and routes it to appropriate stack components.
 *                  It also performs timed operations.
 *
 *                  This function must be called periodically to
 *                  ensure timely responses.
 ********************************************************************/
void                TCPIP_STACK_Task(void);

//interface access functions

/*********************************************************************
 * Function:        int TCPIP_STACK_NetworksNo(void)
 *
 * PreCondition:    TCPIP stack should have been initialized by TCPIP_STACK_Init()
 *
 * Input:           None
 *
 * Output:          Number of network interfaces
 *
 * Side Effects:    None
 *
 * Note:            None
 ********************************************************************/
int                 TCPIP_STACK_NetworksNo(void);

/*********************************************************************
 * Function:        TCPIP_NET_HANDLE TCPIP_STACK_NetHandle(const char* interface)
 *
 * PreCondition:    TCPIP stack should have been initialized by TCPIP_STACK_Init()
 *
 * Input:           interface - The names specified in tcpip_config.h::TCPIP_NETWORK_CONFIG. 
 *
 * Output:          Resolves an interface name to a handle.
 *               
 * Side Effects:    None
 *
 * Example:			TCPIP_NET_HANDLE hNet = TCPIP_STACK_NetHandle("PIC32INT");
 *
 * Note:            None
 ********************************************************************/
TCPIP_NET_HANDLE    TCPIP_STACK_NetHandle(const char* interface);

/*********************************************************************
 * Function:        uint32_t TCPIP_STACK_NetAddress(TCPIP_NET_HANDLE netH)
 *
 * PreCondition:    TCPIP stack should have been initialized by TCPIP_STACK_Init()
 *
 * Input:           netH - Interface handle to get address of.
 *
 * Output:          if interface is enabled then it returns the IP address of that interface 
 * 					else return 0
 *               
 * Side Effects:    None
 *
 * Example:			TCPIP_NET_HANDLE netH = TCPIP_STACK_NetHandle("PIC32INT");
 * 					uint32_t ipAdd = TCPIP_STACK_NetAddress(netH);
 *
 * Note:            None
 ********************************************************************/
uint32_t            TCPIP_STACK_NetAddress(TCPIP_NET_HANDLE netH);


/*********************************************************************
 * Function:        uint32_t TCPIP_STACK_NetGatewayAddress(TCPIP_NET_HANDLE netH)
 *
 * PreCondition:    TCPIP stack should have been initialized by TCPIP_STACK_Init()
 *
 * Input:           netH - Interface handle to get address of.
 *
 * Output:          if interface is enabled then it returns the gateway address of that interface 
 * 					else return 0
 *               
 * Side Effects:    None
 *
 * Example:			TCPIP_NET_HANDLE netH = TCPIP_STACK_NetHandle("PIC32INT");
 * 					uint32_t ipAdd = TCPIP_STACK_NetGatewayAddress(netH);
 *
 * Note:            None
 ********************************************************************/
uint32_t            TCPIP_STACK_NetGatewayAddress(TCPIP_NET_HANDLE netH);


/*********************************************************************
 * Function:        uint32_t TCPIP_STACK_NetPriDNSAddress(TCPIP_NET_HANDLE netH)
 *
 * PreCondition:    TCPIP stack should have been initialized by TCPIP_STACK_Init()
 *
 * Input:           netH - Interface handle to get the DNS address of.
 *
 * Output:          the primary DNS address if succes
 *                  false if not such interface or interface is down
 *               
 * Side Effects:    None
 *
 * Example:			TCPIP_NET_HANDLE netH = TCPIP_STACK_NetHandle("PIC32INT");
 * 					myIPAddress = TCPIP_STACK_NetPriDNSAddress(netH);
 *
 * Note:            None
 ********************************************************************/
uint32_t            TCPIP_STACK_NetPriDNSAddress(TCPIP_NET_HANDLE netH);


/*********************************************************************
 * Function:        uint32_t TCPIP_STACK_NetSecondDNSAddress(TCPIP_NET_HANDLE netH)
 *
 * PreCondition:    TCPIP stack should have been initialized by TCPIP_STACK_Init()
 *
 * Input:           netH - Interface handle to get the DNS address of.
 *
 * Output:          the secondary DNS address if succes
 *                  false if not such interface or interface is down
 *               
 * Side Effects:    None
 *
 * Example:			TCPIP_NET_HANDLE netH = TCPIP_STACK_NetHandle("PIC32INT");
 * 					myIPAddress = TCPIP_STACK_NetSecondDNSAddress(netH);
 *
 * Note:            None
 ********************************************************************/
uint32_t            TCPIP_STACK_NetSecondDNSAddress(TCPIP_NET_HANDLE netH);


/*********************************************************************
 * Function:        uint32_t TCPIP_STACK_NetMask(TCPIP_NET_HANDLE netH)
 *
 * PreCondition:    None
 *
 * Input:           netH - Interface handle to get address of.
 *
 * Output:          if interface is enabled then it returns the IP address mask of that interface 
 * 					else return 0
 *
 * Example:         TCPIP_NET_HANDLE netH = TCPIP_STACK_NetHandle("PIC32INT");
 * 					uint32_t subMask = TCPIP_STACK_NetMask(netH);
 *
 * Note:            None
 ********************************************************************/
uint32_t            TCPIP_STACK_NetMask(TCPIP_NET_HANDLE netH);


/*********************************************************************
 * Function:        const char* TCPIP_STACK_NetBIOSName(TCPIP_NET_HANDLE netH)
 *
 * PreCondition:    None
 *
 * Input:           netH - Interface handle to get name of.
 *
 * Output:          if interface is enabled then it returns the NetBIOS name of that interface 
 * 					else return 0
 *
 * Example:         TCPIP_NET_HANDLE netH = TCPIP_STACK_NetHandle("PIC32INT");
 * 					const char* biosName = TCPIP_STACK_NetBIOSName(netH);
 *
 * Note:            None
 ********************************************************************/
const char*         TCPIP_STACK_NetBIOSName(TCPIP_NET_HANDLE netH);

/*********************************************************************
 * Function:        const MAC_ADDR* TCPIP_STACK_NetMacAddress(TCPIP_NET_HANDLE netH)
 *
 * PreCondition:    None
 *
 * Input:           netH - Interface handle to get the address of.
 *
 * Output:          if interface is enabled then it returns a constant pointer to the MAC address
 *                  of that interface 
 * 					else return 0
 *
 * Example:         TCPIP_NET_HANDLE netH = TCPIP_STACK_NetHandle("PIC32INT");
 * 					const MAC_ADDR* pAdd = TCPIP_STACK_NetMacAddress(netH);
 *
 * Note:            None
 ********************************************************************/
const uint8_t*     TCPIP_STACK_NetMacAddress(TCPIP_NET_HANDLE netH);


/*********************************************************************
 * Function:        uint32_t TCPIP_STACK_NetBcastAddress(TCPIP_NET_HANDLE netH)
 *
 * PreCondition:    TCPIP stack should have been initialized by TCPIP_STACK_Init()
 *
 * Input:           netH - Interface handle to get address of.
 *
 * Output:          if interface is enabled then it returns the broadcast IP address mask of that interface 
 * 					else return 0
 *
 * Side Effects:    None
 *
 * Note:	   	 	None
 ********************************************************************/
uint32_t            TCPIP_STACK_NetBcastAddress(TCPIP_NET_HANDLE netH);

/*********************************************************************
 * Function:        TCPIP_NET_HANDLE TCPIP_STACK_GetDefaultNet(void)
 *
 * PreCondition:    TCPIP stack should have been initialized by TCPIP_STACK_Init()
 *
 * Input:           None
 *
 * Output:          The default net interface for multi-homed hosts
 *
 * Side Effects:    None
 *
 * Note:            Function to dynamically change the default interface
 *                  will be added.
 ********************************************************************/
TCPIP_NET_HANDLE    TCPIP_STACK_GetDefaultNet(void);

/*********************************************************************
 * Function:        bool TCPIP_STACK_SetDefaultNet(TCPIP_NET_HANDLE netH)
 *
 * PreCondition:    None
 *
 * Input:           netH - Interface handle to get address of.
 *
 * Output:          true if success
 * 					false if failed (the old interface does not change)
 *
 * Side Effects:    None
 *
 * Note:            sets the default interface
 ********************************************************************/
bool                TCPIP_STACK_SetDefaultNet(TCPIP_NET_HANDLE netH);


/*********************************************************************
 * Function:        TCPIP_NET_HANDLE TCPIP_STACK_IxToNet(int netIx)
 *
 * PreCondition:    TCPIP stack should have been initialized by TCPIP_STACK_Init()
 *
 * Input:           netH - Interface handle to get address of.
 *
 * Output:          Resolves an index to an network handle.
 *               
 * Side Effects:    None
 *
 * Note:            None
 ********************************************************************/
TCPIP_NET_HANDLE    TCPIP_STACK_IxToNet(int netIx);

/*********************************************************************
 * Function:        int TCPIP_STACK_NetIx(TCPIP_NET_HANDLE hNet);
 *
 * PreCondition:    None
 *
 * Input:           hNet - Interface handle to get address of.
 *
 * Output:          Index of this entry in the stack network handles
 *
 * Side Effects:    None
 *
 * Note:            None
 ********************************************************************/
int                 TCPIP_STACK_NetIx(TCPIP_NET_HANDLE hNet);

/*********************************************************************
 * Function:        bool TCPIP_STACK_IsNetUp(TCPIP_NET_HANDLE hNet);
 *
 * PreCondition:    None
 *
 * Input:           hNet - Interface handle to get address of.
 *
 * Output:          true if interface exists and is enabled
 *                  false otherwise
 *
 * Side Effects:    None
 *
 * Note:            None
 ********************************************************************/
bool                TCPIP_STACK_IsNetUp(TCPIP_NET_HANDLE hNet);

/*********************************************************************
 * Function:        bool TCPIP_STACK_NetUp(TCPIP_NET_HANDLE netH, const TCPIP_STACK_MODULE_CONFIG* pModConfig, int nModules)
 *
 * PreCondition:    None
 *
 * Input:           netH 		- Interface handle to get address of.
 * 				 	pModConfig  - pointer to an array of TCPIP_STACK_MODULE_CONFIG
 *                  nModules    - number of modules to initialize
 *
 * Output:          true if success
 *                  false if no such network or an error occurred
 *
 * Side Effects:    None
 *
 * Overview:        This function performs the de-initialization of a net interface
 *
 * Note:            None
 ********************************************************************/
bool                TCPIP_STACK_NetUp(TCPIP_NET_HANDLE netH, const TCPIP_NETWORK_CONFIG* pUsrConfig);

/*********************************************************************
 * Function:        void TCPIP_STACK_NetDown(TCPIP_NET_HANDLE netH)
 *
 * PreCondition:    None
 *
 * Input:           netH - Interface handle to get address of.
 *
 * Output:          true if success
 *                  false if no such network
 *
 * Side Effects:    None
 *
 * Overview:        This function performs the de-initialization of a net interface
 *
 * Note:            None
 ********************************************************************/
bool                TCPIP_STACK_NetDown(TCPIP_NET_HANDLE netH);


/*********************************************************************
 * Function:        bool TCPIP_STACK_NetSetEnableDHCP(TCPIP_NET_HANDLE netH, bool enable);
 *
 * PreCondition:    None
 *
 * Input:           netH - Interface handle to set the DHCP on.
 *                  enable - if true, the DHCP is enabled
 *                           else the DHCP is disabled
 *
 * Output:          if interface exists then it enables/disables the DHCP on that interface 
 *                  and returns true
 * 					else return false
 *
 * Example:         TCPIP_NET_HANDLE netH = TCPIP_STACK_NetHandle("PIC32INT");
 * 					TCPIP_STACK_NetSetEnableDHCP(netH, true);
 *
 * Note:            None
 *                  
 ********************************************************************/
bool                TCPIP_STACK_NetSetEnableDHCP(TCPIP_NET_HANDLE netH, bool enable);


/*********************************************************************
 * Function:        bool TCPIP_STACK_NetIsDHCPEnabled(TCPIP_NET_HANDLE netH);
 *
 * PreCondition:    None
 *
 * Input:           netH - Interface handle to set the DHCP on.
 *
 * Output:          if interface exists and the DHCP service is enabled on that interface 
 *                  it returns true
 * 					else return false
 *
 * Example:         TCPIP_NET_HANDLE netH = TCPIP_STACK_NetHandle("PIC32INT");
 * 					bool isDhcpEnabled = TCPIP_STACK_NetIsDHCPEnabled(netH);
 *
 * Note:            None
 *                  
 ********************************************************************/
bool                TCPIP_STACK_NetIsDHCPEnabled(TCPIP_NET_HANDLE netH);

/*********** Set Parameter functions ******************/
/*
 * The main purpose for these functions is to allow changing the network configuration parameters
 * and save them for storage using the TCPIP storage services.
 * A network configuration is created by using TCPIP_STACK_CreateNetInfo().
 * Once such a configuration is created then various parameters can be changed using one of the following functions.
 * 
 * Important Note: these functions take a network handle as a parameter.
 * One should use extreme caution when using these functions with an active network interface handle.
 * Changing these parameters at runtime can lead to unexpected behavior
 * or loss of network connectivity.
 * The preferred way to change the parameters for a running interface is to do so 
 * as part of the network configuration passed at the stack initialization.
*/ 

/*********************************************************************
 * Function:        TCPIP_NET_HANDLE TCPIP_STACK_CreateNetInfo(TCPIP_NET_HANDLE hNet, uint8_t* buffer, int buffSize)
 *
 * PreCondition:    TCPIP stack should have been initialized by TCPIP_STACK_Init()
 *
 * Input:           hNet -  master interface handle to use
 *                  buffer - buffer to create network information into
 *                  buffSize    - size of the buffer
 *                  copyInfo    - if true, then the master interface info is copied to the created
 *                                network info
 *                                else the new network info is zeroed out  
 *
 * Description:     This function will create a replica of the hNet interface into the supplied user buffer.
 *                  The caller can then use the suppied buffer and handle to get/set corresponding network
 *                  parameters.
 *                  
 * Output:          A network handle that can be used to set/get network interface
 *                  parameters.
 *                  0 if the supplied buffer is not large enough to hold the interface associated info
 *                  (see TCPIP_STACK_NetInfoSize()) or the network interface does not exist 
 *               
 * Side Effects:    None
 *
 * Example:         TCPIP_NET_HANDLE netH = TCPIP_STACK_NetHandle("PIC32INT");
 *                  TCPIP_NET_HANDLE hNet = TCPIP_STACK_CreateNetInfo(netH, myNetInfo, sizeof(MyNetInfo));
 *
 * Note:            This is a helper function for initializing a network interface structure that 
 *                  can be later on saved to storage.
 *
 *                  The user of this function cannot make any assumption on the internal structure
 *                  and layout of the returned info.
 *                  This may not only change between stack versions but the stack itself
 *                  can maintain info that's not accessible and irrelevant to the user.
 ********************************************************************/
TCPIP_NET_HANDLE    TCPIP_STACK_CreateNetInfo(TCPIP_NET_HANDLE hNet, uint8_t* buffer, int buffSize, bool copyInfo);


/*********************************************************************
 * Function:        int TCPIP_STACK_NetInfoSize(TCPIP_NET_HANDLE hNet)
 *
 * PreCondition:    TCPIP stack should have been initialized by TCPIP_STACK_Init()
 *
 * Input:           hNet - handle identifying a network interface
 *
 * Output:          The size in bytes required to store a complete set of the corresponding network interface
 *                  parameters.
 *                  0 if the supplied interface does not exist;
 *               
 * Side Effects:    None
 *
 * Example:         TCPIP_NET_HANDLE netH = TCPIP_STACK_NetHandle("PIC32INT");
 *                  int netInfoSize = TCPIP_STACK_NetInfoSize(hNet);
 *
 * Note:            None
 ********************************************************************/
int                 TCPIP_STACK_NetInfoSize(TCPIP_NET_HANDLE hNet);


/*********************************************************************
 * Function:        bool TCPIP_STACK_SetNetAddress(TCPIP_NET_HANDLE netH, IP_ADDR* ipAddress, bool setDefault)
 *
 * PreCondition:    TCPIP stack should have been initialized by TCPIP_STACK_Init()
 *
 * Input:           netH - Interface handle to set address of.
 *                  ipAddress - IP address to set
 *                  setDefault  - if true, the interface default address is also set
 *
 * Output:          true if succes
 *                  false if not such interface
 *               
 * Side Effects:    None
 *
 * Example:			TCPIP_NET_HANDLE netH = TCPIP_STACK_NetHandle("PIC32INT");
 * 					TCPIP_STACK_SetNetAddress(netH, &myIPAddress);
 *
 * Note:            See Important Note above!
 ********************************************************************/
bool            TCPIP_STACK_SetNetAddress(TCPIP_NET_HANDLE netH, IP_ADDR* ipAddress, bool setDefault);

/*********************************************************************
 * Function:        bool TCPIP_STACK_SetNetGatewayAddress(TCPIP_NET_HANDLE netH, IP_ADDR* ipAddress)
 *
 * PreCondition:    TCPIP stack should have been initialized by TCPIP_STACK_Init()
 *
 * Input:           netH - Interface handle to set the gateway address of.
 *                  ipAddress - IP address to set
 *
 * Output:          true if succes
 *                  false if not such interface
 *               
 * Side Effects:    None
 *
 * Example:			TCPIP_NET_HANDLE netH = TCPIP_STACK_NetHandle("PIC32INT");
 * 					TCPIP_STACK_SetNetGatewayAddress(netH, &myIPAddress);
 *
 * Note:            See Important Note above!
 ********************************************************************/
bool            TCPIP_STACK_SetNetGatewayAddress(TCPIP_NET_HANDLE netH, IP_ADDR* ipAddress);

/*********************************************************************
 * Function:        bool TCPIP_STACK_SetNetPriDNSAddress(TCPIP_NET_HANDLE netH, IP_ADDR* ipAddress)
 *
 * PreCondition:    TCPIP stack should have been initialized by TCPIP_STACK_Init()
 *
 * Input:           netH - Interface handle to set the DNS address of.
 *                  ipAddress - IP address to set
 *
 * Output:          true if succes
 *                  false if not such interface
 *               
 * Side Effects:    None
 *
 * Example:			TCPIP_NET_HANDLE netH = TCPIP_STACK_NetHandle("PIC32INT");
 * 					TCPIP_STACK_SetNetPriDNSAddress(netH, &myIPAddress);
 *
 * Note:            See Important Note above!
 ********************************************************************/
bool            TCPIP_STACK_SetNetPriDNSAddress(TCPIP_NET_HANDLE netH, IP_ADDR* ipAddress);

/*********************************************************************
 * Function:        bool TCPIP_STACK_SetNetSecondDNSAddress(TCPIP_NET_HANDLE netH, IP_ADDR* ipAddress)
 *
 * PreCondition:    TCPIP stack should have been initialized by TCPIP_STACK_Init()
 *
 * Input:           netH - Interface handle to set the secondary DNS address of.
 *                  ipAddress - IP address to set
 *
 * Output:          true if succes
 *                  false if not such interface
 *               
 * Side Effects:    None
 *
 * Example:			TCPIP_NET_HANDLE netH = TCPIP_STACK_NetHandle("PIC32INT");
 * 					TCPIP_STACK_SetNetSecondDNSAddress(netH, &myIPAddress);
 *
 * Note:            See Important Note above!
 ********************************************************************/
bool            TCPIP_STACK_SetNetSecondDNSAddress(TCPIP_NET_HANDLE netH, IP_ADDR* ipAddress);

/*********************************************************************
 * Function:        bool TCPIP_STACK_SetNetMask(TCPIP_NET_HANDLE netH, IP_ADDR* mask, bool setDefault)
 *
 * PreCondition:    None
 *
 * Input:           netH - Interface handle to get address of.
 *                  mask    - network mask to use
 *                  setDefault  - if true, the default mask is also set
 *
 * Output:          true if success
 *                  false if no such interface 
 *
 * Example:         TCPIP_NET_HANDLE netH = TCPIP_STACK_NetHandle("PIC32INT");
 * 					uint32_t subMask = TCPIP_STACK_SetNetMask(netH, &myIpAddr);
 *
 * Note:            See Important Note above!
 ********************************************************************/
bool            TCPIP_STACK_SetNetMask(TCPIP_NET_HANDLE netH, IP_ADDR* mask, bool setDefault);

/*********************************************************************
 * Function:        bool TCPIP_STACK_SetNetBIOSName(TCPIP_NET_HANDLE netH, const char* biosName);
 *
 * PreCondition:    None
 *
 * Input:           netH - Interface handle to set name of.
 *
 * Output:          if interface exists then it sets the NetBIOS name of that interface 
 *                  and returns true
 * 					else return false
 *
 * Example:         TCPIP_NET_HANDLE netH = TCPIP_STACK_NetHandle("PIC32INT");
 * 					TCPIP_STACK_SetNetBIOSName(netH, myBiosName);
 *
 * Note:            See Important Note above!
 ********************************************************************/
bool                TCPIP_STACK_SetNetBIOSName(TCPIP_NET_HANDLE netH, const char* biosName);

/*********************************************************************
 * Function:        bool TCPIP_STACK_SetNetMacAddress(TCPIP_NET_HANDLE netH, const MAC_ADDR* pAddr)
 *
 * PreCondition:    None
 *
 * Input:           netH - Interface handle to get the address of.
 *
 * Output:          if interface is enabled then it returns a constant pointer to the MAC address
 *                  of that interface 
 * 					else return 0
 *
 * Example:         TCPIP_NET_HANDLE netH = TCPIP_STACK_NetHandle("PIC32INT");
 * 					TCPIP_STACK_SetNetMacAddress(netH, &myMacAddress);
 *
 * Note:            See Important Note above!
 ********************************************************************/
bool                TCPIP_STACK_SetNetMacAddress(TCPIP_NET_HANDLE netH, const MAC_ADDR* pAddr);



#endif  // __TCPIP_MANAGER_H_



