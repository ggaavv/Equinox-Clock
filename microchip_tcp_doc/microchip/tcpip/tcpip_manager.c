/*******************************************************************************
  TCP/IP Stack Manager

  Summary:
    Module for Microchip TCP/IP Stack
    
  Description:
    -Handles internal RX packet pre-processing prior to dispatching 
     to upper application layers.
    -Reference: AN833
*******************************************************************************/

/*******************************************************************************
FileName:   tcpip_manager.c
Copyright ?2012 released Microchip Technology Inc.  All rights
reserved.

Microchip licenses to you the right to use, modify, copy and distribute
Software only when embedded on a Microchip microcontroller or digital signal
controller that is integrated into your product or third party product
(pursuant to the sublicense terms in the accompanying license agreement).

You should refer to the license agreement accompanying this Software for
additional information regarding your rights and obligations.

SOFTWARE AND DOCUMENTATION ARE PROVIDED “AS IS?WITHOUT WARRANTY OF ANY KIND,
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

#include "tcpip_private.h"


#if defined( TCPIP_IF_MRF24W )
    #if defined( WF_CONFIG_CONSOLE )
        #include "wifi/wf_console.h"
    #endif
    #if defined( TCPIP_STACK_USE_EZ_CONFIG ) || defined( EZ_CONFIG_SCAN )
        #include "wifi/wf_easy_config.h"
    #endif
	#include "tcpip/wf_api.h"
#endif

#include "mac_private.h"

#include "tcpip_module_manager.h"


NODE_INFO remoteNode;


// Declare NetConfig structure and some other supporting stack variables
// Note that for multi-homed hosts
// there is one NetConfig entry per physical interface
static NET_CONFIG* NetConfig = 0;       // dynamically allocated


// Main default interfaces
typedef struct
{
    NET_CONFIG* defaultNet;     // default network interface
}TCPIPDefaultIF;
    

static TCPIPDefaultIF tcpipDefIf = { 0 }; 
    
static volatile int	totTcpipEventsCnt = 0;	
static volatile int	newTcpipTickAvlbl = 0;	

static volatile int	newTcpipErrorEventCnt = 0;	
static volatile int	newTcpipStackEventCnt = 0;	



static void TCPIP_STACK_Applications(NET_CONFIG* pNetIf);
static void TCPIP_STACK_NonEventApplications(NET_CONFIG* pNetIf);


// TCPIP event descriptor
// Some events are not necessarily triggered by hardware
// (link the link up/down events)
// so this structure still has some members even if the hardware 
// events are not enabled
typedef struct
{
    union
    {
        struct
        {
            volatile int    newTcpipEventAvlbl: 1;     // event available flag
            int             notifyEnabled:      1;      // notification enabled flag
            unsigned                      :     6;      // not used
        };
        unsigned char   b;
    };
    TCPIP_EVENT     activeEvents;          // accumulated event available
    bool            linkPrev;              // previous status of the link
#if defined(TCPIP_STACK_USE_EVENT_NOTIFICATION)
    pTCPIPNotifyF   notifyHandler;         // notification handler
    void*           notifyfParam;          // notification handler parameter
                                           // false if link down true if link up
                                           // used for link up/down event generation
#endif  // defined(TCPIP_STACK_USE_EVENT_NOTIFICATION)
}TCPIPEventDcpt;


static TCPIPEventDcpt tcpipEventDcpt[TCPIP_NETWORK_INTERFACES] = { {{{0}}} }; 

static void	TCPIPMacEventCB(void* hParam, TCPIP_EVENT event);

static uint8_t*    tcpip_heap = 0;          // the actual TCPIP heap

static TCPIP_STACK_MODULE_CTRL  tcpip_stack_ctrl_data = {0};

//
static SystemTickHandle    tcpip_stack_tickH = 0;      // tick handle

static void TCPIP_STACK_TickHandler(SYS_TICK currSysTick);        // stack tick handler
    
static void ProcessTCPIPTickEvent(void);
static void ProcessTCPIPMacEvents(NET_CONFIG* pNetIf, TCPIP_EVENT activeEvent);
static void ProcessTCPIPMacGeneric(NET_CONFIG* pNetIf);
static void ProcessTCPIPMacErrorEvents(NET_CONFIG* pNetIf, TCPIP_EVENT activeEvent);

static bool InitNetConfig(const TCPIP_NETWORK_CONFIG* pUsrConfig, int nNets);
static bool LoadDefaultConfig(const TCPIP_NETWORK_CONFIG* pUsrConfig, NET_CONFIG* pNetIf);

static bool TCPIP_STACK_VerifyPktIf(NET_CONFIG* pNetIf, IP_ADDR* pktDestIP);

static const TCPIP_STACK_MODULE_CONFIG* TcpipStackFindModuleData(TCPIP_STACK_MODULE moduleId, const TCPIP_STACK_MODULE_CONFIG* pModConfig, int nModules);

static void  TCPIP_STACK_BringNetDown(TCPIP_STACK_MODULE_CTRL* stackCtrlData);
static bool  TCPIP_STACK_BringNetUp(TCPIP_STACK_MODULE_CTRL* stackCtrlData, const TCPIP_NETWORK_CONFIG* pNetConf, const TCPIP_STACK_MODULE_CONFIG* pModConfig, int nModules);

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
 *
 ********************************************************************/
bool TCPIP_STACK_Init(const TCPIP_NETWORK_CONFIG* pUsrConfig, int nNets, const TCPIP_STACK_MODULE_CONFIG* pModConfig, int nModules)
{
    int                     netIx, modIx, initFail;
    const TCPIP_STACK_MODULE_ENTRY*  pEntry;
    TCPIP_HEAP_HANDLE       heapH;
    NET_CONFIG*             pIf;
    TCPIP_STACK_POWER_MODE  powerMode;
    
    if(nNets == 0 || NetConfig != 0)
    {   // already up and running
        return true;
    }
 
    totTcpipEventsCnt = 0;	

    newTcpipErrorEventCnt = 0;	
    newTcpipStackEventCnt = 0;
    newTcpipTickAvlbl = 0;    


    // start stack initialization
    pEntry = TCPIP_STACK_MODULE_ENTRY_TBL + 0;
    for(modIx = 0; modIx < sizeof(TCPIP_STACK_MODULE_ENTRY_TBL)/sizeof(*TCPIP_STACK_MODULE_ENTRY_TBL); modIx++)
    {
        if(pEntry->asyncPending != 0 && pEntry->asyncHandler == 0)
        {
            SYS_ERROR(SYS_ERROR_ERROR, "Module error: missing asynchronous handler!");
            return 0;
        }
        pEntry++;
    }

    memset(&tcpip_stack_ctrl_data, 0, sizeof(tcpip_stack_ctrl_data));

    tcpip_heap = (uint8_t*)SystemMalloc(TCPIP_STACK_DRAM_SIZE);   
                                                       

    heapH = TCPIP_HEAP_Create(tcpip_heap, TCPIP_STACK_DRAM_SIZE, 0, 0);     // get handle to the heap memory
    if(heapH == 0)
    {
        SystemFree(tcpip_heap);     // free the allocated memory
        tcpip_heap = 0;
        SYS_ERROR(SYS_ERROR_ERROR, "Heap creation failed");
        return 0;
    }

    NetConfig = (NET_CONFIG*)TCPIP_HEAP_Malloc(heapH, sizeof(NET_CONFIG) * nNets); // allocate for each network interface
    if(NetConfig == 0)
    {   // failed
        SYS_ERROR(SYS_ERROR_ERROR, "Network configuration allocation failed");
        TCPIP_HEAP_Delete(heapH);     // destroy the heap 
        SystemFree(tcpip_heap);     // free the allocated memory
        tcpip_heap = 0;
        return 0;
    }

    tcpip_stack_ctrl_data.memH = heapH;
    tcpip_stack_ctrl_data.mallocCallback = TCPIP_HEAP_Malloc;
    tcpip_stack_ctrl_data.callocCallback = TCPIP_HEAP_Calloc;
    tcpip_stack_ctrl_data.freeCallback = (void (*)( const void*, void*))TCPIP_HEAP_Free;

    tcpip_stack_ctrl_data.nIfs = nNets;

	// Seed the LFSRRand() function
	LFSRSeedRand(SYS_GENERATE_RANDOM_DWORD());

    if(InitNetConfig(pUsrConfig, nNets))
    {
        initFail=0;
    }
    else
    {
        SYS_ERROR(SYS_ERROR_ERROR, "Network configuration initialization failed");
        initFail = 1;   // failed the initialization
    }

    tcpipDefIf.defaultNet = 0;          // delete the old default
    // start per interface initializing 
    tcpip_stack_ctrl_data.stackAction = TCPIP_STACK_ACTION_INIT;

    for(netIx = 0, pIf = NetConfig; initFail == 0 && netIx < nNets; netIx++, pIf++)
    {
        memset(tcpipEventDcpt + netIx, 0x0, sizeof(*tcpipEventDcpt));

#if defined(TCPIP_STACK_USE_IP_GLEANING) || defined(TCPIP_STACK_USE_DHCP_CLIENT)
        /*
         * If DHCP or IP Gleaning is enabled,
         * startup in Config Mode.
         */
        pIf->Flags.bInConfigMode = true;
#endif        
        
        // get the power mode
#if defined (__C30__) 
        powerMode = TCPIP_STACK_POWER_FULL;
#else
        powerMode = StringToTCPIPPowerMode(pUsrConfig->powerMode);
#endif
        if(powerMode == TCPIP_STACK_POWER_NONE || powerMode != TCPIP_STACK_POWER_FULL)
        {   
            SYS_ERROR(SYS_ERROR_ERROR, "Stack Power Mode initialization fail");
            initFail = 1;
            break;
        }

        // set transient data
        tcpip_stack_ctrl_data.powerMode = powerMode;
        tcpip_stack_ctrl_data.pNetIf = pIf;
        tcpip_stack_ctrl_data.netIx = netIx;
        if(!TCPIP_STACK_BringNetUp(&tcpip_stack_ctrl_data, pUsrConfig, pModConfig, nModules))
        {
            initFail = 1;
            break;
        }
        
        
#if defined(TCPIP_STACK_USE_DHCP_CLIENT)
        if(!pIf->Flags.bIsDHCPEnabled)
        {
            DHCPDisable(pIf, false);
        }
#endif

		// interface success
        // set the default interfaces
		if(tcpipDefIf.defaultNet == 0)
		{
			tcpipDefIf.defaultNet = pIf;    // set as the 1st valid interface
		}
    }

    // initializing the rest of the services
    // (that don't need interface specific initialization)

    while(!initFail)
    {    
        tcpip_stack_tickH = SYS_TICK_TimerCreate(TCPIP_STACK_TickHandler);
        if(tcpip_stack_tickH)
        {
            SYS_TICK_TimerSetRate(tcpip_stack_tickH, (SYS_TICK_ResolutionGet()+TCPIP_STACK_TICKS_PER_SECOND-1)/TCPIP_STACK_TICKS_PER_SECOND);
        }
        else
        {
            SYS_ERROR(SYS_ERROR_ERROR, "Stack tick registration failed");
            initFail = 1;
            break;
        }

        break;
    }

    // initialization done

    if(initFail)
    {
        TCPIP_STACK_DeInit();
        return false;
    }

    return true;
}

/*********************************************************************
 * Function:        bool TCPIP_STACK_BringNetUp(NET_CONFIG* pNetIf, const TCPIP_NETWORK_CONFIG* pNetConf, const TCPIP_STACK_MODULE_CONFIG* pModConfig, int nModules)
 *
 * PreCondition:    None
 *
 * Input:           net interface to bring up
 *
 * Output:          None
 *
 * Side Effects:    None
 *
 * Overview:        This function performs the initialization of a net interface
 *
 * Note:            None
 ********************************************************************/
static bool TCPIP_STACK_BringNetUp(TCPIP_STACK_MODULE_CTRL* stackCtrlData, const TCPIP_NETWORK_CONFIG* pNetConf, const TCPIP_STACK_MODULE_CONFIG* pModConfig, int nModules)
{
    NET_CONFIG*             pNetIf;
    bool                    netUpFail;
    TCPIP_MAC_MODULE_GONFIG   macInitData;

    // MAC initialization
    memset(&macInitData, 0, sizeof(macInitData));

    macInitData._nTxDescriptors = pNetConf->nTxDescriptors;
    macInitData._TxBuffSize = pNetConf->txBuffSize;
    macInitData._nRxDescriptors = pNetConf->nRxDescriptors;
    macInitData._RxBuffSize = pNetConf->rxBuffSize;


    netUpFail = false;
    pNetIf = stackCtrlData->pNetIf;
    // restore the dynamic interface data
    pNetIf->netIfIx = stackCtrlData->netIx;
    pNetIf->powerMode = stackCtrlData->powerMode;
   

    while(true)
    {
        if(MACInitialize(stackCtrlData, &macInitData) != TCPIP_MAC_RES_OK)
        {
            SYS_ERROR(SYS_ERROR_ERROR, "MAC initialization failed");
            netUpFail = 1;
            break;
        }

        if( (pNetIf->hIfMac = MACOpen(pNetIf->macId)) == 0)
        {
            SYS_ERROR(SYS_ERROR_ERROR, "MAC Open failed");
            netUpFail = 1;
            break;
        }


		// start stack initialization per module
		int modIx;
		const TCPIP_STACK_MODULE_ENTRY*  pEntry = TCPIP_STACK_MODULE_ENTRY_TBL + 0;

		for(modIx = 0; modIx < sizeof(TCPIP_STACK_MODULE_ENTRY_TBL)/sizeof(*TCPIP_STACK_MODULE_ENTRY_TBL); modIx++)
		{
			if(pEntry->initFunc)
			{
				const void *		configData;
				if (pModConfig != 0) {
					const TCPIP_STACK_MODULE_CONFIG* pConfig = TcpipStackFindModuleData(pEntry->moduleId, pModConfig, nModules);
					if(pConfig == 0)
					{
						SYS_ERROR(SYS_ERROR_ERROR, "Module Initialization data missing");
						netUpFail = 1;
						break;
					}
					configData = pConfig->configData;
				}
				else {
					configData = 0;
				}
					
				if(!pEntry->initFunc(stackCtrlData, configData))
				{
					SYS_ERROR(SYS_ERROR_ERROR, "Module Initialization failed");
					netUpFail = 1;
					break;
				}
			}
			pEntry++;
		}

        if(!netUpFail)
		{
#if defined(TCPIP_STACK_USE_EVENT_NOTIFICATION)
			TCPIP_MAC_EVENT_RESULT  evRes;

			// Stack can use one handler for all network interfaces, like in this case
			// Each time a notification is received, all interfaces are checked
			// Or, more efficient, use a handler per interface
			evRes = MACEventSetNotifyHandler(pNetIf->hIfMac, TCPIP_MAC_EVGROUP_ALL, TCPIPMacEventCB, pNetIf);

			if(evRes != TCPIP_MAC_EVRES_OK)
			{
				SYS_ERROR(SYS_ERROR_ERROR, "MAC event notification registration failed");
				netUpFail = 1;
				break;
			}

			evRes = MACEventSetNotifyEvents(pNetIf->hIfMac, TCPIP_MAC_EVGROUP_ALL,
                                            TCPIP_EV_RX_PKTPEND | TCPIP_EV_TX_DONE | TCPIP_EV_RXTX_ERRORS | TCPIP_EV_CONN_ALL);
			if(evRes != TCPIP_MAC_EVRES_OK)
			{
				SYS_ERROR(SYS_ERROR_ERROR, "MAC event notification setting failed");
				netUpFail = 1;
				break;
			}
#endif  // defined(TCPIP_STACK_USE_EVENT_NOTIFICATION)
            
            // completed the MAC initialization
            // connect
            MACConnect(pNetIf->hIfMac);
		}
        
        break;
    }

    if(netUpFail)
    {
        return false;
    }
    

    pNetIf->Flags.bInterfaceEnabled = true;
    pNetIf->powerMode = stackCtrlData->powerMode;

    return true;

}

/*********************************************************************
 * Function:        void TCPIP_STACK_NetUp(TCPIP_NET_HANDLE netH, const TCPIP_NETWORK_CONFIG* pUsrConfig)
 *
 * PreCondition:    None
 *
 * Input:           net interface to bring up
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
bool TCPIP_STACK_NetUp(TCPIP_NET_HANDLE netH, const TCPIP_NETWORK_CONFIG* pUsrConfig)
{
    bool    success;
    TCPIP_STACK_POWER_MODE  powerMode;

    NET_CONFIG* pNetIf = _TCPIPStackHandleToNet(netH);
    
    if(pNetIf)
    {
        if(pNetIf->Flags.bInterfaceEnabled)
        {   // already up
            return true;
        }
		
		// Before we load the default config, we should save what used to be the netIfIx
        // set transient data 
        tcpip_stack_ctrl_data.pNetIf = pNetIf;
        tcpip_stack_ctrl_data.netIx = pNetIf->netIfIx;
        tcpip_stack_ctrl_data.stackAction = TCPIP_STACK_ACTION_IF_UP;

        if(!LoadDefaultConfig(pUsrConfig, pNetIf))
        {
            SYS_ERROR(SYS_ERROR_ERROR, "Default Flash Network configuration load failed");
            return false;
        }

#if defined (__C30__)  
        powerMode = TCPIP_STACK_POWER_FULL;
#else
        powerMode = StringToTCPIPPowerMode(pUsrConfig->powerMode);
#endif
        if(powerMode == TCPIP_STACK_POWER_NONE || powerMode != TCPIP_STACK_POWER_FULL)
        {   
            SYS_ERROR(SYS_ERROR_ERROR, "Stack Power Mode initialization fail");
            return false;
        }
        
        tcpip_stack_ctrl_data.powerMode = powerMode;

        success = TCPIP_STACK_BringNetUp(&tcpip_stack_ctrl_data, pUsrConfig, 0, 0);
        if(!success)
        {   // don't let the MAC hanging because of a module failure
            tcpip_stack_ctrl_data.stackAction = TCPIP_STACK_ACTION_IF_DOWN;
            tcpip_stack_ctrl_data.powerMode = TCPIP_STACK_POWER_DOWN;
            TCPIP_STACK_BringNetDown(&tcpip_stack_ctrl_data);
        }
        return success;
    }

    return false;

}
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
void TCPIP_STACK_DeInit(void)
{
    int         netIx;
    NET_CONFIG* pIf;

    if(NetConfig == 0)
    {   // already shut down
        return;
    }
    
    SYS_TICK_TimerDelete(tcpip_stack_tickH);
    tcpip_stack_tickH = 0;

    // set transient data
    tcpip_stack_ctrl_data.stackAction = TCPIP_STACK_ACTION_DEINIT;
    tcpip_stack_ctrl_data.powerMode = TCPIP_STACK_POWER_DOWN;

    for(netIx = 0, pIf = NetConfig; netIx < tcpip_stack_ctrl_data.nIfs; netIx++, pIf++)
    {
        tcpip_stack_ctrl_data.pNetIf = pIf;
        tcpip_stack_ctrl_data.netIx = pIf->netIfIx;
        if(pIf->Flags.bInterfaceEnabled)
        {
            TCPIP_STACK_BringNetDown(&tcpip_stack_ctrl_data);
        }
    }


    TCPIP_HEAP_Free(tcpip_stack_ctrl_data.memH, NetConfig);
    TCPIP_HEAP_Delete(tcpip_stack_ctrl_data.memH);     // destry the heap
    tcpip_stack_ctrl_data.memH = 0;
    NetConfig = 0;

    SystemFree(tcpip_heap);     // free the allocated memory
    tcpip_heap = 0;

    tcpip_stack_ctrl_data.nIfs = 0;
}


/*********************************************************************
 * Function:        void TCPIP_STACK_BringNetDown(TCPIP_STACK_MODULE_CTRL* stackCtrlData)
 *
 * PreCondition:    None
 *
 * Input:           net interface to bring down
 *
 * Output:          None
 *
 * Side Effects:    None
 *
 * Overview:        This function performs the de-initialization of a net interface
 *
 * Note:            None
 ********************************************************************/
static void TCPIP_STACK_BringNetDown(TCPIP_STACK_MODULE_CTRL* stackCtrlData)
{
    const TCPIP_STACK_MODULE_ENTRY*  pEntry;
    NET_CONFIG* pNetIf;


    // Go to the last entry in the table
    pEntry = TCPIP_STACK_MODULE_ENTRY_TBL + sizeof(TCPIP_STACK_MODULE_ENTRY_TBL)/sizeof(*TCPIP_STACK_MODULE_ENTRY_TBL);
    do
    {
        pEntry--;
        if(pEntry->deInitFunc)
        {
            pEntry->deInitFunc(stackCtrlData);
        }
    }
    while (pEntry != TCPIP_STACK_MODULE_ENTRY_TBL);

    pNetIf = stackCtrlData->pNetIf;
    if(pNetIf->hIfMac)
    {
        MACClose(pNetIf->hIfMac);
        pNetIf->hIfMac = 0;
    }
    MACDeinitialize(stackCtrlData);

    pNetIf->Flags.bInterfaceEnabled = false;
    pNetIf->powerMode = stackCtrlData->powerMode;

}

/*********************************************************************
 * Function:        void TCPIP_STACK_NetDown(TCPIP_NET_HANDLE netH)
 *
 * PreCondition:    None
 *
 * Input:           net interface to bring down
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
bool TCPIP_STACK_NetDown(TCPIP_NET_HANDLE netH)
{
    int netIx;
    NET_CONFIG *pIf, *pNewIf;
    NET_CONFIG* pDownIf = _TCPIPStackHandleToNet(netH);

    if(pDownIf)
    {
        if(pDownIf->Flags.bInterfaceEnabled)
        {
            // set transient data
            tcpip_stack_ctrl_data.stackAction = TCPIP_STACK_ACTION_IF_DOWN;
            tcpip_stack_ctrl_data.powerMode = TCPIP_STACK_POWER_DOWN;
            tcpip_stack_ctrl_data.pNetIf = pDownIf;
            tcpip_stack_ctrl_data.netIx = pDownIf->netIfIx;

            if(tcpipDefIf.defaultNet == pDownIf)
            {   // since this interface is going down change the default interface
                pNewIf = 0;
                for(netIx = 0, pIf = NetConfig; netIx < tcpip_stack_ctrl_data.nIfs; netIx++, pIf++)
                {
                    if(pIf != pDownIf && pIf->Flags.bInterfaceEnabled)
                    {   // select this one
                        pNewIf = pIf;
                        break;
                    }
                }
                tcpipDefIf.defaultNet = pNewIf;
            }

            TCPIP_STACK_BringNetDown(&tcpip_stack_ctrl_data);
        }
        return true;
    }

    return false;

}


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
void TCPIP_STACK_Task(void)
{
    int                 netIx, modIx;
    const TCPIP_STACK_MODULE_ENTRY*  pEntry;
    NET_CONFIG*         pNetIf;
    bool                tickEvent;

    #if defined (TCPIP_STACK_USE_IPV6)
    TCPIP_NDP_Task();
    #endif

    if( newTcpipTickAvlbl)
    {
        ProcessTCPIPTickEvent();
        tickEvent = true;
    }
    else
    {
        tickEvent = false;
    }
    
#if !defined(TCPIP_STACK_USE_EVENT_NOTIFICATION) || defined(TCPIP_IF_MRF24W)
    totTcpipEventsCnt = 1;  // fake that we always have an event pending
                            // the MRF24W cannot really work in interrupt mode! 
#endif

    if( totTcpipEventsCnt )
    {   // event pending
        
        TCPIP_EVENT activeEvents;

        totTcpipEventsCnt = 0;

    
        for(netIx = 0, pNetIf = NetConfig; netIx < tcpip_stack_ctrl_data.nIfs; netIx++, pNetIf++)
        {
			if (!pNetIf->Flags.bInterfaceEnabled)
            {
				continue;
			}
            TCPIPEventDcpt* pEvDcpt = tcpipEventDcpt + netIx;
            activeEvents =  pEvDcpt->activeEvents;
            
#if defined(TCPIP_STACK_USE_EVENT_NOTIFICATION)
            TCPIP_MAC_HANDLE activeMac = pNetIf->hIfMac;
            {
                TCPIP_EVENT macEvents = 0;
                if(pEvDcpt->newTcpipEventAvlbl)
                {
                    pEvDcpt->newTcpipEventAvlbl = 0;               
                    macEvents = MACEventGetPending(activeMac, TCPIP_MAC_EVGROUP_ALL);
                }
#if defined(TCPIP_IF_MRF24W)
                else if(pNetIf->macId == TCPIP_MAC_ID_MRF24W)
                {
                    macEvents = TCPIP_EV_RX_PKTPEND|TCPIP_EV_TX_DONE;    // fake pending MAC events
                    // the MRF24W cannot really work in interrupt mode!
                } 
#endif  // defined(TCPIP_IF_MRF24W)
                activeEvents |= macEvents;
            }
#else
            activeEvents |= TCPIP_EV_RX_PKTPEND|TCPIP_EV_TX_DONE;    // just fake pending events
#endif  // defined(TCPIP_STACK_USE_EVENT_NOTIFICATION)
            
            // clear processed events    
            pEvDcpt->activeEvents &= ~activeEvents;
            
            if(activeEvents || tickEvent)
            {   
                if(pNetIf->Flags.bInterfaceEnabled)
                {
                    ProcessTCPIPMacGeneric(pNetIf);

                    if(activeEvents&(TCPIP_EV_RX_PKTPEND|TCPIP_EV_TX_DONE))
                    {
                        ProcessTCPIPMacEvents(pNetIf, activeEvents);
                        newTcpipStackEventCnt++;
                    }

                    TCPIP_STACK_Applications(pNetIf);

                    if(activeEvents&TCPIP_EV_RXTX_ERRORS)
                    {	// some error has occurred
                        ProcessTCPIPMacErrorEvents(pNetIf, activeEvents);	
                    }
                }
                
                if((activeEvents & TCPIP_EV_CONN_ALL) != 0)
                {
                    for(modIx = 0; modIx < sizeof(TCPIP_STACK_CONN_EVENT_TBL)/sizeof(*TCPIP_STACK_CONN_EVENT_TBL); modIx++)
                    {
                        (*TCPIP_STACK_CONN_EVENT_TBL[modIx])(pNetIf, activeEvents);
                    }
                }
                

#if defined(TCPIP_STACK_USE_EVENT_NOTIFICATION)
                activeEvents &= ~TCPIP_EV_CONN_ALL; // connection events generated internally so no need to be ack-ed
                if(activeEvents)
                {
                    MACEventAck(activeMac, TCPIP_MAC_EVGROUP_ALL, activeEvents);
                }
#endif  // defined(TCPIP_STACK_USE_EVENT_NOTIFICATION)
            }
        }
    }

	for(netIx = 0, pNetIf = NetConfig; netIx < tcpip_stack_ctrl_data.nIfs; netIx++, pNetIf++)
    {
        if(pNetIf->Flags.bInterfaceEnabled)
        {
            TCPIP_STACK_NonEventApplications(NetConfig+netIx);
        }
    }
    
    // process the asynchronous handlers
    pEntry = TCPIP_STACK_MODULE_ENTRY_TBL + 0;
    for(modIx = 0; modIx < sizeof(TCPIP_STACK_MODULE_ENTRY_TBL)/sizeof(*TCPIP_STACK_MODULE_ENTRY_TBL); modIx++)
    {
        if(pEntry->asyncPending != 0 )
        {
            if((*pEntry->asyncPending)())
            {
                (*pEntry->asyncHandler)();
            }
        }
        pEntry++; 
    }	
}

// non event specific processing
static void ProcessTCPIPMacGeneric(NET_CONFIG* pNetIf)
{

    MACProcess(pNetIf->hIfMac);

#if defined(TCPIP_STACK_USE_DHCP_CLIENT)
    DHCPTask(pNetIf);
#endif
    // WFEasyConfigMgr(); Note: moved to MRF24W_MACProcess().

#if defined(TCPIP_STACK_USE_UDP)
    UDPTask(pNetIf);
#endif
}

static void ProcessTCPIPTickEvent(void)
{
    int netIx;
    NET_CONFIG* pNetIf;
    bool    linkCurr;
    
    newTcpipTickAvlbl = 0;

    for(netIx = 0, pNetIf = NetConfig; netIx < tcpip_stack_ctrl_data.nIfs; netIx++, pNetIf++)
    {
        if(pNetIf->Flags.bInterfaceEnabled)
        {
#if defined (TCPIP_STACK_USE_AUTO_IP)
            AutoIPTasks(pNetIf);
#endif

            linkCurr = MACCheckLink(pNetIf->hIfMac);     // check link status
            if(pNetIf->linkPrev != linkCurr)
            {   // link status changed
                // just call directly the CB, and do not involve the MAC notification mechanism
                TCPIPMacEventCB(pNetIf, linkCurr?TCPIP_EV_CONN_ESTABLISHED:TCPIP_EV_CONN_LOST);
                pNetIf->linkPrev = linkCurr;
            }
        }
    }

}

static void ProcessTCPIPMacEvents(NET_CONFIG* pNetIf, TCPIP_EVENT activeEvent)
{
    
    uint16_t dataCount;
    IP_ADDR pktDestIP;
	uint16_t cFrameType;
	uint8_t cIPFrameType;
  
	// Process as many incomming packets as we can
	while(activeEvent & TCPIP_EV_RX_PKTPEND)
	{
		// if using the random module, generate entropy
        // Entropy is generated as follows:
        //	1. 	For every packet received, the last byte of the remote 
        //		MAC address and the four bytes of SYS_TICK_Get() are added
        //		to a SHA-1 Hash (the seed)
        //	2.	Every time a random byte is requested, the hash is
        //		calculated and the hash sum is hashed into the seed.
        //	3.	Up to 20 bytes are returned from this hash sum before
        //		a new hash is calculated.
        //	4.	Every time new entropy is added, the remaining random
        //		output is discarded and new random bytes will be 
        //		generated upon future calls to SYS_RANDOM_GET().
		#if defined(TCPIP_STACK_USE_RANDOM)
			SYS_RANDOM_ADD(remoteNode.MACAddr.v[5]);
		#endif

		// We are about to fetch a new packet, make sure that the 
		// UDP module knows that any old RX data it has laying 
		// around will now be gone.
		#if defined(TCPIP_STACK_USE_UDP)
			UDPDiscardNet(pNetIf);
		#endif

		// Fetch a packet (throws old one away, if not thrown away 
		// yet)
		if(!MACGetHeader(pNetIf->hIfMac, &remoteNode.MACAddr, &cFrameType))
			break;

        if(MACRxFilter(pNetIf->hIfMac, &remoteNode))
            continue;       // packet filtered out
                
		// Dispatch the packet to the appropriate handler
		switch(cFrameType)
		{
			case ETHERTYPE_ARP:
				ARPProcess(pNetIf);
				break;
	
			case ETHERTYPE_IPV4:
				if(!TCPIP_IP_GetHeader(pNetIf, &pktDestIP, &remoteNode, &cIPFrameType, &dataCount))
					break;


                #if (defined(TCPIP_STACK_USE_ICMP_SERVER) || defined(TCPIP_STACK_USE_ICMP_CLIENT)) && defined(TCPIP_STACK_USE_IP_GLEANING)
				if(cIPFrameType == IP_PROT_ICMP && pNetIf->Flags.bInConfigMode && pNetIf->Flags.bIsDHCPEnabled)
                {
                    // According to "IP Gleaning" procedure,
                    // when we receive an ICMP packet with a valid
                    // IP address while we are still in configuration
                    // mode, accept that address as ours and conclude
                    // configuration mode.
                    if(pktDestIP.Val != 0xffffffff)
                    {
                        pNetIf->Flags.bInConfigMode = false;
                        pNetIf->MyIPAddr = pktDestIP;
                    }
                }
			    #endif // (defined(TCPIP_STACK_USE_ICMP_SERVER) || defined(TCPIP_STACK_USE_ICMP_CLIENT)) && defined(TCPIP_STACK_USE_IP_GLEANING)
                   
                // check the packet arrived on the proper interface
                if(!TCPIP_STACK_VerifyPktIf(pNetIf, &pktDestIP))
                {
                    break;   // discard
                }
                
				#if defined(TCPIP_STACK_USE_ICMP_SERVER) || defined(TCPIP_STACK_USE_ICMP_CLIENT)
				if(cIPFrameType == IP_PROT_ICMP)
                {
                    ICMPProcess(pNetIf, &remoteNode, dataCount);
                    break;
                }
				#endif
				
				#if defined(TCPIP_STACK_USE_TCP)
				if(cIPFrameType == IP_PROT_TCP)
				{
					TCPProcess(pNetIf, &remoteNode, &pktDestIP, dataCount);
					break;
				}
				#endif
				
				#if defined(TCPIP_STACK_USE_UDP)
				if(cIPFrameType == IP_PROT_UDP)
				{
					// Stop processing packets if we came upon a UDP frame with application data in it
					if(UDPProcess(pNetIf, &remoteNode, &pktDestIP, dataCount))
                    {
						return;
                    }
				}
				#endif

				break;
#if defined(TCPIP_STACK_USE_IPV6)
            case ETHERTYPE_IPV6:
                if (pNetIf->Flags.bIPv6Enabled)
                    TCPIP_IPV6_Process(pNetIf, &remoteNode.MACAddr);
                break;
#endif
		}
	}
}

/*********************************************************************
 * Function:        void TCPIP_STACK_Applications(NET_CONFIG* pNetIf)
 *
 * PreCondition:    TCPIP_STACK_Init() is already called.
 *
 * Input:           pNetIf   - interface configuration
 * 
 * Output:          Calls all loaded application modules.
 *
 * Side Effects:    None
 *
 * Note:            This function must be called periodically to
 *                  ensure timely responses.
 *
 ********************************************************************/
static void TCPIP_STACK_Applications(NET_CONFIG* pNetIf)
{
	#if defined(TCPIP_STACK_USE_HTTP_SERVER) || defined(TCPIP_STACK_USE_HTTP2_SERVER)
	HTTPServer();
	#endif
	
	#if defined(TCPIP_STACK_USE_FTP_SERVER) && defined(TCPIP_STACK_USE_MPFS)
	FTPServer();
	#endif
	
	#if defined(TCPIP_STACK_USE_SNMP_SERVER)
	SNMPTask(pNetIf);
	#endif
	
	#if defined(TCPIP_STACK_USE_ANNOUNCE)
	ANNOUNCE_Task(pNetIf);
	#endif
	
	#if defined(TCPIP_STACK_USE_NBNS)
	NBNSTask(pNetIf);
	#endif
	
	#if defined(TCPIP_STACK_USE_DHCP_SERVER)
	DHCPServerTask(pNetIf);
	#endif
	
	#if defined(TCPIP_STACK_USE_DNS) && DNS_CLIENT_VERSION_NO < 2
	DNSClientTask();
	#endif
	
	#if defined(TCPIP_STACK_USE_DNS_SERVER)
	DNSServerTask(pNetIf);
	#endif
	
	#if defined (TCPIP_STACK_USE_DYNAMICDNS_CLIENT)
	DDNSTask();
	#endif
	
	#if defined(TCPIP_STACK_USE_TELNET_SERVER)
	TelnetTask();
	#endif
	
	#if defined(TCPIP_STACK_USE_REBOOT_SERVER)
	RebootTask(pNetIf);
	#endif
	
	#if defined(TCPIP_STACK_USE_SNTP_CLIENT)
    SNTPClient(pNetIf);
	#endif
	
	#if defined(TCPIP_STACK_USE_SMTP_CLIENT)
	SMTPTask();
	#endif
}



/*********************************************************************
 * Function:        void TCPIP_STACK_NonEventApplications(NET_CONFIG* pNetIf)
 *
 * PreCondition:    TCPIP_STACK_Init() is already called.
 *
 * Input:           pNetIf   - interface configuration
 * 
 * Output:          Calls all loaded application modules.
 *
 * Side Effects:    None
 *
 * Note:            This function must be called periodically to
 *                  ensure timely responses.
 *                  It is meant for tasks that are non event triggerred but must always
 *                  be performed, periodically.
 ********************************************************************/
static void TCPIP_STACK_NonEventApplications(NET_CONFIG* pNetIf)
{
	#if defined(TCPIP_STACK_USE_UDP_PERFORMANCE_TEST)
	UDPPerformanceTask(pNetIf);
	#endif
	
	#if defined(TCPIP_STACK_USE_TCP_PERFORMANCE_TEST)
	TCPPerformanceTask();
	#endif
}


static void	TCPIPMacEventCB(void* hParam, TCPIP_EVENT event)
{
    TCPIPEventDcpt* pEvDcpt = tcpipEventDcpt + ((NET_CONFIG*)hParam)->netIfIx;
    
	pEvDcpt->activeEvents |= event;
    pEvDcpt->newTcpipEventAvlbl = 1;
	totTcpipEventsCnt++;
    
#if defined(TCPIP_STACK_USE_EVENT_NOTIFICATION)
    if(pEvDcpt->notifyEnabled && pEvDcpt->notifyHandler)
    {
        (*pEvDcpt->notifyHandler)(hParam, event, pEvDcpt->notifyfParam); // call user's notification 
    }
#endif  // defined(TCPIP_STACK_USE_EVENT_NOTIFICATION)
    
}



/*******************************************************************************
  Function:
    void    TCPIP_STACK_TickHandler(SYS_TICK currSysTick)

  Summary:
    Stack tick handler.

  Description:
    This function is called from within the System Tick ISR.
    It provides the Stack tick processing.
    It will call the notification handler registered with SYS_TICK_TimerCreate
    

  Precondition:
   System Tick should have been initialized
   and the Stack tick handler should have been registered with the SYS_TICK_TimerCreate. 

  Parameters:
    currSysTick   - current system tick value at the time of call
    
  Returns:
    None

  Remarks:   
    None
*****************************************************************************/
static void TCPIP_STACK_TickHandler(SYS_TICK currSysTick)  
{
    	newTcpipTickAvlbl++;
}





static void ProcessTCPIPMacErrorEvents(NET_CONFIG* pNetIf, TCPIP_EVENT activeEvent)
{
    newTcpipErrorEventCnt++;
}





/*********************************************************************
 * Function:        TCPIP_STACK_GetDefaultNet(void)
 *
 * PreCondition:    TCPIP stack should have been initialized by 
 *					TCPIP_STACK_Init()
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
TCPIP_NET_HANDLE TCPIP_STACK_GetDefaultNet(void)
{
    return tcpipDefIf.defaultNet;
}

// sets the default interface
// returns true if success,
// false if failed (the old interface does not change)
bool TCPIP_STACK_SetDefaultNet(TCPIP_NET_HANDLE netH)
{
    NET_CONFIG* pNewIf = _TCPIPStackHandleToNet(netH);
    if(pNewIf && pNewIf->Flags.bInterfaceEnabled)
    {
        tcpipDefIf.defaultNet = pNewIf;
        return true;
    }
    
    return false;
}

/*********************************************************************
 * Function:        _TCPIPStackIpAddToNet(IP_ADDR* pIpAddress, bool useDefault)
 *
 * PreCondition:    TCPIP stack should have been initialized by 
 *					TCPIP_STACK_Init()
 *
 * Input:           pIpAddress - pointer to an IP address
 * 
 *                  useDefault - when no interface is found,
 *                               if true: return the default interface
 *                               else return 0;
 *
 * Output:          Resolves a local IP address to a network interface.
 *               
 *
 * Side Effects:    None
 *
 * Note:            None
 ********************************************************************/
NET_CONFIG* _TCPIPStackIpAddToNet(IP_ADDR* pIpAddress, bool useDefault)
{
    NET_CONFIG* pNetIf = 0;
    
    if(pIpAddress && pIpAddress->Val != 0)
    {
        pNetIf = _TCPIPStackNetByAddress(pIpAddress);
    }

    if(pNetIf == 0 && useDefault)
    {
        pNetIf = tcpipDefIf.defaultNet;
    }
    
    return pNetIf;
}

/*********************************************************************
 * Function:        _TCPIPStackIpAddFromNet(IP_ADDR* pIpAddress)
 *
 * PreCondition:    TCPIP stack should have been initialized by 
 *					TCPIP_STACK_Init()
 *
 * Input:           pIpAddress - pointer to an IP address
 * 
 * Output:          Resolves a local IP address to a network interface
 *                  to which it belongs
 *               
 * Side Effects:    None
 *
 * Note:            None
 ********************************************************************/
NET_CONFIG* _TCPIPStackIpAddFromNet(IP_ADDR* pIpAddress)
{
    int netIx;
    NET_CONFIG* pIf;

    if(pIpAddress && pIpAddress->Val != 0)
    {
        for(netIx = 0, pIf = NetConfig ; netIx < tcpip_stack_ctrl_data.nIfs; netIx++, pIf++)
        {
            if(pIf->Flags.bInterfaceEnabled)
            {
                if(((pIf->MyIPAddr.Val & pIpAddress->Val) ^ pIf->MyMask.Val) == 0)
                {
                    return pIf;
                }
            }
        }
    }

    return 0;
}

/*********************************************************************
 * Function:        _TCPIPStackMacToNet(TCPIP_MAC_HANDLE hMac)
 *
 * PreCondition:    TCPIP stack should have been initialized by 
 *					TCPIP_STACK_Init()
 *
 * Input:           None
 *
 * Output:          Resolves a MAC Id to an NetConfig entry.
 *               
 *
 * Side Effects:    None
 *
 * Note:            The NetConfig entries match 1 to 1 the network interfaces
 *                  A more efficient algorithm to find MAC<->NetConfig entry correspondence
 *                  will be eventually added.
 ********************************************************************/
NET_CONFIG* _TCPIPStackMacToNet(TCPIP_MAC_HANDLE hMac)
{
    int netIx;
    NET_CONFIG* pNetIf;

    for(netIx = 0, pNetIf = NetConfig; netIx < tcpip_stack_ctrl_data.nIfs; netIx++, pNetIf++)
    {
        if(pNetIf->hIfMac == hMac)
        {
            return pNetIf;
        }
    }


    return 0;
}

/*********************************************************************
 * Function:        _TCPIPStackMacToNetIx(TCPIP_MAC_HANDLE hMac)
 *
 * PreCondition:    TCPIP stack should have been initialized by 
 *					TCPIP_STACK_Init()
 *
 * Input:           None
 *
 * Output:          Resolves a MAC to an NetConfig entry.
 *               
 *
 * Side Effects:    None
 *
 * Note:            The NetConfig entries match 1 to 1 the network interfaces
 *                  A more efficient algorithm to find MAC<->NetConfig entry correspondence
 *                  will be eventually added.
 ********************************************************************/
int _TCPIPStackMacToNetIx(TCPIP_MAC_HANDLE hMac)
{
    int netIx;
    NET_CONFIG* pNetIf;

    for(netIx = 0, pNetIf = NetConfig; netIx < tcpip_stack_ctrl_data.nIfs; netIx++, pNetIf++)
    {
        if(pNetIf->hIfMac == hMac)
        {
            return netIx;
        }
    }


    return -1;
}

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
int TCPIP_STACK_NetworksNo(void)
{
    return tcpip_stack_ctrl_data.nIfs;
}

/*********************************************************************
 * Function:        TCPIP_STACK_IxToNet(int netIx)
 *
 * PreCondition:    TCPIP stack should have been initialized by 
 *					TCPIP_STACK_Init()
 *
 * Input:           None
 *
 * Output:          Resolves an index to an NetConfig entry.
 *               
 *
 * Side Effects:    None
 *
 * Note:            The NetConfig entries match 1 to 1 the network interfaces
 ********************************************************************/
TCPIP_NET_HANDLE TCPIP_STACK_IxToNet(int netIx)
{
    if(netIx < tcpip_stack_ctrl_data.nIfs)
    {
        return NetConfig + netIx;
    }

    return 0;
}

int  TCPIP_STACK_NetIx(TCPIP_NET_HANDLE hNet)
{
    NET_CONFIG* pNetIf = _TCPIPStackHandleToNet(hNet);
    return _TCPIPStackNetIx(pNetIf);
}

/*********************************************************************
 * Function:        _TCPIPStackMacIdToNet(TCPIP_MAC_ID macId)
 *
 * PreCondition:    TCPIP stack should have been initialized by 
 *					TCPIP_STACK_Init()
 *
 * Input:           None
 *
 * Output:          Resolves an MAC id to a Net entry.
 *               
 *
 * Side Effects:    None
 *
 * Note:            In multi-homed hosts with multiple
 *                  interfaces of the same type,
 *                  the translation might not be unique.
 *                  The first match is returned!
 *
 ********************************************************************/
NET_CONFIG* _TCPIPStackMacIdToNet(TCPIP_MAC_ID macId)
{
    NET_CONFIG* pNetIf;

    if(macId != TCPIP_MAC_ID_NONE)
    {
        int netIx;
        for(netIx = 0, pNetIf = NetConfig; netIx < tcpip_stack_ctrl_data.nIfs; netIx++, pNetIf++)
        {
            if(pNetIf->macId == macId)
            {
                return pNetIf;
            }
        }
    }


    return 0;
    
}

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
TCPIP_NET_HANDLE TCPIP_STACK_NetHandle(const char* interface)
{
    return _TCPIPStackMacIdToNet(StringToMACId(interface));
}


/*********************************************************************
 * Function:        _TCPIPStackNetByAddress(IP_ADDR* pIpAddress)
 *
 * PreCondition:    TCPIP stack should have been initialized by 
 *					TCPIP_STACK_Init()
 *
 * Input:           pointer to an IP address
 *
 * Output:          The network interface pointer to which this ip
 *                  address belongs to.
 *                  NULL if not one of our addresses.
 *
 * Side Effects:    None
 *
 * Note:            A single network interface can support multiple IP addresses.
 *                  For now this feature is not implemented/supported.
 *
 ********************************************************************/
NET_CONFIG* _TCPIPStackNetByAddress(IP_ADDR* pIpAddress)
{
    int netIx;
    NET_CONFIG* pIf;

    for(netIx = 0, pIf = NetConfig ; netIx < tcpip_stack_ctrl_data.nIfs; netIx++, pIf++)
    {
        if(pIf->Flags.bInterfaceEnabled && pIf->MyIPAddr.Val == pIpAddress->Val)
        {
            return pIf;
        }
    }


    return 0;
}

/*********************************************************************
 * Function:        uint32_t TCPIP_STACK_NetAddress(TCPIP_NET_HANDLE netH)
 *
 * PreCondition:    TCPIP stack should have been initialized by TCPIP_STACK_Init()
 *
 * Input:           netH - Interface handle to get address of.
 *
 * Output:          The IP address of an interface.
 *               
 * Side Effects:    None
 *
 * Example:			TCPIP_NET_HANDLE netH = TCPIP_STACK_NetHandle("PIC32INT");
 * 					uint32_t ipAdd = TCPIP_STACK_NetAddress(netH);
 *
 * Note:            None
 ********************************************************************/
uint32_t TCPIP_STACK_NetAddress(TCPIP_NET_HANDLE netH)
{
    NET_CONFIG* pNetIf = _TCPIPStackHandleToNet(netH);
    return _TCPIPStackNetAddress(pNetIf);
}


bool TCPIP_STACK_SetNetAddress(TCPIP_NET_HANDLE netH, IP_ADDR* ipAddress, bool setDefault)
{
    NET_CONFIG* pNetIf = _TCPIPStackHandleToNet(netH);
    if(pNetIf)
    {
        pNetIf->MyIPAddr.Val = ipAddress->Val;
        if(setDefault)
        {
            pNetIf->DefaultIPAddr.Val = ipAddress->Val;
        }
    }

    return pNetIf != 0;
}
bool TCPIP_STACK_SetNetGatewayAddress(TCPIP_NET_HANDLE netH, IP_ADDR* ipAddress)
{
    NET_CONFIG* pNetIf = _TCPIPStackHandleToNet(netH);
    _TCPIPStackSetGatewayAddress(pNetIf, ipAddress);
    
    return pNetIf != 0;
}

uint32_t TCPIP_STACK_NetGatewayAddress(TCPIP_NET_HANDLE netH)
{
    NET_CONFIG* pNetIf = _TCPIPStackHandleToNet(netH);

    if(pNetIf && pNetIf->Flags.bInterfaceEnabled)
    {
        return pNetIf->MyGateway.Val;
    }

    return 0;
}

bool TCPIP_STACK_SetNetMask(TCPIP_NET_HANDLE netH, IP_ADDR* mask, bool setDefault)
{
    NET_CONFIG* pNetIf = _TCPIPStackHandleToNet(netH);
    if(pNetIf)
    {
        pNetIf->MyMask.Val = mask->Val;
        if(setDefault)
        {
            pNetIf->DefaultMask.Val = mask->Val;
        }
    }
    
    return pNetIf != 0;
}

uint32_t TCPIP_STACK_NetPriDNSAddress(TCPIP_NET_HANDLE netH)
{
    NET_CONFIG* pNetIf = _TCPIPStackHandleToNet(netH);

    if(pNetIf && pNetIf->Flags.bInterfaceEnabled)
    {
        return pNetIf->PrimaryDNSServer.Val;
    }

    return 0;
}

uint32_t TCPIP_STACK_NetSecondDNSAddress(TCPIP_NET_HANDLE netH)
{
    NET_CONFIG* pNetIf = _TCPIPStackHandleToNet(netH);

    if(pNetIf && pNetIf->Flags.bInterfaceEnabled)
    {
        return pNetIf->SecondaryDNSServer.Val;
    }

    return 0;
}

bool TCPIP_STACK_SetNetPriDNSAddress(TCPIP_NET_HANDLE netH, IP_ADDR* ipAddress)
{
    NET_CONFIG* pNetIf = _TCPIPStackHandleToNet(netH);
    _TCPIPStackSetPriDNSAddress(pNetIf, ipAddress);

    return pNetIf != 0;
}

bool TCPIP_STACK_SetNetSecondDNSAddress(TCPIP_NET_HANDLE netH, IP_ADDR* ipAddress)
{
    NET_CONFIG* pNetIf = _TCPIPStackHandleToNet(netH);
    _TCPIPStackSetSecondDNSAddress(pNetIf, ipAddress);

    return pNetIf != 0;
}

/*********************************************************************
 * Function:        uint32_t TCPIP_STACK_NetMask(TCPIP_NET_HANDLE netH)
 *
 * PreCondition:    None
 *
 * Input:           netH - Interface handle to get address of.
 *
 * Output:          if interface enabled then Value of subnet mask 
 * 					else 0
 *
 * Example:         TCPIP_NET_HANDLE netH = TCPIP_STACK_NetHandle("PIC32INT");
 * 					uint32_t subMask = TCPIP_STACK_NetMask(netH);
 *
 * Note:            None
 ********************************************************************/
uint32_t TCPIP_STACK_NetMask(TCPIP_NET_HANDLE netH)
{
    NET_CONFIG* pNetIf = _TCPIPStackHandleToNet(netH);
    return _TCPIPStackNetMask(pNetIf);
}


bool TCPIP_STACK_SetNetMacAddress(TCPIP_NET_HANDLE netH, const MAC_ADDR* pAddr)
{
    NET_CONFIG* pNetIf = _TCPIPStackHandleToNet(netH);
    if(pNetIf)
    {
        memcpy(pNetIf->MyMACAddr.v, pAddr->v, sizeof(pNetIf->MyMACAddr));
        return true;
    }
    
    return false;
}

const char* TCPIP_STACK_NetBIOSName(TCPIP_NET_HANDLE netH)
{
    NET_CONFIG* pNetIf = _TCPIPStackHandleToNet(netH);
    if(pNetIf && pNetIf->Flags.bInterfaceEnabled)
    {
        return (const char*)pNetIf->NetBIOSName;
    }

    return 0;

}

bool TCPIP_STACK_SetNetBIOSName(TCPIP_NET_HANDLE netH, const char* biosName)
{
    NET_CONFIG* pNetIf = _TCPIPStackHandleToNet(netH);
    if(pNetIf)
    {
        memcpy(pNetIf->NetBIOSName, biosName, sizeof(pNetIf->NetBIOSName));
        FormatNetBIOSName(pNetIf->NetBIOSName);
        return true;
    }

    return false;
}

bool TCPIP_STACK_NetSetEnableDHCP(TCPIP_NET_HANDLE netH, bool enable)
{
    NET_CONFIG* pNetIf = _TCPIPStackHandleToNet(netH);
    if(pNetIf)
    {
        pNetIf->Flags.bIsDHCPEnabled = enable;
        return true;
    }

    return false;

}

bool TCPIP_STACK_NetIsDHCPEnabled(TCPIP_NET_HANDLE netH)
{
    NET_CONFIG* pNetIf = _TCPIPStackHandleToNet(netH);
    
    return pNetIf && pNetIf->Flags.bIsDHCPEnabled;
}

const uint8_t* TCPIP_STACK_NetMacAddress(TCPIP_NET_HANDLE netH)
{
    NET_CONFIG* pNetIf = _TCPIPStackHandleToNet(netH);
    return _TCPIPStackNetMacAddress(pNetIf);
}

int TCPIP_STACK_NetInfoSize(TCPIP_NET_HANDLE hNet)
{
    NET_CONFIG* pNetIf = _TCPIPStackHandleToNet(hNet);
    if(pNetIf)
    {
        // constant for now; can change later
        return sizeof(*pNetIf);
    }

    return 0;
    
}

TCPIP_NET_HANDLE TCPIP_STACK_CreateNetInfo(TCPIP_NET_HANDLE hNet, uint8_t* buffer, int buffSize, bool copyInfo)
{
    NET_CONFIG* pNewIf;
    NET_CONFIG* pNetIf = _TCPIPStackHandleToNet(hNet);
    if(!pNetIf || buffSize < sizeof(*pNetIf) )
    {
        return 0;
    }

    pNewIf = (NET_CONFIG*)buffer;
    
    if(copyInfo)
    {
        memcpy(pNewIf, pNetIf, sizeof(*pNetIf));
    }
    else
    {
        memset(pNewIf, 0x0, sizeof(*pNetIf));
    }
    
    // fake interface enabled so that all managing functions work OK
    pNewIf->Flags.bInterfaceEnabled = true;

    return pNewIf;
}



/*********************************************************************
 * Function:        TCPIP_STACK_NetBcastAddress(TCPIP_NET_HANDLE netH)
 *
 * PreCondition:    TCPIP stack should have been initialized by 
 *					TCPIP_STACK_Init()
 *
 * Input:           interface handle to get address of
 *
 * Output:          The broadcast IP address of an interface.
 *               
 *
 * Side Effects:    None
 *
 * Note:           
 ********************************************************************/
uint32_t TCPIP_STACK_NetBcastAddress(TCPIP_NET_HANDLE netH)
{
    NET_CONFIG* pNetIf = _TCPIPStackHandleToNet(netH);
    if(pNetIf->Flags.bInterfaceEnabled)
    {
        return (pNetIf->MyIPAddr.Val & pNetIf->MyMask.Val) | ~pNetIf->MyMask.Val; 
    }

    return 0;


}

bool TCPIP_STACK_IsNetUp(TCPIP_NET_HANDLE netH)
{
    NET_CONFIG* pNetIf = _TCPIPStackHandleToNet(netH);
    return _TCPIPStackIsNetUp(pNetIf);
}

/*********************************************************************
 * Function:        TCPIP_STACK_VerifyPktIf((NET_CONFIG* pNetIf, IP_ADDR* pktDestIP)
 *
 * PreCondition:    TCPIP stack should have been initialized by 
 *					TCPIP_STACK_Init()
 *
 * Input:           pNetIf      - interface receiveing a packet
 *                  pktDestIP   - address of packet destination IP
 *
 * Output:          true if valid packet
 *                  false otherwise.
 *               
 *
 * Side Effects:    None
 *
 * Note:            Discard any packets received that were generated by ourself?
 *                  - In structured Wi-Fi networks, the Access Point rebroadcasts
 *                    our broadcast and multicast packets, causing self-reception
 *                    to occur unless filtered out.
 *                    Should use remoteNode info for that.
 *                  - On the other hand, we want to be able to send packets from one interface
 *                    to another or from one IP add to another even belonging to same interface.
 *                  - Should be discarded at WiFi level, if needed!
 ********************************************************************/
static bool TCPIP_STACK_VerifyPktIf(NET_CONFIG* pNetIf, IP_ADDR* pktDestIP)
{
    if( 
        _TCPIPStackIsAddressOfNet(pNetIf, pktDestIP) || // unicast to me
        _TCPIPStackIsBcastAddress(pNetIf, pktDestIP) || // net or limited bcast
        TCPIP_HELPER_IsMcastAddress(pktDestIP)                 // multicast
        )
    {
        return true;
    }

    return false;


}

/*********************************************************************
 * Function:        bool InitNetConfig(const TCPIP_NETWORK_CONFIG* pUsrConfig, int nNets)
 *
 * PreCondition:    MPFSInit() is already called.
 *
 * Input:           pUsrConfig  - pointer to user configurations
 *                  nNets       - number of networks configurations provided
 *
 * Output:          Write/Read non-volatile config variables.
 *
 * Side Effects:    None
 *
 * Overview:        None
 *
 * Note:            None
 ********************************************************************/
static bool InitNetConfig(const TCPIP_NETWORK_CONFIG* pUsrConfig, int nNets)
{
    int     ix;
    NET_CONFIG* pNetConfig;
#if defined(_TCPIP_STACK_CHECK_STORAGE_VERSION)
    TCPIP_STORAGE_HANDLE hStg;      // tcpip storage handle
    int                  storageIx;
    TCPIP_UINT16_VAL     buildChecksum, stgChecksum;
    int                  stgLabelSize;
    bool                 stgWriteNeeded, stgWriteFail;
#endif  // defined(_TCPIP_STACK_CHECK_STORAGE_VERSION)


    for(ix =0, pNetConfig = NetConfig; ix < nNets; ix++, pNetConfig++, pUsrConfig++)
    {
        if(!LoadDefaultConfig(pUsrConfig, pNetConfig))
        {
            SYS_ERROR(SYS_ERROR_ERROR, "Default Flash Network configuration load failed");
            return false;
        }
    }

#if defined(_TCPIP_STACK_CHECK_STORAGE_VERSION)
    // see if we have a storage to use
    //
    if(!TCPIP_STORAGE_Init(0) || (hStg = TCPIP_STORAGE_Open(0, true)) == 0)
    {
        SYS_ERROR(SYS_ERROR_WARN, "Failed to open the storage");
        return true;    // continue with the default configuration
    }


    stgWriteNeeded = stgWriteFail = false;
    // calculate the checksum of the current network settings
    // as specified by the current build
    buildChecksum.Val = CalcIPChecksum((uint8_t*)NetConfig, sizeof(NET_CONFIG)*nNets);
    stgLabelSize = TCPIP_STORAGE_GetLabel(hStg, stgChecksum.v, sizeof(stgChecksum.Val));

    if(stgLabelSize != sizeof(stgChecksum.Val) || buildChecksum.Val != stgChecksum.Val)
    {   // checksum mismatch or different build/label, etc
        stgWriteNeeded = true;
        TCPIP_STORAGE_SetLabel(hStg, buildChecksum.v, sizeof(buildChecksum.Val));
    }

    // OK, valid storage; write/read entries
    for(ix =0, pNetConfig = NetConfig; ix < nNets; ix++, pNetConfig++)
    {
        if(stgWriteNeeded ||  (storageIx = TCPIP_STORAGE_FindEntry(hStg, pNetConfig)) == -1)
        {   // either re-write needed or this interface is missing anyway from storage
            // save this entry
            if(!TCPIP_STORAGE_WriteEntry(hStg, pNetConfig, true))
            {   // storage save failed. abort
                stgWriteFail = true;
                break;
            }
        }
        else
        {   // the entry for this interface exists; load it
            TCPIP_STORAGE_ReadEntryIx(hStg, storageIx, pNetConfig);
        }
    }
        

    // check the result
    if(stgWriteFail)
    {
        SYS_ERROR(SYS_ERROR_WARN, "Failed to save entries in the storage");
    }
    else
    {   // successfully saved for this network interface
        SYS_CONSOLE_MESSAGE("Stack Configuration Storage updated");
    }
    // If we get down here, it means the EEPROM/Flash has valid contents 
    // and either matches the const defaults or previously matched and 
    // was run-time reconfigured by the user.  In this case, we shall 
    // use the contents loaded from EEPROM/Flash.
    
    TCPIP_STORAGE_Close(hStg);
#endif  // defined(_TCPIP_STACK_CHECK_STORAGE_VERSION)
 
    
    return true;
}

    
/*********************************************************************
 * Function:        bool LoadDefaultConfig(const TCPIP_NETWORK_CONFIG* pUsrConfig, NET_CONFIG* pNetIf)
 *
 * PreCondition:    None
 *
 * Input:           pUsrConfig  - pointer to configurations to use
 *                  pNetIf     - network interface to default configure
 *
 * Output:          true if the default configuration sucessfully loaded,
 *                  false otherwise
 *
 * Side Effects:    None
 *
 * Overview:        Loads the default values (flash) for the network configuration
 *
 * Note:            None
 ********************************************************************/
static bool LoadDefaultConfig(const TCPIP_NETWORK_CONFIG* pUsrConfig, NET_CONFIG* pNetIf)
{

    memset(pNetIf, 0, sizeof(*pNetIf));

    pNetIf->Flags.bIsDHCPEnabled = true;
    pNetIf->Flags.bInConfigMode = true;
    // memcpy((void*)&pNetIf->MyMACAddr, (const void*)SerializedMACAddress, sizeof(NetConfig[0].MyMACAddr));
    //	{
    //		_prog_addressT MACAddressAddress;
    //		MACAddressAddress.next = 0x157F8;
    //		_memcpy_p2d24((char*)&NetConfig.MyMACAddr, MACAddressAddress, sizeof(NetConfig.MyMACAddr));
    //	}


    StringToMACAddress(pUsrConfig->macAddr, pNetIf->MyMACAddr.v);
    TCPIP_HELPER_StringToIPAddress(pUsrConfig->ipAddr, &pNetIf->MyIPAddr);
    pNetIf->DefaultIPAddr.Val = pNetIf->MyIPAddr.Val;
    TCPIP_HELPER_StringToIPAddress(pUsrConfig->ipMask, &pNetIf->MyMask);
    pNetIf->DefaultMask.Val = pNetIf->MyMask.Val;

    TCPIP_HELPER_StringToIPAddress(pUsrConfig->gateway, &pNetIf->MyGateway);
    TCPIP_HELPER_StringToIPAddress(pUsrConfig->priDNS, &pNetIf->PrimaryDNSServer);
    TCPIP_HELPER_StringToIPAddress(pUsrConfig->secondDNS, &pNetIf->SecondaryDNSServer);

    memcpy(pNetIf->NetBIOSName, pUsrConfig->hostName, sizeof(NetConfig[0].NetBIOSName));

    pNetIf->macId = StringToMACId(pUsrConfig->interface);
    if(pNetIf->macId == TCPIP_MAC_ID_NONE)
    {
        return false;   // no such MAC interface
    }
    
     // init for storage purposes; proper values will be assigned
    _TCPIPStackClearDynNetSettings(pNetIf);

        // SNMP Community String configuration
        #if defined(TCPIP_STACK_USE_SNMP_SERVER)
        {
            uint8_t i;
            static const char * const cReadCommunities[] = SNMP_READ_COMMUNITIES;
            static const char * const cWriteCommunities[] = SNMP_WRITE_COMMUNITIES;
            const char * strCommunity;

            for(i = 0; i < SNMP_MAX_COMMUNITY_SUPPORT; i++)
            {
                // Get a pointer to the next community string
                strCommunity = cReadCommunities[i];
                if(i >= sizeof(cReadCommunities)/sizeof(cReadCommunities[0]))
                    strCommunity = "";

                // Ensure we don't buffer overflow.  If your code gets stuck here, 
                // it means your SNMP_COMMUNITY_MAX_LEN definition in tcpip_config.h 
                // is either too small or one of your community string lengths 
                // (SNMP_READ_COMMUNITIES) are too large.  Fix either.
                if(strlen(strCommunity) >= sizeof(NetConfig[0].readCommunity[0]))
                    return false; 

                // Copy string into NetConfig
            strcpy((char*)pNetIf->readCommunity[i], strCommunity);

                // Get a pointer to the next community string
                strCommunity = cWriteCommunities[i];
                if(i >= sizeof(cWriteCommunities)/sizeof(cWriteCommunities[0]))
                    strCommunity = "";

                // Ensure we don't buffer overflow.  If your code gets stuck here, 
                // it means your SNMP_COMMUNITY_MAX_LEN definition in tcpip_config.h 
                // is either too small or one of your community string lengths 
                // (SNMP_WRITE_COMMUNITIES) are too large.  Fix either.
                if(strlen(strCommunity) >= sizeof(NetConfig[0].writeCommunity[0]))
                    return false; 

                // Copy string into NetConfig
            strcpy((char*)pNetIf->writeCommunity[i], strCommunity);
            }
        }
        #endif

    // Load the default NetBIOS Host Name
    //	memcpy(NetConfig.NetBIOSName, (const void*)WF_DEFAULT_HOST_NAME, 16);
    FormatNetBIOSName(pNetIf->NetBIOSName);

#if defined(TCPIP_IF_MRF24W)
    if(pNetIf->macId == TCPIP_MAC_ID_MRF24W)
    { // init the MRF24W interface 
        // Load the default SSID Name
        if(sizeof(WF_DEFAULT_SSID_NAME) > sizeof(NetConfig[0].MySSID))
        {
            return false;
        }
        memcpy(pNetIf->MySSID, (const void*)WF_DEFAULT_SSID_NAME, sizeof(WF_DEFAULT_SSID_NAME));
        pNetIf->SsidLength = sizeof(WF_DEFAULT_SSID_NAME) - 1;

#if defined(TCPIP_STACK_USE_EZ_CONFIG)
        pNetIf->Flags.bWFEasyConfig = 1;
#endif  // defined(TCPIP_STACK_USE_EZ_CONFIG)

        pNetIf->SecurityMode = WF_DEFAULT_WIFI_SECURITY_MODE;
        pNetIf->WepKeyIndex  = WF_DEFAULT_WEP_KEY_INDEX;

#if (WF_DEFAULT_WIFI_SECURITY_MODE == WF_SECURITY_OPEN)
        memset(pNetIf->SecurityKey, 0x00, sizeof(pNetIf->SecurityKey));
        pNetIf->SecurityKeyLength = 0;

#elif WF_DEFAULT_WIFI_SECURITY_MODE == WF_SECURITY_WEP_40
        memcpy(pNetIf->SecurityKey, (const void*)WF_DEFAULT_WEP_KEYS_40, sizeof(WF_DEFAULT_WEP_KEYS_40) - 1);
        pNetIf->SecurityKeyLength = sizeof(WF_DEFAULT_WEP_KEYS_40) - 1;

#elif WF_DEFAULT_WIFI_SECURITY_MODE == WF_SECURITY_WEP_104
        memcpy(pNetIf->SecurityKey, (const void*)WF_DEFAULT_WEP_KEYS_104, sizeof(WF_DEFAULT_WEP_KEYS_104) - 1);
        pNetIf->SecurityKeyLength = sizeof(WF_DEFAULT_WEP_KEYS_104) - 1;

#elif (WF_DEFAULT_WIFI_SECURITY_MODE == WF_SECURITY_WPA_WITH_KEY)       || \
        (WF_DEFAULT_WIFI_SECURITY_MODE == WF_SECURITY_WPA2_WITH_KEY)      || \
        (WF_DEFAULT_WIFI_SECURITY_MODE == WF_SECURITY_WPA_AUTO_WITH_KEY)
        memcpy(pNetIf->SecurityKey, (const void*)WF_DEFAULT_PSK, sizeof(WF_DEFAULT_PSK) - 1);
        pNetIf->SecurityKeyLength = sizeof(WF_DEFAULT_PSK) - 1;

#elif (WF_DEFAULT_WIFI_SECURITY_MODE == WF_SECURITY_WPA_WITH_PASS_PHRASE)     || \
        (WF_DEFAULT_WIFI_SECURITY_MODE == WF_SECURITY_WPA2_WITH_PASS_PHRASE)    || \
        (WF_DEFAULT_WIFI_SECURITY_MODE == WF_SECURITY_WPA_AUTO_WITH_PASS_PHRASE)
        memcpy(pNetIf->SecurityKey, (const void*)WF_DEFAULT_PSK_PHRASE, sizeof(WF_DEFAULT_PSK_PHRASE) - 1);
        pNetIf->SecurityKeyLength = sizeof(WF_DEFAULT_PSK_PHRASE) - 1;

#else 
#error "No security defined"
#endif /* WF_DEFAULT_WIFI_SECURITY_MODE */
    }

#endif


    return true;
}


#if defined(TCPIP_STACK_USE_EVENT_NOTIFICATION)

// Stack external event notification support
// Limited support for now

TCPIP_EVENT_RESULT TCPIP_STACK_SetNotifyEvents(const void* h, TCPIP_EVENT tcpipEvents)
{
    int netIx = ((NET_CONFIG*)h)->netIfIx;
    tcpipEventDcpt[netIx].notifyEnabled = 1;
    return TCPIP_EVRES_OK; 
}

TCPIP_EVENT_RESULT TCPIP_STACK_ClearNotifyEvents(const void* h, TCPIP_EVENT tcpipEvents)
{
    int netIx = ((NET_CONFIG*)h)->netIfIx;
    tcpipEventDcpt[netIx].notifyEnabled = 0;
    return TCPIP_EVRES_OK; 
}

TCPIP_EVENT TCPIP_STACK_GetPending(const void* h)
{
    return MACEventGetPending(((NET_CONFIG*)h)->hIfMac, TCPIP_MAC_EVGROUP_ALL);
}

TCPIP_EVENT_RESULT TCPIP_STACK_SetNotifyHandler(const void* h, pTCPIPNotifyF notifyHandler, void* notifyfParam)
{
    int netIx = ((NET_CONFIG*)h)->netIfIx;

    tcpipEventDcpt[netIx].notifyHandler = notifyHandler;   // set user's notification
    tcpipEventDcpt[netIx].notifyfParam = notifyfParam; 

    return TCPIP_EVRES_OK; 
}

#endif  // defined(TCPIP_STACK_USE_EVENT_NOTIFICATION)

static const TCPIP_STACK_MODULE_CONFIG* TcpipStackFindModuleData(TCPIP_STACK_MODULE moduleId, const TCPIP_STACK_MODULE_CONFIG* pModConfig, int nModules)
{
    while(nModules--)
    {
        if(pModConfig->moduleId == moduleId)
        {
            return pModConfig;
        }
        pModConfig++;
    }

    return 0;
}

// clear the dynamic part of the NET_CONFIG
void  _TCPIPStackClearDynNetSettings(NET_CONFIG* pNetIf)
{
    pNetIf->hIfMac = 0;
    pNetIf->netIfIx = 0;   
    pNetIf->powerMode = 0;
    pNetIf->linkPrev = 0;
}

int  _TCPIPStackNetIx(NET_CONFIG* pNetIf)
{
    if(pNetIf)
    {
        return pNetIf->netIfIx;
    }
    return -1;
}


uint32_t  _TCPIPStackNetAddress(NET_CONFIG* pNetIf)
{
    if(pNetIf && pNetIf->Flags.bInterfaceEnabled)
    {
        return pNetIf->MyIPAddr.Val;
    }
    return 0;
}

void  _TCPIPStackSetNetAddress(NET_CONFIG* pNetIf, IP_ADDR* ipAddress)
{
    if(pNetIf)
    {
        pNetIf->MyIPAddr.Val = ipAddress->Val;
    }
}
void  _TCPIPStackSetDefaultAddress(NET_CONFIG* pNetIf)
{
    if(pNetIf)
    {
        pNetIf->MyIPAddr.Val = pNetIf->DefaultIPAddr.Val;
    }
}



uint32_t  _TCPIPStackNetMask(NET_CONFIG* pNetIf)
{
    if(pNetIf && pNetIf->Flags.bInterfaceEnabled)
    {
        return pNetIf->MyMask.Val;
    }
    return 0;
}

void  _TCPIPStackSetNetMask(NET_CONFIG* pNetIf, IP_ADDR* mask)
{
    if(pNetIf)
    {
        pNetIf->MyMask.Val = mask->Val;
    }
}

void  _TCPIPStackSetGatewayAddress(NET_CONFIG* pNetIf, IP_ADDR* ipAddress)
{
    if(pNetIf)
    {
        pNetIf->MyGateway.Val = ipAddress->Val;
    }
}
void  _TCPIPStackSetPriDNSAddress(NET_CONFIG* pNetIf, IP_ADDR* ipAddress)
{
    if(pNetIf)
    {
        pNetIf->PrimaryDNSServer.Val = ipAddress->Val;
    }
}

void  _TCPIPStackSetSecondDNSAddress(NET_CONFIG* pNetIf, IP_ADDR* ipAddress)
{
    if(pNetIf)
    {
        pNetIf->SecondaryDNSServer.Val = ipAddress->Val;
    }
}

NET_CONFIG*         _TCPIPStackNetByAddress(IP_ADDR* pIpAddress);


bool  _TCPIPStackIsAddressOfNet( NET_CONFIG* pNetIf, IP_ADDR* pIpAdd)
{
    if(pNetIf && pNetIf->Flags.bInterfaceEnabled)
    {
        return pNetIf->MyIPAddr.Val == pIpAdd->Val;
    }
    return false;
}

// detects net-directed bcast
bool  _TCPIPStackIsNetBcastAddress(NET_CONFIG* pNetIf, IP_ADDR* pIpAdd)
{
    if(pNetIf && pNetIf->Flags.bInterfaceEnabled)
    {
       return (pIpAdd->Val == ((pNetIf->MyIPAddr.Val & pNetIf->MyMask.Val) | ~pNetIf->MyMask.Val)); 
    }
    return false;
}

// detects limited or net-directed bcast
bool  _TCPIPStackIsBcastAddress(NET_CONFIG* pNetIf, IP_ADDR* pIpAdd)
{
    if(pNetIf && pNetIf->Flags.bInterfaceEnabled)
    {
       return (TCPIP_HELPER_IsBcastAddress(pIpAdd) ||  _TCPIPStackIsNetBcastAddress(pNetIf, pIpAdd)); 
    }
    return false;
}

bool  _TCPIPStackIsNetUp(NET_CONFIG* pNetIf)
{
    if(pNetIf && pNetIf->Flags.bInterfaceEnabled)
    {
        return true;
    }
    return false;
}


TCPIP_MAC_ID  _TCPIPStackNetMacId(NET_CONFIG* pNetIf)
{
    return pNetIf?pNetIf->macId:TCPIP_MAC_ID_NONE;
}


TCPIP_MAC_HANDLE  _TCPIPStackNetToMac(NET_CONFIG* pNetIf)
{
    return pNetIf?pNetIf->hIfMac:0;
}



const uint8_t*  _TCPIPStackNetMacAddress(NET_CONFIG* pNetIf)
{
    if(pNetIf && pNetIf->Flags.bInterfaceEnabled)
    {
        return pNetIf->MyMACAddr.v;
    }

    return 0;
}

NET_CONFIG*  _TCPIPStackHandleToNet(TCPIP_NET_HANDLE hNet)
{
    // do some checking
    // if #debug enabled, etc
    return hNet?(NET_CONFIG*)hNet:0;
}





