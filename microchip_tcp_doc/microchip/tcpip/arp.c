/*******************************************************************************
  Address Resolution Protocol (ARP) Client and Server

  Summary:
    ARP implementation file
    
  Description:
    This source file contains the functions and storage of the 
    ARP routines
    
    Provides IP address to Ethernet MAC address translation
    Reference: RFC 826
*******************************************************************************/

/*******************************************************************************
FileName:   arp.c
Copyright © 2011 released Microchip Technology Inc.  All rights
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


#include "tcpip_private.h"

#include "arp_private.h"
#include "hash_fnv.h"

/****************************************************************************
  Section:
    Constants and Variables
  ***************************************************************************/
// ARP caches per interface
static ARP_CACHE_DCPT       arpCache[TCPIP_NETWORK_INTERFACES] = { {0} };

static int                  arpTickPending = 0;        // ARP processing tick
static uint32_t             arpTimeSeconds = 0;        // coarse ARP time keeping, seconds
static SystemTickHandle     arpTimerHandle = 0;

static MAC_ADDR             arpBcastAdd = { {0xff, 0xff, 0xff, 0xff, 0xff, 0xff} };

#ifdef TCPIP_STACK_USE_ZEROCONF_LINK_LOCAL
#define MAX_REG_APPS            2           // MAX num allowed registrations of Modules/Apps
static struct arp_app_callbacks reg_apps[MAX_REG_APPS]; // Call-Backs storage for MAX of two Modules/Apps

#endif

// ARP event registration

static const void*      arpMemH = 0;        // memory allocation handle

static SINGLE_LIST      arpRegisteredUsers = { 0 };

/****************************************************************************
  Section:
    Helper Function Prototypes
  ***************************************************************************/

static void ARPTmoHandler(SYS_TICK currSysTick);

static bool ARP_SendIfPkt(NET_CONFIG* pIf, uint16_t oper, uint32_t srcIP, uint32_t dstIP, MAC_ADDR* dstMAC);

#ifdef TCPIP_STACK_USE_ZEROCONF_LINK_LOCAL
static void ARPProcessRxPkt(NET_CONFIG* pIf, ARP_PACKET* packet);
#endif

static void SwapARPPacket(ARP_PACKET* p);

static void         _ARPUpdateEntry(NET_CONFIG* pIf, ARP_HASH_ENTRY* arpHE, MAC_ADDR* hwAdd);
static ARP_RESULT   _ARPAddCompleteEntry(NET_CONFIG* pIf, IP_ADDR* pIPAddr, MAC_ADDR* hwAdd);
    
static void         _ARPCleanupCache(ARP_CACHE_DCPT* pArpDcpt);
static void         _ARPCleanupClients(void);

static void         _ARPNotifyClients(NET_CONFIG* pNetIf, const MAC_ADDR* MACAddr, ARP_EVENT_TYPE evType);

/*static __inline__*/static  void /*__attribute__((always_inline))*/ _ARPSetEntry(ARP_HASH_ENTRY* arpHE, ARP_ENTRY_FLAGS newFlags,
                                                                      MAC_ADDR* hwAdd, SINGLE_LIST* addList)
{
    arpHE->hEntry.flags.value &= ~ARP_FLAG_ENTRY_VALID_MASK;
    arpHE->hEntry.flags.value |= newFlags;
    
    if(hwAdd)
    {
        arpHE->hwAdd = *hwAdd;
    }
    
    arpHE->tInsert = arpTimeSeconds;
    arpHE->nRetries = 1;
    if(addList)
    {
        SingleListAddTail(addList, (SGL_LIST_NODE*)&arpHE->next);
    }
}


// re-inserts at the tail, makes the entry fresh
/*static __inline__*/static  void /*__attribute__((always_inline))*/ _ARPRefreshEntry(ARP_HASH_ENTRY* arpHE, SINGLE_LIST* pL)
{
    SingleListRemoveNode(pL, (SGL_LIST_NODE*)&arpHE->next);
    arpHE->tInsert = arpTimeSeconds;
    SingleListAddTail(pL, (SGL_LIST_NODE*)&arpHE->next);
}

/*static __inline__*/static  void /*__attribute__((always_inline))*/ _ARPRemoveCacheEntries(ARP_CACHE_DCPT* pArpDcpt)
{

    if(pArpDcpt->cacheDcpt)
    {
        OAHashRemoveAll(pArpDcpt->cacheDcpt);
        SingleListDelete(&pArpDcpt->incompleteList);
        SingleListDelete(&pArpDcpt->completeList);
        SingleListDelete(&pArpDcpt->permList);
    }
}


/****************************************************************************
  Section:
    Function Implementations
  ***************************************************************************/
#ifdef TCPIP_STACK_USE_ZEROCONF_LINK_LOCAL
/************ User Application APIs ****************************************/

/*****************************************************************************
  Function:
    int8_t ARPRegisterCallbacks(struct arp_app_callbacks *app)

  Summary:
    Registering callback with ARP module to get notified about certian events.
    
  Description:
    This function allows end user application to register with callbacks, which
    will be called by ARP module to give notification to user-application about 
    events occurred at ARP layer. For ex: when a ARP-packet is received, which is
    conflicting with our own pair of addresses (MAC-Address and IP-address).
    This is an extension for zeroconf protocol implementation (ZeroconfLL.c)

  Precondition:
    None

  Parameters:
    app - ARP-Application callbacks structure supplied by user-application 
    
  Returns:
    id > 0 - Returns non-negative value that represents the id of registration
             The same id needs to be used in de-registration
    -1     - When registered applications exceed MAX_REG_APPS and there is no
             free slot for registration
 
  ***************************************************************************/
int8_t ARPRegisterCallbacks(struct arp_app_callbacks *app)
{
    uint8_t i;
    for(i=0; i<MAX_REG_APPS; i++)
    {
        if(!reg_apps[i].used)
        {
            reg_apps[i].ARPPkt_notify = app->ARPPkt_notify;
            reg_apps[i].used = 1;
            return (i+1); // Return Code. Should be used in deregister.
        }
    }
    return -1; // No space for registration
}

/*****************************************************************************
  Function:
    bool ARPDeRegisterCallbacks(int8_t reg_id)

  Summary:
    De-Registering callbacks with ARP module that are registered previously.
    
  Description:
    This function allows end user-application to de-register with callbacks, 
    which were registered previously.
    This is called by user-application, when its no longer interested in 
    notifications from ARP-Module. This allows the other application to get 
    registered with ARP-module.   

  Precondition:
    None

  Parameters:
    reg_id - Registration-id returned in ARPRegisterCallbacks call
    
  Returns:
    true  - On success
    false - Failure to indicate invalid reg_id  
  ***************************************************************************/ 
bool ARPDeRegisterCallbacks(int8_t reg_id)
{
    if(reg_id <= 0 || reg_id > MAX_REG_APPS)
        return false;

    reg_apps[reg_id-1].used = 0; // To indicate free slot for registration
    return true;
}


/*****************************************************************************
  Function:
    void ARPProcessRxPkt(NET_CONFIG* pIf, ARP_PACKET* packet)

  Summary:
    Processes Received-ARP packet (ARP request/Reply).
    
  Description:
    This function is to pass-on the ARP-packet to registered application,
    with the notification of Rx-ARP packet. 

  Precondition:
    ARP packet is received completely from MAC

  Parameters:
    pIf   - interface to use 
    packet - Rx packet to be processed     

  Returns:
    None   
  ***************************************************************************/
static void ARPProcessRxPkt(NET_CONFIG* pIf, ARP_PACKET* packet)
{
    uint8_t pass_on = 0; // Flag to indicate whether need to be forwarded
    uint8_t i;

    // Probing Stage
    if(pIf->MyIPAddr.Val == 0x00)
    {
        pass_on = 1; // Pass to Registered-Application for further processing        
    }
    else if(pIf->MyIPAddr.Val)
    {
        /* Late-conflict */
        if(packet->SenderIPAddr.Val == pIf->MyIPAddr.Val)
        {
            pass_on = 1;
        }
    }
    if(pass_on)
    {
    
        for(i =0; i< MAX_REG_APPS; i++)
        {
            if(reg_apps[i].used)
            {
                reg_apps[i].ARPPkt_notify(packet->SenderIPAddr.Val,
                                      packet->TargetIPAddr.Val,
                                      &packet->SenderMACAddr,
                                      &packet->TargetMACAddr,
                                      packet->Operation);                
            }
        }
    }
}

#endif  // TCPIP_STACK_USE_ZEROCONF_LINK_LOCAL


/*****************************************************************************
  Function:
    static bool ARP_SendIfPkt(NET_CONFIG* pIf, uint16_t oper, uint32_t srcIP, uint32_t dstIP, MAC_ADDR* dstMAC)

  Description:
    Writes an ARP packet to the MAC using the interface pointer for src IP and MAC address.

  Precondition:
    None

  Parameters:

  Return Values:
    true - The ARP packet was generated properly
    false - otherwise

  
  ***************************************************************************/
static bool ARP_SendIfPkt(NET_CONFIG* pIf, uint16_t oper, uint32_t srcIP, uint32_t dstIP, MAC_ADDR* dstMAC)
{
    ARP_PACKET       packet;
    TCPIP_MAC_HANDLE hMac;


    packet.HardwareType  = HW_ETHERNET;
    packet.Protocol      = ARP_IP;
    packet.MACAddrLen    = sizeof(MAC_ADDR);
    packet.ProtocolLen   = sizeof(IP_ADDR);
    packet.Operation = oper;
    
    packet.SenderMACAddr = pIf->MyMACAddr;
    packet.SenderIPAddr.Val  = srcIP;
    packet.TargetMACAddr = *dstMAC;
    packet.TargetIPAddr.Val  = dstIP;

    SwapARPPacket(&packet);

    
    hMac = _TCPIPStackNetToMac(pIf);
    if(!MACIsTxReady(hMac))
    {
        return false;
    }
    
    MACSetWritePtr(hMac, MACGetTxBaseAddr(hMac));
    MACPutHeader(hMac, &packet.TargetMACAddr, ETHERTYPE_ARP, sizeof(packet));
    MACPutArray(hMac, (uint8_t*)&packet, sizeof(packet));
    MACFlush(hMac);
    
    return true;
}

/*****************************************************************************
  Function:
    static void _ARPUpdateEntry(NET_CONFIG* pIf, ARP_HASH_ENTRY* arpHE, MAC_ADDR* hwAdd)

  Description:
    Updates the info for an existing ARP cache entry

  Precondition:
    None

  Parameters:
    pIf             - interface
    arpHE           - particular cache entry to be updated
    hwAdd           - the (new) hardware address

  Return Values:
    None
  ***************************************************************************/
static void _ARPUpdateEntry(NET_CONFIG* pIf, ARP_HASH_ENTRY* arpHE, MAC_ADDR* hwAdd)
{
    ARP_EVENT_TYPE evType; 
    ARP_CACHE_DCPT  *pArpDcpt;
    
    pArpDcpt = arpCache + _TCPIPStackNetIx(pIf);
    if((arpHE->hEntry.flags.value & ARP_FLAG_ENTRY_PERM) == 0)
    {   

        if((arpHE->hEntry.flags.value & ARP_FLAG_ENTRY_COMPLETE) == 0)
        {   // was waiting for this one, it was queued
            evType = ARP_EVENT_SOLVED;
            SingleListRemoveNode(&pArpDcpt->incompleteList, (SGL_LIST_NODE*)&arpHE->next);
        }
        else
        {   // completed entry, but now updated
            evType = ARP_EVENT_UPDATED;
            SingleListRemoveNode(&pArpDcpt->completeList, (SGL_LIST_NODE*)&arpHE->next);
        }
        
        // move to tail, updated
        _ARPSetEntry(arpHE, ARP_FLAG_ENTRY_COMPLETE, hwAdd, &pArpDcpt->completeList);
    }
    else
    {   // permanent entries are not updated
        evType = ARP_EVENT_PERM_UPDATE;
    }

    _ARPNotifyClients(pIf, &arpHE->hwAdd, evType);

}


/*****************************************************************************
  Function:
    static ARP_RESULT _ARPAddCompleteEntry(NET_CONFIG* pIf, IP_ADDR* pIPAddr, MAC_ADDR* hwAdd)

  Description:
    Updates the info for an existing ARP cache entry

  Precondition:
    None

  Parameters:
    pIf             - network interface 
    arpHE           - particular cache entry to be updated
    hwAdd           - the (new) hardware address

  Return Values:
    ARP_RES_CACHE_FULL  - cache full error
    ARP_RES_OK          - success
  ***************************************************************************/
static ARP_RESULT _ARPAddCompleteEntry(NET_CONFIG* pIf, IP_ADDR* pIPAddr, MAC_ADDR* hwAdd)
{
    ARP_CACHE_DCPT  *pArpDcpt;
    ARP_HASH_ENTRY  *arpHE;
    OA_HASH_ENTRY   *hE;

    pArpDcpt = arpCache + _TCPIPStackNetIx(pIf);
    
    hE = OAHashLookUpInsert(pArpDcpt->cacheDcpt, pIPAddr);
    if(hE == 0)
    {   // oops, hash full?
        return ARP_RES_CACHE_FULL;
    }

    // now in cache
    arpHE = (ARP_HASH_ENTRY*)hE;
    if(arpHE->hEntry.flags.newEntry != 0)
    {   // populate the new entry
        _ARPSetEntry(arpHE, ARP_FLAG_ENTRY_COMPLETE, hwAdd, &pArpDcpt->completeList);
    }
    else
    {   // existent entry
        _ARPUpdateEntry(pIf, arpHE, hwAdd);
    }

    return ARP_RES_OK;
}



/*****************************************************************************
  Function:
    void ARPInitialize(const TCPIP_STACK_MODULE_CTRL* const stackCtrl, const ARP_MODULE_CONFIG* const arpData)

  Summary:
    Initializes the ARP module.
    
  Description:
    Initializes the ARP module.
    Calls can be done with the request of not tearing down the ARP cache
    This helps for ifup/ifdown sequences.
    Of course, if this is the case the memory allocated for the ARP cache
    has to be from a persistent heap.
    
  Precondition:
    None

  Parameters:
    stackCtrl  - stack initialization parameters
    arpData    - ARP specific initialization parameters

  Returns:
    true if initialization succeded,
    false otherwise
  
  Remarks:
  ***************************************************************************/
bool ARPInitialize(const TCPIP_STACK_MODULE_CTRL* const stackCtrl, const ARP_MODULE_CONFIG* const arpData)
{
    OA_HASH_DCPT*   cacheDcpt;
    ARP_CACHE_DCPT* pArpDcpt;
    size_t          memSize;
    bool            newCache;

    if(stackCtrl->stackAction == TCPIP_STACK_ACTION_IF_UP)
    {   // interface going up
        return true;
    }

    // stack going up
    pArpDcpt = arpCache + stackCtrl->netIx;

    // store the delete option for de-initialization
    pArpDcpt->deleteOld = arpData->deleteOld;

    if(arpData->deleteOld)
    {   // remove the old stuff, if there
        _ARPCleanupCache(pArpDcpt);
    }
    // else do not re-initialize
    
    if(pArpDcpt->cacheDcpt == 0)
    {  
        // some initialization to be done
        // allocate hash + descriptor contiguously
        memSize = sizeof(OA_HASH_DCPT) + arpData->cacheEntries * sizeof(ARP_HASH_ENTRY);
        cacheDcpt = (OA_HASH_DCPT*)(*stackCtrl->mallocCallback)(stackCtrl->memH, memSize);

        if(cacheDcpt == 0)
        {   // failed
            return false;
        }

        newCache = true;
        // populate the entries
        cacheDcpt->memBlk = cacheDcpt + 1;
        cacheDcpt->hEntrySize = sizeof(ARP_HASH_ENTRY);
        cacheDcpt->hEntries = arpData->cacheEntries;
        cacheDcpt->probeStep = ARP_HASH_PROBE_STEP;

        OAHashInit(cacheDcpt);

        pArpDcpt->cacheDcpt = cacheDcpt;
        SingleListInit(&pArpDcpt->permList);
        SingleListInit(&pArpDcpt->completeList);
        SingleListInit(&pArpDcpt->incompleteList);

        pArpDcpt->purgeThres = (ARP_CACHE_PURGE_THRESHOLD * pArpDcpt->cacheDcpt->hEntries)/100;
        pArpDcpt->purgeQuanta = ARP_CACHE_PURGE_QUANTA;
    }
    else
    {   // didn't create anything now
        newCache = false;
    }
    
    if(arpTimerHandle == 0)
    {   // once per service
        SingleListInit(&arpRegisteredUsers);
        // store the memory allocation handle
        arpMemH = stackCtrl->memH;

        arpTimerHandle = SYS_TICK_TimerCreate(ARPTmoHandler);
        if(arpTimerHandle)
        {
            arpTickPending = arpTimeSeconds = 0;
            SYS_TICK_TimerSetRate(arpTimerHandle, SYS_TICK_ResolutionGet() * ARP_TASK_PROCESS_RATE);
        }
        else
        {   // failed
            if(newCache)
            {
                _ARPCleanupCache(pArpDcpt);
            }
            return false;
        }
    }

    pArpDcpt->inited = true;

    return true;
}




void ARPDeinitialize(const TCPIP_STACK_MODULE_CTRL* const stackCtrl)
{
    int ix;
    ARP_CACHE_DCPT* pArpDcpt = arpCache + stackCtrl->netIx;
    
    // interface going down
    if(pArpDcpt->deleteOld)
    {
        _ARPRemoveCacheEntries(pArpDcpt);
    }

    // done
    pArpDcpt->inited = false;

    if(stackCtrl->stackAction == TCPIP_STACK_ACTION_DEINIT)
    {   // stack shut down
        pArpDcpt = arpCache;
        for(ix = 0; ix < stackCtrl->nIfs; ix++, pArpDcpt++)
        {
            if(pArpDcpt->inited)
            {
                return; // some interface still on
            }
        }

        for(ix = 0, pArpDcpt = arpCache; ix < stackCtrl->nIfs; ix++, pArpDcpt++)
        {
            _ARPCleanupCache(pArpDcpt);
        }

        // all interfaces down
        _ARPCleanupClients();
        if(arpTimerHandle)
        {
            SYS_TICK_TimerDelete(arpTimerHandle);
            arpTimerHandle = 0;
            arpTickPending = 0;
        }
    }
}


static void _ARPCleanupCache(ARP_CACHE_DCPT* pArpDcpt)
{

    if(pArpDcpt->cacheDcpt)
    {
        _ARPRemoveCacheEntries(pArpDcpt);
        TCPIP_HEAP_Free(arpMemH, pArpDcpt->cacheDcpt);
        pArpDcpt->cacheDcpt = 0;
    }

}

static void _ARPCleanupClients(void)
{
    SGL_LIST_NODE* aNode;

    while( (aNode = SingleListRemoveHead(&arpRegisteredUsers)) != 0 )
    {
        TCPIP_HEAP_Free(arpMemH, aNode);
    }
}

ARP_HANDLE ARPRegisterHandler(TCPIP_NET_HANDLE hNet, ARP_EVENT_HANDLER handler, const void* hParam)
{
    if(arpMemH)
    {
        ARP_LIST_NODE* newNode = TCPIP_HEAP_Malloc(arpMemH, sizeof(*newNode));

        if(newNode)
        {
            newNode->handler = handler;
            newNode->hParam = hParam;
            newNode->hNet = hNet;
            SingleListAddTail(&arpRegisteredUsers, (SGL_LIST_NODE*)newNode);
            return newNode;
        }
    }

    return 0;

}

// deregister the event handler
bool ARPDeRegisterHandler(ARP_HANDLE hArp)
{
    if(hArp && arpMemH)
    {
        if(SingleListRemoveNode(&arpRegisteredUsers, (SGL_LIST_NODE*)hArp))
        {
            TCPIP_HEAP_Free(arpMemH, hArp);
            return true;
        }
    }

    return false;
}

static void _ARPNotifyClients(NET_CONFIG* pNetIf, const MAC_ADDR* MACAddr, ARP_EVENT_TYPE evType)
{
    ARP_LIST_NODE* aNode;

    for(aNode = (ARP_LIST_NODE*)arpRegisteredUsers.head; aNode != 0; aNode = aNode->next)
    {
        if(aNode->hNet == 0 || aNode->hNet == pNetIf)
        {   // trigger event
            (*aNode->handler)(pNetIf, MACAddr, evType, aNode->hParam);
        }
    }
    
}


static void ARPTmoHandler(SYS_TICK currSysTick)
{
    arpTimeSeconds += ARP_TASK_PROCESS_RATE;
    arpTickPending++;
}
    

// returns true if service needed
// called by the stack manager
bool ARPTaskPending(void)
{
    return arpTickPending != 0;
}

// called after service needed reported
// maintain the queues
void ARPTask(void)
{
    int netIx, purgeIx;
    ARP_HASH_ENTRY  *pE;
    ARP_CACHE_DCPT  *pArpDcpt;
    SGL_LIST_NODE   *pN;
    NET_CONFIG *pIf;
    int         nArpIfs;

    nArpIfs = TCPIP_STACK_NetworksNo();

    for(netIx = 0, pArpDcpt = arpCache; netIx < nArpIfs; netIx++, pArpDcpt++)
    {
        pIf = (NET_CONFIG*)TCPIP_STACK_IxToNet(netIx);

        // process the incomplete queue
        // see if there's something to remove
        while( (pN = pArpDcpt->incompleteList.head) != 0)
        {
            pE = (ARP_HASH_ENTRY*) ((uint8_t*)pN - offsetof(ARP_HASH_ENTRY, next));
            if( (arpTimeSeconds - pE->tInsert) >= ARP_CACHE_PENDING_ENTRY_TMO)
            {   // expired, remove it
                OAHashRemoveEntry(pArpDcpt->cacheDcpt, &pE->hEntry);
                SingleListRemoveHead(&pArpDcpt->incompleteList);
            }
            else
            {   // this list is ordered, we can safely break out
                break;
            }
        }

        // see if we have to query again
        for(pN = pArpDcpt->incompleteList.head; pN != 0; pN = pN->next)
        {
            pE = (ARP_HASH_ENTRY*) ((uint8_t*)pN - offsetof(ARP_HASH_ENTRY, next));
            if( (arpTimeSeconds - pE->tInsert) >= pE->nRetries * ARP_CACHE_PENDING_RETRY_TMO)
            {   // expired, retry it
                ARP_SendIfPkt(pIf, ARP_OPERATION_REQ, (uint32_t)pIf->MyIPAddr.Val, pE->ipAddress, &arpBcastAdd);
                pE->nRetries++;
            }
        }

        // see the completed entries queue
        while( (pN = pArpDcpt->completeList.head) != 0)
        {
            pE = (ARP_HASH_ENTRY*) ((uint8_t*)pN - offsetof(ARP_HASH_ENTRY, next));
            if( (arpTimeSeconds - pE->tInsert) >= ARP_CACHE_SOLVED_ENTRY_TMO)
            {   // expired, remove it
                OAHashRemoveEntry(pArpDcpt->cacheDcpt, &pE->hEntry);
                SingleListRemoveHead(&pArpDcpt->completeList);
            }
            else
            {   // this list is ordered, we can safely break out
                break;
            }
        }

        // finally purge, if needed
        if(pArpDcpt->cacheDcpt->fullSlots >= pArpDcpt->purgeThres)
        {
            for(purgeIx = 0; purgeIx < pArpDcpt->purgeQuanta; purgeIx++)
            {
                pN = SingleListRemoveHead(&pArpDcpt->completeList);
                if(pN)
                {
                    pE = (ARP_HASH_ENTRY*) ((uint8_t*)pN - offsetof(ARP_HASH_ENTRY, next));
                    OAHashRemoveEntry(pArpDcpt->cacheDcpt, &pE->hEntry);
                }
                else
                {   // no more entries
                    break;
                }
            }
        } 

        
    } 

    
    arpTickPending = 0;
}



/*****************************************************************************
  Function:
    ARP_RESULT ARPProcess(NET_CONFIG* pIf)

  Summary:
    Processes an incoming ARP packet.
    
  Description:
    Retrieves an ARP packet from the MAC buffer and determines if it is a
    response to our request (in which case the ARP is resolved) or if it
    is a request requiring our response (in which case we transmit one.)

  Precondition:
    ARP packet is ready in the MAC buffer.

  Parameters:
    None

  Return Values:
    ARP_RES_OK      - processing OK.
    ARP_RES_error   - some error occurred
  ***************************************************************************/
ARP_RESULT ARPProcess(NET_CONFIG* pIf)
{
    ARP_PACKET      packet;
    MAC_ADDR        *dstMAC; 
    OA_HASH_ENTRY   *hE;
    ARP_CACHE_DCPT  *pArpDcpt;
    int              netIx;
    TCPIP_MAC_HANDLE hMac;
    ARP_RESULT       arpReqRes;

    netIx = _TCPIPStackNetIx(pIf);
    pArpDcpt = arpCache + netIx;
    hMac = _TCPIPStackNetToMac(pIf);
    
    // Obtain the incoming ARP packet and process
    MACGetArray(hMac, (uint8_t*)&packet, sizeof(packet));       
    MACDiscardRx(hMac);
    SwapARPPacket(&packet);

    // Validate the ARP packet
    if ( packet.HardwareType != HW_ETHERNET     ||
            packet.MACAddrLen != sizeof(MAC_ADDR)  ||
            packet.ProtocolLen != sizeof(IP_ADDR) )
    {
        return ARP_RES_OK;
    }
#ifdef TCPIP_STACK_USE_ZEROCONF_LINK_LOCAL
    ARPProcessRxPkt(pIf, &packet);
#endif

    arpReqRes = ARP_RES_OK;
    // Handle incoming ARP packet
    hE = OAHashLookUp(pArpDcpt->cacheDcpt, &packet.SenderIPAddr.Val);
    if(hE != 0)
    {   // we already have this sender and we should update it
        _ARPUpdateEntry(pIf, (ARP_HASH_ENTRY*)hE, &packet.SenderMACAddr);
    }
    
    while(packet.TargetIPAddr.Val == pIf->MyIPAddr.Val)
    {   // we are the target and we should add to cache anyway
        if(hE == 0)
        {   // not there yet
            arpReqRes = _ARPAddCompleteEntry(pIf, &packet.SenderIPAddr, &packet.SenderMACAddr);
        }
   
        // Handle incoming ARP operation
        if(packet.Operation == ARP_OPERATION_REQ)
        {   
            // ARP packet asking for this host IP address 
#ifdef TCPIP_STACK_USE_ZEROCONF_LINK_LOCAL
            /* Fix for Loop-Back suppression:
             * For ZCLL-Claim packets, host should not respond.
             * Check Sender's MAC-address with own MAC-address and 
             * if it is matched, response will not be sent back. This
             * was leading to flooding of ARP-answeres */
            if(!memcmp (&packet.SenderMACAddr, &pIf->MyMACAddr, 6))
            {
                SYS_CONSOLE_MESSAGE("Loopback answer suppressed \r\n");
                break;
            }
#endif
#if defined(TCPIP_STACK_USE_AUTO_IP)
            if ((packet.SenderIPAddr.Val == pIf->MyIPAddr.Val) || AutoIPConfigIsInProgress(pIf))
            {
                AutoIPConflict(pIf);
                break;
            }             
#endif
    
            // Need to send a reply to the requestor 
#if defined(TCPIP_STACK_USE_AUTO_IP)
            if (AutoIPIsConfigured(pIf))
            {
                dstMAC = &arpBcastAdd;                
            }
            else
#endif
            {
                dstMAC = &packet.SenderMACAddr;
            }
            // Send an ARP response to the received request
            if(!ARP_SendIfPkt(pIf, ARP_OPERATION_RESP, (uint32_t)pIf->MyIPAddr.Val, (uint32_t)packet.SenderIPAddr.Val, dstMAC))
            {
                arpReqRes =  ARP_RES_TX_FAILED;
            }
        }
        break;
    }

    return arpReqRes;
}

    
/*****************************************************************************
  Function:
    ARP_RESULT ARPResolve(TCPIP_NET_HANDLE hNet, IP_ADDR* IPAddr)

  Summary:
    Transmits an ARP request to resolve an IP address.
    
  Description:
    This function transmits and ARP request to determine the hardware
    address of a given IP address.
    Upon the address resolution it calls the registered handler
    (if available) with the supplied notification parameter (if != 0)

  Precondition:
    None

  Parameters:
    IPAddr - The IP address to be resolved.  The address must be specified 
             in network byte order (big endian).

  Returns:
    ARP_RES_ENTRY_SOLVED     - if the required entry is already solved
    ARP_RES_ENTRY_QUEUED     - if the required entry was already queued
    ARP_RES_ENTRY_NEW        - if the operation succeeded and a new entry
                               was added (and queued for resolving)
    ARP_RES_CACHE_FULL       - if new entry could not be inserted,
                               the cache was full                            
    ARP_RES_BAD_ADDRESS      - bad address specified                           

  Remarks:

    To retrieve the ARP query result, call the ARPIsResolved() function.
  ***************************************************************************/
ARP_RESULT ARPResolve(TCPIP_NET_HANDLE hNet, IP_ADDR* IPAddr)
{
    IP_ADDR     targetIPAddr;
    ARP_CACHE_DCPT  *pArpDcpt;
    OA_HASH_ENTRY   *hE;
    NET_CONFIG      *pIf;
   
    if(IPAddr->Val == 0)
    {   // do not store 0's in cache
        return ARP_RES_BAD_ADDRESS;
    }
    
#ifdef TCPIP_STACK_USE_ZEROCONF_LINK_LOCAL
    if ((IPAddr->v[0] >= 224) &&(IPAddr->v[0] <= 239))
    {
        // "Resolve" of the IP to MAC address mapping for
        // IP multicast address range from 224.0.0.0 to 239.255.255.255
        // can be done locally; No need for an ARP request.
        return ARP_RES_ENTRY_SOLVED;
    }
#endif
    
    pIf = _TCPIPStackHandleToNet(hNet);
    if(!pIf)
    {
        return ARP_RES_NO_INTERFACE;
    }
    
    // ARP query either the IP address directly (on our subnet), or do an ARP query for our Gateway if off of our subnet
    targetIPAddr = ((pIf->MyIPAddr.Val ^ IPAddr->Val) & pIf->MyMask.Val) ? pIf->MyGateway : *IPAddr;
    
    pArpDcpt = arpCache + _TCPIPStackNetIx(pIf);

    hE = OAHashLookUpInsert(pArpDcpt->cacheDcpt, &targetIPAddr.Val);
    if(hE == 0)
    {   // oops!
        return ARP_RES_CACHE_FULL;
    }
        
    if(hE->flags.newEntry != 0)
    {   // new entry; add it to the not done list 
        _ARPSetEntry((ARP_HASH_ENTRY*)hE, 0, 0, &pArpDcpt->incompleteList);

        // initiate an ARP request operation
        ARP_SendIfPkt(pIf, ARP_OPERATION_REQ, (uint32_t)pIf->MyIPAddr.Val, ((ARP_HASH_ENTRY*)hE)->ipAddress, &arpBcastAdd);
        return ARP_RES_ENTRY_NEW;
    }
    // else, even if it is not complete, ARPTask will initiate retransmission
    // Normally if the entry is existent, it should be refreshed, since it's obviously needed.
    // However, the ARPIsResolved() will do it, because that's the call that actually uses the entry!
    if((hE->flags.value & ARP_FLAG_ENTRY_VALID_MASK) != 0)
    {
        return ARP_RES_ENTRY_SOLVED;
    }
    
    // incomplete
    return ARP_RES_ENTRY_QUEUED;
}



/*****************************************************************************
  Function:
    bool ARPIsResolved(TCPIP_NET_HANDLE hNet, IP_ADDR* IPAddr, MAC_ADDR* MACAddr)

  Summary:
    Determines if an ARP request has been resolved yet.
    
  Description:
    This function checks if an ARP request has been resolved yet, and if
    so, stores the resolved MAC address in the pointer provided.

  Precondition:
    ARP packet is ready in the MAC buffer.

  Parameters:
    hNet   - interface to use
    IPAddr - The IP address to be resolved.  This must match the IP address 
             provided to the ARPResolve() function call.
    MACAddr - A buffer to store the corresponding MAC address retrieved from 
             the ARP query.

  Return Values:
    true - The IP address has been resolved and MACAddr MAC address field
           indicates the response.
    false - The IP address is not yet resolved.  Try calling ARPIsResolved() 
           again at a later time.  If you don't get a response after a 
           application specific timeout period, you may want to call 
           ARPResolve() again to transmit another ARP query (in case if the 
           original query or response was lost on the network).  If you never 
           receive an ARP response, this may indicate that the IP address 
           isn't in use.

  Remarks:
  ***************************************************************************/
bool ARPIsResolved(TCPIP_NET_HANDLE hNet, IP_ADDR* IPAddr, MAC_ADDR* MACAddr)
{
#ifdef TCPIP_STACK_USE_ZEROCONF_LINK_LOCAL
    if ((IPAddr->v[0] >= 224) &&(IPAddr->v[0] <= 239))
    {
        // "Resolve" of the IP to MAC address mapping for
        // IP multicast address range from 224.0.0.0 to 239.255.255.255
        // can be done locally; No need for an ARP request.
        MACAddr->v[0] = 0x01;
        MACAddr->v[1] = 0x00;
        MACAddr->v[2] = 0x5E;
        MACAddr->v[3] = 0x7f & IPAddr->v[1];
        MACAddr->v[4] = IPAddr->v[2];
        MACAddr->v[5] = IPAddr->v[3];
        return true;
    }
#endif

    OA_HASH_ENTRY   *hE;
    ARP_CACHE_DCPT  *pArpDcpt;
    IP_ADDR     targetIPAddr;
    NET_CONFIG  *pIf;

    pIf = _TCPIPStackHandleToNet(hNet);
    if(!pIf)
    {
        return ARP_RES_NO_INTERFACE;
    }
    

    pArpDcpt = arpCache + _TCPIPStackNetIx(pIf);
    
    targetIPAddr = ((pIf->MyIPAddr.Val ^ IPAddr->Val) & pIf->MyMask.Val) ? pIf->MyGateway : *IPAddr;
    
    hE = OAHashLookUp(pArpDcpt->cacheDcpt, &targetIPAddr.Val);
    if(hE != 0 && (hE->flags.value & ARP_FLAG_ENTRY_VALID_MASK) != 0 )
    {   // found address in cache
        ARP_HASH_ENTRY  *arpHE = (ARP_HASH_ENTRY*)hE;
        *MACAddr = arpHE->hwAdd;
        if((hE->flags.value & ARP_FLAG_ENTRY_COMPLETE) != 0 )
        {   // an existent entry, re-used, gets refreshed
            _ARPRefreshEntry(arpHE, &pArpDcpt->completeList);
        }
        return true;
    }
    
    return false;
    
}



/*****************************************************************************
  Function:
    void SwapARPPacket(ARP_PACKET* p)

  Description:
    Swaps endian-ness of header information in an ARP packet.

  Precondition:
    None

  Parameters:
    p - The ARP packet to be swapped

  Returns:
    None
  ***************************************************************************/
static void SwapARPPacket(ARP_PACKET* p)
{
    p->HardwareType     = swaps(p->HardwareType);
    p->Protocol         = swaps(p->Protocol);
    p->Operation        = swaps(p->Operation);
}

/*****************************************************************************
  Function:
    void ARPSendPkt(IP_ADDR* SrcIPAddr, IP_ADDR* DestIPAddr, uint16_t op_req )

  Summary:
    Transmits an ARP request/Reply initated by Application or external module.
    
  Description:
    This function transmits and ARP request/reply to determine the hardware
    address of a given IP address (or) Announce self-address to all nodes in
    network. Extended for zeroconf protocol. 

  Precondition:
    ARP packet is ready in the MAC buffer.

  Parameters:
    SrcIPAddr - The Source IP-address 
    DestIPAddr - The Destination IP-Address
    op_req     - Operation Request (ARP_REQ/ARP_RESP)

  Returns:
    true - The ARP packet was generated properly
    false - Not possible return value

  Remarks:
    This API is to give control over AR-packet to external modules. 
  ***************************************************************************/
bool ARPSendPkt(NET_CONFIG* pIf, uint32_t SrcIPAddr, uint32_t DestIPAddr, uint16_t op_req )
{
    return ARP_SendIfPkt(pIf, op_req, SrcIPAddr, DestIPAddr, &arpBcastAdd);
}

ARP_RESULT ARPEntrySet(TCPIP_NET_HANDLE hNet, IP_ADDR* ipAdd, MAC_ADDR* hwAdd, bool perm)
{
    ARP_CACHE_DCPT  *pArpDcpt;
    ARP_HASH_ENTRY  *arpHE;
    OA_HASH_ENTRY   *hE;
    SINGLE_LIST     *oldList, *newList;
    ARP_ENTRY_FLAGS newFlags;
    ARP_RESULT      res;
    NET_CONFIG  *pIf;

    if(ipAdd->Val == 0)
    {   // do not store 0's in cache
        return ARP_RES_BAD_ADDRESS;
    }
    
    pIf = _TCPIPStackHandleToNet(hNet);
    if(!pIf)
    {
        return ARP_RES_NO_INTERFACE;
    }
    
    pArpDcpt = arpCache + _TCPIPStackNetIx(pIf);

    hE = OAHashLookUpInsert(pArpDcpt->cacheDcpt, &ipAdd->Val);
    if(hE == 0)
    {   // oops!
        return ARP_RES_CACHE_FULL;
    }

    // where to put it
    if(perm)
    {
        newList = &pArpDcpt->permList;
        newFlags = ARP_FLAG_ENTRY_PERM;
    }
    else
    {
        newList = &pArpDcpt->completeList;
        newFlags = ARP_FLAG_ENTRY_COMPLETE;       // complete
    }
    
    arpHE = (ARP_HASH_ENTRY*)hE;
   
    if(hE->flags.newEntry == 0)
    {   // existent entry
        if( (hE->flags.value & ARP_FLAG_ENTRY_PERM) != 0 )
        {
            oldList =  &pArpDcpt->permList;
        }
        else if( (hE->flags.value & ARP_FLAG_ENTRY_COMPLETE) != 0 )
        {
            oldList =  &pArpDcpt->completeList;
        }
        else
        {
            oldList =  &pArpDcpt->incompleteList;
        }

        if(newList != oldList)
        {   // remove from the old list
            SingleListRemoveNode(oldList, (SGL_LIST_NODE*)&arpHE->next);
        }
        res = ARP_RES_ENTRY_EXIST;
    }
    else
    {
        res = ARP_RES_OK;
    }
    
    // add it to where it belongs
    _ARPSetEntry(arpHE, newFlags, hwAdd, newList);

    if(SingleListCount(&pArpDcpt->permList) >= ARP_CACHE_PERMANENT_QUOTA * pArpDcpt->cacheDcpt->fullSlots)
    {   // quota exceeded
        res = ARP_RES_PERM_QUOTA_EXCEED;
    }

    return res;
}

ARP_RESULT ARPEntryGet(TCPIP_NET_HANDLE hNet, IP_ADDR* ipAdd, MAC_ADDR* pHwAdd)
{   
    NET_CONFIG  *pIf;
    pIf = _TCPIPStackHandleToNet(hNet);
    
    if(pIf)
    {
        if(ARPIsResolved(pIf, ipAdd, pHwAdd))
        {
            return ARP_RES_OK;
        }
    }

    return ARP_RES_NO_ENTRY;
}

ARP_RESULT ARPEntryRemove(TCPIP_NET_HANDLE hNet,  IP_ADDR* ipAdd)
{
    OA_HASH_ENTRY   *hE;
    ARP_CACHE_DCPT  *pArpDcpt;
    SINGLE_LIST     *remList;
    NET_CONFIG  *pIf;

    pIf = _TCPIPStackHandleToNet(hNet);
    if(!pIf)
    {
        return ARP_RES_NO_INTERFACE;
    }
    

    pArpDcpt = arpCache + _TCPIPStackNetIx(pIf);

    hE = OAHashLookUp(pArpDcpt->cacheDcpt, &ipAdd->Val);

    if(hE == 0)
    {
        return ARP_RES_NO_ENTRY;
    }

    if((hE->flags.value & ARP_FLAG_ENTRY_PERM) != 0 )
    {
        remList =  &pArpDcpt->permList;
    }
    else if((hE->flags.value & ARP_FLAG_ENTRY_COMPLETE) != 0 )
    {
        remList =  &pArpDcpt->completeList;
    }
    else
    {
        remList =  &pArpDcpt->incompleteList;
    }

    SingleListRemoveNode(remList, (SGL_LIST_NODE*)&((ARP_HASH_ENTRY*)hE)->next);     

    OAHashRemoveEntry(pArpDcpt->cacheDcpt, hE);

    return ARP_RES_OK;
}

ARP_RESULT ARPEntryRemoveAll(TCPIP_NET_HANDLE hNet)
{
    NET_CONFIG  *pIf;
    pIf = _TCPIPStackHandleToNet(hNet);
    if(!pIf)
    {
        return ARP_RES_NO_INTERFACE;
    }
    
    ARP_CACHE_DCPT  *pArpDcpt = arpCache + _TCPIPStackNetIx(pIf);

    _ARPRemoveCacheEntries(pArpDcpt);

    return ARP_RES_OK;
}

ARP_RESULT ARPEntryQuery(TCPIP_NET_HANDLE hNet, size_t index, ARP_ENTRY_QUERY* pArpQuery)
{
    OA_HASH_ENTRY   *hE;
    ARP_HASH_ENTRY  *arpHE;
    ARP_CACHE_DCPT  *pArpDcpt;
    MAC_ADDR        noHwAdd = {{0}};
    NET_CONFIG  *pIf;

    pIf = _TCPIPStackHandleToNet(hNet);
    if(!pIf)
    {
        return ARP_RES_NO_INTERFACE;
    }

    pArpDcpt = arpCache + _TCPIPStackNetIx(pIf);
    hE = OAHashGetEntry(pArpDcpt->cacheDcpt, index);
    

    if(hE == 0)
    {
        return ARP_RES_BAD_INDEX;
    }
    
    arpHE = (ARP_HASH_ENTRY*)hE;

    if(pArpQuery)
    {
        pArpQuery->entryIpAdd.Val = 0;
        pArpQuery->entryHwAdd = noHwAdd;
        
        if(hE->flags.busy == 0)
        {
            pArpQuery->entryType = ARP_ENTRY_TYPE_INVALID;
        }
        else if((hE->flags.value & ARP_FLAG_ENTRY_VALID_MASK) != 0)
        {
            pArpQuery->entryType = ((hE->flags.value & ARP_FLAG_ENTRY_PERM) != 0)?
                                ARP_ENTRY_TYPE_PERMANENT:ARP_ENTRY_TYPE_COMPLETE;
            pArpQuery->entryIpAdd.Val = arpHE->ipAddress;
            pArpQuery->entryHwAdd = arpHE->hwAdd;
        }
        else
        {
            pArpQuery->entryType = ARP_ENTRY_TYPE_INCOMPLETE;
            pArpQuery->entryIpAdd.Val = arpHE->ipAddress;
        }
    }

    return ARP_RES_OK;
}

size_t ARPCacheGetEntriesNo(TCPIP_NET_HANDLE hNet, ARP_ENTRY_TYPE type)
{
    NET_CONFIG  *pIf;
    
    pIf = _TCPIPStackHandleToNet(hNet);
    if(!pIf)
    {
        return ARP_RES_NO_INTERFACE;
    }
    
    ARP_CACHE_DCPT  *pArpDcpt = arpCache + _TCPIPStackNetIx(pIf);
    OA_HASH_DCPT    *pOH = pArpDcpt->cacheDcpt;

    switch(type)
    {
        case ARP_ENTRY_TYPE_INVALID:
           return pOH->hEntries - pOH->fullSlots;

        case ARP_ENTRY_TYPE_PERMANENT:
           return SingleListCount(&pArpDcpt->permList);

        case ARP_ENTRY_TYPE_COMPLETE:
           return SingleListCount(&pArpDcpt->completeList);

        case ARP_ENTRY_TYPE_INCOMPLETE:
           return SingleListCount(&pArpDcpt->incompleteList);

        default:    // case ARP_ENTRY_TYPE_TOTAL:
           return pOH->hEntries;
    }

}

ARP_RESULT ARPCacheSetThreshold(TCPIP_NET_HANDLE hNet, int purgeThres, int purgeEntries)
{
    NET_CONFIG  *pIf;

    pIf = _TCPIPStackHandleToNet(hNet);
    if(!pIf)
    {
        return ARP_RES_NO_INTERFACE;
    }
    
    ARP_CACHE_DCPT  *pArpDcpt = arpCache + _TCPIPStackNetIx(pIf);

    pArpDcpt->purgeThres = (purgeThres * pArpDcpt->cacheDcpt->hEntries)/100;
    pArpDcpt->purgeQuanta = purgeEntries;

    return ARP_RES_OK;
}

#if !defined ( OA_HASH_DYNAMIC_KEY_MANIPULATION )

// static versions
// 
size_t OAHashKeyHash(OA_HASH_DCPT* pOH, void* key)
{
    return fnv_32_hash(key, 4) % (pOH->hEntries);
}

#if defined(OA_DOUBLE_HASH_PROBING)
size_t OAHashProbeHash(OA_HASH_DCPT* pOH, void* key)
{
    return fnv_32a_hash(key, 4) % (pOH->hEntries);
}

#endif  // defined(OA_DOUBLE_HASH_PROBING)

// Deletes an entry to make room in the hash table.
// This shouldn't normally occur if ARPTask()
// does its job of periodically performing the cache clean-up.
// However, since the threshold can be dynamically adjusted,
// the situation could still occur
OA_HASH_ENTRY* OAHashDeleteEntry(OA_HASH_DCPT* pOH)
{
    ARP_CACHE_DCPT  *pArpDcpt;
    ARP_HASH_ENTRY  *pE;
    SGL_LIST_NODE   *pN;
    SINGLE_LIST     *pRemList = 0;    
    
    pArpDcpt = (ARP_CACHE_DCPT*)pOH;

    if( (pN = pArpDcpt->incompleteList.head) != 0)
    {
        pE = (ARP_HASH_ENTRY*) ((uint8_t*)pN - offsetof(ARP_HASH_ENTRY, next));
        if( (arpTimeSeconds - pE->tInsert) >= ARP_CACHE_PENDING_ENTRY_TMO)
        {   // we remove this one
            pRemList = &pArpDcpt->incompleteList;
        }
    }

    if(pRemList == 0)
    {   // no luck with the incomplete list; use the complete one
            pRemList = &pArpDcpt->completeList;
    }

    pN = SingleListRemoveHead(pRemList);

    if(pN)
    {
        pE = (ARP_HASH_ENTRY*) ((uint8_t*)pN - offsetof(ARP_HASH_ENTRY, next));
        return &pE->hEntry;    
    }

    // it's possible to be unable to make room in the cache
    // for example, too many permanent entries added...
                   
    return 0;
}


int OAHashKeyCompare(OA_HASH_DCPT* pOH, OA_HASH_ENTRY* hEntry, void* key)
{
    return ((ARP_HASH_ENTRY*)hEntry)->ipAddress != ((ARP_UNALIGNED_KEY*)key)->v;
}

void OAHashKeyCopy(OA_HASH_DCPT* pOH, OA_HASH_ENTRY* dstEntry, void* key)
{

    ((ARP_HASH_ENTRY*)dstEntry)->ipAddress = ((ARP_UNALIGNED_KEY*)key)->v;
}


#endif  // !defined ( OA_HASH_DYNAMIC_KEY_MANIPULATION )




