/*******************************************************************************
  tcpip MRF24W MAC events implementation

  Summary:
    
  Description:
*******************************************************************************/

/*******************************************************************************
FileName:   MRF24W_events.c
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

#include "tcpip_private.h"


// PIC32 embedded MAC implementation

#if defined(TCPIP_IF_MRF24W)

#if defined(TCPIP_STACK_USE_EVENT_NOTIFICATION)

#include "mac_private.h"
#include "mrf24w_mac_pic32.h"

/*********************************
 *  local definitions
 ******************************************/
#if defined(PIC32_ENET_SK_DM320004) && defined(MRF24W_USE_CN_INT)
    #define MRFWB0M_INT_SOURCE  PLIB_INT_SOURCE_CHANGE_NOTICE

#elif defined( MRF24W_IN_SPI2 ) || defined( MRF24W_IN_SPI4 )
    #define MRFWB0M_INT_SOURCE  PLIB_INT_SOURCE_EXTERNAL_3

#elif defined(MRF24W_IN_SPI1)
    #define MRFWB0M_INT_SOURCE  PLIB_INT_SOURCE_EXTERNAL_1

#elif defined(PIC32_USB_DM320003_2) || (MRF24W_SPI_CHN == 2)
    #define MRFWB0M_INT_SOURCE  PLIB_INT_SOURCE_EXTERNAL_3

#else
    #error  "Either MRF24W_IN_SPI1, MRF24W_IN_SPI2 or MRF24W_IN_SPI4 have to be defined for MRF24W module!"
#endif


// stack internal notification
typedef struct
{
    bool                    _mrfEnabledEvents;          // group enabled notification events
    volatile TCPIP_EVENT    _mrfPendingEvents;          // group notification events that are set, waiting to be re-acknowledged
    pMacEventF              _mrfNotifyFnc;              // group notification handler
    void*                   _mrfNotifyParam;            // notification parameter
}MRF24W_EV_GROUP_DCPT;   // event descriptor per group



// stack client notification
// 
// Storage for MRF24 events
// Stored until the stack user asks for them
// Keep different copies for RX and MGMT events
// since it seems that the WiFi code treats them seaparately
typedef struct
{
    uint8_t   trafficEvents;
    uint16_t      trafficEventInfo;
    uint8_t   mgmtEvents;
    uint16_t      mgmtEventInfo;
}MRF24W_USR_EV_DCPT;



/*************************
 * local data
 *************************/

static MRF24W_EV_GROUP_DCPT  _mrfGroupDcpt = 
{
    TCPIP_EV_NONE, TCPIP_EV_NONE, 0                  // TCPIP_MAC_EVGROUP_ALL
};

static MRF24W_USR_EV_DCPT    _mrfUsrEvent;        // stack user events


/*********************************
 *  local proto
 *  referenced by WFMAc::MACInit() too! 
 ******************************************/

void MRF24W_ISR(void*);


/*********************************
 *  implementation
 ******************************************/


/*******************************************************************************
  Function:
    TCPIP_MAC_EVENT_RESULT    MRF24W_MACEventInit(TCPIP_MAC_HANDLE hMac, int intPri, int intSubPri)

  Summary:
    Initializes the ethernet event notification.

  Description:
     This function initializes the ethernet event notification.
     It performs any resource allocation that may be needed.

  Precondition:
     None.

  Parameters:
    hMac      - parameter identifying the intended MAC  
    intPri     - priority of the TCPIP interrupt events
    intSubPri  - sub-priority of the TCPIP interrupt events
    
  Returns:
    TCPIP_MAC_EVRES_OK  if initialization succeeded,
    an error code otherwise

  Example:
    <code>
    MRF24W_MACEventInit( hMac, 4, 3 );
    </code>

  Remarks:

    Not multi-threaded safe.
*****************************************************************************/
TCPIP_MAC_EVENT_RESULT MRF24W_MACEventInit(TCPIP_MAC_HANDLE hMac, int intPri, int intSubPri)
{
    SYS_INT_SourceDisable(MRFWB0M_INT_SOURCE);      // stop MRF ints
    SYS_INT_SourceStatusClear(MRFWB0M_INT_SOURCE);
    SYS_INT_PrioritySet(MRFWB0M_INT_SOURCE, (INT_PRIORITY_LEVEL_1-1)+intPri);
#if defined (__C32__)
    SYS_INT_SubprioritySet(MRFWB0M_INT_SOURCE, INT_SUB_PRIORITY_LEVEL_0+intSubPri);
#endif
    SYS_INT_DynamicRegister(MRFWB0M_INT_SOURCE, MRF24W_ISR, 0); 
    
    _mrfGroupDcpt._mrfNotifyFnc = 0;
    _mrfGroupDcpt._mrfEnabledEvents = false;
    _mrfGroupDcpt._mrfPendingEvents = 0;

    _mrfUsrEvent.trafficEvents = _mrfUsrEvent.mgmtEvents = 0;
    _mrfUsrEvent.trafficEventInfo = _mrfUsrEvent.mgmtEventInfo =0;


    return TCPIP_MAC_EVRES_OK;
}

/*******************************************************************************
  Function:
    TCPIP_MAC_EVENT_RESULT    MRF24W_MACEventDeInit(TCPIP_MAC_HANDLE hMac )

  Summary:
    De-initializes the ethernet event notification.

  Description:
     This function de-initializes the ethernet event notification.
     It performs any resource clean-up that may be needed.

  Precondition:
     None.

  Parameters:
    hMac      - parameter identifying the intended MAC  
    
  Returns:
    TCPIP_MAC_EVRES_OK  always

  Example:
    <code>
    MRF24W_MACEventDeInit( hMac );
    </code>

  Remarks:

    Not multi-threaded safe.
*****************************************************************************/
TCPIP_MAC_EVENT_RESULT MRF24W_MACEventDeInit(TCPIP_MAC_HANDLE hMac)
{


    SYS_INT_SourceDisable(MRFWB0M_INT_SOURCE);      // stop MRF ints
    SYS_INT_SourceStatusClear(MRFWB0M_INT_SOURCE);
    SYS_INT_DynamicRegister(MRFWB0M_INT_SOURCE, 0, 0); 

    _mrfGroupDcpt._mrfNotifyFnc = 0;
    _mrfGroupDcpt._mrfEnabledEvents = false;
    _mrfGroupDcpt._mrfPendingEvents = 0;

    return TCPIP_MAC_EVRES_OK;
}


/*******************************************************************************
  Function:
    TCPIP_MAC_EVENT_RESULT    MRF24W_MACEventSetNotifyEvents(TCPIP_MAC_HANDLE hMac, TCPIP_MAC_EVENT_GROUP tcpEvGroup, TCPIP_EVENT tcpipEvents)

  Summary:
    Adds new events to the list of the enabled ones.

  Description:
     This function sets new enabled events.
     Multiple events can be orr-ed together.
     All events that are set will be added to the notification process. The other events will not ne touched.
     The stack (or stack user) has to catch the events that are notified and process them:
         - The stack should process the TCPIP_EV_RX_PKTPEND, TCPIP_EV_TX_DONE transfer events
         - Process the specific condition and acknowledge them calling MRF24W_MACEventAck() so that they can be re-enabled.

  Precondition:
   TCPIPInit should have been called.
   tcpEvGroup, tcpSetEv valid values 

  Parameters:
    hMac      - parameter identifying the intended MAC  
    tcpEvGroup  - group of events the notification refers to
    tcpipEvents - events the user of the stack wants to add for notification
    
  Returns:
    TCPIP_MAC_EVRES_OK  if operation succeeded,
    an error code otherwise

  Example:
    <code>
    MRF24W_MACEventSetNotifyEvents( hMac, TCPIP_MAC_EVGROUP_RX, TCPIP_EV_RX_OVFLOW | TCPIP_EV_RX_BUFNA );
    </code>

  Remarks:
    Globally enable/disable all notifications for now.

*****************************************************************************/
TCPIP_MAC_EVENT_RESULT MRF24W_MACEventSetNotifyEvents(TCPIP_MAC_HANDLE hMac, TCPIP_MAC_EVENT_GROUP tcpEvGroup, TCPIP_EVENT tcpSetEv)
{
    if(tcpEvGroup != TCPIP_MAC_EVGROUP_ALL)
    {
        return TCPIP_MAC_EVRES_GROUP_ERR;
    }
       
    _mrfGroupDcpt._mrfEnabledEvents = true;
    SYS_INT_SourceEnable(MRFWB0M_INT_SOURCE);      // start MRF ints

    return TCPIP_MAC_EVRES_OK;
}



/*******************************************************************************
  Function:
    TCPIP_MAC_EVENT_RESULT MRF24W_MACEventClearNotifyEvents(TCPIP_MAC_HANDLE hMac, TCPIP_MAC_EVENT_GROUP tcpEvGroup, TCPIP_EVENT tcpipEvents)

  Summary:
    Removes events from the list of the enabled ones.

  Description:
     This function removes from the enabled events.
     Multiple events can be orr-ed together.
     All events that are set will be removed from the notification process. The other events will not ne touched.


  Precondition:
   TCPIPInit should have been called.
   tcpEvGroup, tcpSetEv valid values 

  Parameters:
    hMac      - parameter identifying the intended MAC  
    tcpEvGroup  - group of events the notification refers to
    tcpipEvents - events the user of the stack wants to remove from notification
    
  Returns:
    TCPIP_MAC_EVRES_OK  if operation succeeded,
    an error code otherwise

  Example:
    <code>
    MRF24W_MACEventClearNotifyEvents( hMac, TCPIP_MAC_EVGROUP_ALL, TCPIP_EV_RX_OVFLOW | TCPIP_EV_RX_BUFNA );
    </code>

  Remarks:   
    Globally enable/disable all notifications for now.
*****************************************************************************/
TCPIP_MAC_EVENT_RESULT MRF24W_MACEventClearNotifyEvents(TCPIP_MAC_HANDLE hMac, TCPIP_MAC_EVENT_GROUP tcpEvGroup, TCPIP_EVENT tcpClrEv)
{
    SYS_INT_SourceDisable(MRFWB0M_INT_SOURCE);      // stop MRF ints
    _mrfGroupDcpt._mrfEnabledEvents = false;
            
    return TCPIP_MAC_EVRES_OK;
}

/*******************************************************************************
  Function:
    TCPIP_MAC_EVENT_RESULT    MRF24W_MACEventAck(TCPIP_MAC_HANDLE hMac, TCPIP_MAC_EVENT_GROUP tcpEvGroup, TCPIP_EVENT tcpipEvents)

  Summary:
    Acknowledges and re-enables processed events.

  Description:
    This function acknowledges and re-enables processed events.
    Multiple events can be orr-ed together as they are processed together.
    The events acknowledged by this function should be the events that have been retrieved from the stack
    by calling MRF24W_MACEventGetPending() or have been passed to the user by the stack using the notification handler
    (MRF24W_MACEventSetNotifyHandler()) and have been processed and have to be re-enabled.


  Precondition:
   TCPIPInit should have been called.
   tcpEvGroup, tcpSetEv valid values 

  Parameters:
    hMac      - parameter identifying the intended MAC  
    tcpEvGroup  - group of events the acknowledge refers to
    tcpipEvents - the events that the user processed and need to be re-enabled
    
  Returns:
    TCPIP_MAC_EVRES_ACK_OK if events acknowledged
    TCPIP_MAC_EVRES_ACK_NONE if no events to be acknowledged
    an error code otherwise

  Example:
    <code>
    MRF24W_MACEventAck( hMac, TCPIP_MAC_EVGROUP_ALL, stackNewEvents );
    </code>

  Remarks:   
    All events should be acknowledged, in order to be re-enabled.

    For now, the re-enabling is done internally by the MRF processing events code.
*****************************************************************************/
TCPIP_MAC_EVENT_RESULT MRF24W_MACEventAck(TCPIP_MAC_HANDLE hMac, TCPIP_MAC_EVENT_GROUP tcpEvGroup, TCPIP_EVENT tcpAckEv)
{
    if(_mrfGroupDcpt._mrfPendingEvents)
    {
        _mrfGroupDcpt._mrfPendingEvents = 0;
        return TCPIP_MAC_EVRES_ACK_OK;
    }
    else
    {
        return TCPIP_MAC_EVRES_ACK_NONE;
    }
}


/*******************************************************************************
  Function:
    TCPIP_EVENT MRF24W_MACEventGetPending(TCPIP_MAC_HANDLE hMac, TCPIP_MAC_EVENT_GROUP tcpEvGroup)

  Summary:
    Returns the currently pending events.

  Description:
    This function returns the currently pending events belonging to a group.
    Multiple events can be orr-ed together as they accumulate.
    The stack should be called for processing whenever a stack managed event (TCPIP_EV_RX_PKTPEND, TCPIP_EV_TX_DONE) is present.
    The other, non critical events, may not be managed by the stack and passed to an user.
    They will have to be eventually acknowledged if re-enabling is needed.

  Precondition:
   hMac      - parameter identifying the intended MAC  
   TCPIPInit should have been called.
   tcpEvGroup valid value 

  Parameters:
    tcpEvGroup  - group of events needed
    
  Returns:
    The currently stack pending events.

  Example:
    <code>
    TCPIP_EVENT currEvents = MRF24W_MACEventGetPending( hMac, TCPIP_MAC_EVGROUP_ALL);
    </code>

  Remarks:   
    This is the preferred method to get the current pending MAC events.
    The stack maintains a proper image of the events from their occurrence to their acknowledgement.
    
    Even with a notification handler in place it's better to use this function to get the current pending events
    rather than using the events passed by the notification handler which could be stale.
    
    The events are persistent. They shouldn't be re-enabled unless they have been processed and
    the condition that generated them was removed.
    Re-enabling them immediately without proper processing will have dramatic effects on system performance.

    The returned value is just a momentary value. The pending events can change any time.
*****************************************************************************/
TCPIP_EVENT MRF24W_MACEventGetPending(TCPIP_MAC_HANDLE hMac, TCPIP_MAC_EVENT_GROUP tcpEvGroup)
{
    return _mrfGroupDcpt._mrfPendingEvents;
}

/*******************************************************************************
  Function:
    TCPIP_MAC_EVENT_RESULT    MRF24W_MACEventSetNotifyHandler(TCPIP_MAC_HANDLE hMac, TCPIP_MAC_EVENT_GROUP tcpEvGroup, pMacEventF eventHandler, void* hParam)

  Summary:
    Sets a new event notification handler.

  Description:
    This function sets a new event notification handler.
    This is a handler specified by the user of the stack.
    The caller can use the handler to be notified of MAC events.
    Whenever a notification occurs the passed events have to be eventually processed:
    - Stack should process the TCPIP_EV_RX_PKTPEND, TCPIP_EV_TX_DONE  events
    - Process the specific (error) condition
    - Acknowledge the events by calling MRF24W_MACEventAck() so that they can be re-enabled.

  Precondition:
   TCPIPInit should have been called.
   tcpEvGroup valid value 

  Parameters:
    hMac      - parameter identifying the intended MAC  
    tcpEvGroup   - group of events the notification handler refers to
    eventHandler - the event notification handler
    
  Returns:
    TCPIP_MAC_EVRES_OK  if operation succeeded,
    an error code otherwise

  Example:
    <code>
    MRF24W_MACEventSetNotifyHandler( hMac, TCPIP_MAC_EVGROUP_ALL, myEventHandler, myParam );
    </code>

  Remarks:   
    The notification handler will be called from the ISR which detects the corresponding event.
    The event notification handler has to be kept as short as possible and non-blocking.
    Mainly useful for RTOS integration where this handler will wake-up a thread that waits for a MAC event to occur.

    The event notification system also enables the user of the TCPIP stack to call into the stack
    for processing only when there are relevant events rather than being forced to periodically call
    from within a loop at unknown moments.

    Without a notification handler the stack user can still call MRF24W_MACEventGetPending() to see if processing by the stack needed.

    This is a default way of adding interrupt processing to the stack.
    A separate ISR to catch the Ethernet MAC events and process accordingly
    could be added.

    All the groups specified in the TCPIP_MAC_EVENT_GROUP enumeration is supported.
    However, once a handler for TCPIP_MAC_EVGROUP_ALL is registered all event processing
    will be reported using this global handler.

    Use 0 to remove the handler for a specific event group.

    Not multi-threaded safe accross different TCPIP_MAC_EVENT_GROUP groups.
*****************************************************************************/
TCPIP_MAC_EVENT_RESULT MRF24W_MACEventSetNotifyHandler(TCPIP_MAC_HANDLE hMac, TCPIP_MAC_EVENT_GROUP tcpEvGroup, pMacEventF eventHandler, void* hParam)
{
    if(tcpEvGroup != TCPIP_MAC_EVGROUP_ALL)
    {
        return TCPIP_MAC_EVRES_GROUP_ERR;
    }
        
    int rfILev;
    
    if(_mrfGroupDcpt._mrfEnabledEvents)
    {   // already have some active     
        rfILev = SYS_INT_SourceDisable(MRFWB0M_INT_SOURCE);      // stop ints for a while

        _mrfGroupDcpt._mrfNotifyFnc = eventHandler;     // set new handler
        _mrfGroupDcpt._mrfNotifyParam = hParam;   
    
        SYS_INT_SourceStatusClear(MRFWB0M_INT_SOURCE);
        SYS_INT_SourceRestore(MRFWB0M_INT_SOURCE, rfILev);   // re-enable
    }
    else
    {
        _mrfGroupDcpt._mrfNotifyFnc = eventHandler;     // set new handler
        _mrfGroupDcpt._mrfNotifyParam = hParam;   
    }
    return TCPIP_MAC_EVRES_OK;
}

// store the user events 
void MRF24W_SetUserEvents(uint8_t event, uint16_t eventInfo, bool isMgmt)
{
    if(isMgmt)
    {
        _mrfUsrEvent.mgmtEvents = event;
        _mrfUsrEvent.mgmtEventInfo = eventInfo;
    }
    else
    {
        _mrfUsrEvent.trafficEvents = event;
        _mrfUsrEvent.trafficEventInfo = eventInfo;
    }
        

} 

// provide the user with the traffic events
uint16_t MRF24W_GetTrafficEvents(uint16_t* pEventInfo)
{
    uint16_t res = _mrfUsrEvent.trafficEvents;
    if(pEventInfo)
    {
        *pEventInfo = _mrfUsrEvent.trafficEventInfo;
    }
    
    _mrfUsrEvent.trafficEvents = 0;
    _mrfUsrEvent.trafficEventInfo = 0;

    return res;
}

// provide the user with the management events
uint16_t MRF24W_GetMgmtEvents(uint16_t* pEventInfo)
{
    uint16_t res = _mrfUsrEvent.mgmtEvents;
    if(pEventInfo)
    {
        *pEventInfo = _mrfUsrEvent.mgmtEventInfo;
    }
    
    _mrfUsrEvent.mgmtEvents = 0;
    _mrfUsrEvent.mgmtEventInfo = 0;

    return res;
}


/**************************************
 * local functions
 ****************************************/


/****************************************************************************
 * Function:        MRF24W_ISR
 *
 * PreCondition:    TCPIPInit, MRF24W_MACEventSetNotifyEvents should have been called.
 *
 * Input:           p - unused
 *
 * Output:          None
 *
 * Side Effects:    None
 *
 * Overview:        This function processes the Ethernet interrupts and reports the events back to the user.
 *
 * Note:            None
 ******************************************************************************/
void MRF24W_ISR(void* p)
{
    
    if(_mrfGroupDcpt._mrfEnabledEvents )
    {
        _mrfGroupDcpt._mrfPendingEvents = TCPIP_EV_RX_PKTPEND|TCPIP_EV_TX_DONE;
        if(_mrfGroupDcpt._mrfNotifyFnc)
        {
            (*_mrfGroupDcpt._mrfNotifyFnc)(_mrfGroupDcpt._mrfNotifyParam, _mrfGroupDcpt._mrfPendingEvents);  
        }
        
    }
    
    SYS_INT_SourceDisable(MRFWB0M_INT_SOURCE);        // disable further interrupts
    SYS_INT_SourceStatusClear(MRFWB0M_INT_SOURCE);      // acknowledge the int Controller

    /* invoke MRF handler */
    WFEintHandler();
}



#endif  // defined(TCPIP_STACK_USE_EVENT_NOTIFICATION)

#endif  // defined(TCPIP_IF_MRF24W)

