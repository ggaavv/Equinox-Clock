/*******************************************************************************
  tcpip PIC32 MAC events implementation

  Summary:
    
  Description:
*******************************************************************************/

/*******************************************************************************
FileName:   mac_events.c
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

#include "peripheral/eth.h"
#include "peripheral/int.h"

#include "mac_private.h"
#include "tcpip/tcpip_mac_object.h"


// PIC32 embedded MAC implementation

#if ( ((__PIC32_FEATURE_SET__ >= 500) && (__PIC32_FEATURE_SET__ <= 799) && defined (_ETH)) && defined(TCPIP_IF_PIC32INT) )

#if defined(TCPIP_STACK_USE_EVENT_NOTIFICATION)


#include "eth_pic32_int_mac.h"

/*************************
 * local data
 *************************/


static TCPIP_MAC_HANDLE         _hEventMac;     // the MAC we belong to

#define     _PIC32_MAC_EV_GROUPS_  sizeof(((PIC32_EMB_MAC_DATA*)0)->_pic32_ev_group_dcpt)/sizeof(*((PIC32_EMB_MAC_DATA*)0)->_pic32_ev_group_dcpt)

static const unsigned int       _pic32_mac_group_ev_mask[_PIC32_MAC_EV_GROUPS_] = 
{
    0xffffffff,                  // TCPIP_MAC_EVGROUP_ALL
    TCPIP_EV_RX_ALL,             // TCPIP_MAC_EVGROUP_RX
    TCPIP_EV_TX_ALL,             // TCPIP_MAC_EVGROUP_TX
};



/*********************************
 *  local proto 
 ******************************************/

static void TcpIpEthIsr(void* p);


/****************************************************************************
 * Function:        _XtlEventsTcp2Eth
 *
 * PreCondition:    None
 *
 * Input:           tcpEv - the TcpIp events to be translated
 *
 * Output:          Ethernet events
 *
 * Side Effects:    None
 *
 * Overview:        This function translates from  events to Ethernet events
 *
 * Note:            - The translation could be done in a more systematic way
 *                    using a (few) LUT(s).
 *                    For now this direct approach is taken.
 *                  - The CONN_LOST/CONN_ESTABLISHED events do NOT map to Ethernet events! 
 *                    The current translation function assumes no overlap between a CONN event
 *                    and an Ethernet one!
 *                    By this translation the CONN_LOST/CONN_ESTABLISHED event is lost!
 ******************************************************************************/
/*static __inline__*/static  eEthEvents /*__attribute__((always_inline))*/ _XtlEventsTcp2Eth(TCPIP_EVENT tcpEv)
{
    eEthEvents  eEvents;

    eEvents =  (tcpEv&(TCPIP_EV_TX_BUSERR))<<4;
    eEvents |= (tcpEv&(TCPIP_EV_TX_ABORT))>>7;
    eEvents |= (tcpEv&(TCPIP_EV_TX_DONE))>>5;
    eEvents |= (tcpEv&(TCPIP_EV_RX_BUSERR|TCPIP_EV_RX_PKTPEND))<<6;
    eEvents |= (tcpEv&(TCPIP_EV_RX_EWMARK|TCPIP_EV_RX_FWMARK|TCPIP_EV_RX_DONE))<<3;
    eEvents |= (tcpEv&(TCPIP_EV_RX_ACT))<<2;
    eEvents |= (tcpEv&(TCPIP_EV_RX_BUFNA|TCPIP_EV_RX_OVFLOW))>>1;
    
    return eEvents;
}

/****************************************************************************
 * Function:        _XtlEventsEth2Tcp
 *
 * PreCondition:    None
 *
 * Input:           eEvents - the Eth events to be translated
 *
 * Output:          TcpIp events
 *
 * Side Effects:    None
 *
 * Overview:        This function translates from Ethernet events to TcpIp events
 *
 * Note:            - The translation could be done in a more systematic way
 *                    using a (few) LUT(s).
 *                    For now this direct approach is taken.
 *                  - The CONN_LOST/CONN_ESTABLISHED events do NOT map to Ethernet events! 
 *                    The current translation function assumes no overlap between a CONN event
 *                    and an Ethernet one!
 *                    By this translation a CONN_LOST/CONN_ESTABLISHED event cannot be generated!
 ******************************************************************************/
/*static __inline__*/static  TCPIP_EVENT /*__attribute__((always_inline))*/ _XtlEventsEth2Tcp(eEthEvents eEvents)
{
    TCPIP_EVENT tcpEv;

    tcpEv =  (eEvents&(_ETHIRQ_TXBUSE_MASK))>>4;
    tcpEv |= (eEvents&(_ETHIRQ_TXABORT_MASK))<<7;
    tcpEv |= (eEvents&(_ETHIRQ_TXDONE_MASK))<<5;
    tcpEv |= (eEvents&(_ETHIRQ_RXBUSE_MASK|_ETHIRQ_PKTPEND_MASK))>>6;
    tcpEv |= (eEvents&(_ETHIRQ_EWMARK_MASK|_ETHIRQ_FWMARK_MASK|_ETHIRQ_RXDONE_MASK))>>3;
    tcpEv |= (eEvents&(_ETHIRQ_RXACT_MASK))>>2;
    tcpEv |= (eEvents&(_ETHIRQ_RXBUFNA_MASK|_ETHIRQ_RXOVFLW_MASK))<<1;
    
    return tcpEv;
}

/*********************************
 *  implementation
 ******************************************/

/*******************************************************************************
  Function:
    TCPIP_MAC_EVENT_RESULT    PIC32MACEventInit(TCPIP_MAC_HANDLE hMac, int intPri, int intSubPri)

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
    PIC32MACEventInit( hMac, 4, 3 );
    </code>

  Remarks:

    Not multi-threaded safe.
*****************************************************************************/
TCPIP_MAC_EVENT_RESULT PIC32MACEventInit(TCPIP_MAC_HANDLE hMac, int intPri, int intSubPri)
{
    int grpIx;
    PIC32_EV_GROUP_DCPT*    pDcpt; 
    PIC32_EMB_MAC_DCPT*     pMacD = (PIC32_EMB_MAC_DCPT*)hMac;

    SYS_INT_SourceDisable(pMacD->mData._macIntSrc);      // stop Eth ints
    SYS_INT_SourceStatusClear(pMacD->mData._macIntSrc);
    SYS_INT_PrioritySet(pMacD->mData._macIntSrc, (INT_PRIORITY_LEVEL_1-1)+intPri);
    SYS_INT_SubprioritySet(pMacD->mData._macIntSrc, INT_SUB_PRIORITY_LEVEL_0+intSubPri);
    SYS_INT_DynamicRegister(pMacD->mData._macIntSrc, TcpIpEthIsr, pMacD); 
    
    pDcpt = pMacD->mData._pic32_ev_group_dcpt;
    for(grpIx = 0; grpIx < sizeof(pMacD->mData._pic32_ev_group_dcpt)/sizeof(*pMacD->mData._pic32_ev_group_dcpt); grpIx++)
    {
        pDcpt->_TcpGroupEventsMask = _pic32_mac_group_ev_mask[grpIx];
        pDcpt->_TcpEnabledEvents = pDcpt->_TcpPendingEvents = TCPIP_EV_NONE;
        pDcpt->_EthEnabledEvents = pDcpt->_EthPendingEvents = 0;
        pDcpt->_TcpNotifyFnc = 0;
        pDcpt++;
    }

    _hEventMac = hMac;    
    return TCPIP_MAC_EVRES_OK;
}

/*******************************************************************************
  Function:
    TCPIP_MAC_EVENT_RESULT    PIC32MACEventDeInit(TCPIP_MAC_HANDLE hMac )

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
    PIC32MACEventDeInit( hMac );
    </code>

  Remarks:

    Not multi-threaded safe.
*****************************************************************************/
TCPIP_MAC_EVENT_RESULT PIC32MACEventDeInit(TCPIP_MAC_HANDLE hMac)
{
    int grpIx;
    PIC32_EV_GROUP_DCPT*  pDcpt; 
    PIC32_EMB_MAC_DCPT*     pMacD = (PIC32_EMB_MAC_DCPT*)hMac;


    SYS_INT_SourceDisable(pMacD->mData._macIntSrc);      // stop Eth ints
    SYS_INT_SourceStatusClear(pMacD->mData._macIntSrc);
    SYS_INT_DynamicRegister(pMacD->mData._macIntSrc, 0, 0); 

    pDcpt = pMacD->mData._pic32_ev_group_dcpt;
    for(grpIx = 0; grpIx < sizeof(pMacD->mData._pic32_ev_group_dcpt)/sizeof(*pMacD->mData._pic32_ev_group_dcpt); grpIx++)
    {
        pDcpt->_TcpNotifyFnc = 0;
        pDcpt->_TcpEnabledEvents = pDcpt->_TcpPendingEvents = TCPIP_EV_NONE;
        pDcpt->_EthEnabledEvents = pDcpt->_EthPendingEvents = 0;
        pDcpt++;
    }

    _hEventMac = 0;
    return TCPIP_MAC_EVRES_OK;
}


/*******************************************************************************
  Function:
    TCPIP_MAC_EVENT_RESULT    PIC32MACEventSetNotifyEvents(TCPIP_MAC_HANDLE hMac, TCPIP_MAC_EVENT_GROUP tcpEvGroup, TCPIP_EVENT tcpipEvents)

  Summary:
    Adds new events to the list of the enabled ones.

  Description:
     This function sets new enabled events.
     Multiple events can be orr-ed together.
     All events that are set will be added to the notification process. The other events will not ne touched.
     The stack (or stack user) has to catch the events that are notified and process them:
         - The stack should process the TCPIP_EV_RX_PKTPEND, TCPIP_EV_TX_DONE transfer events
         - Process the specific condition and acknowledge them calling PIC32MACEventAck() so that they can be re-enabled.

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
    PIC32MACEventSetNotifyEvents( hMac, TCPIP_MAC_EVGROUP_RX, TCPIP_EV_RX_OVFLOW | TCPIP_EV_RX_BUFNA );
    </code>

  Remarks:
    The event notification system enables the user of the TCPIP stack to call into the stack
    for processing only when there are relevant events rather than being forced to periodically call
    from within a loop.
    
    If the notification events are nill (accross all groups) the interrupt processing will be disabled.
    Otherwise the event notification will be enabled and the interrupts relating to the requested events will be enabled.
    
    Note that once an event has been caught by the stack ISR (and reported if a notification handler is in place)
    it will be disabled until the PIC32MACEventAck() is called.

    Not multi-threaded safe accross different TCPIP_MAC_EVENT_GROUP groups.
*****************************************************************************/
TCPIP_MAC_EVENT_RESULT PIC32MACEventSetNotifyEvents(TCPIP_MAC_HANDLE hMac, TCPIP_MAC_EVENT_GROUP tcpEvGroup, TCPIP_EVENT tcpSetEv)
{
    eEthEvents  ethSetEvents;
    PIC32_EMB_MAC_DCPT*     pMacD = (PIC32_EMB_MAC_DCPT*)hMac;
    PIC32_EV_GROUP_DCPT*  pDcpt = pMacD->mData._pic32_ev_group_dcpt+tcpEvGroup; 

    tcpSetEv &= pDcpt->_TcpGroupEventsMask; 
    ethSetEvents = _XtlEventsTcp2Eth(tcpSetEv);
    
    if(pDcpt->_TcpEnabledEvents != 0)
    {   // already have some active     
        SYS_INT_SourceDisable(pMacD->mData._macIntSrc);      // stop ints for a while
    }
    
    pDcpt->_TcpEnabledEvents |= tcpSetEv;        // add more
    pDcpt->_EthEnabledEvents |= ethSetEvents;  
    
    if(pDcpt->_TcpEnabledEvents != 0)
    {
        ethSetEvents &= ~pDcpt->_EthPendingEvents;     // keep just the new un-ack events
        EthEventsClr( ethSetEvents );                       // clear the old pending ones
        EthEventsEnableSet( ethSetEvents );                 // enable the new un-ack ones!
        
        SYS_INT_SourceEnable(pMacD->mData._macIntSrc);       // enable
    }

    return TCPIP_MAC_EVRES_OK;
}



/*******************************************************************************
  Function:
    TCPIP_MAC_EVENT_RESULT PIC32MACEventClearNotifyEvents(TCPIP_MAC_HANDLE hMac, TCPIP_MAC_EVENT_GROUP tcpEvGroup, TCPIP_EVENT tcpipEvents)

  Summary:
    Removes events from the list of the enabled ones.

  Description:
     This function removes from the enabled events.
     Multiple events can be orr-ed together.
     All events that are set will be removed from the notification process. The other events will not ne touched.


  Precondition:
   TCPIPInit should have been called.
   tcpEvGroup, tcpClrEv valid values 

  Parameters:
    hMac      - parameter identifying the intended MAC  
    tcpEvGroup  - group of events the notification refers to
    tcpipEvents - events the user of the stack wants to remove from notification
    
  Returns:
    TCPIP_MAC_EVRES_OK  if operation succeeded,
    an error code otherwise

  Example:
    <code>
    PIC32MACEventClearNotifyEvents( hMac, TCPIP_MAC_EVGROUP_ALL, TCPIP_EV_RX_OVFLOW | TCPIP_EV_RX_BUFNA );
    </code>

  Remarks:   
    If the notification events are nill (accross all groups) the interrupt processing will be disabled.
    Otherwise the event notification will be enabled and the interrupts relating to the requested events will be enabled.
    
    Not multi-threaded safe accross different TCPIP_MAC_EVENT_GROUP groups.
*****************************************************************************/
TCPIP_MAC_EVENT_RESULT PIC32MACEventClearNotifyEvents(TCPIP_MAC_HANDLE hMac, TCPIP_MAC_EVENT_GROUP tcpEvGroup, TCPIP_EVENT tcpClrEv)
{
    eEthEvents  ethClrEvents;
    int         ethILev = 0;
    PIC32_EMB_MAC_DCPT*     pMacD = (PIC32_EMB_MAC_DCPT*)hMac;
    PIC32_EV_GROUP_DCPT*  pDcpt = pMacD->mData._pic32_ev_group_dcpt+tcpEvGroup; 

    tcpClrEv &= pDcpt->_TcpGroupEventsMask; 
    
    tcpClrEv &= pDcpt->_TcpEnabledEvents;                  // keep just the enabled ones
    ethClrEvents = _XtlEventsTcp2Eth(tcpClrEv);
    
    if(pDcpt->_TcpEnabledEvents != 0)
    {   // already have some active     
        ethILev = SYS_INT_SourceDisable(pMacD->mData._macIntSrc);      // stop ints for a while
    }

    pDcpt->_TcpEnabledEvents &= ~tcpClrEv;     // clear some of them
    pDcpt->_EthEnabledEvents &= ~ethClrEvents;

    pDcpt->_TcpPendingEvents &= ~tcpClrEv;     // remove them from un-ack list
    pDcpt->_EthPendingEvents &= ~ethClrEvents;

    EthEventsEnableClr(ethClrEvents);   // no longer enabled
    EthEventsClr(ethClrEvents);         // clear the pending ones

    if(pDcpt->_TcpEnabledEvents != 0)
    {
        SYS_INT_SourceRestore(pMacD->mData._macIntSrc, ethILev);   // re-enable
    }

    return TCPIP_MAC_EVRES_OK;
}

/*******************************************************************************
  Function:
    TCPIP_MAC_EVENT_RESULT    PIC32MACEventAck(TCPIP_MAC_HANDLE hMac, TCPIP_MAC_EVENT_GROUP tcpEvGroup, TCPIP_EVENT tcpAckEv)

  Summary:
    Acknowledges and re-enables processed events.

  Description:
    This function acknowledges and re-enables processed events.
    Multiple events can be orr-ed together as they are processed together.
    The events acknowledged by this function should be the events that have been retrieved from the stack
    by calling PIC32MACEventGetPending() or have been passed to the user by the stack using the notification handler
    (PIC32MACEventSetNotifyHandler()) and have been processed and have to be re-enabled.


  Precondition:
   TCPIPInit should have been called.
   tcpEvGroup, tcpAckEv valid values 

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
    PIC32MACEventAck( hMac, TCPIP_MAC_EVGROUP_ALL, stackNewEvents );
    </code>

  Remarks:   
    All events should be acknowledged, in order to be re-enabled.

    Some events are fatal errors and should not be acknowledged (TCPIP_EV_RX_BUSERR, TCPIP_EV_TX_BUSERR).
    Stack re-initialization is needed under such circumstances.

    Some events are just system/application behavior and they are intended only as simple info (TCPIP_EV_RX_OVFLOW,
    TCPIP_EV_RX_BUFNA, TCPIP_EV_TX_ABORT, TCPIP_EV_RX_ACT, TCPIP_EV_RX_DONE).

    The TCPIP_EV_RX_FWMARK and TCPIP_EV_RX_EWMARK events are part of the normal flow control operation (if auto flow control was enabled).
    They should be enabled alternatively, if needed.

    The events are persistent. They shouldn't be re-enabled unless they have been processed and the condition that generated them was removed.
    Re-enabling them immediately without proper processing will have dramatic effects on system performance.

    Not multi-threaded safe accross different TCPIP_MAC_EVENT_GROUP groups.
*****************************************************************************/
TCPIP_MAC_EVENT_RESULT PIC32MACEventAck(TCPIP_MAC_HANDLE hMac, TCPIP_MAC_EVENT_GROUP tcpEvGroup, TCPIP_EVENT tcpAckEv)
{
    int                   ethILev;
    PIC32_EMB_MAC_DCPT*     pMacD = (PIC32_EMB_MAC_DCPT*)hMac;
    PIC32_EV_GROUP_DCPT*  pDcpt = pMacD->mData._pic32_ev_group_dcpt+tcpEvGroup; 

    tcpAckEv &= pDcpt->_TcpGroupEventsMask;

    if(pDcpt->_TcpEnabledEvents != 0)
    {   // already have some active     
        eEthEvents  ethAckEv;

        ethAckEv=_XtlEventsTcp2Eth(tcpAckEv);

        ethILev = SYS_INT_SourceDisable(pMacD->mData._macIntSrc);  // stop ints for a while

        pDcpt->_TcpPendingEvents &= ~tcpAckEv;         // no longer pending

        pDcpt->_EthPendingEvents &= ~ethAckEv;         // no longer pending

        EthEventsClr(ethAckEv);                 // clear the old pending ones
        EthEventsEnableSet(ethAckEv);           // re-enable the ack ones
        
        SYS_INT_SourceRestore(pMacD->mData._macIntSrc, ethILev);   // re-enable
        return TCPIP_MAC_EVRES_ACK_OK;
    }

    return TCPIP_MAC_EVRES_ACK_NONE;
}


/*******************************************************************************
  Function:
    TCPIP_EVENT PIC32MACEventGetPending(TCPIP_MAC_HANDLE hMac, TCPIP_MAC_EVENT_GROUP tcpEvGroup)

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
    TCPIP_EVENT currEvents = PIC32MACEventGetPending( hMac, TCPIP_MAC_EVGROUP_ALL);
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
TCPIP_EVENT PIC32MACEventGetPending(TCPIP_MAC_HANDLE hMac, TCPIP_MAC_EVENT_GROUP tcpEvGroup)
{
    PIC32_EMB_MAC_DCPT*     pMacD = (PIC32_EMB_MAC_DCPT*)hMac;
    return (pMacD->mData._pic32_ev_group_dcpt+tcpEvGroup)->_TcpPendingEvents;
}

/*******************************************************************************
  Function:
    TCPIP_MAC_EVENT_RESULT    PIC32MACEventSetNotifyHandler(TCPIP_MAC_HANDLE hMac, TCPIP_MAC_EVENT_GROUP tcpEvGroup, pMacEventF eventHandler, void* hParam)

  Summary:
    Sets a new event notification handler.

  Description:
    This function sets a new event notification handler.
    This is a handler specified by the user of the stack.
    The caller can use the handler to be notified of MAC events.
    Whenever a notification occurs the passed events have to be eventually processed:
    - Stack should process the TCPIP_EV_RX_PKTPEND, TCPIP_EV_TX_DONE  events
    - Process the specific (error) condition
    - Acknowledge the events by calling PIC32MACEventAck() so that they can be re-enabled.

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
    PIC32MACEventSetNotifyHandler( hMac, TCPIP_MAC_EVGROUP_ALL, myEventHandler, myParam );
    </code>

  Remarks:   
    The notification handler will be called from the ISR which detects the corresponding event.
    The event notification handler has to be kept as short as possible and non-blocking.
    Mainly useful for RTOS integration where this handler will wake-up a thread that waits for a MAC event to occur.

    The event notification system also enables the user of the TCPIP stack to call into the stack
    for processing only when there are relevant events rather than being forced to periodically call
    from within a loop at unknown moments.

    Without a notification handler the stack user can still call PIC32MACEventGetPending() to see if processing by the stack needed.

    This is a default way of adding interrupt processing to the stack.
    A separate ISR to catch the Ethernet MAC events and process accordingly
    could be added.

    All the groups specified in the TCPIP_MAC_EVENT_GROUP enumeration is supported.
    However, once a handler for TCPIP_MAC_EVGROUP_ALL is registered all event processing
    will be reported using this global handler.

    Use 0 to remove the handler for a specific event group.

    Not multi-threaded safe accross different TCPIP_MAC_EVENT_GROUP groups.
*****************************************************************************/
TCPIP_MAC_EVENT_RESULT PIC32MACEventSetNotifyHandler(TCPIP_MAC_HANDLE hMac, TCPIP_MAC_EVENT_GROUP tcpEvGroup, pMacEventF eventHandler, void* hParam)
{
    int                   ethILev;
    PIC32_EMB_MAC_DCPT*     pMacD = (PIC32_EMB_MAC_DCPT*)hMac;
    PIC32_EV_GROUP_DCPT*  pDcpt = pMacD->mData._pic32_ev_group_dcpt+tcpEvGroup; 
    
    if(pDcpt->_TcpEnabledEvents != 0)
    {   // already have some active     
        ethILev = SYS_INT_SourceDisable(pMacD->mData._macIntSrc);      // stop ints for a while

        pDcpt->_TcpNotifyFnc = eventHandler;     // set new handler
        pDcpt->_TcpNotifyParam = hParam;
    
        SYS_INT_SourceStatusClear(pMacD->mData._macIntSrc);
        SYS_INT_SourceRestore(pMacD->mData._macIntSrc, ethILev);   // re-enable
    }
    else
    {
        pDcpt->_TcpNotifyFnc = eventHandler;     // set new handler
        pDcpt->_TcpNotifyParam = hParam;
    }
    return TCPIP_MAC_EVRES_OK;
}



/**************************************
 * local functions
 ****************************************/


/****************************************************************************
 * Function:        TcpIpEthIsr
 *
 * PreCondition:    TCPIPInit, PIC32MACEventSetNotifyEvents should have been called.
 *
 * Input:           p - PIC32 MAC descriptor pointer
 *
 * Output:          None
 *
 * Side Effects:    None
 *
 * Overview:        This function processes the Ethernet interrupts and reports the events back to the user.
 *
 * Note:            None
 ******************************************************************************/
static void TcpIpEthIsr(void* p)
{
    eEthEvents          currEthEvents, currGroupEvents;
    PIC32_EV_GROUP_DCPT* pDcpt;
    int                 grpIx;
    PIC32_EMB_MAC_DCPT* pMacD = (PIC32_EMB_MAC_DCPT*)p;

    currEthEvents = EthEventsGet();

    // process per group
    pDcpt = pMacD->mData._pic32_ev_group_dcpt;
    for(grpIx = 0; grpIx < sizeof(pMacD->mData._pic32_ev_group_dcpt)/sizeof(*pMacD->mData._pic32_ev_group_dcpt); grpIx++)
    {
        currGroupEvents = currEthEvents & pDcpt->_EthEnabledEvents;     //  keep just the relevant ones
        if(currGroupEvents)
        {
            pDcpt->_EthPendingEvents |= currGroupEvents;                    // add the new events
            pDcpt->_TcpPendingEvents |= _XtlEventsEth2Tcp(currGroupEvents);

            EthEventsEnableClr(currGroupEvents);         // these will get reported; disable them until ack is received back
            EthEventsClr(currGroupEvents);               // acknowledge the ETHC
            if(pDcpt->_TcpNotifyFnc)
            {
                (*pDcpt->_TcpNotifyFnc)(pDcpt->_TcpNotifyParam, pDcpt->_TcpPendingEvents);     // let the user know
            }
        }
        pDcpt++;
    }



    SYS_INT_SourceStatusClear(pMacD->mData._macIntSrc);         // acknowledge the int Controller
}


#endif  // defined(TCPIP_STACK_USE_EVENT_NOTIFICATION)


#endif  // ( ((__PIC32_FEATURE_SET__ >= 500) && (__PIC32_FEATURE_SET__ <= 799) && defined (_ETH)) && defined(TCPIP_IF_PIC32INT) )

