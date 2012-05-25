/*******************************************************************************
  TCPIP MAC Events Interface Definition

  Summary:
    This file contains the Application Program Interface (API) definition  for 
    the TCPIP MAC Events library.
    
  Description:
    This library provides a low-level abstraction of the TCPIP events  
    on Microchip PIC32MX family microcontrollers with a convenient C language 
    interface.  It can be used to simplify low-level access to the module 
    without the necessity of interacting directly with the module's registers, 
    thus hiding differences from one microcontroller variant to another.
*******************************************************************************/

/*******************************************************************************
FileName:  mac_events.h 
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

//DOM-IGNORE-END

#ifndef _TCPIP_MAC_EVENTS_H_
#define _TCPIP_MAC_EVENTS_H_

#include "tcpip/tcpip_events.h"

// *****************************************************************************
/* TCPIP MAC Events Groups

  Summary:
    Defines the possible MAC event groups.

  Description:
    This enumeration defines the groups that all events belong to.
    Note that, depending on the type of the hardware Ethernet MAC, not all
    groups are available.
*/

typedef enum
{
    // Generic group aggregating all the generated events.
    // This group should be supported by all implementations.
    // Some implementations may support separate group notifications.
    TCPIP_MAC_EVGROUP_ALL,

    // The group of RX related events.
    // Some of these events require stack processing
    TCPIP_MAC_EVGROUP_RX,
    
    // The group of TX related events.
    // Some of these events require stack processing
    TCPIP_MAC_EVGROUP_TX,
    
    // The group of Connection related events.
    // These events may not require stack processing
    TCPIP_MAC_EVGROUP_CONN,
    //
}TCPIP_MAC_EVENT_GROUP;


// *****************************************************************************
/* TCPIP MAC Events Codes

   Use the same codes as the general TCPIP_EVENT definition.
*/


// *****************************************************************************
/* TCPIP Event Operation Result

  Summary:
    Defines the possible return values from the event API.

  Description:
    This enumeration defines the result codes that could be returned
    from the TCPIP event API.
*/

typedef enum
{
    // Operation succeeded
    TCPIP_MAC_EVRES_OK,

    // Some allocation of resources failed
    TCPIP_MAC_EVRES_ALLOC_ERR,
    
    // The specified event group is not supported.
    TCPIP_MAC_EVRES_GROUP_ERR,
    
    // Some events have been acknowledged for the specified event group
    TCPIP_MAC_EVRES_ACK_OK,
    
    // No events to be acknowledged for the specified event group
    TCPIP_MAC_EVRES_ACK_NONE,

    // no supported MAC type
    TCPIP_MAC_EVRES_MAC_TYPE_ERR,
    
    //
}TCPIP_MAC_EVENT_RESULT;

// *****************************************************************************
/* TCPIP MAC event notification handler Pointer

  Function:
    void* <FunctionName> ( void* hParam, TCPIP_EVENT tcpEvent )

  Summary:
    Pointer to a function(handler) that will get called to process an event 

  Description:
    Pointer to a function that gets called from within an ISR
    when a TCPIP event is available. 
    
  Precondition:
    None

  Parameters:
    hParam        - parameter used when the handler is called
    tcpEvent      - OR-ed mask of events that occurred

  Returns:
    None
    
  Remarks:
    This function will be invoked  from within an ISR.  
    It should be kept as short as possible and it should not include
    blocking or polling code.
*/

typedef void (*pMacEventF)(void* hParam, TCPIP_EVENT);


#if defined(TCPIP_STACK_USE_EVENT_NOTIFICATION)

/*******************************************************************************
  Function:
    TCPIP_MAC_EVENT_RESULT    MACEventInit(TCPIP_MAC_HANDLE hMac, int intPri, int intSubPri)

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
    MACEventInit( hMac, 4, 3 );
    </code>

  Remarks:

    Not multi-threaded safe.

    This function is not part of the MAC interface.
    It is simply called in MACInit();
*****************************************************************************/
TCPIP_MAC_EVENT_RESULT    MACEventInit(TCPIP_MAC_HANDLE hMac, int intPri, int intSubPri);


/*******************************************************************************
  Function:
    TCPIP_MAC_EVENT_RESULT    MACEventDeInit(TCPIP_MAC_HANDLE hMac)

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
    MACEventDeInit( hMac );
    </code>

  Remarks:

    Not multi-threaded safe.

    This function is not part of the MAC interface.
    It is simply called in MACDelete();
*****************************************************************************/
TCPIP_MAC_EVENT_RESULT    MACEventDeInit(TCPIP_MAC_HANDLE hMac);


/*******************************************************************************
  Function:
    TCPIP_MAC_EVENT_RESULT    MACEventSetNotifyEvents(TCPIP_MAC_HANDLE hMac, TCPIP_MAC_EVENT_GROUP tcpEvGroup, TCPIP_EVENT tcpipEvents)

  Summary:
    Adds new events to the list of the enabled ones.

  Description:
     This function sets new enabled events.
     Multiple events can be orr-ed together.
     All events that are set will be added to the notification process. The other events will not ne touched.
     The stack (or stack user) has to catch the events that are notified and process them:
         - Process the specific condition and acknowledge them calling MACEventAck() so that they can be re-enabled.

  Precondition:
   MACEventInit should have been called.
   tcpEvGroup, tcpSetEv valid values 

  Parameters:
    hMac       - parameter identifying the intended MAC 
    tcpEvGroup  - group of events the notification refers to
    tcpipEvents - events the user of the stack wants to add for notification
    
  Returns:
    TCPIP_MAC_EVRES_OK  if operation succeeded,
    an error code otherwise

  Example:
    <code>
    MACEventSetNotifyEvents( hMac, TCPIP_MAC_EVGROUP_RX, TCPIP_EV_RX_OVFLOW | TCPIP_EV_RX_BUFNA );
    </code>

  Remarks:
    The event notification system enables the user of the TCPIP stack to call into the stack
    for processing only when there are relevant events rather than being forced to periodically call
    from within a loop.
    
    If the notification events are nill (accross all groups) the interrupt processing will be disabled.
    Otherwise the event notification will be enabled and the interrupts relating to the requested events will be enabled.
    
    Note that once an event has been caught by the stack ISR (and reported if a notification handler is in place)
    it will be disabled until the MACEventAck() is called.

    Not multi-threaded safe accross different TCPIP_MAC_EVENT_GROUP groups.
*****************************************************************************/

TCPIP_MAC_EVENT_RESULT    MACEventSetNotifyEvents(TCPIP_MAC_HANDLE hMac, TCPIP_MAC_EVENT_GROUP tcpEvGroup, TCPIP_EVENT tcpipEvents);

/*******************************************************************************
  Function:
    TCPIP_MAC_EVENT_RESULT    MACEventClearNotifyEvents(TCPIP_MAC_HANDLE hMac, TCPIP_MAC_EVENT_GROUP tcpEvGroup, TCPIP_EVENT tcpipEvents)

  Summary:
    Removes events from the list of the enabled ones.

  Description:
     This function removes from the enabled events.
     Multiple events can be orr-ed together.
     All events that are set will be removed from the notification process. The other events will not ne touched.


  Precondition:
   MACEventInit should have been called.
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
    MACEventClearNotifyEvents( hMac, TCPIP_MAC_EVGROUP_ALL, TCPIP_EV_RX_OVFLOW | TCPIP_EV_RX_BUFNA );
    </code>

  Remarks:   
    If the notification events are nill (accross all groups) the interrupt processing will be disabled.
    Otherwise the event notification will be enabled and the interrupts relating to the requested events will be enabled.
    
    Not multi-threaded safe accross different TCPIP_MAC_EVENT_GROUP groups.
*****************************************************************************/

TCPIP_MAC_EVENT_RESULT    MACEventClearNotifyEvents(TCPIP_MAC_HANDLE hMac, TCPIP_MAC_EVENT_GROUP tcpEvGroup, TCPIP_EVENT tcpipEvents);

/*******************************************************************************
  Function:
    TCPIP_MAC_EVENT_RESULT    MACEventAck(TCPIP_MAC_HANDLE hMac, TCPIP_MAC_EVENT_GROUP tcpEvGroup, TCPIP_EVENT tcpipEvents)

  Summary:
    Acknowledges and re-enables processed events.

  Description:
    This function acknowledges and re-enables processed events.
    Multiple events can be orr-ed together as they are processed together.
    The events acknowledged by this function should be the events that have been retrieved from the stack
    by calling MACEventGetPending() or have been passed to the user by the stack using the notification handler
    (MACEventSetNotifyHandler()) and have been processed and have to be re-enabled.


  Precondition:
   MACEventInit should have been called.
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
    MACEventAck( hMac, TCPIP_MAC_EVGROUP_ALL, stackNewEvents );
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

TCPIP_MAC_EVENT_RESULT    MACEventAck(TCPIP_MAC_HANDLE hMac, TCPIP_MAC_EVENT_GROUP tcpEvGroup, TCPIP_EVENT tcpipEvents);


/*******************************************************************************
  Function:
    TCPIP_EVENT MACEventGetPending(TCPIP_MAC_HANDLE hMac, TCPIP_MAC_EVENT_GROUP tcpEvGroup)

  Summary:
    Returns the currently pending events.

  Description:
    This function returns the currently pending events belonging to a group.
    Multiple events can be orr-ed together as they accumulate.
    They will have to be eventually acknowledged if re-enabling is needed.

  Precondition:
   MACEventInit should have been called.
   tcpEvGroup valid value 

  Parameters:
    hMac      - parameter identifying the intended MAC 
    tcpEvGroup  - group of events needed
    
  Returns:
    The currently TCPIP pending events.

  Example:
    <code>
    TCPIP_EVENT currEvents = MACEventGetPending( hMac, TCPIP_MAC_EVGROUP_ALL);
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

TCPIP_EVENT MACEventGetPending(TCPIP_MAC_HANDLE hMac, TCPIP_MAC_EVENT_GROUP tcpEvGroup);


/*******************************************************************************
  Function:
    TCPIP_MAC_EVENT_RESULT    MACEventSetNotifyHandler(TCPIP_MAC_HANDLE hMac, TCPIP_MAC_EVENT_GROUP tcpEvGroup, pMacEventF eventHandler, void* hParam)

  Summary:
    Sets a new event notification handler.

  Description:
    This function sets a new event notification handler.
    This is a handler specified by the user of the stack.
    The caller can use the handler to be notified of MAC events.
    Whenever a notification occurs the passed events have to be eventually processed:
    - Process the specific (error) condition
    - Acknowledge the events by calling MACEventAck() so that they can be re-enabled.

  Precondition:
   MACEventInit should have been called.
   tcpEvGroup valid value 

  Parameters:
    hMac      - parameter identifying the intended MAC 
    tcpEvGroup   - group of events the notification handler refers to
    eventHandler - the event notification handler
    hParam       - parameter to be used by the handler
    
  Returns:
    TCPIP_MAC_EVRES_OK  if operation succeeded,
    an error code otherwise

  Example:
    <code>
    MACEventSetNotifyHandler( hMac, TCPIP_MAC_EVGROUP_ALL, myEventHandler, myParam );
    </code>

  Remarks:   
    The notification handler will be called from the ISR which detects the corresponding event.
    The event notification handler has to be kept as short as possible and non-blocking.
    Mainly useful for RTOS integration where this handler will wake-up a thread that waits for a MAC event to occur.

    The event notification system also enables the user of the TCPIP stack to call into the stack
    for processing only when there are relevant events rather than being forced to periodically call
    from within a loop at unknown moments.

    Without a notification handler the stack user can still call MACEventGetPending() to see if processing by the stack needed.

    This is a default way of adding interrupt processing to the stack.
    A separate ISR to catch the Ethernet MAC events and process accordingly
    could be added.

    All the groups specified in the TCPIP_MAC_EVENT_GROUP enumeration is supported.
    However, once a handler for TCPIP_MAC_EVGROUP_ALL is registered all event processing
    will be reported using this global handler.

    Use 0 to remove the handler for a specific event group.

    Not multi-threaded safe accross different TCPIP_MAC_EVENT_GROUP groups.
*****************************************************************************/

TCPIP_MAC_EVENT_RESULT    MACEventSetNotifyHandler(TCPIP_MAC_HANDLE hMac, TCPIP_MAC_EVENT_GROUP tcpEvGroup, pMacEventF eventHandler, void* hParam);

#endif  // defined(TCPIP_STACK_USE_EVENT_NOTIFICATION)

#endif  // _TCPIP_MAC_EVENTS_H_

