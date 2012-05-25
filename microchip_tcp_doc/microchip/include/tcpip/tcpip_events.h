/*******************************************************************************
  TCPIP Events Interface Definition

  Summary:
    This file contains the Application Program Interface (API) definition  for 
    the TCPIP Events library.
    
  Description:
    This library provides a low-level abstraction of the TCPIP events  
    on Microchip PIC32MX family microcontrollers with a convenient C language 
    interface.  It can be used to simplify low-level access to the module 
    without the necessity of interacting directly with the module's registers, 
    thus hiding differences from one microcontroller variant to another.
*******************************************************************************/

/*******************************************************************************
FileName:  tcpip_events.h 
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

#ifndef _TCPIP_EVENTS_H_
#define _TCPIP_EVENTS_H_

#include "hardware_profile.h"
#include "tcpip_config.h"


// *****************************************************************************
// *****************************************************************************
// Section: Constants & Data Types
// *****************************************************************************
// *****************************************************************************

// *****************************************************************************
/* TCPIP Events Codes

  Summary:
    Defines the possible event types.

  Description:
    This enumeration defines all the possible events that can be reported by the TCPIP stack.

    Note that, depending on the type of the hardware Ethernet interface, not all events are possible.

    Note that specific interfaces can offer specific events and functions to retrieve those events.
*/

typedef enum
{
    // no event
    TCPIP_EV_NONE               = 0x0000,
    
    // RX triggered events
    //
    // A receive packet is pending
    TCPIP_EV_RX_PKTPEND         = 0x0001,

    // RX FIFO overflow (system level latency, no descriptors, etc.)            
    TCPIP_EV_RX_OVFLOW          = 0x0002,
    
    // no RX descriptor available to receive a new packet
    TCPIP_EV_RX_BUFNA           = 0x0004,
    
    // There's RX data available 
    TCPIP_EV_RX_ACT             = 0x0008,

    // A packet was sucessfully received
    TCPIP_EV_RX_DONE            = 0x0010,

    // the number of received packets is >= than the RX Full Watermark
    TCPIP_EV_RX_FWMARK          = 0x0020,

    // the number of received packets is <= than the RX Empty Watermark
    TCPIP_EV_RX_EWMARK          = 0x0040,

    // a bus error encountered during an RX transfer
    TCPIP_EV_RX_BUSERR          = 0x0080,


    // TX triggered events.
    //
    // A packet was transmitted and it's status is available
    TCPIP_EV_TX_DONE            = 0x0100,          

    // a TX packet was aborted by the MAC (jumbo/system underrun/excessive defer/late collision/excessive collisions)
    TCPIP_EV_TX_ABORT           = 0x0200,

    // a bus error encountered during a TX transfer
    TCPIP_EV_TX_BUSERR          = 0x0400,


    // Connection triggered events
    // 
    // Connection Established
    TCPIP_EV_CONN_ESTABLISHED   = 0x0800,

    // Connection Lost
    TCPIP_EV_CONN_LOST          = 0x1000,

    // Some useful masks: 

    // Mask of all RX related events 
    TCPIP_EV_RX_ALL             = (TCPIP_EV_RX_PKTPEND|TCPIP_EV_RX_OVFLOW|TCPIP_EV_RX_BUFNA|TCPIP_EV_RX_ACT|
                                   TCPIP_EV_RX_DONE|TCPIP_EV_RX_FWMARK|TCPIP_EV_RX_EWMARK|TCPIP_EV_RX_BUSERR),

    // Mask of all TX related events 
    TCPIP_EV_TX_ALL            = (TCPIP_EV_TX_DONE|TCPIP_EV_TX_ABORT|TCPIP_EV_TX_BUSERR),

    // All events showing some abnormal traffic/system condition
    // Action should be taken accordingly by the stack (or the stack user)
    TCPIP_EV_RXTX_ERRORS        = (TCPIP_EV_RX_OVFLOW|TCPIP_EV_RX_BUFNA|TCPIP_EV_RX_BUSERR|TCPIP_EV_TX_ABORT|TCPIP_EV_TX_BUSERR),

    
    // Mask of all Connection related events 
    TCPIP_EV_CONN_ALL            = (TCPIP_EV_CONN_ESTABLISHED|TCPIP_EV_CONN_LOST),
}TCPIP_EVENT;

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
    TCPIP_EVRES_OK,

    // Some allocation of resources failed
    TCPIP_EVRES_ALLOC_ERR,
    
    // not supported operation/not implemented
    TCPIP_EVRES_OP_ERR,

    //
}TCPIP_EVENT_RESULT;

// *****************************************************************************
/* TCPIP event notification handler Pointer

  Function:
    void* <FunctionName> ( const void* hNet, TCPIP_EVENT tcpEvent )

  Summary:
    Pointer to a function(handler) that will get called to process an event 

  Description:
    Pointer to a function that may be called from within an ISR
    when a TCPIP event is available. 
    
  Precondition:
    None

  Parameters:
    hNet        - network handle  
    tcpEvent    - OR-ed mask of events that occurred
    fParam      - user passed parameter

  Returns:
    None
    
  Remarks:
    This function may be invoked  from within an ISR.  
    It should be kept as short as possible and it should not include
    blocking or polling code.
*/

typedef void (*pTCPIPNotifyF)(const void* hNet, TCPIP_EVENT, void* fParam);


#if defined(TCPIP_STACK_USE_EVENT_NOTIFICATION)

/*******************************************************************************
  Function:
    TCPIP_EVENT_RESULT    TCPIP_STACK_SetNotifyEvents(const void* hNet, TCPIP_EVENT tcpipEvents)

  Summary:
    Adds new events to the list of the enabled ones.

  Description:
     This function sets new notification events.
     Multiple events can be orr-ed together.
     All events that are set will be added to the notification process. The other events will not ne touched.

  Precondition:
   tcpSetEv valid values 

  Parameters:
    hNet    - network handle  
    tcpipEvents - events the user of the stack wants to add for notification
    
  Returns:
    TCPIP_EVRES_OK  if operation succeeded,
    an error code otherwise

  Example:
    <code>
    TCPIP_STACK_SetNotifyEvents( hNet, TCPIP_EV_RX_OVFLOW | TCPIP_EV_RX_BUFNA );
    </code>

  Remarks:
    The event notification system enables the user of the TCPIP stack to call into the stack
    for processing only when there are relevant events rather than being forced to periodically call
    from within a loop.
    

*****************************************************************************/

TCPIP_EVENT_RESULT    TCPIP_STACK_SetNotifyEvents(const void* hNet, TCPIP_EVENT tcpipEvents);

/*******************************************************************************
  Function:
    TCPIP_EVENT_RESULT    TCPIP_STACK_ClearNotifyEvents(const void* hNet, TCPIP_EVENT tcpipEvents)

  Summary:
    Removes events from the list of the enabled ones.

  Description:
     This function removes from the enabled events.
     Multiple events can be orr-ed together.
     All events that are set will be removed from the notification process. The other events will not ne touched.


  Precondition:
   tcpSetEv valid values 

  Parameters:
    hNet    - network handle  
    tcpipEvents - events the user of the stack wants to remove from notification
    
  Returns:
    TCPIP_EVRES_OK  if operation succeeded,
    an error code otherwise

  Example:
    <code>
    TCPIP_STACK_ClearNotifyEvents( hNet, TCPIP_EV_RX_OVFLOW | TCPIP_EV_RX_BUFNA );
    </code>

  Remarks:   
*****************************************************************************/

TCPIP_EVENT_RESULT    TCPIP_STACK_ClearNotifyEvents(const void* hNet, TCPIP_EVENT tcpipEvents);



/*******************************************************************************
  Function:
    TCPIP_EVENT TCPIP_STACK_GetPending(const void* hNet) 

  Summary:
    Returns the currently pending events.

  Description:
    This function returns the currently pending events 
    Multiple events can be orr-ed together as they accumulate.

  Precondition:
    None

  Parameters:
    hNet    - network handle  
    
  Returns:
    The currently TCPIP pending events.

  Example:
    <code>
    TCPIP_EVENT currEvents = TCPIP_STACK_GetPending( hNet);
    </code>

  Remarks:   
    This is the preferred method to get the current pending stack events.
    
    Even with a notification handler in place it's better to use this function to get the current pending events
    
    The returned value is just a momentary value. The pending events can change any time.
*****************************************************************************/

TCPIP_EVENT TCPIP_STACK_GetPending(const void* hNet);


/*******************************************************************************
  Function:
    TCPIP_EVENT_RESULT    TCPIP_STACK_SetNotifyHandler(const void* hNet, pTCPIPNotifyF notifyHandler, void* notifyfParam)

  Summary:
    Sets a new event notification handler.

  Description:
    This function sets a new event notification handler.
    The caller can use the handler to be notified of stack events.

  Precondition:
    None

  Parameters:
    hNet            - network handle  
    notifyHandler   - the event notification handler
    notifyfParam    - notification handler parameter 
    
  Returns:
    TCPIP_EVRES_OK  if operation succeeded,
    an error code otherwise

  Example:
    <code>
    TCPIP_STACK_SetNotifyHandler( hNet, myEventHandler );
    </code>

  Remarks:   
    The notification handler may be called from the ISR which detects the corresponding event.
    The event notification handler has to be kept as short as possible and non-blocking.

    Without a notification handler the stack user can still call TCPIP_STACK_GetPending() to see if processing by the stack needed.

    Use 0 to remove the handler for a specific event.

*****************************************************************************/

TCPIP_EVENT_RESULT    TCPIP_STACK_SetNotifyHandler(const void* hNet, pTCPIPNotifyF notifyHandler, void* notifyfParam);


#endif  // defined(TCPIP_STACK_USE_EVENT_NOTIFICATION)

#endif  // _TCPIP_EVENTS_H_

