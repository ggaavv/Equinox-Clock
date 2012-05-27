/*******************************************************************************
  MRF24W Driver Event Handler

  Summary:
    Module for Microchip TCP/IP Stack
    
  Description:
    -Provides access to MRF24W WiFi controller
    -Reference: MRF24W Data sheet, IEEE 802.11 Standard
*******************************************************************************/

/*******************************************************************************
FileName:   wf_event_handler.c
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

/*==========================================================================*/
/*                                  INCLUDES                                */
/*==========================================================================*/
#include "tcpip_private.h"



#if defined(TCPIP_IF_MRF24W)

#include "wf_debug_output.h"

/*==========================================================================*/
/*                                  DEFINES                                 */
/*==========================================================================*/

/*-------------------------------------------*/
/* Connection Manager Event Message Subtypes */
/* (Used in Mgmt Indicate messages)          */
/*-------------------------------------------*/
#define WF_EVENT_CONNECTION_ATTEMPT_STATUS_SUBTYPE   (6)  
#define WF_EVENT_CONNECTION_LOST_SUBTYPE             (7)
#define WF_EVENT_CONNECTION_REESTABLISHED_SUBTYPE    (8)
#define WF_EVENT_KEY_CALCULATION_COMPLETE_SUBTYPE    (9)
#define WF_EVENT_SCAN_RESULTS_READY_SUBTYPE          (11)
#define WF_EVENT_SCAN_IE_RESULTS_READY_SUBTYPE       (12)

/* event values for index 2 of WF_CONNECTION_ATTEMPT_STATUS_EVENT_SUBTYPE */
#define CONNECTION_ATTEMPT_SUCCESSFUL    ((uint8_t)1)   /* if not 1 then failed to connect and info field is error code */
#define CONNECTION_ATTEMPT_FAILED        ((uint8_t)2)

/* event values for index 2 of WF_EVENT_CONNECTION_LOST_SUBTYPE */
#define CONNECTION_TEMPORARILY_LOST      ((uint8_t)1)
#define CONNECTION_PERMANENTLY_LOST      ((uint8_t)2)
#define CONNECTION_REESTABLISHED         ((uint8_t)3)   

bool g_DhcpRenew = false;

/*==========================================================================*/
/*                                  LOCAL FUNCTIONS                         */
/*==========================================================================*/
#if defined(TCPIP_STACK_USE_EVENT_NOTIFICATION)
    static bool isNotifyApp(uint8_t event);
    static bool isEventNotifyBitSet(uint8_t notifyMask, uint8_t notifyBit);
#else
    void WF_ProcessEvent(uint8_t event, uint16_t eventInfo);
#endif  // defined(TCPIP_STACK_USE_EVENT_NOTIFICATION)

static void RenewDhcp(void);

#define RAW_MGMT_RX_ID   RAW_RX_ID

extern void SetDhcpProgressState(void);
extern void SignalWiFiConnectionChanged(bool state);
void SignalDHCPSuccessful(void);



/*****************************************************************************
 * FUNCTION: WFProcessMgmtIndicateMsg
 *
 * RETURNS:  error code
 *
 * PARAMS:   None
 *
 *  NOTES:   Processes a management indicate message
 *****************************************************************************/
void WFProcessMgmtIndicateMsg()
{
    tMgmtIndicateHdr  hdr;
    uint8_t buf[6];
    uint8_t event = 0xff;
    uint16_t eventInfo;

    /* read 2-byte header of management message */
    RawRead(RAW_MGMT_RX_ID, 0, sizeof(tMgmtIndicateHdr), (uint8_t *)&hdr); 
        
    /* Determine which event occurred and handle it */
    switch (hdr.subType)
    {
        /*-----------------------------------------------------------------*/        
        case WF_EVENT_CONNECTION_ATTEMPT_STATUS_SUBTYPE:
        /*-----------------------------------------------------------------*/
            /* There is one data byte with this message */
            RawRead(RAW_MGMT_RX_ID, sizeof(tMgmtIndicateHdr), 1, buf); /* read first byte after header */ 
            /* if connection attempt successful */
            if (buf[0] == CONNECTION_ATTEMPT_SUCCESSFUL)
            {
                event     = WF_EVENT_CONNECTION_SUCCESSFUL;
                eventInfo = WF_NO_ADDITIONAL_INFO;
                SignalWiFiConnectionChanged(true);
                #if defined (TCPIP_STACK_USE_DHCP_CLIENT)
                RenewDhcp();
                #endif
                SetLogicalConnectionState(true);
            }
            /* else connection attempt failed */
            else 
            {
                event     = WF_EVENT_CONNECTION_FAILED;
                eventInfo = (uint16_t)buf[0];             /* contains connection failure code */
                SetLogicalConnectionState(false);                
            }  
            break;
            
        /*-----------------------------------------------------------------*/
        case WF_EVENT_CONNECTION_LOST_SUBTYPE:
        /*-----------------------------------------------------------------*/ 
            /* read index 2 and 3 from message and store in buf[0] and buf[1]
               buf[0] -- 1: Connection temporarily lost  2: Connection permanently lost 3: Connection Reestablished 
               buf[1] -- 0: Beacon Timeout  1: Deauth from AP  */                         
            RawRead(RAW_MGMT_RX_ID, sizeof(tMgmtIndicateHdr), 2, buf); 

            if (buf[0] == CONNECTION_TEMPORARILY_LOST)
            {
                event     = WF_EVENT_CONNECTION_TEMPORARILY_LOST;
                eventInfo = (uint16_t)buf[1];    /* lost due to beacon timeout or deauth */
                SignalWiFiConnectionChanged(false);
                
                SetLogicalConnectionState(false);   // TEST!!!!
            }    
            else if (buf[0] == CONNECTION_PERMANENTLY_LOST)
            {
                event     = WF_EVENT_CONNECTION_PERMANENTLY_LOST;
                eventInfo = (uint16_t)buf[1];   /* lost due to beacon timeout or deauth */
                SetLogicalConnectionState(false);
                SignalWiFiConnectionChanged(false);                
            }
            else if (buf[0] == CONNECTION_REESTABLISHED)
            {
                event     = WF_EVENT_CONNECTION_REESTABLISHED;
                eventInfo = (uint16_t)buf[1];    /* originally lost due to beacon timeout or deauth */
				#if defined(TCPIP_STACK_USE_DHCP_CLIENT)
                RenewDhcp();
				#endif
                SignalWiFiConnectionChanged(true);  
                
                SetLogicalConnectionState(true);   // TEST!!!!
                              
            }    
            else
            {
                /* invalid parameter in message */
                SYS_ASSERT(false, "");
            }        
            break;
        
        /*-----------------------------------------------------------------*/                    
        case WF_EVENT_SCAN_RESULTS_READY_SUBTYPE:        
        /*-----------------------------------------------------------------*/        
            /* read index 2 of mgmt indicate to get the number of scan results */
            RawRead(RAW_MGMT_RX_ID, sizeof(tMgmtIndicateHdr), 1, buf);            
            event = WF_EVENT_SCAN_RESULTS_READY;
            eventInfo = (uint16_t)buf[0];          /* number of scan results */
            break;
            
        /*-----------------------------------------------------------------*/
        case WF_EVENT_SCAN_IE_RESULTS_READY_SUBTYPE:
        /*-----------------------------------------------------------------*/
            event = WF_EVENT_IE_RESULTS_READY;
            /* read indexes 2 and 3 containing the 16-bit value of IE bytes */
            RawRead(RAW_MGMT_RX_ID, sizeof(tMgmtIndicateHdr), 2, (uint8_t *)&eventInfo);
            eventInfo = WFSTOHS(eventInfo);     /* fix endianess of 16-bit value */
            break;    
        
        /*-----------------------------------------------------------------*/
        default:
        /*-----------------------------------------------------------------*/
            SYS_ASSERT(false, "");
            break;        
    }
    
    /* free mgmt buffer */
    DeallocateMgmtRxBuffer();

    /* if the application wants to be notified of the event */
#if defined(TCPIP_STACK_USE_EVENT_NOTIFICATION)
    if (isNotifyApp(event))
    {
        MRF24W_SetUserEvents(event, eventInfo, 1);  
    }    
// TCPIP_STACK_USE_EVENT_NOTIFICATION not currently working, so call directly
#else
    WF_ProcessEvent(event, eventInfo);
#endif  
}

#if defined(TCPIP_STACK_USE_DHCP_CLIENT)
static void RenewDhcp(void)
{
    g_DhcpRenew = true;
    SetDhcpProgressState();
}    
#endif




#if defined(TCPIP_STACK_USE_EVENT_NOTIFICATION)
/*****************************************************************************
 * FUNCTION: isEventNotifyBitSet
 *
 * RETURNS:  true if the notify bit is set in the notify mask.
 *
 * PARAMS:   notifyMask -- the bit mask of events the application wishes to be 
 *                         notified of
 *           notifyBit  -- the specific event that occurred
 *
 *  NOTES:   Determines if the input event it enabled in the notify mask
 *****************************************************************************/
static bool isEventNotifyBitSet(uint8_t notifyMask, uint8_t notifyBit)
{
    /* check if the event notify bit is set */    
    return ((notifyMask & notifyBit) > 0);
}

/*****************************************************************************
 * FUNCTION: isNotifyApp
 *
 * RETURNS:  true if application wants to be notified of event, else false
 *
 * PARAMS:   event -- the event that occurred
 *
 *  NOTES:   Determines if the input event is one which the application should be 
 *           notified of.
 *****************************************************************************/
static bool isNotifyApp(uint8_t event)
{
    bool notify = false;
    uint8_t notifyMask = GetEventNotificationMask();
    
    /* determine if user wants to be notified of event */
    switch (event)
    {
        case WF_EVENT_CONNECTION_SUCCESSFUL:
            if (isEventNotifyBitSet(notifyMask, WF_NOTIFY_CONNECTION_ATTEMPT_SUCCESSFUL))      
            {
                notify = true;          
            }    
            break;
            
        case WF_EVENT_CONNECTION_FAILED:
            if (isEventNotifyBitSet(notifyMask, WF_NOTIFY_CONNECTION_ATTEMPT_FAILED))      
            {
                notify = true;          
            }    
            break;
            
        case WF_EVENT_CONNECTION_TEMPORARILY_LOST:
            if (isEventNotifyBitSet(notifyMask, WF_NOTIFY_CONNECTION_TEMPORARILY_LOST))      
            {
                notify = true;          
            }    
            break;
            
        case WF_EVENT_CONNECTION_PERMANENTLY_LOST:
            if (isEventNotifyBitSet(notifyMask, WF_NOTIFY_CONNECTION_PERMANENTLY_LOST))      
            {
                notify = true;          
            }    
            break;
            
        case WF_EVENT_CONNECTION_REESTABLISHED:
            if (isEventNotifyBitSet(notifyMask, WF_NOTIFY_CONNECTION_REESTABLISHED))      
            {
                notify = true;          
            }    
            break;
 
        default:
            notify = true;  /* the app gets notified of all other events */
            break;
    }  
    
    return notify;  
    
}  


#endif  // defined(TCPIP_STACK_USE_EVENT_NOTIFICATION)

// callback function; stack calls this upon DHCP events
void DhcpEventHandler(TCPIP_NET_HANDLE hNet, DHCP_EVENT_TYPE evType, const void* param)
{
    
    if (evType == DHCP_EVENT_BOUND)
    {
        SignalDHCPSuccessful();
    }    
}    





#endif /* TCPIP_IF_MRF24W */


