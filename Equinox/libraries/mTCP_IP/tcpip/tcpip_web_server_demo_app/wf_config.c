/*******************************************************************************
  MRF24W Driver Customization

  Summary:
    Module for Microchip TCP/IP Stack
    
  Description:
    -Provides access to MRF24W WiFi controller
    -Reference: MRF24W Data sheet, IEEE 802.11 Standard
*******************************************************************************/

/*******************************************************************************
FileName:  WF_Config.c 
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

#include "hardware_profile.h"

#if defined(TCPIP_IF_MRF24W)

/*==========================================================================*/
/*                                  INCLUDES                                */
/*==========================================================================*/
#include "tcpip/tcpip.h"
#include "wifi/wf_debug_output.h"

#if defined ( EZ_CONFIG_SCAN )
#include "tcpip/wf_easy_config.h"
#endif /* EZ_CONFIG_SCAN */


/*==========================================================================*/
/*                                  DEFINES                                 */
/*==========================================================================*/


/*****************************************************************************
 * FUNCTION: WF_ProcessEvent
*
 * RETURNS:  None
*
 * PARAMS:   event      -- event that occurred
 *           eventInfo  -- additional information about the event.  Not all events
 *                         have associated info, in which case this value will be
 *                         set to WF_NO_ADDITIONAL_INFO (0xff)
*
 *  NOTES:   The Host application must NOT directly call this function.  This 
 *           function is called by the WiFi Driver code when an event occurs
 *           that may need processing by the Host CPU.  
*
 *           No other WiFi Driver function should be called from this function.
*                   It is recommended that if the application wishes to be 
*                   notified of an event that it simply set a flag and let application code in the 
*                   main loop handle the event. 
*               
 *           WFSetFuncState must be called when entering and leaving this function.  
*                   When SYS_ASSERT is enabled this allows a runtime check if any illegal WF functions 
*                   are called from within this function.
* 
 *           For events that the application is not interested in simply leave the
 *           case statement empty.
 *
 *
 *           Customize this function as needed for your application.
 *****************************************************************************/
void WF_ProcessEvent(uint8_t event, uint16_t eventInfo)
{
    char buf[8];
  
    /* this function tells the WF driver that we are in this function */
    WFSetFuncState(WF_PROCESS_EVENT_FUNC, WF_ENTERING_FUNCTION);
      
    switch (event)
    {
        /*--------------------------------------*/
        case WF_EVENT_CONNECTION_SUCCESSFUL:
        /*--------------------------------------*/   
            SYS_CONSOLE_MESSAGE("Event: Connection Successful\r\n"); 
            break;
        
        /*--------------------------------------*/            
        case WF_EVENT_CONNECTION_FAILED:
        case WF_EVENT_CONNECTION_TEMPORARILY_LOST:
        case WF_EVENT_CONNECTION_PERMANENTLY_LOST:            
        /*--------------------------------------*/
            #if defined(SYS_CONSOLE_ENABLE)
            WF_OutputConnectionDebugMsg(event, eventInfo);
            #endif
            break; 
            
        /*--------------------------------------*/    
        case WF_EVENT_CONNECTION_REESTABLISHED:
        /*--------------------------------------*/
            SYS_CONSOLE_MESSAGE("Event: Connection Reestablished\r\n");
            break;
            
        /*--------------------------------------*/    
        case WF_EVENT_SCAN_RESULTS_READY:
        /*--------------------------------------*/  
            SYS_CONSOLE_MESSAGE("Event: Scan Results Ready,");
            sprintf(buf, "%d", eventInfo);
            SYS_CONSOLE_MESSAGE(buf);
            SYS_CONSOLE_MESSAGE("results\r\n");
            #if defined ( EZ_CONFIG_SCAN )
            WFScanEventHandler(eventInfo);
			#endif /* EZ_CONFIG_SCAN */
            break;

        default:
            SYS_ASSERT(false, "Unknown WiFi Event");  /* unknown event */
            break;
    }        
    
    /* Informs the WF driver that we are leaving this function */
    WFSetFuncState(WF_PROCESS_EVENT_FUNC, WF_LEAVING_FUNCTION);
}    
  
 

#endif /* TCPIP_IF_MRF24W */

