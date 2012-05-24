/*******************************************************************************
  MRF24W Driver Management Messages

  Summary:
    Module for Microchip TCP/IP Stack
    
  Description:
    -Provides access to MRF24W WiFi controller
    -Reference: MRF24W Data sheet, IEEE 802.11 Standard
*******************************************************************************/

/*******************************************************************************
FileName:  wf_mgmt_msg.c 
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

/*
*********************************************************************************************************
*                                           INCLUDES                               
*********************************************************************************************************
*/

#include "tcpip_private.h"


#if defined(TCPIP_IF_MRF24W)
#include "wf_mac.h"
#include "mrf24w_mac.h"
#include "wf_debug_output.h"

/*
*********************************************************************************************************
*                                           DEFINES                               
*********************************************************************************************************
*/

#if defined(SYS_DEBUG_ENABLE)
    /* Macro used to determine is application is trying to send a management when in the driver function  */
    /* WF_ProcessEvent().                                                                                 */
    #define isInWFProcessEvent()    ((g_FuncFlags & WF_PROCESS_EVENT_FUNC) > 0)
#endif

/*
*********************************************************************************************************
*                                           LOCAL GLOBAL VARIABLES                               
*********************************************************************************************************
*/

static volatile bool gMgmtConfirmMsgReceived  = false;
static bool RestoreRxData = false;
static bool g_IgnoreNextMgmtResult = false;

uint8_t g_WaitingForMgmtResponse = false;

#if defined(SYS_DEBUG_ENABLE)
   static uint8_t g_FuncFlags = 0x00;  
#endif


/*****************************************************************************
 * FUNCTION: SendMgmtMsg
 *
 * RETURNS:  error code
 *
 * PARAMS:   p_header      -- pointer to mgmt message header data
 *           headerLength  -- number of bytes in the header
 *                              will be written
 *           p_data        -- pointer to mgmt message data
 *           dataLength    -- number of byte of data
 *
 *  NOTES:   Sends a management message

 *****************************************************************************/
void SendMgmtMsg(uint8_t *p_header,
                 uint8_t headerLength,
                 uint8_t *p_data,
                 uint8_t dataLength)
{
    SYS_TICK  startTickCount;
    SYS_TICK  maxAllowedTicks;
    NET_CONFIG* pConfig;

    /* cannot send management messages while in WF_ProcessEvent() */
    SYS_ASSERT(!isInWFProcessEvent(), "");
    
    EnsureWFisAwake();
    
    /* if a Rx Data packet is mounted that has not yet been processed */
    if (GetRawWindowState(RAW_RX_ID) == WF_RAW_DATA_MOUNTED)
    {
        /* save it, so after mgmt response received it can be restored */
        PushRawWindow(RAW_RX_ID);
        RestoreRxData = true;     
    }    


    pConfig = _TCPIPStackMacIdToNet(TCPIP_MAC_ID_MRF24W);
    /* mounts a tx mgmt buffer on the MRF24W when data tx is done */
    maxAllowedTicks = SYS_TICK_TicksPerSecondGet() / 200;  /* 5 ms timeout */
    if(maxAllowedTicks == 0)
    {
        maxAllowedTicks = 1;    // at least on tick wait
    }
    startTickCount = SYS_TICK_Get();
    while (!WFisTxMgmtReady() )
    {
        MRF24W_MACProcess(_TCPIPStackNetToMac(pConfig));
        
        /* DEBUG -- REMOVE AFTER FIGURE OUT WHY TIMING OUT (RARELY HAPPENS) */
        if (SYS_TICK_Get() - startTickCount >= maxAllowedTicks)
        {
            /* force flags so WFisTxMgmtReady will return true */
            SetRawWindowState(RAW_TX_ID, WF_RAW_UNMOUNTED);
            RawWindowReady[RAW_TX_ID] = false;
        }    
    }    

    /* write out management header */
    RawSetByte(RAW_TX_ID, p_header, headerLength); 
    
    /* write out data (if any) */
    if (dataLength > 0)
    {
        RawSetByte(RAW_TX_ID, p_data, dataLength);         
    }  

    /* send mgmt msg to MRF24W */
    SendRAWManagementFrame(headerLength + dataLength);
}                           

/*****************************************************************************
 * FUNCTION: SignalMgmtConfirmReceivedEvent
 *
 * RETURNS:  None
 *
 * PARAMS:   None
 *
 *  NOTES:   Called by ProcessMgmtRxMsg when a mgmt confirm has been received.  
 *           This function then sets a local flag for this module indicating 
 *           the event.  
 *****************************************************************************/
void SignalMgmtConfirmReceivedEvent(void)
{
    gMgmtConfirmMsgReceived = true;        
}    


void IgnoreNextMgmtResult()
{
    g_IgnoreNextMgmtResult = true;
}    

/*****************************************************************************
 * FUNCTION: WaitForMgmtResponse
 *
 * RETURNS:  None
 *
 * PARAMS:   expectedSubtype -- The expected subtype of the mgmt response
 *           freeAction      -- FREE_MGMT_BUFFER or DO_NOT_FREE_MGMT_BUFFER
 *
 *  NOTES:   Called after sending a mgmt request.  This function waits for a mgmt
 *           response.  The caller can optionally request the the management 
 *           response be freed immediately (by this function) or not freed.  If not
 *           freed the caller is responsible to free the response buffer.
 *****************************************************************************/
void WaitForMgmtResponse(uint8_t expectedSubtype, uint8_t freeAction)
{
    tMgmtMsgRxHdr  hdr;
    
    g_WaitingForMgmtResponse = true;
        
    /* Wait until mgmt response is received */
    while (gMgmtConfirmMsgReceived == false)
    {
        WFProcess();
        
        /* if received a data packet while waiting for mgmt packet */
        if (g_HostRAWDataPacketReceived)
        {
            // We can't let the StackTask processs data messages that come in while waiting for mgmt 
            // response because the application might send another mgmt message, which is illegal until the response
            // is received for the first mgmt msg.  And, we can't prevent the race condition where a data message 
            // comes in before a mgmt response is received.  Thus, the only solution is to throw away a data message
            // that comes in while waiting for a mgmt response.  This should happen very infrequently.  If using TCP then the 
            // stack takes care of retries.  If using UDP, the application has to deal with occasional data messages not being
            // received.  Also, applications typically do not send a lot of management messages after connected.

            // throw away the data rx 
            RawMountRxBuffer();
            DeallocateDataRxBuffer();
            g_HostRAWDataPacketReceived = false;

            /* ensure interrupts enabled */
            WF_EintEnable();
        }    
    }    
 
    /* set this back to false so the next mgmt send won't think he has a response before one is received */
    gMgmtConfirmMsgReceived = false;
    
    
    /* if the caller wants to delete the response immediately (doesn't need any data from it */
    if (freeAction == FREE_MGMT_BUFFER)
    {
        /* read and verify result before freeing up buffer to ensure our message send was successful */
        RawRead(RAW_RX_ID, 0, (uint16_t)(sizeof(tMgmtMsgRxHdr)), (uint8_t *)&hdr);
               
        /* Mgmt response 'result' field should always indicate success.  If this assert is hit the error codes are located */
        /* wf_api.h.  Search for WF_SUCCESS for the list of error codes.                                                    */
        if (g_IgnoreNextMgmtResult)
        {    
            g_IgnoreNextMgmtResult = false;
        }    
        else
        {
            SYS_ASSERT(hdr.result == WF_SUCCESS, "");
        }    

        /* mgmt response subtype had better match subtype we were expecting */
        SYS_ASSERT(hdr.subtype == expectedSubtype, "");

        /* free mgmt buffer */
        DeallocateMgmtRxBuffer();  
        
        /* if there was a mounted data packet prior to the mgmt tx/rx transaction, then restore it */
        if (RestoreRxData == true)
        {
            RestoreRxData = false;
            PopRawWindow(RAW_RX_ID);
            SetRawWindowState(RAW_RX_ID, WF_RAW_DATA_MOUNTED); 
        }          
    }   
}  


/*****************************************************************************
 * FUNCTION: WaitForMgmtRespAndReadData
 *
 * RETURNS:  None
 *
 * PARAMS:   expectedSubtype -- management message subtype that we are expecting
 *           p_data          -- pointer where any desired management data bytes 
 *                              will be written
 
 *           numDataBytes    -- Number of data bytes from mgmt response to write to
 *                              p_data.  Data always starts at index 4 of mgmt response.
 *           skipDataRead    -- if true, then no data will be read and the mgmt buffer will not
 *                              be freed.  If false, the data will be read and the mgmt buffer
 *                              will be freed.
 *
 *  NOTES:   Waits for the mgmt response message and validates it by:
 *             1) checking the result field
 *             2) verifying that the received subtype matches the execpted subtype
 *
 *            In addition, this function reads the desired number of data bytes from 
 *            the mgmt response, copies them to p_data, and then frees the mgmt buffer. 
 *****************************************************************************/
void WaitForMgmtResponseAndReadData(uint8_t expectedSubtype, 
                                    uint8_t numDataBytes,  
                                    uint8_t startIndex, 
                                    uint8_t *p_data)

{
    tMgmtMsgRxHdr  hdr;  /* management msg header struct */
    
    WaitForMgmtResponse(expectedSubtype, DO_NOT_FREE_MGMT_BUFFER);
        
    /* if made it here then received a management message */
    RawRead(RAW_RX_ID, 0, (uint16_t)(sizeof(tMgmtMsgRxHdr)), (uint8_t *)&hdr);

    /* check header result field */
    if (hdr.result != WF_SUCCESS)
    {
        SYS_ASSERT(false, "");
    }    

    /* make sure mgmt response is the expected one */
    if (hdr.subtype != expectedSubtype)
    {
        SYS_ASSERT(false, "");
    }   

    /* if caller wants to read data from this mgmt response */
    if (numDataBytes > 0) 
    {
        RawRead(RAW_RX_ID, startIndex, numDataBytes, p_data);  
    }    
    
    /* free the mgmt buffer */    
    DeallocateMgmtRxBuffer();
    
     /* if there was a mounted data packet prior to the mgmt tx/rx transaction, then restore it */    
    if (RestoreRxData == true)
    {
        RestoreRxData = false;
        PopRawWindow(RAW_RX_ID);
        SetRawWindowState(RAW_RX_ID, WF_RAW_DATA_MOUNTED); 
    }          
}


#if defined(SYS_DEBUG_ENABLE)
/*****************************************************************************
 * FUNCTION: WFSetFuncState
 *
 * RETURNS:  None
 *
 * PARAMS:   funcMask -- bit mask indicating the calling function
 *           state    -- WF_ENTERING_FUNCTION or WF_LEAVING_FUNCTION
 *
 *  NOTES:   Called by WF_ProcessEvent() to be able to detect if there is an attempt 
 *           to send a management message while processing the event (not allowed).
 *****************************************************************************/
void WFSetFuncState(uint8_t funcMask, uint8_t state)
{
    if (state == WF_ENTERING_FUNCTION)
    {
        g_FuncFlags |= funcMask;
    }    
    else
    {
        g_FuncFlags &= ~funcMask;
    }        
    
}    
#endif /* SYS_DEBUG_ENABLE */


#endif /* TCPIP_IF_MRF24W */
