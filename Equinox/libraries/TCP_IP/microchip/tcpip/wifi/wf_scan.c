/*******************************************************************************
  MRF24W Driver Scan functions

  Summary:
    Module for Microchip TCP/IP Stack
    
  Description:
    -Provides access to MRF24W WiFi controller
    -Reference: MRF24W Data sheet, IEEE 802.11 Standard
*******************************************************************************/

/*******************************************************************************
FileName:  wf_scan.c 
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


/*
*********************************************************************************************************
*                                           INCLUDES                               
*********************************************************************************************************
*/


/*******************************************************************************
  Function:	
    void WF_Scan(uint8_t CpId)

  Summary:
    Commands the MRF24W to start a scan operation.  This will generate the 
    WF_EVENT_SCAN_RESULTS_READY event.

  Description:
    Directs the MRF24W to initiate a scan operation utilizing the input 
    Connection Profile ID.  The Host Application will be notified that the scan 
    results are ready when it receives the WF_EVENT_SCAN_RESULTS_READY event.  
    The eventInfo field for this event will contain the number of scan results.  
    Once the scan results are ready they can be retrieved with 
    WF_ScanGetResult().

    Scan results are retained on the MRF24W until:
    1.	Calling WF_Scan() again (after scan results returned from previous 
         call).
    2.	MRF24W reset.

  Precondition:
  	MACInit must be called first.

  Parameters:
    CpId - Connection Profile to use.  
            If the CpId is valid then the values from that Connection Profile 
            will be used for filtering scan results.  If the CpId is set to 
            WF_SCAN_ALL (0xFF) then a default filter will be used.

            Valid CpId
            * If CP has a defined SSID only scan results with that SSID are 
               retained.  
            * If CP does not have a defined SSID then all scanned SSID’s will be 
               retained
            * Only scan results from Infrastructure or AdHoc networks are 
               retained, depending on the value of networkType in the Connection Profile
            * The channel list that is scanned will be determined from 
               channelList in the Connection Algorithm (which must be defined 
               before calling this function).

            CpId is equal to WF_SCAN_ALL
            * All scan results are retained (both Infrastructure and Ad Hoc 
               networks).
            * All channels within the MRF24W’s regional domain will be 
               scanned.
            * No Connection Profiles need to be defined before calling this 
               function.
            * The Connection Algorithm does not need to be defined before 
               calling this function.

  Returns:
  	None.
  	
  Remarks:
  	None.
  *****************************************************************************/
void WF_Scan(uint8_t CpId)
{
    uint8_t   hdr[4];
    
    hdr[0] = WF_MGMT_REQUEST_TYPE;
    hdr[1] = WF_SCAN_START_SUBTYPE; 
    hdr[2] = CpId;                  /* Connection Profile ID */
    hdr[3] = 0;                     /* not used              */
    
    SendMgmtMsg(hdr,             /* header           */
                sizeof(hdr),     /* size of header   */
                NULL,            /* no data          */
                0);              /* no data          */

    /* wait for mgmt response, free it after it comes in (no data needed) */
	WaitForMgmtResponse(WF_SCAN_START_SUBTYPE, FREE_MGMT_BUFFER); 
}

/*******************************************************************************
  Function:	
    void WF_ScanGetResult(uint8_t listIndex, tWFScanResult  *p_scanResult)

  Summary:
    Read scan results back from MRF24W.

  Description:
    After a scan has completed this function is used to read one or more of the 
    scan results from the MRF24W.  The scan results will be written 
    contiguously starting at p_scanResults (see tWFScanResult structure for 
    format of scan result).    

  Precondition:
  	MACInit must be called first.  WF_EVENT_SCAN_RESULTS_READY event must have
    already occurrerd.

  Parameters:
    listIndex - Index (0-based list) of the scan entry to retrieve.
    p_scanResult - Pointer to location to store the scan result structure

  Retrieve RSSI: RSSI_MAX (200) , RSSI_MIN (106)
    p_scanResult->rssi

  Returns:
  	None.
  	
  Remarks:
  	None.
  *****************************************************************************/
void WF_ScanGetResult(uint8_t          listIndex, 
                       tWFScanResult  *p_scanResult)
{
    
    uint8_t   hdr[4];
	/* char rssiChan[48]; */ /* reference for how to retrieve RSSI */
    
    hdr[0] = WF_MGMT_REQUEST_TYPE;
    hdr[1] = WF_SCAN_GET_RESULTS_SUBTYPE; 
    hdr[2] = listIndex;        /* scan result index to read from */
    hdr[3] = 1;                /* number of results to read            */
    
    SendMgmtMsg(hdr,             /* header           */
                sizeof(hdr),     /* size of header   */
                NULL,            /* no data          */
                0);              /* no data          */

    /* index 4 contains number of scan results returned, index 5 is first byte of first scan result */
    WaitForMgmtResponseAndReadData(WF_SCAN_GET_RESULTS_SUBTYPE,         /* expected subtype               */ 
                                   sizeof(tWFScanResult),               /* num data bytes to read         */
                                   5,                                   /* starting at this index         */
                                  (uint8_t *)p_scanResult);               /* write the response data here   */
                                  

    /* fix up endianness on the two 16-bit values in the scan results */
    p_scanResult->beaconPeriod = WFSTOHS(p_scanResult->beaconPeriod);
    p_scanResult->atimWindow   = WFSTOHS(p_scanResult->atimWindow);

	/* reference for how to retrieve RSSI */
	/* Display SSID  & Channel */ 
	/* sprintf(rssiChan, "  => RSSI: %u, Channel: %u\r\n",  p_scanResult->rssi, p_scanResult->channel);  */
    /* SYS_CONSOLE_MESSAGE(rssiChan); */
}                        


#endif /* TCPIP_IF_MRF24W */
