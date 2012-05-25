/*******************************************************************************
  MRF24W Driver Initialization

  Summary:
    Module for Microchip TCP/IP Stack
    
  Description:
    -Provides access to MRF24W WiFi controller
    -Reference: MRF24W Data sheet, IEEE 802.11 Standard
*******************************************************************************/

/*******************************************************************************
FileName:  wf_init.c 
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
#include "wf_mac.h"

#if defined(TCPIP_IF_MRF24W)
#include "wf_debug_output.h"


#if defined(APP_USE_IPERF)
#include "iperf_config.h"
#endif

#if defined(WF_CONSOLE)
#include "wifi/wf_console.h"
#endif

bool gRFModuleVer1209orLater = false;




/*
*********************************************************************************************************
*                                           DEFINES                               
*********************************************************************************************************
*/

#define EXPECTED_MRF24W_VERSION_NUMBER      (2)

/* This MAC address is the default MAC address used in tcpip_config.h.  If the */
/* user leaves this MAC address unchanged then the WiFi Driver will get the   */
/* unique MAC address from the MRF24W and have the stack use it.           */
#define MCHP_DEFAULT_MAC_ADDRESS_BYTE_1     (0x00)
#define MCHP_DEFAULT_MAC_ADDRESS_BYTE_2     (0x04)
#define MCHP_DEFAULT_MAC_ADDRESS_BYTE_3     (0xa3)
#define MCHP_DEFAULT_MAC_ADDRESS_BYTE_4     (0x00)
#define MCHP_DEFAULT_MAC_ADDRESS_BYTE_5     (0x00)
#define MCHP_DEFAULT_MAC_ADDRESS_BYTE_6     (0x00)


/*
*********************************************************************************************************
*                                           LOCAL GLOBAL VARIABLES                               
*********************************************************************************************************
*/

/* This MAC address is the default MAC address used in tcpip_config.h.  If the */
/* user leaves this MAC address unchanged then the WiFi Driver will get the  */
/* unique MAC address from the MRF24W and have the stack use it.              */
static const uint8_t MchpDefaultMacAddress[WF_MAC_ADDRESS_LENGTH] = {0x00u, 0x04u, 0xA3u, 0x00u, 0x00u, 0x00u};


/*
*********************************************************************************************************
*                                           LOCAL FUNCTION PROTOTYPES                              
*********************************************************************************************************
*/

static void WF_LibInitialize(NET_CONFIG* pNetIf);

extern void SetAppPowerSaveMode(bool state);


#if defined(CONSOLE)
extern void IperfAppInit(void);
#endif

void SetDhcpProgressState(void);


/*****************************************************************************
 * FUNCTION: WF_Init
 *
 * RETURNS: true if success
 *          false otherwise
 *
 * PARAMS:
 *      N/A.
 *
 *
 *  NOTES: This function must be called once prior to calling any other WF...
 *          functions.  This function initializes the Wifi Driver internal State.
 *          It also verifies functionality of the lower level SPI driver and
 *          connected hardware.
 *****************************************************************************/
bool WF_Init(const void* pNetIf)
{
    tWFDeviceInfo deviceInfo;
    
    if(!WFHardwareInit())
    {
        return false;
    }

    RawInit();
    WFEnableMRF24WMode();
    WF_GetDeviceInfo(&deviceInfo);
    
    // if MRF24W
    if (deviceInfo.deviceType == MRF24WB0M_DEVICE)
    {
        SYS_ASSERT(deviceInfo.romVersion == 0x12, "");
        SYS_ASSERT(deviceInfo.patchVersion >= 0x02, "");

        if (deviceInfo.romVersion == 0x12 && deviceInfo.patchVersion >= 0x09)
        {
            gRFModuleVer1209orLater = true;
        }    
    }
    
    /* send init messages to MRF24W */
    WF_LibInitialize((NET_CONFIG*)pNetIf);
    
  	#if defined(WF_CONSOLE)
    	WFConsoleInit();

        #if defined(APP_USE_IPERF)
           // IperfConsoleInit();
            IperfAppInit("MRF24W");
        #endif
	#endif
	
	
	// save network handle
	SetNetworkConfig((NET_CONFIG *)pNetIf);
    	
    return true;
}


bool isWiFiVer1209OrLater(void)
{
    return gRFModuleVer1209orLater;
}    


extern void WFMgmtMessageTest();
/*****************************************************************************
 * FUNCTION: WF_LibInitialize
 *
 * RETURNS:  None
 *
 * PARAMS:   None
 *
 *  NOTES:   Performs initialization which is specific to the Microchip Demo code.
 *****************************************************************************/
static void WF_LibInitialize(NET_CONFIG* pNetIf)
{
    /* Disable Tx Data confirms (from the MRF24W) */
    WF_SetTxDataConfirm(WF_DISABLED);

    /* if the user has left the default MAC address in tcpip_config.h unchanged then use */
    /* the unique MRF24W MAC address so prevent multiple devices from having the same   */
    /* MAC address.                                                                     */
    if ( memcmp((void *)pNetIf->MyMACAddr.v, (void *)MchpDefaultMacAddress, WF_MAC_ADDRESS_LENGTH) == 0)
    {
        /* get the MRF24W MAC address and overwrite the MAC in pNetIf */
        WF_GetMacAddress((uint8_t *)pNetIf->MyMACAddr.v);
    }
    /* else presume the user has a unique MAC address of their own that they wish to use */    
    else
    {
        // set MAC address with user-supplied MAC */
        WF_SetMacAddress((uint8_t *)pNetIf->MyMACAddr.v);
    }    

    #ifdef WF_CONFIG_DHCP
    WF_SET_DHCP_STATE(DHCP_ENABLED);
    #endif
}



#endif /* TCPIP_IF_MRF24W */

