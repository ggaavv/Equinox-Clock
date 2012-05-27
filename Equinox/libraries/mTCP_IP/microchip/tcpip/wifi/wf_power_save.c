/*******************************************************************************
  MRF24W Driver Power Save functions

  Summary:
    Module for Microchip TCP/IP Stack
    
  Description:
    -Provides access to MRF24W WiFi controller
    -Reference: MRF24W Data sheet, IEEE 802.11 Standard
*******************************************************************************/

/*******************************************************************************
FileName:  wf_power_save.c 
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


/*
*********************************************************************************************************
*                                           DEFINES                               
*********************************************************************************************************
*/


#define REG_ENABLE_LOW_POWER_MASK   ((uint16_t)(0x01))
#define REG_DISABLE_LOW_POWER_MASK  ((uint16_t)(0x00))
/*
*********************************************************************************************************
*                                           LOCAL DATA TYPES                               
*********************************************************************************************************
*/

/* Enumeration of valid values for WFSetPowerSaveMode() */
typedef enum
{
    PS_POLL_ENABLED = 0,    /* power save mode enabled  */
    PS_POLL_DISABLED        /* power save mode disabled */ 
} tWFPsPwrMode;

typedef struct pwrModeRequestStruct
{
    uint8_t mode;
    uint8_t wake;
    uint8_t rcvDtims;
    uint8_t reserved;            /* pad byte */
} tWFPwrModeReq;


/*
*********************************************************************************************************
*                                           LOCAL GLOBAL VARIABLES                              
*********************************************************************************************************
*/

static uint8_t g_powerSaveState = WF_PS_OFF;
static bool    g_psPollActive   = false;     
static bool    g_sleepNeeded    = false;
static bool    g_AppPowerSaveModeEnabled = false;

bool g_rxDtim;
bool g_aggressivePs;

/*
*********************************************************************************************************
*                                           LOCAL FUNCTION PROTOTYPES                          
*********************************************************************************************************
*/

static void SendPowerModeMsg(tWFPwrModeReq *p_powerMode);
static void SetPowerSaveState(uint8_t powerSaveState);


void SetAppPowerSaveMode(bool state);
bool GetAppPowerSaveMode(void);

/*******************************************************************************
  Function:	
    void WFConfigureLowPowerMode(uint8_t action)

  Summary:
    Driver function to configure PS Poll mode.

  Description:
    This function is only used by the driver, not the application.  This
    function, other than at initialization, is only used when the application
    has enabled PS-Poll mode.  This function is used to temporarily deactivate 
    PS-Poll mode when there is mgmt or data message tx/rx and then, when message 
    activity has ceased, to again activate PS-Poll mode. 

  Precondition:
  	MACInit must be called first.

  Parameters:
    action - Can be either:
             * WF_LOW_POWER_MODE_ON
             * WF_LOW_POWER_MODE_OFF

  Returns:
  	None.
  	
  Remarks:
  	None.
  *****************************************************************************/
void WFConfigureLowPowerMode(uint8_t action)
{
    uint16_t lowPowerStatusRegValue;

    /*-----------------------------------------*/
    /* if activating PS-Poll mode on MRF24W */
    /*-----------------------------------------*/
    if (action == WF_LOW_POWER_MODE_ON)
    {
        SYS_CONSOLE_MESSAGE("EPS\r\n");
        Write16BitWFRegister(WF_PSPOLL_H_REG, REG_ENABLE_LOW_POWER_MASK);
        g_psPollActive = true;           
    }        
    /*---------------------------------------------------------------------------------------------*/    
    /* else deactivating PS-Poll mode on MRF24W (taking it out of low-power mode and waking it up) */
    /*---------------------------------------------------------------------------------------------*/
    else /* action == WF_LOW_POWER_MODE_OFF */
    {
        SYS_CONSOLE_MESSAGE("DPS\r\n");
        Write16BitWFRegister(WF_PSPOLL_H_REG, REG_DISABLE_LOW_POWER_MASK);      
        g_psPollActive = false;                 

        /* poll the response bit that indicates when the MRF24W has come out of low power mode */
        do
        {
            /* set the index register to the register we wish to read */
            Write16BitWFRegister(WF_INDEX_ADDR_REG, WF_LOW_POWER_STATUS_REG);
            lowPowerStatusRegValue = Read16BitWFRegister(WF_INDEX_DATA_REG);

        } while (lowPowerStatusRegValue & REG_ENABLE_LOW_POWER_MASK);
    }    
}

/*******************************************************************************
  Function:	
    void WF_PsPollEnable(bool rxDtim,  bool aggressive)

  Summary:
    Enables PS Poll mode.

  Description:
    Enables PS Poll mode.  PS-Poll (Power-Save Poll) is a mode allowing for 
    longer battery life.  The MRF24W coordinates with the Access Point to go 
    to sleep and wake up at periodic intervals to check for data messages, which 
    the Access Point will buffer.  The listenInterval in the Connection 
    Algorithm defines the sleep interval.  By default, PS-Poll mode is disabled.

    When PS Poll is enabled, the WF Host Driver will automatically force the 
    MRF24W to wake up each time the Host sends Tx data or a control message 
    to the MRF24W.  When the Host message transaction is complete the 
    MRF24W driver will automatically re-enable PS Poll mode.

    When the application is likely to experience a high volume of data traffic 
    then PS-Poll mode should be disabled for two reasons:
    1. No power savings will be realized in the presence of heavy data traffic.
    2. Performance will be impacted adversely as the WiFi Host Driver 
        continually activates and deactivates PS-Poll mode via SPI messages.

  Precondition:
  	MACInit must be called first.

  Parameters:
    rxDtim - true if MRF24W should wake up periodically and check for
             buffered broadcast messages, else false

  Returns:
  	None.
  	
  Remarks:
  	None.
  *****************************************************************************/
void WF_PsPollEnable(bool rxDtim, bool aggressive)
{
    tWFPwrModeReq   pwrModeReq;
    
    if (isWiFiVer1209OrLater() && !WFisConnected())
    {
        // save caller parameters for later, when we can enable this mode    
        g_rxDtim = rxDtim;
        g_aggressivePs = aggressive;
        SetAppPowerSaveMode(true);        
        return;
    }    

    /* fill in request structure and send message to MRF24W */
    pwrModeReq.mode     = PS_POLL_ENABLED;
    pwrModeReq.wake     = 0;
    pwrModeReq.rcvDtims = rxDtim;
	pwrModeReq.reserved = aggressive;
    SendPowerModeMsg(&pwrModeReq);
    
    if (rxDtim == true)
    {
        SetPowerSaveState(WF_PS_PS_POLL_DTIM_ENABLED);
    }    
    else
    {
        SetPowerSaveState(WF_PS_PS_POLL_DTIM_DISABLED);
    }    
    
    WFConfigureLowPowerMode(WF_LOW_POWER_MODE_ON);
    SetAppPowerSaveMode(true);
    
}


bool isSleepNeeded(void)
{
    return g_sleepNeeded;    
}       

void ClearSleepNeeded(void)
{
    g_sleepNeeded = false;
}    

void SetSleepNeeded(void)
{
    g_sleepNeeded = true;
}


void SetAppPowerSaveMode(bool state)  // true or false
{
    g_AppPowerSaveModeEnabled = state;
}    

bool GetAppPowerSaveMode(void)
{
    return g_AppPowerSaveModeEnabled;
}    


/*******************************************************************************
  Function:	
    void WF_PsPollDisable(void)

  Summary:
    Disables PS-Poll mode.

  Description:
    Disables PS Poll mode.  The MRF24W will stay active and not go sleep.

  Precondition:
  	MACInit must be called first.

  Parameters:
    None.

  Returns:
  	None.
  	
  Remarks:
  	None.
  *****************************************************************************/
void WF_PsPollDisable(void)
{
    tWFPwrModeReq   pwrModeReq;
    
    pwrModeReq.mode     = PS_POLL_DISABLED;
    pwrModeReq.wake     = 1;
    pwrModeReq.rcvDtims = 1;
    SendPowerModeMsg(&pwrModeReq);

    SetPowerSaveState(WF_PS_OFF);
    WFConfigureLowPowerMode(WF_LOW_POWER_MODE_OFF);    
    SetAppPowerSaveMode(false);
}   

/*******************************************************************************
  Function:	
    void WF_GetPowerSaveState(uint8_t *p_powerSaveState)

  Summary:
    Returns current power-save state.

  Description:
    Returns the current MRF24W power save state.

    <table>
    Value                       Definition
    -----                       ----------
    WF_PS_HIBERNATE             MRF24W in hibernate state
    WF_PS_PS_POLL_DTIM_ENABLED  MRF24W in PS-Poll mode with DTIM enabled
    WF_PS_PS_POLL_DTIM_DISABLED MRF24W in PS-Poll mode with DTIM disabled
    WF_PS_POLL_OFF              MRF24W is not in any power-save state
    </table>

  Precondition:
  	MACInit must be called first.

  Parameters:
    p_powerSaveState - Pointer to where power state is written

  Returns:
  	None.
  	
  Remarks:
  	None.
  *****************************************************************************/
void WF_GetPowerSaveState(uint8_t *p_powerSaveState)
{
    *p_powerSaveState = g_powerSaveState;
}  

/*******************************************************************************
  Function:	
    bool WFisPsPollEnabled(void)

  Summary:
    Determines if application has enable PS-Poll mode.

  Description:

  Precondition:
  	MACInit must be called first.

  Parameters:
    None.

  Returns:
  	true if application has enabled PS-Poll mode, else returns false
  	
  Remarks:
  	None.
  *****************************************************************************/
bool WFisPsPollEnabled(void)
{
    if ((g_powerSaveState == WF_PS_PS_POLL_DTIM_ENABLED) || (g_powerSaveState == WF_PS_PS_POLL_DTIM_DISABLED)) 
    {
        return true;
    }    
    else
    {
        return false;
    }    
}    

/*******************************************************************************
  Function:	
    bool WFIsPsPollActive(void)

  Summary: Determine if PS Poll is currently active.

  Description:
    This function is only called when PS-Poll mode has been enabled by the 
    application.  When transmitting or receiving data or mgmt messages the 
    driver will temporarily disable PS-Poll.  This function is used by the 
    driver to determine if PS-Poll is active or has been temporarily disabled. 

  Precondition:
  	MACInit must be called first.

  Parameters:
    None.

  Returns:
  	true if driver has enabled PS-Poll, else false
  	
  Remarks:
  	None.
  *****************************************************************************/
bool WFIsPsPollActive(void)
{
    return g_psPollActive;
}    

/*******************************************************************************
  Function:	
    void EnsureWFisAwake()

  Summary:
    If PS-Poll is active or the MRF24W is asleep, ensure that it is woken up.

  Description:
    Called by the WiFi driver when it needs to transmit or receive a data or 
    mgmt message. If the application has enabled PS-Poll mode and the WiFi 
    driver has activated PS-Poll mode then this function will deactivate PS-Poll
    mode and wake up the MRF24W.

  Precondition:
  	MACInit must be called first.

  Parameters:
    None.

  Returns:
  	None.
  	
  Remarks:
  	None.
  *****************************************************************************/
void EnsureWFisAwake()
{
    /* if the application desires the MRF24W to be in PS-Poll mode (PS-Poll with DTIM enabled or disabled */
    if ((g_powerSaveState == WF_PS_PS_POLL_DTIM_ENABLED) || (g_powerSaveState == WF_PS_PS_POLL_DTIM_DISABLED)) 
    {
        /* if the WF driver has activated PS-Poll */
        if (g_psPollActive == true)
        {
            /* wake up MRF24W */
            WFConfigureLowPowerMode(WF_LOW_POWER_MODE_OFF);
        }    
            
            // will need to put device back into PS-Poll sleep mode after transaction
            SetSleepNeeded();
    }
}        
            
            
/*******************************************************************************
  Function:	
    void WF_HibernateEnable()

  Summary:
    Puts the MRF24W into hibernate mode.

  Description:
    Enables Hibernate mode on the MRF24W, which effectively turns off the 
    device for maximum power savings.  

    MRF24W state is not maintained when it transitions to hibernate mode.  
    To remove the MRF24W from hibernate mode call WF_Init().

  Precondition:
  	MACInit must be called first.

  Parameters:
    None.

  Returns:
  	None.
  	
  Remarks:
  	Note that because the MRF24W does not save state, there will be a
    disconnect between the TCP/IP stack and the MRF24B0M state.  If it is
    desired by the application to use hibernate, additional measures must be
    taken to save application state.  Then the host should be reset.  This will
    ensure a clean connection between MRF24W and TCP/IP stack

    Future versions of the stack might have the ability to save stack context
    as well, ensuring a clean wake up for the MRF24W without needing a host
    reset.
  *****************************************************************************/
void WF_HibernateEnable()
{
    WF_SetCE_N(WF_HIGH);   /* set XCEN33 pin high, which puts MRF24W in hibernate mode */

    /* SetPowerSaveState(WF_PS_HIBERNATE);	*/
}

/*******************************************************************************
  Function:	
    static void SendPowerModeMsg(tWFPwrModeReq *p_powerMode)

  Summary:
    Send power mode management message to the MRF24W.

  Description:

  Precondition:
  	MACInit must be called first.

  Parameters:
    p_powerMode - Pointer to tWFPwrModeReq structure to send to MRF24W.

  Returns:
  	None.
  	
  Remarks:
  	None.
  *****************************************************************************/
static void SendPowerModeMsg(tWFPwrModeReq *p_powerMode)
{
    uint8_t hdr[2];
    
    hdr[0] = WF_MGMT_REQUEST_TYPE;
    hdr[1] = WF_SET_POWER_MODE_SUBTYPE;

    SendMgmtMsg(hdr,
                sizeof(hdr),
               (uint8_t *)p_powerMode,
               sizeof(tWFPwrModeReq));

    /* wait for mgmt response, free buffer after it comes in (no data to read) */
	WaitForMgmtResponse(WF_SET_POWER_MODE_SUBTYPE, FREE_MGMT_BUFFER);
    
}    

/*******************************************************************************
  Function:	
    static void SetPowerSaveState(uint8_t powerSaveState)

  Summary:
    Sets the desired power save state of the MRF24W.

  Description:

  Precondition:
  	MACInit must be called first.

  Parameters:
    powerSaveState - Value of the power save state desired.

    <table>
    Value                       Definition
    -----                       ----------
    WF_PS_HIBERNATE             MRF24W in hibernate state
    WF_PS_PS_POLL_DTIM_ENABLED  MRF24W in PS-Poll mode with DTIM enabled
    WF_PS_PS_POLL_DTIM_DISABLED MRF24W in PS-Poll mode with DTIM disabled
    WF_PS_POLL_OFF              MRF24W is not in any power-save state
    </table>

  Returns:
  	None.
  	
  Remarks:
  	None.
  *****************************************************************************/
static void SetPowerSaveState(uint8_t powerSaveState)
{
    g_powerSaveState = powerSaveState;
}    
#endif /* TCPIP_IF_MRF24W */
