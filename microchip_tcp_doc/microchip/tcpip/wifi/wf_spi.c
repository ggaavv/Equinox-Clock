/*******************************************************************************
  MRF24W Driver SPI interface routines

  Summary:
    SUMMARY
    
  Description:
    Module for Microchip TCP/IP Stack
*******************************************************************************/

/*******************************************************************************
FileName:  wf_spi.c 
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
#include "system/drivers/drv_spi.h"
#include "wf_spi.h"
#include "system/system_services.h"

#if defined(TCPIP_IF_MRF24W)

#include "wf_debug_output.h"

/*
*********************************************************************************************************
*                                           DEFINES                               
*********************************************************************************************************
*/

//============================================================================
//                          SPI Definitions
//============================================================================

#if defined(__C30__)
    #define ClearSPIDoneFlag()
    static inline __attribute__((__always_inline__)) void WaitForDataByte( void )
    {
        while ((WF_SPISTATbits.SPITBF == 1) || (WF_SPISTATbits.SPIRBF == 0));
    }

    #define SPI_ON_BIT          (WF_SPISTATbits.SPIEN)
#endif

/*
*********************************************************************************************************
*                                           LOCAL FUNCTION PROTOTYPES                               
*********************************************************************************************************
*/

static bool ConfigureSpiMRF24W(void);


/*****************************************************************************
  Function:
	bool WF_SpiInit(void)

  Summary:
	Initializes the SPI interface to the MRF24W device.

  Description:
	Configures the SPI interface for communications with the MRF24W.

  Precondition:
	None

  Parameters:
	None

  Returns:
  	None
  	
  Remarks:
	This function is called by WFHardwareInit.
*****************************************************************************/
bool WF_SpiInit(void)
{
    bool res;
    
    /* disable the spi interrupt if necessary */
    // No need because SPI interrupt is not used

    // Initialize IO for WF chip select    
	WF_CS_Init();     
	WF_SpiDisableChipSelect();  // Disable chip select before initialization

    res = ConfigureSpiMRF24W();  

    /* clear the completion flag */
	#if !defined( __PIC32MX__ )
    ClearSPIDoneFlag();
	#endif

    return res;
}


/*
  PIC32 SPI clock speed:
  ---------------------
    Fsck =        Fpb
           ------------------
           2 * (SPIxBRG + 1)
           
Note that the maximum possible baud rate is
Fpb/2 (SPIXBRG = 0) and the minimum possible baud
rate is Fpb /1024.           
*/


/*****************************************************************************
  Function:
	void ConfigureSpiMRF24W(void)

  Summary:
	Configures the SPI interface to the MRF24W.

  Description:
	Configures the SPI interface for communications with the MRF24W.

  Precondition:
	None

  Parameters:
	None

  Returns:
  	true if success
    false otherwise
  	
  Remarks:
	1) If the SPI bus is shared with other peripherals this function is called
	   each time an SPI transaction occurs by WF_SpiEnableChipSelect.  Otherwise it 
	   is called once during initialization by WF_SpiInit. 
	   
	2) Maximum SPI clock rate for the MRF24W is 25MHz.
*****************************************************************************/
static bool ConfigureSpiMRF24W(void)
{
    /*----------------------------------------------------------------*/
    /* After we save context, configure SPI for MRF24W communications */
    /*----------------------------------------------------------------*/
    /* enable the SPI clocks            */
    /* set as master                    */
    /* clock idles high                 */
    /* ms bit first                     */
    /* 8 bit tranfer length             */
    /* data changes on falling edge     */
    /* data is sampled on rising edge   */
    /* set the clock divider            */
	#if defined(__PIC32MX__)
	return DRV_SPI_Initialize(MRF24W_SPI_CHN, 
				SPI_OPEN_MSTEN|SPI_OPEN_MODE8|SPI_OPEN_SMP_END|SPI_OPEN_CKP_HIGH, 
				SYS_CLK_PeripheralClockGet()/WF_MAX_SPI_FREQ);
	#elif defined(__C30__)
	return DRV_SPI_Initialize(MRF24W_SPI_CHN, 
				SPI_SMP_END|SPI_CLK_POL|SPI_MSTEN, 
				SYS_CLK_PeripheralClockGet()/WF_MAX_SPI_FREQ);
	#else
	return false;
	#endif
}    

/*****************************************************************************
  Function:
	void WF_SpiEnableChipSelect(void)

  Summary:
	Enables the MRF24W SPI chip select.

  Description:
	Enables the MRF24W SPI chip select as part of the sequence of SPI 
	communications.

  Precondition:
	None

  Parameters:
	None

  Returns:
  	None
  	
  Remarks:
	If the SPI bus is shared with other peripherals then the current SPI context
	is saved.
*****************************************************************************/
void WF_SpiEnableChipSelect(void)
{
    /* set Slave Select low (enable SPI chip select on MRF24W) */
    WF_CS_Assert(); 
}    

/*****************************************************************************
  Function:
	void WF_SpiDisableChipSelect(void)

  Summary:
	Disables the MRF24W SPI chip select.

  Description:
	Disables the MRF24W SPI chip select as part of the sequence of SPI 
	communications.

  Precondition:
	None

  Parameters:
	None

  Returns:
  	None
  	
  Remarks:
	If the SPI bus is shared with other peripherals then the current SPI context
	is restored.
*****************************************************************************/
void WF_SpiDisableChipSelect(void)
{
    /* set Slave Select high ((disable SPI chip select on MRF24W)   */
    WF_CS_Deassert();
}    

/*****************************************************************************
  Function:
	void WFSpiTxRx(void)

  Summary:
	Transmits and receives SPI bytes

  Description:
	Transmits and receives N bytes of SPI data.

  Precondition:
	None

  Parameters:
	p_txBuf - pointer to SPI tx data
	txLen   - number of bytes to Tx
	p_rxBuf - pointer to where SPI rx data will be stored
	rxLen   - number of SPI rx bytes caller wants copied to p_rxBuf

  Returns:
  	None
  	
  Remarks:
	Will clock out the larger of txLen or rxLen, and pad if necessary.
*****************************************************************************/
void WFSpiTxRx(uint8_t   *p_txBuf, 
               uint16_t  txLen, 
               uint8_t   *p_rxBuf,
               uint16_t  rxLen)
{
#if defined(SYS_DEBUG_ENABLE)
    /* Cannot communicate with MRF24W when it is in hibernate mode */
    {
        static uint8_t state;  /* avoid local vars in functions called from interrupt */
        WF_GetPowerSaveState(&state);
        SYS_ASSERT(state != WF_PS_HIBERNATE, "");
    }    
#endif 

    DRV_SPI_TxRx(MRF24W_SPI_CHN, p_txBuf, txLen, p_rxBuf, rxLen);
}                      

#else
// dummy func to keep compiler happy when module has no executeable code
void MCHP_Spi_EmptyFunc(void)
{
}
#endif /* TCPIP_IF_MRF24W */

