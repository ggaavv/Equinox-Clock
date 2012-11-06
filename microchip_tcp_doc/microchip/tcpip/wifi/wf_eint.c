/*******************************************************************************
  MRF24W Driver External Interrupt

  Summary:
    Module for Microchip TCP/IP Stack
    
  Description:
    -Provides access to MRF24W WiFi controller
    -Reference: MRF24W Data sheet, IEEE 802.11 Standard
*******************************************************************************/

/*******************************************************************************
FileName:  wf_eint.c 
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


/******************************************************************************

 MRF24W Driver Exernal Interrupt
 Module for Microchip TCP/IP Stack
  -Provides access to MRF24W WiFi controller
  -Reference: MRF24W Data sheet, IEEE 802.11 Standard

*******************************************************************************
 FileName:		WF_Eint.c
 Dependencies:	TCP/IP Stack header files
 Processor:		PIC24F, PIC24H, dsPIC30F, dsPIC33F, PIC32
 Compiler:		Microchip C32 v1.10b or higher
				Microchip C30 v3.22 or higher
 Company:		Microchip Technology, Inc.

 Software License Agreement

 Copyright (C) 2002-2010 Microchip Technology Inc.  All rights reserved.

 Microchip licenses to you the right to use, modify, copy, and distribute:
 (i)  the Software when embedded on a Microchip microcontroller or digital 
      signal controller product ("Device") which is integrated into 
      Licensee's product; or
 (ii) ONLY the Software driver source files ENC28J60.c, ENC28J60.h,
      ENCX24J600.c and ENCX24J600.h ported to a non-Microchip device used in 
	  conjunction with a Microchip ethernet controller for the sole purpose 
	  of interfacing with the ethernet controller.

 You should refer to the license agreement accompanying this Software for 
 additional information regarding your rights and obligations.

 THE SOFTWARE AND DOCUMENTATION ARE PROVIDED "AS IS" WITHOUT WARRANTY OF ANY
 KIND, EITHER EXPRESS OR IMPLIED, INCLUDING WITHOUT LIMITATION, ANY WARRANTY
 OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE, TITLE AND
 NON-INFRINGEMENT. IN NO EVENT SHALL MICROCHIP BE LIABLE FOR ANY INCIDENTAL,
 SPECIAL, INDIRECT OR CONSEQUENTIAL DAMAGES, LOST PROFITS OR LOST DATA, COST
 OF PROCUREMENT OF SUBSTITUTE GOODS, TECHNOLOGY OR SERVICES, ANY CLAIMS BY
 THIRD PARTIES (INCLUDING BUT NOT LIMITED TO ANY DEFENSE THEREOF), ANY CLAIMS
 FOR INDEMNITY OR CONTRIBUTION, OR OTHER SIMILAR COSTS, WHETHER ASSERTED ON
 THE BASIS OF CONTRACT, TORT (INCLUDING NEGLIGENCE), BREACH OF WARRANTY, OR
 OTHERWISE.


 Author				Date		Comment
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
 Michael Palladino	10/13/07	Original
 KO					31 Oct 2008	Port to PIC24F and PIC32 for TCP/IP stack v4.52
 KH					19 Jun 2009	Modified MACMemCopyAsync to support TCB to TCB copy
 KH                 27 Jan 2010 Updated for MRF24W
******************************************************************************/


/*
*********************************************************************************************************
*                                           INCLUDES                               
*********************************************************************************************************
*/
#include "tcpip_private.h"

#include "wf_mac.h"
#include "system/drivers/drv_spi.h"
#include "wf_eint.h"

#if defined(TCPIP_IF_MRF24W)


/*****************************************************************************
 * FUNCTION:WF_EintIsDisabled
 *
 * RETURNS: true if MRF24W External Interrupt is disabled, else returns false
 *
 * PARAMS:  None
 *
 * NOTES:   Called by WiFi Driver to determine if the MRF24W External Interrupt 
 *          is disabled.  
 *****************************************************************************/
bool WF_EintIsDisabled(void)
{
    return(WF_INT_IE == 0);  /* works for PIC24, and PIC32 */
}

bool WF_EintIsPending(void)
{
    return(((WF_INT_IO == 0) && (WF_INT_IF == 0)));  /* works for PIC24, and PIC32 */
}

/*====================================================================================================================*/
/*====================================================================================================================*/
/*                                  PIC24 Interrupt Routines                                                          */
/*====================================================================================================================*/
/*====================================================================================================================*/
#if defined( __C30__ )

/*****************************************************************************
 * PIC24 INTERRUPT SERVICE ROUTINE
 *****************************************************************************/
#if  !defined(TCPIP_STACK_USE_EVENT_NOTIFICATION)
void WF_NoEventISR(void* p)
	{
	    // clear EINT
	    if (WF_INT_IF && WF_INT_IE)
	    {
	        WF_INT_IF = 0;
	        WF_INT_IE = 0;         /* disable external interrupt */
	        // invoke handler
	        WFEintHandler();
	    }
	}
#endif  // !defined(TCPIP_STACK_USE_EVENT_NOTIFICATION)
/*****************************************************************************
 * FUNCTION: WF_EintEnable (Specific to PIC24)
 *
 * RETURNS: None
 *
 * PARAMS:  None
 *
 * NOTES:   Called by WiFi Driver to enable the MRF24W External Interrupt.  
 *****************************************************************************/
void WF_EintEnable(void)
{
    // if interrupt line is low, then we may have missed a falling edge
    // while the interrupt was disabled.
    if ( WF_INT_IO == 0 )
    {
        // if the interrupt pin is active
        // then the MRF24W has another event that needs to be serviced.
        // This means that the MRF24W will never generate another falling edge
        // required to trigger the interrupt... So, we must force an interrupt.
        // force the EINT
        WF_INT_IF = 1;
    }

    /* enable the external interrupt */
    WF_INT_IE = 1;
}


/*****************************************************************************
 * FUNCTION: WF_EintDisable (Specific to PIC24)
 *
 * RETURNS: None
 *
 * PARAMS:  None
 *
 * NOTES:   Called by WiFi Driver to disable the MRF24W External Interrupt.  
 *****************************************************************************/
void WF_EintDisable(void)
{
    /* disable the external interrupt */
    WF_INT_IE = 0;
}

/*****************************************************************************
 * FUNCTION: WF_EintInit (Specific to PIC24)
 *
 * RETURNS: None
 *
 * PARAMS:  None
 *
 * NOTES:   Called by WiFi Driver to initialize the MRF24W External Interrupt.  
 *****************************************************************************/
void WF_EintInit(void)
{
    /* disable the external interrupt */
    WF_INT_IE = 0;
    
#if !defined(TCPIP_STACK_USE_EVENT_NOTIFICATION)
	#if defined(MRF24W_IN_SPI2 ) || defined( MRF24W_IN_SPI4 )
        SYS_INT_DynamicRegister(PLIB_INT_SOURCE_EXTERNAL_3, WF_NoEventISR, 0);
	#else
        SYS_INT_DynamicRegister(PLIB_INT_SOURCE_EXTERNAL_1, WF_NoEventISR, 0);
	#endif
    // #else the MRF ISR is already hooked by MRF24W_MACEventInit()!
#endif  // !defined(TCPIP_STACK_USE_EVENT_NOTIFICATION)

    /* configure IO pin as input and External Interrupt pin*/
    /* set the I/O high since we do not have pull-ups */
    WF_INT_IO   = 1;
    WF_INT_TRIS = 1;
    WF_INT_EDGE = 1;   /* falling edge triggered */

	IPC13bits.INT3IP = 7;

    /* clear and enable the interrupt */
    WF_INT_IF = 0;
    // WF_INT_IE = 1;   // Should not enable interrupt here
}

/*====================================================================================================================*/
/*====================================================================================================================*/
/*                                  PIC32 Interrupt Routines                                                          */
/*====================================================================================================================*/
/*====================================================================================================================*/
#elif defined( __PIC32MX__ )

/*****************************************************************************
 * PIC32 INTERRUPT SERVICE ROUTINE
 *****************************************************************************/
// This is used only when no event notification needed.
// Otherwise the MRF24W_MACEventInit() hooks the ISR
//
#if !defined(TCPIP_STACK_USE_EVENT_NOTIFICATION)
void WF_NoEventISR(void* p)
{
    // clear EINT
    if (WF_INT_IF && WF_INT_IE)
    {
        WF_INT_IF_CLEAR = WF_INT_BIT;
        WF_INT_IE_CLEAR = WF_INT_BIT;         /* disable external interrupt */
        
        /* invoke handler */
        WFEintHandler();
    }
}
#endif  // defined(TCPIP_STACK_USE_EVENT_NOTIFICATION)

/*****************************************************************************
 * FUNCTION:WF_EintEnable (Specific to PIC32)
 *
 * RETURNS: None
 *
 * PARAMS:  None
 *
 * NOTES:   Called by WiFi Driver to enable the MRF24W External Interrupt.  
 *****************************************************************************/
void WF_EintEnable(void)
{
    // if interrupt line is low, then we may have missed a falling edge
    // while the interrupt was disabled.
    if ( WF_INT_IO == 0 )
    {
        // if the interrupt pin is active
        // then the MRF24W has another event that needs to be serviced.
        // This means that the MRF24W will never generate another falling edge
        // required to trigger the interrupt... So, we must force an interrupt.
        // force the EINT
        WF_INT_IF_SET = WF_INT_BIT;
    }

    /* enable the external interrupt */
    WF_INT_IE_SET = WF_INT_BIT;
}


/*****************************************************************************
 * FUNCTION:WF_EintDisable (Specific to PIC32)
 *
 * RETURNS: None
 *
 * PARAMS:  None
 *
 * NOTES:   Called by WiFi Driver to disable the MRF24W External Interrupt.  
 *****************************************************************************/
void WF_EintDisable(void)
{
    /* disable the external interrupt */
    WF_INT_IE_CLEAR = WF_INT_BIT;
}

/*****************************************************************************
 * FUNCTION:WF_EintInit (Specific to PIC32)
 *
 * RETURNS: None
 *
 * PARAMS:  None
 *
 * NOTES:   Called by WiFi Driver to initialize the MRF24W External Interrupt.  
 *****************************************************************************/
//
extern void MRF24W_ISR(void* p);
void WF_EintInit(void)
{
    /* disable the external interrupt */
    WF_INT_IE_CLEAR = WF_INT_BIT;

	#if !defined(TCPIP_STACK_USE_EVENT_NOTIFICATION)
	SYS_INT_DynamicRegister(MRFWB0M_INT_SOURCE, WF_NoEventISR, 0);
    // #else the MRF ISR is already hooked by MRF24W_MACEventInit()!
	#endif
    /* configure IO pin as input and External Interrupt pin*/
    /* set the I/O high since we do not have pull-ups */
	#if defined(MRF24W_USE_CN_INT)
    WF_INT_TRIS = 1;
    WF_INT_CNPUE_SET = WF_INT_BIT_MASK;
    WF_INT_CN_ENABLE = 1;
    WF_INT_CNEN_SET = WF_INT_BIT_MASK; 
	#elif defined(MRF24W_USE_INT3_INT) || defined(MRF24W_USE_INT1_INT)
    WF_INT_IO   = 1;
    WF_INT_TRIS = 1;
    WF_INT_EDGE = 0;   /* falling edge triggered */   
    #endif
 
    /* clear and enable the interrupt */
    WF_INT_IF_CLEAR    = WF_INT_BIT;
    WF_INT_IPCCLR      = WF_INT_IPC_MASK;
    WF_INT_IPCSET      = WF_INT_IPC_VALUE;
    //WF_INT_IE_SET      = WF_INT_BIT; // Should not enable interrupt here
}
#endif 

#else
    // dummy func to keep compiler happy when module has no executeable code
    void MCHP_Eint_EmptyFunc(void)
    {
    }

#endif /* TCPIP_IF_MRF24W */
