/*******************************************************************************
  SMSC LAN8720 PHY API for Microchip TCP/IP Stack
*******************************************************************************/

/*******************************************************************************
FileName:   ETHPIC32ExtPhySMSC8720.c
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

#include "tcpip_private.h"

#include <peripheral/ports.h>
#include <peripheral/eth.h>


// Compile only for PIC32MX with Ethernet MAC interface
#if ( ((__PIC32_FEATURE_SET__ >= 500) && (__PIC32_FEATURE_SET__ <= 799) && defined (_ETH)) && defined(TCPIP_IF_PIC32INT) )

#include "tcpip/eth_pic32_ext_phy.h"

#include "tcpip/eth_pic32_ext_phy_smsc8720.h"



/****************************************************************************
 *                 interface functions
 ****************************************************************************/


/****************************************************************************
 * Function:        EthPhyConfigureMII
 *
 * PreCondition:    - Communication to the PHY should have been established.
 *
 * Input:           cFlags - the requested configuration flags: ETH_PHY_CFG_RMII/ETH_PHY_CFG_MII
 *
 * Output:          ETH_RES_OK - success,
 *                  an error code otherwise
 *
 *
 * Side Effects:    None
 *
 * Overview:        This function configures the PHY in one of MII/RMII operation modes.
 *
 * Note:            SMSC 8720 supports RMII  mode only!
 *****************************************************************************/
eEthRes EthPhyConfigureMII(eEthPhyCfgFlags cFlags)
{
	return (cFlags&ETH_PHY_CFG_RMII)?ETH_RES_OK:ETH_RES_CFG_ERR;
}


/****************************************************************************
 * Function:        EthPhyConfigureMdix
 *
 * PreCondition:    - Communication to the PHY should have been established.
 *
 * Input:           oFlags - the requested open flags: ETH_OPEN_MDIX_AUTO, ETH_OPEN_MDIX_NORM/ETH_OPEN_MDIX_SWAP
 *
 * Output:          ETH_RES_OK - success,
 *                  an error code otherwise
 *
 *
 * Side Effects:    None
 *
 * Overview:        This function configures the MDIX mode for the PHY.
 *
 * Note:            None
 *****************************************************************************/
eEthRes EthPhyConfigureMdix(eEthOpenFlags oFlags)
{
	unsigned short	phyReg;

	EthMIIMReadStart(PHY_REG_SPECIAL_CTRL, PHY_ADDRESS);
	phyReg=EthMIIMReadResult()&(_SPECIALCTRL_XPOL_MASK);	// mask off not used bits

	if(oFlags&ETH_OPEN_MDIX_AUTO)
	{	// enable Auto-MDIX
		phyReg&=~_SPECIALCTRL_AMDIXCTRL_MASK;
	}
	else
	{	// no Auto-MDIX
		phyReg|=_SPECIALCTRL_AMDIXCTRL_MASK;	// disable Auto-MDIX
	       if(oFlags&ETH_OPEN_MDIX_SWAP)
	       {
		       phyReg|=_SPECIALCTRL_CH_SELECT_MASK;	// swap
	       }
	       else
	       {
		       phyReg&=~_SPECIALCTRL_CH_SELECT_MASK;	// normal
	       }
	}
	
	EthMIIMWriteStart(PHY_REG_SPECIAL_CTRL, PHY_ADDRESS, phyReg);	

	return ETH_RES_OK;	

}

/****************************************************************************
 * Function:        EthPhyMIIMAddress
 *
 * PreCondition:    None
 *
 * Input:           None
 *
 * Output:          PHY MIIM address
 *
 *
 * Side Effects:    None
 *
 * Overview:        This function returns the address the PHY uses for MIIM transactions
 *
 * Note:            None
 *****************************************************************************/
unsigned int EthPhyMIIMAddress(void)
{
	return PHY_ADDRESS;
}


/****************************************************************************
 * Function:        EthPhyMIIMClock
 *
 * PreCondition:    None
 *
 * Input:           None
 *
 * Output:          PHY MIIM clock, Hz
 *
 *
 * Side Effects:    None
 *
 * Overview:        This function returns the maximum clock frequency that the PHY can use for the MIIM transactions
 *
 * Note:            None
 *****************************************************************************/
unsigned int EthPhyMIIMClock(void)
{
	return 2500000;		//  2.5 MHz max clock supported
}

#endif  // ( ((__PIC32_FEATURE_SET__ >= 500) && (__PIC32_FEATURE_SET__ <= 799) && defined (_ETH)) && defined(TCPIP_IF_PIC32INT) )

