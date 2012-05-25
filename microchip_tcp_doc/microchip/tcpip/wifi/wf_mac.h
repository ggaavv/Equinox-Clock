/*******************************************************************************
  MRF24W Driver

  Summary:
    Module for Microchip TCP/IP Stack
    
  Description:
    -Provides access to MRF24W WiFi controller
    -Reference: MRF24W Data sheet, IEEE 802.11 Standard
*******************************************************************************/

/*******************************************************************************
FileName:  wf_mac.h 
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

#ifndef __WFMAC_H
#define __WFMAC_H

//============================================================================
//                                  Include files
//============================================================================

#include "hardware_profile.h"

#if defined(TCPIP_IF_MRF24W)
    #include "wf_config.h"  
    #include "tcpip/wf_api.h"
    #include "wf_mgmt_msg.h"
    #include "wf_driver_priv.h"        
    #include "wf_raw.h"

	// type definition
	typedef union {
		uint8_t v[4];
		struct {
			uint16_t			ByteCount;
			unsigned char	PreviouslyIgnored:1;
			unsigned char	RXDCPreviouslySeen:1;
			unsigned char	CarrierPreviouslySeen:1;
			unsigned char	CodeViolation:1;
			unsigned char	CRCError:1;
			unsigned char	LengthCheckError:1;
			unsigned char	LengthOutOfRange:1;
			unsigned char	ReceiveOk:1;
			unsigned char	Multicast:1;
			unsigned char	Broadcast:1;
			unsigned char	DribbleNibble:1;
			unsigned char	ControlFrame:1;
			unsigned char	PauseControlFrame:1;
			unsigned char	UnsupportedOpcode:1;
			unsigned char	VLANType:1;
			unsigned char	Zero:1;
		} bits;
	} RXSTATUS;
#endif

void SetNetworkConfig(NET_CONFIG* pNetIf);
NET_CONFIG* GetNetworkConfig(void);


#endif /* __WFMAC_H */

