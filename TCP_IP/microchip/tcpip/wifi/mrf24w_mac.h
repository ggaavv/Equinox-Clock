/*******************************************************************************
  MRF24W Driver

  Summary:
    Module for Microchip TCP/IP Stack
    
  Description:
    -Provides access to MRF24W WiFi controller
    -Reference: MRF24W Data sheet, IEEE 802.11 Standard
*******************************************************************************/

/*******************************************************************************
FileName:  MRF24W_Mac.h 
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

#ifndef _MRF24W_MAC_H_
#define _MRF24W_MAC_H_

//============================================================================
//                                  Include files
//============================================================================

#include "hardware_profile.h"

#if defined(TCPIP_IF_MRF24W)

#include "tcpip_config.h"  
#include "wf_config.h"  
#include "tcpip/wf_api.h"
#include "wf_mgmt_msg.h"
#include "wf_driver_priv.h"
#include "wf_raw.h"
#include "tcpip/tcpip_events.h"
#include "tcpip/mac_events.h"
#include "system/drivers/drv_spi.h"


// local definitions
#define MAX_PACKET_SIZE     (1514ul)

#define RAMSIZE			14170ul
#define TXSTART			(RAMSIZE - ((4ul + MAX_PACKET_SIZE + 4ul)*2) - TCP_ETH_RAM_SIZE - RESERVED_HTTP_MEMORY - RESERVED_SSL_MEMORY)
#define RXSTART			(0ul)
#define	RXSTOP			((TXSTART-2ul) | 0x0001ul)
#define RXSIZE			(RXSTOP-RXSTART+1ul)
#define BASE_TX_ADDR	(TXSTART + 4ul)
#define BASE_TCB_ADDR	(BASE_TX_ADDR + ((MAX_PACKET_SIZE + 4ul)*2))
#define BASE_HTTPB_ADDR (BASE_TCB_ADDR + TCP_ETH_RAM_SIZE)
#define BASE_SSLB_ADDR	(BASE_HTTPB_ADDR + RESERVED_HTTP_MEMORY)

// initialization function
TCPIP_MAC_RES   MRF24W_MACInit(NET_CONFIG* pNetIf);

// interface functions

TCPIP_MAC_RES   MRF24W_MACClose(TCPIP_MAC_HANDLE hMac);
bool            MRF24W_MACIsLinked(void);
int             MRF24W_MACGetHeader(MAC_ADDR *remote, uint16_t* type);
void            MRF24W_MACSetReadPtrInRx(uint16_t offset);
PTR_BASE        MRF24W_MACSetWritePtr(PTR_BASE address);
PTR_BASE        MRF24W_MACGetReadPtrInRx(void);
PTR_BASE        MRF24W_MACSetReadPtr(PTR_BASE address);
PTR_BASE        MRF24W_MACSetBaseReadPtr(PTR_BASE address);
uint8_t            MRF24W_MACGet();
uint16_t            MRF24W_MACGetArray(uint8_t *val, uint16_t len);
void            MRF24W_MACDiscardRx(void);
uint16_t            MRF24W_MACGetFreeRxSize(void);
void            MRF24W_MACMemCopyAsync(PTR_BASE destAddr, PTR_BASE sourceAddr, uint16_t len);
bool            MRF24W_MACIsMemCopyDone(void);
void            MRF24W_MACPutHeader(MAC_ADDR *remote, uint16_t type, uint16_t dataLen);
bool            MRF24W_MACIsTxReady(void);
void            MRF24W_MACPut(uint8_t val);
void            MRF24W_MACPutArray(uint8_t *val, uint16_t len);
void            MRF24W_MACFlush(void);
uint16_t            MRF24W_MACCalcRxChecksum(TCPIP_MAC_HANDLE hMac, uint16_t offset, uint16_t len);
uint16_t            MRF24W_MACCalcIPBufferChecksum(TCPIP_MAC_HANDLE hMac, uint16_t len);
void            MRF24W_MACPowerDown(void);
void            MRF24W_MACPowerUp(void);
void            MRF24W_MACProcess(TCPIP_MAC_HANDLE hMac);
bool            MRF24W_MACRxFilter(TCPIP_MAC_HANDLE hMac, NODE_INFO* pPkt);
void            MRF24W_MACConnect(TCPIP_MAC_HANDLE hMac);

#if defined(TCPIP_STACK_USE_EVENT_NOTIFICATION)
TCPIP_MAC_EVENT_RESULT    MRF24W_MACEventInit(TCPIP_MAC_HANDLE hMac, int intPri, int subPri);
TCPIP_MAC_EVENT_RESULT    MRF24W_MACEventDeInit(TCPIP_MAC_HANDLE hMac);
TCPIP_MAC_EVENT_RESULT    MRF24W_MACEventSetNotifyEvents(TCPIP_MAC_HANDLE hMac, TCPIP_MAC_EVENT_GROUP tcpEvGroup, TCPIP_EVENT tcpipEvents);
TCPIP_MAC_EVENT_RESULT    MRF24W_MACEventClearNotifyEvents(TCPIP_MAC_HANDLE hMac, TCPIP_MAC_EVENT_GROUP tcpEvGroup, TCPIP_EVENT tcpipEvents);
TCPIP_MAC_EVENT_RESULT    MRF24W_MACEventAck(TCPIP_MAC_HANDLE hMac, TCPIP_MAC_EVENT_GROUP tcpEvGroup, TCPIP_EVENT tcpipEvents);
TCPIP_EVENT               MRF24W_MACEventGetPending(TCPIP_MAC_HANDLE hMac, TCPIP_MAC_EVENT_GROUP tcpEvGroup);
TCPIP_MAC_EVENT_RESULT    MRF24W_MACEventSetNotifyHandler(TCPIP_MAC_HANDLE hMac, TCPIP_MAC_EVENT_GROUP tcpEvGroup, pMacEventF eventHandler, void* hParam);

void                      MRF24W_SetUserEvents(uint8_t event, uint16_t eventInfo, bool isMgmt);

#endif  // (TCPIP_STACK_USE_EVENT_NOTIFICATION)










#endif  // defined(TCPIP_IF_MRF24W)

#endif /* _MRF24W_MAC_H_ */

