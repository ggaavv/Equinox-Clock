/*******************************************************************************
  Multiple MAC Module implementation for Microchip Stack

  Summary:
    
  Description:
*******************************************************************************/

/*******************************************************************************
FileName:  tcpip_mac_object.h 
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

#ifndef _MAC_PIC32_H_ 
#define _MAC_PIC32_H_ 

#include "tcpip_mac.h"
#include "tcpip_manager.h"


/************************************
 *  the PIC32 MAC parameterized interface implementation
 *************************************/

typedef struct
{
    TCPIP_MAC_RES       (*MACClose)(TCPIP_MAC_HANDLE hMac);
    bool                (*MACIsLinked)(TCPIP_MAC_HANDLE hMac);
    int                 (*MACGetHeader)(TCPIP_MAC_HANDLE hMac, MAC_ADDR *remote, uint16_t* type);
    void                (*MACSetReadPtrInRx)(TCPIP_MAC_HANDLE hMac, uint16_t offset);
    PTR_BASE            (*MACSetWritePtr)(TCPIP_MAC_HANDLE hMac, PTR_BASE address);
    PTR_BASE            (*MACGetReadPtrInRx)(TCPIP_MAC_HANDLE hMac);
    PTR_BASE            (*MACSetBaseReadPtr)(TCPIP_MAC_HANDLE hMac, PTR_BASE address);
    PTR_BASE            (*MACSetReadPtr)(TCPIP_MAC_HANDLE hMac, PTR_BASE address);
    uint8_t                (*MACGet)(TCPIP_MAC_HANDLE hMac);
    uint16_t              (*MACGetArray)(TCPIP_MAC_HANDLE hMac, uint8_t *val, uint16_t len);
    void                (*MACDiscardRx)(TCPIP_MAC_HANDLE hMac);
    uint16_t              (*MACGetFreeRxSize)(TCPIP_MAC_HANDLE hMac);
    void                (*MACMemCopyAsync)(TCPIP_MAC_HANDLE hMac, PTR_BASE destAddr, PTR_BASE sourceAddr, uint16_t len);
    bool                (*MACIsMemCopyDone)(TCPIP_MAC_HANDLE hMac);
    void                (*MACPutHeader)(TCPIP_MAC_HANDLE hMac, MAC_ADDR *remote, uint16_t type, uint16_t dataLen);
    bool                (*MACIsTxReady)(TCPIP_MAC_HANDLE hMac);
    void                (*MACPut)(TCPIP_MAC_HANDLE hMac, uint8_t val);
    void                (*MACPutArray)(TCPIP_MAC_HANDLE hMac, uint8_t *val, uint16_t len);
    void                (*MACFlush)(TCPIP_MAC_HANDLE hMac);
    bool                (*MACCheckLink)(TCPIP_MAC_HANDLE hMac);
    PTR_BASE            (*MACGetTxBaseAddr)(TCPIP_MAC_HANDLE hMac);
    PTR_BASE            (*MACGetHttpBaseAddr)(TCPIP_MAC_HANDLE hMac);
    PTR_BASE            (*MACGetSslBaseAddr)(TCPIP_MAC_HANDLE hMac);
    uint16_t              (*MACGetRxSize)(TCPIP_MAC_HANDLE hMac);
    uint16_t              (*MACGetRamSize)(TCPIP_MAC_HANDLE hMac);
    uint16_t              (*MACGetTxBuffSize)(TCPIP_MAC_HANDLE hMac);
    uint16_t              (*MACCalcRxChecksum)(TCPIP_MAC_HANDLE hMac, uint16_t offset, uint16_t len);
    uint16_t              (*MACCalcIPBufferChecksum)(TCPIP_MAC_HANDLE hMac, uint16_t len);
    void                (*MACSetRXHashTableEntry)(TCPIP_MAC_HANDLE hMac, MAC_ADDR DestMACAddr);
    void                (*MACPowerDown)(TCPIP_MAC_HANDLE hMac);
    void	            (*MACEDPowerDown)(TCPIP_MAC_HANDLE hMac);
    void 	            (*MACPowerUp)(TCPIP_MAC_HANDLE hMac);
    void 	            (*MACProcess)(TCPIP_MAC_HANDLE hMac);
    bool                (*MACRxFilter)(TCPIP_MAC_HANDLE hMac, NODE_INFO* pPkt);
    void 	            (*MACConnect)(TCPIP_MAC_HANDLE hMac);
#if defined(TCPIP_STACK_USE_EVENT_NOTIFICATION)
TCPIP_MAC_EVENT_RESULT  (*MACEventSetNotifyEvents)(TCPIP_MAC_HANDLE hMac, TCPIP_MAC_EVENT_GROUP tcpEvGroup, TCPIP_EVENT tcpipEvents);
TCPIP_MAC_EVENT_RESULT  (*MACEventClearNotifyEvents)(TCPIP_MAC_HANDLE hMac, TCPIP_MAC_EVENT_GROUP tcpEvGroup, TCPIP_EVENT tcpipEvents);
TCPIP_MAC_EVENT_RESULT  (*MACEventAck)(TCPIP_MAC_HANDLE hMac, TCPIP_MAC_EVENT_GROUP tcpEvGroup, TCPIP_EVENT tcpipEvents);
TCPIP_EVENT         (*MACEventGetPending)(TCPIP_MAC_HANDLE hMac, TCPIP_MAC_EVENT_GROUP tcpEvGroup);
TCPIP_MAC_EVENT_RESULT  (*MACEventSetNotifyHandler)(TCPIP_MAC_HANDLE hMac, TCPIP_MAC_EVENT_GROUP tcpEvGroup, pMacEventF eventHandler, void* hParam);
#endif  // (TCPIP_STACK_USE_EVENT_NOTIFICATION)
}TCPIP_MAC_OBJECT;        // TCPIP MAC object descriptor

typedef struct
{
    const TCPIP_MAC_OBJECT* pObj;           // associated object
                                        // pointer to the object here is intended to allow
                                        // multiple MAC objects of the same type
                                        // to share an unique const object table
    void*               mac_data[0];    // specific MAC object data
}TCPIP_MAC_DCPT; 
    
// supported MAC objects

TCPIP_MAC_RES       PIC32MACInitialize(const TCPIP_STACK_MODULE_CTRL* const stackData,
                                        const TCPIP_MAC_MODULE_GONFIG* const initData );
TCPIP_MAC_RES       PIC32MACDeinitialize(const TCPIP_STACK_MODULE_CTRL* const stackData );
TCPIP_MAC_HANDLE    PIC32MACOpen( TCPIP_MAC_ID macId );


TCPIP_MAC_RES       MRF24W_MACInitialize(const TCPIP_STACK_MODULE_CTRL* const stackData,
                                        const TCPIP_MAC_MODULE_GONFIG* const initData );
TCPIP_MAC_RES       MRF24W_MACDeinitialize(const TCPIP_STACK_MODULE_CTRL* const stackData );
TCPIP_MAC_HANDLE    MRF24W_MACOpen( TCPIP_MAC_ID macId );

TCPIP_MAC_RES 		ENC28_MACInitialize(const TCPIP_STACK_MODULE_CTRL* const stackData,
                                        const TCPIP_MAC_MODULE_GONFIG* const initData );
TCPIP_MAC_RES 		ENC28_MACDeinitialize(const TCPIP_STACK_MODULE_CTRL* const stackData );
TCPIP_MAC_HANDLE 	ENC28_MACOpen( TCPIP_MAC_ID macId );

TCPIP_MAC_RES 		ENCX24_MACInitialize(const TCPIP_STACK_MODULE_CTRL* const stackData,
                                        const TCPIP_MAC_MODULE_GONFIG* const initData );
TCPIP_MAC_RES 		ENCX24_MACDeinitialize(const TCPIP_STACK_MODULE_CTRL* const stackData );
TCPIP_MAC_HANDLE 	ENCX24_MACOpen( TCPIP_MAC_ID macId );


#endif  // _MAC_PIC32_H_ 

