/*******************************************************************************
  Multiple MAC Module implementation for Microchip Stack

  Summary:
    
  Description:
*******************************************************************************/

/*******************************************************************************
FileName:   tcpip_mac_object.c
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

#include "mac_private.h"
#include "tcpip/tcpip_mac_object.h"
#include "wifi/mrf24w_mac.h"



/************************************
 *  the PIC32 MAC parameterized interface implementation
 *************************************/



TCPIP_MAC_RES MACInitialize(const TCPIP_STACK_MODULE_CTRL* const stackData,
                            const TCPIP_MAC_MODULE_GONFIG* const moduleData )
{
    TCPIP_MAC_ID    macId;
    TCPIP_MAC_RES (*macInitF)(const TCPIP_STACK_MODULE_CTRL* const stackData, const TCPIP_MAC_MODULE_GONFIG* const moduleData );
    
    macId = stackData->pNetIf->macId;

    switch(macId)
    {
#if ( ((__PIC32_FEATURE_SET__ >= 500) && (__PIC32_FEATURE_SET__ <= 799) && defined (_ETH)) && defined(TCPIP_IF_PIC32INT) )
        case TCPIP_MAC_ID_PIC32INT:
            macInitF = PIC32MACInitialize;
            break;
#endif
            
#if defined(TCPIP_IF_MRF24W)
        case TCPIP_MAC_ID_MRF24W:
            macInitF = MRF24W_MACInitialize;
            break;
#endif

#if defined(TCPIP_IF_ENC28J60)
		case TCPIP_MAC_ID_ENCJ60:
			macInitF = ENC28_MACInitialize;
			break;
#endif
		
#if defined(TCPIP_IF_ENCX24J600)
		case TCPIP_MAC_ID_ENCJ600:
			macInitF = ENCX24_MACInitialize;
			break;
#endif

        default:
            macInitF = 0;
            break;
    }

    return macInitF?(*macInitF)(stackData, moduleData):TCPIP_MAC_RES_TYPE_ERR;
}

TCPIP_MAC_RES MACDeinitialize(const TCPIP_STACK_MODULE_CTRL* const stackData )
{
    TCPIP_MAC_ID    macId = stackData->pNetIf->macId;
    TCPIP_MAC_RES   (*macDeinitF)(const TCPIP_STACK_MODULE_CTRL* const stackData );

    switch(macId)
    {
#if ( ((__PIC32_FEATURE_SET__ >= 500) && (__PIC32_FEATURE_SET__ <= 799) && defined (_ETH)) && defined(TCPIP_IF_PIC32INT) )
        case TCPIP_MAC_ID_PIC32INT:
            macDeinitF = PIC32MACDeinitialize;
            break;
#endif
            
#if defined(TCPIP_IF_MRF24W)
        case TCPIP_MAC_ID_MRF24W:
            macDeinitF = MRF24W_MACDeinitialize;
            break;
#endif

#if defined(TCPIP_IF_ENC28J60)
		case TCPIP_MAC_ID_ENCJ60:
			macDeinitF = ENC28_MACDeinitialize;
			break;
#endif
		
#if defined(TCPIP_IF_ENCX24J600)
		case TCPIP_MAC_ID_ENCJ600:
			macDeinitF = ENCX24_MACDeinitialize;
			break;
#endif

        default:
            macDeinitF = 0;
            break;
    }

    return macDeinitF?(*macDeinitF)(stackData):TCPIP_MAC_RES_TYPE_ERR;
}

TCPIP_MAC_HANDLE MACOpen( TCPIP_MAC_ID macId )
{
    TCPIP_MAC_HANDLE    (*macOpenF)( TCPIP_MAC_ID macId );
    
    switch(macId)
    {
#if ( ((__PIC32_FEATURE_SET__ >= 500) && (__PIC32_FEATURE_SET__ <= 799) && defined (_ETH)) && defined(TCPIP_IF_PIC32INT) )
        case TCPIP_MAC_ID_PIC32INT:
            macOpenF = PIC32MACOpen;
            break;
#endif
            
#if defined(TCPIP_IF_MRF24W)
        case TCPIP_MAC_ID_MRF24W:
            macOpenF = MRF24W_MACOpen;
            break;
#endif

#if defined(TCPIP_IF_ENC28J60)
		case TCPIP_MAC_ID_ENCJ60:
			macOpenF = ENC28_MACOpen;
			break;
#endif
		
#if defined(TCPIP_IF_ENCX24J600)
		case TCPIP_MAC_ID_ENCJ600:
			macOpenF = ENCX24_MACOpen;
			break;
#endif

        default:
            macOpenF = 0;
            break;
    }

    return macOpenF?(*macOpenF)(macId):0;
    
}




TCPIP_MAC_RES MACClose(TCPIP_MAC_HANDLE hMac)
{
    return (*((const TCPIP_MAC_DCPT*)hMac)->pObj->MACClose)(hMac);
}
    
bool MACIsLinked(TCPIP_MAC_HANDLE hMac)
{
    return (*((const TCPIP_MAC_DCPT*)hMac)->pObj->MACIsLinked)(hMac);
}
    
int MACGetHeader(TCPIP_MAC_HANDLE hMac, MAC_ADDR *remote, uint16_t* type)
{
    return (*((const TCPIP_MAC_DCPT*)hMac)->pObj->MACGetHeader)(hMac, remote, type);
}
    
void MACSetReadPtrInRx(TCPIP_MAC_HANDLE hMac, uint16_t offset)
{
    (*((const TCPIP_MAC_DCPT*)hMac)->pObj->MACSetReadPtrInRx)(hMac, offset);
}
    
PTR_BASE MACSetWritePtr(TCPIP_MAC_HANDLE hMac, PTR_BASE address)
{
    return (*((const TCPIP_MAC_DCPT*)hMac)->pObj->MACSetWritePtr)(hMac, address);
}
 
PTR_BASE MACGetReadPtrInRx(TCPIP_MAC_HANDLE hMac)
{
    return (*((const TCPIP_MAC_DCPT*)hMac)->pObj->MACGetReadPtrInRx)(hMac);
}
   
PTR_BASE MACSetReadPtr(TCPIP_MAC_HANDLE hMac, PTR_BASE address)
{
    return (*((const TCPIP_MAC_DCPT*)hMac)->pObj->MACSetReadPtr)(hMac, address);
}

PTR_BASE MACSetBaseReadPtr(TCPIP_MAC_HANDLE hMac, PTR_BASE address)
{
    return (*((const TCPIP_MAC_DCPT*)hMac)->pObj->MACSetBaseReadPtr)(hMac, address);
}
    
uint8_t MACGet(TCPIP_MAC_HANDLE hMac)
{
    return (*((const TCPIP_MAC_DCPT*)hMac)->pObj->MACGet)(hMac);
}
    
uint16_t MACGetArray(TCPIP_MAC_HANDLE hMac, uint8_t *val, uint16_t len)
{
    return (*((const TCPIP_MAC_DCPT*)hMac)->pObj->MACGetArray)(hMac, val, len);
}
    
void MACDiscardRx(TCPIP_MAC_HANDLE hMac)
{
    return (*((const TCPIP_MAC_DCPT*)hMac)->pObj->MACDiscardRx)(hMac);
}
    
uint16_t MACGetFreeRxSize(TCPIP_MAC_HANDLE hMac)
{
    return (*((const TCPIP_MAC_DCPT*)hMac)->pObj->MACGetFreeRxSize)(hMac);
}
    
void  MACMemCopyAsync(TCPIP_MAC_HANDLE hMac, PTR_BASE destAddr, PTR_BASE sourceAddr, uint16_t len)
{
    (*((const TCPIP_MAC_DCPT*)hMac)->pObj->MACMemCopyAsync)(hMac, destAddr, sourceAddr, len);
}
    
bool MACIsMemCopyDone(TCPIP_MAC_HANDLE hMac)
{
    return (*((const TCPIP_MAC_DCPT*)hMac)->pObj->MACIsMemCopyDone)(hMac);
}
    
void MACPutHeader(TCPIP_MAC_HANDLE hMac, MAC_ADDR *remote, uint16_t type, uint16_t dataLen)
{
    (*((const TCPIP_MAC_DCPT*)hMac)->pObj->MACPutHeader)(hMac, remote, type, dataLen);
}
    
bool MACIsTxReady(TCPIP_MAC_HANDLE hMac)
{
    return (*((const TCPIP_MAC_DCPT*)hMac)->pObj->MACIsTxReady)(hMac);
}
    
void MACPut(TCPIP_MAC_HANDLE hMac, uint8_t val)
{
    (*((const TCPIP_MAC_DCPT*)hMac)->pObj->MACPut)(hMac, val);
}
    
void MACPutArray(TCPIP_MAC_HANDLE hMac, uint8_t *val, uint16_t len)
{
    (*((const TCPIP_MAC_DCPT*)hMac)->pObj->MACPutArray)(hMac, val, len);
}
    
void MACFlush(TCPIP_MAC_HANDLE hMac)
{
    (*((const TCPIP_MAC_DCPT*)hMac)->pObj->MACFlush)(hMac);
}
    
bool MACCheckLink(TCPIP_MAC_HANDLE hMac)
{
    return (*((const TCPIP_MAC_DCPT*)hMac)->pObj->MACCheckLink)(hMac);
}
    
PTR_BASE MACGetTxBaseAddr(TCPIP_MAC_HANDLE hMac)
{
    return (*((const TCPIP_MAC_DCPT*)hMac)->pObj->MACGetTxBaseAddr)(hMac);
}
    
PTR_BASE MACGetHttpBaseAddr(TCPIP_MAC_HANDLE hMac)
{
    return (*((const TCPIP_MAC_DCPT*)hMac)->pObj->MACGetHttpBaseAddr)(hMac);
}
    
PTR_BASE MACGetSslBaseAddr(TCPIP_MAC_HANDLE hMac)
{
    return (*((const TCPIP_MAC_DCPT*)hMac)->pObj->MACGetSslBaseAddr)(hMac);
}
    
uint16_t MACGetRxSize(TCPIP_MAC_HANDLE hMac)
{
    return (*((const TCPIP_MAC_DCPT*)hMac)->pObj->MACGetRxSize)(hMac);
}
    
uint16_t MACGetRamSize(TCPIP_MAC_HANDLE hMac)
{
    return (*((const TCPIP_MAC_DCPT*)hMac)->pObj->MACGetRamSize)(hMac);
}
    
uint16_t MACGetTxBuffSize(TCPIP_MAC_HANDLE hMac)
{
    return (*((const TCPIP_MAC_DCPT*)hMac)->pObj->MACGetTxBuffSize)(hMac);
}
    
uint16_t MACCalcRxChecksum(TCPIP_MAC_HANDLE hMac, uint16_t offset, uint16_t len)
{
    return (*((const TCPIP_MAC_DCPT*)hMac)->pObj->MACCalcRxChecksum)(hMac, offset, len);
}
    
uint16_t MACCalcIPBufferChecksum(TCPIP_MAC_HANDLE hMac, uint16_t len)
{
    return (*((const TCPIP_MAC_DCPT*)hMac)->pObj->MACCalcIPBufferChecksum)(hMac, len);
}
    
void MACSetRXHashTableEntry(TCPIP_MAC_HANDLE hMac, MAC_ADDR DestMACAddr)
{
    (*((const TCPIP_MAC_DCPT*)hMac)->pObj->MACSetRXHashTableEntry)(hMac, DestMACAddr);
}
    
void MACPowerDown(TCPIP_MAC_HANDLE hMac)
{
    (*((const TCPIP_MAC_DCPT*)hMac)->pObj->MACPowerDown)(hMac);
}
    
void MACEDPowerDown(TCPIP_MAC_HANDLE hMac)
{
    (*((const TCPIP_MAC_DCPT*)hMac)->pObj->MACEDPowerDown)(hMac);
}
    
void MACPowerUp(TCPIP_MAC_HANDLE hMac)
{
    (*((const TCPIP_MAC_DCPT*)hMac)->pObj->MACPowerUp)(hMac);
}
    
void MACProcess(TCPIP_MAC_HANDLE hMac)
{
    (*((const TCPIP_MAC_DCPT*)hMac)->pObj->MACProcess)(hMac);
}

bool MACRxFilter(TCPIP_MAC_HANDLE hMac, NODE_INFO* pPkt)
{
    return (*((const TCPIP_MAC_DCPT*)hMac)->pObj->MACRxFilter)(hMac, pPkt);
}

void MACConnect(TCPIP_MAC_HANDLE hMac)
{
    (*((const TCPIP_MAC_DCPT*)hMac)->pObj->MACConnect)(hMac);
}

#if defined(TCPIP_STACK_USE_EVENT_NOTIFICATION)
TCPIP_MAC_EVENT_RESULT MACEventSetNotifyEvents(TCPIP_MAC_HANDLE hMac, TCPIP_MAC_EVENT_GROUP tcpEvGroup, TCPIP_EVENT tcpipEvents)
{
        return (*((const TCPIP_MAC_DCPT*)hMac)->pObj->MACEventSetNotifyEvents)(hMac, tcpEvGroup, tcpipEvents);
}

TCPIP_MAC_EVENT_RESULT MACEventClearNotifyEvents(TCPIP_MAC_HANDLE hMac, TCPIP_MAC_EVENT_GROUP tcpEvGroup, TCPIP_EVENT tcpipEvents)
{
        return (*((const TCPIP_MAC_DCPT*)hMac)->pObj->MACEventClearNotifyEvents)(hMac, tcpEvGroup, tcpipEvents);
}

TCPIP_MAC_EVENT_RESULT MACEventAck(TCPIP_MAC_HANDLE hMac, TCPIP_MAC_EVENT_GROUP tcpEvGroup, TCPIP_EVENT tcpipEvents)
{
        return (*((const TCPIP_MAC_DCPT*)hMac)->pObj->MACEventAck)(hMac, tcpEvGroup, tcpipEvents);
}

TCPIP_EVENT MACEventGetPending(TCPIP_MAC_HANDLE hMac, TCPIP_MAC_EVENT_GROUP tcpEvGroup)
{
        return (*((const TCPIP_MAC_DCPT*)hMac)->pObj->MACEventGetPending)(hMac, tcpEvGroup);
}

TCPIP_MAC_EVENT_RESULT MACEventSetNotifyHandler(TCPIP_MAC_HANDLE hMac, TCPIP_MAC_EVENT_GROUP tcpEvGroup, pMacEventF eventHandler, void* hParam)
{
        return (*((const TCPIP_MAC_DCPT*)hMac)->pObj->MACEventSetNotifyHandler)(hMac, tcpEvGroup, eventHandler, hParam);
}


#endif  // (TCPIP_STACK_USE_EVENT_NOTIFICATION)




