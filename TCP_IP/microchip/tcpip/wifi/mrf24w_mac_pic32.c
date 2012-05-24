/*******************************************************************************
  MRF24W Driver Medium Access Control (MAC) Layer

  Summary:
    Module for Microchip TCP/IP Stack PIC32 implementation
    for multiple MAC support
    
  Description:
*******************************************************************************/

/*******************************************************************************
FileName:   mrf24w_Mac_pic32.c
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



// Compile only for PIC32MX MRF24W MAC interface
#if (defined(__PIC32MX__) && defined(TCPIP_IF_MRF24W))||( defined(__dsPIC33F__) && defined(TCPIP_IF_MRF24W))||( defined(__PIC24F__) && defined(TCPIP_IF_MRF24W))

#include "mac_private.h"
#include "tcpip/tcpip_mac_object.h"

#include "wf_mac.h"
#include "mrf24w_mac.h"
#include "mrf24w_mac_pic32.h"

#if SSL_SAVE_CONTEXT_IN_PIC_RAM
// SSL buffers
static unsigned char		_SSlBuffer[RESERVED_SSL_MEMORY];
#endif  // SSL_SAVE_CONTEXT_IN_PIC_RAM


// function proto
static TCPIP_MAC_RES       mrf24w_MACClose(TCPIP_MAC_HANDLE hMac);
static bool                mrf24w_MACIsLinked(TCPIP_MAC_HANDLE hMac);
static int                 mrf24w_MACGetHeader(TCPIP_MAC_HANDLE hMac, MAC_ADDR *remote, uint16_t* type);
static void                mrf24w_MACSetReadPtrInRx(TCPIP_MAC_HANDLE hMac, uint16_t offset);
static PTR_BASE            mrf24w_MACSetWritePtr(TCPIP_MAC_HANDLE hMac, PTR_BASE address);
static PTR_BASE            mrf24w_MACGetReadPtrInRx(TCPIP_MAC_HANDLE hMac);
static PTR_BASE            mrf24w_MACSetBaseReadPtr(TCPIP_MAC_HANDLE hMac, PTR_BASE address);
static PTR_BASE            mrf24w_MACSetReadPtr(TCPIP_MAC_HANDLE hMac, PTR_BASE address);
static uint8_t                mrf24w_MACGet(TCPIP_MAC_HANDLE hMac);
static uint16_t              mrf24w_MACGetArray(TCPIP_MAC_HANDLE hMac, uint8_t *val, uint16_t len);
static void                mrf24w_MACDiscardRx(TCPIP_MAC_HANDLE hMac);
static uint16_t              mrf24w_MACGetFreeRxSize(TCPIP_MAC_HANDLE hMac);
static void                mrf24w_MACMemCopyAsync(TCPIP_MAC_HANDLE hMac, PTR_BASE destAddr, PTR_BASE sourceAddr, uint16_t len);
static bool                mrf24w_MACIsMemCopyDone(TCPIP_MAC_HANDLE hMac);
static void                mrf24w_MACPutHeader(TCPIP_MAC_HANDLE hMac, MAC_ADDR *remote, uint16_t type, uint16_t dataLen);
static bool                mrf24w_MACIsTxReady(TCPIP_MAC_HANDLE hMac);
static void                mrf24w_MACPut(TCPIP_MAC_HANDLE hMac, uint8_t val);
static void                mrf24w_MACPutArray(TCPIP_MAC_HANDLE hMac, uint8_t *val, uint16_t len);
static void                mrf24w_MACFlush(TCPIP_MAC_HANDLE hMac);
static bool                mrf24w_MACCheckLink(TCPIP_MAC_HANDLE hMac);
static PTR_BASE            mrf24w_MACGetTxBaseAddr(TCPIP_MAC_HANDLE hMac);
static PTR_BASE            mrf24w_MACGetHttpBaseAddr(TCPIP_MAC_HANDLE hMac);
static PTR_BASE            mrf24w_MACGetSslBaseAddr(TCPIP_MAC_HANDLE hMac);
static uint16_t              mrf24w_MACGetRxSize(TCPIP_MAC_HANDLE hMac);
static uint16_t              mrf24w_MACGetRamSize(TCPIP_MAC_HANDLE hMac);
static uint16_t              mrf24w_MACGetTxBuffSize(TCPIP_MAC_HANDLE hMac);
static uint16_t              mrf24w_MACCalcRxChecksum(TCPIP_MAC_HANDLE hMac, uint16_t offset, uint16_t len);
static uint16_t              mrf24w_MACCalcIPBufferChecksum(TCPIP_MAC_HANDLE hMac, uint16_t len);
static void                mrf24w_MACSetRXHashTableEntry(TCPIP_MAC_HANDLE hMac, MAC_ADDR DestMACAddr);
static void                mrf24w_MACPowerDown(TCPIP_MAC_HANDLE hMac);
static void	               mrf24w_MACEDPowerDown(TCPIP_MAC_HANDLE hMac);
static void 	           mrf24w_MACPowerUp(TCPIP_MAC_HANDLE hMac);
static void 	           mrf24w_MACProcess(TCPIP_MAC_HANDLE hMac);
static bool                mrf24w_MACRxFilter(TCPIP_MAC_HANDLE hMac, NODE_INFO* pNode);
static void                mrf24w_MACConnect(TCPIP_MAC_HANDLE hMac);
#if defined(TCPIP_STACK_USE_EVENT_NOTIFICATION)
static TCPIP_MAC_EVENT_RESULT mrf24w_MACEventSetNotifyEvents(TCPIP_MAC_HANDLE hMac, TCPIP_MAC_EVENT_GROUP tcpEvGroup, TCPIP_EVENT tcpipEvents);
static TCPIP_MAC_EVENT_RESULT mrf24w_MACEventClearNotifyEvents(TCPIP_MAC_HANDLE hMac, TCPIP_MAC_EVENT_GROUP tcpEvGroup, TCPIP_EVENT tcpipEvents);
static TCPIP_MAC_EVENT_RESULT mrf24w_MACEventAck(TCPIP_MAC_HANDLE hMac, TCPIP_MAC_EVENT_GROUP tcpEvGroup, TCPIP_EVENT tcpipEvents);
static TCPIP_EVENT        mrf24w_MACEventGetPending(TCPIP_MAC_HANDLE hMac, TCPIP_MAC_EVENT_GROUP tcpEvGroup);
static TCPIP_MAC_EVENT_RESULT mrf24w_MACEventSetNotifyHandler(TCPIP_MAC_HANDLE hMac, TCPIP_MAC_EVENT_GROUP tcpEvGroup, pMacEventF eventHandler, void* hParam);
#endif  // (TCPIP_STACK_USE_EVENT_NOTIFICATION)

static void     mrf24w_MACDeinitialize(MRF24W_MAC_DCPT* pDcpt );
    
// the PIC32 MRF24W MAC descriptor
// no support for multiple instances
static const TCPIP_MAC_OBJECT _pic32_mrf24w_mac_obj = 
{
    mrf24w_MACClose,
    mrf24w_MACIsLinked,
    mrf24w_MACGetHeader,
    mrf24w_MACSetReadPtrInRx,
    mrf24w_MACSetWritePtr,
    mrf24w_MACGetReadPtrInRx,
    mrf24w_MACSetBaseReadPtr,
    mrf24w_MACSetReadPtr,
    mrf24w_MACGet,
    mrf24w_MACGetArray,
    mrf24w_MACDiscardRx,
    mrf24w_MACGetFreeRxSize,
    mrf24w_MACMemCopyAsync,
    mrf24w_MACIsMemCopyDone,
    mrf24w_MACPutHeader,
    mrf24w_MACIsTxReady,
    mrf24w_MACPut,
    mrf24w_MACPutArray,
    mrf24w_MACFlush,
    mrf24w_MACCheckLink,
    mrf24w_MACGetTxBaseAddr,
    mrf24w_MACGetHttpBaseAddr,
    mrf24w_MACGetSslBaseAddr,
    mrf24w_MACGetRxSize,
    mrf24w_MACGetRamSize,
    mrf24w_MACGetTxBuffSize,
    mrf24w_MACCalcRxChecksum,
    mrf24w_MACCalcIPBufferChecksum,
    mrf24w_MACSetRXHashTableEntry,
    mrf24w_MACPowerDown,
    mrf24w_MACEDPowerDown,
    mrf24w_MACPowerUp,
    mrf24w_MACProcess,
    mrf24w_MACRxFilter,
    mrf24w_MACConnect,
#if defined(TCPIP_STACK_USE_EVENT_NOTIFICATION)
    mrf24w_MACEventSetNotifyEvents,
    mrf24w_MACEventClearNotifyEvents,
    mrf24w_MACEventAck,
    mrf24w_MACEventGetPending,
    mrf24w_MACEventSetNotifyHandler,
#endif  // (TCPIP_STACK_USE_EVENT_NOTIFICATION)
};

// only one hardware instance for now!
static MRF24W_MAC_DCPT _pic32_mrf24w_mac_dcpt[1] = 
{
    {
        &_pic32_mrf24w_mac_obj,
        // specific PIC32 MAC data 
        0,                                  // pNetIf
        0,                                  // isOpen
    }
};



/*
 * interface functions
 *
*/

TCPIP_MAC_RES MRF24W_MACInitialize(const TCPIP_STACK_MODULE_CTRL* const stackData,
                                      const TCPIP_MAC_MODULE_GONFIG* const initData )
{
    MRF24W_MAC_DCPT* pDcpt;
    TCPIP_MAC_RES       res;
    
    if(stackData->pNetIf->macId != TCPIP_MAC_ID_MRF24W)
    {
        return TCPIP_MAC_RES_TYPE_ERR;      // no other type supported
    }

    pDcpt = _pic32_mrf24w_mac_dcpt + 0; // no other instance supported
    if(pDcpt->isOpen != 0)
    { 
        return TCPIP_MAC_RES_IS_BUSY;     // have a client connected
    }

#if defined(TCPIP_STACK_USE_EVENT_NOTIFICATION)
    if(MRF24W_MACEventInit(pDcpt, WIFI_EVENT_IPL, WIFI_EVENT_SIPL) != TCPIP_MAC_EVRES_OK)
    {
        return TCPIP_MAC_RES_INIT_FAIL;
    }
#endif
    
    pDcpt->pNetIf = stackData->pNetIf;
    res =  MRF24W_MACInit(pDcpt->pNetIf);

    if(res !=  TCPIP_MAC_RES_OK)
    {
        mrf24w_MACDeinitialize(pDcpt);
    }
    
    return res;
}

TCPIP_MAC_RES MRF24W_MACDeinitialize(const TCPIP_STACK_MODULE_CTRL* const stackData )
{
    if(stackData->pNetIf->macId != TCPIP_MAC_ID_MRF24W)
    {
        return TCPIP_MAC_RES_TYPE_ERR;      // no other type supported
    }

    mrf24w_MACDeinitialize(_pic32_mrf24w_mac_dcpt + 0);   // no other instance supported

    return TCPIP_MAC_RES_OK;

}


TCPIP_MAC_HANDLE MRF24W_MACOpen( TCPIP_MAC_ID macId )
{
    MRF24W_MAC_DCPT* pMacD;
    TCPIP_MAC_HANDLE    hMac = 0;

    if(macId == TCPIP_MAC_ID_MRF24W)
    {
        pMacD = _pic32_mrf24w_mac_dcpt + 0; // no other instance supported
        if((pMacD->isOpen) == 0)
        {   // only one client for now
            pMacD->isOpen = 1;
            hMac = pMacD;
        }
    }
   
    return hMac;
}


static TCPIP_MAC_RES mrf24w_MACClose(TCPIP_MAC_HANDLE hMac)
{
    MRF24W_MAC_DCPT* pMacD = (MRF24W_MAC_DCPT*)hMac;

    pMacD->isOpen = 0;

    return TCPIP_MAC_RES_OK;


}



/****************************************************************************
 * Function:        mrf24w_MACIsLinked
 *
 * PreCondition:    None
 *
 * Input:           None
 *
 * Output:          true if link is up
 *                  false otherwise
 *
 * Side Effects:    None
 *
 * Overview:        This function checks the link status
 *
 * Note:            None 
 *****************************************************************************/
static bool mrf24w_MACIsLinked(TCPIP_MAC_HANDLE hMac)
{
	return MRF24W_MACIsLinked();
}


/****************************************************************************
 * Function:        mrf24w_MACGetTxBaseAddr
 *
 * PreCondition:    None
 *
 * Input:           None
 *
 * Output:          TX buffer base address
 *                
 * Side Effects:    None
 *
 * Overview:        This function returns the address of the current TX buffer.
 *
 * Note:            The returned value could be 0 if currently there's no available TX buffer. 
 *****************************************************************************/
static PTR_BASE mrf24w_MACGetTxBaseAddr(TCPIP_MAC_HANDLE hMac )
{
	return (PTR_BASE)BASE_TX_ADDR;
}

/****************************************************************************
 * Function:        mrf24w_MACGetHttpBaseAddr
 *
 * PreCondition:    None
 *
 * Input:           None
 *
 * Output:          HTTP buffer base address
 *                
 * Side Effects:    None
 *
 * Overview:        This function returns the address of the HTTP buffer.
 *
 * Note:            The HTTP buffer is a static one, always available. 
 *****************************************************************************/
static PTR_BASE mrf24w_MACGetHttpBaseAddr(TCPIP_MAC_HANDLE hMac )
{
	return (PTR_BASE)BASE_HTTPB_ADDR;
}

/****************************************************************************
 * Function:        mrf24w_MACGetSslBaseAddr
 *
 * PreCondition:    None
 *
 * Input:           None
 *
 * Output:          SSL buffer base address
 *                
 * Side Effects:    None
 *
 * Overview:        This function returns the address of the SSL buffer.
 *
 * Note:            The SSL buffer is a static one, always available. 
 *****************************************************************************/
static PTR_BASE mrf24w_MACGetSslBaseAddr(TCPIP_MAC_HANDLE hMac )
{
#if SSL_SAVE_CONTEXT_IN_PIC_RAM
    return (PTR_BASE)_SSlBuffer;
#else
	return (PTR_BASE)BASE_SSLB_ADDR;
#endif  // SSL_SAVE_CONTEXT_IN_PIC_RAM
}

/****************************************************************************
 * Function:        mrf24w_MACGetRxSize
 *
 * PreCondition:    None
 *
 * Input:           None
 *
 * Output:         RX size
 *                
 * Side Effects:    None
 *
 * Overview:        This function returns the size of the RX buffer.
 *
 * Note:            None 
 *****************************************************************************/
static uint16_t mrf24w_MACGetRxSize(TCPIP_MAC_HANDLE hMac)
{
    return RXSIZE;
}

/****************************************************************************
 * Function:        mrf24w_MACGetRamSize
 *
 * PreCondition:    None
 *
 * Input:           None
 *
 * Output:         RAM size
 *                
 * Side Effects:    None
 *
 * Overview:        This function returns the size of the RAM buffer.
 *
 * Note:            Not used, no RAM on the PIC32 ETH controller
 *****************************************************************************/
static uint16_t mrf24w_MACGetRamSize(TCPIP_MAC_HANDLE hMac)
{
    return EMAC_RX_DESCRIPTORS*EMAC_RX_BUFF_SIZE;
}

static uint16_t mrf24w_MACGetTxBuffSize(TCPIP_MAC_HANDLE hMac)
{
    return RAMSIZE;
}

static PTR_BASE mrf24w_MACGetReadPtrInRx(TCPIP_MAC_HANDLE hMac)
{
    return MRF24W_MACGetReadPtrInRx();
}

/**************************
 * TX functions
 ***********************************************/

/****************************************************************************
 * Function:        mrf24w_MACSetWritePtr
 *
 * PreCondition:    None
 *
 * Input:           None
 *
 * Output:          old write pointer
 *                
 * Side Effects:    None
 *
 * Overview:        This function sets the new write pointer.
 *
 * Note:            None
 *****************************************************************************/
static PTR_BASE mrf24w_MACSetWritePtr(TCPIP_MAC_HANDLE hMac, PTR_BASE address)
{
    return MRF24W_MACSetWritePtr(address);
}


/******************************************************************************
 * Function:        bool mrf24w_MACIsTxReady(TCPIP_MAC_HANDLE hMac )
 *
 * PreCondition:    None
 *
 * Input:           None
 *
 * Output:          true: If data can be inserted in the current TX buffer
 *                  false: there is no free TX buffer
 *
 * Side Effects:    None
 *
 * Overview:        Checks if there is an available current TX buffer
 *
 * Note:            None
 *****************************************************************************/
static bool mrf24w_MACIsTxReady(TCPIP_MAC_HANDLE hMac )
{
	return MRF24W_MACIsTxReady();
}

/******************************************************************************
 * Function:        void mrf24w_MACPut(TCPIP_MAC_HANDLE hMac, uint8_t val)
 *
 * PreCondition:    None
 *
 * Input:           uint8_t to be written
 *
 * Output:          None
 *
 * Side Effects:    None
 *
 * Overview:       Writes a uint8_t to the current write location and increments the write pointer. 
 *
 * Note:            None
 *****************************************************************************/
static void mrf24w_MACPut(TCPIP_MAC_HANDLE hMac, uint8_t val)
{
	MRF24W_MACPut(val);
}

/******************************************************************************
 * Function:        void mrf24w_MACPutArray(TCPIP_MAC_HANDLE hMac, uint8_t* buff, uint16_t len)
 *
 * PreCondition:    None
 *
 * Input:           buff - buffer to be written
 *                  len - buffer length
 *
 * Output:          None
 *
 * Side Effects:    None
 *
 * Overview:        Writes a buffer to the current write location and updates the write pointer. 
 *
 * Note:            None
 *****************************************************************************/
static void mrf24w_MACPutArray(TCPIP_MAC_HANDLE hMac, uint8_t *buff, uint16_t len)
{
	MRF24W_MACPutArray(buff, len);
}


/******************************************************************************
 * Function:        void mrf24w_MACPutHeader(TCPIP_MAC_HANDLE hMac, MAC_ADDR *remote, uint16_t type, uint16_t dataLen)
 *
 * PreCondition:    None
 *
 * Input:           remote - Pointer to memory which contains the destination MAC address (6 uint8_ts)
 *                  type - packet type: MAC_IP or ARP
 *                  dataLen - ethernet frame payload
 *
 * Output:          None
 *
 * Side Effects:    None
 *
 * Overview:       Sets the write pointer at the beginning of the current TX buffer
 *                 and sets the ETH header and the frame length. Updates the write pointer
 *
 * Note:            Assumes there is an available TX buffer, i.e. mrf24w_MACIsTxReady() returned !0
 *****************************************************************************/
static void mrf24w_MACPutHeader(TCPIP_MAC_HANDLE hMac, MAC_ADDR *remote, uint16_t type, uint16_t dataLen)
{
    MRF24W_MACPutHeader(remote, type, dataLen);
}



static void mrf24w_MACFlush(TCPIP_MAC_HANDLE hMac )
{
    MRF24W_MACFlush();
}

/**************************
 * RX functions
 ***********************************************/


/******************************************************************************
 * Function:        void mrf24w_MACDiscardRx(TCPIP_MAC_HANDLE hMac )
 *
 * PreCondition:    None
 *
 * Input:           None
 *
 * Output:          None
 *
 * Side Effects:    None
 *
 * Overview:        Marks the last received packet (obtained using 
 *                  mrf24w_MACGetHeader())as being processed and frees the buffer
 *                  memory associated with it.
 *                  It acknowledges the ETHC.
 *
 * Note:            Is is safe to call this function multiple times between
 *                  mrf24w_MACGetHeader() calls.  Extra packets won't be thrown away 
 *                  until mrf24w_MACGetHeader() makes it available.
 *****************************************************************************/
static void mrf24w_MACDiscardRx(TCPIP_MAC_HANDLE hMac )
{
    MRF24W_MACDiscardRx();
}



/******************************************************************************
 * Function:        int mrf24w_MACGetHeader(TCPIP_MAC_HANDLE hMac, MAC_ADDR *remote, uint16_t* type)
 *
 * PreCondition:    None
 *
 * Input:           *remote: Location to store the Source MAC address of the
 *                           received frame.
 *                  *type: Location of a uint16_t to store the constant
 *                         ETHERTYPE_UNKNOWN, ETHERTYPE_IP, or ETHERTYPE_ARP,
 *                         representing the contents of the Ethernet Type
 *                         field.
 *
 * Output:          !0: If a packet of this returned size is waiting in the RX buffer.  The
 *                        remote, and type values are updated.
 *                  false: If a packet was not pending.  remote and type are
 *                         not changed.
 *
 * Side Effects:    Last packet is discarded if mrf24w_MACDiscardRx() hasn't already
 *                  been called.
 *
 * Overview:        None
 *
 * Note:            Sets the read pointer at the beginning of the new packet
 *****************************************************************************/
static int mrf24w_MACGetHeader(TCPIP_MAC_HANDLE hMac, MAC_ADDR *remote, uint16_t* type)
{
    return MRF24W_MACGetHeader(remote, type);
}



/******************************************************************************
 * Function:        void mrf24w_MACSetReadPtrInRx(TCPIP_MAC_HANDLE hMac, uint16_t offset)
 *
 * PreCondition:    A packet has been obtained by calling mrf24w_MACGetHeader() and
 *                  getting a true result.
 *
 * Input:           offset: uint16_t specifying how many uint8_ts beyond the Ethernet
 *                          header's type field to relocate the read pointer.
 *
 * Output:          None
 *
 * Side Effects:    None
 *
 * Overview:        The current read pointer is updated.  All calls to
 *                  mrf24w_MACGet() and mrf24w_MACGetArray() will use these new values.
 *
 * Note:            
 ******************************************************************************/
static void mrf24w_MACSetReadPtrInRx(TCPIP_MAC_HANDLE hMac, uint16_t offset)
{
	MRF24W_MACSetReadPtrInRx(offset);
}


/****************************************************************************
 * Function:        mrf24w_MACSetReadPtr
 *
 * PreCondition:    None
 *
 * Input:           None
 *
 * Output:          old read pointer
 *                
 * Side Effects:    None
 *
 * Overview:        This function sets the new read pointer value.
 *
 * Note:            None
 *****************************************************************************/
static PTR_BASE mrf24w_MACSetReadPtr(TCPIP_MAC_HANDLE hMac, PTR_BASE address)
{
    return MRF24W_MACSetReadPtr(address);
}

/****************************************************************************
 * Function:        mrf24w_MACSetBaseReadPtr
 *
 * PreCondition:    None
 *
 * Input:           None
 *
 * Output:          old read pointer
 *                
 * Side Effects:    None
 *
 * Overview:        This function sets the new read pointer value.
 *
 * Note:            None
 *****************************************************************************/
static PTR_BASE mrf24w_MACSetBaseReadPtr(TCPIP_MAC_HANDLE hMac, PTR_BASE address)
{
    return MRF24W_MACSetBaseReadPtr(address);
}



/******************************************************************************
 * Function:        uint8_t mrf24w_MACGet(TCPIP_MAC_HANDLE hMac )
 *
 * PreCondition:    A valid packet should vahe been obtained or the read pointer properly set.
 *
 * Input:           None
 *
 * Output:          uint8_t read from the current read pointer location
 *
 * Side Effects:    None
 *
 * Overview:        mrf24w_MACGet returns the uint8_t pointed to by the current read pointer location and
 *                  increments the read pointer.
 *
 * Note:            None
 *****************************************************************************/
static uint8_t mrf24w_MACGet(TCPIP_MAC_HANDLE hMac )
{
	return MRF24W_MACGet();
}


/******************************************************************************
 * Function:        uint16_t mrf24w_MACGetArray(TCPIP_MAC_HANDLE hMac, uint8_t *address, uint16_t len)
 *
 * PreCondition:    A valid packet should vahe been obtained or the read pointer properly set.
 *
 * Input:           address: Pointer to storage location
 *                  len:  Number of uint8_ts to read from the data buffer.
 *
 * Output:          number of uint8_ts copied to the data buffer.
 *
 * Side Effects:    None
 *
 * Overview:        Copies data in the supplied buffer.
 *
 * Note:            The read pointer is updated
 *****************************************************************************/
static uint16_t mrf24w_MACGetArray(TCPIP_MAC_HANDLE hMac, uint8_t *address, uint16_t len)
{
	return MRF24W_MACGetArray(address, len);
}

/******************************************************************************
 * Function:        uint16_t mrf24w_MACGetFreeRxSize(TCPIP_MAC_HANDLE hMac )
 *
 * PreCondition:    None
 *
 * Input:           None
 *
 * Output:          An estimate of how much RX buffer space is free at the present time.
 *
 * Side Effects:    None
 *
 * Overview:        None
 *
 * Note:            None
 *****************************************************************************/
static uint16_t mrf24w_MACGetFreeRxSize(TCPIP_MAC_HANDLE hMac )
{
	return MRF24W_MACGetFreeRxSize();
}


/******************************************************************************
 * Function:        void mrf24w_MACMemCopyAsync(TCPIP_MAC_HANDLE hMac, PTR_BASE destAddr, PTR_BASE sourceAddr, uint16_t len)
 *
 * PreCondition:    Read and write pointers properly set if using the current ponter values
 *
 * Input:           destAddr - Destination address in the memory to copy to.  If it equals -1,
 *                     the current write pointer will be used.
 *                  sourceAddr - Source address to read from.  If it equals -1,
 *                     the current read pointer will be used.
 *                  len - number of uint8_ts to copy
 *
 * Output:          None
 *
 * Side Effects:    None
 *
 * Overview:        Copies data from one address to another within the Ethernet memory.
 *                  Overlapped memory regions are allowed only if the destination start address
 *                  is at a lower memory address than the source address.
 *
 * Note:            The addresses do not have to be aligned.
 *****************************************************************************/
static void mrf24w_MACMemCopyAsync(TCPIP_MAC_HANDLE hMac, PTR_BASE destAddr, PTR_BASE sourceAddr, uint16_t len)
{
    MRF24W_MACMemCopyAsync(destAddr, sourceAddr, len);
}

/******************************************************************************
 * Function:        void mrf24w_MACIsMemCopyDone(TCPIP_MAC_HANDLE hMac )
 *
 * PreCondition:    None
 *
 * Input:           None
 *
 * Output:          true
 *
 * Side Effects:    None
 *
 * Overview:        Since there's no copy initiated by the DMA, the function returns always true for now.
 *
 * Note:            None
 *****************************************************************************/
static bool mrf24w_MACIsMemCopyDone(TCPIP_MAC_HANDLE hMac )
{
	return MRF24W_MACIsMemCopyDone();
}


/******************************************************************************
 * Function:        uint16_t mrf24w_MACCalcRxChecksum(TCPIP_MAC_HANDLE hMac, uint16_t offset, uint16_t len)
 *
 * PreCondition:    None
 *
 * Input:           offset  - Number of uint8_ts beyond the beginning of the
 *                          Ethernet data (first uint8_t after the type field)
 *                          where the checksum should begin
 *                  len     - Total number of uint8_ts to include in the checksum
 *
 * Output:          16-bit checksum as defined by RFC 793.
 *
 * Side Effects:    None
 *
 * Overview:        This function performs a checksum calculation in the current receive buffer.
 *
 * Note:            None
 *****************************************************************************/
static uint16_t mrf24w_MACCalcRxChecksum(TCPIP_MAC_HANDLE hMac, uint16_t offset, uint16_t len)
{
	return MRF24W_MACCalcRxChecksum(hMac, offset, len);
}

/******************************************************************************
 * Function:        uint16_t mrf24w_MACCalcIPBufferChecksum(TCPIP_MAC_HANDLE hMac, uint16_t len)
 *
 * PreCondition:    Read buffer pointer set to starting of checksum data
 *
 * Input:           len: Total number of uint8_ts to calculate the checksum over.
 *
 * Output:          16-bit checksum as defined by RFC 793
 *
 * Side Effects:    None
 *
 * Overview:        This function performs a checksum calculation of the buffer
 *                  pointed by the current value of the read pointer.
 *
 * Note:            None
 *****************************************************************************/
static uint16_t mrf24w_MACCalcIPBufferChecksum(TCPIP_MAC_HANDLE hMac, uint16_t len)
{
	return MRF24W_MACCalcIPBufferChecksum(hMac, len);
}


	// verify the link status

/****************************************************************************
 * Function:        mrf24w_MACCheckLink
 *
 * PreCondition:    None
 *
 * Input:           None
 *
 * Output:          true if the link is up
 *                  false otherwise
 *
 * Side Effects:    None
 *
 * Overview:        This function periodically checks the link status
 *                  performing the MAC reconfiguration if the link went up
 *                  after being down.
 *
 * Note:            If auto negotiation is enabled the MAC we may have to be reconfigured.
 *****************************************************************************/
static bool mrf24w_MACCheckLink(TCPIP_MAC_HANDLE hMac)
{
	return MRF24W_MACIsLinked();
}


static void mrf24w_MACSetRXHashTableEntry(TCPIP_MAC_HANDLE hMac, MAC_ADDR DestMACAddr)
{
}

static void mrf24w_MACPowerDown(TCPIP_MAC_HANDLE hMac)
{
    MRF24W_MACPowerDown();
}

static void mrf24w_MACEDPowerDown(TCPIP_MAC_HANDLE hMac)
{
}

static void mrf24w_MACPowerUp(TCPIP_MAC_HANDLE hMac)
{
    MRF24W_MACPowerUp();
}

static    void mrf24w_MACProcess(TCPIP_MAC_HANDLE hMac)
{
    MRF24W_MACProcess(hMac);
}

static bool mrf24w_MACRxFilter(TCPIP_MAC_HANDLE hMac, NODE_INFO* pPkt)
{
    return MRF24W_MACRxFilter(hMac, pPkt);
}

static void mrf24w_MACConnect(TCPIP_MAC_HANDLE hMac)
{
    WF_Connect(hMac);
}


#if defined(TCPIP_STACK_USE_EVENT_NOTIFICATION)
static TCPIP_MAC_EVENT_RESULT mrf24w_MACEventSetNotifyEvents(TCPIP_MAC_HANDLE hMac, TCPIP_MAC_EVENT_GROUP tcpEvGroup, TCPIP_EVENT tcpipEvents)
{
    return MRF24W_MACEventSetNotifyEvents(hMac, tcpEvGroup, tcpipEvents);
}

static TCPIP_MAC_EVENT_RESULT mrf24w_MACEventClearNotifyEvents(TCPIP_MAC_HANDLE hMac, TCPIP_MAC_EVENT_GROUP tcpEvGroup, TCPIP_EVENT tcpipEvents)
{
    return MRF24W_MACEventClearNotifyEvents(hMac, tcpEvGroup, tcpipEvents);
}

static TCPIP_MAC_EVENT_RESULT mrf24w_MACEventAck(TCPIP_MAC_HANDLE hMac, TCPIP_MAC_EVENT_GROUP tcpEvGroup, TCPIP_EVENT tcpipEvents)
{
    return MRF24W_MACEventAck(hMac, tcpEvGroup, tcpipEvents);
}
    
static TCPIP_EVENT mrf24w_MACEventGetPending(TCPIP_MAC_HANDLE hMac, TCPIP_MAC_EVENT_GROUP tcpEvGroup)
{
    return MRF24W_MACEventGetPending(hMac, tcpEvGroup);
}

static TCPIP_MAC_EVENT_RESULT mrf24w_MACEventSetNotifyHandler(TCPIP_MAC_HANDLE hMac, TCPIP_MAC_EVENT_GROUP tcpEvGroup, pMacEventF eventHandler, void* hParam)
{
    return MRF24W_MACEventSetNotifyHandler(hMac, tcpEvGroup, eventHandler, hParam);
}

#endif  // (TCPIP_STACK_USE_EVENT_NOTIFICATION)


static void mrf24w_MACDeinitialize(MRF24W_MAC_DCPT* pDcpt )
{
    MRF24W_ChipReset();  // will stop the MR24W from transmitting
    
#if defined(TCPIP_STACK_USE_EVENT_NOTIFICATION)
    MRF24W_MACEventDeInit(pDcpt);    
#endif    

}
#endif  // defined(__PIC32MX__) && defined(TCPIP_IF_MRF24W)


