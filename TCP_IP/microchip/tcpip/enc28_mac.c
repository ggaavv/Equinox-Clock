/*******************************************************************************
  ENC28J60 Driver Medium Access Control (MAC) 

  Summary:
    Layer Module for Microchip TCP/IP Stack
    
  Description:
    -PIC32 implementation for multiple MAC support
*******************************************************************************/

/*******************************************************************************
FileName:   enc28_mac.c
Copyright ?2012 released Microchip Technology Inc.  All rights
reserved.

Microchip licenses to you the right to use, modify, copy and distribute
Software only when embedded on a Microchip microcontroller or digital signal
controller that is integrated into your product or third party product
(pursuant to the sublicense terms in the accompanying license agreement).

You should refer to the license agreement accompanying this Software for
additional information regarding your rights and obligations.

SOFTWARE AND DOCUMENTATION ARE PROVIDED “AS IS?WITHOUT WARRANTY OF ANY KIND,
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
#include "tcpip/enc28j60.h"
#include "enc28_mac.h"

// Compile only for ENC28J60 MAC interface
#if defined(TCPIP_IF_ENC28J60)

#if SSL_SAVE_CONTEXT_IN_PIC_RAM
// SSL buffers
static unsigned char		_SSlBuffer[RESERVED_SSL_MEMORY];
#endif  // SSL_SAVE_CONTEXT_IN_PIC_RAM


// Function proto declaration
static TCPIP_MAC_RES       ENC28_MACClose(TCPIP_MAC_HANDLE hMac);
static bool                ENC28_MACIsLinked(TCPIP_MAC_HANDLE hMac);
static int                 ENC28_MACGetHeader(TCPIP_MAC_HANDLE hMac, MAC_ADDR *remote, uint16_t* type);
static void                ENC28_MACSetReadPtrInRx(TCPIP_MAC_HANDLE hMac, uint16_t offset);
static PTR_BASE            ENC28_MACSetWritePtr(TCPIP_MAC_HANDLE hMac, PTR_BASE address);
static PTR_BASE            ENC28_MACSetReadPtr(TCPIP_MAC_HANDLE hMac, PTR_BASE address);
static uint8_t                ENC28_MACGet(TCPIP_MAC_HANDLE hMac);
static uint16_t              ENC28_MACGetArray(TCPIP_MAC_HANDLE hMac, uint8_t *val, uint16_t len);
static void                ENC28_MACDiscardRx(TCPIP_MAC_HANDLE hMac);
static uint16_t              ENC28_MACGetFreeRxSize(TCPIP_MAC_HANDLE hMac);
static void                ENC28_MACMemCopyAsync(TCPIP_MAC_HANDLE hMac, PTR_BASE destAddr, PTR_BASE sourceAddr, uint16_t len);
static bool                ENC28_MACIsMemCopyDone(TCPIP_MAC_HANDLE hMac);
static void                ENC28_MACPutHeader(TCPIP_MAC_HANDLE hMac, MAC_ADDR *remote, uint16_t type, uint16_t dataLen);
static bool                ENC28_MACIsTxReady(TCPIP_MAC_HANDLE hMac);
static void                ENC28_MACPut(TCPIP_MAC_HANDLE hMac, uint8_t val);
static void                ENC28_MACPutArray(TCPIP_MAC_HANDLE hMac, uint8_t *val, uint16_t len);
static void                ENC28_MACFlush(TCPIP_MAC_HANDLE hMac);
static bool                ENC28_MACCheckLink(TCPIP_MAC_HANDLE hMac);
static PTR_BASE            ENC28_MACGetTxBaseAddr(TCPIP_MAC_HANDLE hMac);
static PTR_BASE            ENC28_MACGetHttpBaseAddr(TCPIP_MAC_HANDLE hMac);
static PTR_BASE            ENC28_MACGetSslBaseAddr(TCPIP_MAC_HANDLE hMac);
static uint16_t              ENC28_MACGetRxSize(TCPIP_MAC_HANDLE hMac);
static uint16_t              ENC28_MACGetRamSize(TCPIP_MAC_HANDLE hMac);
static uint16_t              ENC28_MACGetTxBuffSize(TCPIP_MAC_HANDLE hMac);
static PTR_BASE 		   ENC28_MACGetReadPtrInRx(TCPIP_MAC_HANDLE hMac);
static uint16_t              ENC28_MACCalcRxChecksum(TCPIP_MAC_HANDLE hMac, uint16_t offset, uint16_t len);
static uint16_t              ENC28_MACCalcIPBufferChecksum(TCPIP_MAC_HANDLE hMac, uint16_t len);
static void                ENC28_MACSetRXHashTableEntry(TCPIP_MAC_HANDLE hMac, MAC_ADDR DestMACAddr);
static void                ENC28_MACPowerDown(TCPIP_MAC_HANDLE hMac);
static void	               ENC28_MACEDPowerDown(TCPIP_MAC_HANDLE hMac);
static void 	           ENC28_MACPowerUp(TCPIP_MAC_HANDLE hMac);
static void 	           ENC28_MACProcess(TCPIP_MAC_HANDLE hMac);
static bool                ENC28_MACRxFilter(TCPIP_MAC_HANDLE hMac, NODE_INFO* pNode);
static void                ENC28_MACConnect(TCPIP_MAC_HANDLE hMac);
#if defined(TCPIP_STACK_USE_EVENT_NOTIFICATION)
static TCPIP_MAC_EVENT_RESULT ENC28_MACEventSetNotifyEvents(TCPIP_MAC_HANDLE hMac, TCPIP_MAC_EVENT_GROUP tcpEvGroup, TCPIP_EVENT tcpipEvents);
static TCPIP_MAC_EVENT_RESULT ENC28_MACEventClearNotifyEvents(TCPIP_MAC_HANDLE hMac, TCPIP_MAC_EVENT_GROUP tcpEvGroup, TCPIP_EVENT tcpipEvents);
static TCPIP_MAC_EVENT_RESULT ENC28_MACEventAck(TCPIP_MAC_HANDLE hMac, TCPIP_MAC_EVENT_GROUP tcpEvGroup, TCPIP_EVENT tcpipEvents);
static TCPIP_EVENT        ENC28_MACEventGetPending(TCPIP_MAC_HANDLE hMac, TCPIP_MAC_EVENT_GROUP tcpEvGroup);
static TCPIP_MAC_EVENT_RESULT ENC28_MACEventSetNotifyHandler(TCPIP_MAC_HANDLE hMac, TCPIP_MAC_EVENT_GROUP tcpEvGroup, pMacEventF eventHandler, void* hParam);
#endif  // (TCPIP_STACK_USE_EVENT_NOTIFICATION)

static void     _ENC28_Deinitialize(ENC28_MAC_DCPT* pDcpt );


// the PIC32 ENC28J60 MAC descriptor
// no support for multiple instances
static const TCPIP_MAC_OBJECT _pic32_enc28_mac_obj = 
{
    ENC28_MACClose,
    ENC28_MACIsLinked,
    ENC28_MACGetHeader,
    ENC28_MACSetReadPtrInRx,
    ENC28_MACSetWritePtr,
    ENC28_MACGetReadPtrInRx,
    0,          // ENC28_MACSetBaseReadPtr
    ENC28_MACSetReadPtr,
    ENC28_MACGet,
    ENC28_MACGetArray,
    ENC28_MACDiscardRx,
    ENC28_MACGetFreeRxSize,
    ENC28_MACMemCopyAsync,
    ENC28_MACIsMemCopyDone,
    ENC28_MACPutHeader,
    ENC28_MACIsTxReady,
    ENC28_MACPut,
    ENC28_MACPutArray,
    ENC28_MACFlush,
    ENC28_MACCheckLink,
    ENC28_MACGetTxBaseAddr,
    ENC28_MACGetHttpBaseAddr,
    ENC28_MACGetSslBaseAddr,
    ENC28_MACGetRxSize,
    ENC28_MACGetRamSize,
    ENC28_MACGetTxBuffSize,
    ENC28_MACCalcRxChecksum,
    ENC28_MACCalcIPBufferChecksum,
    ENC28_MACSetRXHashTableEntry,
    ENC28_MACPowerDown,
    ENC28_MACEDPowerDown,
    ENC28_MACPowerUp,
    ENC28_MACProcess,
    ENC28_MACRxFilter,
    ENC28_MACConnect,
#if defined(TCPIP_STACK_USE_EVENT_NOTIFICATION)
    ENC28_MACEventSetNotifyEvents,
    ENC28_MACEventClearNotifyEvents,
    ENC28_MACEventAck,
    ENC28_MACEventGetPending,
    ENC28_MACEventSetNotifyHandler,
#endif  // (TCPIP_STACK_USE_EVENT_NOTIFICATION)
};

// only one hardware instance for now!
static ENC28_MAC_DCPT _pic32_enc28_mac_dcpt[1] = 
{
    {
        &_pic32_enc28_mac_obj,
        // specific ENC28 MAC data 
        0,                                  // pNetIf
        0,									// 0 -link down
        0,                                  // isOpen
    }
};

////////////////////////////////////////////////////////////////////////////////
/*******************************************************************************
 *
 * interface functions
 *
*******************************************************************************/
////////////////////////////////////////////////////////////////////////////////

/*******************************************************************************
  Function:
	ENC28_MACInitialize

  Summary:
	This function initializes the NIC, the MAC and the associated PHY.

  Description:
    This function initializes the NIC, the MAC and the associated PHY. It should
    be called to be able to schedule any Eth transmit or receive operation.

  Precondition:
	None

  Parameters:
	stackData - ponter to stack initialization data structure
	initData - ponter to MAC initialization data structure

  Returns:
  	TCPIP_MAC_RES
  	
  Remarks:
	Only one client per MAC supported..
*******************************************************************************/

TCPIP_MAC_RES ENC28_MACInitialize(const TCPIP_STACK_MODULE_CTRL* const stackData,
                                    const TCPIP_MAC_MODULE_GONFIG * const initData)
{
	ENC28_MAC_DCPT* pDcpt;
	TCPIP_MAC_RES		res;
	
	if(stackData->pNetIf->macId != TCPIP_MAC_ID_ENCJ60)
	{
		return TCPIP_MAC_RES_TYPE_ERR;		// no other type supported
	}

	pDcpt = _pic32_enc28_mac_dcpt + 0; // no other instance supported
	if(pDcpt->isOpen != 0)
	{ 
		return TCPIP_MAC_RES_IS_BUSY;	  // have a client connected
	}

	pDcpt->pNetIf = stackData->pNetIf;
	res =  ENC28J60_MACInit(pDcpt->pNetIf);

	if(res !=  TCPIP_MAC_RES_OK)
	{
		_ENC28_Deinitialize(pDcpt);
	}

	return res;
}

/*******************************************************************************
  Function:
	ENC28_MACDeinitialize

  Summary:
	This function de-initializes the Eth MAC controller.

  Description:
    This function de-initializes the Eth MAC controller.

  Precondition:
	None

  Parameters:
	stackData    - standard interface init structure

  Returns:
  	TCPIP_MAC_RES
  	
  Remarks:
	None
*******************************************************************************/

TCPIP_MAC_RES ENC28_MACDeinitialize(const TCPIP_STACK_MODULE_CTRL* const stackData )
{
	if(stackData->pNetIf->macId != TCPIP_MAC_ID_ENCJ60)
	{
		return TCPIP_MAC_RES_TYPE_ERR;		// no other type supported
	}

	_ENC28_Deinitialize(_pic32_enc28_mac_dcpt + 0); // no other instance supported

	return TCPIP_MAC_RES_OK;
}

/*******************************************************************************
  Function:
	ENC28_MACOpen

  Summary:
	This function returns a client handle.

  Description:
    This function returns a client handle.

  Precondition:
	ENC28_MACInitialize has been called

  Parameters:
	macId    - standard MAC ID

  Returns:
  	TCPIP_MAC_RES
  	
  Remarks:
	Currently only one client is supported 
*******************************************************************************/

TCPIP_MAC_HANDLE ENC28_MACOpen( TCPIP_MAC_ID macId )
{    
	ENC28_MAC_DCPT* pMacD;
    TCPIP_MAC_HANDLE    hMac = 0;

    if(macId == TCPIP_MAC_ID_ENCJ60)
    {
        pMacD = _pic32_enc28_mac_dcpt + 0; // no other instance supported
        if((pMacD->isOpen) == 0)
        {   // only one client for now
            pMacD->isOpen = 1;
            hMac = pMacD;
        }
    }
   
    return hMac;
}

 /*******************************************************************************
  Function:
	ENC28_MACClose

  Summary:
	This function closes a client handle.

  Description:
    This function closes a client handle.

  Precondition:
	ENC28_MACInitialize has been called

  Parameters:
	hMac - MAC handle

  Returns:
  	TCPIP_MAC_RES
  	
  Remarks:
	Currently only one client is supported 
*******************************************************************************/
static TCPIP_MAC_RES ENC28_MACClose( TCPIP_MAC_HANDLE hMac )
{
	 ENC28_MAC_DCPT* pMacD = (ENC28_MAC_DCPT*)hMac;
	 
	 pMacD->isOpen = 0;
	 
	 return TCPIP_MAC_RES_OK;
}

 /*******************************************************************************
  Function:
	ENC28_MACIsLinked

  Summary:
	This function checks the link status

  Description:
    This function checks the link status

  Precondition:
	ENC28_MACInitialize has been called

  Parameters:
	hMac - MAC handle

  Returns:
  	true if link is up, false otherwise
  	
  Remarks:
	none 
*******************************************************************************/
static bool ENC28_MACIsLinked(TCPIP_MAC_HANDLE hMac)
{
	 
	 ENC28_MAC_DCPT*	 pMacD = (ENC28_MAC_DCPT*)hMac;
	 return (pMacD->_linkPrev==1);
}
	
/*******************************************************************************
  Function:
	ENC28_MACGetTxBaseAddr

  Summary:
	This function returns the address of the current TX buffer

  Description:
	This function returns the address of the current TX buffer

  Precondition:
	None

  Parameters:
	hMac - MAC handle
	
  Returns:
	TX buffer base address
	
  Remarks:
	The returned value could be 0 if currently there's no available TX buffer. 
*******************************************************************************/

static PTR_BASE ENC28_MACGetTxBaseAddr(TCPIP_MAC_HANDLE hMac)
{
	return (PTR_BASE)BASE_TX_ADDR;
}

/*******************************************************************************
  Function:
	ENC28_MACGetHttpBaseAddr

  Summary:
	This function returns the address of the HTTP buffer.

  Description:
	This function returns the address of the HTTP buffer.

  Precondition:
	None

  Parameters:
	hMac - MAC handle
	
  Returns:
	HTTP buffer base address
	
  Remarks:
	 The HTTP buffer is a static one, always available. 
*******************************************************************************/

static PTR_BASE ENC28_MACGetHttpBaseAddr(TCPIP_MAC_HANDLE hMac)
{
	return (PTR_BASE)BASE_HTTPB_ADDR;
}

/*******************************************************************************
  Function:
	ENC28_MACGetSslBaseAddr

  Summary:
	This function returns the address of the SSL buffer.

  Description:
	This function returns the address of the SSL buffer.

  Precondition:
	None

  Parameters:
	hMac - MAC handle
	
  Returns:
	SSL buffer base address
	
  Remarks:
	 The SSL buffer is a static one, always available.  
*******************************************************************************/

static PTR_BASE ENC28_MACGetSslBaseAddr(TCPIP_MAC_HANDLE hMac)
{	
#if SSL_SAVE_CONTEXT_IN_PIC_RAM
	return (PTR_BASE)_SSlBuffer;
#else
	return (PTR_BASE)BASE_SSLB_ADDR;
#endif  // SSL_SAVE_CONTEXT_IN_PIC_RAM
}

/*******************************************************************************
  Function:
	ENC28_MACGetRxSize

  Summary:
	This function returns the size of the RX buffer.

  Description:
	This function returns the size of the RX buffer.

  Precondition:
	None

  Parameters:
	hMac - MAC handle
	
  Returns:
	RX size
	
  Remarks:
	None	
*******************************************************************************/

static uint16_t ENC28_MACGetRxSize(TCPIP_MAC_HANDLE hMac)
{
	return (uint16_t)RXSIZE;
}

/*******************************************************************************
  Function:
	ENC28_MACGetRamSize

  Summary:
	This function returns the size of the RAM buffer.

  Description:
	This function returns the size of the RAM buffer.

  Precondition:
	None

  Parameters:
	hMac - MAC handle
	
  Returns:
	RAM size
	
  Remarks:
	Not used, no RAM on the PIC32 ETH controller	
*******************************************************************************/

static uint16_t ENC28_MACGetRamSize(TCPIP_MAC_HANDLE hMac)
{
    return RAMSIZE; //EMAC_RX_DESCRIPTORS*EMAC_RX_BUFF_SIZE;
}

/*******************************************************************************
  Function:
	ENC28_MACGetTxBuffSize

  Summary:
	This function returns the size of the TXbuffer.

  Description:
	This function returns the size of the TX buffer.

  Precondition:
	None

  Parameters:
	hMac - MAC handle
	
  Returns:
	TX buffer size
	
  Remarks:
	None	
*******************************************************************************/

static uint16_t ENC28_MACGetTxBuffSize(TCPIP_MAC_HANDLE hMac)
{
    return RAMSIZE;
}

static PTR_BASE ENC28_MACGetReadPtrInRx(TCPIP_MAC_HANDLE hMac)
{
    return ENC28J60_MACGetReadPtrInRx(hMac);
}


////////////////////////////////////////////////////////////////////////////////
/*******************************************************************************
 *
 * TX functions
 *
*******************************************************************************/
////////////////////////////////////////////////////////////////////////////////

/*******************************************************************************
  Function:
	ENC28_MACSetWritePtr

  Summary:
	This function sets the new write pointer.

  Description:
	This function sets the new write pointer.

  Precondition:
	None

  Parameters:
	hMac - MAC handle
    address - new address
	
  Returns:
	TX buffer size
	
  Remarks:
	None	
*******************************************************************************/

static PTR_BASE ENC28_MACSetWritePtr(TCPIP_MAC_HANDLE hMac, PTR_BASE address)
{
	return ENC28J60_MACSetWritePtr(address);
}

/*******************************************************************************
  Function:
	bool ENC28_MACIsTxReady(TCPIP_MAC_HANDLE hMac )

  Summary:
	Checks if there is an available current TX buffer.

  Description:
	Checks if there is an available current TX buffer.
	
  Precondition:
	None

  Parameters:
	hMac - MAC handle
	
  Returns:
	true: If data can be inserted in the current TX buffer
	false: there is no free TX buffer
	
  Remarks:
	None	
*******************************************************************************/

static bool ENC28_MACIsTxReady(TCPIP_MAC_HANDLE hMac)
{
	return ENC28J60_MACIsTxReady();
}

/*******************************************************************************
  Function:
	void ENC28_MACPut(TCPIP_MAC_HANDLE hMac, uint8_t val)

  Summary:
	Writes a byte to the current write location and increments the write pointer.
	
  Description:
	Writes a byte to the current write location and increments the write pointer.
	
  Precondition:
	None

  Parameters:
	hMac - MAC handle
	val - data to be written
	
  Returns:
	None
	
  Remarks:
	None	
*******************************************************************************/

static void ENC28_MACPut(TCPIP_MAC_HANDLE hMac, uint8_t val)
{
	ENC28J60_MACPut(val);
}

/*******************************************************************************
  Function:
	void ENC28_MACPut(TCPIP_MAC_HANDLE hMac, uint8_t val)

  Summary:
	Writes a buffer to the current write location and updates the write pointer.
	
  Description:
	Writes a buffer to the current write location and updates the write pointer.
	
  Precondition:
	None

  Parameters:
	hMac - MAC handle
	buff - buffer to be written
	len - buffer length
	
  Returns:
	None
	
  Remarks:
	None	
*******************************************************************************/

static void ENC28_MACPutArray(TCPIP_MAC_HANDLE hMac, uint8_t *val, uint16_t len)
{
	ENC28J60_MACPutArray(val, len);
}

/*******************************************************************************
  Function:
	void ENC28_MACPutHeader(TCPIP_MAC_HANDLE hMac, (MAC_ADDR *remote, uint16_t type, uint16_t dataLen)

  Summary:
	Sets the write pointer at the beginning of the current TX buffer
    and sets the ETH header and the frame length. Updates the write pointer
	
  Description:
	Sets the write pointer at the beginning of the current TX buffer
    and sets the ETH header and the frame length. Updates the write pointer
	
  Precondition:
	None

  Parameters:
	hMac - MAC handle
	remote - Pointer to memory which contains the destination MAC address (6 bytes)
    type - packet type: ETHERTYPE_IPV4/6, ETHERTYPE_ARP
    dataLen - ethernet frame payload
	
  Returns:
	None
	
  Remarks:
	Assumes there is an available TX buffer, i.e. PIC32MACIsTxReady() returned !0	
*******************************************************************************/

static void ENC28_MACPutHeader(TCPIP_MAC_HANDLE hMac, MAC_ADDR *remote, uint16_t type, uint16_t dataLen)
{
	ENC28_MAC_DCPT* pDcpt;

	pDcpt = _pic32_enc28_mac_dcpt + 0; // no other instance supported

	ENC28J60_MACPutHeader(pDcpt->pNetIf, remote, type, dataLen);
}

static void ENC28_MACFlush(TCPIP_MAC_HANDLE hMac)
{
	ENC28J60_MACFlush();
}

////////////////////////////////////////////////////////////////////////////////
/*******************************************************************************
 *
 * RX functions
 *
*******************************************************************************/
////////////////////////////////////////////////////////////////////////////////

/*******************************************************************************
  Function:
	void ENC28_MACDiscardRx(TCPIP_MAC_HANDLE hMac )

  Summary:
	Marks the last received packet (obtained using ENC28_MACGetHeader())as being 
	processed and frees the buffer memory associated with it. 
	It acknowledges the ETHC.
	
  Description:
	Marks the last received packet (obtained using ENC28_MACGetHeader())as being 
	processed and frees the buffer memory associated with it. 
	It acknowledges the ETHC.
	
  Precondition:
	None

  Parameters:
	hMac - MAC handle
	
  Returns:
	None
	
  Remarks:
	Is is safe to call this function multiple times between
    ENC28_MACGetHeader() calls.  Extra packets won't be thrown away 
    until ENC28_MACGetHeader() makes it available.	
*******************************************************************************/

static void ENC28_MACDiscardRx(TCPIP_MAC_HANDLE hMac)
{
	ENC28J60_MACDiscardRx();
}

/*******************************************************************************
  Function:
	int ENC28_MACGetHeader(TCPIP_MAC_HANDLE hMac, MAC_ADDR *remote, uint16_t* type)

  Summary:
	Checks if a new RX packet is available and returns the packet payload size
   (without the Ethernet frame header)
	
  Description:
	Checks if a new RX packet is available and returns the packet payload size
   (without the Ethernet frame header)
	
  Precondition:
	None

  Parameters:
	hMac - MAC handle
	*remote: Location to store the Source MAC address of the received frame.
	*type: Location of a uint16_t to store the constant ETHERTYPE_UNKNOWN, 
		   ETHERTYPE_IPVx, or ETHERTYPE_ARP, representing the contents of the 
		   Ethernet type field
	
  Returns:
	 !0: If a packet of this size is waiting in the RX buffer.  The remote, and 
		 type values are updated.
     0: If a packet was not pending.  remote and type are not changed.
	
  Remarks:
	Last packet is discarded if ENC28_MACDiscardRx() hasn't already been called;
	Sets the read pointer at the beginning of the new packet.
*******************************************************************************/

static int ENC28_MACGetHeader(TCPIP_MAC_HANDLE hMac, MAC_ADDR *remote, uint16_t* type)
{
	return ENC28J60_MACGetHeader(remote, type);
}
	
/*******************************************************************************
  Function:
	void ENC28_MACSetReadPtrInRx(TCPIP_MAC_HANDLE hMac, uint16_t offset)

  Summary:
	 The current read pointer is updated.  All calls to ENC28_MACGet() and 
	 ENC28_MACGetArray() will use these new values.
	
  Description:
	 The current read pointer is updated.  All calls to ENC28_MACGet() and 
	 ENC28_MACGetArray() will use these new values.
	
  Precondition:
	A packet has been obtained by calling ENC28_MACGetHeader() and getting a true
	result.

  Parameters:
	hMac - MAC handle
	offset: uint16_t specifying how many bytes beyond the Ethernet header's type
	        field to relocate the read pointer.
	
  Returns:
	None
	
  Remarks:
	Last packet is discarded if ENC28_MACDiscardRx() hasn't already been called;
	Sets the read pointer at the beginning of the new packet.
*******************************************************************************/

static void ENC28_MACSetReadPtrInRx(TCPIP_MAC_HANDLE hMac, uint16_t offset)
{
	ENC28J60_MACSetReadPtrInRx(offset);
}

/*******************************************************************************
  Function:
	ENC28_MACSetReadPtr

  Summary:
	 This function sets the new read pointer value.
	
  Description:
	 This function sets the new read pointer value.
	
  Precondition:
	None

  Parameters:
	hMac - MAC handle
	address - new address to be used
	
  Returns:
	old read pointer
	
  Remarks:
	None
*******************************************************************************/

static PTR_BASE ENC28_MACSetReadPtr(TCPIP_MAC_HANDLE hMac, PTR_BASE address)
{
	return ENC28J60_MACSetReadPtr(address);
}

/*******************************************************************************
  Function:
	ENC28_MACGet(TCPIP_MAC_HANDLE hMac )

  Summary:
	 ENC28_MACGet returns the byte pointed to by the current read pointer location 
	 and increments the read pointer.
	
  Description:
	 ENC28_MACGet returns the byte pointed to by the current read pointer location 
	 and increments the read pointer.
	
  Precondition:
	A valid packet should have been obtained or the read pointer properly set.

  Parameters:
	hMac - MAC handle
	
  Returns:
	Byte read from the current read pointer location
	
  Remarks:
	None
*******************************************************************************/

static uint8_t ENC28_MACGet(TCPIP_MAC_HANDLE hMac)
{
	return ENC28J60_MACGet();
}

/*******************************************************************************
  Function:
	ENC28_MACGetArray(TCPIP_MAC_HANDLE hMac, uint8_t *address, uint16_t len)

  Summary:
	 Copies data in the supplied buffer.
	
  Description:
	 Copies data in the supplied buffer.
	
  Precondition:
	A valid packet should have been obtained or the read pointer properly set.

  Parameters:
	hMac - MAC handle
	*val: Pointer to storage location
    len:  Number of bytes to read from the data buffer.
	
  Returns:
	Byte read from the current read pointer location
	
  Remarks:
	The read pointer is updated
*******************************************************************************/

static uint16_t ENC28_MACGetArray(TCPIP_MAC_HANDLE hMac, uint8_t *val, uint16_t len)
{
	return ENC28J60_MACGetArray(val, len);
}

/*******************************************************************************
  Function:
	uint16_t ENC28_MACGetFreeRxSize(TCPIP_MAC_HANDLE hMac )

  Summary:
	 An estimate of how much RX buffer space is free at the present time.
	
  Description:
	 An estimate of how much RX buffer space is free at the present time.
	
  Precondition:
	A valid packet should have been obtained or the read pointer properly set.

  Parameters:
	hMac - MAC handle
	
  Returns:
	Estimated free RX buffer space at the present time.
	
  Remarks:
	None
*******************************************************************************/

static uint16_t ENC28_MACGetFreeRxSize(TCPIP_MAC_HANDLE hMac)
{
	return ENC28J60_MACGetFreeRxSize();
}

/*******************************************************************************
  Function:
	void ENC28_MACMemCopyAsync(TCPIP_MAC_HANDLE hMac, PTR_BASE destAddr, PTR_BASE sourceAddr, uint16_t len)

  Summary:
	 Copies data from one address to another within the Ethernet memory.
	
  Description:
	 Copies data from one address to another within the Ethernet memory.
	 Overlapped memory regions are allowed only if the destination start address
	 is at a lower memory address than the source address.
	
  Precondition:
	Read and write pointers properly set if using the current ponter values

  Parameters:
	hMac - MAC handle
	destAddr - Destination address in the memory to copy to.  If it equals -1,
              the current write pointer will be used.
	sourceAddr - Source address to read from.  If it equals -1,
              the current read pointer will be used.
 	len - number of bytes to copy
	
  Returns:
	None
	
  Remarks:
	The addresses do not have to be aligned.
*******************************************************************************/

static void ENC28_MACMemCopyAsync(TCPIP_MAC_HANDLE hMac, PTR_BASE destAddr, PTR_BASE sourceAddr, uint16_t len)
{
	ENC28J60_MACMemCopyAsync(destAddr, sourceAddr, len);
}

/*******************************************************************************
  Function:
	void ENC28_MACIsMemCopyDone(TCPIP_MAC_HANDLE hMac )

  Summary:
	 Since there's no copy initiated by the DMA, the function returns always true for now.
	
  Description:
	 Since there's no copy initiated by the DMA, the function returns always true for now.
	
  Precondition:
	None

  Parameters:
	hMac - MAC handle
	
  Returns:
	true
	
  Remarks:
	None
*******************************************************************************/

static bool ENC28_MACIsMemCopyDone(TCPIP_MAC_HANDLE hMac)
{
	return ENC28J60_MACIsMemCopyDone();
}

/*******************************************************************************
  Function:
	ENC28_MACCalcIPBufferChecksum(TCPIP_MAC_HANDLE hMac, uint16_t len)

  Summary:
	 This function performs a checksum calculation of the buffer pointed by the 
	 current value of the read pointer.
	
  Description:
	 This function performs a checksum calculation of the buffer pointed by the 
	 current value of the read pointer.
	
  Precondition:
	Read buffer pointer set to starting of checksum data

  Parameters:
	hMac - MAC handle
	len: Total number of bytes to calculate the checksum over.
	
  Returns:
	16-bit checksum as defined by RFC 793
	
  Remarks:
	None
*******************************************************************************/

static uint16_t ENC28_MACCalcIPBufferChecksum(TCPIP_MAC_HANDLE hMac, uint16_t len)
{
	return ENC28J60_CalcIPBufferChecksum(hMac, len);
}

/*******************************************************************************
  Function:
	ENC28_MACCalcRxChecksum(TCPIP_MAC_HANDLE hMac, uint16_t offset, uint16_t len)

  Summary:
	 This function performs a checksum calculation in the current receive buffer.
	
  Description:
	 This function performs a checksum calculation in the current receive buffer.
	
  Precondition:
	Read buffer pointer set to starting of checksum data

  Parameters:
	hMac - MAC handle
	offset  - Number of bytes beyond the beginning of the Ethernet data 
			(first byte after the type field)nwhere the checksum should begin
    len     - Total number of bytes to include in the checksum
	
  Returns:
	16-bit checksum as defined by RFC 793.
	
  Remarks:
	None
*******************************************************************************/

static uint16_t ENC28_MACCalcRxChecksum(TCPIP_MAC_HANDLE hMac, uint16_t offset, uint16_t len)
{
	return ENC28J60_MACCalcRxChecksum(hMac, offset, len);
}

/*******************************************************************************
  Function:
	bool ENC28_MACCheckLink(TCPIP_MAC_HANDLE hMac)

  Summary:
	This function periodically checks the link statusperforming the MAC 
	reconfiguration if the link went up after being down.
	
  Description:
	This function periodically checks the link statusperforming the MAC 
	reconfiguration if the link went up after being down.
	
  Precondition:
	None
	
  Parameters:
	hMac - MAC handle
	
  Returns:
	None
	
  Remarks:
	If auto negotiation is enabled the MAC we may have to be reconfigured.
*******************************************************************************/

static bool ENC28_MACCheckLink(TCPIP_MAC_HANDLE hMac)
{
    int	    linkCurr;
    ENC28_MAC_DCPT* pMacD = (ENC28_MAC_DCPT*)hMac;

    linkCurr = ENC28J60_MACCheckLink();
    if(pMacD->_linkPrev != linkCurr)
    {   // PHY state changed 
		if (linkCurr == 1)
        {
		}

        // update the new stat
        pMacD->_linkPrev = linkCurr;
    }
    // else same old state
    // 
    return (linkCurr==1);

}

/*******************************************************************************
  Function:
	void ENC28_MACSetRXHashTableEntry(TCPIP_MAC_HANDLE hMac, MAC_ADDR DestMACAddr)

  Summary:
	Calculates a CRC-32 using polynomial 0x4C11DB7 and then,using bits 28:23 of
	the CRC, sets the appropriate bit in the ETHHT0-ETHHT1 registers.
	
  Description:
	Calculates a CRC-32 using polynomial 0x4C11DB7 and then,using bits 28:23 of
	the CRC, sets the appropriate bit in the ETHHT0-ETHHT1 registers.
	
  Precondition:
	MACInitialize() should have been called.
	
  Parameters:
	hMac - MAC handle
	DestMACAddr: 6 byte group destination MAC address to allow through the Hash
			Table Filter.  If DestMACAddr is set to 00-00-00-00-00-00, then the 
			hash table will be cleared of all entries and the filter will be 
			disabled.
	
  Returns:
	Sets the appropriate bit in the ETHHT0/1 registers to allow packets sent to 
	DestMACAddr to be received and enabled the Hash Table receive filter (if not 
	already).
	
  Remarks:
	This code is commented out to save code space on systems that do not need 
	this function.  Change the "#if TCPIP_STACK_USE_ZEROCONF_MDNS_SD" line to "#if 1" 
	to uncomment it, assuming you aren't using the Zeroconf module, which 
	requires mutlicast support and enables this function automatically.
    There is no way to individually unset destination MAC addresses from the 
    hash table since it is possible to have a hash collision and therefore 
    multiple MAC addresses relying on the same hash table bit.  The stack would
    have to individually store each 6 byte MAC address to support this feature, 
    which would waste a lot of RAM and be unnecessary in most applications.  As 
    a simple compromise,you can call ENC28_MACSetRXHashTableEntry() using a 
    00-00-00-00-00-00 destination MAC address, which will clear the entire hash 
    table and disable the hash table filter. This will allow you to then readd 
    the necessary destination addresses.
*******************************************************************************/

static void ENC28_MACSetRXHashTableEntry(TCPIP_MAC_HANDLE hMac, MAC_ADDR DestMACAddr)
{
#if defined(TCPIP_STACK_USE_ZEROCONF_MDNS_SD)
	ENC28J60_SetRXHashTableEntry(DestMACAddr);
#endif
}
static void ENC28_MACPowerDown(TCPIP_MAC_HANDLE hMac)
{
	ENC28J60_MACPowerDown();
}
static void	ENC28_MACEDPowerDown(TCPIP_MAC_HANDLE hMac)
{
}
static void ENC28_MACPowerUp(TCPIP_MAC_HANDLE hMac)
{
	ENC28J60_ACPowerUp();
}
static void ENC28_MACProcess(TCPIP_MAC_HANDLE hMac)
{
	// no extra processing needed for PIC32 internal MAC
}
static bool ENC28_MACRxFilter(TCPIP_MAC_HANDLE hMac, NODE_INFO* pNode)
{
	return false;	// the wired interface does not filter out packets at this level

}
static void ENC28_MACConnect(TCPIP_MAC_HANDLE hMac)
{
	// no connect function for the wired interface
}

#if defined(TCPIP_STACK_USE_EVENT_NOTIFICATION)
static TCPIP_MAC_EVENT_RESULT ENC28_MACEventSetNotifyEvents(TCPIP_MAC_HANDLE hMac, TCPIP_MAC_EVENT_GROUP tcpEvGroup, TCPIP_EVENT tcpipEvents)
{
}
static TCPIP_MAC_EVENT_RESULT ENC28_MACEventClearNotifyEvents(TCPIP_MAC_HANDLE hMac, TCPIP_MAC_EVENT_GROUP tcpEvGroup, TCPIP_EVENT tcpipEvents)
{
}
static TCPIP_MAC_EVENT_RESULT ENC28_MACEventAck(TCPIP_MAC_HANDLE hMac, TCPIP_MAC_EVENT_GROUP tcpEvGroup, TCPIP_EVENT tcpipEvents)
{
}
static TCPIP_EVENT        ENC28_MACEventGetPending(TCPIP_MAC_HANDLE hMac, TCPIP_MAC_EVENT_GROUP tcpEvGroup)
{
}
static TCPIP_MAC_EVENT_RESULT ENC28_MACEventSetNotifyHandler(TCPIP_MAC_HANDLE hMac, TCPIP_MAC_EVENT_GROUP tcpEvGroup, pMacEventF eventHandler, void* hParam)
{
}
#endif  // (TCPIP_STACK_USE_EVENT_NOTIFICATION)

static void _ENC28_Deinitialize(ENC28_MAC_DCPT* pDcpt )
{
}

#endif

