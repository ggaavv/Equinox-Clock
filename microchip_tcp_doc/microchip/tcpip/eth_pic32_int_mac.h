/*******************************************************************************
  MAC Header Module (Microchip PIC32MX5-7) Interface Definition for 
  Microchip TCP/IP Stack

  Summary:
    This file contains the internal PIC32 MAC Interface definitions for 
    the TCPIP PIC32 MAC implementation.
    
  Description:
*******************************************************************************/

/*******************************************************************************
FileName:   eth_pic32_int_mac.h
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

//DOM-IGNORE-END

#ifndef _ETH_PIC32_INT_MAC_H_ 
#define _ETH_PIC32_INT_MAC_H_ 


typedef struct
{
	int		        txBusy;			// busy flag
	unsigned int	dataBuff[0];    // dataBuff[_TxBuffSize];// tx data buffer
}sEthTxDcpt;	// TX buffer descriptor


#if defined(TCPIP_STACK_USE_EVENT_NOTIFICATION)
typedef struct
{
    unsigned int            _TcpGroupEventsMask;        // masking of an OR-ed event to it's own group
    TCPIP_EVENT             _TcpEnabledEvents;          // group enabled notification events
    volatile TCPIP_EVENT    _TcpPendingEvents;          // group notification events that are set, waiting to be re-acknowledged
    eEthEvents              _EthEnabledEvents;          // copy in eEthEvents space
    volatile eEthEvents     _EthPendingEvents;          // copy in eEthEvents space
    pMacEventF              _TcpNotifyFnc;              // group notification handler
    void*                   _TcpNotifyParam;            // notification parameter
}PIC32_EV_GROUP_DCPT;   // event descriptor per group
#endif  // defined(TCPIP_STACK_USE_EVENT_NOTIFICATION)

// flags used by the PIC32 embedded MAC driver
typedef enum
{
    PIC32_MAC_FLAG_OPEN     = 0x01,     // the corresponding MAC is opened

    // add another flags here
}PIC32_EMB_MAC_FLAGS;

// dynamic data needed for the embedded PIC32 MAC
typedef struct
{
    // default initialized data first
    PIC32_EMB_MAC_FLAGS _macFlags;          // corresponding MAC flags
    int                 _macIx;             // index of the MAC, for multiple MAC's support
    int		        	_nTxDescriptors;	// number of active TX descriptors
    unsigned short      _TxBuffSize;        // size of the TX buffer
    int		        	_nRxDescriptors;	// number of active RX descriptors
    unsigned short      _RxBuffSize;        // size of each RX buffer 16 multiple
    
    // TX buffers
    sEthTxDcpt*	        _TxDescriptors;     // the dynamically allocated TX buffers
    sEthTxDcpt*	        _pTxCurrDcpt;		// the current TX buffer
    sEthTxDcpt*	        _pNextTxDcpt;	    // the next TX buffer to use
    sEthTxDcpt*	        _pLastTxDcpt;   	// the last TX buffer available

    unsigned short int	_TxCurrSize;						// the current TX buffer size

    // RX buffers
    unsigned char*		_RxBuffers;         // [_nRxDescriptors][_RxBuffSize];	// rx buffers for incoming data
                                            // each buffer has _RxBuffSize bytes
                                            // there are _nRxDescriptors buffers

    unsigned char*		_pRxCurrBuff;		// the current RX buffer
    unsigned short int	_RxCurrSize;		// the current RX buffer size



    // HTTP +SSL buffers
    unsigned char		_HttpSSlBuffer[RESERVED_HTTP_MEMORY+RESERVED_SSL_MEMORY];


    // general stuff
    unsigned char*		_CurrWrPtr;			// the current write pointer
    unsigned char*		_CurrRdPtr;			// the current read pointer

    // allocation functions
    void* (*_MallocCallback)( const void* h, size_t size);                   // malloc
    void* (*_CallocCallback)( const void* h, size_t nitems, size_t size) ;    // calloc
    void (*_FreeCallback)( const void* h, void* ptr) ;                        // free
    const void* _AllocH ;                   // allocation handle  


    // timing and link status maintenance
    SYS_TICK		_linkUpdTick;			// last tick value when the link update was started
    eEthLinkStat	_linkPrev;				// last value of the link status
    int			    _linkPresent;			// if connection to the PHY properly detected
    int			    _linkNegotiation;		// if an auto-negotiation is in effect

#if defined(TCPIP_STACK_USE_EVENT_NOTIFICATION)
    PLIB_INT_SOURCE _macIntSrc;             // this MAC interrupt source
    // TCPIP_MAC_EVGROUP_ALL, TCPIP_MAC_EVGROUP_RX, TCPIP_MAC_EVGROUP_TX
    PIC32_EV_GROUP_DCPT  _pic32_ev_group_dcpt[3];
#endif  // defined(TCPIP_STACK_USE_EVENT_NOTIFICATION)

    // debug: run time statistics
    int			_stackMgrRxOkPkts;
    int			_stackMgrRxBadPkts;
    int			_stackMgrRxDiscarded;
    int			_stackMgrTxNotReady;


    
}PIC32_EMB_MAC_DATA;


// the embedded PIC32 MAC descriptor
// support for multiple instances
typedef struct  
{
    const TCPIP_MAC_OBJECT* pObj;       // safe cast to TCPIP_MAC_DCPT   
    // specific PIC32 MAC data 
    PIC32_EMB_MAC_DATA  mData;
}PIC32_EMB_MAC_DCPT;


#if defined(TCPIP_STACK_USE_EVENT_NOTIFICATION)

#endif  // defined(TCPIP_STACK_USE_EVENT_NOTIFICATION)

#endif  //  _ETH_PIC32_INT_MAC_H_ 

