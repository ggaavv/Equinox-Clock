/*******************************************************************************
  tcpip PIC32 MAC events implementation

  Summary:
    
  Description:
*******************************************************************************/

/*******************************************************************************
FileName:   mac_events.c
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


#include "tcpip/spi_flash.h"
#include "tcpip/xeeprom.h"
#include "tcpip/tcpip_storage.h"

#if defined(_TCPIP_STACK_STORAGE_ENABLED)

// data saved into the NVM storage per interface
typedef struct __attribute__((__packed__))
{
    NET_CONFIG          entryData;          // the actual configuration data saved
                                            // differentiation per interfaces is done using the macId field.
}NVM_STORAGE_ENTRY;


// header structure for validating the NVM_STORAGE_ENTRY data storage in EEPROM/Flash
typedef struct __attribute__((__packed__))
{
	int16_t             hdrLength;          // Size of the header, in bytes
                                            // The length of the label is variable so is the header length
    uint16_t            hdrChecksum;        // header only checksum

	int16_t             nEntries;           // Number of configuration entries in the storage
	int16_t             entryLength;        // Length in bytes of a configuration entry
                                            // Note: all entries have to have the same size for
                                            // configuration to be valid!
	uint16_t            storageChecksum; 	// Checksum of the contents.
                                            // Validates the storage contents
#if _TCPIP_STORAGE_USER_LABEL_SIZE > 0
    uint8_t             storageLabel[_TCPIP_STORAGE_USER_LABEL_SIZE];    // size of the proprietary data field, bytes
#endif

   NVM_STORAGE_ENTRY    entry[0];           // actual storage
   
}NVM_STORAGE_HDR;

typedef void (*pStorageInit)(void);     // init function		
typedef	void (*pStorageReadArray)(uint32_t dwAddress, uint8_t *vData, uint16_t wLen);
typedef void (*pStorageBeginWrite)(uint32_t dwAddr);
typedef void (*pStorageWriteArray)(uint8_t *vData, uint16_t wLen);

typedef enum
{
    NVM_ST_FLAG_VALID   = 0x01,     // has been validated
    NVM_ST_FLAG_DIRTY   = 0x02,     // dirty, hasn't been flushed
    
}NVM_STORAGE_FLAGS;


// functions supported by a storage
typedef struct
{
    pStorageInit        init;
    pStorageReadArray   readArray;
    pStorageBeginWrite  beginWrite;
    pStorageWriteArray  writeArray;
}NVM_STORAGE_ACCESS_FNC;    



// identifies a storage medium
typedef struct
{
    // storage access functions 
    const NVM_STORAGE_ACCESS_FNC*  pAccFnc;
    // storage dynamic data
    int                     instCount;      // copy/instance count
    NVM_STORAGE_FLAGS       flags;          // valid, hasn't been flushed, etc.
    NVM_STORAGE_HDR         sHdr;           // actual data storage header
}NVM_STORAGE_DCPT;    
    

// access functions
#if defined(EEPROM_CS_TRIS)
// for now these are the only storage types supported
    static void _XEEInit(void);   // local configuration function

    static const NVM_STORAGE_ACCESS_FNC  _nvmAccessDcpt = 
    {
        _XEEInit,
        (pStorageReadArray)XEEReadArray,
        (pStorageBeginWrite)XEEBeginWrite,
        XEEWriteArray
    }; 
#elif defined(SPIFLASH_CS_TRIS)
    static void _SPIFlashInit(void);   // local configuration function		
    
    static const NVM_STORAGE_ACCESS_FNC  _nvmAccessDcpt = 
    {
        _SPIFlashInit,
        SPIFlashReadArray,
        SPIFlashBeginWrite,
        SPIFlashWriteArray
    }; 
#endif

// the one and only NVM storage descriptor
// for now we support only one
static NVM_STORAGE_DCPT  _nvmStorageDcpt = 
{
    &_nvmAccessDcpt,
    0,
}; 

static NVM_STORAGE_DCPT*  _hStorage = 0;    // the one and only storage handle  


// local functions
// 
static bool _TCPIPStorageVerifyLoad(NVM_STORAGE_DCPT* sDcpt);
static int _TCPIPStorageFindEntryIx(NVM_STORAGE_DCPT* sDcpt, const NET_CONFIG* pConfig, NVM_STORAGE_ENTRY* pwEntry);
static bool _TCPIPStorageWriteEntry(NVM_STORAGE_DCPT* sDcpt, int entryIx, const NET_CONFIG* pConfig, NVM_STORAGE_ENTRY* pwEntry);
static void _TCPIPStorageUpdateHdr(NVM_STORAGE_DCPT* sDcpt);
static uint16_t _TCPIPStorageCheckSum(NVM_STORAGE_DCPT* sDcpt);
static void _TCPIPStorageFormatCopy(NET_CONFIG* pDst, const NET_CONFIG* pSrc);

/*static __inline__*/static  void /*__attribute__((always_inline))*/ _TCPIPStorageClear(NVM_STORAGE_DCPT* sDcpt)
{
    memset(sDcpt, 0x0, sizeof(*sDcpt));
    sDcpt->pAccFnc = &_nvmAccessDcpt;
    sDcpt->sHdr.hdrLength = sizeof(sDcpt->sHdr);
        
}
/*static __inline__*/static  void /*__attribute__((always_inline))*/ _TCPIPStorageInvalidate(NVM_STORAGE_DCPT* sDcpt)
{
    sDcpt->flags &= ~NVM_ST_FLAG_VALID;
    sDcpt->sHdr.nEntries = sDcpt->sHdr.entryLength = 0;        
}

/*static __inline__*/static  bool /*__attribute__((always_inline))*/ _TCPIPStorageIsValid(NVM_STORAGE_DCPT* sDcpt)
{
    return (sDcpt->flags & NVM_ST_FLAG_VALID) != 0;
}

/*static __inline__*/static  void /*__attribute__((always_inline))*/ _TCPIPStorageValidate(NVM_STORAGE_DCPT* sDcpt)
{
    sDcpt->flags |= NVM_ST_FLAG_VALID;
}


// implementation
//


// inits a storage
// storageId will allow selection between multiple storages
// not supported right now
// returns false if storage could not be opened
bool TCPIP_STORAGE_Init(int storageId)
{
    if(_hStorage == 0)
    {
        _hStorage = &_nvmStorageDcpt;
        _TCPIPStorageClear(_hStorage);
        (*_hStorage->pAccFnc->init)();
    }
    return true;
}


// de-inits a storage
// storageId will allow selection between multiple storages
// not supported right now
// returns true if de-init possible, false if clients still having valid handles
bool TCPIP_STORAGE_DeInit(int storageId)
{
    if(_hStorage != 0)
    {
        if(_hStorage->instCount == 0)
        {   // all clients closed
            //(*_hStorage->deinit)();
            _TCPIPStorageClear(_hStorage);
            _hStorage = 0;
            return true;
        }
        return false;
    }
    return true;
}

// opens a storage and returns a handle
// storageId will allow selection between multiple storages
// not supported right now
// refresh instructs to read the storage (default).
// no refresh is useful for just getting a handle to erase, for example 
// return 0 if storage could not be opened
TCPIP_STORAGE_HANDLE TCPIP_STORAGE_Open(int storageId, bool refresh)
{
    
    if(_hStorage)
    {
        if(_hStorage->instCount == 0)
        {   // open the storage
            if(refresh)
            {
                TCPIP_STORAGE_Refresh(_hStorage);
            }
        }
        _hStorage->instCount++;     // add another user
    }

    return (TCPIP_STORAGE_HANDLE)_hStorage;
}

// closes the storage
void TCPIP_STORAGE_Close(TCPIP_STORAGE_HANDLE hS)
{
    NVM_STORAGE_DCPT* sDcpt = (NVM_STORAGE_DCPT*)hS;

    // release instance
    if(--sDcpt->instCount == 0)
    {   // the last user, can close
        // normally should check isDirty
        // but we don't buffer data internally
        // we perform the write immediately
        _TCPIPStorageClear(sDcpt);  // it will have to be refreshed
    }    
}

// copies the current label existent in the storage
// the less of TCPIP_STORAGE_USER_LABEL_SIZE and bufferSize is copied to the buffer
// returns the number of bytes copied
int TCPIP_STORAGE_GetLabel(TCPIP_STORAGE_HANDLE hS, uint8_t* buffer, uint16_t bufferSize)
{
    int nBytes = 0;
#if _TCPIP_STORAGE_USER_LABEL_SIZE > 0
    NVM_STORAGE_DCPT* sDcpt = (NVM_STORAGE_DCPT*)hS;
    
    nBytes = sizeof(sDcpt->sHdr.storageLabel);
    if(nBytes > bufferSize)
    {
        nBytes = bufferSize;
    }
    memcpy(buffer, sDcpt->sHdr.storageLabel, nBytes);
#endif

    return nBytes;
}

// returns the current label size
// this function is useful when variable length labels will be supported
// right now this feature is not implemented
// and the function returns a build time value 
int TCPIP_STORAGE_GetLabelSize(TCPIP_STORAGE_HANDLE hS)
{
#if _TCPIP_STORAGE_USER_LABEL_SIZE > 0
    NVM_STORAGE_DCPT* sDcpt = (NVM_STORAGE_DCPT*)hS;
    
    return sizeof(sDcpt->sHdr.storageLabel);
#endif

    return 0;
}


// sets the current label to be used in write or verification
// the less of TCPIP_STORAGE_USER_LABEL_SIZE and labelSize are used from pLabel (none if 0)
// returns the number of bytes copied
int TCPIP_STORAGE_SetLabel(TCPIP_STORAGE_HANDLE hS, const uint8_t* pLabel, int labelSize)
{
    int nBytes = 0;
#if _TCPIP_STORAGE_USER_LABEL_SIZE > 0
    NVM_STORAGE_DCPT* sDcpt = (NVM_STORAGE_DCPT*)hS;

    nBytes = sizeof(sDcpt->sHdr.storageLabel);
    if(nBytes > labelSize)
    {
        nBytes = labelSize;
    }
    memset(sDcpt->sHdr.storageLabel, 0, sizeof(sDcpt->sHdr.storageLabel));
    memcpy(sDcpt->sHdr.storageLabel, pLabel, nBytes);
#endif
    return nBytes;
}


// forces a storage refresh 
// repeats the action that is done at Open time, for the 1st caller
// returns true if a valid configuration found
bool TCPIP_STORAGE_Refresh(TCPIP_STORAGE_HANDLE hS)
{
    NVM_STORAGE_HDR stgHdr;
    uint16_t        chksum1, chksum2;        
    NVM_STORAGE_DCPT* sDcpt = (NVM_STORAGE_DCPT*)hS;

    // read the header
    (*sDcpt->pAccFnc->readArray)(0, (uint8_t*)&stgHdr, sizeof(sDcpt->sHdr));
    // sanity check
    if(stgHdr.hdrLength == sDcpt->sHdr.hdrLength)
    {   // size match
        chksum1 = stgHdr.hdrChecksum;
        stgHdr.hdrChecksum = 0;
        chksum2 = CalcIPChecksum((uint8_t*)&stgHdr, sizeof(stgHdr));
        if(chksum1 == chksum2)
        {   // checksum match
            stgHdr.hdrChecksum = chksum1;
            if(stgHdr.nEntries > 0 && stgHdr.entryLength == sizeof(NVM_STORAGE_ENTRY))
            {   // so far so good: the storage contains records of this size/build
                memcpy(&sDcpt->sHdr, &stgHdr, sizeof(stgHdr));
                if(_TCPIPStorageVerifyLoad(sDcpt))
                {
                    _TCPIPStorageValidate(sDcpt);
                    return true;
                }
            }
        }
    }
    
    _TCPIPStorageInvalidate(sDcpt);
    return false;
}



// returns true if valid storage: checksum OK and entries 
bool TCPIP_STORAGE_IsValid(TCPIP_STORAGE_HANDLE hS)
{
    return _TCPIPStorageIsValid((NVM_STORAGE_DCPT*)hS);
}

// returns the no of entries 
int TCPIP_STORAGE_Entries(TCPIP_STORAGE_HANDLE hS)
{
    NVM_STORAGE_DCPT* sDcpt = (NVM_STORAGE_DCPT*)hS;
    if(_TCPIPStorageIsValid(sDcpt))
    {
        return sDcpt->sHdr.nEntries;
    }

    return 0;
}

// returns the size of an entry
// Note: all the storage entries have the same size! 
int TCPIP_STORAGE_EntryLength(TCPIP_STORAGE_HANDLE hS)
{
    NVM_STORAGE_DCPT* sDcpt = (NVM_STORAGE_DCPT*)hS;
    if(_TCPIPStorageIsValid(sDcpt))
    {
        return sDcpt->sHdr.entryLength;
    }

    return 0;
}

// erases/initializes the storage, marks it invalid
void TCPIP_STORAGE_Erase(TCPIP_STORAGE_HANDLE hS)
{
    NVM_STORAGE_HDR eraseHdr;
    NVM_STORAGE_DCPT* sDcpt = (NVM_STORAGE_DCPT*)hS;

    memset(&eraseHdr, 0xff, sizeof(eraseHdr));
    (*sDcpt->pAccFnc->beginWrite)(0);
    (*sDcpt->pAccFnc->writeArray)((uint8_t*)&eraseHdr, sizeof(eraseHdr));
    
    _TCPIPStorageInvalidate(sDcpt);     // no longer valid

}


// writes an entry and optionally verifies that everything is OK
// returns true if success, false otherwise
// The entries are processed by the interface (MAC) type. 
bool TCPIP_STORAGE_WriteEntry(TCPIP_STORAGE_HANDLE hS, TCPIP_NET_HANDLE netH, bool verify)
{
    int replaceIx, entryIx;
    NVM_STORAGE_ENTRY wEntry;
    NVM_STORAGE_DCPT* sDcpt = (NVM_STORAGE_DCPT*)hS;
    const NET_CONFIG* pConfig = (const NET_CONFIG*)netH;
    
    if(_TCPIPStorageIsValid(sDcpt))
    {   // adding an entry
        if(sDcpt->sHdr.entryLength != sizeof(NVM_STORAGE_ENTRY))
        {   // make sure we don't write into the wrong storage
            // that hasn't been erased if it's inconsistent with current build/save
            return false;
        }
    }

    replaceIx = _TCPIPStorageFindEntryIx(sDcpt, pConfig, &wEntry);
    if(replaceIx == -1)
    {   // non-existent; add tail new entry
        entryIx = sDcpt->sHdr.nEntries; 
    }
    else
    {   // existent entry, replace
        entryIx = replaceIx;
    }
    
    if(!_TCPIPStorageWriteEntry(sDcpt, entryIx, pConfig, &wEntry))
    {   // failed
        return false;
    }

    if(replaceIx == -1)
    {   // just added something
        sDcpt->sHdr.entryLength = sizeof(NVM_STORAGE_ENTRY); // just in case this is the 1st valid entry
        sDcpt->sHdr.nEntries++;
    }
    
    _TCPIPStorageUpdateHdr(sDcpt);
    
    if(verify)
    {
        return TCPIP_STORAGE_Refresh(sDcpt);
    }

    _TCPIPStorageValidate(sDcpt);      // assume is valid now
    return true;

}

// reads an entry with the required index into pDest.
// returns true if such index found
bool TCPIP_STORAGE_ReadEntryIx(TCPIP_STORAGE_HANDLE hS, int entryIx, TCPIP_NET_HANDLE netH)
{
    NVM_STORAGE_DCPT* sDcpt = (NVM_STORAGE_DCPT*)hS;
    NET_CONFIG*       pDest = (NET_CONFIG*)netH;

    if( entryIx < sDcpt->sHdr.nEntries)  
    {   // valid entry
        int offset = sizeof(NVM_STORAGE_HDR) + entryIx * sizeof(NVM_STORAGE_ENTRY);
        (*sDcpt->pAccFnc->readArray)(offset, (uint8_t*)pDest, sizeof(*pDest));
        return true;
    }
    return false;
}

// finds a reference entry 
// returns the entry index (0 -  TCPIP_STORAGE_Entries()) if such entry found and the MAC id matches
// or -1 if not found
// chkFlags tells what is checked to have a match
int TCPIP_STORAGE_FindEntry(TCPIP_STORAGE_HANDLE hS, TCPIP_NET_HANDLE refNetH)
{
    return _TCPIPStorageFindEntryIx((NVM_STORAGE_DCPT*)hS, (NET_CONFIG*)refNetH, 0);
}

// helper to store a network interface configuration
// on the external storage
// returns true if succeeded
// Note: has to be called only AFTER the stack was initialized
// i.e. the call to the TCPIP_STACK_Init()
// otherwise the TCPIP_STORAGE_Init() must be called
bool TCPIP_STORAGE_SaveNetConfig(TCPIP_STORAGE_HANDLE hS, TCPIP_NET_HANDLE netH, bool verify)
{
    const NET_CONFIG* pConfig = (const NET_CONFIG*)netH;
    if(pConfig)
    {
        return TCPIP_STORAGE_WriteEntry(hS, pConfig, verify);
    }

    return false;   // no such interface
}

// helper to store the whole stack configuration
// on the external storage
// returns true if succeeded
// Note: has to be called only AFTER the stack was initialized
// i.e. the call to the TCPIP_STACK_Init()
// otherwise the TCPIP_STORAGE_Init() must be called
bool TCPIP_STORAGE_SaveStackConfig(TCPIP_STORAGE_HANDLE hS, bool verify)
{
    int netIx;
    const NET_CONFIG* pConfig;
    int netNo = TCPIP_STACK_NetworksNo();  // all existent interfaces

    for(netIx = 0; netIx < netNo - 1; netIx++)
    {
        pConfig = (const NET_CONFIG*)TCPIP_STACK_IxToNet(netIx);
        if(!TCPIP_STORAGE_WriteEntry(hS, pConfig, false))
        {
            return false;
        }
    }

    return TCPIP_STORAGE_WriteEntry(hS, (const NET_CONFIG*)TCPIP_STACK_IxToNet(netIx), verify);

}



// local functions
// 

// checks consistency of a storage payload
// the header is supposed to be already validated 
static bool _TCPIPStorageVerifyLoad(NVM_STORAGE_DCPT* sDcpt)
{

    uint16_t chkSum = _TCPIPStorageCheckSum(sDcpt);

    return chkSum == sDcpt->sHdr.storageChecksum;
}

// finds the index of a specific configuration in the storage
// returns the entry ix or, if not found, -1 
static int _TCPIPStorageFindEntryIx(NVM_STORAGE_DCPT* sDcpt, const NET_CONFIG* pConfig, NVM_STORAGE_ENTRY* pwEntry)
{
    int ix;
    NVM_STORAGE_ENTRY   stgEntry;
    int offset = sizeof(NVM_STORAGE_HDR);
    
    for(ix = 0; ix < sDcpt->sHdr.nEntries; ix++)
    {
        (*sDcpt->pAccFnc->readArray)(offset, (uint8_t*)&stgEntry, sizeof(stgEntry));
        if(stgEntry.entryData.macId == pConfig->macId)
        {   // found it
            if(pwEntry)
            {
                memcpy(pwEntry, &stgEntry, sizeof(stgEntry));
            }
            return ix;
        }
        offset += sizeof(NVM_STORAGE_ENTRY);
    }

    return -1;
}

// writes a (new) entry at the entryIx position
// note that source configuration is properly formatted 
// so that the dynamic part of the NET_CONFIG is zeroed out
static bool _TCPIPStorageWriteEntry(NVM_STORAGE_DCPT* sDcpt, int entryIx, const NET_CONFIG* pConfig, NVM_STORAGE_ENTRY* pwEntry)
{
    int offset = sizeof(NVM_STORAGE_HDR) + entryIx * sizeof(NVM_STORAGE_ENTRY);
    
#if defined(TCPIP_STACK_USE_MPFS) || defined(TCPIP_STACK_USE_MPFS2)
    if( offset + sizeof(NVM_STORAGE_ENTRY) > MPFS_RESERVE_BLOCK)
    {
        return false;
    }
#endif

    _TCPIPStorageFormatCopy(&pwEntry->entryData, pConfig);

    (*sDcpt->pAccFnc->beginWrite)(offset);
    (*sDcpt->pAccFnc->writeArray)((uint8_t*)pwEntry, sizeof(NVM_STORAGE_ENTRY));


    return true;
    
}

// writes the updated header info
// it assumes some entries have changed
static void _TCPIPStorageUpdateHdr(NVM_STORAGE_DCPT* sDcpt)
{    
    sDcpt->sHdr.storageChecksum = _TCPIPStorageCheckSum(sDcpt);
    sDcpt->sHdr.hdrChecksum = 0;
    sDcpt->sHdr.hdrChecksum = CalcIPChecksum((uint8_t*)&sDcpt->sHdr, sizeof(sDcpt->sHdr));


    (*sDcpt->pAccFnc->beginWrite)(0);
    (*sDcpt->pAccFnc->writeArray)((uint8_t*)&sDcpt->sHdr, sizeof(sDcpt->sHdr));
    
}

// returns the storage of a VALID header storage
static uint16_t _TCPIPStorageCheckSum(NVM_STORAGE_DCPT* sDcpt)
{
    uint8_t  chksumBuff[40] __attribute__((aligned));
                            // 2 multiple size buffer!
                            // the exact size does not matter, but it has to be even
                            // and aligned in order to correctly calculate the check sum
                            // 
    int storageSz = sDcpt->sHdr.nEntries * sizeof(NVM_STORAGE_ENTRY);

    int nBuffs = storageSz/sizeof(chksumBuff);
    int rBytes = storageSz - nBuffs * sizeof(chksumBuff);

    int ix;
    TCPIP_UINT32_VAL chkSum;
    int offset = sizeof(NVM_STORAGE_HDR);
    
    chkSum.Val = 0;
    
    for(ix = 0; ix < nBuffs; ix++)
    {
        (*sDcpt->pAccFnc->readArray)(offset, chksumBuff, sizeof(chksumBuff));
        offset += sizeof(chksumBuff);
        chkSum.Val += ~CalcIPChecksum(chksumBuff, sizeof(chksumBuff));
    }

    if(rBytes)
    {
        (*sDcpt->pAccFnc->readArray)(offset, chksumBuff, rBytes);
        chkSum.Val += ~CalcIPChecksum(chksumBuff, rBytes);
    }

    // Perform the end-around carry final adjustment
	chkSum.Val = (uint32_t)chkSum.w[0] + (uint32_t)chkSum.w[1];
    chkSum.Val = (uint32_t)chkSum.w[0] + (uint32_t)chkSum.w[1];
    chkSum.w[0] += chkSum.w[1];
    return ~chkSum.w[0];
}

// copy src to destination
// properly formats the destination
// source could be 0
static void _TCPIPStorageFormatCopy(NET_CONFIG* pDst, const NET_CONFIG* pSrc)
{
    if(pSrc)
    {
        memcpy(pDst, pSrc, sizeof(*pDst));
    }
    // format the destination
    _TCPIPStackClearDynNetSettings(pDst);
}

#if defined(EEPROM_CS_TRIS)
static void _XEEInit(void)
{
	EEPROM_CS_IO = 1;
	EEPROM_CS_TRIS = 0;
    XEEInit();
    
}
#elif defined(SPIFLASH_CS_TRIS)
static void _SPIFlashInit(void)
{
	SPIFLASH_CS_IO = 1;
	SPIFLASH_CS_TRIS = 0;

    SPIFlashInit();
}

#endif

#endif  // defined(_TCPIP_STACK_STORAGE_ENABLED)

