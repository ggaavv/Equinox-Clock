/*******************************************************************************
  TCPIP Storage Interface Definition

  Summary:
    This file contains the Application Program Interface (API) definition  for 
    the TCPIP Storage library.
    
  Description:
    This library provides functions to load/store TCPIP network configurationss
*******************************************************************************/

/*******************************************************************************
FileName:  tcpip_storage.h 
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

#ifndef _TCPIP_STORAGE_H_
#define _TCPIP_STORAGE_H_

// definitions
// 


//  handle to a storage
typedef const void* TCPIP_STORAGE_HANDLE;

// interface functions
// 

// inits a storage
// storageId will allow selection between multiple storages
// not supported right now
// returns false if storage could not be opened
bool             TCPIP_STORAGE_Init(int storageId);

// de-inits a storage
// storageId will allow selection between multiple storages
// not supported right now
// returns true if de-init possible, false if clients still having valid handles
bool            TCPIP_STORAGE_DeInit(int storageId);

// opens a storage and returns a handle
// storageId will allow selection between multiple storages
// not supported right now
// refresh instructs to read the storage (default).
// no refresh is useful for just getting a handle to erase, for example 
// return 0 if storage could not be opened
TCPIP_STORAGE_HANDLE    TCPIP_STORAGE_Open(int storageId, bool refresh);

// closes the storage
void    TCPIP_STORAGE_Close(TCPIP_STORAGE_HANDLE hS);

// copies the current label existent in the storage
// the less of TCPIP_STORAGE_USER_LABEL_SIZE and bufferSize is copied to the buffer
// returns the number of bytes copied
int     TCPIP_STORAGE_GetLabel(TCPIP_STORAGE_HANDLE hS, uint8_t* buffer, uint16_t bufferSize);

// sets the current label to be used in write
// the less of TCPIP_STORAGE_USER_LABEL_SIZE and labelSize are used from pLabel (none if 0)
// returns the number of bytes copied
// Note: once the new label is set it any new call to TCPIP_STORAGE_GetLabel()
// will return the new value even if the storage was not updated!
// The new value of the label won't be used until a
// TCPIP_STORAGE_WriteEntry is called.
int     TCPIP_STORAGE_SetLabel(TCPIP_STORAGE_HANDLE hS, const uint8_t* pLabel, int labelSize);


// returns the current label size
// this function is useful when variable length labels will be supported
// right now this feature is not implemented
// and the function returns a build time value 
int     TCPIP_STORAGE_GetLabelSize(TCPIP_STORAGE_HANDLE hS);

// forces a storage refresh 
// repeats the action that is done at Open time, for the 1st caller
// rerutns true if a valid configuration found
bool    TCPIP_STORAGE_Refresh(TCPIP_STORAGE_HANDLE hS);

// returns true if valid storage: checksum OK and entries 
bool    TCPIP_STORAGE_IsValid(TCPIP_STORAGE_HANDLE hS);

// returns the no of entries 
int     TCPIP_STORAGE_Entries(TCPIP_STORAGE_HANDLE hS);

// returns the size of an entry
// Note: all the storage entries have the same size! 
int     TCPIP_STORAGE_EntryLength(TCPIP_STORAGE_HANDLE hS);

// erases/initializes the storage, marks it invalid
void    TCPIP_STORAGE_Erase(TCPIP_STORAGE_HANDLE hS);

// writes an entry and optionally verifies that everything is OK
// returns true if success, false otherwise
// The entries are processed by the interface (MAC) type.
// This operation performs the write operation on the storage
// It also updates the latest version of the label that was set 
bool     TCPIP_STORAGE_WriteEntry(TCPIP_STORAGE_HANDLE hS, TCPIP_NET_HANDLE netH, bool verify);

// reads an entry with the required index into pDest.
// returns true if such index found
// NOTE: the returned structure contains only static data.
// Do NOT expect that the value retrieved from the storage is valid
// to be used as is by the stack!
// Actually the storage zeroes out the dynamic data.
bool     TCPIP_STORAGE_ReadEntryIx(TCPIP_STORAGE_HANDLE hS, int entryIx, TCPIP_NET_HANDLE netH);

// finds a reference entry 
// returns the entry index (0 -  TCPIP_STORAGE_Entries()) if such entry found and the MAC type matches
// or -1 if not found
int     TCPIP_STORAGE_FindEntry(TCPIP_STORAGE_HANDLE hS, TCPIP_NET_HANDLE refNetH);



// helper to store a network interface configuration
// on the external storage
// returns true if succeeded
// Note: has to be called only AFTER the stack was initialized
// i.e. the call to the TCPIP_STACK_Init()
// otherwise the TCPIP_STORAGE_Init() must be called
bool    TCPIP_STORAGE_SaveNetConfig(TCPIP_STORAGE_HANDLE hS, TCPIP_NET_HANDLE netH, bool verify);


// helper to store the whole stack configuration
// on the external storage
// returns true if succeeded
// Note: has to be called only AFTER the stack was initialized
// i.e. the call to the TCPIP_STACK_Init()
// otherwise the TCPIP_STORAGE_Init() must be called
bool    TCPIP_STORAGE_SaveStackConfig(TCPIP_STORAGE_HANDLE hS, bool verify);


#endif  // _TCPIP_STORAGE_H_

