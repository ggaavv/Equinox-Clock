/*******************************************************************************
  Address Resolution Protocol (ARP) Header file

  Summary:
    ARP definitions file
    
  Description:
    This source file contains the ARP module API
*******************************************************************************/

/*******************************************************************************
FileName:  arp.h 
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

#ifndef __ARP_H_
#define __ARP_H_

#include "arp_config.h"

// ARP API definitions

typedef enum
{
    // success codes
    ARP_RES_OK                  = 0,    // operation succeeded
    ARP_RES_ENTRY_NEW,                  // operation succeeded and a new entry was added
    ARP_RES_ENTRY_SOLVED,               // the required entry is already solved
    ARP_RES_ENTRY_QUEUED,               // the required entry was already queued
    ARP_RES_ENTRY_EXIST,                // the required entry was already cached
    ARP_RES_PERM_QUOTA_EXCEED,          // info: the quota of permanent entries was exceeded


    // failure codes
    ARP_RES_NO_ENTRY            = -1,   // no such entry exists
    ARP_RES_CACHE_FULL          = -2,   // the cache is full and no entry could be
                                        // removed to make room
    ARP_RES_TX_FAILED           = -3,   // failed to transmit an ARP message
    ARP_RES_BAD_INDEX           = -4,   // bad query index    
    ARP_RES_BAD_ADDRESS         = -5,   // bad IP address specified 
    ARP_RES_NO_INTERFACE        = -6,   // no such interface exists   
    

}ARP_RESULT;


// type of ARP entry
typedef enum
{
    ARP_ENTRY_TYPE_INVALID,             // empty entry
    ARP_ENTRY_TYPE_PERMANENT,           // entry valid and permanent
    ARP_ENTRY_TYPE_COMPLETE,            // entry valid
    ARP_ENTRY_TYPE_INCOMPLETE,          // entry not resolved yet
    ARP_ENTRY_TYPE_TOTAL,               // total entries
}ARP_ENTRY_TYPE;

typedef struct
{
    ARP_ENTRY_TYPE  entryType;      // what entry type
    IP_ADDR         entryIpAdd;     // the entry IP address
    MAC_ADDR        entryHwAdd;     // the entry hardware address
}ARP_ENTRY_QUERY;

// events reported by ARP
// possibly multiple events can be set,
// where it makes sense
typedef enum
{
    ARP_EVENT_SOLVED        = 0x01,     // a queued cache entry was solved                        
    ARP_EVENT_UPDATED       = 0x02,     // an existent cache entry was updated
    ARP_EVENT_PERM_UPDATE   = 0x04,     // an update for an permanent entry was received
                                        // however the permanent entry was not updated    
}ARP_EVENT_TYPE;



// Notification handler that can be called when a specific
// entry is resolved
// The param member significance is module dependent.
// It can be an IP address, pointer to some other structure, etc.
// The handler is called when an event of some sort
// occurs for the particular IP address entry
// if pNetIf == 0 then the notification is called for events on any interface
typedef void    (*ARP_EVENT_HANDLER)(TCPIP_NET_HANDLE hNet, const MAC_ADDR* MACAddr, ARP_EVENT_TYPE evType, const void* param);

// a handle that a client can use
// after the event handler has been registered
typedef const void* ARP_HANDLE;



// Register an ARP resolve handler
// Use hNet == 0 to register on all interfaces available
// Returns a valid handle if the call succeeds,
// or a null handle if the call failed.
// Function has to be called after the ARP is initialized
// The hParam is passed by the client and will be used by the ARP when the notification is made.
// It is used for per-thread content or if more modules, for example, share the same handler
// and need a way to differentiate the callback.
ARP_HANDLE      ARPRegisterHandler(TCPIP_NET_HANDLE hNet, ARP_EVENT_HANDLER handler, const void* hParam);

// deregister the event handler
// returns true or false if no such handler registered
bool            ARPDeRegisterHandler(ARP_HANDLE hArp);


// Resolves an entry
// Returns:
// ARP_RES_ENTRY_NEW        - if the operation succeeded and a new
//                            entry was added (and queued for resolving)
// ARP_RES_ENTRY_SOLVED     - if the required entry is already solved
// ARP_RES_ENTRY_QUEUED     - if the required entry was already queued
// ARP_RES_CACHE_FULL       - if new entry could not be inserted,
//                            the cache was full                            
// ARP_RES_BAD_ADDRESS      - bad address specified                           
// Stack clients can initiate an ARP resolve by using this function
ARP_RESULT      ARPResolve(TCPIP_NET_HANDLE hNet, IP_ADDR* IPAddr);

bool            ARPIsResolved(TCPIP_NET_HANDLE hNet, IP_ADDR* IPAddr, MAC_ADDR* MACAddr);



// cache manipulation routines
// 

// adds an ARP cache entry for the specified interface
// can add it as permanent, not subject to timeouts
// If cache is full, an entry will be deleted to make room
// returns ARP_RES_OK/ARP_RES_ENTRY_EXIST if succeded,
// an error otherwise (for example, cache is full with
// permanent entries that cannot be purged or the permanent quota exceeded)  
ARP_RESULT      ARPEntrySet(TCPIP_NET_HANDLE hNet, IP_ADDR* ipAdd, MAC_ADDR* hwAdd, bool perm);

// gets the current mapping for an IP address
// returns ARP_RES_OK
// or ARP_RES_NO_ENTRY if no such mapping exists
// similar to ARPIsResolved()
ARP_RESULT      ARPEntryGet(TCPIP_NET_HANDLE hNet, IP_ADDR* ipAdd, MAC_ADDR* pHwAdd);


// removes the mapping of an address, even a permanent one
// returns ARP_RES_OK
// or ARP_RES_NO_ENTRY if no such mapping exists
ARP_RESULT      ARPEntryRemove(TCPIP_NET_HANDLE hNet,  IP_ADDR* ipAdd);

// removes all the mapping belonging to an interface
// returns ARP_RES_OK
ARP_RESULT      ARPEntryRemoveAll(TCPIP_NET_HANDLE hNet);

// querries an ARP cache entry using the index of the cache line
// the index has to be a valid one i.e. < then ARPCacheGetEntries()
// populates the supplied query routine if not NULL
// returns ARP_RES_OK if valid index
// or ARP_RES_BAD_INDEX if index is out of range
// use it for displaying the cache contents
ARP_RESULT      ARPEntryQuery(TCPIP_NET_HANDLE hNet, size_t index, ARP_ENTRY_QUERY* pArpQuery);


// returns the number of entries of the specified type per interface
size_t          ARPCacheGetEntriesNo(TCPIP_NET_HANDLE hNet, ARP_ENTRY_TYPE type);


// sets the cache threshold for the specified interface in %
// once the number of entries in the cache is > than the threshold
// a number of purgeEntries (usually one) will be discarded
// returns ARP_RES_OK
ARP_RESULT       ARPCacheSetThreshold(TCPIP_NET_HANDLE hNet, int purgeThres, int purgeEntries);


#endif  // __ARP_H_



