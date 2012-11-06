/*******************************************************************************
  ARP module private header

  Summary:
    Header definitions file for ARP module
    
  Description:
    This file contains the private definitions for the ARP module
*******************************************************************************/

/*******************************************************************************
FileName:   arp_private.h
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

#ifndef _ARP_PRIVATE_H_ 
#define _ARP_PRIVATE_H_ 


//#define OA_HASH_DYNAMIC_KEY_MANIPULATION
// do not use multiple hashes for this implementation


#include "hash_tbl.h"
#include "link_list.h"


// definitions
// 

#define ARP_OPERATION_REQ       0x0001u     // Operation code indicating an ARP Request
#define ARP_OPERATION_RESP      0x0002u     // Operation code indicating an ARP Response



#define HW_ETHERNET             (0x0001u)   // ARP Hardware type as defined by IEEE 802.3
#define ARP_IP                  (0x0800u)   // ARP IP packet type as defined by IEEE 802.3



// ARP packet structure
typedef struct __attribute__((aligned(2), packed))
{
    uint16_t    HardwareType;   // Link-layer protocol type (Ethernet is 1)
    uint16_t    Protocol;       // The upper-layer protocol issuing an ARP request (0x0800 for IPv4)
    uint8_t     MACAddrLen;     // MAC address length (6)
    uint8_t     ProtocolLen;    // Length of addresses used in the upper-layer protocol (4)
    uint16_t    Operation;      // The operation the sender is performing (ARP_OPERATION_REQ or ARP_OPERATION_RESP)
    MAC_ADDR    SenderMACAddr;  // The sender's hardware (MAC) address
    IP_ADDR     SenderIPAddr;   // The sender's IP address
    MAC_ADDR    TargetMACAddr;  // The target node's hardware (MAC) address
    IP_ADDR     TargetIPAddr;   // The target node's IP address
} ARP_PACKET;





// ARP cache entry
typedef struct  __attribute__((packed)) _TAG_ARP_HASH_ENTRY 
{
    OA_HASH_ENTRY               hEntry;         // hash header;
    struct _TAG_ARP_HASH_ENTRY* next;           // ordered link list by tInsert
    uint32_t                    ipAddress;      // the hash key: the IP address
    uint32_t                    tInsert;        // arp time it was inserted 
    MAC_ADDR                    hwAdd;          // the hardware address
    uint16_t                    nRetries;       // number of retries for an incomplete entry
}ARP_HASH_ENTRY;

// ARP flags used in hEntry->flags
// note that only the hEntry->flags.user fields
// should be used! 
typedef enum
{
    ARP_FLAG_ENTRY_PERM         = 0x0040,          // entry is permanent
    ARP_FLAG_ENTRY_COMPLETE     = 0x0080,          // regular entry, complete
                                                   // else it's incomplete
                                                   //
    ARP_FLAG_ENTRY_VALID_MASK   = (ARP_FLAG_ENTRY_PERM | ARP_FLAG_ENTRY_COMPLETE )
                                                     
                                                  
}ARP_ENTRY_FLAGS;

    
        


#define     ARP_HASH_PROBE_STEP      1


// each ARP cache consists of
typedef struct
{
    OA_HASH_DCPT*       cacheDcpt;      // contiguous space for a hash descriptor
                                        // and hash table entries
    SINGLE_LIST         permList;       // list of active entries that never expire
    SINGLE_LIST         completeList;   // list of completed, valid entries
    SINGLE_LIST         incompleteList; // list of not completed yet entries
    size_t              purgeThres;     // threshold to start cache purging
    size_t              purgeQuanta;    // how many entries to purge
    bool    deleteOld;                  // if 0 and old cache still in place don't re-initialize it
    bool    inited;                     // arp initialized on this interface
}ARP_CACHE_DCPT;

// ARP unaligned key
// the ip address field is unaligned
// in ARP packets
typedef struct __attribute__((packed))
{
    uint32_t    v;
}ARP_UNALIGNED_KEY;



// ARP event registration

typedef struct  _TAG_ARP_LIST_NODE
{
	struct _TAG_ARP_LIST_NODE*		next;		// next node in list
                                                // makes it valid SGL_LIST_NODE node
    ARP_EVENT_HANDLER               handler;    // handler to be called for event
    const void*                     hParam;     // handler parameter
    TCPIP_NET_HANDLE                hNet;       // interface that's registered for
                                                // 0 if all    
}ARP_LIST_NODE;


#endif  // _ARP_PRIVATE_H_ 

