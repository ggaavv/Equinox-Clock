/*******************************************************************************
  Address Resolution Protocol (ARP) Configuration file

  Summary:
    ARP configuration file
    
  Description:
    This file contains the ARP module configuration options
    
*******************************************************************************/

/*******************************************************************************
FileName:   arp_config.h
Copyright © 2011 released Microchip Technology Inc.  All rights
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

#ifndef _ARP_CONFIG_H_
#define _ARP_CONFIG_H_


// configuration options

// number of entries in the cache
// default number of entries per interface
#define ARP_CACHE_ENTRIES_DEFAULT       11

// specific interaces numbers
#define ARP_CACHE_ENTRIES_PIC32INT      11
#define ARP_CACHE_ENTRIES_MRF24W     11
#define ARP_CACHE_ENTRIES_ENCJ600       11
#define ARP_CACHE_ENTRIES_ENCJ60        11
#define ARP_CACHE_ENTRIES_97J60         11


// timeout for a solved entry in the cache, in seconds
// the entry will be removed if the tmo elapsed
// and the entry has not been referenced again
#define ARP_CACHE_SOLVED_ENTRY_TMO      (20 * 60)

// timeout for a pending to be solved entry in the cache, in seconds
// the entry will be removed if the tmo elapsed
// and the entry has not been solved
// a solved entry moves to the solved entries timeout
#define ARP_CACHE_PENDING_ENTRY_TMO      (1 * 60)

// timeout for resending an ARP request for a pending entry
// In order to prevent the ARP flooding the standard recommends
// it to be greater than 1 sec.
// Of course, it should be less than ARP_CACHE_PENDING_ENTRY_TMO  
#define ARP_CACHE_PENDING_RETRY_TMO       (2)


// max percentage of permanent entries in the cache
// note that since permanent entries cannot be removed
// they tend to degrade the efficiency of the cache
// look up  
#define ARP_CACHE_PERMANENT_QUOTA         50


// default purge threshold, percentage
// once the number of resolved entries in the cache gets
// beyond the threshold some resolved entries will be purged
#define ARP_CACHE_PURGE_THRESHOLD         75

// how many entries to delete, once the threshold is reached
//
#define ARP_CACHE_PURGE_QUANTA            3


// ARP task processing rate, in seconds
// the ARP module will process a timer event with this rate
// for maintaining its own queues, processing timeouts, etc.
// Choose it so that the other ARP_CACHE_xxx_TMO are multiple of this
//
#define ARP_TASK_PROCESS_RATE              (2)

// MAX num allowed registrations of Modules/Apps
#define MAX_REG_APPS            2           

typedef struct
{
    // specific ARP params
    size_t  cacheEntries;   // cache entries for this interface
    bool    deleteOld;      // delete old cache if still in place,
                            // else don't re-initialize it

}ARP_MODULE_CONFIG;

// This is a template of how the ARP module should be initialized and
// the parameters that it needs.
static const ARP_MODULE_CONFIG arpConfigData = 
{
	ARP_CACHE_ENTRIES_DEFAULT,
	true
};

#endif  // _ARP_CONFIG_H_



