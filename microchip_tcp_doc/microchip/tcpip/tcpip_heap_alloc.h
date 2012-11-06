/*******************************************************************************
  TCPIP Heap Allocation Manager

  Summary:
    
  Description:
*******************************************************************************/

/*******************************************************************************
FileName:   tcpip_heap_alloc.h
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

#ifndef _TCPIP_HEAP_ALLOC_H_
#define _TCPIP_HEAP_ALLOC_H_

#include <stdint.h>
#include <stdbool.h>

typedef enum
{
    TCPIP_HEAP_RES_OK            = 0,   // everything OK

    TCPIP_HEAP_RES_BUFF_ERR,            // invalid supplied heap buffer
    
    TCPIP_HEAP_RES_BUFF_ALIGN_ERR,     // supplied heap buffer not properly aligned
    
    TCPIP_HEAP_RES_BUFF_SIZE_ERR,      // supplied heap buffer too small
    
    TCPIP_HEAP_RES_NO_MEM,              // out of memory
    
    TCPIP_HEAP_RES_IN_USE,              // heap in use, cannot be de-allocated
    
    TCPIP_HEAP_RES_PTR_ERR,            // corrupt pointer

}TCPIP_HEAP_RES;    // result of a heap operation


typedef enum
{
    TCPIP_HEAP_FLAG_NONE      = 0x00,
    
}TCPIP_HEAP_FLAGS;  // heap creation/allocation flags


// handle to a heap
typedef const void* TCPIP_HEAP_HANDLE;


/********************************
 * Interface Functions
*******************************************/ 

// Creates a TCPIP memory heap
// A memory heap consists of block of data that is
// allocated in chunks using a first fit algorithm.
// Allocation and de-allocation operations are not very fast
// (but reasonably fast).
// However, the allocation granularity is pretty good.

// There is no data maintained in this heap on a per client basis.
// What this means is that is OK to pass the heap handle to other
// clients (software modules) to use it.
// Note that this is a private TCPIP heap and no multi-threaded protection is provided!
//
/*********************************************************************
 * Function:        TCPIP_HEAP_HANDLE TCPIP_HEAP_Create(uint8_t* heapBuffer, uint8_t size_t buffSize, TCPIP_HEAP_FLAGS flags, TCPIP_HEAP_RES* pRes)
 *
 * PreCondition:    heapBuffer, buffSize    - valid buffer and size values
 *                  heapBuffer - properly aligned buffer
 *
 * Input:           heapBuffer  - buffer to contain the heap
 *                  buffSize    - size of the supplied buffer
 *                  flags       - heap creation/allocation flags.
 *                                Not used for now.
 *                  pRes        - Address to store the operation result.
 *                                Can be NULL if result not needed         
 *
 * Output:          A valid handle if the creation succeeded,
 *                  NULL otherwise.         
 *
 * Side Effects:    None
 *
 * Overview:        The function creates a heap of the maximum possible size in the supplied buffer space.
 *                  The size of the heap cannot be changed afterwards.
 *
 * Note:            The current heap alignment unit is an unsigned long, i.e. 32 bits.
 *                  The supplied heapBuffer value has to be correctly 32 bits aligned.
 *                  Other wise a TCPIP_HEAP_RES_ALIGN_ERR error code will be returned!
 *
 ********************************************************************/
TCPIP_HEAP_HANDLE   TCPIP_HEAP_Create(uint8_t* heapBuffer, size_t buffSize, TCPIP_HEAP_FLAGS flags, TCPIP_HEAP_RES* pRes);

/*********************************************************************
 * Function:        int TCPIP_HEAP_Delete(TCPIP_HEAP_HANDLE heapH);
 *
 * PreCondition:    heapH       - valid heap handle 
 *
 * Input:           heapH       - handle to a heap that was created with TCPIP_HEAP_Create()
 *
 * Output:          true if deletion succeeded,
 *                  false otherwise.         
 *
 * Side Effects:    None
 *
 * Overview:        The function deletes a previously created heap.
 *
 * Note:            The call will fail if blocks are still in use.
 ********************************************************************/
int                 TCPIP_HEAP_Delete(TCPIP_HEAP_HANDLE heapH);


/*********************************************************************
 * Function:        void* TCPIP_HEAP_Malloc(TCPIP_HEAP_HANDLE heapH, size_t nBytes);
 *
 * PreCondition:    heapH       - valid heap handle 
 *
 * Input:           heapH       - handle to a heap
 *                  nBytes      - size of the block to be allocated
 *
 * Output:          a valid pointer to an allocated block if allocation succeeded,
 *                  NULL otherwise.         
 *
 * Side Effects:    None
 *
 * Overview:        The function checks the heap for finding a block large enough to accomodate the request.
 *                  A first fit algorithm is used.
 *
 * Note:            None
 ********************************************************************/
void*               TCPIP_HEAP_Malloc(TCPIP_HEAP_HANDLE heapH, size_t nBytes);

/*********************************************************************
 * Function:        void* TCPIP_HEAP_Calloc(TCPIP_HEAP_HANDLE heapH, size_t nElems, size_t elemSize);
 *
 * PreCondition:    heapH       - valid heap handle 
 *
 * Input:           heapH       - handle to a heap
 *                  nElems      - number of elements to be allocated
 *                  elemSize    - size of each element, in bytes
 *
 * Output:          a valid pointer to an allocated block if allocation succeeded,
 *                  NULL otherwise.         
 *
 * Side Effects:    None
 *
 * Overview:        The function checks the heap for finding a block large enough to accomodate
 *                  nElems*elemSize request.
 *                  If the block is found, it is zero initialized and returned to user.
 *                  A first fit algorithm is used.
 *
 * Note:            None
 ********************************************************************/
void*               TCPIP_HEAP_Calloc(TCPIP_HEAP_HANDLE heapH, size_t nElems, size_t elemSize);

/*********************************************************************
 * Function:        int TCPIP_HEAP_Free(TCPIP_HEAP_HANDLE heapH, void* pBuff);
 *
 * PreCondition:    heapH       - valid heap handle 
 *
 * Input:           heapH       - handle to a heap
 *                  pBuff       - pointer to a buffer previously allocated from the heap 
 *
 * Output:          true if the operation succeeded,
 *                  false otherwise       
 *
 * Side Effects:    None
 *
 * Overview:        The function returns the buffer to the heap.
 *                  Left and/or right defragment operations are performed if possible.
 *
 * Note:            None
 ********************************************************************/
int                 TCPIP_HEAP_Free(TCPIP_HEAP_HANDLE heapH, const void* pBuff);



/*********************************************************************
 * Function:        size_t TCPIP_HEAP_Size(TCPIP_HEAP_HANDLE heapH);
 *
 * PreCondition:    heapH       - valid heap handle 
 *
 * Input:           heapH       - handle to a heap
 *
 * Output:          the size of the heap as it was created
 *
 * Side Effects:    None
 *
 * Overview:        The function returns the size of the heap.
 *                  This is the size that was specified when the heap was created.
 *
 * Note:            None
 ********************************************************************/
size_t                 TCPIP_HEAP_Size(TCPIP_HEAP_HANDLE heapH);


/*********************************************************************
 * Function:        size_t TCPIP_HEAP_MaxSize(TCPIP_HEAP_HANDLE heapH);
 *
 * PreCondition:    heapH       - valid heap handle 
 *
 * Input:           heapH       - handle to a heap
 *
 * Output:          the max size of a block that can be allocated from the heap
 *
 * Side Effects:    None
 *
 * Overview:        The function returns the maximum size of a block that can be 
 *                  currently allocated from this heap.
 *
 * Note:            This is info only.
 *                  It can change is the heap has multiple clients.
 *
 *                  The call is expensive.
 *                  The whole heap has to be traversed to find the maximum.
 *                  If the heap is really fragmented this might take some time.
 *
 ********************************************************************/
size_t                 TCPIP_HEAP_MaxSize(TCPIP_HEAP_HANDLE heapH);



/*********************************************************************
 * Function:        size_t TCPIP_HEAP_FreeSize(TCPIP_HEAP_HANDLE heapH);
 *
 * PreCondition:    heapH       - valid heap handle 
 *
 * Input:           heapH       - handle to a heap
 *
 * Output:          the size of the available space in the heap
 *
 * Side Effects:    None
 *
 * Overview:        The function returns the size of the space currently
 *                  available in the heap.
 *
 * Note:            This is a cumulative number, counting all the existent free space.
 *                  It is not the maximum blocks size that could be allocated from the heap.
 ********************************************************************/
size_t                 TCPIP_HEAP_FreeSize(TCPIP_HEAP_HANDLE heapH);


/*********************************************************************
 * Function:      TCPIP_HEAP_RES TCPIP_HEAP_LastError(TCPIP_HEAP_HANDLE heapH)
 *
 * PreCondition:    heapH       - valid heap handle
 *
 * Input:           heapH       - handle of a heap
 *
 * Output:          The last error encountered in an operation
 *                  or TCPIP_HEAP_RES_OK if no error occurred
 *
 * Side Effects:    None
 *
 * Overview:        The function returns the last error encountered in a heap operation.
 *                  It clears the value of the last error variable.
 *
 * Note:            The heap holds an error variable storing the last error
 *                  encountered in an operation.
 *                  This should be consulted by the caller after each operation
 *                  that returns an invalid result for checking what the error condition
 *                  was.
 ********************************************************************/
TCPIP_HEAP_RES      TCPIP_HEAP_LastError(TCPIP_HEAP_HANDLE heapH);


#endif  // _TCPIP_HEAP_ALLOC_H_

