/*******************************************************************************
  Linked list helper file

  Summary:
    Linked lists manipulation Interface Header
    
  Description:
    This header file contains the function prototypes and definitions of the 
    linked lists manipulation routines
*******************************************************************************/

/*******************************************************************************
FileName:   link_list.h
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

#ifndef _LINK_LISTS_H_
#define _LINK_LISTS_H_

#include <stdbool.h>

typedef struct _TAG_SGL_LIST_NODE
{
	struct _TAG_SGL_LIST_NODE*		next;		// next node in list
    void*                           data[0];    // generic payload    
}SGL_LIST_NODE;	// generic linked list node definition


typedef struct
{
	SGL_LIST_NODE*	head;	// list head
	SGL_LIST_NODE*	tail;
    int             nNodes; // number of nodes in the list 
}SINGLE_LIST;	// single linked list


/////  single linked lists manipulation ///////////
//


void  SingleListInit(SINGLE_LIST* pL);


extern __inline__ bool __attribute__((always_inline)) SingleListIsEmpty(SINGLE_LIST* pL)
{
    return pL->head == 0;
}


extern __inline__ int __attribute__((always_inline)) SingleListCount(SINGLE_LIST* pL)
{
    return pL->nNodes;
}

void  SingleListAddHead(SINGLE_LIST* pL, SGL_LIST_NODE* pN);

void  SingleListAddTail(SINGLE_LIST* pL, SGL_LIST_NODE* pN);


// insertion in the middle, not head or tail
void  SingleListAddMid(SINGLE_LIST* pL, SGL_LIST_NODE* pN, SGL_LIST_NODE* after);


SGL_LIST_NODE*  SingleListRemoveHead(SINGLE_LIST* pL);

// removes a node somewhere in the middle
// Note: this is lengthy!
// Use a double linked list if faster operation needed!
SGL_LIST_NODE*  SingleListRemoveNode(SINGLE_LIST* pL, SGL_LIST_NODE* pN);


void  SingleListAppendList(SINGLE_LIST* pL, SINGLE_LIST* pAList);


extern __inline__ void __attribute__((always_inline)) SingleListDelete(SINGLE_LIST* pL)
{
	while((SingleListRemoveHead(pL)));
}

/////  double linked lists manipulation ///////////
//

typedef struct _TAG_DBL_LIST_NODE
{
	struct _TAG_DBL_LIST_NODE*		next;		// next node in list
	struct _TAG_DBL_LIST_NODE*		prev;		// prev node in list
    void*                           data[0];    // generic payload    
}DBL_LIST_NODE;	// generic linked list node definition


typedef struct
{
	DBL_LIST_NODE*	head;	// list head
	DBL_LIST_NODE*	tail;   // list tail;
    int             nNodes; // number of nodes in the list 
}DOUBLE_LIST;	// double linked list


void  DoubleListInit(DOUBLE_LIST* pL);


extern __inline__ bool __attribute__((always_inline)) DoubleListIsEmpty(DOUBLE_LIST* pL)
{
    return pL->head == 0;
}

extern __inline__ int __attribute__((always_inline)) DoubleListCount(DOUBLE_LIST* pL)
{
    return pL->nNodes;
}

void  DoubleListAddHead(DOUBLE_LIST* pL, DBL_LIST_NODE* pN);

void  DoubleListAddTail(DOUBLE_LIST* pL, DBL_LIST_NODE* pN);

// add node pN in the middle, after existing node "after"
void  DoubleListAddMid(DOUBLE_LIST* pL, DBL_LIST_NODE* pN, DBL_LIST_NODE* after);

DBL_LIST_NODE*  DoubleListRemoveHead(DOUBLE_LIST* pL);

DBL_LIST_NODE*  DoubleListRemoveTail(DOUBLE_LIST* pL);

// remove existing node, neither head, nor tail
void  DoubleListRemoveMid(DOUBLE_LIST* pL, DBL_LIST_NODE* pN);

void  DoubleListRemoveNode(DOUBLE_LIST* pL, DBL_LIST_NODE* pN);

extern __inline__ void __attribute__((always_inline)) DoubleListDelete(DOUBLE_LIST* pL)
{
    while((DoubleListRemoveHead(pL)));
}

#endif //  _LINK_LISTS_H_


