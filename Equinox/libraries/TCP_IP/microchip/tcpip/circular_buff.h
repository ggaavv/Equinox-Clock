/*******************************************************************************
  Circular buffer helper file

  Summary:
    Circular buffer manipulation Interface Header
    
  Description:
    This header file contains the function prototypes and definitions of the 
    circular buffer manipulation routines
*******************************************************************************/

/*******************************************************************************
FileName:   circular_buff.h
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
//
#ifndef __CIRCULAR_BUFF_H_
#define __CIRCULAR_BUFF_H_




// Circular Buffers
// the payload is in bytes
typedef struct
{
	unsigned char*	start;		// start of buffer
	unsigned char*	end;		// end of buffer
	unsigned char*	rd;		    // current read slot
	unsigned char*	wr;		    // current write slot
}CIRCULAR_BUFFER;


// init a circular buffer
void		CircularBuffInit(unsigned char* buff, int size, CIRCULAR_BUFFER* pbHdr);


// adds a byte an updates appropriately
// returns number of bytes added to the buffer: 1 or 0
int		    CircularBuffAddByte(CIRCULAR_BUFFER* pbHdr, int newB);

// adds an array of bytes an updates appropriately
// returns number of bytes added to the buffer
int         CircularBuffAddArray(CIRCULAR_BUFFER* pbHdr, unsigned char* source, int sourceSize);

// reads the data to an user buffer
// and flushes it from the buffer
// returns number of bytes copied
// if the dest == 0, it just flushes the buffer
int		    CircularBuffRead(CIRCULAR_BUFFER* pbHdr, unsigned char* dest, int destSize);

// returns number of available bytes in the buffer
int		    CircularBuffAvlblBytes(CIRCULAR_BUFFER* pbHdr);


#endif //  __CIRCULAR_BUFF_H_


