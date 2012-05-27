//

#include <string.h>

#include "circular_buff.h"

// init a circular buffer
void CircularBuffInit(unsigned char* buff, int size, CIRCULAR_BUFFER* pbHdr)
{
	pbHdr->start=buff;
	pbHdr->end=buff+size;
	pbHdr->rd=pbHdr->wr=buff;
}


// adds a byte an updates appropriately
int CircularBuffAddByte(CIRCULAR_BUFFER* pbHdr, int newB)
{
	unsigned char* p=pbHdr->wr+1;
	if(p==pbHdr->end)
	{
		p=pbHdr->start;
	}
	
	if(p!=pbHdr->rd)
	{
		*(pbHdr->wr)=(unsigned char)newB;
		pbHdr->wr=p;	// update the wr ptr
        return 1;
	}	

    return 0;   // buffer full
}

// adds an array of bytes an updates appropriately
// returns number of bytes added to the buffer
int CircularBuffAddArray(CIRCULAR_BUFFER* pbHdr, unsigned char* source, int sourceSize)
{
	unsigned char*	rd, *wr;
	int endSpace, begSpace, writeBytes;


	wr=pbHdr->wr+1;	// this might change beneath our feet
	rd=pbHdr->rd;	// for updating purposes only

    if(wr == rd)
    {   // full
        endSpace = begSpace = 0;
    }
    else if(wr > rd)
    {
        endSpace = pbHdr->end - wr;
        begSpace = rd - pbHdr->start;
    }
    else
    {
        endSpace = pbHdr->rd - wr;
        begSpace = 0;
    }

	if(sourceSize>endSpace+begSpace)
	{	// cannot copy more than avlbl
		sourceSize=endSpace+begSpace;
	}
       
    writeBytes = 0;	
	if(endSpace)
	{
		writeBytes=sourceSize>endSpace?endSpace:sourceSize;
        memcpy(wr, source, writeBytes);
        source+=writeBytes;
		sourceSize-=writeBytes;
		wr+=writeBytes;
		if(wr==pbHdr->end)
		{
			wr=pbHdr->start;
		}
	}

	if(sourceSize)
	{
        memcpy(wr, source, sourceSize);	// copy what's left
		writeBytes+=sourceSize;
		wr+=sourceSize;
	}

	pbHdr->wr=wr;	// update the write pointer

	return writeBytes;
}

// reads the data to an user buffer
// and flushes it from the buffer
// returns number of bytes copied
// if the dest == 0, it just flushes the buffer
int CircularBuffRead(CIRCULAR_BUFFER* pbHdr, unsigned char* dest, int destSize)
{
	unsigned char*	rd, *wr;
	int endBytes, begBytes, copyBytes;


	wr=pbHdr->wr;	// this might change beneath our feet
	rd=pbHdr->rd;	// for updating purposes only

	if((endBytes=wr-rd)==0)
	{	// empty
		begBytes=0;
	}
	else if(endBytes>0 )
	{	// bytes just ahead
		begBytes=0;
	}
	else
	{	// some ahead, some at the beg of the buff
		endBytes=pbHdr->end-rd;
		begBytes=wr-pbHdr->start;
	}
	
	copyBytes=0;
	if(destSize>endBytes+begBytes)
	{	// cannot copy more than avlbl
		destSize=endBytes+begBytes;
	}
	
	if(endBytes)
	{
		copyBytes=destSize>endBytes?endBytes:destSize;
        if(dest)
        {
            memcpy(dest, rd, copyBytes);
            dest+=copyBytes;
        }
		destSize-=copyBytes;
		rd+=copyBytes;
		if(rd==pbHdr->end)
		{
			rd=pbHdr->start;
		}
	}

	if(destSize)
	{
        if(dest)
        {
            memcpy(dest, rd, destSize);	// copy what's left
        }
		copyBytes+=destSize;
		rd+=destSize;
	}

	pbHdr->rd=rd;	// update the read pointer

	return copyBytes;
}


// returns number of available bytes in the buffer
int  CircularBuffAvlblBytes(CIRCULAR_BUFFER* pbHdr)
{
	int nBytes=(pbHdr->wr-pbHdr->rd);
	if(nBytes<0)
	{
		nBytes+=pbHdr->end-pbHdr->start;
	}
	return nBytes-1;
}


