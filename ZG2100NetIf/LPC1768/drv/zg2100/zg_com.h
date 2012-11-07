
/*
Copyright (c) 2010 Donatien Garnier (donatiengar [at] gmail [dot] com)
 
Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:
 
The above copyright notice and this permission notice shall be included in
all copies or substantial portions of the Software.
 
THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
THE SOFTWARE.
*/

/**
@file 
ZG2100 Low-level communication functions (SPI, CS, Interrupt)
*/
//Donatien Garnier 2010

#ifndef ZG_COM_H
#define ZG_COM_H

#include "mbed.h"
#include "zg_defs.h"

//Defines

#define ZG_SPI_CS_ON  0 //Active on low
#define ZG_SPI_CS_OFF 1

#define ZG_SPI_RST_ON  0 //Active on low
#define ZG_SPI_RST_OFF 1

//Prototypes
/*
class SPI;
class DigitalOut;
class InterruptIn;
*/
///Opens SPI interface with pins
void zg_com_init(SPI* pSpi, DigitalOut* pCs, /*InterruptIn*/ DigitalIn* pInt, DigitalOut* pNrst);

//Registers Access
///Reads register
uint32_t zg_register_read(uint32_t addr);

///Writes register
void zg_register_write(uint32_t addr, uint32_t reg);

//Indexed Registers Access
///Reads indexed register
uint32_t zg_indexed_register_read(uint32_t addr);

///writes indexed register
void zg_indexed_register_write(uint32_t addr, uint32_t reg);

//Fifos
///Reads FIFO
void zg_fifo_read( byte fifo, byte* pType, byte* pSubtype, byte* buf, int len );

///Writes FIFO (can be chunked)
void zg_fifo_write( byte fifo, byte type, byte subtype, byte* buf, int len, bool start = true, bool stop = true ); //Write by chunks

//Spi

///SPI transfers directions
typedef enum __ZG_SPI_DIR
{
  ZG_SPI_READ, ///Read
  ZG_SPI_WRITE, ///Write
  ZG_SPI_TRF ///Read & Write
} ZG_SPI_DIR;

///Transfers SPI frame
void zg_spi_trf(byte* buf, int len, bool resetCs = true, ZG_SPI_DIR dir = ZG_SPI_TRF);

///Is there an interrupt to serve?
bool zg_is_int();

//Callbacks, must be implemented in zg_drv.c

void zg_on_int(); //On data available interrupt


#endif
