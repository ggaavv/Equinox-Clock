
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
ZG2100 Low-level driver
*/
//Donatien Garnier 2010

#ifndef ZG_DRV_H
#define ZG_DRV_H

#include "zg_defs.h"
#include "zg_err.h"

//Stores relevant data
typedef struct _ZG_DATA
{
  byte mac_addr[6];
  ZG_SYSV sys_version;
  
} ZG_DATA;

//Stores which data is actually available
typedef struct _ZG_DATA_MASK
{
  bool mac_addr;
  bool sys_version;

} ZG_DATA_MASK; //true when the corresponding data field is valid, false otherwise

//Exported Data
extern byte fifo_buf[ZG_FIFO_BUF_SIZE] ZG_MEM; //Big buffer used for fifo transfers
extern ZG_DATA zg_data; //Container for all data received from the chip
extern ZG_DATA_MASK zg_data_mask; //Flags valid data

//Spi intf, Chip Select pin, Interrupt pin
///Initializes driver (zg_com_init must be called before).
zg_err zg_drv_init();

//FIXME: Not implemented, not used (to be deleted?)
void zg_on_data_attach( void (*on_data)() );

///Must be called regularly to process interrupts.
void zg_process(); //Must be called regularly by user

///Processes interrupt. (Called by zg_process if needed)
void zg_int_process(); //Process interrupt

///Can a management request be sent?
bool zg_mgmt_is_busy();

///Sends management request
void zg_mgmt_req(byte subtype, byte* buf, int len, bool close = true);

///Writes additional management data
void zg_mgmt_data(byte* buf, int len, bool close = true);

///Gets parameter
void zg_mgmt_get_param(byte param);

///Sets parameter
void zg_mgmt_set_param(byte param, byte* buf, int len);

///Called on management request result
void zg_on_mgmt_avl(byte subtype, byte* buf, int len); //Data is available

///Called on management event
void zg_on_mgmt_evt(byte subtype, byte* buf, int len); //Management event

///Called when get parameter request completes
void zg_on_mgmt_get_param(byte* buf, int len); //Get param completed

//uint32_t zg_fifo_room();

//Useful to be split in several function because Lwip stores buffers in chunks
///Sends Data (start)
void zg_send_start();
///Sends Data (main)
void zg_send(byte* buf, int len);
///Sends Data (end)
void zg_send_end();

///Can more data be sent?
bool zg_send_is_busy();

//Callbacks implemented in zg_if
void zg_on_scan_results(byte* buf, int len);
void zg_on_psk_key(byte* buf, int len);
void zg_on_connect(zg_err result);

//Handled by zg_net
///Data received
void zg_on_input(byte* buf, int len); 

//Callbacks from zg_com
void zg_on_int(); //On data available interrupt

#endif
