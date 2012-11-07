
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
ZG2100 NetServices high-level interface : configuration & setup
*/
//Donatien Garnier 2010

#ifndef ZG_IF_H
#define ZG_IF_H

#include "zg_defs.h"

///Scans for available networks on given \a channel.
void zg_scan(byte channel);

///Will be called on scan completion, for now it is just a debug dump.
void zg_on_scan_results(byte* buf, int len);

///Sets the SSID of the network to be joined.
void zg_set_ssid(const char* ssid);

///Sets WEP key.
void zg_set_wep_key(const byte* key, int len);

///Sets WPA passphrase (will compute PSK key and set it).
void zg_set_wpa_pass(const char* pass);

///On completion of the passphrase computation.
void zg_on_psk_key(byte* buf, int len);

///Sets PSK key (preferred to be called directly than recomputing it every time using \a zg_set_wpa_pass).
void zg_set_psk_key(const byte* key, int len);

///Connects to network.
void zg_connect(ZG_BSS_TYPE type, ZG_SECURITY security);

///On connection result.
void zg_on_connect(zg_err result);

///Gets connection result.
zg_err zg_get_connection_result();

///Disconnects from network.
void zg_disconnect();

void hexdump(byte* buffer, int size);

#endif
