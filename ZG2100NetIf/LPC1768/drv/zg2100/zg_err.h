
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
ZG2100 driver errors
*/
//Donatien Garnier 2010

#ifndef ZG_ERR_H
#define ZG_ERR_H

#include "zg_defs.h"

///ZG2100 Wi-Fi Module driver error codes
typedef enum __zg_err 
{
  __ZG_MIN = -0xFFFF,
  ZG_RESOURCES, ///< Not enough resources
  ZG_TIMEOUT, ///< Timeout
  ZG_FRAME_ERROR, ///< Framing error
  ZG_AUTH_REFUSED, ///< Authentication refused
  ZG_ASSOC_REFUSED, ///< Association refused
  ZG_IN_PROGRESS, ///< Command is being processed
  ZG_SUPPLICANT_FAILED, ///< Error with WPA-handling
  ZG_UNKNOWN, ///< Unknown error
  ZG_OK = 0 ///< Success

} zg_err;

zg_err zg_errcode(ZG_INT_ERR int_err); //Errcodes conversion

#endif
