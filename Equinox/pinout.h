/* Copyright (C) 2009-2010 Michael Moon aka Triffid_Hunter   */
/* Copyright (c) 2011 Jorge Pinto - casainho@gmail.com       */
/* All rights reserved.

   Redistribution and use in source and binary forms, with or without
   modification, are permitted provided that the following conditions are met:

   * Redistributions of source code must retain the above copyright
     notice, this list of conditions and the following disclaimer.
   * Redistributions in binary form must reproduce the above copyright
     notice, this list of conditions and the following disclaimer in
     the documentation and/or other materials provided with the
     distribution.
   * Neither the name of the copyright holders nor the names of
     contributors may be used to endorse or promote products derived
     from this software without specific prior written permission.

  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
  ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
  LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
  CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
  SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
  INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
  CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
  ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
  POSSIBILITY OF SUCH DAMAGE.
*/

#ifndef _PINOUT_H
#define _PINOUT_H

//#include "ios.h"

/*
        LED Pins
*/

#define LED_CS_PORT			0
#define LED_CS_PIN			(1 << 16)
#define LED_LE_PORT			0
#define LED_LE_PIN			(1 << 19)
#define LED_SCK_PORT     	0
#define LED_SCK_PIN			(1 << 15)
#define LED_MISO_PORT		0
#define LED_MISO_PIN		(1 << 17)
#define LED_MOSI_PORT		0
#define LED_MOSI_PIN		(1 << 18)

/*
        Wifi Pins
*/

#define WF_CS_PORT			0
#define WF_CS_PIN			(1 <<  6)
#define WF_RESET_PORT		0
#define WF_RESET_PIN		(1 << 11)
#define WF_HIBERNATE_PORT	0
#define WF_HIBERNATE_PIN	(1 << 10)
#define WF_SCK_PORT     	0
#define WF_SCK_PIN			(1 <<  7)
#define WF_MISO_PORT		0
#define WF_MISO_PIN			(1 <<  8)
#define WF_MOSI_PORT		0
#define WF_MOSI_PIN			(1 <<  9)

#define MRF24W_SPI_CHN		1
#define MRF24W_INT		1



#endif  /* _PINOUT_H */
