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

#include "ios.h"
//#include "machine.h"

/*
        Pin Definitions
*/

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
/*
        X Stepper
*/
#define x_enable() digital_write(X_ENABLE_PORT, X_ENABLE_PIN, 0)
#define x_disable() digital_write(X_ENABLE_PORT, X_ENABLE_PIN, 1)
#define x_step() digital_write(X_STEP_PORT, X_STEP_PIN, 1)
#define x_unstep() digital_write(X_STEP_PORT, X_STEP_PIN, 0)
#define x_direction(dir) digital_write(X_DIR_PORT, X_DIR_PIN, dir)
#define x_min() ((digital_read(X_MIN_PORT, X_MIN_PIN))?0:1)
#define x_max() ((digital_read(X_MAX_PORT, X_MAX_PIN))?0:1)

/*
        Y Stepper
*/
#define y_enable() digital_write(Y_ENABLE_PORT, Y_ENABLE_PIN, 0)
#define y_disable() digital_write(Y_ENABLE_PORT, Y_ENABLE_PIN, 1)
#define y_step() digital_write(Y_STEP_PORT, Y_STEP_PIN, 1)
#define y_unstep() digital_write(Y_STEP_PORT, Y_STEP_PIN, 0)
#define y_direction(dir) digital_write(Y_DIR_PORT, Y_DIR_PIN, dir)
#define y_min() ((digital_read(Y_MIN_PORT, Y_MIN_PIN))?0:1)
#define y_max() ((digital_read(Y_MAX_PORT, Y_MAX_PIN))?0:1)

/*
        Z Stepper
*/
#define z_enable() digital_write(Z_ENABLE_PORT, Z_ENABLE_PIN, 0)
#define z_disable() digital_write(Z_ENABLE_PORT, Z_ENABLE_PIN, 1)
#define z_step() digital_write(Z_STEP_PORT, Z_STEP_PIN, 1)
#define z_unstep() digital_write(Z_STEP_PORT, Z_STEP_PIN, 0)
#define z_direction(dir) digital_write(Z_DIR_PORT, Z_DIR_PIN, dir)
#define z_min() ((digital_read(Z_MIN_PORT, Z_MIN_PIN))?0:1)
#define z_max() ((digital_read(Z_MAX_PORT, Z_MAX_PIN))?0:1)

/*
        Extruder
*/
#define e_enable() digital_write(E_ENABLE_PORT, E_ENABLE_PIN, 0)
#define e_disable() digital_write(E_ENABLE_PORT, E_ENABLE_PIN, 1)
#define e_step() digital_write(E_STEP_PORT, E_STEP_PIN, 1)
#define e_unstep() digital_write(E_STEP_PORT, E_STEP_PIN, 0)
#define e_direction(dir) digital_write(E_DIR_PORT, E_DIR_PIN, dir)

#define extruder_heater_on() digital_write(EXTRUDER_0_HEATER_PORT, EXTRUDER_0_HEATER_PIN, HIGH);
#define extruder_heater_off() digital_write(EXTRUDER_0_HEATER_PORT, EXTRUDER_0_HEATER_PIN, LOW);

#define extruder_fan_on() digital_write(EXTRUDER_0_FAN_PORT, EXTRUDER_0_FAN_PIN, HIGH);
#define extruder_fan_off() digital_write(EXTRUDER_0_FAN_PORT, EXTRUDER_0_FAN_PIN, LOW);

/*
        Heated Bed
*/
#define heated_bed_on() digital_write(HEATED_BED_0_HEATER_PORT, HEATED_BED_0_HEATER_PIN, HIGH);
#define heated_bed_off() digital_write(HEATED_BED_0_HEATER_PORT, HEATED_BED_0_HEATER_PIN, LOW);

#define power_on()      if (0) {}
#define power_off()     if (0) {}


#endif  /* _PINOUT_H */
