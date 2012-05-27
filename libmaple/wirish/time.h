/******************************************************************************
 * The MIT License
 *
 * Copyright (c) 2010 Perry Hung.
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
 *****************************************************************************/

/**
 *  @brief Timing and delay functions.
 */

#ifndef _TIME_H
#define _TIME_H

#ifdef __cplusplus
extern "C"{
#endif

#include "nvic.h"
#include "systick.h"
#include "boards.h"

#define US_PER_MS               1000

extern volatile uint32 systick_timer_millis;

/* time in milliseconds since boot  */
static inline uint32 millis(void) {
    return systick_timer_millis;
}

/* Time in microseconds since boot  */
static inline uint32 micros(void) {
    uint32 ms;
    uint32 cycle_cnt;
    uint32 res;

    nvic_globalirq_disable();

    cycle_cnt = systick_get_count();
    ms = millis();

    nvic_globalirq_enable();

    /* MAPLE_RELOAD_VAL is 1 less than the number of cycles it actually
       takes to complete a systick reload */
    res = (ms * US_PER_MS) +
        (MAPLE_RELOAD_VAL + 1 - cycle_cnt)/CYCLES_PER_MICROSECOND;

    return res;
}

void delay(unsigned long ms);
void delayMicroseconds(uint32 us);

#ifdef __cplusplus
} // extern "C"
#endif


#endif

