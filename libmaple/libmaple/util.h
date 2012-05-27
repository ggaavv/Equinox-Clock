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
 *  @file util.h
 *
 *  @brief Various macros and utility procedures.
 */

/* Generally "useful" utility procedures  */
#ifndef _UTIL_H_
#define _UTIL_H_

#include "libmaple.h"

#define BIT(shift)                     (1UL << (shift))
#define BIT_MASK_SHIFT(mask, shift)    ((mask) << (shift))

/* Return bits m to n of x  */
#define GET_BITS(x, m, n) ((((uint32)x) << (31 - (n))) >> ((31 - (n)) + (m)))

/* Bit-banding macros  */
/* Convert SRAM address */
#define BITBAND_SRAM(a,b) ((BITBAND_SRAM_BASE+(a-BITBAND_SRAM_REF)*32+(b*4)))
/* Convert PERI address */
#define BITBAND_PERI(a,b) ((BITBAND_PERI_BASE+(a-BITBAND_PERI_REF)*32+(b*4)))

#define REG_SET(reg, val)         (*(volatile uint32*)(reg)  = (val))
#define REG_SET_BIT(reg, bit)     (*(volatile uint32*)(reg) |= BIT(bit))
#define REG_CLEAR_BIT(reg, bit)   (*(volatile uint32*)(reg) &= ~BIT(bit))
#define REG_SET_MASK(reg, mask)   (*(volatile uint32*)(reg) |= (uint32)(mask))
#define REG_CLEAR_MASK(reg, mask) (*(volatile uint32*)(reg) &= (uint32)~(mask))

#define REG_GET(reg)              *(volatile uint32*)(reg)

#define __set_bits(addr, mask)    *(volatile uint32*)(addr) |= (uint32)(mask)
#define __clear_bits(addr, mask) (*(volatile uint32*)(addr) &= (uint32)~(mask))
#define __get_bits(addr, mask)   (*(volatile uint32*)(addr) & (uint32)(mask))

#define __read(reg)               *(volatile uint32*)(reg)
#define __write(reg, value)       *(volatile uint32*)(reg) = (value)

#define IS_POWER_OF_TWO(v)  (v && !(v & (v - 1)))

#ifdef __cplusplus
extern "C"{
#endif

void _fail(const char*, int, const char*);
void throb(void);

#ifdef __cplusplus
} // extern "C"
#endif

/* Asserts for sanity checks, redefine DEBUG_LEVEL in libmaple.h to
 * compile out these checks */

#if DEBUG_LEVEL >= DEBUG_ALL
#define ASSERT(exp)                              \
    if (exp) {                                   \
    } else {                                     \
        _fail(__FILE__, __LINE__, #exp);         \
    }

#else
#define ASSERT(exp) (void)((0))
#endif

#if DEBUG_LEVEL >= DEBUG_FAULT
#define ASSERT_FAULT(exp)                       \
    if (exp) {                                  \
    } else {                                    \
        _fail(__FILE__, __LINE__, #exp);        \
    }

#else
#define ASSERT_FAULT(exp) (void)((0))
#endif

#endif

