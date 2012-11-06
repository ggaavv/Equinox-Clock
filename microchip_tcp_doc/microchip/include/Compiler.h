/*********************************************************************
 *
 *                  Compiler and hardware specific definitions
 *
 *********************************************************************
 * FileName:        Compiler.h
 * Dependencies:    None
 * Processor:       PIC24F, PIC24H, PIC24E, dsPIC30F, dsPIC33F, 
 *					dsPIC33E, PIC32
 * Compiler:        Microchip C32 v1.00 or higher
 *					Microchip C30 v3.01 or higher
 *					Microchip C18 v3.13 or higher
 *					HI-TECH PICC-18 PRO 9.63 or higher
 * Company:         Microchip Technology, Inc.
 *
 * Software License Agreement
 *
 * Copyright (C) 2002-2010 Microchip Technology Inc.  All rights 
 * reserved.
 *
 * Microchip licenses to you the right to use, modify, copy, and 
 * distribute: 
 * (i)  the Software when embedded on a Microchip microcontroller or 
 *      digital signal controller product ("Device") which is 
 *      integrated into Licensee's product; or
 * (ii) ONLY the Software driver source files ENC28J60.c and 
 *      ENC28J60.h ported to a non-Microchip device used in 
 *      conjunction with a Microchip ethernet controller for the 
 *      sole purpose of interfacing with the ethernet controller. 
 *
 * You should refer to the license agreement accompanying this 
 * Software for additional information regarding your rights and 
 * obligations.
 *
 * THE SOFTWARE AND DOCUMENTATION ARE PROVIDED "AS IS" WITHOUT 
 * WARRANTY OF ANY KIND, EITHER EXPRESS OR IMPLIED, INCLUDING WITHOUT 
 * LIMITATION, ANY WARRANTY OF MERCHANTABILITY, FITNESS FOR A 
 * PARTICULAR PURPOSE, TITLE AND NON-INFRINGEMENT. IN NO EVENT SHALL 
 * MICROCHIP BE LIABLE FOR ANY INCIDENTAL, SPECIAL, INDIRECT OR 
 * CONSEQUENTIAL DAMAGES, LOST PROFITS OR LOST DATA, COST OF 
 * PROCUREMENT OF SUBSTITUTE GOODS, TECHNOLOGY OR SERVICES, ANY CLAIMS 
 * BY THIRD PARTIES (INCLUDING BUT NOT LIMITED TO ANY DEFENSE 
 * THEREOF), ANY CLAIMS FOR INDEMNITY OR CONTRIBUTION, OR OTHER 
 * SIMILAR COSTS, WHETHER ASSERTED ON THE BASIS OF CONTRACT, TORT 
 * (INCLUDING NEGLIGENCE), BREACH OF WARRANTY, OR OTHERWISE.
 *
 *
 * Date         Comment
 *~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
 * 10/03/2006	Original, copied from old Compiler.h
 * 11/07/2007	Reorganized and simplified
 * 03/31/2010	Removed dependency on WORD and DWORD typedefs
 * 04/14/2010   Added defines to uniquely identify each compiler
 ********************************************************************/
#ifndef __COMPILER_H
#define __COMPILER_H

// Include proper device header file
#if (defined(__PIC24F__) || defined(__PIC24FK__)) && defined(__C30__)	// Microchip C30 compiler
	// PIC24F processor
    #define COMPILER_MPLAB_C30
	#include <p24Fxxxx.h>
#elif defined(__PIC24H__) && defined(__C30__)	// Microchip C30 compiler
	// PIC24H processor
    #define COMPILER_MPLAB_C30
	#include <p24Hxxxx.h>
#elif defined(__PIC24E__) && defined(__C30__)	// Microchip C30 compiler
	// PIC24E processor
    #define COMPILER_MPLAB_C30
	#include <p24Exxxx.h>
#elif defined(__dsPIC33F__) && defined(__C30__)	// Microchip C30 compiler
	// dsPIC33F processor
    #define COMPILER_MPLAB_C30
	#include <p33Fxxxx.h>
#elif defined(__dsPIC33E__) && defined(__C30__)	// Microchip C30 compiler
	// dsPIC33E processor
    #define COMPILER_MPLAB_C30
	#include <p33Exxxx.h>
#elif defined(__dsPIC30F__) && defined(__C30__)	// Microchip C30 compiler
	// dsPIC30F processor
    #define COMPILER_MPLAB_C30
	#include <p30fxxxx.h>
#elif defined(__C30__)		// Microchip C30 compiler, but targeting "generic-16bit" processor.
    #define COMPILER_MPLAB_C30
	#include <p30sim.h>
	// Define some useful inline assembly functions which are normally in the 
	// processor header files, but absent from the generic p30sim.h file.
	#if !defined(Nop)
		#define Nop()    __builtin_nop()
		#define ClrWdt() {__asm__ volatile ("clrwdt");}
		#define Sleep()  {__asm__ volatile ("pwrsav #0");}
		#define Idle()   {__asm__ volatile ("pwrsav #1");}
	#endif
#elif defined(__PIC32MX__)	// Microchip C32 compiler
	#if !defined(__C32__)
		#define __C32__
	#endif
    #define COMPILER_MPLAB_C32
	#include <p32xxxx.h>
	#include <plib.h>
#else
	#error Unknown processor or compiler.  See Compiler.h
#endif

#include <stdio.h>
#include <stdlib.h>
#include <string.h>


// Base pointer types for given architecture
#if defined(__PIC32MX__)
	#define PTR_BASE		unsigned long
#elif defined(__C30__)
	#define PTR_BASE		unsigned short
#endif



	// 16-bit specific defines (PIC24F, PIC24H, dsPIC30F, dsPIC33F)
	#if defined(__C30__)
		#define Reset()				asm("reset")
        #define FAR                 __attribute__((far))
	#endif

	// 32-bit specific defines (PIC32)
	#if defined(__PIC32MX__)
		#define persistent
		#define far
        #define FAR
		#define Reset()				SoftReset()
		#define ClrWdt()			(WDTCONSET = _WDTCON_WDTCLR_MASK)

		// MPLAB C Compiler for PIC32 MCUs version 1.04 and below don't have a 
		// Nop() function. However, version 1.05 has Nop() declared as _nop().
		#if !defined(Nop) && (__C32_VERSION__ <= 104)
			#define Nop()				asm("nop")
		#endif
	#endif
//#endif



#endif
