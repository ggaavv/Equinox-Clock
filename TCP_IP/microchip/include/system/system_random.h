/*******************************************************************************

Summary: System Random interface

Description:

 *******************************************************************************/

/*******************************************************************************
FileName: 	system_random.h
Copyright © 2012 released Microchip Technology Inc.  All rights
reserved.

Microchip licenses to you the right to use, modify, copy and distribute
Software only when embedded on a Microchip microcontroller or digital signal
controller that is integrated into your product or third party product
(pursuant to the sublicense terms in the accompanying license agreement).

You should refer to the license agreement accompanying this Software for
additional information regarding your rights and obligations.

SOFTWARE AND DOCUMENTATION ARE PROVIDED “AS IS” WITHOUT WARRANTY OF ANY KIND,
EITHER EXPRESS OR IMPLIED, INCLUDING WITHOUT LIMITATION, ANY WARRANTY OF
MERCHANTABILITY, TITLE, NON-INFRINGEMENT AND FITNESS FOR A PARTICULAR PURPOSE.
IN NO EVENT SHALL MICROCHIP OR ITS LICENSORS BE LIABLE OR OBLIGATED UNDER
CONTRACT, NEGLIGENCE, STRICT LIABILITY, CONTRIBUTION, BREACH OF WARRANTY, OR
OTHER LEGAL EQUITABLE THEORY ANY DIRECT OR INDIRECT DAMAGES OR EXPENSES
INCLUDING BUT NOT LIMITED TO ANY INCIDENTAL, SPECIAL, INDIRECT, PUNITIVE OR
CONSEQUENTIAL DAMAGES, LOST PROFITS OR LOST DATA, COST OF PROCUREMENT OF
SUBSTITUTE GOODS, TECHNOLOGY, SERVICES, OR ANY CLAIMS BY THIRD PARTIES
(INCLUDING BUT NOT LIMITED TO ANY DEFENSE THEREOF), OR OTHER SIMILAR COSTS.
 *******************************************************************************/


#ifndef __SYS_RANDOM_H_
#define __SYS_RANDOM_H_



/*********************************************************************
 * Function:        uint8_t SYS_RANDOM_GET(void)
 *
 * PreCondition:    None
 *
 * Input:           None
 *
 * Output:          A random byte is generated
 *
 * Side Effects:    None
 *
 * Overview:        None
 *
 * Note:            None
 ********************************************************************/
uint8_t SYS_RANDOM_GET(void);

/*********************************************************************
 * Function:        void SYS_RANDOM_ADD(uint8_t data)
 *
 * PreCondition:    None
 *
 * Input:           a random byte to add to the seed
 *
 * Output:          None
 *
 * Side Effects:    None
 *
 * Overview:        Hashes the byte and an asynchronous timer value
 *
 * Note:            None
 ********************************************************************/
void SYS_RANDOM_ADD(uint8_t data);


/*****************************************************************************
  Function:
	uint32_t SYS_GENERATE_RANDOM_DWORD(void)

  Summary:
	Generates a random uint32_t.

  Description:
	This function generates a random 32-bit integer.  It collects
	randomness by comparing the A/D converter's internal R/C oscillator
	clock with our main system clock.  By passing collected entropy to the
	LFSRSeedRand()/LFSRRand() functions, the output is normalized (deskewed) 
	in the hopes of meeting statistical randomness tests.

  Precondition:
	None

  Parameters:
	None

  Returns:
  	Random 32-bit number.
  	
  Side Effects:
	This function uses the A/D converter (and so you must disable 
	interrupts if you use the A/D converted in your ISR).  The LFSRRand() 
	function will be reseeded, and Timer1 (PIC24, 
	dsPIC, and PIC32) will be used.  TMR#H:TMR#L will have a new value.
	Note that this is the same timer used by the Tick module.
	
  Remarks:
	This function times out after 1 second of attempting to generate the 
	random uint32_t.  In such a case, the output may not be truly random.  
	Typically, this function executes in around 500,000 instruction cycles.
	
	The intent of this function is to produce statistically random and
	cryptographically secure random number.  Whether or not this is true on
	all (or any) devices/voltages/temperatures is not tested.
  ***************************************************************************/
uint32_t        SYS_GENERATE_RANDOM_DWORD(void);





#endif //__SYS_RANDOM_H_
