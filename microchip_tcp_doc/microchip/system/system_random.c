/*******************************************************************************

  Summary: System Debugging and messaging interface
    
  Description:

*******************************************************************************/

/*******************************************************************************
FileName: 	system_random.c
Copyright ?2012 released Microchip Technology Inc.  All rights
reserved.

Microchip licenses to you the right to use, modify, copy and distribute
Software only when embedded on a Microchip microcontroller or digital signal
controller that is integrated into your product or third party product
(pursuant to the sublicense terms in the accompanying license agreement).

You should refer to the license agreement accompanying this Software for
additional information regarding your rights and obligations.

SOFTWARE AND DOCUMENTATION ARE PROVIDED “AS IS?WITHOUT WARRANTY OF ANY KIND,
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
#define __RANDOM_C

#include "system/system_services.h"
#include "system_profile.h"
#include "Compiler.h"
#if defined(SYS_RANDOM_ENABLE)

#include "common/hashes.h"
#include "common/lfsr.h"

#include "system_random_private.h"

static HASH_SUM randHash;
static uint8_t output[20];
static uint8_t bCount;


/*********************************************************************
 * Function:        void _SYS_RANDOM_INIT(void)
 *
 * PreCondition:    None
 *
 * Input:           None
 *
 * Output:          Random number generator is initialized.
 *
 * Side Effects:    None
 *
 * Overview:        Sets up the random generator structure.
 *
 * Note:            Data may not be secure until several packets have
 *					been received.
 ********************************************************************/
bool _SYS_RANDOM_INIT(void)
{

	unsigned char i;
	unsigned long dw;
	
	SHA1Initialize(&randHash);
	
	// Add some starting entropy to the pool.  This is slow.
	for(i = 0; i < 5; i++)
	{
		dw = SYS_GENERATE_RANDOM_DWORD();
		SYS_RANDOM_ADD(((uint8_t*)&dw)[0]);
		SYS_RANDOM_ADD(((uint8_t*)&dw)[1]);
		SYS_RANDOM_ADD(((uint8_t*)&dw)[2]);
		SYS_RANDOM_ADD(((uint8_t*)&dw)[3]);
	}
		
	bCount = 20;
	
	return true;
}

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
uint8_t SYS_RANDOM_GET(void)
{
	if(bCount >= 20u)
	{//we need to get new random bytes
		SHA1Calculate(&randHash, output);
		SYS_RANDOM_ADD(output[0]);
		bCount = 0;
	}
	
	//return the random byte
	return output[bCount++];
}

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
void SYS_RANDOM_ADD(uint8_t data)
{
	SYS_TICK dTemp;
	
	SHA1AddData(&randHash, &data, 1);
	dTemp = SYS_TICK_Get();
	SHA1AddData(&randHash, (uint8_t*)&dTemp, 1);
	
	bCount = 20;
}

/*********************************************************************
 * Function:        void _SYS_RANDOM_DE_INIT(void)
 *
 * PreCondition:    None
 *
 * Input:           None
 *
 * Output:          Random number generator is deinitialized.
 *
 * Side Effects:    None
 *
 * Overview:        Takes down Random number generator.
 *
 * Note:            Data may not be secure until several packets have
 *					been received.
 ********************************************************************/


void _SYS_RANDOM_DE_INIT(void)
{

}

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
uint32_t SYS_GENERATE_RANDOM_DWORD(void)
{
	uint8_t vBitCount;
	uint16_t w, wTime, wLastValue;
	uint32_t dwTotalTime;
	union
	{
		uint32_t	dw;
		uint16_t	w[2];
	} randomResult;

	uint16_t AD1CON1Save, AD1CON2Save, AD1CON3Save;
	uint16_t T1CONSave, PR1Save;

	// Save hardware SFRs
	AD1CON1Save = AD1CON1;
	AD1CON2Save = AD1CON2;
	AD1CON3Save = AD1CON3;
	T1CONSave = T1CON;
	PR1Save = PR1;

	// Set up Timer and A/D converter module
	AD1CON1 = 0x0000;		// Turn off the ADC so we can write to it
	AD1CON3 = 0x9F00;		// Frc A/D clock, 31 Tad acquisition
	AD1CON2 = 0x003F;		// Interrupt after every 16th sample/convert
	AD1CON1 = 0x80E4;		// Turn on the A/D module, auto-convert
	T1CON = 0x8000;			// TON = 1, no prescalar
	PR1 = 0xFFFF;			// Don't clear timer early
	vBitCount = 0;
	dwTotalTime = 0;
	wLastValue = 0;
	randomResult.dw = LFSRRand();
	while(1)
	{
		SYS_WDT_Clear();
		#if defined(__C30__)
			while(!IFS0bits.AD1IF);
		#else
			while(!IFS1bits.AD1IF);
		#endif
		wTime = TMR1;
		TMR1 = 0x0000;

		#if defined(__C30__)
			IFS0bits.AD1IF = 0;
		#else
			IFS1CLR = _IFS1_AD1IF_MASK;
		#endif
		w = LFSRRand();
	
		// Wait no longer than 1 second obtaining entropy
		dwTotalTime += wTime;
		if(dwTotalTime >= SYS_CLK_ClockGet())
		{
			randomResult.w[0] ^= LFSRRand();
			randomResult.w[1] ^= LFSRRand();
			break;
		}
	
		// Keep sampling if minimal entropy was likely obtained this round
		if(wLastValue == wTime)
			continue;
	
		// Add this entropy into the pseudo random number generator by reseeding
		LFSRSeedRand(w + (wLastValue - wTime));
		wLastValue = wTime;
	
		// Accumulate at least 32 bits of randomness over time
		randomResult.dw <<= 1;
		if(LFSRRand() & 0x0080)
			randomResult.w[0] |= 0x1;
	
		// See if we've collected a fair amount of entropy and can quit early
		if(++vBitCount == 0u)
			break;
	}


	// Restore hardware SFRs
	AD1CON1 = 0x0000;		// Turn off the ADC so we can write to it
	AD1CON3 = AD1CON3Save;
	AD1CON2 = AD1CON2Save;
	AD1CON1 = AD1CON1Save;
	T1CON = T1CONSave;
	PR1 = PR1Save;

	return randomResult.dw;
}

#endif  // defined(SYS_RANDOM_ENABLE)

