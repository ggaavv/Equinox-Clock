/*******************************************************************************

  Summary:
    
  Description:

*******************************************************************************/

/*******************************************************************************
FileName: 	lcd.c
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

#include "system_profile.h"

#if defined (SYS_OUT_ENABLE)

#include "hardware_profile.h"
#include "lcd.h"

#define SAMSUNG_S6A0032		// This LCD driver chip has a different means of entering 4-bit mode.  

// LCDText is a 32 byte shadow of the LCD text.  Write to it and 
// then call LCD_UPDATE() to copy the string into the LCD module.
char LCDText[16*2+1];

/****************************************************************************
  Function:
    void Delay10us( uint32_t tenMicroSecondCounter )

  Description:
    This routine performs a software delay in intervals of 10 microseconds.

  Precondition:
    None

  Parameters:
    uint32_t tenMicroSecondCounter - number of ten microsecond delays
    to perform at once.

  Returns:
    None

  Remarks:
    None
  ***************************************************************************/
void Delay10us( uint32_t tenMicroSecondCounter )
{
    volatile int32_t cyclesRequiredForEntireDelay;    
        
    #if defined(__18CXX)
    
        if (SystemGetInstructionClock() <= 2500000) //for all FCY speeds under 2MHz (FOSC <= 10MHz)
        {
            //26 cycles burned through this path (includes return to caller).
            //For FOSC == 1MHZ, it takes 104us.
            //For FOSC == 4MHZ, it takes 26us
            //For FOSC == 8MHZ, it takes 13us.
            //For FOSC == 10MHZ, it takes 10.5us.
        }
        else
        {
            //14 cycles burned to this point.
            
            //We want to pre-calculate number of cycles required to delay 10us * tenMicroSecondCounter using a 1 cycle granule.
            cyclesRequiredForEntireDelay = (int32_t)(SystemGetInstructionClock()/100000) * tenMicroSecondCounter;
            
            //We subtract all the cycles used up until we reach the while loop below, where each loop cycle count is subtracted.
            //Also we subtract the 22 cycle function return.
            cyclesRequiredForEntireDelay -= (153 + 22);
            
            if (cyclesRequiredForEntireDelay <= 45)
            {
                // If we have exceeded the cycle count already, bail! Best compromise between FOSC == 12MHz and FOSC == 24MHz.
            }    
            else
            {
                //Try as you may, you can't get out of this heavier-duty case under 30us. ;]
                
                while (cyclesRequiredForEntireDelay>0) //153 cycles used to this point.
                {
                    Nop(); //Delay one instruction cycle at a time, not absolutely necessary.
                    cyclesRequiredForEntireDelay -= 42; //Subtract cycles burned while doing each delay stage, 42 in this case.
                }
            }
        }
    
    #elif defined(__C30__) || defined(__PIC32MX__)
    
        if(SystemGetInstructionClock() <= 500000) //for all FCY speeds under 500KHz (FOSC <= 1MHz)
        {
            //10 cycles burned through this path (includes return to caller).
            //For FOSC == 1MHZ, it takes 5us.
            //For FOSC == 4MHZ, it takes 0.5us
            //For FOSC == 8MHZ, it takes 0.25us.
            //For FOSC == 10MHZ, it takes 0.2us.
        }    
        else
        {
            //7 cycles burned to this point.
            
            //We want to pre-calculate number of cycles required to delay 10us * tenMicroSecondCounter using a 1 cycle granule.
            cyclesRequiredForEntireDelay = (int32_t)(SystemGetInstructionClock()/100000)*tenMicroSecondCounter;
            
            #if defined(__C30__)
                //We subtract all the cycles used up until we reach the while loop below, where each loop cycle count is subtracted.
                //Also we subtract the 5 cycle function return.
                cyclesRequiredForEntireDelay -= 44; //(29 + 5) + 10 cycles padding
            #elif defined(__PIC32MX__)
                //We subtract all the cycles used up until we reach the while loop below, where each loop cycle count is subtracted.
                //Also we subtract the 5 cycle function return.
                cyclesRequiredForEntireDelay -= 24; //(19 + 5)
            #endif
            
            if(cyclesRequiredForEntireDelay <= 0)
            {
                // If we have exceeded the cycle count already, bail!
            }
            else
            {   
                while(cyclesRequiredForEntireDelay>0) //19 cycles used to this point.
                {
                    #if defined(__C30__)
                        cyclesRequiredForEntireDelay -= 11; //Subtract cycles burned while doing each delay stage, 12 in this case. Add one cycle as padding.
                    #elif defined(__PIC32MX__)
                        cyclesRequiredForEntireDelay -= 8; //Subtract cycles burned while doing each delay stage, 8 in this case.
                    #endif
                }
            }
        }
    #endif
}

void DelayMs( uint16_t ms )
{
    #if defined(__18CXX)
        
        int32_t cyclesRequiredForEntireDelay;
        
        // We want to pre-calculate number of cycles required to delay 1ms, using a 1 cycle granule.
        cyclesRequiredForEntireDelay = (signed long)(SystemGetInstructionClock()/1000) * ms;
        
        // We subtract all the cycles used up until we reach the while loop below, where each loop cycle count is subtracted.
        // Also we subtract the 22 cycle function return.
        cyclesRequiredForEntireDelay -= (148 + 22);

        if (cyclesRequiredForEntireDelay <= (170+25)) 
        {
            return;     // If we have exceeded the cycle count already, bail!
        }    
        else
        {
            while (cyclesRequiredForEntireDelay > 0) //148 cycles used to this point.
            {
                Nop();                              // Delay one instruction cycle at a time, not absolutely necessary.
                cyclesRequiredForEntireDelay -= 39; // Subtract cycles burned while doing each delay stage, 39 in this case.
            }
        }
        
    #elif defined(__C30__) || defined(__PIC32MX__)
    
        volatile uint8_t i;
        
        while (ms--)
        {
            i = 4;
            while (i--)
            {
                Delay10us( 25 );
            }
        }
    #endif
}


/******************************************************************************
 * Function:        static void LCD_WRITE(int RS, int Data)
 *
 * PreCondition:    None
 *
 * Input:           RS - Register Select - 1:RAM, 0:Config registers
 *					Data - 8 bits of data to write
 *
 * Output:          None
 *
 * Side Effects:    None
 *
 * Overview:        Controls the Port I/O pins to cause an LCD write
 *
 * Note:            None
 *****************************************************************************/
void LCD_WRITE(int RS, int Data)
{
	#if defined(LCD_DATA_TRIS)
		LCD_DATA_TRIS = 0x00;
	#else
		LCD_DATA0_TRIS = 0;
		LCD_DATA1_TRIS = 0;
		LCD_DATA2_TRIS = 0;
		LCD_DATA3_TRIS = 0;
		#if !defined(FOUR_BIT_MODE)
		LCD_DATA4_TRIS = 0;
		LCD_DATA5_TRIS = 0;
		LCD_DATA6_TRIS = 0;
		LCD_DATA7_TRIS = 0;
		#endif
	#endif
	LCD_RS_TRIS = 0;
	LCD_RD_WR_TRIS = 0;
	LCD_RD_WR_IO = 0;
	LCD_RS_IO = RS;

#if defined(FOUR_BIT_MODE)
	#if defined(LCD_DATA_IO)
		LCD_DATA_IO = Data>>4;
	#else
		LCD_DATA0_IO = ((Data & 0x10) == 0x10);
		LCD_DATA1_IO = ((Data & 0x20) == 0x20);
		LCD_DATA2_IO = ((Data & 0x40) == 0x40);
		LCD_DATA3_IO = ((Data & 0x80) == 0x80);
	#endif
	Nop();					// Wait Data setup time (min 40ns)
	Nop();
	LCD_E_IO = 1;
	Nop();					// Wait E Pulse width time (min 230ns)
	Nop();
	Nop();
	Nop();
	Nop();
	Nop();
	Nop();
	Nop();
	Nop();
	LCD_E_IO = 0;
#endif

	#if defined(LCD_DATA_IO)
		LCD_DATA_IO = Data;
	#else
		LCD_DATA0_IO = ((Data & 0x01) == 0x01);
		LCD_DATA1_IO = ((Data & 0x02) == 0x02);
		LCD_DATA2_IO = ((Data & 0x04) == 0x04);
		LCD_DATA3_IO = ((Data & 0x08) == 0x08);
		#if !defined(FOUR_BIT_MODE)
		LCD_DATA4_IO = ((Data & 0x10) == 0x10);
		LCD_DATA5_IO = ((Data & 0x20) == 0x20);
		LCD_DATA6_IO = ((Data & 0x40) == 0x40);
		LCD_DATA7_IO = ((Data & 0x80) == 0x80);
		#endif
	#endif
	Nop();					// Wait Data setup time (min 40ns)
	Nop();
	LCD_E_IO = 1;
	Nop();					// Wait E Pulse width time (min 230ns)
	Nop();
	Nop();
	Nop();
	Nop();
	Nop();
	Nop();
	Nop();
	Nop();
	LCD_E_IO = 0;

//	// Uncomment if you want the data bus to go High-Z when idle
//	// Note that this may make analog functions work poorly when using 
//	// Explorer 16 revision 5 boards with a 5V LCD on it.  The 5V LCDs have 
//  // internal weak pull ups to 5V on each of the I/O pins, which will 
//  // backfeed 5V weekly onto non-5V tolerant PIC I/O pins.
//	#if defined(LCD_DATA_TRIS)
//		LCD_DATA_TRIS = 0xFF;
//	#else
//		LCD_DATA0_TRIS = 1;
//		LCD_DATA1_TRIS = 1;
//		LCD_DATA2_TRIS = 1;
//		LCD_DATA3_TRIS = 1;
//		#if !defined(FOUR_BIT_MODE)
//		LCD_DATA4_TRIS = 1;
//		LCD_DATA5_TRIS = 1;
//		LCD_DATA6_TRIS = 1;
//		LCD_DATA7_TRIS = 1;
//		#endif
//	#endif
//	LCD_RS_TRIS = 1;
//	LCD_RD_WR_TRIS = 1;
}


/******************************************************************************
 * Function:        void LCDInit(void)
 *
 * PreCondition:    None
 *
 * Input:           None
 *
 * Output:          None
 *
 * Side Effects:    None
 *
 * Overview:        LCDText[] is blanked, port I/O pin TRIS registers are 
 *					configured, and the LCD is placed in the default state
 *
 * Note:            None
 *****************************************************************************/
void LCD_INIT(void)
{
	int i;

	memset(LCDText, ' ', sizeof(LCDText)-1);
	LCDText[sizeof(LCDText)-1] = 0;

	// Setup the I/O pins
	LCD_E_IO = 0;
	LCD_RD_WR_IO = 0;


	#if defined(LCD_DATA_TRIS)
		LCD_DATA_TRIS = 0x00;
	#else
		LCD_DATA0_TRIS = 0;
		LCD_DATA1_TRIS = 0;
		LCD_DATA2_TRIS = 0;
		LCD_DATA3_TRIS = 0;
		#if !defined(FOUR_BIT_MODE)
		LCD_DATA4_TRIS = 0;
		LCD_DATA5_TRIS = 0;
		LCD_DATA6_TRIS = 0;
		LCD_DATA7_TRIS = 0;
		#endif
	#endif
	LCD_RD_WR_TRIS = 0;
	LCD_RS_TRIS = 0;
	LCD_E_TRIS = 0;


	// Wait the required time for the LCD to reset
	DelayMs(40);

	// Set the default function
	// Go to 8-bit mode first to reset the instruction state machine
	// This is done in a loop 3 times to absolutely ensure that we get 
	// to 8-bit mode in case if the device was previously booted into 
	// 4-bit mode and our PIC got reset in the middle of the LCD 
	// receiving half (4-bits) of an 8-bit instruction
	LCD_RS_IO = 0;
	#if defined(LCD_DATA_IO)
		LCD_DATA_IO = 0x03;
	#else
		LCD_DATA0_IO = 1;
		LCD_DATA1_IO = 1;
		LCD_DATA2_IO = 0;
		LCD_DATA3_IO = 0;
		#if !defined(FOUR_BIT_MODE)
		LCD_DATA4_IO = 0;
		LCD_DATA5_IO = 0;
		LCD_DATA6_IO = 0;
		LCD_DATA7_IO = 0;
		#endif
	#endif
	Nop();					// Wait Data setup time (min 40ns)
	Nop();
	for(i = 0; i < 3u; i++)
	{
		LCD_E_IO = 1;
		Delay10us(1);			// Wait E Pulse width time (min 230ns)
		LCD_E_IO = 0;
		DelayMs(2);
	}
	
#if defined(FOUR_BIT_MODE)
	#if defined(SAMSUNG_S6A0032)
		// Enter 4-bit mode (requires only 4-bits on the S6A0032)
		#if defined(LCD_DATA_IO)
			LCD_DATA_IO = 0x02;
		#else
			LCD_DATA0_IO = 0;
			LCD_DATA1_IO = 1;
			LCD_DATA2_IO = 0;
			LCD_DATA3_IO = 0;
		#endif
		Nop();					// Wait Data setup time (min 40ns)
		Nop();
		LCD_E_IO = 1;
		Delay10us(1);			// Wait E Pulse width time (min 230ns)
		LCD_E_IO = 0;
	#else
		// Enter 4-bit mode with two lines (requires 8-bits on most LCD controllers)
		LCD_WRITE(0, 0x28);
	#endif
#else
	// Use 8-bit mode with two lines
	LCD_WRITE(0, 0x38);
#endif
	Delay10us(5);
	
	// Set the entry mode
	LCD_WRITE(0, 0x06);	// Increment after each write, do not shift
	Delay10us(5);

	// Set the display control
	LCD_WRITE(0, 0x0C);		// Turn display on, no cusor, no cursor blink
	Delay10us(5);

	// Clear the display
	LCD_WRITE(0, 0x01);	
	DelayMs(2);
	
}


void LCD_UPDATE_OFFSET(int offset){
	int i, j, k=0;
	if(offset < LCD_CHARS_PER_LINE){
		LCD_WRITE(0, (0x80 | offset));
		DelayMs(2);
		for(i = 0; i < (LCD_CHARS_PER_LINE-offset); i++)
		{
			// Erase the rest of the line if a null char is 
			// encountered (good for printing strings directly)
			if(LCDText[i] == 0u)
			{
				for(j=i; j < 16u; j++)
				{
					LCDText[j] = ' ';
				}
			}
			LCD_WRITE(1, LCDText[i]);
			Delay10us(5);
		}
		k = (LCD_CHARS_PER_LINE-offset);
		offset = 0;
	}	
	
	if(offset!=0)
		offset = offset-LCD_CHARS_PER_LINE;
	
	// Set the address to the second line
	LCD_WRITE(0, (0xC0 | offset));
	Delay10us(5);

	// Output second line
	for(i = 0; i < (LCD_CHARS_PER_LINE-offset); i++)
	{
		// Erase the rest of the line if a null char is 
		// encountered (good for printing strings directly)
		if(LCDText[k] == 0u)
		{
			for(j=k; j < (LCD_CHARS_PER_LINE*LCD_NUM_LINES); j++)
			{
				LCDText[j] = ' ';
			}
		}
		LCD_WRITE(1, LCDText[k++]);
		Delay10us(5);
	}	
	
}

/******************************************************************************
 * Function:        void LCD_UPDATE(void)
 *
 * PreCondition:    LCDInit() must have been called once
 *
 * Input:           LCDText[]
 *
 * Output:          None
 *
 * Side Effects:    None
 *
 * Overview:        Copies the contents of the local LCDText[] array into the 
 *					LCD's internal display buffer.  Null terminators in 
 *					LCDText[] terminate the current line, so strings may be 
 *					printed directly to LCDText[].
 *
 * Note:            None
 *****************************************************************************/
/*
void LCD_UPDATE(void)
{
	int i, j;

	// Go home
	LCD_WRITE(0, 0x02);
	DelayMs(2);

	// Output first line
	for(i = 0; i < 16u; i++)
	{
		// Erase the rest of the line if a null char is 
		// encountered (good for printing strings directly)
		if(LCDText[i] == 0u)
		{
			for(j=i; j < 16u; j++)
			{
				LCDText[j] = ' ';
			}
		}
		LCD_WRITE(1, LCDText[i]);
		Delay10us(5);
	}
	
	// Set the address to the second line
	LCD_WRITE(0, 0xC0);
	Delay10us(5);

	// Output second line
	for(i = 16; i < 32u; i++)
	{
		// Erase the rest of the line if a null char is 
		// encountered (good for printing strings directly)
		if(LCDText[i] == 0u)
		{
			for(j=i; j < 32u; j++)
			{
				LCDText[j] = ' ';
			}
		}
		LCD_WRITE(1, LCDText[i]);
		Delay10us(5);
	}
}
*/

/******************************************************************************
 * Function:        void LCDErase(void)
 *
 * PreCondition:    LCDInit() must have been called once
 *
 * Input:           None
 *
 * Output:          None
 *
 * Side Effects:    None
 *
 * Overview:        Clears LCDText[] and the LCD's internal display buffer
 *
 * Note:            None
 *****************************************************************************/
void LCD_ERASE(void)
{
	// Clear display
	LCD_WRITE(0, 0x01);
	DelayMs(2);

	// Clear local copy
	memset(LCDText, ' ', 32);
}

#endif  // defined (SYS_OUT_ENABLE)



