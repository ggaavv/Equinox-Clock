/*
ShiftPWM.h - Library for Arduino to PWM many outputs using shift registers - Version 1
Copyright (c) 2011 Elco Jacobs, Technical University of Eindhoven, department of 
Industrial Design, Electronics Atelier. All right reserved.

This library is free software; you can redistribute it and/or
modify it under the terms of the GNU Lesser General Public
License as published by the Free Software Foundation; either
version 2.1 of the License, or (at your option) any later version.

This library is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
Lesser General Public License for more details.

You should have received a copy of the GNU Lesser General Public
License along with this library; if not, write to the Free Software
Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA
*/

/* 

This library is intended to control the outputs of a chain of shift registers. 
It uses software PWM to control the duty cycle of all shift register outputs.

This code is optimized for speed: the interrupt duration is minimal, so that 
the load of the interrupt on your program is minimal (or the amount of registers
and brightness levels is maximal. The SPI is used to send data to the shift regisers,
while the CPU is already calculating the next byte to send out.

Timer1 (16 bit) is used, unless it is already in use by the servo library. 
Then, timer2 (8 bit) will be configured as the interrupt timer, with a prescaler for 
the highest possible precision.

A timer interrupt is configured by ShiftPWM.Start(pwmFrequency,maxBrightness)
The interrupt frequency is set to pwmFrequency * (maxBrightness+1)
Each interrupt all duty cycles are compared to the counter and the corresponding pin
is written 1 or 0 based on the result. Then the counter is increased by one.

The duration of the interrupt depends on the number of shift registers (N).
T = 97 + 43*N (worst case)

The load of the interrupt function on your program can be calculated:
L = Interrupt frequency * interrupt duration / clock frequency
L = F*(Bmax+1)*(97+43*N)/F_CPU 
The duration also depends on the number of brightness levels, but the impact is minimal.


The following functions are used:

ShiftPWM.Start(int ledFrequency, int max_Brightness)		Enable ShiftPWM with desired frequency and brightness levels
SetAmountOfRegisters(int newAmount)						Set or change the amount of output registers. Can be changed at runtime.
PrintInterruptLoad(void)									Print information on timer usage, frequencies and interrupt load
void OneByOne(void)										Fade in and fade out all outputs slowly
void OneByOneFast(void)									Fade in and fade out all outputs fast
void SetOne(int pin, unsigned char value)					Set the duty cycle of one output
void SetAll(unsigned char value)							Set all outputs to the same duty cycle

SetGroupOf2(int group, unsigned char v0, unsigned char v1);
SetGroupOf3(int group, unsigned char v0, unsigned char v1, unsigned char v2);
SetGroupOf4(int group, unsigned char v0, unsigned char v1, unsigned char v2, unsigned char v3);
SetGroupOf5(int group, unsigned char v0, unsigned char v1, unsigned char v2, unsigned char v3, unsigned char v4);
--> Set a group of outputs to the given values. SetGroupOf3 is useful for RGB LED's. Each LED will be a group.

*/

#ifndef ShiftPWM_H
#define ShiftPWM_H

//extern SoftwareSerial mySerial;

#include "pins_arduino_compile_time.h" // My own version of pins arduino, which does not define the arrays in program memory
#include <WProgram.h>
#include "CShiftPWM.h"

// These should be defined in the file where ShiftPWM.h is included.
extern const int ShiftPWM_latchPin;
extern const bool ShiftPWM_invertOutputs;
//const int ShiftPWM_latchPin=10; //atmega1284p PD2
//const int ShiftPWM_dataPin=9; //atmega1284p PD1

// The ShiftPWM object is created in the header file, instead of defining it as extern here and creating it in the cpp file.
// If the ShiftPWM object is created in the cpp file, it is separately compiled with the library.
// The compiler cannot treat it as constant and cannot optimize well: it will generate many memory accesses in the interrupt function.

#ifndef _useTimer1 //This is defined in Servo.h
CShiftPWM ShiftPWM(1);
#else
CShiftPWM ShiftPWM(2);  // if timer1 is in use by servo, use timer 2
#endif


//unsigned char precalc[ShiftPWM.m_amountOfBits][ShiftPWM.m_amountOfRegisters*2];
unsigned char precalc[8*20*2];


// The macro below uses 3 instructions per pin to generate the byte to transfer with SPI
// Retreive duty cycle setting from memory (ldd, 2 clockcycles)
// Compare with the counter (cp, 1 clockcycle) --> result is stored in carry
// Use the rotate over carry right to shift the compare result into the byte. (1 clockcycle).
#define add_one_pin_to_byte(sendbyte, counter, ledPtr) \
{ \ 
unsigned char pwmval=*ledPtr; \ 
	asm volatile ("cp %0, %1" : /* No outputs */ : "r" (counter), "r" (pwmval): ); \
	asm volatile ("ror %0" : "+r" (sendbyte) : "r" (sendbyte) : ); 			\
}

static inline void ShiftPWM_handleInterrupt(void){
	sei(); //enable interrupt nesting to prevent disturbing other interrupt functions (servo's for example).

	// Look up which bit of which output register corresponds to the pin.
	// This should be constant, so the compiler can optimize this code away and use sbi and cbi instructions
	// The compiler only knows this if this function is compiled in the same file as the pin setting.
	// That is the reason the full funcion is in the header file, instead of only the prototype.
	// If this function is defined in cpp files of the library, it is compiled seperately from the main file.
	// The compiler does not recognize the pins/ports as constant and sbi and cbi instructions cannot be used.
//	volatile uint8_t * const latchPort = port_to_output_PGM_ct[digital_pin_to_port_PGM_ct[ShiftPWM_latchPin]];
//	const uint8_t latchBit =  digital_pin_to_bit_PGM_ct[ShiftPWM_latchPin];


	// Define a pointer that will be used to access the values for each output. 
	// Let it point one past the last value, because it is decreased before it is used.
	unsigned char * ledPtr=&ShiftPWM.m_PWMValues[ShiftPWM.m_amountOfOutputs];

	// Write shift register latch clock low
//	bitClear(*latchPort, latchBit);
	PORTD &= ~(_BV(PD2));

	unsigned char counter = ShiftPWM.counter;
//	SPDR = 0; // write bogus bit to the SPI, because in the loop there is a receive before send.
	for(unsigned char i =ShiftPWM.m_amountOfRegisters; i>0;--i){  // do a whole shift register at once. This unrolls the loop for extra speed
		unsigned char sendbyte=0;  // no need to initialize, all bits are replaced

		//Fisrt byte sent is for OUT15-OUT8

		add_one_pin_to_byte(sendbyte, counter,  --ledPtr);

		if(ShiftPWM_invertOutputs) {
			sendbyte = ~sendbyte; // Invert the byte if needed.
		}
		UDR0 = sendbyte; // Send the byte to the USART_MSPI
//		UDR0 = 0x00; // Send the byte to the USART_MSPI
//		UDR0 = 0x01; // Send the byte to the USART_MSPI //OUT15
//		UDR0 = 0x02; // Send the byte to the USART_MSPI //OUT14
//		UDR0 = 0x04; // Send the byte to the USART_MSPI //OUT13
//		UDR0 = 0x08; // Send the byte to the USART_MSPI //OUT12
//		UDR0 = 0x10; // Send the byte to the USART_MSPI //OUT11
//		UDR0 = 0x20; // Send the byte to the USART_MSPI //OUT10
//		UDR0 = 0x40; // Send the byte to the USART_MSPI //OUT9
//		UDR0 = 0x80; // Send the byte to the USART_MSPI //OUT8

		//vvvv Wait for TX sent vvvv
////		while (!(UCSR0A & _BV(TXC0)));
//		UCSR0A |= _BV(TXC0);//sbi
		//^^^^ Wait for TX sent ^^^^
		//The TXCn Flag can be used to check that
		//the Transmitter has completed all transfers,
		//and the RXC Flag can be used to check that
		//there are no unread data in the receive buffer.
		//Note that the TXCn Flag must be cleared before each transmission
		//(before UDRn is written) if it is used for this purpose.

		//Calculate "sendbyte" while "UDR0" is sent
		add_one_pin_to_byte(sendbyte, counter, --ledPtr);
		add_one_pin_to_byte(sendbyte, counter, --ledPtr);
		add_one_pin_to_byte(sendbyte, counter, --ledPtr);
		add_one_pin_to_byte(sendbyte, counter, --ledPtr);
		add_one_pin_to_byte(sendbyte, counter, --ledPtr);
		add_one_pin_to_byte(sendbyte, counter, --ledPtr);
		add_one_pin_to_byte(sendbyte, counter, --ledPtr);
		add_one_pin_to_byte(sendbyte, counter, --ledPtr);

		if(ShiftPWM_invertOutputs) {
			sendbyte = ~sendbyte; // Invert the byte if needed.
		}

		UDR0 = sendbyte; // Send the byte to the USART_MSPI
//		UDR0 = 0x00; // Send the byte to the USART_MSPI
//		UDR0 = 0x01; // Send the byte to the USART_MSPI //OUT7
//		UDR0 = 0x02; // Send the byte to the USART_MSPI //OUT6
//		UDR0 = 0x04; // Send the byte to the USART_MSPI //OUT5
//		UDR0 = 0x08; // Send the byte to the USART_MSPI //OUT4
//		UDR0 = 0x10; // Send the byte to the USART_MSPI //OUT3
//		UDR0 = 0x20; // Send the byte to the USART_MSPI //OUT2
//		UDR0 = 0x40; // Send the byte to the USART_MSPI //OUT1
//		UDR0 = 0x80; // Send the byte to the USART_MSPI //OUT0

		//vvvv Wait for TX sent vvvv
		while (!(UCSR0A & _BV(TXC0)));
//		UCSR0A |= _BV(TXC0);//sbi
		//^^^^ Wait for TX sent ^^^^

	}
	// Write shift register latch clock high
//	bitSet(*latchPort, latchBit);
	PORTD |= _BV(PD2);

	if(ShiftPWM.counter<ShiftPWM.m_maxBrightness){
		ShiftPWM.counter++; // Increase the counter
	}
	else{
		ShiftPWM.counter=0; // Reset counter if it maximum brightness has been reached
	}
} 

// See table  11-1 for the interrupt vectors */
#ifndef _useTimer1 
//Install the Interrupt Service Routine (ISR) for Timer1 compare and match A.
ISR(TIMER1_COMPA_vect) {
	ShiftPWM_handleInterrupt();
}
#else
//Install the Interrupt Service Routine (ISR) for Timer2 compare and match A.
ISR(TIMER2_COMPA_vect) {
	ShiftPWM_handleInterrupt();
}
#endif

// #endif for include once.
#endif 

/*
static inline void MIPAM_handleInterrupt(uint8 bit){
	sei(); //enable interrupt nesting to prevent disturbing other interrupt functions (servo's for example).

	// Define a pointer that will be used to access the values for each output.
	// Let it point one past the last value, because it is decreased before it is used.
	unsigned char * ledPtr=&ShiftPWM.m_PWMValues[ShiftPWM.m_amountOfOutputs];
	
	//byte to send
	uint8 byte=ShiftPWM.m_amountOfOutputs;

	// Write shift register latch clock low
//	bitClear(*latchPort, latchBit);
	PORTD &= ~(_BV(PD2));

//	SPDR = 0; // write bogus bit to the SPI, because in the loop there is a receive before send.
	for(unsigned char i =ShiftPWM.m_amountOfRegisters; i>0;--i){  // do a whole shift register at once. This unrolls the loop for extra speed
		//Fisrt byte sent is for OUT15-OUT8
		SPDR = sendbytebit[byte--][bit]; // Send the byte to the USART_MSPI
		while(!(SPSR & (1<<SPIF)))//wait for send
		SPDR = sendbytebit[byte--][bit]; // Send the byte to the USART_MSPI
		while(!(SPSR & (1<<SPIF)))//wait for send

		//vvvv Wait for TX sent vvvv
		while (!(UCSR0A & _BV(TXC0)));
//		UCSR0A |= _BV(TXC0);//sbi
		//^^^^ Wait for TX sent ^^^^

	}
	// Write shift register latch clock high
	PORTD |= _BV(PD2);
}

static inline void calulateAllMIBAMBits(){

	for(uint8 bit=8; bit; bit--){
		// Define a pointer that will be used to access the values for each output.
		// Let it point one past the last value, because it is decreased before it is used.
		unsigned char * ledPtr=&ShiftPWM.m_PWMValues[ShiftPWM.m_amountOfOutputs];

		for(unsigned char i =ShiftPWM.m_amountOfRegisters; i>0;--i){  // do a whole shift register at once. This unrolls the loop for extra speed
			uint8 byte=0;

			//Fisrt byte sent is for OUT15-OUT8
			sendbytebit[byte][bit]=0;//needed because only 1 led is connected to 8-15
			add_one_pin_to_byte(sendbytebit[byte][bit], 0x01<<bit, --ledPtr);
			byte++;
			add_one_pin_to_byte(sendbytebit[byte][bit], 0x01<<bit, --ledPtr);
			add_one_pin_to_byte(sendbytebit[byte][bit], 0x01<<bit, --ledPtr);
			add_one_pin_to_byte(sendbytebit[byte][bit], 0x01<<bit, --ledPtr);
			add_one_pin_to_byte(sendbytebit[byte][bit], 0x01<<bit, --ledPtr);
			add_one_pin_to_byte(sendbytebit[byte][bit], 0x01<<bit, --ledPtr);
			add_one_pin_to_byte(sendbytebit[byte][bit], 0x01<<bit, --ledPtr);
			add_one_pin_to_byte(sendbytebit[byte][bit], 0x01<<bit, --ledPtr);
			add_one_pin_to_byte(sendbytebit[byte][bit], 0x01<<bit, --ledPtr);
			byte++;
		}
	}
//	UPDATE_REQUIRED=true;
}


//ShiftPWM.m_amountOfRegisters;
//ShiftPWM.m_amountOfBits;
//ShiftPWM.m_amountOfLEDsPerRegisters

//60,60,60,59,59,59

uint8 LEDLocation[182];
void setupLEDLocation(void){
	uint8 RegisterNo=ShiftPWM.m_amountOfRegisters;
	uint8 count=0;
	for(uint8 LedNo=ShiftPWM.m_amountOfOutputs; LedNo!=0; LedNo--){
		if(count==(ShiftPWM.m_amountOfLEDsPerRegisters/3)){
			count=0;
			RegisterNo--;
		}
		else
			count++;
		
		LEDLocation[LedNo]=RegisterNo;
		
	}
}

static inline void calulateLEDMIBAMBits(uint8 LED){
	uint8 ShiftRegisterLocation=LEDLocation[LED];

	for(uint8 bit=8; bit; bit--){
		// Define a pointer that will be used to access the values for each output.
		// Let it point one past the last value, because it is decreased before it is used.
		unsigned char * ledPtr=&ShiftPWM.m_PWMValues[ShiftPWM.m_amountOfOutputs];

//		for(unsigned char i =ShiftPWM.m_amountOfRegisters; i>0;--i){  // do a whole shift register at once. This unrolls the loop for extra speed
			uint8 byte=0;

			//Fisrt byte sent is for OUT15-OUT8
			sendbytebit[byte][bit]=0;//needed because only 1 led is connected to 8-15
			add_one_pin_to_byte(sendbytebit[byte][bit], 0x01<<bit, --ledPtr);
			byte++;
			add_one_pin_to_byte(sendbytebit[byte][bit], 0x01<<bit, --ledPtr);
			add_one_pin_to_byte(sendbytebit[byte][bit], 0x01<<bit, --ledPtr);
			add_one_pin_to_byte(sendbytebit[byte][bit], 0x01<<bit, --ledPtr);
			add_one_pin_to_byte(sendbytebit[byte][bit], 0x01<<bit, --ledPtr);
			add_one_pin_to_byte(sendbytebit[byte][bit], 0x01<<bit, --ledPtr);
			add_one_pin_to_byte(sendbytebit[byte][bit], 0x01<<bit, --ledPtr);
			add_one_pin_to_byte(sendbytebit[byte][bit], 0x01<<bit, --ledPtr);
			add_one_pin_to_byte(sendbytebit[byte][bit], 0x01<<bit, --ledPtr);
			byte++;
//		}
	}
//	UPDATE_REQUIRED=true;
}
*/
/*
8 bit red,green,blue
8 bit master brightness
	vary off time
*/

/*
ISR(TIMER0_COMPA_vect) //Compare match interrupt handler
{

  //Update LEDs
  PORTD = bcmBuffer[BCMtracker];

  //Set interrupt for next BCM delay value
  if (BCMtracker == 0) OCR0A = 0x01;
  else OCR0A <<= 1;

  //Increment the BCM tracking index
  BCMtracker ++;
  BCMtracker &= 7; //Flip back to zero when it gets to 8
}
*/
