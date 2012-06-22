/*
ShiftPWM.h - Library for Arduino to PWM many outputs using shift registers - Version 1
Copyright (c) 2011 Elco Jacobs, Technical University of Eindhoven, department of 
Industrial Design, Electronics Atelier.
All right reserved.

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


---> See ShiftPWM.h for more info
*/

#include "CShiftPWM.h"

extern const bool ShiftPWM_invertOutputs;

//CShiftPWM::CShiftPWM() : m_timer(int timerInUse=1){ //Timer is set in initializer list, because it is const
CShiftPWM::CShiftPWM(int timerInUse) : m_timer(timerInUse){ //Timer is set in initializer list, because it is const
	m_ledFrequency = 0;
	m_maxBrightness = 0;
	m_amountOfRegisters = 0;
	m_amountOfBits = 8;
	m_amountOfLEDsPerRegisters = 9;
	m_amountOfOutputs=0;
	counter =0;
	p_counter=20;
	m_maxGlobalPower=10;
	unsigned char * m_PWMValues=0;
}

CShiftPWM::~CShiftPWM() {
	if(m_PWMValues>0){
		free( m_PWMValues );
	}
}

bool CShiftPWM::IsValidPin(int pin){
	if(pin<m_amountOfOutputs){
		return 1;
	}
	else{
//jc		mySerial.print("Error: Trying to write duty cycle of pin ");
//jc		mySerial.print(pin);
//jc		mySerial.print(" , while number of outputs is ");
//jc		mySerial.print(m_amountOfOutputs);
//jc		mySerial.print(" , numbered 0-");
//jc		mySerial.println(m_amountOfOutputs-1);
//jc		delay(1000);
		return 0;
	}
}


void CShiftPWM::SetOne(int pin, unsigned char value){
	if(IsValidPin(pin) ){
		m_PWMValues[pin]=value;
	}
}

void CShiftPWM::SetAll(unsigned char value){
	for(int k=0 ; k<(m_amountOfOutputs);k++){
		m_PWMValues[k]=value;
	}   
}

void CShiftPWM::InvertAll(){
	for(int k=0 ; k<(m_amountOfOutputs);k++){
		m_PWMValues[k]=~m_PWMValues[k];
	}
}

void CShiftPWM::SetGroupOf2(int group, unsigned char v0,unsigned char v1){
	if(IsValidPin(group*2+1) ){
		m_PWMValues[group*2]=v0;
		m_PWMValues[group*2+1]=v1;
	}
}

void CShiftPWM::SetGroupOf3(signed int group, unsigned char v0,unsigned char v1,unsigned char v2){
//	if(group<0)
//		group = m_amountOfOutputs + group;
	if(group==-1)
		group = 59;
	if(group==-2)
		group = 58;
	if(group==-3)
		group = 57;
	if(group==-4)
		group = 56;
	if(group==-5)
		group = 55;
//	if(IsValidPin(group*3+2) ){
		m_PWMValues[group*3]=v0;
		m_PWMValues[group*3+1]=v1;
		m_PWMValues[group*3+2]=v2;
//	}
}

void CShiftPWM::SetGroupOf3Add(signed int group, unsigned char v0,unsigned char v1,unsigned char v2){
	if(group<0)
		group = m_amountOfOutputs + group;
//	if(IsValidPin(group*3+2) ){

		//Red		
		if((m_PWMValues[group*3]+v0)>255)
			m_PWMValues[group*3]=255;
		else
			m_PWMValues[group*3]+=v0;
		
		//Green
		if((m_PWMValues[group*3+1]+v1)>255)
			m_PWMValues[group*3+1]=255;
		else
			m_PWMValues[group*3+1]+=v1;

		//Blue
		if((m_PWMValues[group*3+2]+v2)>255)
			m_PWMValues[group*3+2]=255;
		else
			m_PWMValues[group*3+2]+=v2;

//	}
}

void CShiftPWM::SetGroupOf4(int group, unsigned char v0,unsigned char v1,unsigned char v2,unsigned char v3){
	if(IsValidPin(group*4+3) ){
		m_PWMValues[group*4]=v0;
		m_PWMValues[group*4+1]=v1;
		m_PWMValues[group*4+2]=v2;
		m_PWMValues[group*4+3]=v3;
	}
}

void CShiftPWM::SetGroupOf5(int group, unsigned char v0,unsigned char v1,unsigned char v2,unsigned char v3,unsigned char v4){
	if(IsValidPin(group*5+4) ){
		m_PWMValues[group*5]=v0;
		m_PWMValues[group*5+1]=v1;
		m_PWMValues[group*5+2]=v2;
		m_PWMValues[group*5+3]=v3;
		m_PWMValues[group*5+4]=v4;
	}
}

// OneByOne functions are usefull for testing all your outputs
void CShiftPWM::OneByOneSlow(void){
	OneByOne_core(1024/m_maxBrightness);
}

void CShiftPWM::OneByOneFast(void){
	OneByOne_core(1);
}

void CShiftPWM::OneByOne_core(int delaytime){
	int pin,brightness;
	SetAll(0);
	for(int pin=0;pin<m_amountOfOutputs;pin++){
		for(brightness=0;brightness<m_maxBrightness;brightness++){
			m_PWMValues[pin]=brightness;
			delay(delaytime);
		}
		for(brightness=m_maxBrightness;brightness>=0;brightness--){
			m_PWMValues[pin]=brightness;
			delay(delaytime);
		}
	}
}

void CShiftPWM::SetAmountOfRegisters(unsigned char newAmount){
	cli(); // Disable interrupt
	unsigned char oldAmount = m_amountOfRegisters;
	m_amountOfRegisters = newAmount;
	m_amountOfOutputs=m_amountOfRegisters*m_amountOfLEDsPerRegisters; //jc changed 8 to m_amountOfLEDsPerRegisters

	if(LoadNotTooHigh() ){ //Check if new amount will not result in deadlock
		m_PWMValues = (unsigned char *) realloc(m_PWMValues, m_amountOfOutputs); //jc changed 8 to m_amountOfLEDsPerRegisters //resize array for PWMValues

		for(int k=oldAmount; k<(m_amountOfOutputs);k++){ //jc changed 8 to m_amountOfLEDsPerRegisters
			m_PWMValues[k]=0; //set new values to zero
		}
		sei(); //Re-enable interrupt
	}
	else{
		// New value would result in deadlock, keep old values and print an error message
		m_amountOfRegisters = oldAmount;
		m_amountOfOutputs=m_amountOfRegisters*m_amountOfLEDsPerRegisters; //jc changed 8 to m_amountOfLEDsPerRegisters
//jc		mySerial.println("Amount of registers is not increased, because load would become too high");
		sei();
	}
}

void CShiftPWM::Start(int ledFrequency, unsigned char maxBrightness){
	// Configure and enable timer1 or timer 2 for a compare and match A interrupt.    

	m_ledFrequency = ledFrequency;
	m_maxBrightness = maxBrightness;

	if(LoadNotTooHigh() ){
		if(m_timer==1){ 
			InitTimer1();
		}
		else if(m_timer==2){
			InitTimer2();
		}
	}
	else{
//jc		mySerial.println("Interrupts are disabled because load is too high.");
		cli(); //Disable interrupts
	}
}
