/*********************************************************************************
 * 
 * DOTKLOK rev. 1.2
 * Andrew O'Malley, 16 March 2011
 * aomalley_AT_gmail_DOT_com
 *
 * AVAILABLE AS A FULL KIT FROM WWW.AOMALLEY.ORG/DOTKLOK
 * Check www.aomalley.org/dotklok for latest code updates
 * 
 * Animated clock with SURE 24x16 display, based on Electric Window 3
 * 
 * Clock animations by Andrew O'Malley    
 *
 * Display interface from demo16x24.c by Bill Westfield ("WestfW")
 * further modified by Andrew O'Malley
 *
 * Uses RTClib from Adafruit.com
 * Uses Button.h from arduino.cc (by Alexander Brevig)
 * Uses Wire.h from arduino.cc (included w/ Arduino IDE)
 *
 * This code comes with no implied warranty
 * Released under Creative Commons Attribution-ShareAlike (CC BY-SA 3.0) license
 * see http://creativecommons.org/licenses/by-sa/3.0/
 *
 *
 *		http://arduino.cc/en/Hacking/Atmega168Hardware
 *		http://arduino.cc/en/Hacking/Atmega168Hardware
 *		http://arduino.cc/en/Hacking/Atmega168Hardware
 *		http://arduino.cc/en/Hacking/Atmega168Hardware
 *
 *
 ********************************************************************************/
/*
digitalWrite(0, LOW)
PORTA=11111111
PORTB=11111110
PORTC=11111111
PORTD=11111111
digitalWrite(1, LOW)
PORTA=11111111
PORTB=11111101
PORTC=11111111
PORTD=11111111
digitalWrite(2, LOW)
PORTA=11111111
PORTB=11111011
PORTC=11111111
PORTD=11111111
digitalWrite(3, LOW)
PORTA=11111111
PORTB=11110111
PORTC=11111111
PORTD=11111111
digitalWrite(4, LOW)
PORTA=11111111
PORTB=11101111
PORTC=11111111
PORTD=11111111
digitalWrite(5, LOW)
PORTA=11111111
PORTB=11011111
PORTC=11111111
PORTD=11111111
digitalWrite(6, LOW)
PORTA=11111111
PORTB=10111111
PORTC=11111111
PORTD=11111111
digitalWrite(7, LOW)
PORTA=11111111
PORTB=01111111
PORTC=11111111
PORTD=11111111
digitalWrite(8, LOW)
PORTA=11111111
PORTB=11111111
PORTC=11111111
PORTD=11111110
digitalWrite(9, LOW)
PORTA=11111111
PORTB=11111111
PORTC=11111111
PORTD=11111101
digitalWrite(10, LOW)
PORTA=11111111
PORTB=11111111
PORTC=11111111
PORTD=11111011
digitalWrite(11, LOW)
PORTA=11111111
PORTB=11111111
PORTC=11111111
PORTD=11110111
digitalWrite(12, LOW)
PORTA=11111111
PORTB=11111111
PORTC=11111111
PORTD=11101111
digitalWrite(13, LOW)
PORTA=11111111
PORTB=11111111
PORTC=11111111
PORTD=11011111
digitalWrite(14, LOW)
PORTA=11111111
PORTB=11111111
PORTC=11111111
PORTD=10111111
digitalWrite(15, LOW)
PORTA=11111111
PORTB=11111111
PORTC=11111111
PORTD=01111111
digitalWrite(16, LOW)
PORTA=11111111
PORTB=11111111
PORTC=11111110
PORTD=11111111
digitalWrite(17, LOW)
PORTA=11111111
PORTB=11111111
PORTC=11111101
PORTD=11111111
digitalWrite(18, LOW)
PORTA=11111111
PORTB=11111111
PORTC=11111011
PORTD=11111111
digitalWrite(19, LOW)
PORTA=11111111
PORTB=11111111
PORTC=11110111
PORTD=11111111
digitalWrite(20, LOW)
PORTA=11111111
PORTB=11111111
PORTC=11101111
PORTD=11111111
digitalWrite(21, LOW)
PORTA=11111111
PORTB=11111111
PORTC=11011111
PORTD=11111111
digitalWrite(22, LOW)
PORTA=11111111
PORTB=11111111
PORTC=10111111
PORTD=11111111
digitalWrite(23, LOW)
PORTA=11111111
PORTB=11111111
PORTC=01111111
PORTD=11111111
digitalWrite(24, LOW)
PORTA=01111111
PORTB=11111111
PORTC=11111111
PORTD=11111111
digitalWrite(25, LOW)
PORTA=10111111
PORTB=11111111
PORTC=11111111
PORTD=11111111
digitalWrite(26, LOW)
PORTA=11011111
PORTB=11111111
PORTC=11111111
PORTD=11111111
digitalWrite(27, LOW)
PORTA=11101111
PORTB=11111111
PORTC=11111111
PORTD=11111111
digitalWrite(28, LOW)
PORTA=11110111
PORTB=11111111
PORTC=11111111
PORTD=11111111
digitalWrite(29, LOW)
PORTA=11111011
PORTB=11111111
PORTC=11111111
PORTD=11111111
digitalWrite(30, LOW)
PORTA=11111101
PORTB=11111111
PORTC=11111111
PORTD=11111111
digitalWrite(31, LOW)
PORTA=11111110
PORTB=11111111
PORTC=11111111
PORTD=11111111

*/

/*** DEFINES AND GLOBAL VARIABLES ***/

#include <WProgram.h>
#include <avr/pgmspace.h>
#include <WConstants.h>
#include "GlobalVars.h"
#include "equinox_rev_1.h"
//#include "time_animations.h"
//#include "ht1632/ht1632.h"
#include "Ansi/Ansiterm.h"
#include "Coms/Coms.h"
#include "Wire/Wire.h"
#include "RTClib/RTClib.h"
#include "Button/Button.h"
//#include "WiShield_user/WiShield.h"
//#include "WiShield_user/udpapp.h"
//#include "WiShield_user/udpapp.c"
#include "Sunrise/Sunrise.h"
#include "SoftwareSerial/SoftwareSerial.h"
#include "ShiftPWM/hsv2rgb.h"
#include "pinout.h"
#include "lpc17xx_ssp.h"


void LED_init(){

	pin_mode(LED_CS_PORT, LED_CS_PIN, OUTPUT);
	digital_write(LED_CS_PORT, LED_CS_PIN, HIGH);

	pin_mode(LED_LE_PORT, LED_LE_PIN, OUTPUT);
	digital_write(LED_LE_PORT, LED_LE_PIN, HIGH);

	pin_mode(WF_HIBERNATE_PORT, WF_HIBERNATE_PIN, INPUT);
	digital_write(WF_HIBERNATE_PORT, WF_HIBERNATE_PIN, HIGH);

	// Initialize SPI pin connect
	PINSEL_CFG_Type PinCfg;
	SSP_CFG_Type SSP_ConfigStruct;
	/* SCK1 */
	PinCfg.Funcnum   = PINSEL_FUNC_2;
	PinCfg.OpenDrain = PINSEL_PINMODE_NORMAL;
	PinCfg.Pinmode   = PINSEL_PINMODE_PULLDOWN;
	PinCfg.Pinnum    = LED_SCK_PIN;
	PinCfg.Portnum   = LED_SCK_PORT;
	PINSEL_ConfigPin(&PinCfg);
	/* MISO1 */
	PinCfg.Pinmode   = PINSEL_PINMODE_PULLUP;
	PinCfg.Pinnum    = LED_MISO_PIN;
	PinCfg.Portnum   = LED_MISO_PORT;
	PINSEL_ConfigPin(&PinCfg);
	/* MOSI1 */
	PinCfg.Pinnum    = LED_MOSI_PIN;
	PinCfg.Portnum   = LED_MOSI_PORT;
	PINSEL_ConfigPin(&PinCfg);

	/* initialize SSP configuration structure */
	SSP_ConfigStruct.CPHA = SSP_CPHA_FIRST;
	SSP_ConfigStruct.CPOL = SSP_CPOL_HI;
	SSP_ConfigStruct.ClockRate = 10000000; /* 10Mhz */
	SSP_ConfigStruct.Databit = SSP_DATABIT_16;
	SSP_ConfigStruct.Mode = SSP_MASTER_MODE;
	SSP_ConfigStruct.FrameFormat = SSP_FRAME_SPI;
	SSP_Init(LPC_SSP0, &SSP_ConfigStruct);

	/* Enable SSP peripheral */
	SSP_Cmd(LPC_SSP0, ENABLE);

}


//Data pin is MOSI (atmega168/328: pin 11. Mega: 51)
//Clock pin is SCK (atmega168/328: pin 13. Mega: 52)
unsigned char maxBrightness = 50;//255;
unsigned char pwmFrequency = 95;//75
int numRegisters = 20; //each TLC8925 has 1 16bit shift registers
const int ShiftPWM_latchPin=10; //atmega1284p PD2
const int ShiftPWM_dataPin=9; //atmega1284p PD1
const int ShiftPWM_clockPin=0; //atmega1284p PB0
const bool ShiftPWM_invertOutputs = 0; // if invertOutputs is 1, outputs will be active low. Usefull for common anode RGB led's.
#include "ShiftPWM/ShiftPWM.h" // include ShiftPWM.h after setting the pins!

#define OFF 0
#define TIME 1
#define RAINBOW 2
unsigned char DISPLAYING = OFF;

#include <stdlib.h>

// global clock variables
extern RTC_DS3231 RTC;
extern DST DST;
extern DateTime time_now;
//extern Ansiterm ANSI;
//extern WiShield WiFi;
Sunrise mySun(52,-0.9,0);//Northampton, England, Europe - Latitude/Longitude and Timezone

//Alarms format {y, m, dow, d, hh, mm, ss} (0xff all)/(dow 0b10000000-mon)
//unsigned char alarms[2][7] = { { 0xff, 0xff, 0b1111101, 0xff, 6, 0} };

boolean dailyCalculationsDone=false;
uint8_t lastSecond=false;
uint8_t TEST[5];

// Button mapping
Button b1 = Button(6,PULLUP);
Button b2 = Button(5,PULLUP);
Button b3 = Button(8,PULLUP);
Button b4 = Button(7,PULLUP);
Button b5 = Button(4,PULLUP);
Button buttons[] = { b1, b2, b3, b4, b5 };

unsigned long int b1_time;
unsigned long int b2_time;
unsigned long int b3_time;
unsigned long int b4_time;

extern int animation;
extern int ani_max;

//extern void udpapp_init(void);
//extern void send_request(void);

#define rxPin 0
#define txPin 1 //19

// set up a new serial port
//SoftwareSerial mySerial =  SoftwareSerial(rxPin, txPin);

/*** UTILITY FUNCTIONS ***/

// debugging tool only:
// used to estimate available mem
// obtainted from arduino.cc forum
// not sure how accurate this is
int availableMemory() {
  int size = 8192;
  byte *buf;
  while ((buf = (byte *) malloc(--size)) == NULL);
  free(buf);
  return size;
}

int setupPAMTimer(){
	  //Setup Timer
	  cli(); //disable all interrupts
	  TCCR0A |= (1<<WGM01); //Use CTC mode
	  TCCR0B |= (1<<CS02); //Start timer with 256 prescaler
	  TIMSK0 |= (1<<OCIE0A); //Enable the compare A interrupt
	  OCR0A = 0x01; //Set to compare on first timer tick
	  sei(); //enable all interrupts
}

void rgbLedRainbow(int numRGBLeds, int delayVal, int numCycles, int maxBrightnessRainbow, int rainbowWidth, bool fade){
  // Displays a rainbow spread over all LED's, which shifts in hue.
  int hue, sat, val;
  unsigned char red, green, blue, temp_maxBrightness=maxBrightness, fadeInDone=0;

//  if(fade)
//	ShiftPWM.FadeOut();

  ShiftPWM.SetAll(0);

  for(int cycle=0;cycle<numCycles;cycle++){ // shift the raibom numCycles times
    for(int colorshift=0;colorshift<360;colorshift++){ // Shift over full color range (like the hue slider in photoshop)
      for(int led=0;led<numRGBLeds;led++){ // loop over all LED's
        hue = ((led)*360/(rainbowWidth-1)+colorshift)%360; // Set hue from 0 to 360 from first to last led and shift the hue
        sat = 255;
        val = 255;
        hsv2rgb(hue, sat, val, &red, &green, &blue, maxBrightnessRainbow); // convert hsv to rgb values
        ShiftPWM.SetGroupOf3(led, red, green, blue); // write rgb values

      }
//      if((!fadeInDone)&&(fade)) {
//      	//ShiftPWM.FadeIn();
//      }
      delay(delayVal);
    }
  }
//  if(fade)
//	  ShiftPWM.FadeOut();
}

void dailyCheck(void) {
	//calculate sunrise/sunset at
	mySun.Rise(time_now.month(),time_now.day());
	mySun.Set(time_now.month(),time_now.day());
}

void hourlyCheck(void) {
	//Checks and adjusts time for DST
	adjustForDST(time_now.unixtime(),true);
}

void minutelyCheck(void) {
	//update brightness
	DateTime time = DateTime(time_now.unixtime()-DST.adjDST()*3600);
////	pinMode(8, OUTPUT);
////	digitalWrite(8, !mySun.isSunRisen(time.hour(),time.minute()));
//	mySun.isSunRisen(hour,time_now.minute());
	//Send temperature
//	TEST[0]='H';
}

char timeUpdate(void) {
	//update current time
	time_now = RTC.now(DST.adjDST());


#ifdef WIFI_ENABLED
//	Serial.println("b4 wifi run");
	  WiFi.run();
//	Serial.println("after wifi run");
#endif

	//If second not incremented return
	if(time_now.second()==lastSecond)
		return 0;
	lastSecond=time_now.second();

	//Check for serial input
//	checkSerial();

	//run daily checks at 00:00:00
	if(!time_now.hour()&&!time_now.minute()&&!time_now.second())
		dailyCheck();

	//run  checks at xx:00:00
	if(!time_now.minute()&&!time_now.second())
		hourlyCheck();

	//run  checks at xx:xx:00
	if(!time_now.second())
		minutelyCheck();

	//custom alarms
//	for(uint8_t alarmNo=0; alarms[alarmNo]; alarmNo++) {
//
//	}
	return 1;
}

/***********************************************************************
 * traditional Arduino sketch functions: setup and loop.
 ***********************************************************************/


void setup() {
//*
//	cli();
	// init the button inputs
//	pinMode(4, INPUT);
//	pinMode(5, INPUT);
//	pinMode(6, INPUT);
//	pinMode(7, INPUT);
//	pinMode(8, INPUT);
//	digitalWrite(4, HIGH);
//	digitalWrite(5, HIGH);
//	digitalWrite(6, HIGH);
//	digitalWrite(7, HIGH);
//	digitalWrite(8, HIGH);

	// define pin modes for tx, rx, led pins:
	pinMode(rxPin, INPUT);
	pinMode(txPin, OUTPUT);

//	mySerial.begin(4800); //Only 4800 works to PC
//	mySerial.print("Built:");
//	mySerial.print(__DATE__);
//	mySerial.print(".");
//	mySerial.println(__TIME__);
//	delay(500);
//	mySerial.println("Start S uart");

	//setup shiftpwm pins
	pinMode(ShiftPWM_dataPin, OUTPUT);
	digitalWrite(ShiftPWM_dataPin, HIGH);
	pinMode(ShiftPWM_clockPin, OUTPUT);
	digitalWrite(ShiftPWM_clockPin, HIGH);
	pinMode(ShiftPWM_latchPin, OUTPUT);
	digitalWrite(ShiftPWM_latchPin, HIGH);

	//Pin test
	while(0){
		digitalWrite(txPin, HIGH);
		digitalWrite(txPin, LOW);
	}
	//Pin test
	while(0){
		char a=20;
//		for(char a=0; a<31; a++){
//			Serial.println(a, DEC);
			pinMode(a, OUTPUT);
//			for(char b=0; a<255; a++){
				digitalWrite(a, HIGH);
				digitalWrite(a, LOW);
//				delay(500);
//			}
//		}
	}
	//Pin test
	while(0){
		digitalWrite(ShiftPWM_dataPin, HIGH);
		digitalWrite(ShiftPWM_dataPin, LOW);
	}

/*
	Serial.begin(4800);
	Serial.println("Start H uart");

	Serial.println();
	Serial.println();
	Serial.println();
	Serial.println();
	for(unsigned char a=0,b; a<25; a++){
		DDRA=0xff;
		DDRB=0xff;
		DDRC=0xff;
		DDRD=0xff;
		PORTA=0xff;
		PORTB=0xff;
		PORTC=0xff;
		PORTD=0xff;
		digitalWrite(a, LOW);
		Serial.print("digitalWrite(");Serial.print(a,DEC);Serial.println(", LOW)");
		Serial.print("PORTA=");Serial.println(PORTA,BIN);
		Serial.print("PORTB=");Serial.println(PORTB,BIN);
		Serial.print("PORTC=");Serial.println(PORTC,BIN);
		Serial.print("PORTD=");Serial.println(PORTD,BIN);
		digitalWrite(a, HIGH);
//		Serial.print("digitalWrite(");Serial.print(a,DEC);Serial.println(", HIGH)");
//		Serial.print("PORTA=");Serial.println(PORTA,BIN);
//		Serial.print("PORTB=");Serial.println(PORTB,BIN);
//		Serial.print("PORTC=");Serial.println(PORTC,BIN);
//		Serial.print("PORTD=");Serial.println(PORTD,BIN);
	}
*/
	//PWM Pin test
	while(0){
		for(unsigned char a=0,b; a<3; a++){
			b=(1<<a);
			unsigned char b=0xAA;
			//Send the LED output to the shift register
//			mySerial.println(b,BIN);
			digitalWrite(ShiftPWM_latchPin,LOW);
			shiftOut(ShiftPWM_dataPin,ShiftPWM_clockPin,LSBFIRST,b);    //High byte first
			shiftOut(ShiftPWM_dataPin,ShiftPWM_clockPin,LSBFIRST,b);           //Low byte second
			digitalWrite(ShiftPWM_latchPin,HIGH);
			delay(200);
		}
	}

	//Setup USART for MSPIM
	UBRR0L = 0; //Reset for MSPI to work
	UBRR0H = 0; //Reset for MSPI to work
	//Do not modify for fastest MSPIM

	//Disable UART receiver with no interrupts
	UCSR0B = ((0<<TXEN0)|(0<<RXEN0)|(0<<UDRIE0)|(0<<TXCIE0)|(0<<RXCIE0));
//	mySerial.print("Off UCSR0B=");
//	mySerial.println(UCSR0B,BIN);
	//Set Master SPI Mode
	UCSR0C = (
			(0<<UCPOL0)|	//Bit 0 - UCPOLn: Clock polarity
			(0<<UCPHA0)|	//Bit 1 - UCPHAn: Clock phase
			(1<<UDORD0)|	//Bit 2 - UDORDn: Data order (LSB First=1)
							//Bit 5:3 - Reserved bits in MSPI mode3
			(1<<UMSEL00)|	//Bit 7:6 - UMSELn1:0: USART mode select (MSPI=11)
			(1<<UMSEL01));
//	mySerial.print("SPI UCSR0C=");
//	mySerial.println(UCSR0C,BIN);

//	mySerial.println(UBRR0H,BIN);
//	mySerial.println(UBRR0L,BIN);

	//Enable UART receiver with no interrupts
	UCSR0B = ((1<<TXEN0)	//Bit 3 – TXENn: Transmitter Enable n
			|(0<<RXEN0)		//Bit 4 – RXENn: Receiver Enable n
			|(0<<UDRIE0)	//Bit 5 – UDRIEn: USART Data Register Empty Interrupt Enable n
			|(0<<TXCIE0)	//Bit 6 – TXCIEn: TX Complete Interrupt Enable n
			|(0<<RXCIE0));	//Bit 7 – RXCIEn: RX Complete Interrupt Enable n
//	mySerial.print("On UCSR0B=");
//	mySerial.println(UCSR0B,BIN);

//#define MSPIBAUD 8000000
//#define MSPIBAUD 230400
//#define MSPIBAUD 115200
//#define MSPIBAUD 9600
//#define MSPIUBRR F_CPU/16/MSPIBAUD-1
//	UBRR0H = (unsigned char)(MSPIUBRR>>8);
//	UBRR0L = (unsigned char)MSPIUBRR;
//	UBRR0L = 0x00;
//	UBRR0L = 0x00;

	ShiftPWM.SetAmountOfRegisters(numRegisters);
	ShiftPWM.Start(pwmFrequency,maxBrightness);
//	ShiftPWM.PrintInterruptLoad();
//	while(0);//Lock up
//	sei();


	while(0){
//		mySerial.println("255");
//		delay(2500);
//		ShiftPWM.SetAll(50);
//		mySerial.println("0");
//		delay(500);
//		mySerial.println("PWM");
		ShiftPWM.OneByOneSlow();
//		ShiftPWM.SetOne(8,255);delay(1000);ShiftPWM.SetAll(0);
//		ShiftPWM.SetOne(1,255);delay(200);ShiftPWM.SetAll(0);
//		ShiftPWM.SetAll(255);
	}
	while(0)
		rgbLedRainbow(ShiftPWM.m_amountOfOutputs, 5, 3, maxBrightness, ShiftPWM.m_amountOfOutputs,true);

	while(0) {
		for(char a=0; a<9; a++){
//			mySerial.print(a,DEC);
			ShiftPWM.SetAll(0);
			ShiftPWM.SetOne(a,255);
			for(char a=0; a<9; a++) {
//				mySerial.print("	");
//				mySerial.print(ShiftPWM.m_PWMValues[a],HEX);
			}
//			mySerial.println();
			delay(1000);
		}
	}

//	mySerial.println("PWMUSART");
	while(0){
		for(char a=9,b; a>0; a--){
			b=(1<<a);
//			mySerial.println("UDR0");
//			mySerial.println(b,BIN);
			for(int a=0; a<0xfff; a++){
				digitalWrite(ShiftPWM_latchPin,LOW);
				for(int a=0; a<2; a++){
	//				UDR0 = 0x01; // Send the byte to the USART_MSPI
					UDR0 = b; // Send the byte to the USART_MSPI
					//vvvv Wait for TX sent vvvv
					while (!(UCSR0A & _BV(TXC0)));    // wait for last send to finish and retreive answer. Retreive must be done, otherwise the USART_MSPI will not work.
					UCSR0A |= _BV(TXC0);//sbi
					//^^^^ Wait for TX sent ^^^^
				}
				digitalWrite(ShiftPWM_latchPin,HIGH);
			}
			delay(500);
		}
	}
//	cli(); //Disable interrupts


//	Serial.begin(4800);
//	Serial.println("Start H uart");
//	delay(1000);


//	PROGMEMprint(stringDOTKLOK);
//	Serial.println(REV);
//	delay(1000);
#ifdef USE_ANSI
	ANSI.eraseScreen();
	ANSI.home();
#endif

//	if(DEBUG){
//		Serial.println("DEBUG true, serial port open at 57600");
//		Serial.print("Avail mem = ");
//		Serial.println(availableMemory());
//	}
//	else{
//		Serial.print("DK "); Serial.println(REV);
//		Serial.println("DEBUG false, serial port closed");
//		Serial.end();
//	}

  // general set up
  pinMode(13, OUTPUT);

  // RTC setup
  Wire.begin();
  RTC.begin();
  DST.begin();
//	DateTime set = DateTime(__DATE__, __TIME__);
//	DateTime set = DateTime(2011,10,30,00,59,48);
//	set.toString();
//	RTC.adjust(DateTime(__DATE__, __TIME__));
//  RTC.adjust(DateTime(2011,10,30,00,59,48));
//  delay(1000);
  if (!RTC.isrunning()) {
//    if(DEBUG) PROGMEMprint(stringRTC_NOT_running);
    // following line sets the RTC to the date & time this sketch was compiled
    RTC.adjust(DateTime(__DATE__, __TIME__));
  }
///  else {
//    if(DEBUG) PROGMEMprint(stringRTC_running);
///  }
  	//Update time variables

//	cli();mySerial.println("u");sei();
	if(1){
		timeUpdate();
		timeUpdate();
		//Sets sunrise/sunset time for dimming
		dailyCheck();
		dailyCheck();
		//Sets screen brightness
		minutelyCheck();
		minutelyCheck();
		//Checks and adjusts time for DST
		hourlyCheck();
		hourlyCheck();
	}
//////////////goodgood	rgbLedRainbow(ShiftPWM.m_amountOfOutputs, 1, 1, maxBrightness, ShiftPWM.m_amountOfOutputs,true);
//cli();mySerial.println("loop begin");//sei();

  // test buttons
/* TODO
  while( b5.isPressed() ){
    for(int i=0; i<4; i++){
      if( buttons[i].isPressed() )  plot(i,0,1);
      else                          plot(i,0,0);
    }
  }
*/



#ifdef WIFI_ENABLED
	WiFi.init();
//	udpapp_init();
//	Serial.println("after wifi init");
#endif

} // end setup()

//
#define RED1 255
#define RED2 0
#define RED3 0

#define GREEN1 0
#define GREEN2 255
#define GREEN3 0

#define BLUE1 0
#define BLUE2 0
#define BLUE3 255

void colourToRGBled(signed int rgbNo, char r, char g, char b, bool add, char fade) {
	char fadeNo = 1;
	do {
		if((rgbNo-fade)<0) {
			rgbNo=ShiftPWM.m_amountOfOutputs-fadeNo;
			r=r/2;
			g=g/2;
			b=r/2;
		}
		if(add)
			ShiftPWM.SetGroupOf3Add(rgbNo,r,g,b);
		else
			ShiftPWM.SetGroupOf3(rgbNo,r,g,b);
		fade--;
		fadeNo++;
	} while (fade>0);
}

void loop() {
	bool inverted = false;
	char prevSecond;
//	timeUpdate();//Updates time_now and checks alarms

	if(timeUpdate()) {
		ShiftPWM.SetAll(0);
//	colourToRGBled(time_now.hour12()*5,RED1,RED2,RED3,false,0);
//	colourToRGBled(time_now.minute(),GREEN1,GREEN2,GREEN3,false,0);
//	colourToRGBled(time_now.second(),BLUE1,BLUE2,BLUE3,false,0);

/* good
		ShiftPWM.SetGroupOf3(time_now.second()-4,0,0,1);
		ShiftPWM.SetGroupOf3(time_now.second()-3,0,0,2);
		ShiftPWM.SetGroupOf3(time_now.second()-2,0,0,4);
		ShiftPWM.SetGroupOf3(time_now.second()-1,0,0,100);
		ShiftPWM.SetGroupOf3(time_now.second()-0,0,0,255);

		ShiftPWM.SetGroupOf3(time_now.minute()-2,3,0,5);
		ShiftPWM.SetGroupOf3(time_now.minute()-1,10,0,25);
		ShiftPWM.SetGroupOf3(time_now.minute()-0,20,0,180);

		ShiftPWM.SetGroupOf3(time_now.hour12()*5+(time_now.minute()/12),85,0,130);
*/

		ShiftPWM.SetGroupOf3(time_now.second()-0,0,0,1);
//		ShiftPWM.SetGroupOf3(time_now.second()-3,0,0,2);
//		ShiftPWM.SetGroupOf3(time_now.second()-2,0,0,4);
//		ShiftPWM.SetGroupOf3(time_now.second()-1,0,0,100);
//		ShiftPWM.SetGroupOf3(time_now.second()-0,0,0,255);

		ShiftPWM.SetGroupOf3(time_now.minute()-1,1,0,1);
		ShiftPWM.SetGroupOf3(time_now.minute()-0,1,0,1);
//		ShiftPWM.SetGroupOf3(time_now.minute()-0,20,0,180);

		ShiftPWM.SetGroupOf3(time_now.hour12()*5+(time_now.minute()/12),1,1,1);


//		delay(100);
	}
//	if(!time_now.minute()&&!time_now.second())
//		rgbLedRainbow(ShiftPWM.m_amountOfOutputs, 2, 1, maxBrightness/6, ShiftPWM.m_amountOfOutputs,true);

//	ShiftPWM.OneByOneFast();
//	if(inverted)
//		ShiftPWM.InvertAll();

} // end main loop

int main(void) {
	init();
	setup();
	for (;;)
		loop();
	return 0;
}

extern "C" void __cxa_pure_virtual()
{
  cli();
  for (;;);
}

__extension__ typedef int __guard __attribute__((mode (__DI__)));

void * operator new(size_t size)
{
  return malloc(size);
}

void operator delete(void * ptr)
{
  free(ptr);
}

void * operator new[](size_t size)
{
    return malloc(size);
}

void operator delete[](void * ptr)
{
	if (ptr)
		free(ptr);
}

int __cxa_guard_acquire(__guard *g) {return !*(char *)(g);}
void __cxa_guard_release (__guard *g) {*(char *)g = 1;}
void __cxa_guard_abort (__guard *) {}

