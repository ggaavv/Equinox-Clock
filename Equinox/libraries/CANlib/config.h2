/*************************************************************************
**  Sony Unilink Interface (Configuration File)
**  by Michael Wolf
**
**  Released under GNU GENERAL PUBLIC LICENSE
**	See LICENSE.TXT for details
**
**  Revision History
**
**  when         what  who	why
**
**  2003-08-05   2.01  MIC  complete code revision
**  2004-08-22   2.04  MIC  + added option for new op-amp hardware
**
**************************************************************************/
#define CANUSB_COMPATIBLE
//#define DEBUG
//#define DEBUGX
//#define UPDATE_JUMPER
#ifndef UPDATE_JUMPER
//#define CD
#define MD
//#define REMOTE
#endif

//#ifdef DEBUG
//usart_putc('');

//#if defined(__AVR_ATmega162__)
//#define MCU_XTAL     8000000		// MCU crystal speed in Hz (max 16MHz!)
//#else
#define MCU_XTAL     7372800		// MCU crystal speed in Hz (max 16MHz!)
//#endif
#define BAUD_RATE    115200			// UART communication speed (max 115200!)
//#define BAUD_RATE    9600			// UART communication speed (max 115200!)

/*************************************************************************/
/* Enable this option to use this firmware with new op-amp driven hardware */
#define OPAMP_HW

/************************************************************************/
//#define WATCHDOG

/* enable the watchdog.
*************************************************************************/
#define BUS_LOGGING

/* enable packet logging via UART but:
   If bus logging is enabled, key/event output is disabled!
   Note: Baud rates lower than 38400 will not work with
         BUS_LOGGING enabled!
*************************************************************************/
//#define RAW_COMMAND

/* enable raw command mode
   send raw commands in the following way:
   *xxxxxxxxxxxxx
   ||||||       |
   |||||up to 9 data bytes, no checksum, no end byte
   ||||CMD2
   |||CMD1
   ||TAD (transmitter adress in hex)
   |RAD (receiver adress in hex)
   start tag

   All command bytes are hexadecimal!
*************************************************************************/

/************************************************************************
 The following features are especially for use with a Yampp3 MP3 player
 or other MP3 player control.
 Do not enable "YAMPP3" if no Yampp is connected!
*/

//#define LED_OUT

/* enable LED output on ATmega8
LED1 = interface ID status
LED2 = interface selected as source
LED3 = Unilink Rx/Tx traffic */

//#define RELAIS_OUT

/* enable Power relais output on ATmega8
Relais will turned on when the interface is selected
as source by head unit */

//#define YAMPP3

/* enable special "Play" sequence for Yampp support
works in the following way:
- interface receives "Play" from head unit
- interface turns on power relais to power-up the Yampp and waits for RS232 signal
- when Yampp is running he sends a "!" to signal "I'm ready"
- now the interface sends the "Play" command to Yampp to start playing
************************************************************************/
//jc/*
// Define the UART output for each key/event here
#define Play            'P'         // output on play command
#define Stop            'S'         // output on stop command
#define Track_up        'T'         // output on next track command
#define Track_down      't'         // output on previous track command
#define Disc_up         'D'         // output on next disc command
#define Disc_down       'd'         // output on previous disc command
#define FFWD            'F'         // output on fast forward command
#define FRVS            'R'         // output on fast reverse command
#define Direct_Disc     'K'         // output on direct disc key, followed by a number 0-9
#define Intro           'i'         // output on intro on/off
#define Repeat          'r'         // output on repeat mode change, followed by a mode number
#define Shuffle         's'         // output on shuffle mode change, followed by a number
#define DScan           'm'         // output on d.scan mode change
#define Bank            'b'         // output on bank mode change, followed by a number
#define Power_Off       'O'         // output on power off command
#define Start_Tag       'c'
#define Screen_Update   'x'
#define CR              0x0d        //Cariage return
//jc*/
/*
// My output definition to control the Yampp3/USB MP3 player
#define Play            'g'         // play
#define Stop            'G'         // stop
#define Track_up        'n'         // next track
#define Track_down      'p'         // previous track
#define Disc_up         's'         // next playlist
#define Disc_down       'S'         // previous playlist
#define FFWD            ' '         // fast forward command
#define FRVS            ' '         // fast reverse command
#define Direct_Disc     ' '         // direct disc key, followed by a number 0-9
#define Intro           'i'         // intro mode change
#define Repeat          'e'         // repeat mode change
#define Shuffle         'r'         // shuffle mode change
#define DScan           ' '         // d.scan mode change
#define Bank            ' '         // bank mode change, followed by a number
#define Power_Off       ' '         // Power off
*/
