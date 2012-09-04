/*************************************************************************
**  Renault Stereo Interface (Header File)
**  by Jamie Clarke
**
**  Released under GNU GENERAL PUBLIC LICENSE
**	See LICENSE.TXT for details
**
**  Revision History
**
**  when         what  who	why
**
**  2009-08-23   1.00  JC  Initial code revision
**  2009-09-05   2.00  JCC  + slightly modified for CAN
**
**************************************************************************/

#ifndef RENAULT_STEREO_H_
#define RENAULT_STEREO_H_

//##define IPOD 1
//#define PC 2
//#define KEYBOARD 3
#define number_of_sources 3

#define SendScreenTxtStart 1
#define SendDebugHex 1
#define SendScreenAsciiData 1
#define SendScreenTempData 1

#define REMOTE_RADIO 0xA2
#define REMOTE_AUX 0xA3
#define REMOTE_TRAFFIC 0xA4
#define REMOTE_CD 0xA5
#define REMOTE_PAUSE 0xA6
#define IPOD_STATUS_PLAY 0xA7
#define IPOD_STATUS_STOP 0xA8
#define IPOD_STATUS_OFF 0xA9
#define IPOD_STATUS_ON 0xAA
//#define IPOD_STATUS_PAUSE 0xA9

#define BUTTON_PLAY_ONLY 0x61
#define BUTTON_STOP_ONLY 0x62
#define BUTTON_PLAY_PAUSE 0x63
#define BUTTON_STOP_PAUSE 0x64
#define BUTTON_OFF 0x65
#define BUTTON_ON 0x66
#define BUTTON_PREV 0x67
#define BUTTON_NEXT 0x68
#define BUTTON_PLAYPAUSE_TOGGLE 0x69

#define StartOfMessage 0
#define AsciiLength 22
#define Ascii3IconTextStart 5
#define Ascii3IconTextFinish 13 // Finish

#define NEW_STRING 0x52
#define TEMP_STRING 0x54
#define RETURN_STRING 0x58

#define BUTTON_TIMEOUT 500

//                                            | 0x10                                           | 0x21                                      | 0x22                                      | 0x23                                        | 0x24                                          |
//unsigned char DefaultScreenString[4][8] =  { {0x10, 0x19, 0x76, 0x60, 0x01,  ' ',  'I',  'P'}, {0x21, 'O', 'D', ' ', ' ', ' ', 0x10, ' '}, {0x22, ' ',  ' ', 'I', 'P', 'O', 'D', ' '}, {0x23, ' ', ' ', ' ', ' ', 0x00, 0x81, 0x81 }};
//unsigned char DefaultScreenString2[5][8] = { {0x10, 0x1C, 0x7F, 0x55, 0x55, 0xFF, 0x60, 0x01}, {0x21, ' ', 'I', 'P', 'O', 'D', ' ' , ' '}, {0x22, ' ', 0x10, ' ', ' ', ' ', 'I', 'P'}, {0x23, 'O', 'D', ' ', ' ',  ' ',  ' ',  ' '}, {0x24, 0x00 ,0x81 ,0x81 ,0x81 ,0x81 ,0x81 ,0x81 }};
char Spaces[] = {"        "};
unsigned char ScreenText[] =                  { 0x10, 0x19, 0x76, 0x60, 0x01,  'I',  'P',  'O' , 0x21, 'D', ' ', ' ', ' ', ' ', 0x10, ' '  , 0x22, ' ',  ' ', ' ', ' ', ' ', ' ', ' '  , 0x23, ' ', ' ', ' ', ' ', 0x00, 0x81, 0x81  , 0xFF, 0xFF, 0x00};

//unsigned char ScreenText2[] =                  { 0x10, 0x19, 0x76, 0x60, 0x01,  'I',  'P',  'O' , 0x21, 'D', ' ', ' ', ' ', ' ', 0x10, ' '  , 0x22, ' ',  ' ', ' ', ' ', ' ', ' ', ' '  , 0x23, ' ', ' ', ' ', ' ', 0x00, 0x81, 0x81  , 0xFF, 0xFF};

/*
// Icons
// Byte  0         1         2        3
// Bit  0x01    V0x2    v  0x4   v   0x8  v
//       00000001 00000010  00000100  00001000
//etc etc
#define PresetIconByte
#define PresetIconBit
#define TrafficIconByte 0x2
#define TrafficIconBit 0x4
#define TrafficIconByte
#define TrafficIconBit

*/
struct {
    uint8_t NoToSend;		// Number of packet to send
    uint8_t NoSent;		// Number of packet sent
    volatile uint8_t NoAck;		// Number of acknoledged packets
    uint8_t ScreenSendingBusy;		// 
    uint32_t ID;		    // Frame ID
} Screen_tx;			      //


//unsigned char ScreenLitData[12];
//unsigned char ScreenAsciiData[34];
//unsigned char ScreenSendPackets[5][8];
unsigned char PrevScreenText[10];
unsigned char Source;
volatile unsigned int ScreenTimeoutCounter;
volatile unsigned int OffTimeout;
volatile unsigned char ScreenTempData[36];
volatile unsigned char RecieveComplete;
volatile unsigned char RX_In_progress;
volatile unsigned char ButtomPressed;
volatile unsigned char source_change;
volatile unsigned char ButtonTempData[2];

unsigned char source_selected;
#define CD 1
#define IPOD 2
#define PC 3
#define KEYBOARD 4
#define number_of_sources 3 // number of sources + 1

// Functions prototypes
void ScreenTimeout_init ( void);
void ipod_button(char *key);
void ipod_control(unsigned char key);
void Screen_init ( void );
unsigned char SendToScreen ( unsigned int SendId,
			char * String,
			unsigned char ScreenIcons,
			unsigned char Command);
unsigned char SendScreen ( void );
uint8_t send_source_change(char dir);
void Screen_loop( void );
void Screen_Interrupt( void );
void renault_debug_print( void );



#endif




/*

*Key*
00A2A2*AUX*

*Key*
020001*AUX*
s+
*Key*
020002*AUX*
s-
*Key*
020001*AUX*
s+
*Key*
020001*AUX*
s+
*Key*
020002*AUX*
s-
ScreenAsciiData - FM      RADIO FM    
ScreenTempData - 1C7F5555FF6001464D20202020202010524144494F20464D20202020
R fd
ScreenAsciiData - HEART    HEART    P6
ScreenTempData - 1C774555FD7601484541525420202010204845415254202020205036
R fd
*Key*
010001*AUX*
s+
ScreenAsciiData - CD      CD          
ScreenTempData - 1C7F5555FF6001434420202020202010434420202020202020202020
R fd
*Key*
010001*AUX*
s+
ScreenAsciiData - AUX     AUX         
ScreenTempData - 197E6001415558202020202010415558202020202020202020
AUX fdNS
*Key*
020001*AUX*
s+
ScreenAsciiData - MW      RADIO MW    
ScreenTempData - 197E60014D5720202020202010524144494F204D5720202020
R fd
*Key*
010002*AUX*
s-
ScreenAsciiData - AUX     AUX         
ScreenTempData - 197E6001415558202020202010415558202020202020202020
AUX fdNS
*Key*
020002*AUX*
s-
ScreenAsciiData - CD      CD          
ScreenTempData - 197E6001434420202020202010434420202020202020202020
R fd
*Key*
010001*AUX*
s+
ScreenAsciiData - AUX     AUX         
ScreenTempData - 197E6001415558202020202010415558202020202020202020
AUX fdNS
Q
Uart init.
CAN init.
I

*Key*
00A2A2*AUX*

ScreenAsciiData - AUX     AUX         
ScreenTempData - 1C775555FF6001415558202020202010415558202020202020202020
AUX fdNS
*Key*
020005*AUX*
p
ScreenAsciiData -  PAUSE     PAUSE    
ScreenTempData - 19766001205041555345202010202020504155534520202020
R fd
*Key*
010005*AUX*
p
ScreenAsciiData - AUX     AUX         
ScreenTempData - 19766001415558202020202010415558202020202020202020
AUX fdNS
*Key*
020001*AUX*
s+
ScreenAsciiData - MW      RADIO MW    
ScreenTempData - 197E60014D5720202020202010524144494F204D5720202020
R fd
ScreenAsciiData - MW  909 MW  909   P4
ScreenTempData - 1C775555FD74014D57202039303920104D5720203930392020205034
R fd
*Key*
010001*AUX*
s+
ScreenAsciiData - FM      RADIO FM    
ScreenTempData - 1C7F5555FF6001464D20202020202010524144494F20464D20202020
R fd
ScreenAsciiData - HEART    HEART    P6
ScreenTempData - 1C774555FD7601484541525420202010204845415254202020205036
R fd
*Key*
010002*AUX*
s-
ScreenAsciiData - MW      RADIO MW    
ScreenTempData - 1C7F5555FF60014D5720202020202010524144494F204D5720202020
R fd
ScreenAsciiData - MW  909 MW  909   P4
ScreenTempData - 1C775555FD74014D57202039303920104D5720203930392020205034
R fd
*Key*
010001*AUX*
s+
ScreenAsciiData - FM      RADIO FM    
ScreenTempData - 1C7F5555FF6001464D20202020202010524144494F20464D20202020
R fd
ScreenAsciiData - HEART    HEART    P6
ScreenTempData - 1C774555FD7601484541525420202010204845415254202020205036
R fd
*Key*
010001*AUX*
s+
ScreenAsciiData - CD      CD          
ScreenTempData - 1C7F5555FF6001434420202020202010434420202020202020202020
R fd
*Key*
010002*AUX*
s-
ScreenAsciiData - FM      RADIO FM    
ScreenTempData - 197E6001464D20202020202010524144494F20464D20202020
R fd
ScreenAsciiData - HEART    HEART    P6
ScreenTempData - 1C774555FD7601484541525420202010204845415254202020205036
R fd
*Key*
01A2A2*AUX*

ScreenAsciiData - FM      RADIO FM    
ScreenTempData - 1C775555FF6001464D20202020202010524144494F20464D20202020
R fd
ScreenAsciiData - NORTHNTS NORTHNTS P6
ScreenTempData - 1C776555FD76014E4F5254484E545310204E4F5254484E5453205036
R fd
ScreenAsciiData - UUUUUUUUUUUUUUUUUUUUU
ScreenTempData - 55555555555555555555555555555555555555555555555555555555
R fd
ScreenAsciiData - HEART    HEART    P6
ScreenTempData - 19767601484541525420202010204845415254202020205036
R fd
Uart init.
CAN init.
*Key*
000001*AUX*
s+
ScreenAsciiData - CD      CD          
ScreenTempData - 1C7F5555FF6001434420202020202010434420202020202020202020
R fd
*Key*
010002*AUX*
s-
ScreenAsciiData - FM      RADIO FM    
ScreenTempData - 197E6001464D20202020202010524144494F20464D20202020
R fd
ScreenAsciiData - HEART    HEART    P6
ScreenTempData - 1C774555FD7601484541525420202010204845415254202020205036
R fd
*Key*
010001*AUX*
s+
ScreenAsciiData - CD      CD          
ScreenTempData - 1C7F5555FF6001434420202020202010434420202020202020202020
R fd
*Key*
010001*AUX*
s+
ScreenAsciiData - AUX     AUX         
ScreenTempData - 197E6001415558202020202010415558202020202020202020
AUX fdNS
*Key*
020002*AUX*
s-
ScreenAsciiData - CD      CD          
ScreenTempData - 197E6001434420202020202010434420202020202020202020
R fd
*Key*
010001*AUX*
s+
ScreenAsciiData - AUX     AUX         
ScreenTempData - 197E6001415558202020202010415558202020202020202020
AUX fdNS
*Key*
020003*AUX*
v+
ScreenAsciiData -  VOL 12 VOLUME 12   
ScreenTempData - 1976600120564F4C2031322010564F4C554D45203132202020
R fd
*Key*
010004*AUX*
v-
ScreenAsciiData -  VOL 11 VOLUME 11   
ScreenTempData - 1976600120564F4C2031312010564F4C554D45203131202020
R fd
ScreenAsciiData - AUX     AUX         
ScreenTempData - 19766001415558202020202010415558202020202020202020
AUX fdNS
ScreenAsciiData - AUX     AUX         
ScreenTempData - 1C7F5555FF6001415558202020202010415558202020202020202020
AUX fdNS
*Key*
02A2A2*AUX*

ScreenAsciiData - AUX     AUX         
ScreenTempData - 1C775555FF6001415558202020202010415558202020202020202020
AUX fdNS
*Key*
020101*AUX*

*Key*
020101*AUX*

*Key*
020101*AUX*

*Key*
020101*AUX*

*Key*
020101*AUX*

*Key*
020101*AUX*

*Key*
020101*AUX*

*Key*
020141*AUX*

*Key*
020141*AUX*

*Key*
020141*AUX*

*Key*
020141*AUX*

*Key*
020101*AUX*

*Key*
020141*AUX*

*Key*
020141*AUX*

*Key*
020141*AUX*

*Key*
020141*AUX*

*Key*
020141*AUX*

*Key*
020000*AUX*

*Key*
020000*AUX*

*Key*
020000*AUX*

*Key*
020000*AUX*

*Key*
020000*AUX*

*Key*
020000*AUX*

*Key*
020000*AUX*

*Key*
020000*AUX*

*Key*
020000*AUX*

*Key*
020000*AUX*

*Key*
020000*AUX*

*Key*
020000*AUX*

*Key*
020000*AUX*

*Key*
020000*AUX*

*Key*
020000*AUX*

*Key*
020002*AUX*
s-
ScreenAsciiData - CD      CD          
ScreenTempData - 197E6001434420202020202010434420202020202020202020
R fd
*Key*
010001*AUX*
s+
ScreenAsciiData - AUX     AUX         
ScreenTempData - 197E6001415558202020202010415558202020202020202020
AUX fdNS
*Key*
020101*AUX*

*Key*
020101*AUX*

*Key*
020101*AUX*

*Key*
020141*AUX*

*Key*
020101*AUX*

*Key*
020101*AUX*

*Key*
020101*AUX*

*Key*
020101*AUX*

*Key*
020101*AUX*

*Key*
020101*AUX*

*Key*
020141*AUX*

*Key*
020141*AUX*

*Key*
020141*AUX*

*Key*
020141*AUX*

*Key*
020101*AUX*

*Key*
020101*AUX*

*Key*
020000*AUX*

*Key*
020000*AUX*

*Key*
020002*AUX*
s-
ScreenAsciiData - CD      CD          
ScreenTempData - 197E6001434420202020202010434420202020202020202020
R fd
*Key*
010101*AUX*

*Key*
020101*AUX*

ScreenAsciiData - TR 09 CDCD   TR 09  
ScreenTempData - 19766001545220303920434410434420202054522030392020
R fd
*Key*
010001*AUX*
s+
ScreenAsciiData - AUX     AUX         
ScreenTempData - 197E6001415558202020202010415558202020202020202020
AUX fdNS
Uart init.
CAN init.
*Key*
00A2A2*AUX*

ScreenAsciiData - AUX     AUX         
ScreenTempData - 1C775555FF6001415558202020202010415558202020202020202020
AUX fdNS
*Key*
020001*AUX*
s+
ScreenAsciiData - MW      RADIO MW    
ScreenTempData - 197E60014D5720202020202010524144494F204D5720202020
R fd
*Key*
010002*AUX*
s-
ScreenAsciiData - AUX     AUX         
ScreenTempData - 197E6001415558202020202010415558202020202020202020
AUX fdNS
*Key*
020002*AUX*
s-
ScreenAsciiData - CD      CD          
ScreenTempData - 197E6001434420202020202010434420202020202020202020
R fd
*Key*
010001*AUX*
s+
ScreenAsciiData - AUX     AUX         
ScreenTempData - 197E6001415558202020202010415558202020202020202020
AUX fdNS
*Key*
020101*AUX*

*Key*
020101*AUX*

*Key*
020101*AUX*

*Key*
020101*AUX*

*Key*
020141*AUX*

*Key*
020141*AUX*

*Key*
020000*AUX*

*Key*
020000*AUX*

*Key*
020000*AUX*

*Key*
020101*AUX*

*Key*
020101*AUX*

*Key*
020141*AUX*

*Key*
020000*AUX*

*Key*
020000*AUX*

*Key*
020001*AUX*
s+
ScreenAsciiData - MW      RADIO MW    
ScreenTempData - 197E60014D5720202020202010524144494F204D5720202020
R fd
ScreenAsciiData - MW  909 MW  909   P4
ScreenTempData - 1C775555FD74014D57202039303920104D5720203930392020205034
R fd
*Key*
010000*AUX*

ScreenAsciiData - AUTO    AUTO    MODE
ScreenTempData - 1C7F5555DF60014155544F20202020104155544F202020204D4F4445
R fd
*Key*
010000*AUX*

ScreenAsciiData - MANUAL  MANUAL  MODE
ScreenTempData - 197E60014D414E55414C2020104D414E55414C20204D4F4445
R fd
*Key*
010001*AUX*
s+
ScreenAsciiData - FM      RADIO FM    
ScreenTempData - 1C7F5555FF6001464D20202020202010524144494F20464D20202020
R fd
*Key*
010002*AUX*
s-
ScreenAsciiData - MW      RADIO MW    
ScreenTempData - 197E60014D5720202020202010524144494F204D5720202020
R fd
*Key*
010000*AUX*

ScreenAsciiData - MW 1215 MW 1215   P1
ScreenTempData - 1C775555DF71014D57203132313520104D5720313231352020205031
R fd
*Key*
010000*AUX*

ScreenAsciiData - MANUAL  MANUAL  MODE
ScreenTempData - 197E60014D414E55414C2020104D414E55414C20204D4F4445
R fd
*Key*
010000*AUX*

ScreenAsciiData - LIST    LIST    MODE
ScreenTempData - 1C7F1555FF60014C49535420202020104C495354202020204D4F4445
R fd
*Key*
010000*AUX*

ScreenAsciiData - PRESET  PRESET  MODE
ScreenTempData - 1C7F5555FD600150524553455420201050524553455420204D4F4445
R fd
ScreenAsciiData - MW  909 MW  909   P4
ScreenTempData - 197674014D57202039303920104D5720203930392020205034
R fd
*Key*
010001*AUX*
s+
ScreenAsciiData - FM      RADIO FM    
ScreenTempData - 1C7F5555FF6001464D20202020202010524144494F20464D20202020
R fd
ScreenAsciiData - HEART    HEART    P6
ScreenTempData - 1C774555FD7601484541525420202010204845415254202020205036
R fd
*Key*
010000*AUX*

ScreenAsciiData - AUTO    AUTO    MODE
ScreenTempData - 1C7F4555DF60014155544F20202020104155544F202020204D4F4445
R fd
ScreenAsciiData - UUUUUUUUUUUUUUUUUUUUU
ScreenTempData - 55555555555555555555555555555555555555555555555555555555
R fd
ScreenAsciiData - FM  983 FM  98.3    
ScreenTempData - 19762001464D20203938332010464D202039382E3320202020
R fd
*Key*
010000*AUX*

ScreenAsciiData - MANUAL  MANUAL  MODE
ScreenTempData - 197E60014D414E55414C2020104D414E55414C20204D4F4445
R fd
*Key*
010000*AUX*

ScreenAsciiData - LIST    LIST    MODE
ScreenTempData - 1C7F2555FF60014C49535420202020104C495354202020204D4F4445
R fd
*Key*
010000*AUX*

ScreenAsciiData - PRESET  PRESET  MODE
ScreenTempData - 1C7F6555FD600150524553455420201050524553455420204D4F4445
R fd
ScreenAsciiData - UUUUUUUUUUUUUUUUUUUUU
ScreenTempData - 55555555555555555555555555555555555555555555555555555555
R fd
ScreenAsciiData - HEART    HEART    P6
ScreenTempData - 19767601484541525420202010204845415254202020205036
R fd
*Key*
010101*AUX*

ScreenAsciiData - Radio 1  Radio 1  P1
ScreenTempData - 197E71A1526164696F2031201020526164696F203120205031
R fd
ScreenAsciiData - UUUUUUUUUUUUUUUUU  P2
ScreenTempData - 197673A8204242432052332010202042424320523320205033
R fd
ScreenAsciiData - UUUUUUUUUU BBC R4  P4
ScreenTempData - 197675B0424243204E68746E1020424243204E68746E205035
R fd
ScreenAsciiData - UUUUUUUUUUNORTHNTS P6
ScreenTempData - 55555555555555555555555555554E4F5254484E5453205036
R fd
ScreenAsciiData - UUUUUUUUUUUUUUUUUUUUU
ScreenTempData - 55555555555555555555555555555555555555555555555555
R fd
ScreenAsciiData - UUUUUUUUUUUUUUUUUUUUU
ScreenTempData - 55555555555555555555555555555555555555555555555555
R fd
*Key*
010141*AUX*

ScreenAsciiData - NORTHNTS NORTHNTS P6
ScreenTempData - 197E76B54E4F5254484E545310204E4F5254484E5453205036
R fd
ScreenAsciiData - UUUUUUUUUUUUUUUUUUUUU
ScreenTempData - 55555555555555555555555555555555555555555555555555
R fd
ScreenAsciiData - UUUUUUUUUUUUUUUUUUUUU
ScreenTempData - 55555555555555555555555555555555555555555555555555
R fd
ScreenAsciiData - HEART    HEART    P6
ScreenTempData - 19767601484541525420202010204845415254202020205036
R fd
*Key*
010001*AUX*
s+
ScreenAsciiData - CD      CD          
ScreenTempData - 1C7F5555FF6001434420202020202010434420202020202020202020
R fd
ScreenAsciiData - TR 09 CDCD   TR 09  
ScreenTempData - 19766001545220303920434410434420202054522030392020
R fd
*Key*
010002*AUX*
s-
ScreenAsciiData - FM      RADIO FM    
ScreenTempData - 197E6001464D20202020202010524144494F20464D20202020
R fd
ScreenAsciiData - HEART    HEART    P6
ScreenTempData - 1C774555FD7601484541525420202010204845415254202020205036
R fd
*Key*
010002*AUX*
s-
ScreenAsciiData - MW      RADIO MW    
ScreenTempData - 1C7F5555FF60014D5720202020202010524144494F204D5720202020
R fd
ScreenAsciiData - MW  909 MW  909   P4
ScreenTempData - 1C775555FD74014D57202039303920104D5720203930392020205034
R fd
*Key*
010001*AUX*
s+
ScreenAsciiData - FM      RADIO FM    
ScreenTempData - 1C7F5555FF6001464D20202020202010524144494F20464D20202020
R fd
ScreenAsciiData - HEART    HEART    P6
ScreenTempData - 1C774555FD7601484541525420202010204845415254202020205036
R fd
*Key*
010001*AUX*
s+
ScreenAsciiData - CD      CD          
ScreenTempData - 1C7F5555FF6001434420202020202010434420202020202020202020
R fd
ScreenAsciiData - TR 09 CDCD   TR 09  
ScreenTempData - 19766001545220303920434410434420202054522030392020
R fd
*Key*
010001*AUX*
s+
ScreenAsciiData - AUX     AUX         
ScreenTempData - 197E6001415558202020202010415558202020202020202020
AUX fdNS
*Key*
020101*AUX*

*Key*
020101*AUX*

*Key*
020101*AUX*

*Key*
020141*AUX*

*Key*
020141*AUX*

*Key*
020101*AUX*

*Key*
020101*AUX*

*Key*
020141*AUX*

*Key*
020101*AUX*

*Key*
020141*AUX*

*Key*
020141*AUX*

*Key*
020101*AUX*

*Key*
020101*AUX*

*Key*
020000*AUX*

*Key*
020000*AUX*

*Key*
020000*AUX*

*Key*
020101*AUX*

*Key*
020101*AUX*

*Key*
020141*AUX*

*Key*
020141*AUX*

*Key*
020101*AUX*

*Key*
020101*AUX*

*Key*
020141*AUX*

*Key*
020001*AUX*
s+
ScreenAsciiData - MW      RADIO MW    
ScreenTempData - 197E60014D5720202020202010524144494F204D5720202020
R fd
*Key*
010002*AUX*
s-
ScreenAsciiData - AUX     AUX         
ScreenTempData - 197E6001415558202020202010415558202020202020202020
AUX fdNS
*Key*
020101*AUX*

*Key*
020141*AUX*

dowm*Key*
020101*AUX*

up*Key*
020141*AUX*

pause*Key*
020000*AUX*



*******************************
*******************************
*******************************
*******************************
*******************************
t1218 | 10/21/22/23/24 | 3 or 6 *Ascii | 0x01 | 8*Ascii charachters | 0x10 | 12*Ascii charachters | 0x00 | 0x81's to end
t1218 | 1              | 3 or 6        | 1    | 8                   | 1    | 12                   | 1


t1218 | 10/21/22/23/24 | 6 *Ascii | 0x01 | 8*Ascii charachters | 0x10 | 12*Ascii charachters | 0x00 | 0x81 *2

t1218 | 10/21/22/23/24 | 3 *Ascii | 0x01 | 8*Ascii charachters | 0x10 | 12*Ascii charachters | 0x00 | 0x81 *6


 19 76 60           ~01    P  A  U  S  E        10          P  A  U  S  E              00

 1C 7F 55 55 FF 60  ~01 T  R  A  F     O  F  F  10 T  R  A  F  F  I  C     O  F  F     00

 19 76 60           ~01    P  A  U  S  E        10          P  A  U  S  E              00

 1C 7F 51 55 FF 60  ~01 T  R  A  F     O  N     10 T  R  A  F  F  I  C     O  N        00

 19 76 60           ~01    P  A  U  S  E        10          P  A  U  S  E              00

 19 76 60           ~01 A  U  X                 10 A  U  X                             00


*******************************
*******************************
*******************************
*******************************

allways starta 01 befor text
allways ends 00818181818181etc

         0x0A9 = t0A9803890002A2A2A2A2 - 0x0A9_ascii = ........
              0x4A9 = NULL - 0x4A9_ascii = NULL
              0x121_(0-5)
              0 - 1C 7F 51 55 FF 60 01 41 55 58 20 20 20 20 20 10 41 55 58 20 20 20 20 20 20 20 20 20 71 51 55 FF 81 81 81 - ascii = .QU.`.AUX     .AUX         qQU....
     bin = 00000001 00010011 00000111 00010110 00000101 00000001 00000101 00000101 00010110 00010110 00000110 00000000 00000000 00000001
              1 - 19766001205041555345202010202020504155534520202020008181 - ascii = .v`. PAUSE  .   PAUSE    .
     bin = 00000001 00001001 00000111 00000110 00000110 00000000 00000000 00000001
              2 - 19766001415558202020202010415558202020202020202020008181 - ascii = .v`.AUX     .AUX         ...
     bin = 00000001 00001001 00000111 00000110 00000110 00000000 00000000 00000001
              3 - 1C7F5555FF60014D5720202020202010524144494F204D572020202000818181818181 - ascii = .UU.`.MW      .RADIO MW    .
     bin = 00000001 00010011 00000111 00010110 00000101 00000101 00000101 00000101 00010110 00010110 00000110 00000000 00000000 00000001
              4 - 1C7F5555FF600141555820202020201041555820202020202020202000818181818181 - ascii = .UU.`.AUX     .AUX         .......
     bin = 00000001 00010011 00000111 00010110 00000101 00000101 00000101 00000101 00010110 00010110 00000110 00000000 00000000 00000001
              5 - NULL - ascii = NULL - No. 0.
     bin = NULL


*******************************
*******************************
*******************************
*******************************

With nothing happening roughly every second:
t3DF 8 79 00 81 81 81 81 81 81
t3CF 8 69 00 A2 A2 A2 A2 A2 A2
odd:

t3CF86900A2A2A2A2A2A2

t3CF8611100A2A2A2A2A2


t3DF87A01818181818181

t3DF8701A110000000001

t3DF87900818181818181

turn on only
t1B187081818181818181

t1B18045202FFFF818181

t1C1870A2A2A2A2A2A2A2

t1C1802640FA2A2A2A2A2

521 fm stereo has 5 not 4 with aux

allways this order
t5C187481818181818181

t5B1874A2A2A2A2A2A2A2

t5B1874A2A2A2A2A2A2A2

t5C187481818181818181

from stereo/to display
3CF ping
121 screen data (reply t521 8 30 01 00 A2 A2 A2 A2 A2 else
A2

M04203020
mB10F8E0F

Volume up pressed:
t0A9 8 03 89 00 04 A2 A2 A2 A2
reply:
t4A9 8 74 81 81 81 81 81 81 81


RADIO 1 (1)
Power on:
t1B1 8 04 52 02 FF FF 81 81 81
t5B1 8 74 A2 A2 A2 A2 A2 A2 A2

t1C1 8 02 64 0F A2 A2 A2 A2 A2
t5C1 8 74 81 81 81 81 81 81 81

t121 8 10 1C 7F 55 55 FF 60 01
t521 8 30 01 00 A2 A2 A2 A2 A2

t121 8 21 46 4D 20 20 20 20 20
t521 8 30 01 00 A2 A2 A2 A2 A2

t121 8 22 20 10 52 41 44 49 4F
t521 8 30 01 00 A2 A2 A2 A2 A2

t121 8 23 20 46 4D 20 20 20 20
t521 8 30 01 00 A2 A2 A2 A2 A2

t121 8 24 00 81 81 81 81 81 81
t521 8 74 A2 A2 A2 A2 A2 A2 A2<<different end maybe


Working with 8 IDs :
        0A9  000010101001 to display - key pressed
        4A9  010010101001 from display t4A98 74 81818181818181"
        121  000100100001 to display
        521  010100100001 from display "t5218 30 01 00 A2A2A2A2A2" else "t5218 74 A2A2A2A2A2A2A2" for end
        1B1  000110110001 to display
        5B1  010110110001 from display
        1C1  000111000001 to display
        5C1  010111000001 from display
Number of possible combinations = 254
Best result = 32  Worst result = 256
  32 results : 	ACR = 04203020    AMR = B10F8E0F
	 :: 021  029  0A1  0A9  121  129  181  191  1A1  1A9  1B1  1C1  1D1  1E1  1F1  421  429  4A1  4A9  521  529  581  591  5A1  5A9  5B1  5C1  5D1  5E1  5F1
  32 results : 	ACR = 14202020    AMR = A30F9C0F
	 :: 0A1  0A9  0B1  0B9  101  121  141  161  181  1A1  1A9  1B1  1B9  1C1  1E1  4A1  4A9  4B1  4B9  501  521  541  561  581  5A1  5A9  5B1  5B9  5C1  5E1
  32 results : 	ACR = 20201420    AMR = 9C0FA30F
	 :: 0A1  0A9  0B1  0B9  101  121  141  161  181  1A1  1A9  1B1  1B9  1C1  1E1  4A1  4A9  4B1  4B9  501  521  541  561  581  5A1  5A9  5B1  5B9  5C1  5E1
  32 results : 	ACR = 30200420    AMR = 8E0FB10F
	 :: 021  029  0A1  0A9  121  129  181  191  1A1  1A9  1B1  1C1  1D1  1E1  1F1  421  429  4A1  4A9  521  529  581  591  5A1  5A9  5B1  5C1  5D1  5E1  5F1

0x121_1 = t12181019767601486561 - 0x121_1_ascii = ..vv.Hea - No. 283.
0x121_2 = t12182172743936361020 - 0x121_2_ascii = !rt966.  - No. 284.
0x121_3 = t12182248656172743936 - 0x121_3_ascii = "Heart96 - No. 285.
0x121_4 = t12182336205036008181 - 0x121_4_ascii = #6 P6... - No. 286.

0x121_5 = t12181019766001205041 - 0x121_5_ascii = ..v`. PA - No. 287.
0x121_0 = t12182155534520201020 - 0x121_0_ascii = !USE  .  - No. 288.
0x121_1 = t12182220205041555345 - 0x121_1_ascii = "  PAUSE - No. 289.
0x121_2 = t12182320202020008181 - 0x121_2_ascii = #    ... - No. 290.

0x121_3 = t12181019767601486561 - 0x121_3_ascii = ..vv.Hea - No. 291.
0x121_4 = t12182172743936361020 - 0x121_4_ascii = !rt966.  - No. 292.
0x121_5 = t12182248656172743936 - 0x121_5_ascii = "Heart96 - No. 293.
0x121_0 = t12182336205036008181 - 0x121_0_ascii = #6 P6... - No. 294.


0x121_3 = t12181019766001205041 - 0x121_3_ascii = ..v`. PA - No. 303.
0x121_4 = t12182155534520201020 - 0x121_4_ascii = !USE  .  - No. 304.
0x121_5 = t12182220205041555345 - 0x121_5_ascii = "  PAUSE - No. 305.
0x121_0 = t12182320202020008181 - 0x121_0_ascii = #    ... - No. 306.

0x121_5 = t1218101C7F5155FF6001 - 0x121_5_ascii = ..QU.`. - No. 383.
0x121_0 = t12182141555820202020 - 0x121_0_ascii = !AUX     - No. 384.
0x121_1 = t12182220104155582020 - 0x121_1_ascii = " .AUX   - No. 385.
0x121_2 = t12182320202020202020 - 0x121_2_ascii = #        - No. 386.
0x121_3 = t12182400818181818181 - 0x121_3_ascii = $....... - No. 387.

all with traffic

0x121_4 = t12181019766001415558 - 0x121_4_ascii = ..v`.AUX - No. 424.
0x121_5 = t12182120202020201041 - 0x121_5_ascii = !     .A - No. 425.
0x121_0 = t12182255582020202020 - 0x121_0_ascii = "UX      - No. 426.
0x121_1 = t12182320202020008181 - 0x121_1_ascii = #    ... - No. 427.

        0x0A9 = NULL - 0x0A9_ascii = NULL
                             0x4A9 = NULL - 0x4A9_ascii = NULL

                             0x121_0 = t1218101C7F5555FF6001 - 0x121_0_ascii = ..UU.`. - No. 10.
                             0x121_1 = t121821464D2020202020 - 0x121_1_ascii = !FM      - No. 11.
                             0x121_2 = t1218222010524144494F - 0x121_2_ascii = " .RADIO - No. 12.
                             0x121_3 = t12182320464D20202020 - 0x121_3_ascii = # FM     - No. 13.
                             0x121_4 = t12182400818181818181 - 0x121_4_ascii = $....... - No. 14.
                             0x121_5 = NULL - 0x121_5_ascii = NULL - No. 0.

                             0x521_0 = NULL - 0x521_0_ascii = NULL - No. 0.
                             0x521_1 = NULL - 0x521_1_ascii = NULL - No. 0.
                             0x521_2 = NULL - 0x521_2_ascii = NULL - No. 0.
                             0x521_3 = NULL - 0x521_3_ascii = NULL - No. 0.
                             0x521_4 = NULL - 0x521_4_ascii = NULL - No. 0.
                             0x521_5 = NULL - 0x521_5_ascii = NULL - No. 0.

                             0x3CF = t3CF86900A2A2A2A2A2A2 - 0x3CF_ascii = i.......
                             0x3DF = t3DF87900818181818181 - 0x3DF_ascii = y.......
                             0x1C1 = t1C1802640FA2A2A2A2A2 - 0x1C1_ascii = .d......
                             0x5C1 = NULL - 0x5C1_ascii = NULL
                             0x5B1 = NULL - 0x5B1_ascii = NULL
                             0x1B1 = t1B18045200FFFF818181 - 0x1B1_ascii = .R......
                     Done 772 Screen updates.

                             0x0A9 = NULL - 0x0A9_ascii = NULL
                             0x4A9 = NULL - 0x4A9_ascii = NULL

                             0x121_0 = t1218101C7F5555FF6001 - 0x121_0_ascii = ..UU.`. - No. 0.
                             0x121_1 = t12182141555820202020 - 0x121_1_ascii = !AUX     - No. 1.
                             0x121_2 = t12182220104155582020 - 0x121_2_ascii = " .AUX   - No. 2.
                             0x121_3 = t12182320202020202020 - 0x121_3_ascii = #        - No. 3.
                             0x121_4 = t12182400818181818181 - 0x121_4_ascii = $....... - No. 4.
                             0x121_5 = NULL - 0x121_5_ascii = NULL - No. 0.

                             0x521_0 = NULL - 0x521_0_ascii = NULL - No. 0.
                             0x521_1 = NULL - 0x521_1_ascii = NULL - No. 0.
                             0x521_2 = NULL - 0x521_2_ascii = NULL - No. 0.
                             0x521_3 = NULL - 0x521_3_ascii = NULL - No. 0.
                             0x521_4 = NULL - 0x521_4_ascii = NULL - No. 0.
                             0x521_5 = NULL - 0x521_5_ascii = NULL - No. 0.

                             0x3CF = t3CF86900A2A2A2A2A2A2 - 0x3CF_ascii = i.......
                             0x3DF = t3DF87900818181818181 - 0x3DF_ascii = y.......
                             0x1C1 = t1C1802640FA2A2A2A2A2 - 0x1C1_ascii = .d......
                             0x5C1 = NULL - 0x5C1_ascii = NULL
                             0x5B1 = NULL - 0x5B1_ascii = NULL
                             0x1B1 = t1B18045202FFFF818181 - 0x1B1_ascii = .R......
                     Done 261 Screen updates.




t1218101C7F5555FF6001
t121821464D2020202020
t1218222010524144494F
t12182320464D20202020
t12182400818181818181



Radio1
t3CF 8 611100A2A2A2A2A2
t3CF 8 6900A2A2A2A2A2A2
t3CF 8 611100A2A2A2A2A2
t3CF 8 611100A2A2A2A2A2
t3CF 8 611100A2A2A2A2A2
t3CF 8 611100A2A2A2A2A2
t3CF 8 611100A2A2A2A2A2
t3CF 8 6900A2A2A2A2A2A2
t3DF 8 7A01818181818181
t3DF 8 7900818181818181
t3CF 8 611100A2A2A2A2A2
t3DF 8 7A01818181818181
t3DF 8 701A110000000001
t1C1 8 70A2A2A2A2A2A2A2
t5C1 8 7481818181818181
t0A9 8 70A2A2A2A2A2A2A2
t4A9 8 7481818181818181
t3DF 8 701A110000000001
t3DF 8 701A110000000001
t121 8 7081818181818181
t1B187081818181818181
t521874A2A2A2A2A2A2A2
t5B1874A2A2A2A2A2A2A2
t1B18045200FFFF818181
t5B1874A2A2A2A2A2A2A2
t1C1802640FA2A2A2A2A2
t5C187481818181818181
t3CF86900A2A2A2A2A2A2
t3DF87900818181818181
t3CF86900A2A2A2A2A2A2
t3DF87900818181818181
t3CF86900A2A2A2A2A2A2
t3DF87900818181818181
t3CF86900A2A2A2A2A2A2
t3DF87900818181818181
t3CF86900A2A2A2A2A2A2
t3DF87900818181818181
t3CF86900A2A2A2A2A2A2
t3DF87900818181818181
t3CF86900A2A2A2A2A2A2
t3DF87900818181818181
t3CF86900A2A2A2A2A2A2
t3DF87900818181818181
t3CF86900A2A2A2A2A2A2
t3DF87900818181818181
t3CF86900A2A2A2A2A2A2
t3DF87900818181818181
t3CF86900A2A2A2A2A2A2
t3DF87900818181818181
t3CF86900A2A2A2A2A2A2
t3DF87900818181818181 -----------------------------------------------
Date: 28/01/2009 - 18:52:14
End log file

t3CF8611100A2A2A2A2A2
t3CF86900A2A2A2A2A2A2
t3CF8611100A2A2A2A2A2
t3CF8611100A2A2A2A2A2
t3CF8611100A2A2A2A2A2
t3CF8611100A2A2A2A2A2
t3DF87A01818181818181
t3DF87900818181818181
t3CF8611100A2A2A2A2A2
t3DF87A01818181818181
t3DF8701A110000000001
t1C1870A2A2A2A2A2A2A2
t5C187481818181818181
t0A9870A2A2A2A2A2A2A2
t4A987481818181818181
t3CF86900A2A2A2A2A2A2
t3DF8701A110000000001
t3DF8701A110000000001
t12187081818181818181
t521874A2A2A2A2A2A2A2
t1B187081818181818181
t5B1874A2A2A2A2A2A2A2
t1B18045202FFFF818181
t5B1874A2A2A2A2A2A2A2
t1C1802640FA2A2A2A2A2
t5C187481818181818181
t1218101C7F5555FF6001
t5218300100A2A2A2A2A2
t121821464D2020202020
t5218300100A2A2A2A2A2
t1218222010524144494F
t5218300100A2A2A2A2A2
t12182320464D20202020
t5218300100A2A2A2A2A2
t12182400818181818181
t521874A2A2A2A2A2A2A2
t3DF87900818181818181
t3CF86900A2A2A2A2A2A2
t1218101C776555FD7101
t5218300100A2A2A2A2A2
t3DF87900818181818181
t121821526164696F2031
t5218300100A2A2A2A2A2
t12182220102052616469
t5218300100A2A2A2A2A2
t1218236F203120205031
t5218300100A2A2A2A2A2
t12182400818181818181
t521874A2A2A2A2A2A2A2
t3CF86900A2A2A2A2A2A2
t3DF87900818181818181
t3CF86900A2A2A2A2A2A2
t3DF87900818181818181
t3CF86900A2A2A2A2A2A2
t3DF87900818181818181
t3CF86900A2A2A2A2A2A2
t3DF87900818181818181
t3CF86900A2A2A2A2A2A2

*/
