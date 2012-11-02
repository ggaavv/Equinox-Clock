/*

*/

#ifndef CShiftPWM_h
#define CShiftPWM_h

#include "lpc_types.h"

#define MAX_DELAY 1024 // delay uses number squared

#define MAX_PATTERNS 8
#define MAX_PATTERNS_LETTERS 40
const char LED_PATTERN_NAME[MAX_PATTERNS][MAX_PATTERNS_LETTERS] = {
	"All LED's Off"
	"Clock",
	"One by one",
	"Rainbow Rotating",
	"Simple all colors",
	"One by one smooth all on all off red",
	"One by one smooth all on all off green",
	"One by one smooth all on all off blue"
};

void TIMER0_IRQHandler(void);
void TIMER1_IRQHandler(void);
void TIMER2_IRQHandler(void);
void DMA_IRQHandler (void);
void LatchIn(void);
void WaitForSend(void);
void LED_init(void);
void SetHue(uint32_t led, uint32_t hue);
void SetRGB(int32_t group, uint8_t v0, uint8_t v1, uint8_t v2);
void SetLED(uint8_t led, uint8_t v0);
void resetLeds(void);
void calulateLEDMIBAMBits(void);
void Set_LED_Pattern(uint8_t no, uint16_t delay, uint8_t bri);
void Get_LED_Pattern(uint8_t * no, uint16_t * delay, uint8_t * bri);
uint8_t SetBrightness(uint8_t bri);
uint8_t GetBrightness(void);
void LED_loop(void);
void LED_simple_all_colors(void);

#endif
