/*

*/

#ifndef CShiftPWM_h
#define CShiftPWM_h

#include "lpc_types.h"

void TIMER0_IRQHandler(void);
void LatchIn(void);
void DMA_IRQHandler (void);
void LED_init();
void LED_test();
void SetRGB(uint8_t group, uint8_t v0, uint8_t v1, uint8_t v2);
void calulateLEDMIBAMBits();


#endif
