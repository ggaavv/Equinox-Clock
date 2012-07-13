/*

*/

#ifndef CShiftPWM_h
#define CShiftPWM_h


void TIMER0_IRQHandler(void);
void DMA_IRQHandler (void);
void LED_init();
void LED_test();
void SetRGB(uint8_t group, uint8_t v0, uint8_t v1, uint8_t v2);
static inline void calulateLEDMIBAMBits();


#endif
