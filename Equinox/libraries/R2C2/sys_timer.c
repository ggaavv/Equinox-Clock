

#include "sys_timer.h"
#include "lpc_types.h"
#define millis_per_60th_sec 1000/60// (16ms)
#define millis_per_sec 1000
volatile uint32_t USER_MILLIS=0;
volatile uint32_t SEC_MILLIS=0;
volatile int32_t DISPLAY_TIMEOUT=-1;
volatile uint32_t OFF_TIMEOUT;
volatile uint32_t _60th_SEC_COUNT=0;
volatile uint32_t UPDATES_PER_SEC=60;
volatile uint32_t LED_UPDATE_REQUIRED=0;
volatile uint32_t LED_UPDATE_REQUIRED_COUNT=0;


static volatile long millis_ticks;

void SysTick_Handler(void){
	millis_ticks++;
	USER_MILLIS++;

	if(DISPLAY_TIMEOUT>0){
		DISPLAY_TIMEOUT--;
	}

	if(SEC_MILLIS++>millis_per_60th_sec)
		SEC_MILLIS=0;
	else
		_60th_SEC_COUNT++;

	if(LED_UPDATE_REQUIRED_COUNT++>(millis_per_sec/UPDATES_PER_SEC)){
		LED_UPDATE_REQUIRED=TRUE;
		LED_UPDATE_REQUIRED_COUNT=0;
	}
}

long sys_millis(void){
  return millis_ticks;
}

void delay_ms(int ms){
  int start = sys_millis();

  while (sys_millis() - start <= ms)
    ;
}


