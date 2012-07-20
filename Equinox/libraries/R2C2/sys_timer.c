

#include "sys_timer.h"
#include "lpc_types.h"
extern volatile uint32_t USER_MILLIS;

static volatile long millis_ticks;

void SysTick_Handler(void)
{
  millis_ticks++;
  USER_MILLIS++;
}

long sys_millis(void)
{
  return millis_ticks;
}

void delay_ms(int ms)
{
  int start = sys_millis();

  while (sys_millis() - start <= ms)
    ;
}


