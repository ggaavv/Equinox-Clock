

#include "sys_timer.h"

static volatile long millis_ticks;

void SysTick_Handler(void)
{
  millis_ticks++;
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


