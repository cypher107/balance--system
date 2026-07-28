#include "dwt_time.h"

#include "stm32f1xx.h"

void DWT_Time_Init(void)
{
  CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
  DWT->CYCCNT = 0U;
  DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
}

uint32_t DWT_Time_GetCycles(void)
{
  return DWT->CYCCNT;
}

float DWT_Time_CyclesToSeconds(uint32_t cycles)
{
  if (SystemCoreClock == 0U)
  {
    return 0.0f;
  }

  return (float)cycles / (float)SystemCoreClock;
}

float DWT_Time_DeltaSeconds(uint32_t current_cycles, uint32_t previous_cycles)
{
  return DWT_Time_CyclesToSeconds(current_cycles - previous_cycles);
}
