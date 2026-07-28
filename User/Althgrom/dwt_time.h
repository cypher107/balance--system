#ifndef __DWT_TIME_H
#define __DWT_TIME_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

void DWT_Time_Init(void);
uint32_t DWT_Time_GetCycles(void);
float DWT_Time_CyclesToSeconds(uint32_t cycles);
float DWT_Time_DeltaSeconds(uint32_t current_cycles, uint32_t previous_cycles);

#ifdef __cplusplus
}
#endif

#endif
