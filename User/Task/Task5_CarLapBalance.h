#ifndef __TASK5_CAR_LAP_BALANCE_H
#define __TASK5_CAR_LAP_BALANCE_H

#include <stdint.h>
#include "Task_Types.h"

void Task5_CarLapBalance_Start(uint32_t now_ms);
Task_Result_t Task5_CarLapBalance_Update(uint32_t now_ms);
void Task5_CarLapBalance_Stop(void);

#endif
