#ifndef __TASK4_CAR_TO_B_H
#define __TASK4_CAR_TO_B_H

#include <stdint.h>
#include "Task_Types.h"

void Task4_CarToB_Start(uint32_t now_ms);
Task_Result_t Task4_CarToB_Update(uint32_t now_ms);
void Task4_CarToB_Stop(void);

#endif
