#ifndef __TASK6_ARBITRARY_POSITION_H
#define __TASK6_ARBITRARY_POSITION_H

#include <stdint.h>
#include "Task_Types.h"

void Task6_ArbitraryPosition_Start(uint32_t now_ms);
Task_Result_t Task6_ArbitraryPosition_Update(uint32_t now_ms);
void Task6_ArbitraryPosition_Stop(void);

#endif
