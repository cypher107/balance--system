#ifndef __TASK4_CAR_TO_B_H
#define __TASK4_CAR_TO_B_H

#include <stdint.h>
#include "Task_Types.h"

typedef enum
{
  TASK4_STAGE_IDLE = 0,
  TASK4_STAGE_START_COMPENSATE,
  TASK4_STAGE_TRACKING,
  TASK4_STAGE_STOP_COMPENSATE,
  TASK4_STAGE_DONE
} Task4_Stage_t;

void Task4_CarToB_Start(uint32_t now_ms);
Task_Result_t Task4_CarToB_Update(uint32_t now_ms);
void Task4_CarToB_Stop(void);
void Task4_CarMotorStart(void);
void Task4_CarMotorStop(void);
Task4_Stage_t Task4_CarToB_GetStage(void);

#endif
