#ifndef __TASK2_CAR_LAP_H
#define __TASK2_CAR_LAP_H

#include <stdint.h>
#include "Task_Types.h"

typedef enum
{
  TASK2_STAGE_IDLE = 0,
  TASK2_STAGE_LEAVE_START,
  TASK2_STAGE_LAP,
  TASK2_STAGE_FINISH_CONFIRM,
  TASK2_STAGE_DONE,
  TASK2_STAGE_TIMEOUT
} Task2_Stage_t;

extern volatile uint8_t g_task2_stage_debug;
extern volatile float g_task2_elapsed_s_debug;
extern volatile float g_task2_min_lap_time_s_debug;
extern volatile uint8_t g_task2_finish_sensor_count_debug;
extern volatile float g_task2_finish_confirm_s_debug;

void Task2_CarLap_Start(uint32_t now_ms);
Task_Result_t Task2_CarLap_Update(uint32_t now_ms);
void Task2_CarLap_Stop(void);
Task2_Stage_t Task2_CarLap_GetStage(void);

#endif
