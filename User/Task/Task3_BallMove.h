#ifndef __TASK3_BALL_MOVE_H
#define __TASK3_BALL_MOVE_H

#include <stdint.h>
#include "Task_Types.h"

typedef enum
{
  TASK3_STAGE_IDLE = 0,
  TASK3_STAGE_GO_PLUS5,
  TASK3_STAGE_GO_MINUS5,
  TASK3_STAGE_DONE,
  TASK3_STAGE_TIMEOUT
} Task3_Stage_t;

extern volatile uint8_t g_task3_stage_debug;
extern volatile uint32_t g_task3_timeout_ms_debug;
extern volatile uint8_t g_task3_auto_finish_debug;
extern volatile float g_task3_target_debug;
extern volatile float g_task3_position_debug;
extern volatile float g_task3_error_debug;
extern volatile uint32_t g_task3_settle_ms_debug;
extern volatile int32_t g_task3_open_loop_pulse_debug;
extern volatile uint16_t g_task3_open_loop_speed_debug;
extern volatile uint8_t g_task3_open_loop_acceleration_debug;
extern volatile float g_task3_open_loop_switch_position_cm_debug;
extern volatile float g_task3_return_kp_debug;
extern volatile float g_task3_return_ki_debug;
extern volatile float g_task3_return_kd_debug;
extern volatile float g_task3_return_integral_limit_debug;
extern volatile float g_task3_return_integral_range_cm_debug;
extern volatile float g_task3_return_min_pulse_debug;
extern volatile float g_task3_return_min_full_error_cm_debug;
extern volatile int32_t g_task3_return_max_pulse_debug;

void Task3_BallMove_Start(uint32_t now_ms);
Task_Result_t Task3_BallMove_Update(uint32_t now_ms);
void Task3_BallMove_Stop(void);
Task3_Stage_t Task3_BallMove_GetStage(void);

#endif
