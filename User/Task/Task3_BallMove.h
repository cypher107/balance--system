#ifndef __TASK3_BALL_MOVE_H
#define __TASK3_BALL_MOVE_H

#include <stdint.h>
#include "Task_Types.h"

typedef enum
{
  TASK3_STAGE_IDLE = 0,
  TASK3_STAGE_GO_PLUS5,
  TASK3_STAGE_GO_MINUS5
} Task3_Stage_t;

extern volatile uint8_t g_task3_stage_debug;
extern volatile float g_task3_target_debug;
extern volatile float g_task3_position_debug;
extern volatile float g_task3_error_debug;
extern volatile float g_task3_kp_debug;
extern volatile float g_task3_kd_debug;
extern volatile uint8_t g_task3_switch_delay_10ms_debug;
extern volatile uint16_t g_task3_motor_speed_debug;
extern volatile uint8_t g_task3_motor_acceleration_debug;

void Task3_BallMove_Start(uint32_t now_ms);
Task_Result_t Task3_BallMove_Update(uint32_t now_ms);
void Task3_BallMove_Stop(void);
Task3_Stage_t Task3_BallMove_GetStage(void);

#endif
