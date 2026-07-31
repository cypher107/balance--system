#ifndef __TASK4_CAR_TO_B_H
#define __TASK4_CAR_TO_B_H

#include <stdint.h>
#include "Task_Types.h"

extern volatile uint8_t g_task4_active_debug;
extern volatile uint8_t g_task4_ballbeam_state_debug;
extern volatile uint32_t g_task4_elapsed_ms_debug;
extern volatile float g_task4_target_position_cm_debug;
extern volatile float g_task4_position_cm_debug;
extern volatile float g_task4_error_cm_debug;
extern volatile float g_task4_kp_debug;
extern volatile float g_task4_ki_debug;
extern volatile float g_task4_kd_debug;
extern volatile int32_t g_task4_max_motor_pulse_debug;

void Task4_CarToB_Start(uint32_t now_ms);
Task_Result_t Task4_CarToB_Update(uint32_t now_ms);
void Task4_CarToB_Stop(void);

#endif
