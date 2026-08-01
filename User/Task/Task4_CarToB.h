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
  TASK4_STAGE_DONE,
  TASK4_STAGE_TIMEOUT,
  TASK4_STAGE_START_RETURN_CENTER,
  TASK4_STAGE_STOP_RETURN_CENTER
} Task4_Stage_t;

extern volatile uint8_t g_task4_active_debug;
extern volatile uint8_t g_task4_stage_debug;
extern volatile uint8_t g_task4_car_running_debug;
extern volatile uint8_t g_task4_ballbeam_state_debug;
extern volatile uint32_t g_task4_elapsed_ms_debug;
extern volatile uint32_t g_task4_stage_elapsed_ms_debug;
extern volatile float g_task4_target_position_cm_debug;
extern volatile float g_task4_position_cm_debug;
extern volatile float g_task4_error_cm_debug;
extern volatile float g_task4_kp_debug;
extern volatile float g_task4_ki_debug;
extern volatile float g_task4_kd_debug;
extern volatile float g_task4_position_hold_cm_debug;
extern volatile float g_task4_integral_limit_debug;
extern volatile float g_task4_integral_range_cm_debug;
extern volatile float g_task4_min_correction_pulse_debug;
extern volatile float g_task4_min_full_error_cm_debug;
extern volatile int32_t g_task4_max_motor_pulse_debug;
extern volatile uint8_t g_task4_middle_pid_enable_debug;
extern volatile float g_task4_middle_kp_debug;
extern volatile float g_task4_middle_ki_debug;
extern volatile float g_task4_middle_kd_debug;
extern volatile float g_task4_middle_hold_cm_debug;
extern volatile int32_t g_task4_middle_max_motor_pulse_debug;
extern volatile int32_t g_task4_middle_bias_pulse_debug;

extern volatile float g_task4_tracking_kp_debug;
extern volatile float g_task4_tracking_ki_debug;
extern volatile float g_task4_tracking_kd_debug;
extern volatile float g_task4_tracking_base_speed_debug;
extern volatile float g_task4_tracking_left_scale_debug;
extern volatile float g_task4_tracking_right_scale_debug;

extern volatile int32_t g_task4_start_tilt_pulse_debug;
extern volatile int32_t g_task4_stop_tilt_pulse_debug;
extern volatile uint32_t g_task4_start_tilt_lead_ms_debug;
extern volatile uint32_t g_task4_start_tilt_hold_ms_debug;
extern volatile uint32_t g_task4_start_return_hold_ms_debug;
extern volatile uint32_t g_task4_start_filter_ms_debug;
extern volatile int32_t g_task4_start_filter_start_pulse_debug;
extern volatile int32_t g_task4_start_filter_output_pulse_debug;
extern volatile uint8_t g_task4_start_tilt_sent_debug;
extern volatile uint32_t g_task4_stop_tilt_lead_ms_debug;
extern volatile uint32_t g_task4_stop_tilt_hold_ms_debug;
extern volatile uint32_t g_task4_stop_return_hold_ms_debug;
extern volatile uint32_t g_task4_stop_command_delay_ms_debug;
extern volatile uint32_t g_task4_stop_filter_ms_debug;
extern volatile int32_t g_task4_stop_filter_start_pulse_debug;
extern volatile int32_t g_task4_stop_filter_output_pulse_debug;
extern volatile uint8_t g_task4_stop_tilt_sent_debug;
extern volatile uint32_t g_task4_run_time_ms_debug;
extern volatile uint32_t g_task4_car_elapsed_ms_debug;
extern volatile uint32_t g_task4_stop_trigger_ms_debug;
extern volatile int16_t g_task4_locked_left_speed_debug;
extern volatile int16_t g_task4_locked_right_speed_debug;
extern volatile uint32_t g_task4_car_start_count_debug;
extern volatile uint32_t g_task4_car_stop_count_debug;
extern volatile uint8_t g_task4_car_ramp_active_debug;
extern volatile uint32_t g_task4_car_ramp_elapsed_ms_debug;
extern volatile int16_t g_task4_car_ramp_left_speed_debug;
extern volatile int16_t g_task4_car_ramp_right_speed_debug;
extern volatile uint32_t g_task4_car_ramp_ms_debug;

void Task4_CarToB_Start(uint32_t now_ms);
Task_Result_t Task4_CarToB_Update(uint32_t now_ms);
void Task4_CarToB_Stop(void);
void Task4_CarMotorStart(void);
void Task4_CarMotorStop(void);
Task4_Stage_t Task4_CarToB_GetStage(void);

#endif
