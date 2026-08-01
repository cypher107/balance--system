#include "Task5_CarLapBalance.h"

#include "BallBeam.h"
#include "Task3_BallMove.h"
#include "motor.h"
#include "tracking.h"

#define TASK5_DURATION_MS              30000U
#define TASK5_CONTROL_PERIOD_MS        10U
#define TASK5_BALL_TARGET_CM           12.5f
#define TASK5_BALL_DEADBAND_CM         0.8f
#define TASK5_BALL_PULSE_LIMIT         250

#define TASK5_TRACK_KP                 18.0f
#define TASK5_TRACK_KI                 0.0f
#define TASK5_TRACK_KD                 0.0f
#define TASK5_TRACK_BASE_SPEED         145.0f
#define TASK5_TRACK_LEFT_SCALE         1.3f
#define TASK5_TRACK_RIGHT_SCALE        1.0f

static uint32_t task5_start_tick_ms;
static uint32_t task5_last_control_tick_ms;
static uint8_t task5_running;

static void Task5_ApplySettings(void)
{
  BallBeam_Config_t *config = BallBeam_GetConfig();
  PID_t *position_pid = BallBeam_GetPositionPID();

  g_tracking_kp_debug = TASK5_TRACK_KP;
  g_tracking_ki_debug = TASK5_TRACK_KI;
  g_tracking_kd_debug = TASK5_TRACK_KD;
  g_tracking_base_speed_debug = TASK5_TRACK_BASE_SPEED;
  g_tracking_left_speed_scale_debug = TASK5_TRACK_LEFT_SCALE;
  g_tracking_right_speed_scale_debug = TASK5_TRACK_RIGHT_SCALE;

  config->motor_speed_rpm = g_task3_motor_speed_debug;
  config->motor_acceleration = g_task3_motor_acceleration_debug;
  config->max_motor_pulse = TASK5_BALL_PULSE_LIMIT;
  config->motor_center_pulse = 0;

  g_ballbeam_kp_debug = g_task3_kp_debug;
  g_ballbeam_ki_debug = 0.0f;
  g_ballbeam_kd_debug = g_task3_kd_debug;
  g_ballbeam_position_hold_cm_debug = TASK5_BALL_DEADBAND_CM;
  g_ballbeam_output_lpf_cutoff_hz_debug = 0.0f;
  g_ballbeam_output_slew_rate_debug = 0.0f;
  g_ballbeam_min_correction_pulse_debug = 0.0f;

  PID_SetTunings(position_pid,
                 g_task3_kp_debug,
                 0.0f,
                 g_task3_kd_debug);
  PID_SetOutputLimit(position_pid,
                     (float)-TASK5_BALL_PULSE_LIMIT,
                     (float)TASK5_BALL_PULSE_LIMIT);
  PID_SetIntegralLimit(position_pid, 0.0f, 0.0f);
  PID_SetIntegralSeparation(position_pid, 0U, 0.0f);
  PID_SetDeadband(position_pid, 0.0f);
  PID_SetDerivativeFilter(position_pid, 1.0f);
}

void Task5_CarLapBalance_Start(uint32_t now_ms)
{
  int16_t left_speed;
  int16_t right_speed;

  task5_start_tick_ms = now_ms;
  task5_last_control_tick_ms = now_ms - TASK5_CONTROL_PERIOD_MS;
  task5_running = 1U;

  Task5_ApplySettings();
  Tracking_Reset();
  left_speed = (int16_t)(TASK5_TRACK_BASE_SPEED *
                         TASK5_TRACK_LEFT_SCALE);
  right_speed = (int16_t)(TASK5_TRACK_BASE_SPEED *
                          TASK5_TRACK_RIGHT_SCALE);
  Motor_SetSpeed(left_speed, right_speed);
  BallBeam_SetTarget(TASK5_BALL_TARGET_CM);
  BallBeam_SetRun(1U);
}

Task_Result_t Task5_CarLapBalance_Update(uint32_t now_ms)
{
  if (task5_running == 0U)
  {
    return TASK_RESULT_STOPPED;
  }

  if ((now_ms - task5_start_tick_ms) >= TASK5_DURATION_MS)
  {
    Task5_CarLapBalance_Stop();
    return TASK_RESULT_DONE;
  }

  if ((now_ms - task5_last_control_tick_ms) >=
      TASK5_CONTROL_PERIOD_MS)
  {
    task5_last_control_tick_ms = now_ms;
    Tracking_Process();
  }

  return TASK_RESULT_RUNNING;
}

void Task5_CarLapBalance_Stop(void)
{
  Motor_Stop();
  Tracking_Reset();
  BallBeam_SetRun(0U);
  task5_running = 0U;
}
