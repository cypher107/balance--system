#include "Task6_ArbitraryPosition.h"

#include "BallBeam.h"
#include "Task3_BallMove.h"

#define TASK6_DEFAULT_TARGET_CM    12.5f
#define TASK6_MAX_TARGET_CM        25U
#define TASK6_DEADBAND_CM          0.8f
#define TASK6_PULSE_LIMIT          250

static float task6_target_cm = TASK6_DEFAULT_TARGET_CM;
static uint8_t task6_running;

static void Task6_ApplyPdSettings(void)
{
  BallBeam_Config_t *config = BallBeam_GetConfig();
  PID_t *position_pid = BallBeam_GetPositionPID();

  config->motor_speed_rpm = g_task3_motor_speed_debug;
  config->motor_acceleration = g_task3_motor_acceleration_debug;
  config->max_motor_pulse = TASK6_PULSE_LIMIT;
  config->motor_center_pulse = 0;

  g_ballbeam_kp_debug = g_task3_kp_debug;
  g_ballbeam_ki_debug = 0.0f;
  g_ballbeam_kd_debug = g_task3_kd_debug;
  g_ballbeam_position_hold_cm_debug = TASK6_DEADBAND_CM;
  g_ballbeam_output_lpf_cutoff_hz_debug = 0.0f;
  g_ballbeam_output_slew_rate_debug = 0.0f;
  g_ballbeam_min_correction_pulse_debug = 0.0f;

  PID_SetTunings(position_pid,
                 g_task3_kp_debug,
                 0.0f,
                 g_task3_kd_debug);
  PID_SetOutputLimit(position_pid,
                     (float)-TASK6_PULSE_LIMIT,
                     (float)TASK6_PULSE_LIMIT);
  PID_SetIntegralLimit(position_pid, 0.0f, 0.0f);
  PID_SetIntegralSeparation(position_pid, 0U, 0.0f);
  PID_SetDeadband(position_pid, 0.0f);
  PID_SetDerivativeFilter(position_pid, 1.0f);
}

void Task6_ArbitraryPosition_Start(uint32_t now_ms)
{
  (void)now_ms;

  Task6_ApplyPdSettings();
  BallBeam_SetTarget(task6_target_cm);
  BallBeam_SetRun(1U);
  task6_running = 1U;
}

Task_Result_t Task6_ArbitraryPosition_Update(uint32_t now_ms)
{
  (void)now_ms;
  return (task6_running != 0U) ?
      TASK_RESULT_RUNNING : TASK_RESULT_STOPPED;
}

void Task6_ArbitraryPosition_Stop(void)
{
  BallBeam_SetRun(0U);
  task6_running = 0U;
}

void Task6_ArbitraryPosition_SetTarget(uint8_t target_cm)
{
  if (target_cm > TASK6_MAX_TARGET_CM)
  {
    target_cm = TASK6_MAX_TARGET_CM;
  }

  task6_target_cm = (float)target_cm;
  if (task6_running != 0U)
  {
    BallBeam_SetTarget(task6_target_cm);
  }
}
