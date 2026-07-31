#include "Task4_CarToB.h"

#include "BallBeam.h"

#define TASK4_DEFAULT_TARGET_CM          12.5f
#define TASK4_DEFAULT_MOTOR_SPEED_RPM    300U
#define TASK4_DEFAULT_MOTOR_ACCELERATION 255U

static uint32_t task4_start_tick_ms;

volatile uint8_t g_task4_active_debug;
volatile uint8_t g_task4_ballbeam_state_debug;
volatile uint32_t g_task4_elapsed_ms_debug;
volatile float g_task4_target_position_cm_debug = TASK4_DEFAULT_TARGET_CM;
volatile float g_task4_position_cm_debug;
volatile float g_task4_error_cm_debug;
volatile float g_task4_kp_debug = 3.0f;
volatile float g_task4_ki_debug = 0.0f;
volatile float g_task4_kd_debug = 7.0f;
volatile int32_t g_task4_max_motor_pulse_debug = 250;

static void Task4_ApplyControlSettings(void)
{
  BallBeam_Config_t *config = BallBeam_GetConfig();

  if ((g_task4_kp_debug >= 0.0f) && (g_task4_kp_debug <= 1000.0f))
  {
    g_ballbeam_kp_debug = g_task4_kp_debug;
  }
  if ((g_task4_ki_debug >= 0.0f) && (g_task4_ki_debug <= 1000.0f))
  {
    g_ballbeam_ki_debug = g_task4_ki_debug;
  }
  if ((g_task4_kd_debug >= 0.0f) && (g_task4_kd_debug <= 1000.0f))
  {
    g_ballbeam_kd_debug = g_task4_kd_debug;
  }
  if ((g_task4_max_motor_pulse_debug >= 1) &&
      (g_task4_max_motor_pulse_debug <= 250))
  {
    config->max_motor_pulse = g_task4_max_motor_pulse_debug;
  }

  config->motor_speed_rpm = TASK4_DEFAULT_MOTOR_SPEED_RPM;
  config->motor_acceleration = TASK4_DEFAULT_MOTOR_ACCELERATION;

  /* Task 3 enables a minimum correction pulse; center hold uses pure PID. */
  g_ballbeam_position_hold_cm_debug = 0.5f;
  g_ballbeam_min_correction_pulse_debug = 0.0f;
}

void Task4_CarToB_Start(uint32_t now_ms)
{
  task4_start_tick_ms = now_ms;
  g_task4_active_debug = 1U;
  g_task4_elapsed_ms_debug = 0U;
  g_task4_position_cm_debug = g_ballbeam_position_debug;
  g_task4_error_cm_debug =
      g_task4_target_position_cm_debug - g_task4_position_cm_debug;

  Task4_ApplyControlSettings();
  BallBeam_SetTarget(g_task4_target_position_cm_debug);
  BallBeam_SetRun(1U);
  g_task4_ballbeam_state_debug = (uint8_t)BallBeam_GetState();
}

Task_Result_t Task4_CarToB_Update(uint32_t now_ms)
{
  Task4_ApplyControlSettings();
  BallBeam_SetTarget(g_task4_target_position_cm_debug);

  g_task4_elapsed_ms_debug = now_ms - task4_start_tick_ms;
  g_task4_position_cm_debug = g_ballbeam_position_debug;
  g_task4_error_cm_debug =
      BallBeam_GetTarget() - g_task4_position_cm_debug;
  g_task4_ballbeam_state_debug = (uint8_t)BallBeam_GetState();

  return TASK_RESULT_RUNNING;
}

void Task4_CarToB_Stop(void)
{
  BallBeam_SetRun(0U);
  g_task4_active_debug = 0U;
  g_task4_ballbeam_state_debug = (uint8_t)BallBeam_GetState();
}
