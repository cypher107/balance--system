#include "Task3_BallMove.h"

#include "BallBeam.h"
#include "Vision.h"

#define TASK3_PLUS5_TARGET_CM    17.5f
#define TASK3_MINUS5_TARGET_CM   7.0f
#define TASK3_SWITCH_TIME_UNIT_MS 10U
#define TASK3_HARD_PULSE_LIMIT   250

static Task3_Stage_t task3_stage;
static uint32_t task3_start_tick_ms;
static uint32_t task3_vision_sequence;

volatile uint8_t g_task3_stage_debug;
volatile float g_task3_target_debug = TASK3_PLUS5_TARGET_CM;
volatile float g_task3_position_debug;
volatile float g_task3_error_debug;
volatile float g_task3_kp_debug = 21.0f;
volatile float g_task3_kd_debug = 10.0f;
volatile uint8_t g_task3_switch_delay_10ms_debug = 60U;
volatile uint16_t g_task3_motor_speed_debug = 180U;
volatile uint8_t g_task3_motor_acceleration_debug = 80U;

static void Task3_SetStage(Task3_Stage_t stage)
{
  task3_stage = stage;
  g_task3_stage_debug = (uint8_t)stage;
}

static void Task3_ApplyPdSettings(void)
{
  BallBeam_Config_t *config = BallBeam_GetConfig();
  PID_t *position_pid = BallBeam_GetPositionPID();

  config->motor_speed_rpm = g_task3_motor_speed_debug;
  config->motor_acceleration = g_task3_motor_acceleration_debug;
  config->max_motor_pulse = TASK3_HARD_PULSE_LIMIT;
  config->motor_center_pulse = 0;

  g_ballbeam_kp_debug = g_task3_kp_debug;
  g_ballbeam_ki_debug = 0.0f;
  g_ballbeam_kd_debug = g_task3_kd_debug;
  g_ballbeam_position_hold_cm_debug = 0.8f;
  g_ballbeam_output_lpf_cutoff_hz_debug = 0.0f;
  g_ballbeam_output_slew_rate_debug = 0.0f;
  g_ballbeam_min_correction_pulse_debug = 0.0f;

  PID_SetTunings(position_pid,
                 g_task3_kp_debug,
                 0.0f,
                 g_task3_kd_debug);
  PID_SetOutputLimit(position_pid,
                     (float)-TASK3_HARD_PULSE_LIMIT,
                     (float)TASK3_HARD_PULSE_LIMIT);
  PID_SetIntegralLimit(position_pid, 0.0f, 0.0f);
  PID_SetIntegralSeparation(position_pid, 0U, 0.0f);
  PID_SetDeadband(position_pid, 0.0f);
  PID_SetDerivativeFilter(position_pid, 1.0f);
}

void Task3_BallMove_Start(uint32_t now_ms)
{
  Vision_BallSample_t ignored_sample;

  task3_start_tick_ms = now_ms;
  task3_vision_sequence = 0U;
  (void)Vision_GetLatest(&ignored_sample, &task3_vision_sequence);

  Task3_ApplyPdSettings();
  BallBeam_SetTarget(TASK3_PLUS5_TARGET_CM);
  BallBeam_SetRun(1U);

  g_task3_target_debug = TASK3_PLUS5_TARGET_CM;
  g_task3_position_debug = 0.0f;
  g_task3_error_debug = 0.0f;
  Task3_SetStage(TASK3_STAGE_GO_PLUS5);
}

Task_Result_t Task3_BallMove_Update(uint32_t now_ms)
{
  Vision_BallSample_t sample;

  Task3_ApplyPdSettings();

  if ((task3_stage == TASK3_STAGE_GO_PLUS5) &&
      ((now_ms - task3_start_tick_ms) >=
       ((uint32_t)g_task3_switch_delay_10ms_debug *
        TASK3_SWITCH_TIME_UNIT_MS)))
  {
    BallBeam_SetTarget(TASK3_MINUS5_TARGET_CM);
    g_task3_target_debug = TASK3_MINUS5_TARGET_CM;
    Task3_SetStage(TASK3_STAGE_GO_MINUS5);
  }

  if (Vision_GetLatest(&sample, &task3_vision_sequence) != 0U)
  {
    g_task3_position_debug = sample.position_cm;
    g_task3_error_debug = g_task3_target_debug - sample.position_cm;
  }

  return TASK_RESULT_RUNNING;
}

void Task3_BallMove_Stop(void)
{
  BallBeam_SetRun(0U);
  Task3_SetStage(TASK3_STAGE_IDLE);
}

Task3_Stage_t Task3_BallMove_GetStage(void)
{
  return task3_stage;
}
