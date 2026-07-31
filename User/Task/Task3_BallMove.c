#include "Task3_BallMove.h"

#include <limits.h>

#include "BallBeam.h"
#include "Vision.h"

#define TASK3_CENTER_CM             12.5f
#define TASK3_PLUS5_TARGET_CM       (TASK3_CENTER_CM + 5.0f)
#define TASK3_MINUS5_TARGET_CM      (TASK3_CENTER_CM - 6.0f)
#define TASK3_POSITION_TOLERANCE_CM 0.5f
#define TASK3_MINUS_SETTLE_MS       500U
#define TASK3_MAX_SAMPLE_GAP_MS     150U

static Task3_Stage_t task3_stage;
static uint32_t task3_settle_start_ms;
static uint32_t task3_last_sample_tick_ms;
static uint32_t task3_open_loop_hold_start_ms;
static uint32_t task3_vision_sequence;
static uint8_t task3_settle_active;
static int32_t task3_last_open_loop_pulse;
static uint16_t task3_last_open_loop_speed;
static uint8_t task3_last_open_loop_acceleration;

volatile uint8_t g_task3_stage_debug;
volatile uint32_t g_task3_timeout_ms_debug;
volatile uint8_t g_task3_auto_finish_debug;
volatile float g_task3_target_debug;
volatile float g_task3_position_debug;
volatile float g_task3_error_debug;
volatile uint32_t g_task3_settle_ms_debug;
volatile int32_t g_task3_open_loop_pulse_debug = 40;
volatile uint16_t g_task3_open_loop_speed_debug = 100U;
volatile uint8_t g_task3_open_loop_acceleration_debug = 80U;
volatile float g_task3_open_loop_switch_position_cm_debug = 16.5f;
volatile uint32_t g_task3_open_loop_hold_ms_debug = 50U;
volatile uint32_t g_task3_open_loop_hold_elapsed_ms_debug;
volatile uint8_t g_task3_open_loop_hold_active_debug;
volatile float g_task3_return_kp_debug = 7.0f;//14
volatile float g_task3_return_ki_debug = 0.0f;
volatile float g_task3_return_kd_debug = 7.0f;//30
volatile float g_task3_return_hold_cm_debug = 0.1f;
volatile float g_task3_return_integral_limit_debug = 20.0f;
volatile float g_task3_return_integral_range_cm_debug = 4.0f;
volatile float g_task3_return_min_pulse_debug = 40.0f;
volatile float g_task3_return_min_full_error_cm_debug = 2.0f;
volatile int32_t g_task3_return_max_pulse_debug = 100;

static float Task3_Abs(float value)
{
  return (value >= 0.0f) ? value : -value;
}

static void Task3_ResetSettling(void)
{
  task3_settle_start_ms = 0U;
  task3_settle_active = 0U;
  g_task3_settle_ms_debug = 0U;
}

static void Task3_SetStage(Task3_Stage_t stage)
{
  task3_stage = stage;
  g_task3_stage_debug = (uint8_t)stage;
  Task3_ResetSettling();
}

static void Task3_ApplyOpenLoopCommand(void)
{
  BallBeam_Config_t *config = BallBeam_GetConfig();

  if ((g_task3_open_loop_pulse_debug == task3_last_open_loop_pulse) &&
      (g_task3_open_loop_speed_debug == task3_last_open_loop_speed) &&
      (g_task3_open_loop_acceleration_debug ==
       task3_last_open_loop_acceleration))
  {
    return;
  }

  config->motor_speed_rpm = g_task3_open_loop_speed_debug;
  config->motor_acceleration = g_task3_open_loop_acceleration_debug;
  BallBeam_CommandAbsolutePulse(g_task3_open_loop_pulse_debug);

  task3_last_open_loop_pulse = g_task3_open_loop_pulse_debug;
  task3_last_open_loop_speed = g_task3_open_loop_speed_debug;
  task3_last_open_loop_acceleration =
      g_task3_open_loop_acceleration_debug;
}

static void Task3_ApplyReturnTunings(void)
{
  BallBeam_Config_t *config = BallBeam_GetConfig();
  PID_t *position_pid = BallBeam_GetPositionPID();

  if ((g_task3_return_kp_debug >= 0.0f) &&
      (g_task3_return_kp_debug <= 1000.0f))
  {
    g_ballbeam_kp_debug = g_task3_return_kp_debug;
  }
  if ((g_task3_return_ki_debug >= 0.0f) &&
      (g_task3_return_ki_debug <= 1000.0f))
  {
    g_ballbeam_ki_debug = g_task3_return_ki_debug;
  }
  if ((g_task3_return_kd_debug >= 0.0f) &&
      (g_task3_return_kd_debug <= 1000.0f))
  {
    g_ballbeam_kd_debug = g_task3_return_kd_debug;
  }
  if ((g_task3_return_hold_cm_debug >= 0.0f) &&
      (g_task3_return_hold_cm_debug <= 2.0f))
  {
    g_ballbeam_position_hold_cm_debug = g_task3_return_hold_cm_debug;
  }
  if ((g_task3_return_max_pulse_debug >= 1) &&
      (g_task3_return_max_pulse_debug <= 250))
  {
    config->max_motor_pulse = g_task3_return_max_pulse_debug;
  }
  if ((g_task3_return_integral_limit_debug >= 0.0f) &&
      (g_task3_return_integral_limit_debug <= 100.0f))
  {
    PID_SetIntegralLimit(position_pid,
                         -g_task3_return_integral_limit_debug,
                         g_task3_return_integral_limit_debug);
  }
  if ((g_task3_return_integral_range_cm_debug > 0.0f) &&
      (g_task3_return_integral_range_cm_debug <= 10.0f))
  {
    PID_SetIntegralSeparation(position_pid,
                              1U,
                              g_task3_return_integral_range_cm_debug);
  }
  if ((g_task3_return_min_pulse_debug >= 0.0f) &&
      (g_task3_return_min_pulse_debug <= 100.0f))
  {
    g_ballbeam_min_correction_pulse_debug =
        g_task3_return_min_pulse_debug;
  }
  if ((g_task3_return_min_full_error_cm_debug > 0.5f) &&
      (g_task3_return_min_full_error_cm_debug <= 10.0f))
  {
    g_ballbeam_min_correction_full_error_cm_debug =
        g_task3_return_min_full_error_cm_debug;
  }
}

static void Task3_StartReturn(uint32_t now_ms)
{
  Task3_ApplyReturnTunings();
  BallBeam_SetTarget(TASK3_MINUS5_TARGET_CM);
  BallBeam_SetRun(1U);
  g_task3_target_debug = TASK3_MINUS5_TARGET_CM;
  g_task3_open_loop_hold_active_debug = 0U;
  Task3_SetStage(TASK3_STAGE_GO_MINUS5);
}


static uint8_t Task3_TargetSettled(float position_cm,
                                   float target_cm,
                                   uint32_t required_ms,
                                   uint32_t now_ms)
{
  float error = target_cm - position_cm;

  g_task3_target_debug = target_cm;
  g_task3_position_debug = position_cm;
  g_task3_error_debug = error;

  if (Task3_Abs(error) > TASK3_POSITION_TOLERANCE_CM)
  {
    Task3_ResetSettling();
    return 0U;
  }

  if (task3_settle_active == 0U)
  {
    task3_settle_start_ms = now_ms;
    task3_settle_active = 1U;
  }
  g_task3_settle_ms_debug = now_ms - task3_settle_start_ms;
  return (g_task3_settle_ms_debug >= required_ms) ? 1U : 0U;
}

void Task3_BallMove_Start(uint32_t now_ms)
{
  Vision_BallSample_t ignored_sample;

  task3_vision_sequence = 0U;
  (void)Vision_GetLatest(&ignored_sample, &task3_vision_sequence);
  task3_last_sample_tick_ms = 0U;
  task3_open_loop_hold_start_ms = 0U;
  task3_last_open_loop_pulse = INT32_MAX;
  task3_last_open_loop_speed = 0U;
  task3_last_open_loop_acceleration = 0U;
  g_task3_position_debug = 0.0f;
  g_task3_error_debug = 0.0f;
  g_task3_open_loop_hold_elapsed_ms_debug = 0U;
  g_task3_open_loop_hold_active_debug = 0U;
  BallBeam_SetTarget(TASK3_PLUS5_TARGET_CM);
  BallBeam_SetRun(0U);
  Task3_ApplyOpenLoopCommand();
  g_task3_target_debug = TASK3_PLUS5_TARGET_CM;
  Task3_SetStage(TASK3_STAGE_GO_PLUS5);
}

Task_Result_t Task3_BallMove_Update(uint32_t now_ms)
{
  Vision_BallSample_t sample;

  if (task3_stage == TASK3_STAGE_GO_PLUS5)
  {
    Task3_ApplyOpenLoopCommand();
    if (g_task3_open_loop_hold_active_debug != 0U)
    {
      g_task3_open_loop_hold_elapsed_ms_debug =
          now_ms - task3_open_loop_hold_start_ms;
      if (g_task3_open_loop_hold_elapsed_ms_debug >=
          g_task3_open_loop_hold_ms_debug)
      {
        Task3_StartReturn(now_ms);
      }
    }
  }
  else if (task3_stage == TASK3_STAGE_GO_MINUS5)
  {
    Task3_ApplyReturnTunings();
  }

  if (Vision_GetLatest(&sample, &task3_vision_sequence) == 0U)
  {
    if ((task3_settle_active != 0U) &&
        ((now_ms - task3_last_sample_tick_ms) > TASK3_MAX_SAMPLE_GAP_MS))
    {
      Task3_ResetSettling();
    }
    return TASK_RESULT_RUNNING;
  }

  if ((task3_last_sample_tick_ms != 0U) &&
      ((now_ms - task3_last_sample_tick_ms) > TASK3_MAX_SAMPLE_GAP_MS))
  {
    Task3_ResetSettling();
  }
  task3_last_sample_tick_ms = now_ms;

  if (task3_stage == TASK3_STAGE_GO_PLUS5)
  {
    g_task3_target_debug = TASK3_PLUS5_TARGET_CM;
    g_task3_position_debug = sample.position_cm;
    g_task3_error_debug = TASK3_PLUS5_TARGET_CM - sample.position_cm;

    if (sample.position_cm >=
        g_task3_open_loop_switch_position_cm_debug)
    {
      if (g_task3_open_loop_hold_active_debug == 0U)
      {
        task3_open_loop_hold_start_ms = now_ms;
        g_task3_open_loop_hold_elapsed_ms_debug = 0U;
        g_task3_open_loop_hold_active_debug = 1U;
      }
    }
  }
  else if (task3_stage == TASK3_STAGE_GO_MINUS5)
  {
    (void)Task3_TargetSettled(sample.position_cm,
                              TASK3_MINUS5_TARGET_CM,
                              TASK3_MINUS_SETTLE_MS,
                              now_ms);
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
