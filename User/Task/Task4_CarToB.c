#include "Task4_CarToB.h"

#include "BallBeam.h"
#include "motor.h"
#include "stm32f1xx_hal.h"
#include "tracking.h"

#define TASK4_CONTROL_PERIOD_MS        10U
#define TASK4_TIMEOUT_MS               20000U
#define TASK4_DEFAULT_TARGET_CM        12.5f
#define TASK4_DEFAULT_MOTOR_SPEED_RPM  300U
#define TASK4_DEFAULT_MOTOR_ACCEL      255U
#define TASK4_DEFAULT_RUN_TIME_MS        8000U

static Task4_Stage_t task4_stage;
static uint32_t task4_start_tick_ms;
static uint32_t task4_stage_start_tick_ms;
static uint32_t task4_car_start_tick_ms;
static uint32_t task4_last_control_tick_ms;
static uint32_t task4_car_ramp_start_tick_ms;
static uint8_t task4_car_motor_enabled;
static uint8_t task4_car_ramp_active;
static int16_t task4_car_ramp_target_left_speed;
static int16_t task4_car_ramp_target_right_speed;
static uint8_t task4_start_tilt_sent;
static uint32_t task4_start_tilt_sent_tick_ms;
static int32_t task4_start_ramp_start_pulse;
static int32_t task4_start_ramp_last_pulse;
static uint8_t task4_stop_tilt_ramp_started;
static uint8_t task4_stop_tilt_sent;
static int32_t task4_stop_ramp_start_pulse;
static int32_t task4_stop_ramp_last_pulse;

volatile uint8_t g_task4_active_debug;
volatile uint8_t g_task4_stage_debug;
volatile uint8_t g_task4_car_running_debug;
volatile uint8_t g_task4_ballbeam_state_debug;
volatile uint32_t g_task4_elapsed_ms_debug;
volatile uint32_t g_task4_stage_elapsed_ms_debug;
volatile float g_task4_target_position_cm_debug = TASK4_DEFAULT_TARGET_CM;
volatile float g_task4_position_cm_debug;
volatile float g_task4_error_cm_debug;
volatile float g_task4_kp_debug = 12.0f;
volatile float g_task4_ki_debug = 0.0f;
volatile float g_task4_kd_debug = 7.0f;
volatile float g_task4_position_hold_cm_debug = 0.1f;
volatile float g_task4_integral_limit_debug = 20.0f;
volatile float g_task4_integral_range_cm_debug = 4.0f;
volatile float g_task4_min_correction_pulse_debug = 60.0f;
volatile float g_task4_min_full_error_cm_debug = 2.0f;
volatile int32_t g_task4_max_motor_pulse_debug = 100;

volatile uint8_t g_task4_middle_pid_enable_debug = 1U;
volatile float g_task4_middle_kp_debug = 8.0f;
volatile float g_task4_middle_ki_debug = 0.0f;
volatile float g_task4_middle_kd_debug = 9.0f;
volatile float g_task4_middle_hold_cm_debug = 0.8f;
volatile int32_t g_task4_middle_max_motor_pulse_debug = 20;
volatile int32_t g_task4_middle_bias_pulse_debug = 0;

volatile float g_task4_tracking_kp_debug = 18.0f;
volatile float g_task4_tracking_ki_debug = 0.0f;
volatile float g_task4_tracking_kd_debug = 0.0f;
volatile float g_task4_tracking_base_speed_debug = 145.0f;
volatile float g_task4_tracking_left_scale_debug = 1.3f;
volatile float g_task4_tracking_right_scale_debug = 1.0f;

volatile int32_t g_task4_start_tilt_pulse_debug = -40;
volatile int32_t g_task4_stop_tilt_pulse_debug = 40;
volatile uint32_t g_task4_start_tilt_lead_ms_debug = 200U;
volatile uint32_t g_task4_start_tilt_hold_ms_debug = 400U;
volatile uint32_t g_task4_start_return_hold_ms_debug = 200U;
volatile uint32_t g_task4_start_filter_ms_debug = 200U;
volatile int32_t g_task4_start_filter_start_pulse_debug;
volatile int32_t g_task4_start_filter_output_pulse_debug;
volatile uint8_t g_task4_start_tilt_sent_debug;
volatile uint32_t g_task4_stop_tilt_lead_ms_debug = 300U;
volatile uint32_t g_task4_stop_tilt_hold_ms_debug = 400U;
volatile uint32_t g_task4_stop_return_hold_ms_debug = 200U;
volatile uint32_t g_task4_stop_command_delay_ms_debug = 50U;
volatile uint32_t g_task4_stop_filter_ms_debug = 200U;
volatile int32_t g_task4_stop_filter_start_pulse_debug;
volatile int32_t g_task4_stop_filter_output_pulse_debug;
volatile uint8_t g_task4_stop_tilt_sent_debug;
volatile uint32_t g_task4_run_time_ms_debug = TASK4_DEFAULT_RUN_TIME_MS;
volatile uint32_t g_task4_car_elapsed_ms_debug;
volatile uint32_t g_task4_stop_trigger_ms_debug;
volatile int16_t g_task4_locked_left_speed_debug;
volatile int16_t g_task4_locked_right_speed_debug;
volatile uint32_t g_task4_car_start_count_debug;
volatile uint32_t g_task4_car_stop_count_debug;
volatile uint8_t g_task4_car_ramp_active_debug;
volatile uint32_t g_task4_car_ramp_elapsed_ms_debug;
volatile int16_t g_task4_car_ramp_left_speed_debug;
volatile int16_t g_task4_car_ramp_right_speed_debug;
volatile uint32_t g_task4_car_ramp_ms_debug = 500U;

static void Task4_SetStage(Task4_Stage_t stage, uint32_t now_ms)
{
  task4_stage = stage;
  task4_stage_start_tick_ms = now_ms;
  g_task4_stage_debug = (uint8_t)stage;
  g_task4_stage_elapsed_ms_debug = 0U;
}

static int32_t Task4_FilterPulse(int32_t start_pulse,
                                 int32_t target_pulse,
                                 uint32_t elapsed_ms,
                                 uint32_t filter_time_ms)
{
  if ((filter_time_ms == 0U) || (elapsed_ms >= filter_time_ms))
  {
    return target_pulse;
  }

  return start_pulse +
      (int32_t)(((int64_t)(target_pulse - start_pulse) *
                 (int64_t)elapsed_ms) /
                (int64_t)filter_time_ms);
}

static void Task4_ApplyTrackingSettings(void)
{
  g_tracking_kp_debug = g_task4_tracking_kp_debug;
  g_tracking_ki_debug = g_task4_tracking_ki_debug;
  g_tracking_kd_debug = g_task4_tracking_kd_debug;
  g_tracking_base_speed_debug = g_task4_tracking_base_speed_debug;
  g_tracking_left_speed_scale_debug = g_task4_tracking_left_scale_debug;
  g_tracking_right_speed_scale_debug = g_task4_tracking_right_scale_debug;
}

static void Task4_ApplyBallBeamSettings(void)
{
  BallBeam_Config_t *config = BallBeam_GetConfig();
  PID_t *position_pid = BallBeam_GetPositionPID();
  float kp = g_task4_kp_debug;
  float ki = g_task4_ki_debug;
  float kd = g_task4_kd_debug;
  float position_hold_cm = g_task4_position_hold_cm_debug;
  float min_correction_pulse = g_task4_min_correction_pulse_debug;
  int32_t max_motor_pulse = g_task4_max_motor_pulse_debug;
  int32_t motor_center_pulse = 0;

  if ((task4_stage == TASK4_STAGE_TRACKING) &&
      (g_task4_middle_pid_enable_debug != 0U))
  {
    kp = g_task4_middle_kp_debug;
    ki = g_task4_middle_ki_debug;
    kd = g_task4_middle_kd_debug;
    position_hold_cm = g_task4_middle_hold_cm_debug;
    min_correction_pulse = 0.0f;
    max_motor_pulse = g_task4_middle_max_motor_pulse_debug;
    motor_center_pulse = g_task4_middle_bias_pulse_debug;
  }

  if ((kp >= 0.0f) && (kp <= 1000.0f))
  {
    g_ballbeam_kp_debug = kp;
  }
  if ((ki >= 0.0f) && (ki <= 1000.0f))
  {
    g_ballbeam_ki_debug = ki;
  }
  if ((kd >= 0.0f) && (kd <= 1000.0f))
  {
    g_ballbeam_kd_debug = kd;
  }
  if ((max_motor_pulse >= 1) && (max_motor_pulse <= 250))
  {
    config->max_motor_pulse = max_motor_pulse;
  }
  if ((position_hold_cm >= 0.0f) && (position_hold_cm <= 2.0f))
  {
    g_ballbeam_position_hold_cm_debug = position_hold_cm;
  }
  if ((g_task4_integral_limit_debug >= 0.0f) &&
      (g_task4_integral_limit_debug <= 100.0f))
  {
    PID_SetIntegralLimit(position_pid,
                         -g_task4_integral_limit_debug,
                         g_task4_integral_limit_debug);
  }
  if ((g_task4_integral_range_cm_debug > 0.0f) &&
      (g_task4_integral_range_cm_debug <= 10.0f))
  {
    PID_SetIntegralSeparation(position_pid,
                              1U,
                              g_task4_integral_range_cm_debug);
  }
  if ((min_correction_pulse >= 0.0f) &&
      (min_correction_pulse <= 100.0f))
  {
    g_ballbeam_min_correction_pulse_debug = min_correction_pulse;
  }
  if ((g_task4_min_full_error_cm_debug > 0.5f) &&
      (g_task4_min_full_error_cm_debug <= 10.0f))
  {
    g_ballbeam_min_correction_full_error_cm_debug =
        g_task4_min_full_error_cm_debug;
  }

  config->motor_center_pulse = motor_center_pulse;
  config->motor_speed_rpm = TASK4_DEFAULT_MOTOR_SPEED_RPM;
  config->motor_acceleration = TASK4_DEFAULT_MOTOR_ACCEL;
}

static uint32_t Task4_GetRunTimeMs(void)
{
  uint32_t run_time_ms = g_task4_run_time_ms_debug;

  if (run_time_ms < 500U)
  {
    return 500U;
  }
  if (run_time_ms >= TASK4_TIMEOUT_MS)
  {
    return TASK4_TIMEOUT_MS - 1U;
  }
  return run_time_ms;
}

void Task4_CarMotorStart(void)
{
  int16_t left_speed;
  int16_t right_speed;

  if (task4_car_motor_enabled != 0U)
  {
    return;
  }

  Tracking_Reset();
  left_speed = (int16_t)(g_task4_tracking_base_speed_debug *
                         g_task4_tracking_left_scale_debug);
  right_speed = (int16_t)(g_task4_tracking_base_speed_debug *
                          g_task4_tracking_right_scale_debug);
  task4_car_ramp_target_left_speed = left_speed;
  task4_car_ramp_target_right_speed = right_speed;
  task4_car_ramp_start_tick_ms = HAL_GetTick();
  task4_car_ramp_active = 1U;
  g_task4_car_ramp_active_debug = 1U;
  g_task4_car_ramp_elapsed_ms_debug = 0U;
  g_task4_car_ramp_left_speed_debug = 0;
  g_task4_car_ramp_right_speed_debug = 0;
  Motor_SetSpeed(0, 0);
  task4_car_motor_enabled = 1U;
  g_task4_car_running_debug = 1U;
  g_task4_car_start_count_debug++;
}

void Task4_CarMotorStop(void)
{
  uint8_t was_enabled = task4_car_motor_enabled;

  Motor_Stop();
  Tracking_Reset();
  task4_car_motor_enabled = 0U;
  task4_car_ramp_active = 0U;
  g_task4_car_ramp_active_debug = 0U;
  g_task4_car_ramp_elapsed_ms_debug = 0U;
  g_task4_car_ramp_left_speed_debug = 0;
  g_task4_car_ramp_right_speed_debug = 0;
  g_task4_car_running_debug = 0U;
  if (was_enabled != 0U)
  {
    g_task4_car_stop_count_debug++;
  }
}

static void Task4_RunCarTracking(uint32_t now_ms)
{
  uint32_t ramp_elapsed_ms;
  uint32_t ramp_time_ms;
  int32_t left_speed;
  int32_t right_speed;

  if (task4_car_motor_enabled == 0U)
  {
    return;
  }
  ramp_elapsed_ms = now_ms - task4_car_ramp_start_tick_ms;
  ramp_time_ms = g_task4_car_ramp_ms_debug;
  g_task4_car_ramp_elapsed_ms_debug = ramp_elapsed_ms;

  if (task4_car_ramp_active != 0U)
  {
    if ((ramp_time_ms != 0U) && (ramp_elapsed_ms < ramp_time_ms))
    {
      left_speed = ((int32_t)task4_car_ramp_target_left_speed *
                    (int32_t)ramp_elapsed_ms) /
                   (int32_t)ramp_time_ms;
      right_speed = ((int32_t)task4_car_ramp_target_right_speed *
                     (int32_t)ramp_elapsed_ms) /
                    (int32_t)ramp_time_ms;
      g_task4_car_ramp_left_speed_debug = (int16_t)left_speed;
      g_task4_car_ramp_right_speed_debug = (int16_t)right_speed;
    }
    else
    {
      task4_car_ramp_active = 0U;
      g_task4_car_ramp_active_debug = 0U;
      g_task4_car_ramp_left_speed_debug =
          task4_car_ramp_target_left_speed;
      g_task4_car_ramp_right_speed_debug =
          task4_car_ramp_target_right_speed;
    }
  }

  if ((now_ms - task4_last_control_tick_ms) < TASK4_CONTROL_PERIOD_MS)
  {
    return;
  }
  task4_last_control_tick_ms = now_ms;

  if (task4_car_ramp_active != 0U)
  {
    Motor_SetSpeed(g_task4_car_ramp_left_speed_debug,
                   g_task4_car_ramp_right_speed_debug);
    return;
  }
  Tracking_Process();
}

static void Task4_BeginStopCompensation(uint32_t now_ms)
{
  g_task4_locked_left_speed_debug = g_motor_left_speed_debug;
  g_task4_locked_right_speed_debug = g_motor_right_speed_debug;
  if ((g_task4_locked_left_speed_debug == 0) &&
      (g_task4_locked_right_speed_debug == 0))
  {
    g_task4_locked_left_speed_debug =
        (int16_t)(g_task4_tracking_base_speed_debug *
                  g_task4_tracking_left_scale_debug);
    g_task4_locked_right_speed_debug =
        (int16_t)(g_task4_tracking_base_speed_debug *
                  g_task4_tracking_right_scale_debug);
  }
  Motor_SetSpeed(g_task4_locked_left_speed_debug,
                 g_task4_locked_right_speed_debug);

  Task4_SetStage(TASK4_STAGE_STOP_COMPENSATE, now_ms);
  Task4_ApplyBallBeamSettings();
  task4_stop_ramp_start_pulse = g_ballbeam_motor_sent_pulse_debug;
  task4_stop_ramp_last_pulse = task4_stop_ramp_start_pulse;
  task4_stop_tilt_ramp_started = 0U;
  BallBeam_SetRun(0U);
  task4_stop_tilt_sent = 0U;
  g_task4_stop_tilt_sent_debug = 0U;
  g_task4_stop_filter_start_pulse_debug = task4_stop_ramp_start_pulse;
  g_task4_stop_filter_output_pulse_debug = task4_stop_ramp_start_pulse;
}

void Task4_CarToB_Start(uint32_t now_ms)
{
  task4_start_tick_ms = now_ms;
  task4_last_control_tick_ms = now_ms - TASK4_CONTROL_PERIOD_MS;
  task4_car_start_tick_ms = 0U;
  task4_car_ramp_start_tick_ms = 0U;
  task4_car_motor_enabled = 0U;
  task4_car_ramp_active = 0U;
  task4_car_ramp_target_left_speed = 0;
  task4_car_ramp_target_right_speed = 0;
  task4_start_tilt_sent = 0U;
  task4_start_tilt_sent_tick_ms = 0U;
  task4_start_ramp_start_pulse = 0;
  task4_start_ramp_last_pulse = 0;
  g_task4_active_debug = 1U;
  g_task4_car_running_debug = 0U;
  g_task4_elapsed_ms_debug = 0U;
  g_task4_stage_elapsed_ms_debug = 0U;
  g_task4_car_elapsed_ms_debug = 0U;
  g_task4_stop_trigger_ms_debug = 0U;
  task4_stop_tilt_ramp_started = 0U;
  task4_stop_tilt_sent = 0U;
  g_task4_stop_tilt_sent_debug = 0U;
  task4_stop_ramp_start_pulse = 0;
  task4_stop_ramp_last_pulse = 0;
  g_task4_stop_filter_start_pulse_debug = 0;
  g_task4_stop_filter_output_pulse_debug = 0;
  g_task4_car_ramp_active_debug = 0U;
  g_task4_car_ramp_elapsed_ms_debug = 0U;
  g_task4_car_ramp_left_speed_debug = 0;
  g_task4_car_ramp_right_speed_debug = 0;
  g_task4_start_filter_start_pulse_debug = 0;
  g_task4_start_filter_output_pulse_debug = 0;
  g_task4_start_tilt_sent_debug = 0U;
  g_task4_position_cm_debug = g_ballbeam_position_debug;
  g_task4_error_cm_debug =
      g_task4_target_position_cm_debug - g_task4_position_cm_debug;
  g_task4_locked_left_speed_debug = 0;
  g_task4_locked_right_speed_debug = 0;

  Task4_ApplyTrackingSettings();
  Task4_ApplyBallBeamSettings();
  Task4_CarMotorStop();
  BallBeam_SetTarget(g_task4_target_position_cm_debug);
  BallBeam_SetRun(0U);
  task4_start_ramp_start_pulse = g_ballbeam_motor_sent_pulse_debug;
  task4_start_ramp_last_pulse = task4_start_ramp_start_pulse;
  g_task4_start_filter_start_pulse_debug =
      task4_start_ramp_start_pulse;
  g_task4_start_filter_output_pulse_debug =
      task4_start_ramp_start_pulse;
  Task4_SetStage(TASK4_STAGE_START_COMPENSATE, now_ms);
}

Task_Result_t Task4_CarToB_Update(uint32_t now_ms)
{
  uint32_t elapsed_ms = now_ms - task4_start_tick_ms;
  uint32_t stage_elapsed_ms = now_ms - task4_stage_start_tick_ms;
  uint32_t run_time_ms = Task4_GetRunTimeMs();
  uint32_t stop_lead_ms = g_task4_stop_tilt_lead_ms_debug;

  if (stop_lead_ms >= run_time_ms)
  {
    stop_lead_ms = run_time_ms - 1U;
  }
  g_task4_stop_trigger_ms_debug = run_time_ms - stop_lead_ms;
  if (task4_car_start_tick_ms != 0U)
  {
    g_task4_car_elapsed_ms_debug = now_ms - task4_car_start_tick_ms;
  }

  Task4_ApplyTrackingSettings();
  Task4_ApplyBallBeamSettings();
  BallBeam_SetTarget(g_task4_target_position_cm_debug);

  g_task4_elapsed_ms_debug = elapsed_ms;
  g_task4_stage_elapsed_ms_debug = stage_elapsed_ms;
  g_task4_position_cm_debug = g_ballbeam_position_debug;
  g_task4_error_cm_debug =
      BallBeam_GetTarget() - g_task4_position_cm_debug;
  g_task4_ballbeam_state_debug = (uint8_t)BallBeam_GetState();

  if ((elapsed_ms >= TASK4_TIMEOUT_MS) &&
      (task4_stage != TASK4_STAGE_STOP_COMPENSATE))
  {
    Task4_CarMotorStop();
    BallBeam_SetRun(0U);
    Task4_SetStage(TASK4_STAGE_TIMEOUT, now_ms);
    return TASK_RESULT_TIMEOUT;
  }

  if (task4_stage == TASK4_STAGE_START_COMPENSATE)
  {
    if (task4_start_tilt_sent == 0U)
    {
      int32_t filter_pulse = Task4_FilterPulse(
          task4_start_ramp_start_pulse,
          g_task4_start_tilt_pulse_debug,
          stage_elapsed_ms,
          g_task4_start_filter_ms_debug);

      g_task4_start_filter_output_pulse_debug = filter_pulse;
      if (filter_pulse != task4_start_ramp_last_pulse)
      {
        BallBeam_CommandAbsolutePulse(filter_pulse);
        task4_start_ramp_last_pulse = filter_pulse;
      }
      if ((g_task4_start_filter_ms_debug == 0U) ||
          (stage_elapsed_ms >= g_task4_start_filter_ms_debug))
      {
        task4_start_tilt_sent = 1U;
        task4_start_tilt_sent_tick_ms = now_ms;
        g_task4_start_tilt_sent_debug = 1U;
      }
    }

    if ((task4_car_motor_enabled == 0U) &&
        (task4_start_tilt_sent != 0U) &&
        ((now_ms - task4_start_tilt_sent_tick_ms) >=
         g_task4_start_tilt_lead_ms_debug))
    {
      Task4_CarMotorStart();
      task4_car_start_tick_ms = now_ms;
    }
    Task4_RunCarTracking(now_ms);

    if ((task4_car_motor_enabled != 0U) &&
        ((now_ms - task4_car_start_tick_ms) >=
         g_task4_start_tilt_hold_ms_debug))
    {
      BallBeam_SetRun(0U);
      task4_start_ramp_start_pulse =
          g_ballbeam_motor_sent_pulse_debug;
      task4_start_ramp_last_pulse = task4_start_ramp_start_pulse;
      g_task4_start_filter_start_pulse_debug =
          task4_start_ramp_start_pulse;
      g_task4_start_filter_output_pulse_debug =
          task4_start_ramp_start_pulse;
      Task4_SetStage(TASK4_STAGE_START_RETURN_CENTER, now_ms);
    }
  }
  else if (task4_stage == TASK4_STAGE_START_RETURN_CENTER)
  {
    int32_t filter_pulse = Task4_FilterPulse(
        task4_start_ramp_start_pulse,
        0,
        stage_elapsed_ms,
        g_task4_start_return_hold_ms_debug);

    g_task4_start_filter_output_pulse_debug = filter_pulse;
    if (filter_pulse != task4_start_ramp_last_pulse)
    {
      BallBeam_CommandAbsolutePulse(filter_pulse);
      task4_start_ramp_last_pulse = filter_pulse;
    }
    Task4_RunCarTracking(now_ms);
    if (stage_elapsed_ms >= g_task4_start_return_hold_ms_debug)
    {
      Task4_SetStage(TASK4_STAGE_TRACKING, now_ms);
      if (g_task4_middle_pid_enable_debug != 0U)
      {
        Task4_ApplyBallBeamSettings();
        BallBeam_SetRun(1U);
      }
      else
      {
        BallBeam_SetRun(0U);
        BallBeam_CommandAbsolutePulse(0);
      }
    }
  }
  else if (task4_stage == TASK4_STAGE_TRACKING)
  {
    if (g_task4_middle_pid_enable_debug != 0U)
    {
      if (BallBeam_GetRun() == 0U)
      {
        Task4_ApplyBallBeamSettings();
        BallBeam_SetRun(1U);
      }
    }
    else if (BallBeam_GetRun() != 0U)
    {
      BallBeam_SetRun(0U);
      BallBeam_CommandAbsolutePulse(0);
    }
    Task4_RunCarTracking(now_ms);
    if (g_task4_car_elapsed_ms_debug >= g_task4_stop_trigger_ms_debug)
    {
      Task4_BeginStopCompensation(now_ms);
    }
  }
  else if (task4_stage == TASK4_STAGE_STOP_COMPENSATE)
  {
    stage_elapsed_ms = now_ms - task4_stage_start_tick_ms;
    g_task4_stage_elapsed_ms_debug = stage_elapsed_ms;

    if (task4_stop_tilt_ramp_started == 0U)
    {
      if (stage_elapsed_ms < g_task4_stop_command_delay_ms_debug)
      {
        return TASK_RESULT_RUNNING;
      }

      task4_stop_tilt_ramp_started = 1U;
      Task4_SetStage(TASK4_STAGE_STOP_COMPENSATE, now_ms);
      return TASK_RESULT_RUNNING;
    }

    if (task4_stop_tilt_sent == 0U)
    {
      uint32_t filter_elapsed_ms =
          now_ms - task4_stage_start_tick_ms;
      uint32_t filter_time_ms = g_task4_stop_filter_ms_debug;
      int32_t filter_pulse;

      if ((filter_time_ms == 0U) ||
          (filter_elapsed_ms >= filter_time_ms))
      {
        filter_pulse = g_task4_stop_tilt_pulse_debug;
      }
      else
      {
        filter_pulse = task4_stop_ramp_start_pulse +
            (int32_t)(((int64_t)(g_task4_stop_tilt_pulse_debug -
                                 task4_stop_ramp_start_pulse) *
                       (int64_t)filter_elapsed_ms) /
                      (int64_t)filter_time_ms);
      }

      g_task4_stop_filter_output_pulse_debug = filter_pulse;
      if (filter_pulse != task4_stop_ramp_last_pulse)
      {
        BallBeam_CommandAbsolutePulse(filter_pulse);
        task4_stop_ramp_last_pulse = filter_pulse;
      }

      if ((filter_time_ms != 0U) &&
          (filter_elapsed_ms < filter_time_ms))
      {
        return TASK_RESULT_RUNNING;
      }

      if (task4_stop_ramp_last_pulse != g_task4_stop_tilt_pulse_debug)
      {
        BallBeam_CommandAbsolutePulse(g_task4_stop_tilt_pulse_debug);
        task4_stop_ramp_last_pulse = g_task4_stop_tilt_pulse_debug;
      }
      task4_stop_tilt_sent = 1U;
      g_task4_stop_tilt_sent_debug = 1U;
      Task4_SetStage(TASK4_STAGE_STOP_COMPENSATE, now_ms);
      return TASK_RESULT_RUNNING;
    }

    stage_elapsed_ms = now_ms - task4_stage_start_tick_ms;
    g_task4_stage_elapsed_ms_debug = stage_elapsed_ms;
    if ((stage_elapsed_ms >= g_task4_stop_tilt_lead_ms_debug) &&
        (task4_car_motor_enabled != 0U))
    {
      Task4_CarMotorStop();
    }

    if (stage_elapsed_ms >=
        (g_task4_stop_tilt_lead_ms_debug +
         g_task4_stop_tilt_hold_ms_debug))
    {
      BallBeam_CommandAbsolutePulse(0);
      Task4_SetStage(TASK4_STAGE_STOP_RETURN_CENTER, now_ms);
    }
  }
  else if (task4_stage == TASK4_STAGE_STOP_RETURN_CENTER)
  {
    if (stage_elapsed_ms >= g_task4_stop_return_hold_ms_debug)
    {
      Task4_SetStage(TASK4_STAGE_DONE, now_ms);
      return TASK_RESULT_DONE;
    }
  }

  return TASK_RESULT_RUNNING;
}

void Task4_CarToB_Stop(void)
{
  Task4_CarMotorStop();
  BallBeam_SetRun(0U);
  g_task4_active_debug = 0U;
  g_task4_ballbeam_state_debug = (uint8_t)BallBeam_GetState();
}

Task4_Stage_t Task4_CarToB_GetStage(void)
{
  return task4_stage;
}
