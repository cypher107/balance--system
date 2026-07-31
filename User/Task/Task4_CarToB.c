#include "Task4_CarToB.h"

#include "APP_SENSOR.h"
#include "BallBeam.h"
#include "motor.h"
#include "tracking.h"

#define TASK4_CONTROL_PERIOD_MS        10U
#define TASK4_TIMEOUT_MS               20000U
#define TASK4_FINISH_FIRST_SENSOR      4U
#define TASK4_FINISH_SENSOR_NUM        4U
#define TASK4_DEFAULT_TARGET_CM        12.5f
#define TASK4_DEFAULT_MOTOR_SPEED_RPM  300U
#define TASK4_DEFAULT_MOTOR_ACCEL      255U

static Task4_Stage_t task4_stage;
static uint32_t task4_start_tick_ms;
static uint32_t task4_stage_start_tick_ms;
static uint32_t task4_car_start_tick_ms;
static uint32_t task4_last_control_tick_ms;
static uint8_t task4_car_motor_enabled;

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
volatile float g_task4_min_correction_pulse_debug = 40.0f;
volatile float g_task4_min_full_error_cm_debug = 2.0f;
volatile int32_t g_task4_max_motor_pulse_debug = 100;

volatile float g_task4_tracking_kp_debug = 18.0f;
volatile float g_task4_tracking_ki_debug = 0.0f;
volatile float g_task4_tracking_kd_debug = 0.0f;
volatile float g_task4_tracking_base_speed_debug = 200.0f;
volatile float g_task4_tracking_left_scale_debug = 1.3f;
volatile float g_task4_tracking_right_scale_debug = 1.0f;

volatile int32_t g_task4_start_tilt_pulse_debug = 40;
volatile int32_t g_task4_stop_tilt_pulse_debug = -40;
volatile uint32_t g_task4_start_tilt_lead_ms_debug = 150U;
volatile uint32_t g_task4_start_tilt_hold_ms_debug = 400U;
volatile uint32_t g_task4_stop_tilt_lead_ms_debug = 150U;
volatile uint32_t g_task4_stop_tilt_hold_ms_debug = 400U;
volatile float g_task4_min_run_time_s_debug = 3.0f;
volatile uint8_t g_task4_finish_sensor_count_debug = 3U;
volatile uint8_t g_task4_finish_middle_count_debug;
volatile int16_t g_task4_locked_left_speed_debug;
volatile int16_t g_task4_locked_right_speed_debug;
volatile uint32_t g_task4_car_start_count_debug;
volatile uint32_t g_task4_car_stop_count_debug;

static void Task4_SetStage(Task4_Stage_t stage, uint32_t now_ms)
{
  task4_stage = stage;
  task4_stage_start_tick_ms = now_ms;
  g_task4_stage_debug = (uint8_t)stage;
  g_task4_stage_elapsed_ms_debug = 0U;
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
  if ((g_task4_position_hold_cm_debug >= 0.0f) &&
      (g_task4_position_hold_cm_debug <= 2.0f))
  {
    g_ballbeam_position_hold_cm_debug =
        g_task4_position_hold_cm_debug;
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
  if ((g_task4_min_correction_pulse_debug >= 0.0f) &&
      (g_task4_min_correction_pulse_debug <= 100.0f))
  {
    g_ballbeam_min_correction_pulse_debug =
        g_task4_min_correction_pulse_debug;
  }
  if ((g_task4_min_full_error_cm_debug > 0.5f) &&
      (g_task4_min_full_error_cm_debug <= 10.0f))
  {
    g_ballbeam_min_correction_full_error_cm_debug =
        g_task4_min_full_error_cm_debug;
  }

  config->motor_speed_rpm = TASK4_DEFAULT_MOTOR_SPEED_RPM;
  config->motor_acceleration = TASK4_DEFAULT_MOTOR_ACCEL;
}

static uint32_t Task4_GetMinRunTimeMs(void)
{
  float run_time_s = g_task4_min_run_time_s_debug;

  if (run_time_s <= 0.0f)
  {
    return 0U;
  }
  if (run_time_s >= ((float)TASK4_TIMEOUT_MS * 0.001f))
  {
    return TASK4_TIMEOUT_MS;
  }
  return (uint32_t)(run_time_s * 1000.0f);
}

static uint8_t Task4_OnFinishLine(void)
{
  uint8_t black_count = 0U;
  uint8_t required_count = g_task4_finish_sensor_count_debug;
  uint8_t index;

  if ((required_count == 0U) ||
      (required_count > TASK4_FINISH_SENSOR_NUM))
  {
    required_count = 3U;
  }

  for (index = TASK4_FINISH_FIRST_SENSOR;
       index < (TASK4_FINISH_FIRST_SENSOR + TASK4_FINISH_SENSOR_NUM);
       index++)
  {
    if (g_grayscale_values_debug[index] == 1U)
    {
      black_count++;
    }
  }
  g_task4_finish_middle_count_debug = black_count;

  return ((g_tracking_sensor_valid_debug != 0U) &&
          (black_count >= required_count)) ? 1U : 0U;
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
  Motor_SetSpeed(left_speed, right_speed);
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
  g_task4_car_running_debug = 0U;
  if (was_enabled != 0U)
  {
    g_task4_car_stop_count_debug++;
  }
}

static void Task4_RunCarTracking(uint32_t now_ms)
{
  if (task4_car_motor_enabled == 0U)
  {
    return;
  }
  if ((now_ms - task4_last_control_tick_ms) < TASK4_CONTROL_PERIOD_MS)
  {
    return;
  }
  task4_last_control_tick_ms = now_ms;
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

  BallBeam_SetRun(0U);
  BallBeam_CommandAbsolutePulse(g_task4_stop_tilt_pulse_debug);
  Task4_SetStage(TASK4_STAGE_STOP_COMPENSATE, now_ms);
}

void Task4_CarToB_Start(uint32_t now_ms)
{
  task4_start_tick_ms = now_ms;
  task4_last_control_tick_ms = now_ms - TASK4_CONTROL_PERIOD_MS;
  task4_car_start_tick_ms = 0U;
  task4_car_motor_enabled = 0U;
  g_task4_active_debug = 1U;
  g_task4_car_running_debug = 0U;
  g_task4_elapsed_ms_debug = 0U;
  g_task4_stage_elapsed_ms_debug = 0U;
  g_task4_position_cm_debug = g_ballbeam_position_debug;
  g_task4_error_cm_debug =
      g_task4_target_position_cm_debug - g_task4_position_cm_debug;
  g_task4_finish_middle_count_debug = 0U;
  g_task4_locked_left_speed_debug = 0;
  g_task4_locked_right_speed_debug = 0;

  Task4_ApplyTrackingSettings();
  Task4_ApplyBallBeamSettings();
  Task4_CarMotorStop();
  BallBeam_SetTarget(g_task4_target_position_cm_debug);
  BallBeam_SetRun(0U);
  BallBeam_CommandAbsolutePulse(g_task4_start_tilt_pulse_debug);
  Task4_SetStage(TASK4_STAGE_START_COMPENSATE, now_ms);
}

Task_Result_t Task4_CarToB_Update(uint32_t now_ms)
{
  uint32_t elapsed_ms = now_ms - task4_start_tick_ms;
  uint32_t stage_elapsed_ms = now_ms - task4_stage_start_tick_ms;

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
    if ((task4_car_motor_enabled == 0U) &&
        (stage_elapsed_ms >= g_task4_start_tilt_lead_ms_debug))
    {
      Task4_CarMotorStart();
      task4_car_start_tick_ms = now_ms;
    }
    Task4_RunCarTracking(now_ms);

    if ((task4_car_motor_enabled != 0U) &&
        ((now_ms - task4_car_start_tick_ms) >=
         g_task4_start_tilt_hold_ms_debug))
    {
      BallBeam_SetRun(1U);
      Task4_SetStage(TASK4_STAGE_TRACKING, now_ms);
    }
  }
  else if (task4_stage == TASK4_STAGE_TRACKING)
  {
    Task4_RunCarTracking(now_ms);
    if ((elapsed_ms >= Task4_GetMinRunTimeMs()) &&
        (Task4_OnFinishLine() != 0U))
    {
      Task4_BeginStopCompensation(now_ms);
    }
  }
  else if (task4_stage == TASK4_STAGE_STOP_COMPENSATE)
  {
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
