#include "Task4_CarToB.h"

#include "BallBeam.h"
#include "Task3_BallMove.h"
#include "motor.h"
#include "tracking.h"

#define TASK4_CONTROL_PERIOD_MS          10U
#define TASK4_RUN_TIME_MS                8000U

#define TASK4_START_COMPENSATE_PULSE     (-220)
#define TASK4_START_RISE_TIME_MS         200U
#define TASK4_START_HOLD_TIME_MS         150U
#define TASK4_START_FALL_TIME_MS         200U
#define TASK4_START_COMMAND_PERIOD_MS    10U
#define TASK4_STOP_COMPENSATE_PULSE      220
#define TASK4_STOP_RISE_TIME_MS          200U
#define TASK4_STOP_HOLD_TIME_MS          50U
#define TASK4_STOP_FALL_TIME_MS          150U

#define TASK4_ZDT_SPEED_RPM              150U
#define TASK4_ZDT_ACCELERATION           200U
#define TASK4_ZDT_PULSE_LIMIT            250

#define TASK4_MIDDLE_PID_TARGET_CM       12.5f
#define TASK4_MIDDLE_PID_DEADBAND_CM     0.8f
#define TASK4_MIDDLE_PID_PULSE_LIMIT     250

#define TASK4_TRACK_KP                   18.0f
#define TASK4_TRACK_KI                   0.0f
#define TASK4_TRACK_KD                   0.0f
#define TASK4_TRACK_BASE_SPEED           145.0f
#define TASK4_TRACK_LEFT_SCALE           1.3f
#define TASK4_TRACK_RIGHT_SCALE          1.0f

static Task4_Stage_t task4_stage;
static uint32_t task4_stage_start_tick_ms;
static uint32_t task4_car_start_tick_ms;
static uint32_t task4_last_control_tick_ms;
static uint32_t task4_last_zdt_command_tick_ms;
static int32_t task4_last_zdt_command_pulse;
static int32_t task4_stop_start_pulse;
static uint8_t task4_car_motor_enabled;

static void Task4_SetStage(Task4_Stage_t stage, uint32_t now_ms)
{
  task4_stage = stage;
  task4_stage_start_tick_ms = now_ms;
}

static void Task4_ApplyTrackingSettings(void)
{
  g_tracking_kp_debug = TASK4_TRACK_KP;
  g_tracking_ki_debug = TASK4_TRACK_KI;
  g_tracking_kd_debug = TASK4_TRACK_KD;
  g_tracking_base_speed_debug = TASK4_TRACK_BASE_SPEED;
  g_tracking_left_speed_scale_debug = TASK4_TRACK_LEFT_SCALE;
  g_tracking_right_speed_scale_debug = TASK4_TRACK_RIGHT_SCALE;
}

static void Task4_ApplyZdtSettings(void)
{
  BallBeam_Config_t *config = BallBeam_GetConfig();

  config->motor_speed_rpm = TASK4_ZDT_SPEED_RPM;
  config->motor_acceleration = TASK4_ZDT_ACCELERATION;
  config->max_motor_pulse = TASK4_ZDT_PULSE_LIMIT;
  config->motor_center_pulse = 0;
}

static void Task4_StartMiddlePid(void)
{
  BallBeam_Config_t *config = BallBeam_GetConfig();
  PID_t *position_pid = BallBeam_GetPositionPID();

  config->motor_speed_rpm = g_task3_motor_speed_debug;
  config->motor_acceleration = g_task3_motor_acceleration_debug;
  config->max_motor_pulse = TASK4_MIDDLE_PID_PULSE_LIMIT;
  config->motor_center_pulse = 0;

  g_ballbeam_kp_debug = g_task3_kp_debug;
  g_ballbeam_ki_debug = 0.0f;
  g_ballbeam_kd_debug = g_task3_kd_debug;
  g_ballbeam_position_hold_cm_debug = TASK4_MIDDLE_PID_DEADBAND_CM;
  g_ballbeam_output_lpf_cutoff_hz_debug = 0.0f;
  g_ballbeam_output_slew_rate_debug = 0.0f;
  g_ballbeam_min_correction_pulse_debug = 0.0f;

  PID_SetTunings(position_pid,
                 g_task3_kp_debug,
                 0.0f,
                 g_task3_kd_debug);
  PID_SetOutputLimit(position_pid,
                     (float)-TASK4_MIDDLE_PID_PULSE_LIMIT,
                     (float)TASK4_MIDDLE_PID_PULSE_LIMIT);
  PID_SetIntegralLimit(position_pid, 0.0f, 0.0f);
  PID_SetIntegralSeparation(position_pid, 0U, 0.0f);
  PID_SetDeadband(position_pid, 0.0f);
  PID_SetDerivativeFilter(position_pid, 1.0f);

  BallBeam_SetTarget(TASK4_MIDDLE_PID_TARGET_CM);
  BallBeam_SetRun(1U);
}

static int32_t Task4_GetStartCompensationPulse(uint32_t elapsed_ms)
{
  uint32_t fall_elapsed_ms;

  if (elapsed_ms < TASK4_START_RISE_TIME_MS)
  {
    return (TASK4_START_COMPENSATE_PULSE * (int32_t)elapsed_ms) /
           (int32_t)TASK4_START_RISE_TIME_MS;
  }

  elapsed_ms -= TASK4_START_RISE_TIME_MS;
  if (elapsed_ms < TASK4_START_HOLD_TIME_MS)
  {
    return TASK4_START_COMPENSATE_PULSE;
  }

  fall_elapsed_ms = elapsed_ms - TASK4_START_HOLD_TIME_MS;
  return TASK4_START_COMPENSATE_PULSE -
         ((TASK4_START_COMPENSATE_PULSE *
           (int32_t)fall_elapsed_ms) /
          (int32_t)TASK4_START_FALL_TIME_MS);
}

static void Task4_UpdateStartCompensation(uint32_t elapsed_ms,
                                          uint32_t now_ms)
{
  int32_t pulse = Task4_GetStartCompensationPulse(elapsed_ms);

  if ((pulse == task4_last_zdt_command_pulse) ||
      ((now_ms - task4_last_zdt_command_tick_ms) <
       TASK4_START_COMMAND_PERIOD_MS))
  {
    return;
  }

  BallBeam_CommandAbsolutePulse(pulse);
  task4_last_zdt_command_pulse = pulse;
  task4_last_zdt_command_tick_ms = now_ms;
}

static int32_t Task4_GetStopCompensationPulse(uint32_t elapsed_ms)
{
  uint32_t fall_elapsed_ms;

  if (elapsed_ms < TASK4_STOP_RISE_TIME_MS)
  {
    return task4_stop_start_pulse +
           (((TASK4_STOP_COMPENSATE_PULSE - task4_stop_start_pulse) *
             (int32_t)elapsed_ms) /
            (int32_t)TASK4_STOP_RISE_TIME_MS);
  }

  elapsed_ms -= TASK4_STOP_RISE_TIME_MS;
  if (elapsed_ms < TASK4_STOP_HOLD_TIME_MS)
  {
    return TASK4_STOP_COMPENSATE_PULSE;
  }

  fall_elapsed_ms = elapsed_ms - TASK4_STOP_HOLD_TIME_MS;
  return TASK4_STOP_COMPENSATE_PULSE -
         ((TASK4_STOP_COMPENSATE_PULSE *
           (int32_t)fall_elapsed_ms) /
          (int32_t)TASK4_STOP_FALL_TIME_MS);
}

static void Task4_UpdateStopCompensation(uint32_t elapsed_ms,
                                         uint32_t now_ms)
{
  int32_t pulse = Task4_GetStopCompensationPulse(elapsed_ms);

  if ((pulse == task4_last_zdt_command_pulse) ||
      ((now_ms - task4_last_zdt_command_tick_ms) <
       TASK4_START_COMMAND_PERIOD_MS))
  {
    return;
  }

  BallBeam_CommandAbsolutePulse(pulse);
  task4_last_zdt_command_pulse = pulse;
  task4_last_zdt_command_tick_ms = now_ms;
}

static void Task4_RunCarStartRamp(uint32_t elapsed_ms, uint32_t now_ms)
{
  int32_t left_target;
  int32_t right_target;
  int16_t left_speed;
  int16_t right_speed;

  if ((now_ms - task4_last_control_tick_ms) <
      TASK4_CONTROL_PERIOD_MS)
  {
    return;
  }

  task4_last_control_tick_ms = now_ms;
  left_target = (int32_t)(TASK4_TRACK_BASE_SPEED *
                          TASK4_TRACK_LEFT_SCALE);
  right_target = (int32_t)(TASK4_TRACK_BASE_SPEED *
                           TASK4_TRACK_RIGHT_SCALE);
  left_speed = (int16_t)((left_target * (int32_t)elapsed_ms) /
                         (int32_t)TASK4_START_RISE_TIME_MS);
  right_speed = (int16_t)((right_target * (int32_t)elapsed_ms) /
                          (int32_t)TASK4_START_RISE_TIME_MS);
  Motor_SetSpeed(left_speed, right_speed);
}

void Task4_CarMotorStart(void)
{
  if (task4_car_motor_enabled != 0U)
  {
    return;
  }

  Task4_ApplyTrackingSettings();
  Tracking_Reset();
  Motor_SetSpeed(0, 0);
  task4_car_motor_enabled = 1U;
}

void Task4_CarMotorStop(void)
{
  Motor_Stop();
  Tracking_Reset();
  task4_car_motor_enabled = 0U;
}

static void Task4_RunCarTracking(uint32_t now_ms)
{
  if ((task4_car_motor_enabled == 0U) ||
      ((now_ms - task4_last_control_tick_ms) <
       TASK4_CONTROL_PERIOD_MS))
  {
    return;
  }

  task4_last_control_tick_ms = now_ms;
  Tracking_Process();
}

void Task4_CarToB_Start(uint32_t now_ms)
{
  task4_car_motor_enabled = 0U;
  task4_car_start_tick_ms = now_ms;
  task4_last_control_tick_ms = now_ms - TASK4_CONTROL_PERIOD_MS;
  task4_last_zdt_command_tick_ms = now_ms;
  task4_last_zdt_command_pulse = 0;

  Task4_ApplyTrackingSettings();
  Task4_ApplyZdtSettings();
  BallBeam_SetRun(0U);

  BallBeam_CommandAbsolutePulse(0);
  Task4_CarMotorStart();
  Task4_SetStage(TASK4_STAGE_START_COMPENSATE, now_ms);
}

Task_Result_t Task4_CarToB_Update(uint32_t now_ms)
{
  uint32_t stage_elapsed_ms = now_ms - task4_stage_start_tick_ms;

  if (task4_stage == TASK4_STAGE_START_COMPENSATE)
  {
    if (stage_elapsed_ms < TASK4_START_RISE_TIME_MS)
    {
      Task4_RunCarStartRamp(stage_elapsed_ms, now_ms);
    }
    else
    {
      Task4_RunCarTracking(now_ms);
    }
    if (stage_elapsed_ms <
        (TASK4_START_RISE_TIME_MS +
         TASK4_START_HOLD_TIME_MS +
         TASK4_START_FALL_TIME_MS))
    {
      Task4_UpdateStartCompensation(stage_elapsed_ms, now_ms);
    }
    else
    {
      BallBeam_CommandAbsolutePulse(0);
      Task4_StartMiddlePid();
      Task4_SetStage(TASK4_STAGE_TRACKING, now_ms);
    }
  }
  else if (task4_stage == TASK4_STAGE_TRACKING)
  {
    Task4_RunCarTracking(now_ms);
    if ((now_ms - task4_car_start_tick_ms) >= TASK4_RUN_TIME_MS)
    {
      task4_stop_start_pulse = g_ballbeam_motor_sent_pulse_debug;
      BallBeam_SetRun(0U);
      Task4_ApplyZdtSettings();
      task4_last_zdt_command_pulse = task4_stop_start_pulse;
      task4_last_zdt_command_tick_ms = now_ms;
      Task4_CarMotorStop();
      Task4_SetStage(TASK4_STAGE_STOP_COMPENSATE, now_ms);
    }
  }
  else if (task4_stage == TASK4_STAGE_STOP_COMPENSATE)
  {
    if (stage_elapsed_ms <
        (TASK4_STOP_RISE_TIME_MS +
         TASK4_STOP_HOLD_TIME_MS +
         TASK4_STOP_FALL_TIME_MS))
    {
      Task4_UpdateStopCompensation(stage_elapsed_ms, now_ms);
    }
    else
    {
      BallBeam_CommandAbsolutePulse(0);
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
  if (task4_stage != TASK4_STAGE_DONE)
  {
    BallBeam_CommandAbsolutePulse(0);
  }
  Task4_SetStage(TASK4_STAGE_IDLE, 0U);
}

Task4_Stage_t Task4_CarToB_GetStage(void)
{
  return task4_stage;
}
