#include "BallBeam.h"

#include <limits.h>

#include "dwt_time.h"
#include "Emm_V5.h"
#include "stm32f1xx_hal.h"
#include "Vision.h"

#define BALLBEAM_DEFAULT_DT_S       0.02f
#define BALLBEAM_MIN_DT_S           0.001f
#define BALLBEAM_MAX_DT_S           0.20f
#define BALLBEAM_HARD_PULSE_LIMIT   500
#define BALLBEAM_POSITION_HOLD_CM   1.0f
#define BALLBEAM_HOME_MODE          0U

typedef struct
{
  PID_t position_pid;
  BallBeam_Config_t config;
  BallBeam_State_t state;
  float target_position_cm;
  float position_cm;
  uint32_t vision_sequence;
  uint32_t last_position_cycles;
  uint32_t last_position_tick_ms;
  uint8_t have_position;
  uint8_t run_requested;
  uint8_t motor_stopped;
  int32_t last_motor_pulse;
} BallBeam_Controller_t;

static BallBeam_Controller_t ballbeam;

volatile uint8_t g_ballbeam_state_debug;
volatile uint8_t g_ballbeam_run_debug;
volatile int8_t g_ballbeam_motor_direction_debug;
volatile float g_ballbeam_kp_debug;
volatile float g_ballbeam_kd_debug;
volatile float g_ballbeam_target_position_cm_debug;
volatile float g_ballbeam_position_debug;
volatile float g_ballbeam_error_debug;
volatile float g_ballbeam_position_dt_debug;
volatile float g_ballbeam_p_output_debug;
volatile float g_ballbeam_d_output_debug;
volatile float g_ballbeam_pid_output_debug;
volatile uint8_t g_ballbeam_hold_debug;
volatile int32_t g_ballbeam_motor_pulse_debug;
volatile float g_ballbeam_motor_angle_debug;
volatile uint32_t g_ballbeam_control_count_debug;
volatile uint32_t g_ballbeam_position_count_debug;
volatile uint32_t g_ballbeam_stale_count_debug;
volatile uint32_t g_ballbeam_invalid_position_count_debug;
volatile uint32_t g_ballbeam_hold_count_debug;
volatile uint32_t g_ballbeam_home_command_count_debug;

static float BallBeam_ClampFloat(float value, float minimum, float maximum)
{
  if (value > maximum) return maximum;
  if (value < minimum) return minimum;
  return value;
}

static float BallBeam_AbsFloat(float value)
{
  return (value >= 0.0f) ? value : -value;
}

static int32_t BallBeam_ClampPulse(int32_t pulse)
{
  int32_t limit = ballbeam.config.max_motor_pulse;

  if ((limit <= 0) || (limit > BALLBEAM_HARD_PULSE_LIMIT))
  {
    limit = BALLBEAM_HARD_PULSE_LIMIT;
  }
  if (pulse > limit) return limit;
  if (pulse < -limit) return -limit;
  return pulse;
}

static int32_t BallBeam_RoundToInt(float value)
{
  return (value >= 0.0f) ? (int32_t)(value + 0.5f) :
                           (int32_t)(value - 0.5f);
}

static void BallBeam_SetState(BallBeam_State_t state)
{
  ballbeam.state = state;
  g_ballbeam_state_debug = (uint8_t)state;
}

static void BallBeam_StopMotor(void)
{
  if ((ballbeam.config.motor_output_enabled != 0U) &&
      (ballbeam.motor_stopped == 0U))
  {
    Emm_V5_Stop_Now(ballbeam.config.motor_address, false);
  }

  ballbeam.motor_stopped = 1U;
  ballbeam.last_motor_pulse = INT32_MAX;
}

static void BallBeam_ResetControl(void)
{
  PID_Reset(&ballbeam.position_pid);
  ballbeam.last_position_cycles = 0U;
  g_ballbeam_error_debug = 0.0f;
  g_ballbeam_position_dt_debug = 0.0f;
  g_ballbeam_p_output_debug = 0.0f;
  g_ballbeam_d_output_debug = 0.0f;
  g_ballbeam_pid_output_debug = 0.0f;
  g_ballbeam_hold_debug = 0U;
}

static void BallBeam_PrimePositionPid(float position_cm, float error)
{
  ballbeam.position_pid.setpoint = ballbeam.target_position_cm;
  ballbeam.position_pid.feedback = position_cm;
  ballbeam.position_pid.error = error;
  ballbeam.position_pid.last_error = error;
  ballbeam.position_pid.prev_error = error;
  ballbeam.position_pid.proportional = ballbeam.position_pid.kp * error;
  ballbeam.position_pid.integral = 0.0f;
  ballbeam.position_pid.derivative = 0.0f;
  ballbeam.position_pid.last_derivative = 0.0f;
}

static void BallBeam_SendSignedPulse(int32_t signed_pulse)
{
  uint8_t direction;
  uint32_t pulse;

  signed_pulse = BallBeam_ClampPulse(signed_pulse);

  g_ballbeam_motor_pulse_debug = signed_pulse;
  if (ballbeam.config.motor_pulses_per_revolution != 0U)
  {
    g_ballbeam_motor_angle_debug =
        ((float)signed_pulse * 360.0f) /
        (float)ballbeam.config.motor_pulses_per_revolution;
  }
  else
  {
    g_ballbeam_motor_angle_debug = 0.0f;
  }

  if ((ballbeam.config.motor_output_enabled == 0U) ||
      (signed_pulse == ballbeam.last_motor_pulse))
  {
    return;
  }

  if (signed_pulse >= 0)
  {
    direction = ballbeam.config.motor_positive_direction;
    pulse = (uint32_t)signed_pulse;
  }
  else
  {
    direction = (ballbeam.config.motor_positive_direction == 0U) ? 1U : 0U;
    pulse = (uint32_t)(-signed_pulse);
  }

  Emm_V5_Pos_Control(ballbeam.config.motor_address,
                     direction,
                     ballbeam.config.motor_speed_rpm,
                     ballbeam.config.motor_acceleration,
                     pulse,
                     true,
                     false);
  ballbeam.last_motor_pulse = signed_pulse;
  ballbeam.motor_stopped = 0U;
}

static void BallBeam_SendAbsolutePulse(float pid_output)
{
  float signed_target;
  int32_t signed_pulse;

  signed_target = (float)ballbeam.config.motor_center_pulse +
                  (pid_output * ballbeam.config.motor_direction);
  signed_pulse = BallBeam_RoundToInt(signed_target);
  BallBeam_SendSignedPulse(signed_pulse);
}

static void BallBeam_ProcessPosition(const Vision_BallSample_t *sample,
                                     uint32_t now_ms)
{
  float dt_s = BALLBEAM_DEFAULT_DT_S;
  float error;
  float pid_output;

  if ((sample->position_cm < ballbeam.config.position_min_cm) ||
      (sample->position_cm > ballbeam.config.position_max_cm))
  {
    g_ballbeam_invalid_position_count_debug++;
    ballbeam.have_position = 0U;
    PID_Reset(&ballbeam.position_pid);
    ballbeam.last_position_cycles = 0U;
    g_ballbeam_hold_debug = 1U;
    BallBeam_SetState(BALLBEAM_STATE_LOST);
    return;
  }

  if (ballbeam.last_position_cycles != 0U)
  {
    dt_s = DWT_Time_DeltaSeconds(sample->receive_cycles,
                                 ballbeam.last_position_cycles);
  }
  ballbeam.last_position_cycles = sample->receive_cycles;

  if ((dt_s < BALLBEAM_MIN_DT_S) || (dt_s > BALLBEAM_MAX_DT_S))
  {
    dt_s = BALLBEAM_DEFAULT_DT_S;
    PID_Reset(&ballbeam.position_pid);
  }

  ballbeam.position_cm = sample->position_cm;
  ballbeam.last_position_tick_ms = now_ms;
  ballbeam.have_position = 1U;
  g_ballbeam_position_debug = sample->position_cm;
  g_ballbeam_position_dt_debug = dt_s;
  g_ballbeam_position_count_debug++;

  if (ballbeam.run_requested == 0U)
  {
    return;
  }

  if ((g_ballbeam_kp_debug >= 0.0f) &&
      (g_ballbeam_kp_debug <= 1000.0f) &&
      (g_ballbeam_kd_debug >= 0.0f) &&
      (g_ballbeam_kd_debug <= 1000.0f))
  {
    PID_SetTunings(&ballbeam.position_pid,
                   g_ballbeam_kp_debug,
                   0.0f,
                   g_ballbeam_kd_debug);
  }
  ballbeam.config.motor_direction =
      (g_ballbeam_motor_direction_debug >= 0) ? 1.0f : -1.0f;
  error = ballbeam.target_position_cm - sample->position_cm;

  if (BallBeam_AbsFloat(error) < BALLBEAM_POSITION_HOLD_CM)
  {
    BallBeam_PrimePositionPid(sample->position_cm, error);
    g_ballbeam_error_debug = error;
    g_ballbeam_p_output_debug = ballbeam.position_pid.proportional;
    g_ballbeam_d_output_debug = 0.0f;
    g_ballbeam_hold_debug = 1U;
    g_ballbeam_hold_count_debug++;
    BallBeam_SetState(BALLBEAM_STATE_RUN);
    return;
  }

  if (ballbeam.state != BALLBEAM_STATE_RUN)
  {
    PID_Reset(&ballbeam.position_pid);
    dt_s = BALLBEAM_DEFAULT_DT_S;
    ballbeam.last_motor_pulse = INT32_MAX;
    BallBeam_SetState(BALLBEAM_STATE_RUN);
  }

  pid_output = PID_CalculateDt(&ballbeam.position_pid,
                               ballbeam.target_position_cm,
                               sample->position_cm,
                               dt_s) * ballbeam.config.position_direction;
  pid_output = BallBeam_ClampFloat(pid_output,
                                  (float)-ballbeam.config.max_motor_pulse,
                                  (float)ballbeam.config.max_motor_pulse);
  g_ballbeam_error_debug = ballbeam.position_pid.error;
  g_ballbeam_p_output_debug = ballbeam.position_pid.proportional;
  g_ballbeam_d_output_debug = ballbeam.position_pid.derivative;
  g_ballbeam_pid_output_debug = pid_output;
  g_ballbeam_hold_debug = 0U;
  g_ballbeam_control_count_debug++;
  BallBeam_SendAbsolutePulse(pid_output);
}

void BallBeam_Init(void)
{
  ballbeam.config.position_min_cm = 0.0f;
  ballbeam.config.position_max_cm = 25.0f;
  ballbeam.config.position_direction = 1.0f;
  ballbeam.config.motor_direction = 1.0f;
  ballbeam.config.position_timeout_ms = 150U;
  ballbeam.config.motor_address = 1U;
  ballbeam.config.motor_positive_direction = 0U;
  ballbeam.config.motor_speed_rpm = 300U;
  ballbeam.config.motor_acceleration = 255U;
  ballbeam.config.motor_pulses_per_revolution = 3200U;
  ballbeam.config.motor_center_pulse = 0;
  ballbeam.config.max_motor_pulse = BALLBEAM_HARD_PULSE_LIMIT;
  ballbeam.config.motor_output_enabled = 1U;

  PID_Init(&ballbeam.position_pid, PID_MODE_POSITION, 4.0f, 0.0f, 3.0f);
  PID_SetOutputLimit(&ballbeam.position_pid,
                     (float)-BALLBEAM_HARD_PULSE_LIMIT,
                     (float)BALLBEAM_HARD_PULSE_LIMIT);
  PID_SetIntegralLimit(&ballbeam.position_pid, 0.0f, 0.0f);
  PID_SetDeadband(&ballbeam.position_pid, 0.03f);
  PID_SetDerivativeFilter(&ballbeam.position_pid, 0.20f);
  PID_SetIntegralSeparation(&ballbeam.position_pid, 0U, 0.0f);

  ballbeam.target_position_cm = 12.5f;
  ballbeam.position_cm = 0.0f;
  ballbeam.vision_sequence = 0U;
  ballbeam.last_position_cycles = 0U;
  ballbeam.last_position_tick_ms = HAL_GetTick();
  ballbeam.have_position = 0U;
  ballbeam.run_requested = 0U;
  ballbeam.motor_stopped = 1U;
  ballbeam.last_motor_pulse = INT32_MAX;

  g_ballbeam_target_position_cm_debug = ballbeam.target_position_cm;
  g_ballbeam_run_debug = 0U;
  g_ballbeam_motor_direction_debug = 1;
  g_ballbeam_kp_debug = ballbeam.position_pid.kp;
  g_ballbeam_kd_debug = ballbeam.position_pid.kd;
  g_ballbeam_position_debug = 0.0f;
  g_ballbeam_error_debug = 0.0f;
  g_ballbeam_position_dt_debug = 0.0f;
  g_ballbeam_p_output_debug = 0.0f;
  g_ballbeam_d_output_debug = 0.0f;
  g_ballbeam_pid_output_debug = 0.0f;
  g_ballbeam_hold_debug = 0U;
  g_ballbeam_motor_pulse_debug = 0;
  g_ballbeam_motor_angle_debug = 0.0f;
  g_ballbeam_control_count_debug = 0U;
  g_ballbeam_position_count_debug = 0U;
  g_ballbeam_stale_count_debug = 0U;
  g_ballbeam_invalid_position_count_debug = 0U;
  g_ballbeam_hold_count_debug = 0U;
  g_ballbeam_home_command_count_debug = 0U;
  BallBeam_SetState(BALLBEAM_STATE_IDLE);
}

void BallBeam_Task(void)
{
  Vision_BallSample_t sample;
  uint32_t now_ms = HAL_GetTick();

  if (((g_ballbeam_run_debug != 0U) ? 1U : 0U) !=
      ballbeam.run_requested)
  {
    BallBeam_SetRun(g_ballbeam_run_debug);
  }

  if (Vision_GetLatest(&sample, &ballbeam.vision_sequence) != 0U)
  {
    BallBeam_ProcessPosition(&sample, now_ms);
  }

  if (ballbeam.run_requested == 0U)
  {
    if (ballbeam.state != BALLBEAM_STATE_IDLE)
    {
      BallBeam_StopMotor();
      BallBeam_ResetControl();
      BallBeam_SetState(BALLBEAM_STATE_IDLE);
    }
    return;
  }

  if (ballbeam.have_position == 0U)
  {
    if (ballbeam.state != BALLBEAM_STATE_LOST)
    {
      BallBeam_SetState(BALLBEAM_STATE_ARMING);
    }
    return;
  }

  if ((now_ms - ballbeam.last_position_tick_ms) >
      ballbeam.config.position_timeout_ms)
  {
    if (ballbeam.state == BALLBEAM_STATE_RUN)
    {
      g_ballbeam_stale_count_debug++;
      PID_Reset(&ballbeam.position_pid);
    }
    ballbeam.have_position = 0U;
    ballbeam.last_position_cycles = 0U;
    g_ballbeam_hold_debug = 1U;
    BallBeam_SetState(BALLBEAM_STATE_LOST);
  }
}

void BallBeam_SetRun(uint8_t run)
{
  uint8_t requested = (run != 0U) ? 1U : 0U;

  if (requested == ballbeam.run_requested)
  {
    return;
  }

  ballbeam.run_requested = requested;
  g_ballbeam_run_debug = requested;
  if (requested == 0U)
  {
    BallBeam_StopMotor();
    BallBeam_ResetControl();
    BallBeam_SetState(BALLBEAM_STATE_IDLE);
  }
  else
  {
    ballbeam.have_position = 0U;
    BallBeam_ResetControl();
    BallBeam_SetState(BALLBEAM_STATE_ARMING);
  }
}

uint8_t BallBeam_GetRun(void)
{
  return ballbeam.run_requested;
}

void BallBeam_SetTarget(float target_position_cm)
{
  ballbeam.target_position_cm =
      BallBeam_ClampFloat(target_position_cm,
                          ballbeam.config.position_min_cm,
                          ballbeam.config.position_max_cm);
  g_ballbeam_target_position_cm_debug = ballbeam.target_position_cm;
}

void BallBeam_CommandAbsolutePulse(int32_t signed_pulse)
{
  ballbeam.last_motor_pulse = INT32_MAX;
  BallBeam_SendSignedPulse(signed_pulse);
}

void BallBeam_TriggerHome(void)
{
  Emm_V5_Origin_Trigger_Return(ballbeam.config.motor_address,
                               BALLBEAM_HOME_MODE,
                               false);
  ballbeam.last_motor_pulse = INT32_MAX;
  ballbeam.motor_stopped = 0U;
  g_ballbeam_home_command_count_debug++;
}

float BallBeam_GetTarget(void)
{
  return ballbeam.target_position_cm;
}

void BallBeam_EnableMotorOutput(uint8_t enable)
{
  if (enable == 0U)
  {
    BallBeam_StopMotor();
  }
  ballbeam.config.motor_output_enabled = (enable != 0U) ? 1U : 0U;
  ballbeam.last_motor_pulse = INT32_MAX;
}

void BallBeam_Reset(void)
{
  BallBeam_StopMotor();
  ballbeam.have_position = 0U;
  BallBeam_ResetControl();
  BallBeam_SetState((ballbeam.run_requested != 0U) ?
                    BALLBEAM_STATE_ARMING : BALLBEAM_STATE_IDLE);
  BallBeam_TriggerHome();
}

PID_t *BallBeam_GetPositionPID(void)
{
  return &ballbeam.position_pid;
}

BallBeam_Config_t *BallBeam_GetConfig(void)
{
  return &ballbeam.config;
}

BallBeam_State_t BallBeam_GetState(void)
{
  return ballbeam.state;
}
