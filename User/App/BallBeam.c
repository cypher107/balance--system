#include "BallBeam.h"

#include <limits.h>

#include "dwt_time.h"
#include "Emm_V5.h"
#include "ME01.h"
#include "stm32f1xx_hal.h"

typedef struct
{
  PID_t position_pid;
  PID_t angle_pid;
  BallBeam_Config_t config;
  BallBeam_State_t state;
  float target_position_mm;
  float position_mm;
  float target_beam_angle_deg;
  float measured_angle_deg;
  float motor_target_angle_deg;
  volatile float pending_position_mm;
  volatile uint32_t pending_position_cycles;
  volatile uint32_t pending_position_sequence;
  volatile uint8_t pending_position_valid;
  uint32_t position_sequence;
  uint32_t angle_sequence;
  uint32_t last_position_cycles;
  uint32_t last_angle_cycles;
  uint32_t last_position_tick_ms;
  uint32_t last_angle_tick_ms;
  uint8_t have_position;
  uint8_t have_angle;
  uint8_t run_requested;
  uint8_t motor_stopped;
  int32_t last_motor_pulse;
} BallBeam_Controller_t;

static BallBeam_Controller_t ballbeam;

volatile uint8_t g_ballbeam_state_debug;
volatile float g_ballbeam_position_debug;
volatile float g_ballbeam_target_beam_angle_debug;
volatile float g_ballbeam_measured_angle_debug;
volatile float g_ballbeam_motor_angle_debug;
volatile float g_ballbeam_position_dt_debug;
volatile float g_ballbeam_angle_dt_debug;
volatile int32_t g_ballbeam_motor_pulse_debug;
volatile uint32_t g_ballbeam_control_count_debug;
volatile uint32_t g_ballbeam_position_count_debug;
volatile uint32_t g_ballbeam_stale_count_debug;

static float BallBeam_Clamp(float value, float minimum, float maximum)
{
  if (value > maximum) return maximum;
  if (value < minimum) return minimum;
  return value;
}

static float BallBeam_Normalize180(float angle_deg)
{
  while (angle_deg > 180.0f) angle_deg -= 360.0f;
  while (angle_deg <= -180.0f) angle_deg += 360.0f;
  return angle_deg;
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
  PID_Reset(&ballbeam.angle_pid);
  ballbeam.target_beam_angle_deg = 0.0f;
  ballbeam.motor_target_angle_deg = 0.0f;
  ballbeam.last_position_cycles = 0U;
  ballbeam.last_angle_cycles = 0U;
  g_ballbeam_target_beam_angle_debug = 0.0f;
  g_ballbeam_motor_angle_debug = 0.0f;
  g_ballbeam_position_dt_debug = 0.0f;
  g_ballbeam_angle_dt_debug = 0.0f;
}

static void BallBeam_SendMotorAngle(float motor_angle_deg)
{
  float signed_position;
  int32_t signed_pulse;
  uint8_t direction;
  uint32_t pulse;

  motor_angle_deg = BallBeam_Clamp(motor_angle_deg,
                                   -ballbeam.config.max_motor_angle_deg,
                                   ballbeam.config.max_motor_angle_deg);
  signed_position = (float)ballbeam.config.motor_center_pulse +
                    (ballbeam.config.motor_direction * motor_angle_deg *
                     ballbeam.config.motor_pulses_per_degree);
  signed_pulse = BallBeam_RoundToInt(signed_position);

  g_ballbeam_motor_angle_debug = motor_angle_deg;
  g_ballbeam_motor_pulse_debug = signed_pulse;
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
                     15U,
                     5U,
                     pulse,
                     true,
                     false);
  ballbeam.last_motor_pulse = signed_pulse;
  ballbeam.motor_stopped = 0U;
}

static uint8_t BallBeam_ReadPosition(float *position_mm,
                                     uint32_t *receive_cycles,
                                     uint8_t *valid)
{
  uint32_t sequence;
  uint32_t primask;

  primask = __get_PRIMASK();
  __disable_irq();
  sequence = ballbeam.pending_position_sequence;
  if (sequence == ballbeam.position_sequence)
  {
    if (primask == 0U) __enable_irq();
    return 0U;
  }
  *position_mm = ballbeam.pending_position_mm;
  *receive_cycles = ballbeam.pending_position_cycles;
  *valid = ballbeam.pending_position_valid;
  ballbeam.position_sequence = sequence;
  if (primask == 0U) __enable_irq();
  return 1U;
}

static void BallBeam_UpdatePositionLoop(uint32_t now_ms)
{
  float position_mm;
  float dt_s;
  uint32_t receive_cycles;
  uint8_t valid;

  if (BallBeam_ReadPosition(&position_mm, &receive_cycles, &valid) == 0U)
  {
    return;
  }

  if ((valid == 0U) ||
      (position_mm < ballbeam.config.position_min_mm) ||
      (position_mm > ballbeam.config.position_max_mm))
  {
    ballbeam.have_position = 0U;
    return;
  }

  dt_s = 0.02f;
  if (ballbeam.last_position_cycles != 0U)
  {
    dt_s = DWT_Time_DeltaSeconds(receive_cycles,
                                 ballbeam.last_position_cycles);
  }
  ballbeam.last_position_cycles = receive_cycles;
  if ((dt_s < 0.001f) || (dt_s > 0.5f))
  {
    dt_s = 0.02f;
    PID_Reset(&ballbeam.position_pid);
  }

  ballbeam.position_mm = position_mm;
  ballbeam.last_position_tick_ms = now_ms;
  ballbeam.have_position = 1U;
  g_ballbeam_position_debug = position_mm;
  g_ballbeam_position_dt_debug = dt_s;
  g_ballbeam_position_count_debug++;

  if (ballbeam.run_requested != 0U)
  {
    ballbeam.target_beam_angle_deg =
        PID_CalculateDt(&ballbeam.position_pid,
                        ballbeam.target_position_mm,
                        position_mm,
                        dt_s) * ballbeam.config.position_direction;
    ballbeam.target_beam_angle_deg =
        BallBeam_Clamp(ballbeam.target_beam_angle_deg,
                       -ballbeam.config.max_beam_angle_deg,
                       ballbeam.config.max_beam_angle_deg);
    g_ballbeam_target_beam_angle_debug = ballbeam.target_beam_angle_deg;
  }
}

static void BallBeam_UpdateAngleLoop(uint32_t now_ms)
{
  ME01_AngleSample_t sample;
  float dt_s;

  if (ME01_GetLatest(&sample, &ballbeam.angle_sequence) == 0U)
  {
    return;
  }

  dt_s = 0.02f;
  if (ballbeam.last_angle_cycles != 0U)
  {
    dt_s = DWT_Time_DeltaSeconds(sample.receive_cycles,
                                 ballbeam.last_angle_cycles);
  }
  ballbeam.last_angle_cycles = sample.receive_cycles;
  if ((dt_s < 0.001f) || (dt_s > 0.2f))
  {
    dt_s = 0.02f;
    PID_Reset(&ballbeam.angle_pid);
  }

  ballbeam.measured_angle_deg =
      BallBeam_Normalize180(sample.raw_angle_deg -
                            ballbeam.config.angle_zero_deg) *
      ballbeam.config.sensor_direction;
  ballbeam.last_angle_tick_ms = now_ms;
  ballbeam.have_angle = 1U;
  g_ballbeam_measured_angle_debug = ballbeam.measured_angle_deg;
  g_ballbeam_angle_dt_debug = dt_s;

  if ((ballbeam.run_requested != 0U) &&
      (ballbeam.state == BALLBEAM_STATE_RUN) &&
      (ballbeam.have_position != 0U) &&
      ((now_ms - ballbeam.last_position_tick_ms) <=
       ballbeam.config.position_timeout_ms))
  {
    ballbeam.motor_target_angle_deg =
        PID_CalculateDt(&ballbeam.angle_pid,
                        ballbeam.target_beam_angle_deg,
                        ballbeam.measured_angle_deg,
                        dt_s);
    ballbeam.motor_target_angle_deg =
        BallBeam_Clamp(ballbeam.motor_target_angle_deg,
                       -ballbeam.config.max_motor_angle_deg,
                       ballbeam.config.max_motor_angle_deg);
    g_ballbeam_control_count_debug++;
    BallBeam_SendMotorAngle(ballbeam.motor_target_angle_deg);
  }
}

void BallBeam_Init(void)
{
  ballbeam.config.position_min_mm = -500.0f;
  ballbeam.config.position_max_mm = 500.0f;
  ballbeam.config.max_beam_angle_deg = 8.0f;
  ballbeam.config.max_motor_angle_deg = 30.0f;
  ballbeam.config.angle_zero_deg = 0.0f;
  ballbeam.config.motor_pulses_per_degree = 8.888889f;
  ballbeam.config.position_direction = 1.0f;
  ballbeam.config.sensor_direction = 1.0f;
  ballbeam.config.motor_direction = 1.0f;
  ballbeam.config.position_timeout_ms = 150U;
  ballbeam.config.angle_timeout_ms = 150U;
  ballbeam.config.motor_address = 1U;
  ballbeam.config.motor_positive_direction = 0U;
  ballbeam.config.motor_speed_rpm = 15U;
  ballbeam.config.motor_acceleration = 5U;
  ballbeam.config.motor_center_pulse = 0;
  ballbeam.config.motor_output_enabled = 1U;

  PID_Init(&ballbeam.position_pid, PID_MODE_POSITION, 0.05f, 0.0f, 0.01f);
  PID_SetOutputLimit(&ballbeam.position_pid, -8.0f, 8.0f);
  PID_SetIntegralLimit(&ballbeam.position_pid, -3.0f, 3.0f);
  PID_SetDeadband(&ballbeam.position_pid, 1.0f);
  PID_SetDerivativeFilter(&ballbeam.position_pid, 0.20f);

  PID_Init(&ballbeam.angle_pid, PID_MODE_POSITION, 2.0f, 0.0f, 0.05f);
  PID_SetOutputLimit(&ballbeam.angle_pid, -30.0f, 30.0f);
  PID_SetIntegralLimit(&ballbeam.angle_pid, -10.0f, 10.0f);
  PID_SetDeadband(&ballbeam.angle_pid, 0.10f);
  PID_SetDerivativeFilter(&ballbeam.angle_pid, 0.20f);

  ballbeam.target_position_mm = 0.0f;
  ballbeam.position_mm = 0.0f;
  ballbeam.measured_angle_deg = 0.0f;
  ballbeam.pending_position_mm = 0.0f;
  ballbeam.pending_position_cycles = 0U;
  ballbeam.pending_position_sequence = 0U;
  ballbeam.pending_position_valid = 0U;
  ballbeam.position_sequence = 0U;
  ballbeam.angle_sequence = 0U;
  ballbeam.last_position_tick_ms = HAL_GetTick();
  ballbeam.last_angle_tick_ms = HAL_GetTick();
  ballbeam.have_position = 0U;
  ballbeam.have_angle = 0U;
  ballbeam.run_requested = 0U;
  ballbeam.motor_stopped = 1U;
  ballbeam.last_motor_pulse = INT32_MAX;

  g_ballbeam_position_debug = 0.0f;
  g_ballbeam_measured_angle_debug = 0.0f;
  g_ballbeam_motor_pulse_debug = 0;
  g_ballbeam_control_count_debug = 0U;
  g_ballbeam_position_count_debug = 0U;
  g_ballbeam_stale_count_debug = 0U;
  BallBeam_ResetControl();
  BallBeam_SetState(BALLBEAM_STATE_IDLE);
}

void BallBeam_Task(void)
{
  uint32_t now_ms = HAL_GetTick();
  uint8_t data_ready;

  BallBeam_UpdatePositionLoop(now_ms);
  BallBeam_UpdateAngleLoop(now_ms);

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

  data_ready = ((ballbeam.have_position != 0U) &&
                (ballbeam.have_angle != 0U) &&
                ((now_ms - ballbeam.last_position_tick_ms) <=
                 ballbeam.config.position_timeout_ms) &&
                ((now_ms - ballbeam.last_angle_tick_ms) <=
                 ballbeam.config.angle_timeout_ms)) ? 1U : 0U;

  if (data_ready != 0U)
  {
    if (ballbeam.state != BALLBEAM_STATE_RUN)
    {
      PID_Reset(&ballbeam.position_pid);
      PID_Reset(&ballbeam.angle_pid);
      ballbeam.last_position_cycles = 0U;
      ballbeam.last_angle_cycles = 0U;
      ballbeam.last_motor_pulse = INT32_MAX;
      BallBeam_SetState(BALLBEAM_STATE_RUN);
    }
  }
  else
  {
    if ((ballbeam.state == BALLBEAM_STATE_RUN) ||
        (ballbeam.state == BALLBEAM_STATE_ARMING))
    {
      g_ballbeam_stale_count_debug++;
      BallBeam_StopMotor();
    }
    BallBeam_SetState((ballbeam.have_position == 0U ||
                       ballbeam.have_angle == 0U) ?
                      BALLBEAM_STATE_ARMING : BALLBEAM_STATE_LOST);
  }
}

void BallBeam_SetRun(uint8_t run)
{
  uint8_t requested = (run != 0U) ? 1U : 0U;

  if (requested != ballbeam.run_requested)
  {
    ballbeam.run_requested = requested;
    if (requested == 0U)
    {
      BallBeam_StopMotor();
      BallBeam_ResetControl();
      BallBeam_SetState(BALLBEAM_STATE_IDLE);
    }
    else
    {
      BallBeam_SetState(BALLBEAM_STATE_ARMING);
    }
  }
}

uint8_t BallBeam_GetRun(void)
{
  return ballbeam.run_requested;
}

void BallBeam_SetTarget(float target_position_mm)
{
  ballbeam.target_position_mm = BallBeam_Clamp(target_position_mm,
                                               ballbeam.config.position_min_mm,
                                               ballbeam.config.position_max_mm);
}

float BallBeam_GetTarget(void)
{
  return ballbeam.target_position_mm;
}

void BallBeam_SetPositionFeedback(float position_mm, uint8_t valid)
{
  uint32_t primask = __get_PRIMASK();

  __disable_irq();
  ballbeam.pending_position_mm = position_mm;
  ballbeam.pending_position_cycles = DWT_Time_GetCycles();
  ballbeam.pending_position_valid = (valid != 0U) ? 1U : 0U;
  ballbeam.pending_position_sequence++;
  if (primask == 0U) __enable_irq();
}

void BallBeam_EnableMotorOutput(uint8_t enable)
{
  if (enable == 0U) BallBeam_StopMotor();
  ballbeam.config.motor_output_enabled = (enable != 0U) ? 1U : 0U;
  ballbeam.last_motor_pulse = INT32_MAX;
}

void BallBeam_Reset(void)
{
  BallBeam_StopMotor();
  ballbeam.have_position = 0U;
  ballbeam.have_angle = 0U;
  BallBeam_ResetControl();
  BallBeam_SetState((ballbeam.run_requested != 0U) ?
                    BALLBEAM_STATE_ARMING : BALLBEAM_STATE_IDLE);
}

PID_t *BallBeam_GetPositionPID(void)
{
  return &ballbeam.position_pid;
}

PID_t *BallBeam_GetAnglePID(void)
{
  return &ballbeam.angle_pid;
}

BallBeam_Config_t *BallBeam_GetConfig(void)
{
  return &ballbeam.config;
}

BallBeam_State_t BallBeam_GetState(void)
{
  return ballbeam.state;
}
