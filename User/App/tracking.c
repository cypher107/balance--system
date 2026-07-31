#include "tracking.h"

#include "APP_SENSOR.h"
#include "motor.h"
#include "pid.h"

#define TRACK_DEFAULT_KP            1.2f
#define TRACK_DEFAULT_KI            5.0f
#define TRACK_DEFAULT_KD            0.002f
#define TRACK_PERIOD_S              0.01f
#define TRACK_LOST_STOP_COUNT       20U
#define TRACK_OUTPUT_LIMIT          300.0f
#define TRACK_INTEGRAL_LIMIT        120.0f
#define TRACK_SENSOR_TIMEOUT_MS     100U

static PID_t tracking_pid;
static float tracking_last_position;

volatile float g_tracking_kp_debug = TRACK_DEFAULT_KP;
volatile float g_tracking_ki_debug = TRACK_DEFAULT_KI;
volatile float g_tracking_kd_debug = TRACK_DEFAULT_KD;
volatile float g_tracking_base_speed_debug = 300.0f;
volatile float g_tracking_position_debug;
volatile float g_tracking_pid_output_debug;
volatile uint8_t g_tracking_active_count_debug;
volatile uint8_t g_tracking_lost_count_debug;
volatile uint8_t g_tracking_on_line_debug;
volatile uint8_t g_tracking_sensor_valid_debug;

static uint8_t Tracking_ReadGray(float *position)
{
  float sum = 0.0f;
  uint8_t count = 0U;
  uint8_t index;

  if (APP_SENSOR_IsValid(TRACK_SENSOR_TIMEOUT_MS) == 0U)
  {
    g_tracking_sensor_valid_debug = 0U;
    g_tracking_active_count_debug = 0U;
    return 0U;
  }

  g_tracking_sensor_valid_debug = 1U;
  for (index = 0U; index < SENSOR_TOTAL_NUM; index++)
  {
    if (g_grayscale_values_debug[index] != 0U)
    {
      sum += (float)index - 5.5f;
      count++;
    }
  }

  g_tracking_active_count_debug = count;
  if (count == 0U)
  {
    return 0U;
  }

  *position = sum / (float)count;
  return 1U;
}

void Tracking_Init(void)
{
  PID_Init(&tracking_pid,
           PID_MODE_POSITION,
           TRACK_DEFAULT_KP,
           TRACK_DEFAULT_KI,
           TRACK_DEFAULT_KD);
  PID_SetOutputLimit(&tracking_pid,
                     -TRACK_OUTPUT_LIMIT,
                     TRACK_OUTPUT_LIMIT);
  PID_SetIntegralLimit(&tracking_pid,
                       -TRACK_INTEGRAL_LIMIT,
                       TRACK_INTEGRAL_LIMIT);
  PID_SetDerivativeFilter(&tracking_pid, 0.25f);
  Tracking_Reset();
}

void Tracking_Reset(void)
{
  PID_Reset(&tracking_pid);
  tracking_last_position = 0.0f;
  g_tracking_position_debug = 0.0f;
  g_tracking_pid_output_debug = 0.0f;
  g_tracking_active_count_debug = 0U;
  g_tracking_lost_count_debug = 0U;
  g_tracking_on_line_debug = 0U;
  g_tracking_sensor_valid_debug = 0U;
}

void Tracking_SetSpeed(float speed)
{
  if (speed < 0.0f) speed = 0.0f;
  if (speed > 800.0f) speed = 800.0f;
  g_tracking_base_speed_debug = speed;
}

void Tracking_Update(void)
{
  float position;

  if (Tracking_ReadGray(&position) != 0U)
  {
    g_tracking_position_debug = position;
    tracking_last_position = position;
    g_tracking_lost_count_debug = 0U;
    g_tracking_on_line_debug = 1U;
    return;
  }

  if ((g_tracking_sensor_valid_debug != 0U) &&
      (g_tracking_lost_count_debug < TRACK_LOST_STOP_COUNT))
  {
    g_tracking_lost_count_debug++;
    g_tracking_position_debug = tracking_last_position;
    g_tracking_on_line_debug = 1U;
  }
  else
  {
    g_tracking_on_line_debug = 0U;
  }
}

void Tracking_Process(void)
{
  float base_speed;
  float pid_output;
  int16_t left_speed;
  int16_t right_speed;

  Tracking_Update();
  if (g_tracking_on_line_debug == 0U)
  {
    g_tracking_pid_output_debug = 0.0f;
    Motor_Stop();
    return;
  }

  if ((g_tracking_kp_debug >= 0.0f) &&
      (g_tracking_ki_debug >= 0.0f) &&
      (g_tracking_kd_debug >= 0.0f))
  {
    PID_SetTunings(&tracking_pid,
                   g_tracking_kp_debug,
                   g_tracking_ki_debug,
                   g_tracking_kd_debug);
  }

  base_speed = g_tracking_base_speed_debug;
  if (g_tracking_lost_count_debug > 0U)
  {
    base_speed *= 0.5f;
  }

  pid_output = PID_CalculateDt(&tracking_pid,
                               0.0f,
                               g_tracking_position_debug,
                               TRACK_PERIOD_S);
  g_tracking_pid_output_debug = pid_output;
  left_speed = (int16_t)(base_speed - pid_output);
  right_speed = (int16_t)(base_speed + pid_output);
  Motor_SetSpeed(left_speed, right_speed);
}

float Tracking_GetPosition(void)
{
  return g_tracking_position_debug;
}

uint8_t Tracking_GetActiveCount(void)
{
  return g_tracking_active_count_debug;
}

uint8_t Tracking_IsOnLine(void)
{
  return g_tracking_on_line_debug;
}
