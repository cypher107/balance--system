#include "pid.h"

static float PID_Abs(float value)
{
  return (value >= 0.0f) ? value : -value;
}

static float PID_Clamp(float value, float min, float max)
{
  if (min > max)
  {
    float temp = min;
    min = max;
    max = temp;
  }

  if (value > max)
  {
    return max;
  }

  if (value < min)
  {
    return min;
  }

  return value;
}

void PID_Init(PID_t *pid, PID_Mode_t mode, float kp, float ki, float kd)
{
  if (pid == 0)
  {
    return;
  }

  pid->kp = kp;
  pid->ki = ki;
  pid->kd = kd;
  pid->mode = mode;

  pid->output_min = -10000.0f;
  pid->output_max = 10000.0f;
  pid->integral_min = -10000.0f;
  pid->integral_max = 10000.0f;
  pid->deadband = 0.0f;
  pid->derivative_filter = 1.0f;
  pid->enable_integral_separation = 0U;
  pid->integral_separation_error = 0.0f;

  PID_Reset(pid);
}

void PID_Reset(PID_t *pid)
{
  if (pid == 0)
  {
    return;
  }

  pid->setpoint = 0.0f;
  pid->feedback = 0.0f;
  pid->output = 0.0f;

  pid->error = 0.0f;
  pid->last_error = 0.0f;
  pid->prev_error = 0.0f;

  pid->proportional = 0.0f;
  pid->integral = 0.0f;
  pid->derivative = 0.0f;
  pid->last_derivative = 0.0f;
}

void PID_SetMode(PID_t *pid, PID_Mode_t mode)
{
  if (pid == 0)
  {
    return;
  }

  pid->mode = mode;
  pid->last_error = 0.0f;
  pid->prev_error = 0.0f;
}

void PID_SetTarget(PID_t *pid, float setpoint)
{
  if (pid == 0)
  {
    return;
  }

  pid->setpoint = setpoint;
}

void PID_SetFeedback(PID_t *pid, float feedback)
{
  if (pid == 0)
  {
    return;
  }

  pid->feedback = feedback;
}

void PID_SetTunings(PID_t *pid, float kp, float ki, float kd)
{
  if (pid == 0)
  {
    return;
  }

  pid->kp = kp;
  pid->ki = ki;
  pid->kd = kd;
}

void PID_SetOutputLimit(PID_t *pid, float min, float max)
{
  if (pid == 0)
  {
    return;
  }

  pid->output_min = min;
  pid->output_max = max;
  pid->output = PID_Clamp(pid->output, min, max);
}

void PID_SetIntegralLimit(PID_t *pid, float min, float max)
{
  if (pid == 0)
  {
    return;
  }

  pid->integral_min = min;
  pid->integral_max = max;
  pid->integral = PID_Clamp(pid->integral, min, max);
}

void PID_SetDeadband(PID_t *pid, float deadband)
{
  if (pid == 0)
  {
    return;
  }

  pid->deadband = PID_Abs(deadband);
}

void PID_SetDerivativeFilter(PID_t *pid, float alpha)
{
  if (pid == 0)
  {
    return;
  }

  pid->derivative_filter = PID_Clamp(alpha, 0.0f, 1.0f);
}

void PID_SetIntegralSeparation(PID_t *pid, uint8_t enable, float threshold)
{
  if (pid == 0)
  {
    return;
  }

  pid->enable_integral_separation = enable ? 1U : 0U;
  pid->integral_separation_error = PID_Abs(threshold);
}

float PID_Calculate(PID_t *pid, float setpoint, float feedback)
{
  if (pid == 0)
  {
    return 0.0f;
  }

  pid->setpoint = setpoint;
  pid->feedback = feedback;

  return PID_Update(pid);
}

float PID_Update(PID_t *pid)
{
  return PID_UpdateDt(pid, 1.0f);
}

float PID_CalculateDt(PID_t *pid, float setpoint, float feedback, float dt_s)
{
  if (pid == 0)
  {
    return 0.0f;
  }

  pid->setpoint = setpoint;
  pid->feedback = feedback;

  return PID_UpdateDt(pid, dt_s);
}

float PID_UpdateDt(PID_t *pid, float dt_s)
{
  float raw_derivative;
  uint8_t use_integral = 1U;

  if (pid == 0)
  {
    return 0.0f;
  }

  if (dt_s <= 0.000001f)
  {
    return pid->output;
  }

  pid->error = pid->setpoint - pid->feedback;

  if (PID_Abs(pid->error) <= pid->deadband)
  {
    pid->error = 0.0f;
  }

  if ((pid->enable_integral_separation != 0U) &&
      (pid->integral_separation_error > 0.0f) &&
      (PID_Abs(pid->error) > pid->integral_separation_error))
  {
    use_integral = 0U;
  }

  if (pid->mode == PID_MODE_INCREMENTAL)
  {
    pid->proportional = pid->kp * (pid->error - pid->last_error);
    pid->integral = use_integral ? (pid->ki * pid->error * dt_s) : 0.0f;
    pid->derivative = (pid->kd / dt_s) *
                      (pid->error - (2.0f * pid->last_error) + pid->prev_error);
    pid->output += pid->proportional + pid->integral + pid->derivative;
  }
  else
  {
    pid->proportional = pid->kp * pid->error;

    if (use_integral != 0U)
    {
      pid->integral += pid->ki * pid->error * dt_s;
      pid->integral = PID_Clamp(pid->integral, pid->integral_min, pid->integral_max);
    }

    raw_derivative = (pid->error - pid->last_error) / dt_s;
    pid->last_derivative += pid->derivative_filter * (raw_derivative - pid->last_derivative);
    pid->derivative = pid->kd * pid->last_derivative;

    pid->output = pid->proportional + pid->integral + pid->derivative;
  }

  pid->output = PID_Clamp(pid->output, pid->output_min, pid->output_max);

  pid->prev_error = pid->last_error;
  pid->last_error = pid->error;

  return pid->output;
}
