#ifndef __PID_H
#define __PID_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

typedef enum
{
  PID_MODE_POSITION = 0,
  PID_MODE_INCREMENTAL
} PID_Mode_t;

typedef struct
{
  float kp;
  float ki;
  float kd;

  float setpoint;
  float feedback;
  float output;

  float error;
  float last_error;
  float prev_error;

  float proportional;
  float integral;
  float derivative;

  float output_min;
  float output_max;
  float integral_min;
  float integral_max;

  float deadband;
  float derivative_filter;
  float last_derivative;

  PID_Mode_t mode;
  uint8_t enable_integral_separation;
  float integral_separation_error;
} PID_t;

void PID_Init(PID_t *pid, PID_Mode_t mode, float kp, float ki, float kd);
void PID_Reset(PID_t *pid);
void PID_SetMode(PID_t *pid, PID_Mode_t mode);
void PID_SetTarget(PID_t *pid, float setpoint);
void PID_SetFeedback(PID_t *pid, float feedback);
void PID_SetTunings(PID_t *pid, float kp, float ki, float kd);
void PID_SetOutputLimit(PID_t *pid, float min, float max);
void PID_SetIntegralLimit(PID_t *pid, float min, float max);
void PID_SetDeadband(PID_t *pid, float deadband);
void PID_SetDerivativeFilter(PID_t *pid, float alpha);
void PID_SetIntegralSeparation(PID_t *pid, uint8_t enable, float threshold);
float PID_Calculate(PID_t *pid, float setpoint, float feedback);
float PID_Update(PID_t *pid);
float PID_CalculateDt(PID_t *pid, float setpoint, float feedback, float dt_s);
float PID_UpdateDt(PID_t *pid, float dt_s);

#ifdef __cplusplus
}
#endif

#endif
