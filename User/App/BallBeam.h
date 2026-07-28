#ifndef __BALL_BEAM_H
#define __BALL_BEAM_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include "pid.h"

typedef enum
{
  BALLBEAM_STATE_IDLE = 0,
  BALLBEAM_STATE_ARMING,
  BALLBEAM_STATE_RUN,
  BALLBEAM_STATE_LOST,
  BALLBEAM_STATE_FAULT
} BallBeam_State_t;

typedef struct
{
  float position_min_mm;
  float position_max_mm;
  float max_beam_angle_deg;
  float max_motor_angle_deg;
  float angle_zero_deg;
  float motor_pulses_per_degree;
  float position_direction;
  float sensor_direction;
  float motor_direction;
  uint32_t position_timeout_ms;
  uint32_t angle_timeout_ms;
  uint8_t motor_address;
  uint8_t motor_positive_direction;
  uint16_t motor_speed_rpm;
  uint8_t motor_acceleration;
  int32_t motor_center_pulse;
  uint8_t motor_output_enabled;
} BallBeam_Config_t;

extern volatile uint8_t g_ballbeam_state_debug;
extern volatile float g_ballbeam_position_debug;
extern volatile float g_ballbeam_target_beam_angle_debug;
extern volatile float g_ballbeam_measured_angle_debug;
extern volatile float g_ballbeam_motor_angle_debug;
extern volatile float g_ballbeam_position_dt_debug;
extern volatile float g_ballbeam_angle_dt_debug;
extern volatile int32_t g_ballbeam_motor_pulse_debug;
extern volatile uint32_t g_ballbeam_control_count_debug;
extern volatile uint32_t g_ballbeam_position_count_debug;
extern volatile uint32_t g_ballbeam_stale_count_debug;

void BallBeam_Init(void);
void BallBeam_Task(void);
void BallBeam_SetRun(uint8_t run);
uint8_t BallBeam_GetRun(void);
void BallBeam_SetTarget(float target_position_mm);
float BallBeam_GetTarget(void);
void BallBeam_SetPositionFeedback(float position_mm, uint8_t valid);
void BallBeam_EnableMotorOutput(uint8_t enable);
void BallBeam_Reset(void);
PID_t *BallBeam_GetPositionPID(void);
PID_t *BallBeam_GetAnglePID(void);
BallBeam_Config_t *BallBeam_GetConfig(void);
BallBeam_State_t BallBeam_GetState(void);

#ifdef __cplusplus
}
#endif

#endif
