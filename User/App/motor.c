#include "motor.h"

#include "tim.h"

#define MOTOR_SPEED_LIMIT 800

volatile int16_t g_motor_left_speed_debug;
volatile int16_t g_motor_right_speed_debug;
volatile uint32_t g_motor_command_count_debug;

static int16_t Motor_ClampSpeed(int16_t speed)
{
  if (speed > MOTOR_SPEED_LIMIT) return MOTOR_SPEED_LIMIT;
  if (speed < -MOTOR_SPEED_LIMIT) return -MOTOR_SPEED_LIMIT;
  return speed;
}

void Motor_Init(void)
{
  (void)HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_3);
  (void)HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_4);
  (void)HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_3);
  (void)HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_4);
  g_motor_command_count_debug = 0U;
  Motor_Stop();
}

void Motor_SetSpeed(int16_t left_speed, int16_t right_speed)
{
  left_speed = Motor_ClampSpeed(left_speed);
  right_speed = Motor_ClampSpeed(right_speed);

  g_motor_left_speed_debug = left_speed;
  g_motor_right_speed_debug = right_speed;
  g_motor_command_count_debug++;

  if (left_speed >= 0)
  {
    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_3, (uint16_t)left_speed);
    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_4, 0U);
  }
  else
  {
    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_3, 0U);
    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_4, (uint16_t)(-left_speed));
  }

  if (right_speed >= 0)
  {
    __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_3, (uint16_t)right_speed);
    __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_4, 0U);
  }
  else
  {
    __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_3, 0U);
    __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_4, (uint16_t)(-right_speed));
  }
}

void Motor_Stop(void)
{
  __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_3, 0U);
  __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_4, 0U);
  __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_3, 0U);
  __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_4, 0U);
  g_motor_left_speed_debug = 0;
  g_motor_right_speed_debug = 0;
}
