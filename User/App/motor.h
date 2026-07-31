#ifndef __MOTOR_H
#define __MOTOR_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

extern volatile int16_t g_motor_left_speed_debug;
extern volatile int16_t g_motor_right_speed_debug;
extern volatile uint32_t g_motor_command_count_debug;

void Motor_Init(void);
void Motor_SetSpeed(int16_t left_speed, int16_t right_speed);
void Motor_Stop(void);

#ifdef __cplusplus
}
#endif

#endif
