#ifndef __TRACKING_H
#define __TRACKING_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

extern volatile float g_tracking_kp_debug;
extern volatile float g_tracking_ki_debug;
extern volatile float g_tracking_kd_debug;
extern volatile float g_tracking_base_speed_debug;
extern volatile float g_tracking_left_speed_scale_debug;
extern volatile float g_tracking_right_speed_scale_debug;
extern volatile float g_tracking_position_debug;
extern volatile float g_tracking_pid_output_debug;
extern volatile uint8_t g_tracking_active_count_debug;
extern volatile uint8_t g_tracking_lost_count_debug;
extern volatile uint8_t g_tracking_on_line_debug;
extern volatile uint8_t g_tracking_sensor_valid_debug;

void Tracking_Init(void);
void Tracking_Process(void);
void Tracking_Reset(void);
void Tracking_SetSpeed(float speed);
void Tracking_Update(void);
float Tracking_GetPosition(void);
uint8_t Tracking_GetActiveCount(void);
uint8_t Tracking_IsOnLine(void);

#ifdef __cplusplus
}
#endif

#endif
