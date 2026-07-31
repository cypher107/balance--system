#ifndef __TASK_HMI_H
#define __TASK_HMI_H

#include <stdint.h>
#include "usart.h"

extern volatile uint8_t g_task_hmi_rx_byte_debug;
extern volatile uint8_t g_task_hmi_command_debug;
extern volatile uint32_t g_task_hmi_rx_count_debug;
extern volatile uint32_t g_task_hmi_command_count_debug;
extern volatile uint8_t g_task_hmi_rx_status_debug;
extern volatile uint8_t g_task_hmi_frame_debug[8];
extern volatile uint8_t g_task_hmi_frame_index_debug;
extern volatile uint32_t g_task_hmi_frame_count_debug;
extern volatile uint32_t g_task_hmi_invalid_frame_count_debug;
extern volatile uint8_t g_task_hmi_parameter_command_debug;
extern volatile uint8_t g_task_hmi_ball_p_debug;
extern volatile uint8_t g_task_hmi_ball_d_debug;
extern volatile uint8_t g_task_hmi_ball_angle_debug;
extern volatile uint8_t g_task_hmi_tracking_p_debug;
extern volatile uint8_t g_task_hmi_tracking_delay_10ms_debug;
extern volatile uint32_t g_task_hmi_parameter_count_debug;

void TaskHmi_Init(UART_HandleTypeDef *huart);
void TaskHmi_Task(void);
void TaskHmi_UART_RxCpltCallback(UART_HandleTypeDef *huart);
void TaskHmi_UART_ErrorCallback(UART_HandleTypeDef *huart);
uint8_t TaskHmi_GetCommand(uint8_t *task_id);

#endif
