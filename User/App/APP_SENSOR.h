#ifndef APP_SENSOR_H
#define APP_SENSOR_H

#include <stdint.h>
#include "usart.h"

#define SENSOR_TOTAL_NUM       12U
#define APP_SENSOR_FRAME_SIZE  16U

extern volatile uint8_t g_grayscale_values_debug[SENSOR_TOTAL_NUM];

extern volatile uint8_t g_sensor_rx_byte_debug;
extern volatile uint8_t g_sensor_last_frame_debug[APP_SENSOR_FRAME_SIZE];
extern volatile uint16_t g_sensor_calc_crc_debug;
extern volatile uint16_t g_sensor_recv_crc_debug;
extern volatile uint8_t g_sensor_valid_debug;
extern volatile uint8_t g_sensor_rx_status_debug;
extern volatile uint32_t g_sensor_rx_count_debug;
extern volatile uint32_t g_sensor_frame_count_debug;
extern volatile uint32_t g_sensor_valid_frame_count_debug;
extern volatile uint32_t g_sensor_crc_error_count_debug;
extern volatile uint32_t g_sensor_request_count_debug;
extern volatile uint32_t g_sensor_timeout_count_debug;
extern volatile uint32_t g_sensor_uart_error_count_debug;

void APP_SENSOR_Init(UART_HandleTypeDef *huart);
void APP_SENSOR_Task(void);
void Uart_Data_Process(void);
void APP_SENSOR_UART_RxCpltCallback(UART_HandleTypeDef *huart);
void APP_SENSOR_UART_ErrorCallback(UART_HandleTypeDef *huart);
uint8_t APP_SENSOR_IsValid(uint32_t timeout_ms);

#endif
