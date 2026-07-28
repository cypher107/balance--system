#ifndef __ME01_H
#define __ME01_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include "usart.h"

typedef struct
{
  float raw_angle_deg;
  uint32_t receive_cycles;
  uint32_t sequence;
} ME01_AngleSample_t;

extern volatile float g_me01_raw_angle_debug;
extern volatile uint32_t g_me01_frame_count_debug;
extern volatile uint32_t g_me01_parse_error_debug;
extern volatile uint32_t g_me01_rx_overflow_debug;
extern volatile uint8_t g_me01_dma_status_debug;
extern volatile uint8_t g_me01_config_status_debug;

void ME01_Init(UART_HandleTypeDef *huart);
void ME01_Task(void);
void ME01_UART_RxEventCallback(UART_HandleTypeDef *huart, uint16_t size);
void ME01_UART_ErrorCallback(UART_HandleTypeDef *huart);
uint8_t ME01_GetLatest(ME01_AngleSample_t *sample, uint32_t *last_sequence);
void ME01_InjectAngle(float raw_angle_deg);

#ifdef __cplusplus
}
#endif

#endif
