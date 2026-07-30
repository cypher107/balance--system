#ifndef __VISION_H
#define __VISION_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include "usart.h"

typedef struct
{
  uint16_t position_raw;
  float position_cm;
  uint32_t receive_cycles;
  uint32_t sequence;
} Vision_BallSample_t;

extern volatile uint16_t g_vision_position_raw_debug;
extern volatile float g_vision_position_cm_debug;
extern volatile float g_vision_position_debug;
extern volatile uint32_t g_vision_frame_count_debug;
extern volatile uint32_t g_vision_parse_error_debug;
extern volatile uint32_t g_vision_rx_overflow_debug;
extern volatile uint8_t g_vision_rx_status_debug;
extern volatile uint8_t g_vision_dma_status_debug;
extern volatile uint8_t g_vision_rx_byte_debug;

void Vision_Init(UART_HandleTypeDef *huart);
void Vision_Task(void);
void Vision_UART_RxEventCallback(UART_HandleTypeDef *huart, uint16_t size);
void Vision_UART_ErrorCallback(UART_HandleTypeDef *huart);
uint8_t Vision_GetLatest(Vision_BallSample_t *sample, uint32_t *last_sequence);
void Vision_InjectSample(float position_cm);

#ifdef __cplusplus
}
#endif

#endif
