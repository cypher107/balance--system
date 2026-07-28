#ifndef __SCREEN_H
#define __SCREEN_H

#include <stdint.h>
#include "usart.h"

extern volatile uint8_t g_screen_rx_byte_debug;
extern volatile uint8_t g_screen_cmd_debug;
extern volatile uint32_t g_screen_rx_count_debug;
extern volatile uint8_t g_screen_tjc_return_code_debug;
extern volatile uint32_t g_screen_tjc_return_count_debug;

void Screen_Init(void);
void Screen_Task(void);
void Screen_UART4_RxCpltCallback(UART_HandleTypeDef *huart);

#endif
