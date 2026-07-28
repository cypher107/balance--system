#ifndef __VOFA_H
#define __VOFA_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include "pid.h"
#include "usart.h"

#define VOFA_MAX_PID_CHANNELS  8U
#define VOFA_RX_BUFFER_SIZE    96U

typedef struct
{
  uint8_t used;
  uint8_t id;
  const char *name;
  PID_t *pid;
} VOFA_PID_Channel_t;

void VOFA_Init(UART_HandleTypeDef *huart);
void VOFA_AttachPID(uint8_t id, const char *name, PID_t *pid);
void VOFA_DetachPID(uint8_t id);

void VOFA_RxByte(uint8_t data);
void VOFA_RxData(const uint8_t *data, uint16_t len);
void VOFA_ProcessLine(char *line);

void VOFA_SendString(const char *str);
void VOFA_SendPID(uint8_t id);
void VOFA_SendAllPID(void);
void VOFA_SendWave(const float *channels, uint8_t count);
void VOFA_SendNamedWave(float target, float feedback, float output, float error);

#ifdef __cplusplus
}
#endif

#endif
