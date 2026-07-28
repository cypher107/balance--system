#include "tjc_usart_hmi.h"

#include <stdio.h>
#include <string.h>

typedef struct
{
  volatile uint16_t Head;
  volatile uint16_t Tail;
  volatile uint16_t Length;
  volatile uint8_t Ring_data[RINGBUFFER_LEN];
} RingBuffer_t;

static volatile RingBuffer_t ringBuffer;
volatile uint8_t RxBuffer[1];
volatile uint32_t tjc_ring_overflow_count;
volatile uint8_t tjc_tx_data_debug[132];
volatile uint16_t tjc_tx_length_debug;
volatile uint8_t tjc_tx_status_debug;
volatile uint32_t tjc_tx_count_debug;

static uint32_t TJC_EnterCritical(void)
{
  uint32_t primask = __get_PRIMASK();
  __disable_irq();
  return primask;
}

static void TJC_ExitCritical(uint32_t primask)
{
  if ((primask & 1U) == 0U)
  {
    __enable_irq();
  }
}

static void TJC_SendFrame(const uint8_t *data, uint16_t length)
{
  static const uint8_t end[3] = {0xFFU, 0xFFU, 0xFFU};
  HAL_StatusTypeDef status = HAL_OK;
  uint16_t i;
  uint16_t debug_length = length;

  if ((data == 0) && (length > 0U))
  {
    return;
  }

  if (debug_length > 129U)
  {
    debug_length = 129U;
  }
  for (i = 0U; i < debug_length; i++)
  {
    tjc_tx_data_debug[i] = data[i];
  }
  for (i = 0U; i < 3U; i++)
  {
    tjc_tx_data_debug[debug_length + i] = 0xFFU;
  }
  tjc_tx_length_debug = (uint16_t)(debug_length + 3U);
  tjc_tx_count_debug++;

  if (length > 0U)
  {
    status = HAL_UART_Transmit(&TJC_UART, (uint8_t *)data, length, 100U);
  }
  if (status == HAL_OK)
  {
    status = HAL_UART_Transmit(&TJC_UART, (uint8_t *)end, 3U, 100U);
  }
  tjc_tx_status_debug = (uint8_t)status;
}

void TJC_UART_Init(void)
{
  initRingBuffer();
  RxBuffer[0] = 0U;
  tjc_tx_length_debug = 0U;
  tjc_tx_status_debug = 0U;
  tjc_tx_count_debug = 0U;
  (void)HAL_UART_Receive_IT(&TJC_UART, (uint8_t *)RxBuffer, 1U);
}

void TJC_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
  if ((huart != 0) && (huart->Instance == TJC_UART_INS))
  {
    write1ByteToRingBuffer(RxBuffer[0]);
    (void)HAL_UART_Receive_IT(&TJC_UART, (uint8_t *)RxBuffer, 1U);
  }
}

void tjc_send_string(const char *str)
{
  if (str == 0)
  {
    return;
  }
  TJC_SendFrame((const uint8_t *)str, (uint16_t)strlen(str));
}

void tjc_send_txt(const char *objname, const char *attribute, const char *txt)
{
  char command[128];
  int length;

  if ((objname == 0) || (attribute == 0) || (txt == 0))
  {
    return;
  }

  length = snprintf(command, sizeof(command), "%s.%s=\"%s\"",
                    objname, attribute, txt);
  if ((length > 0) && (length < (int)sizeof(command)))
  {
    TJC_SendFrame((const uint8_t *)command, (uint16_t)length);
  }
}

void tjc_send_val(const char *objname, const char *attribute, int val)
{
  char command[64];
  int length;

  if ((objname == 0) || (attribute == 0))
  {
    return;
  }

  length = snprintf(command, sizeof(command), "%s.%s=%d",
                    objname, attribute, val);
  if ((length > 0) && (length < (int)sizeof(command)))
  {
    TJC_SendFrame((const uint8_t *)command, (uint16_t)length);
  }
}

void tjc_send_nstring(const char *str, uint8_t str_length)
{
  if ((str == 0) && (str_length > 0U))
  {
    return;
  }
  TJC_SendFrame((const uint8_t *)str, str_length);
}

void initRingBuffer(void)
{
  uint32_t primask = TJC_EnterCritical();

  ringBuffer.Head = 0U;
  ringBuffer.Tail = 0U;
  ringBuffer.Length = 0U;
  tjc_ring_overflow_count = 0U;

  TJC_ExitCritical(primask);
}

void write1ByteToRingBuffer(uint8_t data)
{
  uint32_t primask = TJC_EnterCritical();

  if (ringBuffer.Length < RINGBUFFER_LEN)
  {
    ringBuffer.Ring_data[ringBuffer.Tail] = data;
    ringBuffer.Tail = (uint16_t)((ringBuffer.Tail + 1U) % RINGBUFFER_LEN);
    ringBuffer.Length++;
  }
  else
  {
    tjc_ring_overflow_count++;
  }

  TJC_ExitCritical(primask);
}

void deleteRingBuffer(uint16_t size)
{
  uint32_t primask = TJC_EnterCritical();

  if (size >= ringBuffer.Length)
  {
    ringBuffer.Head = 0U;
    ringBuffer.Tail = 0U;
    ringBuffer.Length = 0U;
  }
  else
  {
    ringBuffer.Head = (uint16_t)((ringBuffer.Head + size) % RINGBUFFER_LEN);
    ringBuffer.Length = (uint16_t)(ringBuffer.Length - size);
  }

  TJC_ExitCritical(primask);
}

uint16_t getRingBufferLength(void)
{
  uint16_t length;
  uint32_t primask = TJC_EnterCritical();

  length = ringBuffer.Length;

  TJC_ExitCritical(primask);
  return length;
}

uint8_t read1ByteFromRingBuffer(uint16_t position)
{
  uint8_t data = 0U;
  uint32_t primask = TJC_EnterCritical();

  if (position < ringBuffer.Length)
  {
    data = ringBuffer.Ring_data[(ringBuffer.Head + position) % RINGBUFFER_LEN];
  }

  TJC_ExitCritical(primask);
  return data;
}

uint8_t tjc_read_byte(uint8_t *data)
{
  uint8_t result = 0U;
  uint32_t primask;

  if (data == 0)
  {
    return 0U;
  }

  primask = TJC_EnterCritical();
  if (ringBuffer.Length > 0U)
  {
    *data = ringBuffer.Ring_data[ringBuffer.Head];
    ringBuffer.Head = (uint16_t)((ringBuffer.Head + 1U) % RINGBUFFER_LEN);
    ringBuffer.Length--;
    result = 1U;
  }
  TJC_ExitCritical(primask);

  return result;
}

uint8_t isRingBufferOverflow(void)
{
  return (getRingBufferLength() < RINGBUFFER_LEN) ? 1U : 0U;
}
