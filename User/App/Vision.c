#include "Vision.h"

#include <stdlib.h>
#include <string.h>

#include "dwt_time.h"

#define VISION_DMA_BUFFER_SIZE 128U
#define VISION_RING_SIZE       256U
#define VISION_LINE_SIZE       80U

static UART_HandleTypeDef *vision_uart;
static uint8_t vision_dma_buffer[VISION_DMA_BUFFER_SIZE];
static volatile uint8_t vision_ring[VISION_RING_SIZE];
static volatile uint16_t vision_ring_head;
static volatile uint16_t vision_ring_tail;
static volatile uint8_t vision_restart_required;
static uint16_t vision_dma_position;
static char vision_line[VISION_LINE_SIZE];
static uint16_t vision_line_length;
static Vision_BallSample_t vision_latest_sample;

volatile float g_vision_position_debug;
volatile uint8_t g_vision_quality_debug;
volatile uint32_t g_vision_frame_count_debug;
volatile uint32_t g_vision_parse_error_debug;
volatile uint32_t g_vision_rx_overflow_debug;
volatile uint8_t g_vision_dma_status_debug;

static void Vision_RingWrite(const uint8_t *data, uint16_t length)
{
  uint16_t i;

  for (i = 0U; i < length; i++)
  {
    uint16_t next = (uint16_t)((vision_ring_head + 1U) % VISION_RING_SIZE);

    if (next == vision_ring_tail)
    {
      g_vision_rx_overflow_debug++;
      break;
    }

    vision_ring[vision_ring_head] = data[i];
    vision_ring_head = next;
  }
}

static uint8_t Vision_RingRead(uint8_t *data)
{
  if ((data == 0) || (vision_ring_tail == vision_ring_head))
  {
    return 0U;
  }

  *data = vision_ring[vision_ring_tail];
  vision_ring_tail = (uint16_t)((vision_ring_tail + 1U) % VISION_RING_SIZE);
  return 1U;
}

static HAL_StatusTypeDef Vision_StartDma(void)
{
  HAL_StatusTypeDef status;

  if (vision_uart == 0)
  {
    return HAL_ERROR;
  }

  vision_dma_position = 0U;
  status = HAL_UARTEx_ReceiveToIdle_DMA(vision_uart,
                                       vision_dma_buffer,
                                       VISION_DMA_BUFFER_SIZE);
  if ((status == HAL_OK) && (vision_uart->hdmarx != 0))
  {
    __HAL_DMA_DISABLE_IT(vision_uart->hdmarx, DMA_IT_HT);
  }

  g_vision_dma_status_debug = (uint8_t)status;
  return status;
}

static uint8_t Vision_ParseLine(char *line, Vision_BallSample_t *sample)
{
  char *cursor;
  char *end;
  unsigned long value;
  float position;

  if ((line == 0) || (sample == 0) || (strncmp(line, "BALL,", 5U) != 0))
  {
    return 0U;
  }

  cursor = line + 5;
  position = strtof(cursor, &end);
  if ((end == cursor) || (*end != ','))
  {
    return 0U;
  }

  cursor = end + 1;
  value = strtoul(cursor, &end, 10);
  if ((end == cursor) || (*end != ',') || (value > 100UL))
  {
    return 0U;
  }
  sample->quality = (uint8_t)value;

  cursor = end + 1;
  value = strtoul(cursor, &end, 10);
  if (end == cursor)
  {
    return 0U;
  }
  while ((*end == ' ') || (*end == '\r'))
  {
    end++;
  }
  if (*end != '\0')
  {
    return 0U;
  }

  if ((position != position) ||
      (position < -10000.0f) || (position > 10000.0f))
  {
    return 0U;
  }

  sample->position_mm = position;
  sample->source_time_ms = (uint32_t)value;
  sample->receive_cycles = DWT_Time_GetCycles();
  return 1U;
}

static void Vision_PublishSample(Vision_BallSample_t *sample)
{
  sample->sequence = vision_latest_sample.sequence + 1U;
  vision_latest_sample = *sample;

  g_vision_position_debug = sample->position_mm;
  g_vision_quality_debug = sample->quality;
  g_vision_frame_count_debug++;
}

void Vision_Init(UART_HandleTypeDef *huart)
{
  vision_uart = huart;
  vision_ring_head = 0U;
  vision_ring_tail = 0U;
  vision_restart_required = 0U;
  vision_line_length = 0U;
  memset(&vision_latest_sample, 0, sizeof(vision_latest_sample));

  g_vision_position_debug = 0.0f;
  g_vision_quality_debug = 0U;
  g_vision_frame_count_debug = 0U;
  g_vision_parse_error_debug = 0U;
  g_vision_rx_overflow_debug = 0U;
  g_vision_dma_status_debug = (uint8_t)HAL_ERROR;

  (void)Vision_StartDma();
}

void Vision_Task(void)
{
  uint8_t data;

  if (vision_restart_required != 0U)
  {
    vision_restart_required = 0U;
    (void)HAL_UART_AbortReceive(vision_uart);
    (void)Vision_StartDma();
  }

  while (Vision_RingRead(&data) != 0U)
  {
    if (data == '\n')
    {
      Vision_BallSample_t sample;

      vision_line[vision_line_length] = '\0';
      if (Vision_ParseLine(vision_line, &sample) != 0U)
      {
        Vision_PublishSample(&sample);
      }
      else if (vision_line_length > 0U)
      {
        g_vision_parse_error_debug++;
      }
      vision_line_length = 0U;
    }
    else if (vision_line_length < (VISION_LINE_SIZE - 1U))
    {
      vision_line[vision_line_length++] = (char)data;
    }
    else
    {
      vision_line_length = 0U;
      g_vision_parse_error_debug++;
    }
  }
}

void Vision_UART_RxEventCallback(UART_HandleTypeDef *huart, uint16_t size)
{
  if ((huart == 0) || (vision_uart == 0) ||
      (huart->Instance != vision_uart->Instance) ||
      (size > VISION_DMA_BUFFER_SIZE))
  {
    return;
  }

  if (size > vision_dma_position)
  {
    Vision_RingWrite(&vision_dma_buffer[vision_dma_position],
                     (uint16_t)(size - vision_dma_position));
  }
  else if (size < vision_dma_position)
  {
    Vision_RingWrite(&vision_dma_buffer[vision_dma_position],
                     (uint16_t)(VISION_DMA_BUFFER_SIZE - vision_dma_position));
    if (size > 0U)
    {
      Vision_RingWrite(vision_dma_buffer, size);
    }
  }

  vision_dma_position = size;
}

void Vision_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
  if ((huart != 0) && (vision_uart != 0) &&
      (huart->Instance == vision_uart->Instance))
  {
    vision_restart_required = 1U;
  }
}

uint8_t Vision_GetLatest(Vision_BallSample_t *sample, uint32_t *last_sequence)
{
  if ((sample == 0) || (last_sequence == 0) ||
      (vision_latest_sample.sequence == *last_sequence))
  {
    return 0U;
  }

  *sample = vision_latest_sample;
  *last_sequence = sample->sequence;
  return 1U;
}

void Vision_InjectSample(float position_mm, uint8_t quality)
{
  Vision_BallSample_t sample;

  sample.position_mm = position_mm;
  sample.quality = quality;
  sample.source_time_ms = HAL_GetTick();
  sample.receive_cycles = DWT_Time_GetCycles();
  sample.sequence = 0U;
  Vision_PublishSample(&sample);
}
