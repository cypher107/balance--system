#include "Vision.h"

#include <string.h>

#include "dwt_time.h"

#define VISION_RX_BUFFER_SIZE  64U
#define VISION_RING_SIZE       128U
#define VISION_FRAME_SIZE      4U
#define VISION_FRAME_HEADER    0xAAU
#define VISION_FRAME_TAIL      0x55U
#define VISION_POSITION_SCALE  0.01f

static UART_HandleTypeDef *vision_uart;
static uint8_t vision_rx_buffer[VISION_RX_BUFFER_SIZE];
static volatile uint8_t vision_ring[VISION_RING_SIZE];
static volatile uint16_t vision_ring_head;
static volatile uint16_t vision_ring_tail;
static volatile uint8_t vision_restart_required;
static uint8_t vision_frame[VISION_FRAME_SIZE];
static uint8_t vision_frame_index;
static Vision_BallSample_t vision_latest_sample;

volatile uint16_t g_vision_position_raw_debug;
volatile float g_vision_position_cm_debug;
volatile float g_vision_position_debug;
volatile uint32_t g_vision_frame_count_debug;
volatile uint32_t g_vision_parse_error_debug;
volatile uint32_t g_vision_rx_overflow_debug;
volatile uint8_t g_vision_rx_status_debug;
volatile uint8_t g_vision_dma_status_debug;
volatile uint8_t g_vision_rx_byte_debug;

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

static HAL_StatusTypeDef Vision_StartReceive(void)
{
  HAL_StatusTypeDef status;

  if (vision_uart == 0)
  {
    return HAL_ERROR;
  }

  status = HAL_UARTEx_ReceiveToIdle_IT(vision_uart,
                                      vision_rx_buffer,
                                      VISION_RX_BUFFER_SIZE);
  g_vision_rx_status_debug = (uint8_t)status;
  g_vision_dma_status_debug = (uint8_t)status;
  return status;
}

static void Vision_PublishSample(uint16_t position_raw)
{
  Vision_BallSample_t sample;

  sample.position_raw = position_raw;
  sample.position_cm = (float)position_raw * VISION_POSITION_SCALE;
  sample.receive_cycles = DWT_Time_GetCycles();
  sample.sequence = vision_latest_sample.sequence + 1U;
  vision_latest_sample = sample;

  g_vision_position_raw_debug = position_raw;
  g_vision_position_cm_debug = sample.position_cm;
  g_vision_position_debug = sample.position_cm;
  g_vision_frame_count_debug++;
}

static void Vision_ParseByte(uint8_t data)
{
  if (vision_frame_index == 0U)
  {
    if (data == VISION_FRAME_HEADER)
    {
      vision_frame[0] = data;
      vision_frame_index = 1U;
    }
    return;
  }

  vision_frame[vision_frame_index++] = data;
  if (vision_frame_index < VISION_FRAME_SIZE)
  {
    return;
  }

  if (vision_frame[3] == VISION_FRAME_TAIL)
  {
    uint16_t position_raw = (uint16_t)vision_frame[1] |
                            ((uint16_t)vision_frame[2] << 8U);
    Vision_PublishSample(position_raw);
    vision_frame_index = 0U;
  }
  else
  {
    g_vision_parse_error_debug++;
    vision_frame_index = (data == VISION_FRAME_HEADER) ? 1U : 0U;
    if (vision_frame_index != 0U)
    {
      vision_frame[0] = data;
    }
  }
}

void Vision_Init(UART_HandleTypeDef *huart)
{
  vision_uart = huart;
  vision_ring_head = 0U;
  vision_ring_tail = 0U;
  vision_restart_required = 0U;
  vision_frame_index = 0U;
  memset(&vision_latest_sample, 0, sizeof(vision_latest_sample));

  g_vision_position_raw_debug = 0U;
  g_vision_position_cm_debug = 0.0f;
  g_vision_position_debug = 0.0f;
  g_vision_frame_count_debug = 0U;
  g_vision_parse_error_debug = 0U;
  g_vision_rx_overflow_debug = 0U;
  g_vision_rx_status_debug = (uint8_t)HAL_ERROR;
  g_vision_dma_status_debug = (uint8_t)HAL_ERROR;
  g_vision_rx_byte_debug = 0U;

  (void)Vision_StartReceive();
}

void Vision_Task(void)
{
  uint8_t data;

  if (vision_restart_required != 0U)
  {
    vision_restart_required = 0U;
    (void)HAL_UART_AbortReceive(vision_uart);
    (void)Vision_StartReceive();
  }

  while (Vision_RingRead(&data) != 0U)
  {
    Vision_ParseByte(data);
  }
}

void Vision_UART_RxEventCallback(UART_HandleTypeDef *huart, uint16_t size)
{
  if ((huart == 0) || (vision_uart == 0) ||
      (huart->Instance != vision_uart->Instance) ||
      (size > VISION_RX_BUFFER_SIZE))
  {
    return;
  }

  if (size > 0U)
  {
    Vision_RingWrite(vision_rx_buffer, size);
    g_vision_rx_byte_debug = vision_rx_buffer[size - 1U];
  }

  if (Vision_StartReceive() != HAL_OK)
  {
    vision_restart_required = 1U;
  }
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
  uint32_t primask;
  uint32_t sequence;

  if ((sample == 0) || (last_sequence == 0))
  {
    return 0U;
  }

  primask = __get_PRIMASK();
  __disable_irq();
  sequence = vision_latest_sample.sequence;
  if (sequence == *last_sequence)
  {
    if (primask == 0U) __enable_irq();
    return 0U;
  }

  *sample = vision_latest_sample;
  *last_sequence = sequence;
  if (primask == 0U) __enable_irq();
  return 1U;
}

void Vision_InjectSample(float position_cm)
{
  uint32_t position_raw;

  if (position_cm < 0.0f)
  {
    position_cm = 0.0f;
  }
  else if (position_cm > 655.35f)
  {
    position_cm = 655.35f;
  }

  position_raw = (uint32_t)(position_cm * 100.0f + 0.5f);
  Vision_PublishSample((uint16_t)position_raw);
}
