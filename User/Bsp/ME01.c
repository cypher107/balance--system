#include "ME01.h"

#include <stdlib.h>
#include <string.h>

#include "dwt_time.h"

#define ME01_DMA_BUFFER_SIZE 128U
#define ME01_RING_SIZE       256U
#define ME01_LINE_SIZE       48U

static UART_HandleTypeDef *me01_uart;
static uint8_t me01_dma_buffer[ME01_DMA_BUFFER_SIZE];
static volatile uint8_t me01_ring[ME01_RING_SIZE];
static volatile uint16_t me01_ring_head;
static volatile uint16_t me01_ring_tail;
static volatile uint8_t me01_restart_required;
static uint16_t me01_dma_position;
static char me01_line[ME01_LINE_SIZE];
static uint16_t me01_line_length;
static ME01_AngleSample_t me01_latest_sample;

volatile float g_me01_raw_angle_debug;
volatile uint32_t g_me01_frame_count_debug;
volatile uint32_t g_me01_parse_error_debug;
volatile uint32_t g_me01_rx_overflow_debug;
volatile uint8_t g_me01_dma_status_debug;
volatile uint8_t g_me01_config_status_debug;

static float ME01_Normalize360(float angle_deg)
{
  while (angle_deg >= 360.0f)
  {
    angle_deg -= 360.0f;
  }
  while (angle_deg < 0.0f)
  {
    angle_deg += 360.0f;
  }
  return angle_deg;
}

static void ME01_RingWrite(const uint8_t *data, uint16_t length)
{
  uint16_t i;

  for (i = 0U; i < length; i++)
  {
    uint16_t next = (uint16_t)((me01_ring_head + 1U) % ME01_RING_SIZE);

    if (next == me01_ring_tail)
    {
      g_me01_rx_overflow_debug++;
      break;
    }
    me01_ring[me01_ring_head] = data[i];
    me01_ring_head = next;
  }
}

static uint8_t ME01_RingRead(uint8_t *data)
{
  if ((data == 0) || (me01_ring_tail == me01_ring_head))
  {
    return 0U;
  }

  *data = me01_ring[me01_ring_tail];
  me01_ring_tail = (uint16_t)((me01_ring_tail + 1U) % ME01_RING_SIZE);
  return 1U;
}

static HAL_StatusTypeDef ME01_StartDma(void)
{
  HAL_StatusTypeDef status;

  if (me01_uart == 0)
  {
    return HAL_ERROR;
  }

  me01_dma_position = 0U;
  status = HAL_UARTEx_ReceiveToIdle_DMA(me01_uart,
                                       me01_dma_buffer,
                                       ME01_DMA_BUFFER_SIZE);
  if ((status == HAL_OK) && (me01_uart->hdmarx != 0))
  {
    __HAL_DMA_DISABLE_IT(me01_uart->hdmarx, DMA_IT_HT);
  }
  g_me01_dma_status_debug = (uint8_t)status;
  return status;
}

static HAL_StatusTypeDef ME01_SendCommand(const char *command)
{
  if ((me01_uart == 0) || (command == 0))
  {
    return HAL_ERROR;
  }

  return HAL_UART_Transmit(me01_uart,
                           (uint8_t *)command,
                           (uint16_t)strlen(command),
                           100U);
}

static uint8_t ME01_ParseLine(char *line, float *angle_deg)
{
  char *yaw;
  char *end;
  float value;

  if ((line == 0) || (angle_deg == 0))
  {
    return 0U;
  }

  yaw = strstr(line, "Yaw:");
  if (yaw == 0)
  {
    return 0U;
  }

  value = strtof(yaw + 4, &end);
  if ((end == (yaw + 4)) || (value != value) ||
      (value < -720.0f) || (value > 720.0f))
  {
    return 0U;
  }

  *angle_deg = ME01_Normalize360(value);
  return 1U;
}

static void ME01_Publish(float raw_angle_deg)
{
  me01_latest_sample.raw_angle_deg = ME01_Normalize360(raw_angle_deg);
  me01_latest_sample.receive_cycles = DWT_Time_GetCycles();
  me01_latest_sample.sequence++;

  g_me01_raw_angle_debug = me01_latest_sample.raw_angle_deg;
  g_me01_frame_count_debug++;
}

void ME01_Init(UART_HandleTypeDef *huart)
{
  HAL_StatusTypeDef status;

  me01_uart = huart;
  me01_ring_head = 0U;
  me01_ring_tail = 0U;
  me01_restart_required = 0U;
  me01_line_length = 0U;
  memset(&me01_latest_sample, 0, sizeof(me01_latest_sample));

  g_me01_raw_angle_debug = 0.0f;
  g_me01_frame_count_debug = 0U;
  g_me01_parse_error_debug = 0U;
  g_me01_rx_overflow_debug = 0U;
  g_me01_dma_status_debug = (uint8_t)HAL_ERROR;
  g_me01_config_status_debug = (uint8_t)HAL_ERROR;

  status = ME01_StartDma();
  if (status != HAL_OK)
  {
    return;
  }

  HAL_Delay(100U);
  status = ME01_SendCommand("AT+MODE=0\n");
  HAL_Delay(20U);
  if (status == HAL_OK)
  {
    status = ME01_SendCommand("AT+PRATE=20\n");
  }
  g_me01_config_status_debug = (uint8_t)status;
}

void ME01_Task(void)
{
  uint8_t data;

  if (me01_restart_required != 0U)
  {
    me01_restart_required = 0U;
    (void)HAL_UART_AbortReceive(me01_uart);
    (void)ME01_StartDma();
  }

  while (ME01_RingRead(&data) != 0U)
  {
    if (data == '\n')
    {
      float angle_deg;

      me01_line[me01_line_length] = '\0';
      if (ME01_ParseLine(me01_line, &angle_deg) != 0U)
      {
        ME01_Publish(angle_deg);
      }
      else if ((me01_line_length > 0U) &&
               (strstr(me01_line, "OK") == 0))
      {
        g_me01_parse_error_debug++;
      }
      me01_line_length = 0U;
    }
    else if (data != '\r')
    {
      if (me01_line_length < (ME01_LINE_SIZE - 1U))
      {
        me01_line[me01_line_length++] = (char)data;
      }
      else
      {
        me01_line_length = 0U;
        g_me01_parse_error_debug++;
      }
    }
  }
}

void ME01_UART_RxEventCallback(UART_HandleTypeDef *huart, uint16_t size)
{
  if ((huart == 0) || (me01_uart == 0) ||
      (huart->Instance != me01_uart->Instance) ||
      (size > ME01_DMA_BUFFER_SIZE))
  {
    return;
  }

  if (size > me01_dma_position)
  {
    ME01_RingWrite(&me01_dma_buffer[me01_dma_position],
                   (uint16_t)(size - me01_dma_position));
  }
  else if (size < me01_dma_position)
  {
    ME01_RingWrite(&me01_dma_buffer[me01_dma_position],
                   (uint16_t)(ME01_DMA_BUFFER_SIZE - me01_dma_position));
    if (size > 0U)
    {
      ME01_RingWrite(me01_dma_buffer, size);
    }
  }
  me01_dma_position = size;
}

void ME01_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
  if ((huart != 0) && (me01_uart != 0) &&
      (huart->Instance == me01_uart->Instance))
  {
    me01_restart_required = 1U;
  }
}

uint8_t ME01_GetLatest(ME01_AngleSample_t *sample, uint32_t *last_sequence)
{
  if ((sample == 0) || (last_sequence == 0) ||
      (me01_latest_sample.sequence == *last_sequence))
  {
    return 0U;
  }

  *sample = me01_latest_sample;
  *last_sequence = sample->sequence;
  return 1U;
}

void ME01_InjectAngle(float raw_angle_deg)
{
  ME01_Publish(raw_angle_deg);
}
