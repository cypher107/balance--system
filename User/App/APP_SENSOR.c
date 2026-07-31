#include "APP_SENSOR.h"

#include "BSP_CRC.h"

#define SENSOR_HEADER_0                0x77U
#define SENSOR_HEADER_1                0x88U
#define SENSOR_REQUEST_BYTE            0x61U
#define SENSOR_CRC_DATA_SIZE           14U
#define SENSOR_REQUEST_PERIOD_MS       5U
#define SENSOR_RESPONSE_TIMEOUT_MS     20U
#define SENSOR_DEFAULT_VALID_TIMEOUT_MS 100U

static UART_HandleTypeDef *sensor_uart;
static volatile uint8_t sensor_rx_byte;
static volatile uint8_t sensor_rx_frame[APP_SENSOR_FRAME_SIZE];
static volatile uint8_t sensor_rx_index;
static volatile uint8_t sensor_frame_ready;
static volatile uint8_t sensor_request_pending;
static volatile uint8_t sensor_restart_required;
static uint32_t sensor_last_request_ms;
static uint32_t sensor_last_valid_ms;
static uint8_t sensor_has_valid_data;

volatile uint8_t g_grayscale_values_debug[SENSOR_TOTAL_NUM];

volatile uint8_t g_sensor_rx_byte_debug;
volatile uint8_t g_sensor_last_frame_debug[APP_SENSOR_FRAME_SIZE];
volatile uint16_t g_sensor_calc_crc_debug;
volatile uint16_t g_sensor_recv_crc_debug;
volatile uint8_t g_sensor_valid_debug;
volatile uint8_t g_sensor_rx_status_debug;
volatile uint32_t g_sensor_rx_count_debug;
volatile uint32_t g_sensor_frame_count_debug;
volatile uint32_t g_sensor_valid_frame_count_debug;
volatile uint32_t g_sensor_crc_error_count_debug;
volatile uint32_t g_sensor_request_count_debug;
volatile uint32_t g_sensor_timeout_count_debug;
volatile uint32_t g_sensor_uart_error_count_debug;

static HAL_StatusTypeDef APP_SENSOR_StartReceive(void)
{
  HAL_StatusTypeDef status;

  if (sensor_uart == 0)
  {
    return HAL_ERROR;
  }

  status = HAL_UART_Receive_IT(sensor_uart,
                              (uint8_t *)&sensor_rx_byte,
                              1U);
  g_sensor_rx_status_debug = (uint8_t)status;
  return status;
}

static void APP_SENSOR_ResetParser(void)
{
  sensor_rx_index = 0U;
}

static void APP_SENSOR_ParseByte(uint8_t data)
{
  if (sensor_frame_ready != 0U)
  {
    return;
  }

  if (sensor_rx_index == 0U)
  {
    if (data == SENSOR_HEADER_0)
    {
      sensor_rx_frame[0] = data;
      sensor_rx_index = 1U;
    }
    return;
  }

  if (sensor_rx_index == 1U)
  {
    if (data == SENSOR_HEADER_1)
    {
      sensor_rx_frame[1] = data;
      sensor_rx_index = 2U;
    }
    else if (data == SENSOR_HEADER_0)
    {
      sensor_rx_frame[0] = data;
      sensor_rx_index = 1U;
    }
    else
    {
      APP_SENSOR_ResetParser();
    }
    return;
  }

  sensor_rx_frame[sensor_rx_index] = data;
  sensor_rx_index++;
  if (sensor_rx_index >= APP_SENSOR_FRAME_SIZE)
  {
    APP_SENSOR_ResetParser();
    sensor_request_pending = 0U;
    sensor_frame_ready = 1U;
    g_sensor_frame_count_debug++;
  }
}

static void APP_SENSOR_ProcessFrame(uint32_t now_ms)
{
  uint8_t packet[APP_SENSOR_FRAME_SIZE];
  uint16_t calc_crc;
  uint16_t recv_crc;
  uint32_t primask;
  uint8_t index;

  if (sensor_frame_ready == 0U)
  {
    return;
  }

  primask = __get_PRIMASK();
  __disable_irq();
  for (index = 0U; index < APP_SENSOR_FRAME_SIZE; index++)
  {
    packet[index] = sensor_rx_frame[index];
  }
  sensor_frame_ready = 0U;
  if (primask == 0U)
  {
    __enable_irq();
  }

  for (index = 0U; index < APP_SENSOR_FRAME_SIZE; index++)
  {
    g_sensor_last_frame_debug[index] = packet[index];
  }

  calc_crc = GetCRC16(packet, SENSOR_CRC_DATA_SIZE);
  recv_crc = (uint16_t)(((uint16_t)packet[14] << 8U) | packet[15]);
  g_sensor_calc_crc_debug = calc_crc;
  g_sensor_recv_crc_debug = recv_crc;

  if ((packet[0] != SENSOR_HEADER_0) ||
      (packet[1] != SENSOR_HEADER_1) ||
      (calc_crc != recv_crc))
  {
    g_sensor_crc_error_count_debug++;
    return;
  }

  for (index = 0U; index < SENSOR_TOTAL_NUM; index++)
  {
    g_grayscale_values_debug[index] = packet[index + 2U];
  }
  sensor_last_valid_ms = now_ms;
  sensor_has_valid_data = 1U;
  g_sensor_valid_debug = 1U;
  g_sensor_valid_frame_count_debug++;
}

void APP_SENSOR_Init(UART_HandleTypeDef *huart)
{
  uint8_t index;

  sensor_uart = huart;
  sensor_rx_byte = 0U;
  sensor_frame_ready = 0U;
  sensor_request_pending = 0U;
  sensor_restart_required = 0U;
  sensor_last_request_ms = HAL_GetTick() - SENSOR_REQUEST_PERIOD_MS;
  sensor_last_valid_ms = 0U;
  sensor_has_valid_data = 0U;
  APP_SENSOR_ResetParser();

  g_sensor_rx_byte_debug = 0U;
  g_sensor_calc_crc_debug = 0U;
  g_sensor_recv_crc_debug = 0U;
  g_sensor_valid_debug = 0U;
  g_sensor_rx_status_debug = (uint8_t)HAL_ERROR;
  g_sensor_rx_count_debug = 0U;
  g_sensor_frame_count_debug = 0U;
  g_sensor_valid_frame_count_debug = 0U;
  g_sensor_crc_error_count_debug = 0U;
  g_sensor_request_count_debug = 0U;
  g_sensor_timeout_count_debug = 0U;
  g_sensor_uart_error_count_debug = 0U;

  for (index = 0U; index < SENSOR_TOTAL_NUM; index++)
  {
    g_grayscale_values_debug[index] = 0U;
  }
  for (index = 0U; index < APP_SENSOR_FRAME_SIZE; index++)
  {
    sensor_rx_frame[index] = 0U;
    g_sensor_last_frame_debug[index] = 0U;
  }

  if (APP_SENSOR_StartReceive() != HAL_OK)
  {
    sensor_restart_required = 1U;
  }
}

void APP_SENSOR_Task(void)
{
  uint32_t now_ms;
  uint32_t primask;
  uint8_t request = SENSOR_REQUEST_BYTE;

  if (sensor_uart == 0)
  {
    return;
  }

  if (sensor_restart_required != 0U)
  {
    sensor_restart_required = 0U;
    (void)HAL_UART_AbortReceive(sensor_uart);
    APP_SENSOR_ResetParser();
    sensor_frame_ready = 0U;
    sensor_request_pending = 0U;
    if (APP_SENSOR_StartReceive() != HAL_OK)
    {
      sensor_restart_required = 1U;
    }
  }

  now_ms = HAL_GetTick();
  APP_SENSOR_ProcessFrame(now_ms);

  if ((sensor_has_valid_data != 0U) &&
      ((now_ms - sensor_last_valid_ms) > SENSOR_DEFAULT_VALID_TIMEOUT_MS))
  {
    g_sensor_valid_debug = 0U;
  }

  if ((sensor_request_pending != 0U) &&
      ((now_ms - sensor_last_request_ms) > SENSOR_RESPONSE_TIMEOUT_MS))
  {
    primask = __get_PRIMASK();
    __disable_irq();
    sensor_request_pending = 0U;
    APP_SENSOR_ResetParser();
    if (primask == 0U)
    {
      __enable_irq();
    }
    g_sensor_timeout_count_debug++;
  }

  if ((sensor_request_pending == 0U) &&
      (sensor_frame_ready == 0U) &&
      ((now_ms - sensor_last_request_ms) >= SENSOR_REQUEST_PERIOD_MS))
  {
    primask = __get_PRIMASK();
    __disable_irq();
    APP_SENSOR_ResetParser();
    sensor_request_pending = 1U;
    if (primask == 0U)
    {
      __enable_irq();
    }

    sensor_last_request_ms = now_ms;
    if (HAL_UART_Transmit(sensor_uart, &request, 1U, 2U) == HAL_OK)
    {
      g_sensor_request_count_debug++;
    }
    else
    {
      sensor_request_pending = 0U;
    }
  }
}

void Uart_Data_Process(void)
{
  APP_SENSOR_Task();
}

void APP_SENSOR_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
  uint8_t data;

  if ((huart == 0) || (sensor_uart == 0) ||
      (huart->Instance != sensor_uart->Instance))
  {
    return;
  }

  data = sensor_rx_byte;
  g_sensor_rx_byte_debug = data;
  g_sensor_rx_count_debug++;
  APP_SENSOR_ParseByte(data);

  if (APP_SENSOR_StartReceive() != HAL_OK)
  {
    sensor_restart_required = 1U;
  }
}

void APP_SENSOR_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
  if ((huart != 0) && (sensor_uart != 0) &&
      (huart->Instance == sensor_uart->Instance))
  {
    g_sensor_uart_error_count_debug++;
    sensor_restart_required = 1U;
  }
}

uint8_t APP_SENSOR_IsValid(uint32_t timeout_ms)
{
  if (sensor_has_valid_data == 0U)
  {
    return 0U;
  }

  return ((HAL_GetTick() - sensor_last_valid_ms) <= timeout_ms) ? 1U : 0U;
}
