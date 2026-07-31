#include "Task_Hmi.h"

#define TASK_HMI_FRAME_SIZE 8U
#define TASK_HMI_HEADER_0   0x01U
#define TASK_HMI_HEADER_1   0xFFU
#define TASK_HMI_TAIL       0x0AU

static UART_HandleTypeDef *task_hmi_uart;
static volatile uint8_t task_hmi_rx_byte;
static volatile uint8_t task_hmi_pending_command;
static volatile uint8_t task_hmi_restart_required;
static uint8_t task_hmi_frame[TASK_HMI_FRAME_SIZE];
static uint8_t task_hmi_frame_index;

volatile uint8_t g_task_hmi_rx_byte_debug;
volatile uint8_t g_task_hmi_command_debug;
volatile uint32_t g_task_hmi_rx_count_debug;
volatile uint32_t g_task_hmi_command_count_debug;
volatile uint8_t g_task_hmi_rx_status_debug;
volatile uint8_t g_task_hmi_frame_debug[TASK_HMI_FRAME_SIZE];
volatile uint8_t g_task_hmi_frame_index_debug;
volatile uint32_t g_task_hmi_frame_count_debug;
volatile uint32_t g_task_hmi_invalid_frame_count_debug;

static void TaskHmi_ResetParser(void)
{
  task_hmi_frame_index = 0U;
  g_task_hmi_frame_index_debug = 0U;
}

static void TaskHmi_ParseByte(uint8_t data)
{
  uint8_t index;
  uint8_t command;

  if (task_hmi_frame_index == 0U)
  {
    if (data == TASK_HMI_HEADER_0)
    {
      task_hmi_frame[0] = data;
      task_hmi_frame_index = 1U;
    }
    g_task_hmi_frame_index_debug = task_hmi_frame_index;
    return;
  }

  if (task_hmi_frame_index == 1U)
  {
    if (data == TASK_HMI_HEADER_1)
    {
      task_hmi_frame[1] = data;
      task_hmi_frame_index = 2U;
    }
    else if (data == TASK_HMI_HEADER_0)
    {
      task_hmi_frame[0] = data;
      task_hmi_frame_index = 1U;
    }
    else
    {
      task_hmi_frame_index = 0U;
    }
    g_task_hmi_frame_index_debug = task_hmi_frame_index;
    return;
  }

  task_hmi_frame[task_hmi_frame_index] = data;
  task_hmi_frame_index++;
  g_task_hmi_frame_index_debug = task_hmi_frame_index;
  if (task_hmi_frame_index < TASK_HMI_FRAME_SIZE)
  {
    return;
  }

  if (task_hmi_frame[7] == TASK_HMI_TAIL)
  {
    for (index = 0U; index < TASK_HMI_FRAME_SIZE; index++)
    {
      g_task_hmi_frame_debug[index] = task_hmi_frame[index];
    }
    g_task_hmi_frame_count_debug++;

    command = task_hmi_frame[2];
    if ((command >= 0x02U) && (command <= 0x06U))
    {
      task_hmi_pending_command = command;
      g_task_hmi_command_debug = command;
      g_task_hmi_command_count_debug++;
    }
  }
  else
  {
    g_task_hmi_invalid_frame_count_debug++;
  }

  TaskHmi_ResetParser();
}

static HAL_StatusTypeDef TaskHmi_StartReceive(void)
{
  HAL_StatusTypeDef status;

  if (task_hmi_uart == 0)
  {
    return HAL_ERROR;
  }

  status = HAL_UART_Receive_IT(task_hmi_uart,
                              (uint8_t *)&task_hmi_rx_byte,
                              1U);
  g_task_hmi_rx_status_debug = (uint8_t)status;
  return status;
}

void TaskHmi_Init(UART_HandleTypeDef *huart)
{
  task_hmi_uart = huart;
  task_hmi_rx_byte = 0U;
  task_hmi_pending_command = 0U;
  task_hmi_restart_required = 0U;
  TaskHmi_ResetParser();
  g_task_hmi_rx_byte_debug = 0U;
  g_task_hmi_command_debug = 0U;
  g_task_hmi_rx_count_debug = 0U;
  g_task_hmi_command_count_debug = 0U;
  g_task_hmi_rx_status_debug = (uint8_t)HAL_ERROR;
  g_task_hmi_frame_count_debug = 0U;
  g_task_hmi_invalid_frame_count_debug = 0U;
  for (task_hmi_frame_index = 0U;
       task_hmi_frame_index < TASK_HMI_FRAME_SIZE;
       task_hmi_frame_index++)
  {
    task_hmi_frame[task_hmi_frame_index] = 0U;
    g_task_hmi_frame_debug[task_hmi_frame_index] = 0U;
  }
  TaskHmi_ResetParser();
  (void)TaskHmi_StartReceive();
}

void TaskHmi_Task(void)
{
  if (task_hmi_restart_required != 0U)
  {
    task_hmi_restart_required = 0U;
    (void)HAL_UART_AbortReceive(task_hmi_uart);
    TaskHmi_ResetParser();
    (void)TaskHmi_StartReceive();
  }
}

void TaskHmi_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
  uint8_t data;

  if ((huart == 0) || (task_hmi_uart == 0) ||
      (huart->Instance != task_hmi_uart->Instance))
  {
    return;
  }

  data = task_hmi_rx_byte;
  g_task_hmi_rx_byte_debug = data;
  g_task_hmi_rx_count_debug++;
  TaskHmi_ParseByte(data);

  if (TaskHmi_StartReceive() != HAL_OK)
  {
    task_hmi_restart_required = 1U;
  }
}

void TaskHmi_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
  if ((huart != 0) && (task_hmi_uart != 0) &&
      (huart->Instance == task_hmi_uart->Instance))
  {
    task_hmi_restart_required = 1U;
  }
}

uint8_t TaskHmi_GetCommand(uint8_t *task_id)
{
  uint32_t primask;

  if (task_id == 0)
  {
    return 0U;
  }

  primask = __get_PRIMASK();
  __disable_irq();
  if (task_hmi_pending_command == 0U)
  {
    if (primask == 0U) __enable_irq();
    return 0U;
  }

  *task_id = task_hmi_pending_command;
  task_hmi_pending_command = 0U;
  if (primask == 0U) __enable_irq();
  return 1U;
}
