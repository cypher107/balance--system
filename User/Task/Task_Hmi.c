#include "Task_Hmi.h"

#include "BallBeam.h"
#include "Task2_CarLap.h"
#include "Task3_BallMove.h"
#include "Task4_CarToB.h"
#include "tracking.h"

#define TASK_HMI_FRAME_SIZE 8U
#define TASK_HMI_HEADER_0   0x01U
#define TASK_HMI_HEADER_1   0xFFU
#define TASK_HMI_TAIL       0x0AU
#define TASK_HMI_CMD_BALL_SET      0x18U
#define TASK_HMI_CMD_TRACK_SET     0x19U
#define TASK_HMI_CMD_BALL_RESET    0x1AU
#define TASK_HMI_CMD_TRACK_RESET   0x1BU
#define TASK_HMI_BALL_DEFAULT_P    45U
#define TASK_HMI_BALL_DEFAULT_D    43U
#define TASK_HMI_BALL_DEFAULT_ANG  38U
#define TASK_HMI_TRACK_DEFAULT_P   18U
#define TASK_HMI_TRACK_DEFAULT_DELAY_10MS 80U
#define TASK_HMI_MAX_ANGLE_PULSE   250U

static UART_HandleTypeDef *task_hmi_uart;
static volatile uint8_t task_hmi_rx_byte;
static volatile uint8_t task_hmi_pending_command;
static volatile uint8_t task_hmi_pending_parameter_command;
static volatile uint8_t task_hmi_pending_parameter_data[3];
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
volatile uint8_t g_task_hmi_parameter_command_debug;
volatile uint8_t g_task_hmi_ball_p_debug;
volatile uint8_t g_task_hmi_ball_d_debug;
volatile uint8_t g_task_hmi_ball_angle_debug;
volatile uint8_t g_task_hmi_tracking_p_debug;
volatile uint8_t g_task_hmi_tracking_delay_10ms_debug;
volatile uint32_t g_task_hmi_parameter_count_debug;

static void TaskHmi_SetBallParameters(uint8_t p,
                                      uint8_t d,
                                      uint8_t angle)
{
  int32_t angle_pulse = (int32_t)angle;

  if (angle_pulse > (int32_t)TASK_HMI_MAX_ANGLE_PULSE)
  {
    angle_pulse = (int32_t)TASK_HMI_MAX_ANGLE_PULSE;
  }

  g_task3_return_kp_debug = (float)p;
  g_task3_return_kd_debug = (float)d;
  g_task3_open_loop_pulse_debug = angle_pulse;

  g_task4_kp_debug = (float)p;
  g_task4_kd_debug = (float)d;
  g_task4_start_tilt_pulse_debug = angle_pulse;
  g_task4_stop_tilt_pulse_debug = -angle_pulse;

  g_ballbeam_kp_debug = (float)p;
  g_ballbeam_kd_debug = (float)d;

  g_task_hmi_ball_p_debug = p;
  g_task_hmi_ball_d_debug = d;
  g_task_hmi_ball_angle_debug = (uint8_t)angle_pulse;
}

static void TaskHmi_SetTrackingParameters(uint8_t p, uint8_t delay_10ms)
{
  g_tracking_kp_debug = (float)p;
  g_task4_tracking_kp_debug = (float)p;
  g_task2_finish_confirm_s_debug = (float)delay_10ms * 0.01f;

  g_task_hmi_tracking_p_debug = p;
  g_task_hmi_tracking_delay_10ms_debug = delay_10ms;
}

static void TaskHmi_ApplyParameterCommand(uint8_t command,
                                          uint8_t data0,
                                          uint8_t data1,
                                          uint8_t data2)
{
  if (command == TASK_HMI_CMD_BALL_SET)
  {
    TaskHmi_SetBallParameters(data0, data1, data2);
  }
  else if (command == TASK_HMI_CMD_TRACK_SET)
  {
    TaskHmi_SetTrackingParameters(data0, data1);
  }
  else if (command == TASK_HMI_CMD_BALL_RESET)
  {
    TaskHmi_SetBallParameters(TASK_HMI_BALL_DEFAULT_P,
                              TASK_HMI_BALL_DEFAULT_D,
                              TASK_HMI_BALL_DEFAULT_ANG);
  }
  else if (command == TASK_HMI_CMD_TRACK_RESET)
  {
    TaskHmi_SetTrackingParameters(TASK_HMI_TRACK_DEFAULT_P,
                                  TASK_HMI_TRACK_DEFAULT_DELAY_10MS);
  }
  else
  {
    return;
  }

  g_task_hmi_parameter_command_debug = command;
  g_task_hmi_parameter_count_debug++;
}

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
    else if ((command >= TASK_HMI_CMD_BALL_SET) &&
             (command <= TASK_HMI_CMD_TRACK_RESET))
    {
      task_hmi_pending_parameter_data[0] = task_hmi_frame[3];
      task_hmi_pending_parameter_data[1] = task_hmi_frame[4];
      task_hmi_pending_parameter_data[2] = task_hmi_frame[5];
      task_hmi_pending_parameter_command = command;
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
  task_hmi_pending_parameter_command = 0U;
  task_hmi_pending_parameter_data[0] = 0U;
  task_hmi_pending_parameter_data[1] = 0U;
  task_hmi_pending_parameter_data[2] = 0U;
  task_hmi_restart_required = 0U;
  TaskHmi_ResetParser();
  g_task_hmi_rx_byte_debug = 0U;
  g_task_hmi_command_debug = 0U;
  g_task_hmi_rx_count_debug = 0U;
  g_task_hmi_command_count_debug = 0U;
  g_task_hmi_rx_status_debug = (uint8_t)HAL_ERROR;
  g_task_hmi_frame_count_debug = 0U;
  g_task_hmi_invalid_frame_count_debug = 0U;
  g_task_hmi_parameter_command_debug = 0U;
  g_task_hmi_ball_p_debug = 0U;
  g_task_hmi_ball_d_debug = 0U;
  g_task_hmi_ball_angle_debug = 0U;
  g_task_hmi_tracking_p_debug = 0U;
  g_task_hmi_tracking_delay_10ms_debug = 0U;
  g_task_hmi_parameter_count_debug = 0U;
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
  uint32_t primask;
  uint8_t parameter_command;
  uint8_t parameter_data[3];

  if (task_hmi_restart_required != 0U)
  {
    task_hmi_restart_required = 0U;
    (void)HAL_UART_AbortReceive(task_hmi_uart);
    TaskHmi_ResetParser();
    (void)TaskHmi_StartReceive();
  }

  primask = __get_PRIMASK();
  __disable_irq();
  parameter_command = task_hmi_pending_parameter_command;
  parameter_data[0] = task_hmi_pending_parameter_data[0];
  parameter_data[1] = task_hmi_pending_parameter_data[1];
  parameter_data[2] = task_hmi_pending_parameter_data[2];
  task_hmi_pending_parameter_command = 0U;
  if (primask == 0U)
  {
    __enable_irq();
  }

  if (parameter_command != 0U)
  {
    TaskHmi_ApplyParameterCommand(parameter_command,
                                  parameter_data[0],
                                  parameter_data[1],
                                  parameter_data[2]);
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
