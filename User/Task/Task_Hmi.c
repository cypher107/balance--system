#include "Task_Hmi.h"

#include "BallBeam.h"
#include "Emm_V5.h"
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
#define TASK_HMI_CMD_ZERO_PLUS     0x07U
#define TASK_HMI_CMD_ZERO_MINUS    0x08U
#define TASK_HMI_CMD_ZERO_CONFIRM  0x09U
#define TASK_HMI_BALL_DEFAULT_P    45U
#define TASK_HMI_BALL_DEFAULT_D    43U
#define TASK_HMI_BALL_DEFAULT_TIME_10MS 55U
#define TASK_HMI_TRACK_DEFAULT_P   18U
#define TASK_HMI_TRACK_DEFAULT_DELAY_10MS 80U
#define TASK_HMI_ZERO_STEP_PULSE   5
#define TASK_HMI_ZERO_MOVE_SPEED   60U
#define TASK_HMI_ZERO_MOVE_ACCEL   80U
#define TASK_HMI_ZERO_SAVE_DELAY_MS 100U
#define TASK_HMI_HOME_MODE         0U
#define TASK_HMI_HOME_DIRECTION    0U
#define TASK_HMI_HOME_SPEED        30U
#define TASK_HMI_HOME_TIMEOUT_MS   10000U
#define TASK_HMI_HOME_SENSOR_SPEED 300U
#define TASK_HMI_HOME_SENSOR_MA    800U
#define TASK_HMI_HOME_SENSOR_MS    60U

static UART_HandleTypeDef *task_hmi_uart;
static volatile uint8_t task_hmi_rx_byte;
static volatile uint8_t task_hmi_pending_command;
static volatile uint8_t task_hmi_pending_motor_command;
static volatile uint8_t task_hmi_pending_parameter_command;
static volatile uint8_t task_hmi_pending_parameter_data[3];
static volatile uint8_t task_hmi_restart_required;
static uint8_t task_hmi_frame[TASK_HMI_FRAME_SIZE];
static uint8_t task_hmi_frame_index;
static uint8_t task_hmi_zero_save_state;
static uint32_t task_hmi_zero_save_tick_ms;
static int32_t task_hmi_zero_target_pulse;

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
volatile uint8_t g_task_hmi_ball_time_10ms_debug;
volatile uint8_t g_task_hmi_tracking_p_debug;
volatile uint8_t g_task_hmi_tracking_delay_10ms_debug;
volatile uint32_t g_task_hmi_parameter_count_debug;
volatile uint8_t g_task_hmi_motor_command_debug;
volatile int32_t g_task_hmi_zero_target_pulse_debug;
volatile uint8_t g_task_hmi_zero_save_state_debug;
volatile uint32_t g_task_hmi_zero_save_count_debug;

static void TaskHmi_SetBallParameters(uint8_t p,
                                      uint8_t d,
                                      uint8_t time_10ms)
{
  g_task3_kp_debug = (float)p;
  g_task3_kd_debug = (float)d;
  g_task3_switch_delay_10ms_debug = time_10ms;

  g_ballbeam_kp_debug = (float)p;
  g_ballbeam_kd_debug = (float)d;

  g_task_hmi_ball_p_debug = p;
  g_task_hmi_ball_d_debug = d;
  g_task_hmi_ball_time_10ms_debug = time_10ms;
}

static void TaskHmi_SetTrackingParameters(uint8_t p, uint8_t delay_10ms)
{
  g_tracking_kp_debug = (float)p;
  g_task2_finish_confirm_s_debug = (float)delay_10ms * 0.01f;

  g_task_hmi_tracking_p_debug = p;
  g_task_hmi_tracking_delay_10ms_debug = delay_10ms;
}

static int32_t TaskHmi_ClampZeroTarget(int32_t target_pulse)
{
  BallBeam_Config_t *config = BallBeam_GetConfig();
  int32_t limit = config->max_motor_pulse;

  if ((limit <= 0) || (limit > 250))
  {
    limit = 250;
  }
  if (target_pulse > limit)
  {
    return limit;
  }
  if (target_pulse < -limit)
  {
    return -limit;
  }
  return target_pulse;
}

static void TaskHmi_StartZeroSave(void)
{
  BallBeam_Config_t *config = BallBeam_GetConfig();

  BallBeam_SetRun(0U);
  Emm_V5_Stop_Now(config->motor_address, false);

  task_hmi_zero_target_pulse = 0;
  g_task_hmi_zero_target_pulse_debug = 0;
  task_hmi_zero_save_state = 1U;
  task_hmi_zero_save_tick_ms = HAL_GetTick();
  g_task_hmi_zero_save_state_debug = 1U;
}

static void TaskHmi_UpdateZeroSave(uint32_t now_ms)
{
  BallBeam_Config_t *config = BallBeam_GetConfig();

  if (task_hmi_zero_save_state == 1U)
  {
    if ((now_ms - task_hmi_zero_save_tick_ms) <
        TASK_HMI_ZERO_SAVE_DELAY_MS)
    {
      return;
    }

    Emm_V5_Reset_CurPos_To_Zero(config->motor_address);
    task_hmi_zero_save_state = 2U;
    task_hmi_zero_save_tick_ms = now_ms;
    g_task_hmi_zero_save_state_debug = 2U;
  }
  else if (task_hmi_zero_save_state == 2U)
  {
    if ((now_ms - task_hmi_zero_save_tick_ms) <
        TASK_HMI_ZERO_SAVE_DELAY_MS)
    {
      return;
    }

    Emm_V5_Origin_Set_O(config->motor_address, true);
    task_hmi_zero_save_state = 3U;
    task_hmi_zero_save_tick_ms = now_ms;
    g_task_hmi_zero_save_state_debug = 3U;
  }
  else if (task_hmi_zero_save_state == 3U)
  {
    if ((now_ms - task_hmi_zero_save_tick_ms) <
        TASK_HMI_ZERO_SAVE_DELAY_MS)
    {
      return;
    }

    Emm_V5_Origin_Modify_Params(config->motor_address,
                                true,
                                TASK_HMI_HOME_MODE,
                                TASK_HMI_HOME_DIRECTION,
                                TASK_HMI_HOME_SPEED,
                                TASK_HMI_HOME_TIMEOUT_MS,
                                TASK_HMI_HOME_SENSOR_SPEED,
                                TASK_HMI_HOME_SENSOR_MA,
                                TASK_HMI_HOME_SENSOR_MS,
                                true);
    task_hmi_zero_save_state = 0U;
    g_task_hmi_zero_save_state_debug = 0U;
    g_task_hmi_zero_save_count_debug++;
  }
}

static void TaskHmi_ApplyMotorCommand(uint8_t command)
{
  BallBeam_Config_t *config = BallBeam_GetConfig();
  uint16_t saved_speed;
  uint8_t saved_acceleration;

  g_task_hmi_motor_command_debug = command;

  if (command == TASK_HMI_CMD_ZERO_CONFIRM)
  {
    if (task_hmi_zero_save_state == 0U)
    {
      TaskHmi_StartZeroSave();
    }
    return;
  }

  if ((task_hmi_zero_save_state != 0U) ||
      ((command != TASK_HMI_CMD_ZERO_PLUS) &&
       (command != TASK_HMI_CMD_ZERO_MINUS)))
  {
    return;
  }

  if (command == TASK_HMI_CMD_ZERO_PLUS)
  {
    task_hmi_zero_target_pulse += TASK_HMI_ZERO_STEP_PULSE;
  }
  else
  {
    task_hmi_zero_target_pulse -= TASK_HMI_ZERO_STEP_PULSE;
  }
  task_hmi_zero_target_pulse =
      TaskHmi_ClampZeroTarget(task_hmi_zero_target_pulse);

  BallBeam_SetRun(0U);
  saved_speed = config->motor_speed_rpm;
  saved_acceleration = config->motor_acceleration;
  config->motor_speed_rpm = TASK_HMI_ZERO_MOVE_SPEED;
  config->motor_acceleration = TASK_HMI_ZERO_MOVE_ACCEL;
  BallBeam_CommandAbsolutePulse(task_hmi_zero_target_pulse);
  config->motor_speed_rpm = saved_speed;
  config->motor_acceleration = saved_acceleration;
  g_task_hmi_zero_target_pulse_debug = task_hmi_zero_target_pulse;
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
                              TASK_HMI_BALL_DEFAULT_TIME_10MS);
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
    else if ((command >= TASK_HMI_CMD_ZERO_PLUS) &&
             (command <= TASK_HMI_CMD_ZERO_CONFIRM))
    {
      task_hmi_pending_motor_command = command;
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
  task_hmi_pending_motor_command = 0U;
  task_hmi_pending_parameter_command = 0U;
  task_hmi_pending_parameter_data[0] = 0U;
  task_hmi_pending_parameter_data[1] = 0U;
  task_hmi_pending_parameter_data[2] = 0U;
  task_hmi_restart_required = 0U;
  task_hmi_zero_save_state = 0U;
  task_hmi_zero_save_tick_ms = 0U;
  task_hmi_zero_target_pulse = 0;
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
  g_task_hmi_ball_time_10ms_debug = 0U;
  g_task_hmi_tracking_p_debug = 0U;
  g_task_hmi_tracking_delay_10ms_debug = 0U;
  g_task_hmi_parameter_count_debug = 0U;
  g_task_hmi_motor_command_debug = 0U;
  g_task_hmi_zero_target_pulse_debug = 0;
  g_task_hmi_zero_save_state_debug = 0U;
  g_task_hmi_zero_save_count_debug = 0U;
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
  uint32_t now_ms;
  uint8_t parameter_command;
  uint8_t motor_command;
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
  motor_command = task_hmi_pending_motor_command;
  parameter_data[0] = task_hmi_pending_parameter_data[0];
  parameter_data[1] = task_hmi_pending_parameter_data[1];
  parameter_data[2] = task_hmi_pending_parameter_data[2];
  task_hmi_pending_parameter_command = 0U;
  task_hmi_pending_motor_command = 0U;
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

  if (motor_command != 0U)
  {
    TaskHmi_ApplyMotorCommand(motor_command);
  }

  now_ms = HAL_GetTick();
  TaskHmi_UpdateZeroSave(now_ms);
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
