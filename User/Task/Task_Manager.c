#include "Task_Manager.h"

#include <stdio.h>

#include "BallBeam.h"
#include "OLED.h"
#include "Task_Hmi.h"
#include "Task2_CarLap.h"
#include "Task3_BallMove.h"
#include "Task4_CarToB.h"
#include "Task5_CarLapBalance.h"
#include "Task6_ArbitraryPosition.h"
#include "stm32f1xx_hal.h"

#define TASK_MANAGER_OLED_PERIOD_MS 100U

static uint8_t task_manager_oled_enabled;
static uint8_t task_manager_task_id;
static uint8_t task_manager_running;
static Task_Result_t task_manager_result;
static uint32_t task_manager_start_tick_ms;
static uint32_t task_manager_elapsed_ms;
static uint32_t task_manager_last_oled_tick_ms;

volatile uint8_t g_task_manager_task_id_debug;
volatile uint8_t g_task_manager_running_debug;
volatile uint8_t g_task_manager_result_debug;
volatile uint32_t g_task_manager_elapsed_ms_debug;
volatile uint32_t g_task_manager_start_count_debug;

static void TaskManager_ShowLine(uint8_t page, const char *text)
{
  OLED_ShowStr(0U, page, (unsigned char *)text, 2U);
}

static const char *TaskManager_StatusText(void)
{
  if (task_manager_result == TASK_RESULT_DONE) return "DONE";
  if (task_manager_result == TASK_RESULT_TIMEOUT) return "TIMEOUT";
  if (task_manager_result == TASK_RESULT_NOT_IMPLEMENTED) return "NOT READY";
  if (task_manager_result == TASK_RESULT_STOPPED) return "STOPPED";

  if (task_manager_task_id == 3U)
  {
    Task3_Stage_t stage = Task3_BallMove_GetStage();
    if (stage == TASK3_STAGE_GO_PLUS5) return "OPEN +5CM";
    if (stage == TASK3_STAGE_GO_MINUS5) return "PID -5CM";
  }
  if (task_manager_task_id == 2U)
  {
    Task2_Stage_t stage = Task2_CarLap_GetStage();
    if (stage == TASK2_STAGE_LEAVE_START) return "LEAVE START";
    if (stage == TASK2_STAGE_LAP) return "TRACK LAP";
    if (stage == TASK2_STAGE_FINISH_CONFIRM) return "FINISH LINE";
  }
  if (task_manager_task_id == 4U)
  {
    Task4_Stage_t stage = Task4_CarToB_GetStage();
    if (stage == TASK4_STAGE_START_COMPENSATE) return "START TILT";
    if (stage == TASK4_STAGE_TRACKING) return "TRACKING";
    if (stage == TASK4_STAGE_STOP_COMPENSATE) return "STOP TILT";
    if (stage == TASK4_STAGE_DONE) return "DONE";
    if (stage == TASK4_STAGE_TIMEOUT) return "TIMEOUT";
  }
  return "RUNNING";
}

static void TaskManager_DrawOled(void)
{
  char text[17];
  uint32_t seconds = task_manager_elapsed_ms / 1000U;
  uint32_t centiseconds = (task_manager_elapsed_ms % 1000U) / 10U;

  if (task_manager_oled_enabled == 0U)
  {
    return;
  }

  OLED_Fill(0x00U);
  if (task_manager_task_id == 0U)
  {
    TaskManager_ShowLine(0U, "WAIT HMI COMMAND");
    TaskManager_ShowLine(2U, "USART2 38400");
    TaskManager_ShowLine(4U, "TASK: 2 TO 6");
    OLED_Refresh();
    return;
  }

  (void)snprintf(text, sizeof(text), "TASK %u", task_manager_task_id);
  TaskManager_ShowLine(0U, text);
  (void)snprintf(text, sizeof(text), "TIME:%02lu.%02luS",
                 (unsigned long)seconds,
                 (unsigned long)centiseconds);
  TaskManager_ShowLine(2U, text);
  (void)snprintf(text, sizeof(text), "POS:%05.2fCM",
                 (double)g_ballbeam_position_debug);
  TaskManager_ShowLine(4U, text);
  TaskManager_ShowLine(6U, TaskManager_StatusText());
  OLED_Refresh();
}

static void TaskManager_StopSelected(void)
{
  if (task_manager_task_id == 2U) Task2_CarLap_Stop();
  else if (task_manager_task_id == 3U) Task3_BallMove_Stop();
  else if (task_manager_task_id == 4U) Task4_CarToB_Stop();
  else if (task_manager_task_id == 5U) Task5_CarLapBalance_Stop();
  else if (task_manager_task_id == 6U) Task6_ArbitraryPosition_Stop();
}

static void TaskManager_StartSelected(uint32_t now_ms)
{
  if (task_manager_task_id == 2U) Task2_CarLap_Start(now_ms);
  else if (task_manager_task_id == 3U) Task3_BallMove_Start(now_ms);
  else if (task_manager_task_id == 4U) Task4_CarToB_Start(now_ms);
  else if (task_manager_task_id == 5U) Task5_CarLapBalance_Start(now_ms);
  else if (task_manager_task_id == 6U) Task6_ArbitraryPosition_Start(now_ms);
}

static Task_Result_t TaskManager_UpdateSelected(uint32_t now_ms)
{
  if (task_manager_task_id == 2U) return Task2_CarLap_Update(now_ms);
  if (task_manager_task_id == 3U) return Task3_BallMove_Update(now_ms);
  if (task_manager_task_id == 4U) return Task4_CarToB_Update(now_ms);
  if (task_manager_task_id == 5U) return Task5_CarLapBalance_Update(now_ms);
  if (task_manager_task_id == 6U) return Task6_ArbitraryPosition_Update(now_ms);
  return TASK_RESULT_NOT_IMPLEMENTED;
}

void TaskManager_Init(uint8_t oled_enabled)
{
  task_manager_oled_enabled = (oled_enabled != 0U) ? 1U : 0U;
  task_manager_task_id = 0U;
  task_manager_running = 0U;
  task_manager_result = TASK_RESULT_STOPPED;
  task_manager_start_tick_ms = 0U;
  task_manager_elapsed_ms = 0U;
  task_manager_last_oled_tick_ms = 0U;
  g_task_manager_task_id_debug = 0U;
  g_task_manager_running_debug = 0U;
  g_task_manager_result_debug = (uint8_t)TASK_RESULT_STOPPED;
  g_task_manager_elapsed_ms_debug = 0U;
  g_task_manager_start_count_debug = 0U;

  if (task_manager_oled_enabled != 0U)
  {
    TaskManager_DrawOled();
  }
}

void TaskManager_Start(uint8_t task_id)
{
  uint32_t now_ms;

  if ((task_id < 2U) || (task_id > 6U))
  {
    return;
  }

  TaskManager_StopSelected();
  now_ms = HAL_GetTick();
  task_manager_task_id = task_id;
  task_manager_running = 1U;
  task_manager_result = TASK_RESULT_RUNNING;
  task_manager_start_tick_ms = now_ms;
  task_manager_elapsed_ms = 0U;
  task_manager_last_oled_tick_ms = now_ms;
  g_task_manager_task_id_debug = task_id;
  g_task_manager_running_debug = 1U;
  g_task_manager_result_debug = (uint8_t)TASK_RESULT_RUNNING;
  g_task_manager_elapsed_ms_debug = 0U;
  g_task_manager_start_count_debug++;
  TaskManager_StartSelected(now_ms);
  TaskManager_DrawOled();
}

void TaskManager_Task(void)
{
  uint8_t command;
  uint32_t now_ms = HAL_GetTick();

  if (TaskHmi_GetCommand(&command) != 0U)
  {
    TaskManager_Start(command);
    now_ms = HAL_GetTick();
  }

  if (task_manager_running != 0U)
  {
    task_manager_elapsed_ms = now_ms - task_manager_start_tick_ms;
    task_manager_result = TaskManager_UpdateSelected(now_ms);
    if (task_manager_result != TASK_RESULT_RUNNING)
    {
      /* Stop the task first, then force the motor driver back to origin. */
      TaskManager_StopSelected();
      BallBeam_TriggerHome();
      task_manager_running = 0U;
      task_manager_elapsed_ms = now_ms - task_manager_start_tick_ms;
    }

    g_task_manager_running_debug = task_manager_running;
    g_task_manager_result_debug = (uint8_t)task_manager_result;
    g_task_manager_elapsed_ms_debug = task_manager_elapsed_ms;
  }

  if ((task_manager_oled_enabled != 0U) &&
      ((now_ms - task_manager_last_oled_tick_ms) >=
       TASK_MANAGER_OLED_PERIOD_MS))
  {
    task_manager_last_oled_tick_ms = now_ms;
    TaskManager_DrawOled();
  }
}
