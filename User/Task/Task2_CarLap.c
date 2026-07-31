#include "Task2_CarLap.h"

#include "APP_SENSOR.h"
#include "motor.h"
#include "tracking.h"

#define TASK2_CONTROL_PERIOD_MS  10U
#define TASK2_TIMEOUT_MS         20000U

static Task2_Stage_t task2_stage;
static uint32_t task2_start_tick_ms;
static uint32_t task2_last_control_tick_ms;
static uint32_t task2_finish_start_tick_ms;

volatile uint8_t g_task2_stage_debug;
volatile float g_task2_elapsed_s_debug;
volatile float g_task2_min_lap_time_s_debug = 3.0f;
volatile uint8_t g_task2_finish_sensor_count_debug = 8U;
volatile float g_task2_finish_confirm_s_debug = 0.08f;

static void Task2_SetStage(Task2_Stage_t stage)
{
  task2_stage = stage;
  g_task2_stage_debug = (uint8_t)stage;
}

static uint8_t Task2_OnFinishLine(void)
{
  uint8_t left_edge =
      (g_grayscale_values_debug[0] != 0U) ||
      (g_grayscale_values_debug[1] != 0U);
  uint8_t right_edge =
      (g_grayscale_values_debug[10] != 0U) ||
      (g_grayscale_values_debug[11] != 0U);

  return ((g_tracking_sensor_valid_debug != 0U) &&
          (left_edge != 0U) &&
          (right_edge != 0U) &&
          (Tracking_GetActiveCount() >=
           g_task2_finish_sensor_count_debug)) ? 1U : 0U;
}

void Task2_CarLap_Start(uint32_t now_ms)
{
  task2_start_tick_ms = now_ms;
  task2_last_control_tick_ms = now_ms - TASK2_CONTROL_PERIOD_MS;
  task2_finish_start_tick_ms = 0U;
  g_task2_elapsed_s_debug = 0.0f;
  Tracking_Reset();
  Motor_Stop();
  Task2_SetStage(TASK2_STAGE_LEAVE_START);
}

Task_Result_t Task2_CarLap_Update(uint32_t now_ms)
{
  uint32_t elapsed_ms = now_ms - task2_start_tick_ms;
  uint32_t finish_confirm_ms;

  g_task2_elapsed_s_debug = (float)elapsed_ms * 0.001f;
  if (elapsed_ms >= TASK2_TIMEOUT_MS)
  {
    Motor_Stop();
    Task2_SetStage(TASK2_STAGE_TIMEOUT);
    return TASK_RESULT_TIMEOUT;
  }

  if ((now_ms - task2_last_control_tick_ms) < TASK2_CONTROL_PERIOD_MS)
  {
    return TASK_RESULT_RUNNING;
  }
  task2_last_control_tick_ms = now_ms;

  if (task2_stage == TASK2_STAGE_FINISH_CONFIRM)
  {
    Tracking_Update();
    Motor_Stop();
    if (Task2_OnFinishLine() == 0U)
    {
      task2_finish_start_tick_ms = 0U;
      Task2_SetStage(TASK2_STAGE_LAP);
      return TASK_RESULT_RUNNING;
    }

    finish_confirm_ms =
        (uint32_t)(g_task2_finish_confirm_s_debug * 1000.0f);
    if ((now_ms - task2_finish_start_tick_ms) >= finish_confirm_ms)
    {
      Task2_SetStage(TASK2_STAGE_DONE);
      return TASK_RESULT_DONE;
    }
    return TASK_RESULT_RUNNING;
  }

  Tracking_Process();
  if (task2_stage == TASK2_STAGE_LEAVE_START)
  {
    if ((g_tracking_sensor_valid_debug != 0U) &&
        (Task2_OnFinishLine() == 0U) &&
        (Tracking_IsOnLine() != 0U))
    {
      Task2_SetStage(TASK2_STAGE_LAP);
    }
  }
  else if ((task2_stage == TASK2_STAGE_LAP) &&
           (g_task2_elapsed_s_debug >=
            g_task2_min_lap_time_s_debug) &&
           (Task2_OnFinishLine() != 0U))
  {
    Motor_Stop();
    task2_finish_start_tick_ms = now_ms;
    Task2_SetStage(TASK2_STAGE_FINISH_CONFIRM);
  }

  return TASK_RESULT_RUNNING;
}

void Task2_CarLap_Stop(void)
{
  Motor_Stop();
  Tracking_Reset();
  Task2_SetStage(TASK2_STAGE_IDLE);
}

Task2_Stage_t Task2_CarLap_GetStage(void)
{
  return task2_stage;
}
