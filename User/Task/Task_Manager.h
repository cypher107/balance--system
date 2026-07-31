#ifndef __TASK_MANAGER_H
#define __TASK_MANAGER_H

#include <stdint.h>
#include "Task_Types.h"

extern volatile uint8_t g_task_manager_task_id_debug;
extern volatile uint8_t g_task_manager_running_debug;
extern volatile uint8_t g_task_manager_result_debug;
extern volatile uint32_t g_task_manager_elapsed_ms_debug;
extern volatile uint32_t g_task_manager_start_count_debug;

void TaskManager_Init(uint8_t oled_enabled);
void TaskManager_Task(void);
void TaskManager_Start(uint8_t task_id);

#endif
