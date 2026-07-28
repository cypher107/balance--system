#ifndef __CONTROL_SETTINGS_H
#define __CONTROL_SETTINGS_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

typedef enum
{
  CONTROL_SETTINGS_STATUS_DEFAULTS = 0,
  CONTROL_SETTINGS_STATUS_LOADED,
  CONTROL_SETTINGS_STATUS_DIRTY,
  CONTROL_SETTINGS_STATUS_SAVED,
  CONTROL_SETTINGS_STATUS_BUSY,
  CONTROL_SETTINGS_STATUS_ERROR
} ControlSettings_Status_t;

extern volatile uint8_t g_settings_status_debug;
extern volatile uint32_t g_settings_sequence_debug;
extern volatile uint32_t g_settings_save_count_debug;
extern volatile uint32_t g_settings_error_debug;

void ControlSettings_Init(void);
void ControlSettings_MarkDirty(void);
void ControlSettings_Task(void);
ControlSettings_Status_t ControlSettings_SaveNow(void);
ControlSettings_Status_t ControlSettings_GetStatus(void);

#ifdef __cplusplus
}
#endif

#endif
