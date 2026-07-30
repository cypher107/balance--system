#ifndef __M0_MENU_TEST_H
#define __M0_MENU_TEST_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

typedef enum
{
  M0_MENU_KEY_NONE = 0,
  M0_MENU_KEY_UP,
  M0_MENU_KEY_DOWN,
  M0_MENU_KEY_OK,
  M0_MENU_KEY_GO
} M0_MenuTest_Key_t;

extern volatile uint8_t g_m0_menu_parent_debug;
extern volatile uint8_t g_m0_menu_selected_debug;
extern volatile uint8_t g_m0_menu_mode_debug;
extern volatile uint8_t g_m0_menu_task_debug;
extern volatile uint32_t g_m0_menu_key_count_debug;
extern float g_m0_menu_track_kp_debug;
extern float g_m0_menu_track_ki_debug;
extern float g_m0_menu_track_kd_debug;
extern float g_m0_menu_go_vel_debug;

void M0_MenuTest_Init(void);
void M0_MenuTest_Input(M0_MenuTest_Key_t key);
void M0_MenuTest_Task(void);

#ifdef __cplusplus
}
#endif

#endif
