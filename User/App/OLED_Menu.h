#ifndef __OLED_MENU_H
#define __OLED_MENU_H

#include <stdint.h>

typedef enum
{
  OLED_MENU_KEY_NONE = 0,
  OLED_MENU_KEY_UP,
  OLED_MENU_KEY_DOWN,
  OLED_MENU_KEY_ENTER,
  OLED_MENU_KEY_BACK
} OLED_MenuKey_t;

void OLED_Menu_Init(void);
void OLED_Menu_Input(OLED_MenuKey_t key);
void OLED_Menu_Task(void);

#endif
