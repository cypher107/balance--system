#include "OLED_Menu.h"

#include <stdio.h>

#include "BallBeam.h"
#include "ControlSettings.h"
#include "OLED.h"
#include "stm32f1xx_hal.h"

typedef enum
{
  MENU_PAGE_MAIN = 0,
  MENU_PAGE_CONTROL,
  MENU_PAGE_POSITION_PID,
  MENU_PAGE_ANGLE_PID,
  MENU_PAGE_CONFIG,
  MENU_PAGE_LIVE,
  MENU_PAGE_STATUS
} MenuPage_t;

typedef enum
{
  MENU_PARAM_NONE = 0,
  MENU_PARAM_TARGET,
  MENU_PARAM_POSITION_KP,
  MENU_PARAM_POSITION_KI,
  MENU_PARAM_POSITION_KD,
  MENU_PARAM_ANGLE_KP,
  MENU_PARAM_ANGLE_KI,
  MENU_PARAM_ANGLE_KD,
  MENU_PARAM_ANGLE_ZERO,
  MENU_PARAM_BEAM_LIMIT,
  MENU_PARAM_MOTOR_LIMIT,
  MENU_PARAM_PULSE_PER_DEG,
  MENU_PARAM_POSITION_DIR,
  MENU_PARAM_SENSOR_DIR,
  MENU_PARAM_MOTOR_DIR
} MenuParam_t;

static const float menu_steps[] = {0.001f, 0.01f, 0.1f, 1.0f, 10.0f};
static MenuPage_t menu_page;
static MenuParam_t menu_param;
static uint8_t menu_selected;
static uint8_t menu_first_visible;
static uint8_t menu_step_index;
static uint8_t menu_dirty;
static volatile OLED_MenuKey_t menu_pending_key;
static uint32_t menu_last_key_tick;
static uint32_t menu_last_live_tick;
static char menu_status_text[22];

static void Menu_Clear(void)
{
  OLED_Fill(0x00U);
}

static void Menu_ShowString(uint8_t x, uint8_t page, const char *text)
{
  OLED_ShowStr(x, page, (unsigned char *)text, 1U);
}

static float Menu_Clamp(float value, float minimum, float maximum)
{
  if (value > maximum) return maximum;
  if (value < minimum) return minimum;
  return value;
}

static uint8_t Menu_ItemCount(void)
{
  if (menu_page == MENU_PAGE_MAIN) return 6U;
  if (menu_page == MENU_PAGE_CONTROL) return 3U;
  if ((menu_page == MENU_PAGE_POSITION_PID) ||
      (menu_page == MENU_PAGE_ANGLE_PID)) return 4U;
  if (menu_page == MENU_PAGE_CONFIG) return 8U;
  return 0U;
}

static const char *Menu_Title(void)
{
  if (menu_page == MENU_PAGE_CONTROL) return "CONTROL";
  if (menu_page == MENU_PAGE_POSITION_PID) return "POSITION PID";
  if (menu_page == MENU_PAGE_ANGLE_PID) return "ANGLE PID";
  if (menu_page == MENU_PAGE_CONFIG) return "CONFIG";
  if (menu_page == MENU_PAGE_LIVE) return "LIVE DATA";
  if (menu_page == MENU_PAGE_STATUS) return "SETTINGS";
  return "BALL BEAM";
}

static const char *Menu_ItemText(uint8_t index)
{
  static const char *main_items[] = {
    "Control", "Position PID", "Angle PID", "Config", "Live", "Save"
  };
  static const char *pid_items[] = {"Kp", "Ki", "Kd", "Back"};
  static const char *config_items[] = {
    "Angle Zero", "Beam Limit", "Motor Limit", "Pulse / Deg",
    "Position Dir", "Sensor Dir", "Motor Dir", "Back"
  };
  static char control_text[16];

  if (menu_page == MENU_PAGE_MAIN) return main_items[index];
  if (menu_page == MENU_PAGE_CONTROL)
  {
    if (index == 0U)
    {
      (void)snprintf(control_text, sizeof(control_text),
                     "%s", BallBeam_GetRun() ? "Stop" : "Start");
      return control_text;
    }
    if (index == 1U) return "Target";
    return "Back";
  }
  if ((menu_page == MENU_PAGE_POSITION_PID) ||
      (menu_page == MENU_PAGE_ANGLE_PID)) return pid_items[index];
  return config_items[index];
}

static const char *Menu_ParamName(void)
{
  if (menu_param == MENU_PARAM_TARGET) return "TARGET POSITION";
  if (menu_param == MENU_PARAM_POSITION_KP) return "POSITION KP";
  if (menu_param == MENU_PARAM_POSITION_KI) return "POSITION KI";
  if (menu_param == MENU_PARAM_POSITION_KD) return "POSITION KD";
  if (menu_param == MENU_PARAM_ANGLE_KP) return "ANGLE KP";
  if (menu_param == MENU_PARAM_ANGLE_KI) return "ANGLE KI";
  if (menu_param == MENU_PARAM_ANGLE_KD) return "ANGLE KD";
  if (menu_param == MENU_PARAM_ANGLE_ZERO) return "ANGLE ZERO";
  if (menu_param == MENU_PARAM_BEAM_LIMIT) return "BEAM LIMIT";
  if (menu_param == MENU_PARAM_MOTOR_LIMIT) return "MOTOR LIMIT";
  if (menu_param == MENU_PARAM_PULSE_PER_DEG) return "PULSE / DEG";
  if (menu_param == MENU_PARAM_POSITION_DIR) return "POSITION DIR";
  if (menu_param == MENU_PARAM_SENSOR_DIR) return "SENSOR DIR";
  return "MOTOR DIR";
}

static float Menu_GetParamValue(void)
{
  PID_t *position_pid = BallBeam_GetPositionPID();
  PID_t *angle_pid = BallBeam_GetAnglePID();
  BallBeam_Config_t *config = BallBeam_GetConfig();

  if (menu_param == MENU_PARAM_TARGET) return BallBeam_GetTarget();
  if (menu_param == MENU_PARAM_POSITION_KP) return position_pid->kp;
  if (menu_param == MENU_PARAM_POSITION_KI) return position_pid->ki;
  if (menu_param == MENU_PARAM_POSITION_KD) return position_pid->kd;
  if (menu_param == MENU_PARAM_ANGLE_KP) return angle_pid->kp;
  if (menu_param == MENU_PARAM_ANGLE_KI) return angle_pid->ki;
  if (menu_param == MENU_PARAM_ANGLE_KD) return angle_pid->kd;
  if (menu_param == MENU_PARAM_ANGLE_ZERO) return config->angle_zero_deg;
  if (menu_param == MENU_PARAM_BEAM_LIMIT) return config->max_beam_angle_deg;
  if (menu_param == MENU_PARAM_MOTOR_LIMIT) return config->max_motor_angle_deg;
  if (menu_param == MENU_PARAM_PULSE_PER_DEG)
    return config->motor_pulses_per_degree;
  if (menu_param == MENU_PARAM_POSITION_DIR) return config->position_direction;
  if (menu_param == MENU_PARAM_SENSOR_DIR) return config->sensor_direction;
  return config->motor_direction;
}

static void Menu_SetParamValue(float value)
{
  PID_t *position_pid = BallBeam_GetPositionPID();
  PID_t *angle_pid = BallBeam_GetAnglePID();
  BallBeam_Config_t *config = BallBeam_GetConfig();

  if (menu_param == MENU_PARAM_TARGET)
    BallBeam_SetTarget(value);
  else if (menu_param == MENU_PARAM_POSITION_KP)
    position_pid->kp = Menu_Clamp(value, 0.0f, 1000.0f);
  else if (menu_param == MENU_PARAM_POSITION_KI)
    position_pid->ki = Menu_Clamp(value, 0.0f, 1000.0f);
  else if (menu_param == MENU_PARAM_POSITION_KD)
    position_pid->kd = Menu_Clamp(value, 0.0f, 1000.0f);
  else if (menu_param == MENU_PARAM_ANGLE_KP)
    angle_pid->kp = Menu_Clamp(value, 0.0f, 1000.0f);
  else if (menu_param == MENU_PARAM_ANGLE_KI)
    angle_pid->ki = Menu_Clamp(value, 0.0f, 1000.0f);
  else if (menu_param == MENU_PARAM_ANGLE_KD)
    angle_pid->kd = Menu_Clamp(value, 0.0f, 1000.0f);
  else if (menu_param == MENU_PARAM_ANGLE_ZERO)
    config->angle_zero_deg = Menu_Clamp(value, 0.0f, 360.0f);
  else if (menu_param == MENU_PARAM_BEAM_LIMIT)
  {
    config->max_beam_angle_deg = Menu_Clamp(value, 0.1f, 45.0f);
    PID_SetOutputLimit(position_pid,
                       -config->max_beam_angle_deg,
                       config->max_beam_angle_deg);
  }
  else if (menu_param == MENU_PARAM_MOTOR_LIMIT)
  {
    config->max_motor_angle_deg = Menu_Clamp(value, 0.1f, 360.0f);
    PID_SetOutputLimit(angle_pid,
                       -config->max_motor_angle_deg,
                       config->max_motor_angle_deg);
  }
  else if (menu_param == MENU_PARAM_PULSE_PER_DEG)
    config->motor_pulses_per_degree = Menu_Clamp(value, 0.01f, 100000.0f);
  else if (menu_param == MENU_PARAM_POSITION_DIR)
    config->position_direction = (value >= 0.0f) ? 1.0f : -1.0f;
  else if (menu_param == MENU_PARAM_SENSOR_DIR)
    config->sensor_direction = (value >= 0.0f) ? 1.0f : -1.0f;
  else if (menu_param == MENU_PARAM_MOTOR_DIR)
    config->motor_direction = (value >= 0.0f) ? 1.0f : -1.0f;

  ControlSettings_MarkDirty();
}

static void Menu_DrawList(void)
{
  uint8_t row;
  uint8_t index;
  uint8_t count = Menu_ItemCount();

  Menu_Clear();
  Menu_ShowString(0U, 0U, Menu_Title());
  for (row = 0U; row < 6U; row++)
  {
    index = menu_first_visible + row;
    if (index >= count) break;
    Menu_ShowString(0U, (uint8_t)(row + 2U),
                    (index == menu_selected) ? ">" : " ");
    Menu_ShowString(12U, (uint8_t)(row + 2U), Menu_ItemText(index));
  }
}

static void Menu_DrawEdit(void)
{
  char text[22];

  Menu_Clear();
  Menu_ShowString(0U, 0U, Menu_ParamName());
  (void)snprintf(text, sizeof(text), "Value: %.3f", Menu_GetParamValue());
  Menu_ShowString(0U, 2U, text);
  if ((menu_param != MENU_PARAM_POSITION_DIR) &&
      (menu_param != MENU_PARAM_SENSOR_DIR) &&
      (menu_param != MENU_PARAM_MOTOR_DIR))
  {
    (void)snprintf(text, sizeof(text), "Step: %.3f",
                   menu_steps[menu_step_index]);
    Menu_ShowString(0U, 4U, text);
  }
}

static const char *Menu_StateText(void)
{
  BallBeam_State_t state = BallBeam_GetState();

  if (state == BALLBEAM_STATE_IDLE) return "IDLE";
  if (state == BALLBEAM_STATE_ARMING) return "WAIT DATA";
  if (state == BALLBEAM_STATE_RUN) return "RUN";
  if (state == BALLBEAM_STATE_LOST) return "DATA LOST";
  return "FAULT";
}

static void Menu_DrawLive(void)
{
  char text[22];

  Menu_Clear();
  Menu_ShowString(0U, 0U, Menu_Title());
  (void)snprintf(text, sizeof(text), "Pos: %.2f", g_ballbeam_position_debug);
  Menu_ShowString(0U, 2U, text);
  (void)snprintf(text, sizeof(text), "Beam SP: %.2f",
                 g_ballbeam_target_beam_angle_debug);
  Menu_ShowString(0U, 3U, text);
  (void)snprintf(text, sizeof(text), "Angle: %.2f",
                 g_ballbeam_measured_angle_debug);
  Menu_ShowString(0U, 4U, text);
  (void)snprintf(text, sizeof(text), "Motor SP: %.2f",
                 g_ballbeam_motor_angle_debug);
  Menu_ShowString(0U, 5U, text);
  (void)snprintf(text, sizeof(text), "State: %s", Menu_StateText());
  Menu_ShowString(0U, 7U, text);
}

static void Menu_DrawStatus(void)
{
  Menu_Clear();
  Menu_ShowString(0U, 0U, Menu_Title());
  Menu_ShowString(0U, 3U, menu_status_text);
  Menu_ShowString(0U, 6U, "BACK to return");
}

static void Menu_OpenPage(MenuPage_t page)
{
  menu_page = page;
  menu_selected = 0U;
  menu_first_visible = 0U;
  menu_dirty = 1U;
}

static void Menu_EnterSelected(void)
{
  ControlSettings_Status_t status;

  if (menu_page == MENU_PAGE_MAIN)
  {
    if (menu_selected < 5U)
    {
      Menu_OpenPage((MenuPage_t)(menu_selected + 1U));
    }
    else
    {
      status = ControlSettings_SaveNow();
      if (status == CONTROL_SETTINGS_STATUS_SAVED)
        (void)snprintf(menu_status_text, sizeof(menu_status_text), "SAVE OK");
      else if (status == CONTROL_SETTINGS_STATUS_BUSY)
        (void)snprintf(menu_status_text, sizeof(menu_status_text), "STOP BEFORE SAVE");
      else
        (void)snprintf(menu_status_text, sizeof(menu_status_text), "SAVE ERROR");
      Menu_OpenPage(MENU_PAGE_STATUS);
    }
    return;
  }

  if (menu_page == MENU_PAGE_CONTROL)
  {
    if (menu_selected == 0U)
      BallBeam_SetRun((uint8_t)!BallBeam_GetRun());
    else if (menu_selected == 1U)
      menu_param = MENU_PARAM_TARGET;
    else
      Menu_OpenPage(MENU_PAGE_MAIN);
    return;
  }

  if (menu_page == MENU_PAGE_POSITION_PID)
  {
    if (menu_selected < 3U)
      menu_param = (MenuParam_t)(MENU_PARAM_POSITION_KP + menu_selected);
    else
      Menu_OpenPage(MENU_PAGE_MAIN);
    return;
  }

  if (menu_page == MENU_PAGE_ANGLE_PID)
  {
    if (menu_selected < 3U)
      menu_param = (MenuParam_t)(MENU_PARAM_ANGLE_KP + menu_selected);
    else
      Menu_OpenPage(MENU_PAGE_MAIN);
    return;
  }

  if (menu_page == MENU_PAGE_CONFIG)
  {
    if (menu_selected < 7U)
      menu_param = (MenuParam_t)(MENU_PARAM_ANGLE_ZERO + menu_selected);
    else
      Menu_OpenPage(MENU_PAGE_MAIN);
  }
}

void OLED_Menu_Init(void)
{
  menu_page = MENU_PAGE_MAIN;
  menu_param = MENU_PARAM_NONE;
  menu_selected = 0U;
  menu_first_visible = 0U;
  menu_step_index = 2U;
  menu_pending_key = OLED_MENU_KEY_NONE;
  menu_last_key_tick = 0U;
  menu_last_live_tick = 0U;
  menu_status_text[0] = '\0';
  OLED_Init();
  Menu_Clear();
  menu_dirty = 1U;
}

void OLED_Menu_Input(OLED_MenuKey_t key)
{
  uint32_t now_ms = HAL_GetTick();

  if (((now_ms - menu_last_key_tick) >= 40U) &&
      (menu_pending_key == OLED_MENU_KEY_NONE))
  {
    menu_pending_key = key;
    menu_last_key_tick = now_ms;
  }
}

void OLED_Menu_Task(void)
{
  OLED_MenuKey_t key = menu_pending_key;
  uint8_t count;
  float value;

  menu_pending_key = OLED_MENU_KEY_NONE;
  if (menu_param != MENU_PARAM_NONE)
  {
    value = Menu_GetParamValue();
    if ((menu_param == MENU_PARAM_POSITION_DIR) ||
        (menu_param == MENU_PARAM_SENSOR_DIR) ||
        (menu_param == MENU_PARAM_MOTOR_DIR))
    {
      if ((key == OLED_MENU_KEY_UP) || (key == OLED_MENU_KEY_DOWN))
        Menu_SetParamValue(-value);
    }
    else if (key == OLED_MENU_KEY_UP)
      Menu_SetParamValue(value + menu_steps[menu_step_index]);
    else if (key == OLED_MENU_KEY_DOWN)
      Menu_SetParamValue(value - menu_steps[menu_step_index]);
    else if (key == OLED_MENU_KEY_ENTER)
      menu_step_index = (uint8_t)((menu_step_index + 1U) % 5U);
    else if (key == OLED_MENU_KEY_BACK)
      menu_param = MENU_PARAM_NONE;
    if (key != OLED_MENU_KEY_NONE) menu_dirty = 1U;
  }
  else if ((menu_page == MENU_PAGE_LIVE) ||
           (menu_page == MENU_PAGE_STATUS))
  {
    if (key == OLED_MENU_KEY_BACK) Menu_OpenPage(MENU_PAGE_MAIN);
  }
  else
  {
    count = Menu_ItemCount();
    if ((key == OLED_MENU_KEY_UP) && (menu_selected > 0U))
      menu_selected--;
    else if ((key == OLED_MENU_KEY_DOWN) && (menu_selected + 1U < count))
      menu_selected++;
    else if (key == OLED_MENU_KEY_ENTER)
      Menu_EnterSelected();
    else if ((key == OLED_MENU_KEY_BACK) && (menu_page != MENU_PAGE_MAIN))
      Menu_OpenPage(MENU_PAGE_MAIN);

    if (menu_selected < menu_first_visible)
      menu_first_visible = menu_selected;
    else if (menu_selected >= (uint8_t)(menu_first_visible + 6U))
      menu_first_visible = (uint8_t)(menu_selected - 5U);
    if (key != OLED_MENU_KEY_NONE) menu_dirty = 1U;
  }

  if ((menu_page == MENU_PAGE_LIVE) &&
      ((HAL_GetTick() - menu_last_live_tick) >= 100U))
  {
    menu_last_live_tick = HAL_GetTick();
    menu_dirty = 1U;
  }

  if (menu_dirty == 0U) return;
  menu_dirty = 0U;
  if (menu_param != MENU_PARAM_NONE) Menu_DrawEdit();
  else if (menu_page == MENU_PAGE_LIVE) Menu_DrawLive();
  else if (menu_page == MENU_PAGE_STATUS) Menu_DrawStatus();
  else Menu_DrawList();
}
