#include "M0_Menu_Test.h"

#include <stdio.h>
#include <string.h>

#include "OLED.h"
#include "stm32f1xx_hal.h"

#define M0_MENU_VISIBLE_ITEMS 4U
#define M0_MENU_BACK_ACTION   0xFEU
#define M0_MENU_DEFAULT_ACTION 0xFDU

typedef enum
{
  M0_MENU_MODE_LIST = 0,
  M0_MENU_MODE_STEP,
  M0_MENU_MODE_VALUE,
  M0_MENU_MODE_STATUS
} M0_MenuTest_Mode_t;

typedef struct
{
  uint8_t id;
  const char *name;
  float *parameter;
  uint8_t action;
} M0_MenuTest_Item_t;

float g_m0_menu_track_kp_debug = 1.30f;
float g_m0_menu_track_ki_debug = 0.00f;
float g_m0_menu_track_kd_debug = 0.30f;
float g_m0_menu_go_vel_debug = 19.0f;

volatile uint8_t g_m0_menu_parent_debug;
volatile uint8_t g_m0_menu_selected_debug;
volatile uint8_t g_m0_menu_mode_debug;
volatile uint8_t g_m0_menu_task_debug;
volatile uint32_t g_m0_menu_key_count_debug;

static const float m0_menu_steps[] = {
  0.001f, 0.01f, 0.1f, 1.0f, 10.0f, 100.0f
};

static M0_MenuTest_Item_t m0_menu_items[] = {
  {1U,  "Task_Choose", 0, 0U},
  {11U, "Task1", 0, 1U},
  {12U, "Task2", 0, 2U},
  {13U, "Task3", 0, 3U},
  {14U, "Task4", 0, 4U},
  {15U, "Task5", 0, 5U},
  {16U, "Back to Main", 0, M0_MENU_BACK_ACTION},

  {2U,  "Tuning", 0, 0U},
  {21U, "Track_Kp", &g_m0_menu_track_kp_debug, 0U},
  {22U, "Track_Ki", &g_m0_menu_track_ki_debug, 0U},
  {23U, "Track_Kd", &g_m0_menu_track_kd_debug, 0U},
  {24U, "Go_Vel", &g_m0_menu_go_vel_debug, 0U},
  {25U, "Back to Main", 0, M0_MENU_BACK_ACTION},

  {3U,  "Flash", 0, 0U},
  {31U, "Load Defaults", 0, M0_MENU_DEFAULT_ACTION},
  {32U, "Back to Main", 0, M0_MENU_BACK_ACTION}
};

static uint8_t m0_menu_parent;
static uint8_t m0_menu_selected;
static uint8_t m0_menu_step_index;
static uint8_t m0_menu_dirty;
static M0_MenuTest_Mode_t m0_menu_mode;
static M0_MenuTest_Item_t *m0_menu_edit_item;
static volatile M0_MenuTest_Key_t m0_menu_pending_key;
static uint32_t m0_menu_last_key_tick;
static char m0_menu_status[17];

static void M0_MenuTest_Clear(void)
{
  OLED_Fill(0x00U);
}

static void M0_MenuTest_Show16(uint8_t x, uint8_t page, const char *text)
{
  OLED_ShowStr(x, page, (unsigned char *)text, 2U);
}

static uint8_t M0_MenuTest_ItemCount(uint8_t parent)
{
  uint8_t i;
  uint8_t count = 0U;

  for (i = 0U; i < (uint8_t)(sizeof(m0_menu_items) /
                              sizeof(m0_menu_items[0])); i++)
  {
    if ((m0_menu_items[i].id / 10U) == parent) count++;
  }
  return count;
}

static M0_MenuTest_Item_t *M0_MenuTest_GetItem(uint8_t parent,
                                               uint8_t position)
{
  uint8_t i;
  uint8_t current = 0U;

  for (i = 0U; i < (uint8_t)(sizeof(m0_menu_items) /
                              sizeof(m0_menu_items[0])); i++)
  {
    if ((m0_menu_items[i].id / 10U) == parent)
    {
      if (current == position) return &m0_menu_items[i];
      current++;
    }
  }
  return 0;
}

static uint8_t M0_MenuTest_HasChildren(uint8_t id)
{
  return (M0_MenuTest_ItemCount(id) > 0U) ? 1U : 0U;
}

static void M0_MenuTest_SyncDebug(void)
{
  g_m0_menu_parent_debug = m0_menu_parent;
  g_m0_menu_selected_debug = m0_menu_selected;
  g_m0_menu_mode_debug = (uint8_t)m0_menu_mode;
}

static void M0_MenuTest_DrawList(void)
{
  uint8_t count = M0_MenuTest_ItemCount(m0_menu_parent);
  uint8_t first = (uint8_t)((m0_menu_selected /
                             M0_MENU_VISIBLE_ITEMS) *
                            M0_MENU_VISIBLE_ITEMS);
  uint8_t row;

  M0_MenuTest_Clear();
  for (row = 0U; row < M0_MENU_VISIBLE_ITEMS; row++)
  {
    uint8_t position = (uint8_t)(first + row);
    M0_MenuTest_Item_t *item;

    if (position >= count) break;
    item = M0_MenuTest_GetItem(m0_menu_parent, position);
    if (item == 0) break;
    M0_MenuTest_Show16(0U, (uint8_t)(row * 2U), item->name);
    if (position == m0_menu_selected)
    {
      M0_MenuTest_Show16(112U, (uint8_t)(row * 2U), "<-");
    }
  }
}

static void M0_MenuTest_DrawTuning(void)
{
  char text[17];

  M0_MenuTest_Clear();
  (void)snprintf(text, sizeof(text), "%s%s",
                 (m0_menu_mode == M0_MENU_MODE_VALUE) ? ">" : "",
                 m0_menu_edit_item->name);
  M0_MenuTest_Show16(0U, 0U, text);
  (void)snprintf(text, sizeof(text), "Step:%5.3f",
                 m0_menu_steps[m0_menu_step_index]);
  M0_MenuTest_Show16(0U, 2U, text);
  (void)snprintf(text, sizeof(text), "Val:%5.3f",
                 *m0_menu_edit_item->parameter);
  M0_MenuTest_Show16(0U, 4U, text);
  M0_MenuTest_Show16(0U, 6U,
      (m0_menu_mode == M0_MENU_MODE_VALUE) ?
      "OK/GO:Done" : "OK:Edit GO:Back");
}

static void M0_MenuTest_DrawStatus(void)
{
  M0_MenuTest_Clear();
  M0_MenuTest_Show16(0U, 0U, "M0 MENU TEST");
  M0_MenuTest_Show16(0U, 2U, m0_menu_status);
  M0_MenuTest_Show16(0U, 6U, "GO:Back");
}

static void M0_MenuTest_OpenList(uint8_t parent)
{
  m0_menu_parent = parent;
  m0_menu_selected = 0U;
  m0_menu_mode = M0_MENU_MODE_LIST;
  m0_menu_dirty = 1U;
  M0_MenuTest_SyncDebug();
}

static void M0_MenuTest_LoadDefaults(void)
{
  g_m0_menu_track_kp_debug = 1.30f;
  g_m0_menu_track_ki_debug = 0.00f;
  g_m0_menu_track_kd_debug = 0.30f;
  g_m0_menu_go_vel_debug = 19.0f;
  (void)snprintf(m0_menu_status, sizeof(m0_menu_status), "DEFAULTS LOADED");
  m0_menu_mode = M0_MENU_MODE_STATUS;
  m0_menu_dirty = 1U;
  M0_MenuTest_SyncDebug();
}

static void M0_MenuTest_Select(void)
{
  M0_MenuTest_Item_t *item =
      M0_MenuTest_GetItem(m0_menu_parent, m0_menu_selected);

  if (item == 0) return;
  if (M0_MenuTest_HasChildren(item->id) != 0U)
  {
    M0_MenuTest_OpenList(item->id);
  }
  else if (item->action == M0_MENU_BACK_ACTION)
  {
    M0_MenuTest_OpenList(0U);
  }
  else if (item->action == M0_MENU_DEFAULT_ACTION)
  {
    M0_MenuTest_LoadDefaults();
  }
  else if (item->parameter != 0)
  {
    m0_menu_edit_item = item;
    m0_menu_step_index = 1U;
    m0_menu_mode = M0_MENU_MODE_STEP;
    m0_menu_dirty = 1U;
    M0_MenuTest_SyncDebug();
  }
  else if ((item->action >= 1U) && (item->action <= 5U))
  {
    g_m0_menu_task_debug = item->action;
    (void)snprintf(m0_menu_status, sizeof(m0_menu_status),
                   "TASK %u SELECTED", item->action);
    m0_menu_mode = M0_MENU_MODE_STATUS;
    m0_menu_dirty = 1U;
    M0_MenuTest_SyncDebug();
  }
}

void M0_MenuTest_Init(void)
{
  g_m0_menu_task_debug = 0U;
  g_m0_menu_key_count_debug = 0U;
  m0_menu_pending_key = M0_MENU_KEY_NONE;
  m0_menu_last_key_tick = 0U;
  m0_menu_edit_item = 0;
  m0_menu_status[0] = '\0';
  OLED_Init();
  M0_MenuTest_OpenList(0U);
}

void M0_MenuTest_Input(M0_MenuTest_Key_t key)
{
  uint32_t now_ms = HAL_GetTick();

  if (((now_ms - m0_menu_last_key_tick) >= 40U) &&
      (m0_menu_pending_key == M0_MENU_KEY_NONE))
  {
    m0_menu_pending_key = key;
    m0_menu_last_key_tick = now_ms;
    g_m0_menu_key_count_debug++;
  }
}

void M0_MenuTest_Task(void)
{
  M0_MenuTest_Key_t key = m0_menu_pending_key;

  m0_menu_pending_key = M0_MENU_KEY_NONE;
  if (m0_menu_mode == M0_MENU_MODE_LIST)
  {
    uint8_t count = M0_MenuTest_ItemCount(m0_menu_parent);

    if ((key == M0_MENU_KEY_UP) && (m0_menu_selected > 0U))
    {
      m0_menu_selected--;
      m0_menu_dirty = 1U;
    }
    else if ((key == M0_MENU_KEY_DOWN) &&
             ((uint8_t)(m0_menu_selected + 1U) < count))
    {
      m0_menu_selected++;
      m0_menu_dirty = 1U;
    }
    else if (key == M0_MENU_KEY_OK)
    {
      M0_MenuTest_Select();
    }
    else if ((key == M0_MENU_KEY_GO) && (m0_menu_parent != 0U))
    {
      M0_MenuTest_OpenList(0U);
    }
  }
  else if (m0_menu_mode == M0_MENU_MODE_STEP)
  {
    if ((key == M0_MENU_KEY_UP) && (m0_menu_step_index < 5U))
    {
      m0_menu_step_index++;
      m0_menu_dirty = 1U;
    }
    else if ((key == M0_MENU_KEY_DOWN) && (m0_menu_step_index > 0U))
    {
      m0_menu_step_index--;
      m0_menu_dirty = 1U;
    }
    else if (key == M0_MENU_KEY_OK)
    {
      m0_menu_mode = M0_MENU_MODE_VALUE;
      m0_menu_dirty = 1U;
    }
    else if (key == M0_MENU_KEY_GO)
    {
      m0_menu_edit_item = 0;
      m0_menu_mode = M0_MENU_MODE_LIST;
      m0_menu_dirty = 1U;
    }
  }
  else if (m0_menu_mode == M0_MENU_MODE_VALUE)
  {
    if (key == M0_MENU_KEY_UP)
    {
      *m0_menu_edit_item->parameter += m0_menu_steps[m0_menu_step_index];
      m0_menu_dirty = 1U;
    }
    else if (key == M0_MENU_KEY_DOWN)
    {
      *m0_menu_edit_item->parameter -= m0_menu_steps[m0_menu_step_index];
      m0_menu_dirty = 1U;
    }
    else if ((key == M0_MENU_KEY_OK) || (key == M0_MENU_KEY_GO))
    {
      m0_menu_mode = M0_MENU_MODE_STEP;
      m0_menu_dirty = 1U;
    }
  }
  else if ((m0_menu_mode == M0_MENU_MODE_STATUS) &&
           (key == M0_MENU_KEY_GO))
  {
    M0_MenuTest_OpenList(0U);
  }

  M0_MenuTest_SyncDebug();
  if (m0_menu_dirty == 0U) return;
  m0_menu_dirty = 0U;
  if (m0_menu_mode == M0_MENU_MODE_LIST) M0_MenuTest_DrawList();
  else if ((m0_menu_mode == M0_MENU_MODE_STEP) ||
           (m0_menu_mode == M0_MENU_MODE_VALUE)) M0_MenuTest_DrawTuning();
  else M0_MenuTest_DrawStatus();
  OLED_Refresh();
}
