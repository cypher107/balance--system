#include "Screen.h"
#include "tjc_usart_hmi.h"

volatile uint8_t g_screen_rx_byte_debug;
volatile uint8_t g_screen_cmd_debug;
volatile uint32_t g_screen_rx_count_debug;
volatile uint8_t g_screen_tjc_return_code_debug;
volatile uint32_t g_screen_tjc_return_count_debug;

static uint8_t screen_previous_byte;
static uint8_t screen_frame_cmd;
static uint8_t screen_tjc_packet_first;
static uint8_t screen_tjc_packet_length;
static uint8_t screen_tjc_ff_count;
static uint8_t screen_tjc_feedback_enabled;

void Screen_Init(void)
{
  g_screen_rx_byte_debug = 0U;
  g_screen_cmd_debug = 0U;
  g_screen_rx_count_debug = 0U;
  g_screen_tjc_return_code_debug = 0U;
  g_screen_tjc_return_count_debug = 0U;
  screen_previous_byte = 0U;
  screen_frame_cmd = 0U;
  screen_tjc_packet_first = 0U;
  screen_tjc_packet_length = 0U;
  screen_tjc_ff_count = 0U;
  screen_tjc_feedback_enabled = 0U;
  TJC_UART_Init();
}

void Screen_UART4_RxCpltCallback(UART_HandleTypeDef *huart)
{
  TJC_UART_RxCpltCallback(huart);
}

void Screen_Task(void)
{
  uint8_t data;

  while (tjc_read_byte(&data) != 0U)
  {
    g_screen_rx_byte_debug = data;
    g_screen_rx_count_debug++;

    if (data == 0xFFU)
    {
      screen_tjc_ff_count++;
      if ((screen_tjc_ff_count >= 3U) && (screen_tjc_packet_length > 0U))
      {
        g_screen_tjc_return_code_debug = screen_tjc_packet_first;
        g_screen_tjc_return_count_debug++;
        screen_tjc_packet_first = 0U;
        screen_tjc_packet_length = 0U;
        screen_tjc_ff_count = 0U;
      }
    }
    else
    {
      if (screen_tjc_packet_length == 0U)
      {
        screen_tjc_packet_first = data;
      }
      screen_tjc_packet_length++;
      screen_tjc_ff_count = 0U;
    }

    if ((data >= '1') && (data <= '3'))
    {
      screen_frame_cmd = data;
    }

    if ((screen_previous_byte == '\r') && (data == '\n'))
    {
      g_screen_cmd_debug = screen_frame_cmd;

      if ((screen_frame_cmd != 0U) && (screen_tjc_feedback_enabled == 0U))
      {
        tjc_send_string("bkcmd=3");
        screen_tjc_feedback_enabled = 1U;
      }

      if (screen_frame_cmd == '1')
      {
        tjc_send_txt("t2", "txt", "A");
      }
      else if (screen_frame_cmd == '2')
      {
        tjc_send_txt("t2", "txt", "B");
      }
      else if (screen_frame_cmd == '3')
      {
        tjc_send_txt("t2", "txt", "C");
      }

      screen_frame_cmd = 0U;
      screen_tjc_packet_first = 0U;
      screen_tjc_packet_length = 0U;
      screen_tjc_ff_count = 0U;
    }

    screen_previous_byte = data;
  }
}
