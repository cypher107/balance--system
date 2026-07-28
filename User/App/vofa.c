#include "vofa.h"

#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <stdarg.h>

static UART_HandleTypeDef *vofa_uart = &huart3;
static VOFA_PID_Channel_t vofa_pid_channels[VOFA_MAX_PID_CHANNELS];
static char vofa_rx_buffer[VOFA_RX_BUFFER_SIZE];
static uint16_t vofa_rx_index;

static VOFA_PID_Channel_t *VOFA_FindPID(uint8_t id)
{
  uint8_t i;

  for (i = 0U; i < VOFA_MAX_PID_CHANNELS; i++)
  {
    if ((vofa_pid_channels[i].used != 0U) && (vofa_pid_channels[i].id == id))
    {
      return &vofa_pid_channels[i];
    }
  }

  return 0;
}

static void VOFA_SendFormat(const char *format, ...)
{
  char tx_buffer[160];
  va_list args;
  int len;

  va_start(args, format);
  len = vsnprintf(tx_buffer, sizeof(tx_buffer), format, args);
  va_end(args);

  if (len <= 0)
  {
    return;
  }

  if ((uint32_t)len >= sizeof(tx_buffer))
  {
    len = sizeof(tx_buffer) - 1;
  }

  (void)HAL_UART_Transmit(vofa_uart, (uint8_t *)tx_buffer, (uint16_t)len, 20U);
}

static char *VOFA_NextToken(char **cursor)
{
  char *token;

  while ((**cursor == ' ') || (**cursor == '\t') || (**cursor == ','))
  {
    (*cursor)++;
  }

  if (**cursor == '\0')
  {
    return 0;
  }

  token = *cursor;

  while ((**cursor != '\0') && (**cursor != ' ') && (**cursor != '\t') && (**cursor != ','))
  {
    (*cursor)++;
  }

  if (**cursor != '\0')
  {
    **cursor = '\0';
    (*cursor)++;
  }

  return token;
}

static void VOFA_PrintHelp(void)
{
  VOFA_SendString("VOFA commands:\r\n");
  VOFA_SendString("  list\r\n");
  VOFA_SendString("  get <id>\r\n");
  VOFA_SendString("  pid <id> kp|ki|kd|set|dead|df <value>\r\n");
  VOFA_SendString("  pid <id> out|iout <min> <max>\r\n");
  VOFA_SendString("  pid <id> isep <0|1> <threshold>\r\n");
  VOFA_SendString("  pid <id> reset\r\n");
}

void VOFA_Init(UART_HandleTypeDef *huart)
{
  uint8_t i;

  if (huart != 0)
  {
    vofa_uart = huart;
  }

  vofa_rx_index = 0U;

  for (i = 0U; i < VOFA_MAX_PID_CHANNELS; i++)
  {
    vofa_pid_channels[i].used = 0U;
    vofa_pid_channels[i].id = 0U;
    vofa_pid_channels[i].name = 0;
    vofa_pid_channels[i].pid = 0;
  }
}

void VOFA_AttachPID(uint8_t id, const char *name, PID_t *pid)
{
  uint8_t i;
  VOFA_PID_Channel_t *channel;

  if (pid == 0)
  {
    return;
  }

  channel = VOFA_FindPID(id);
  if (channel == 0)
  {
    for (i = 0U; i < VOFA_MAX_PID_CHANNELS; i++)
    {
      if (vofa_pid_channels[i].used == 0U)
      {
        channel = &vofa_pid_channels[i];
        break;
      }
    }
  }

  if (channel == 0)
  {
    return;
  }

  channel->used = 1U;
  channel->id = id;
  channel->name = name;
  channel->pid = pid;
}

void VOFA_DetachPID(uint8_t id)
{
  VOFA_PID_Channel_t *channel = VOFA_FindPID(id);

  if (channel != 0)
  {
    channel->used = 0U;
    channel->name = 0;
    channel->pid = 0;
  }
}

void VOFA_RxByte(uint8_t data)
{
  if ((data == '\r') || (data == '\n'))
  {
    if (vofa_rx_index > 0U)
    {
      vofa_rx_buffer[vofa_rx_index] = '\0';
      VOFA_ProcessLine(vofa_rx_buffer);
      vofa_rx_index = 0U;
    }
    return;
  }

  if (vofa_rx_index < (VOFA_RX_BUFFER_SIZE - 1U))
  {
    vofa_rx_buffer[vofa_rx_index] = (char)data;
    vofa_rx_index++;
  }
  else
  {
    vofa_rx_index = 0U;
    VOFA_SendString("ERR rx overflow\r\n");
  }
}

void VOFA_RxData(const uint8_t *data, uint16_t len)
{
  uint16_t i;

  if (data == 0)
  {
    return;
  }

  for (i = 0U; i < len; i++)
  {
    VOFA_RxByte(data[i]);
  }
}

void VOFA_ProcessLine(char *line)
{
  char *cursor;
  char *cmd;
  char *arg0;
  char *arg1;
  char *arg2;
  char *arg3;
  uint8_t id;
  VOFA_PID_Channel_t *channel;
  PID_t *pid;

  if (line == 0)
  {
    return;
  }

  cursor = line;
  cmd = VOFA_NextToken(&cursor);
  if (cmd == 0)
  {
    return;
  }

  if ((strcmp(cmd, "help") == 0) || (strcmp(cmd, "?") == 0))
  {
    VOFA_PrintHelp();
    return;
  }

  if (strcmp(cmd, "list") == 0)
  {
    VOFA_SendAllPID();
    return;
  }

  if (strcmp(cmd, "get") == 0)
  {
    arg0 = VOFA_NextToken(&cursor);
    if (arg0 == 0)
    {
      VOFA_SendString("ERR get id\r\n");
      return;
    }
    VOFA_SendPID((uint8_t)atoi(arg0));
    return;
  }

  if (strcmp(cmd, "pid") != 0)
  {
    VOFA_SendString("ERR unknown cmd\r\n");
    return;
  }

  arg0 = VOFA_NextToken(&cursor);
  arg1 = VOFA_NextToken(&cursor);
  arg2 = VOFA_NextToken(&cursor);
  arg3 = VOFA_NextToken(&cursor);

  if ((arg0 == 0) || (arg1 == 0))
  {
    VOFA_SendString("ERR pid args\r\n");
    return;
  }

  id = (uint8_t)atoi(arg0);
  channel = VOFA_FindPID(id);
  if ((channel == 0) || (channel->pid == 0))
  {
    VOFA_SendString("ERR pid not found\r\n");
    return;
  }

  pid = channel->pid;

  if (strcmp(arg1, "reset") == 0)
  {
    PID_Reset(pid);
    VOFA_SendPID(id);
    return;
  }

  if (arg2 == 0)
  {
    VOFA_SendString("ERR pid value\r\n");
    return;
  }

  if (strcmp(arg1, "kp") == 0)
  {
    pid->kp = (float)atof(arg2);
  }
  else if (strcmp(arg1, "ki") == 0)
  {
    pid->ki = (float)atof(arg2);
  }
  else if (strcmp(arg1, "kd") == 0)
  {
    pid->kd = (float)atof(arg2);
  }
  else if (strcmp(arg1, "set") == 0)
  {
    PID_SetTarget(pid, (float)atof(arg2));
  }
  else if (strcmp(arg1, "dead") == 0)
  {
    PID_SetDeadband(pid, (float)atof(arg2));
  }
  else if (strcmp(arg1, "df") == 0)
  {
    PID_SetDerivativeFilter(pid, (float)atof(arg2));
  }
  else if ((strcmp(arg1, "out") == 0) && (arg3 != 0))
  {
    PID_SetOutputLimit(pid, (float)atof(arg2), (float)atof(arg3));
  }
  else if ((strcmp(arg1, "iout") == 0) && (arg3 != 0))
  {
    PID_SetIntegralLimit(pid, (float)atof(arg2), (float)atof(arg3));
  }
  else if ((strcmp(arg1, "isep") == 0) && (arg3 != 0))
  {
    PID_SetIntegralSeparation(pid, (uint8_t)atoi(arg2), (float)atof(arg3));
  }
  else
  {
    VOFA_SendString("ERR pid field\r\n");
    return;
  }

  VOFA_SendPID(id);
}

void VOFA_SendString(const char *str)
{
  uint16_t len;

  if ((str == 0) || (vofa_uart == 0))
  {
    return;
  }

  len = (uint16_t)strlen(str);
  (void)HAL_UART_Transmit(vofa_uart, (uint8_t *)str, len, 20U);
}

void VOFA_SendPID(uint8_t id)
{
  VOFA_PID_Channel_t *channel = VOFA_FindPID(id);
  PID_t *pid;

  if ((channel == 0) || (channel->pid == 0))
  {
    VOFA_SendString("ERR pid not found\r\n");
    return;
  }

  pid = channel->pid;
  VOFA_SendFormat("PID,%u,%s,kp,%.6f,ki,%.6f,kd,%.6f,set,%.6f,out,%.6f,err,%.6f\r\n",
                  channel->id,
                  (channel->name != 0) ? channel->name : "-",
                  pid->kp,
                  pid->ki,
                  pid->kd,
                  pid->setpoint,
                  pid->output,
                  pid->error);
}

void VOFA_SendAllPID(void)
{
  uint8_t i;

  for (i = 0U; i < VOFA_MAX_PID_CHANNELS; i++)
  {
    if (vofa_pid_channels[i].used != 0U)
    {
      VOFA_SendPID(vofa_pid_channels[i].id);
    }
  }
}

void VOFA_SendWave(const float *channels, uint8_t count)
{
  char tx_buffer[160];
  uint16_t offset = 0U;
  uint8_t i;
  int len;

  if ((channels == 0) || (count == 0U) || (vofa_uart == 0))
  {
    return;
  }

  for (i = 0U; i < count; i++)
  {
    len = snprintf(&tx_buffer[offset], sizeof(tx_buffer) - offset,
                   (i == (count - 1U)) ? "%.4f\r\n" : "%.4f,",
                   channels[i]);

    if (len <= 0)
    {
      return;
    }

    offset += (uint16_t)len;
    if (offset >= (sizeof(tx_buffer) - 12U))
    {
      break;
    }
  }

  (void)HAL_UART_Transmit(vofa_uart, (uint8_t *)tx_buffer, offset, 20U);
}

void VOFA_SendNamedWave(float target, float feedback, float output, float error)
{
  float channels[4];

  channels[0] = target;
  channels[1] = feedback;
  channels[2] = output;
  channels[3] = error;

  VOFA_SendWave(channels, 4U);
}
