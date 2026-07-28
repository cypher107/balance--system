#include "ControlSettings.h"

#include <string.h>

#include "BallBeam.h"
#include "stm32f1xx_hal.h"

#define SETTINGS_MAGIC          0x42504253UL
#define SETTINGS_VERSION        1U
#define SETTINGS_PAGE_A         0x0807F000UL
#define SETTINGS_PAGE_B         0x0807F800UL
#define SETTINGS_PAGE_SIZE      0x800UL
#define SETTINGS_SAVE_DELAY_MS  2000UL

typedef struct
{
  float position_kp;
  float position_ki;
  float position_kd;
  float angle_kp;
  float angle_ki;
  float angle_kd;
  float target_position_mm;
  float angle_zero_deg;
  float max_beam_angle_deg;
  float max_motor_angle_deg;
  float motor_pulses_per_degree;
  int32_t motor_center_pulse;
  int8_t position_direction;
  int8_t sensor_direction;
  int8_t motor_direction;
  uint8_t motor_positive_direction;
} ControlSettings_Payload_t;

typedef struct
{
  uint32_t magic;
  uint32_t sequence;
  uint16_t version;
  uint16_t payload_length;
  ControlSettings_Payload_t payload;
  uint32_t crc32;
} ControlSettings_Record_t;

static uint32_t settings_active_page;
static uint32_t settings_sequence;
static uint32_t settings_dirty_tick_ms;
static uint8_t settings_dirty;
static uint8_t settings_have_saved_payload;
static ControlSettings_Payload_t settings_saved_payload;
static ControlSettings_Status_t settings_status;

volatile uint8_t g_settings_status_debug;
volatile uint32_t g_settings_sequence_debug;
volatile uint32_t g_settings_save_count_debug;
volatile uint32_t g_settings_error_debug;

static uint32_t ControlSettings_Crc32(const uint8_t *data, uint32_t length)
{
  uint32_t crc = 0xFFFFFFFFUL;
  uint32_t i;

  while (length-- != 0U)
  {
    crc ^= *data++;
    for (i = 0U; i < 8U; i++)
    {
      crc = (crc & 1U) ? ((crc >> 1U) ^ 0xEDB88320UL) : (crc >> 1U);
    }
  }
  return crc ^ 0xFFFFFFFFUL;
}

static uint8_t ControlSettings_FloatInRange(float value,
                                            float minimum,
                                            float maximum)
{
  return ((value == value) &&
          (value >= minimum) &&
          (value <= maximum)) ? 1U : 0U;
}

static uint8_t ControlSettings_ValidatePayload(
    const ControlSettings_Payload_t *payload)
{
  if ((ControlSettings_FloatInRange(payload->position_kp, 0.0f, 1000.0f) == 0U) ||
      (ControlSettings_FloatInRange(payload->position_ki, 0.0f, 1000.0f) == 0U) ||
      (ControlSettings_FloatInRange(payload->position_kd, 0.0f, 1000.0f) == 0U) ||
      (ControlSettings_FloatInRange(payload->angle_kp, 0.0f, 1000.0f) == 0U) ||
      (ControlSettings_FloatInRange(payload->angle_ki, 0.0f, 1000.0f) == 0U) ||
      (ControlSettings_FloatInRange(payload->angle_kd, 0.0f, 1000.0f) == 0U) ||
      (ControlSettings_FloatInRange(payload->target_position_mm, -500.0f, 500.0f) == 0U) ||
      (ControlSettings_FloatInRange(payload->angle_zero_deg, 0.0f, 360.0f) == 0U) ||
      (ControlSettings_FloatInRange(payload->max_beam_angle_deg, 0.1f, 45.0f) == 0U) ||
      (ControlSettings_FloatInRange(payload->max_motor_angle_deg, 0.1f, 360.0f) == 0U) ||
      (ControlSettings_FloatInRange(payload->motor_pulses_per_degree, 0.01f, 100000.0f) == 0U) ||
      ((payload->position_direction != 1) && (payload->position_direction != -1)) ||
      ((payload->sensor_direction != 1) && (payload->sensor_direction != -1)) ||
      ((payload->motor_direction != 1) && (payload->motor_direction != -1)) ||
      (payload->motor_positive_direction > 1U))
  {
    return 0U;
  }
  return 1U;
}

static uint8_t ControlSettings_RecordValid(
    const ControlSettings_Record_t *record)
{
  uint32_t crc;

  if ((record->magic != SETTINGS_MAGIC) ||
      (record->version != SETTINGS_VERSION) ||
      (record->payload_length != sizeof(ControlSettings_Payload_t)))
  {
    return 0U;
  }

  crc = ControlSettings_Crc32((const uint8_t *)&record->payload,
                              sizeof(record->payload));
  if ((crc != record->crc32) ||
      (ControlSettings_ValidatePayload(&record->payload) == 0U))
  {
    return 0U;
  }
  return 1U;
}

static uint8_t ControlSettings_SequenceNewer(uint32_t first, uint32_t second)
{
  return ((int32_t)(first - second) > 0) ? 1U : 0U;
}

static void ControlSettings_Capture(ControlSettings_Payload_t *payload)
{
  PID_t *position_pid = BallBeam_GetPositionPID();
  PID_t *angle_pid = BallBeam_GetAnglePID();
  BallBeam_Config_t *config = BallBeam_GetConfig();

  payload->position_kp = position_pid->kp;
  payload->position_ki = position_pid->ki;
  payload->position_kd = position_pid->kd;
  payload->angle_kp = angle_pid->kp;
  payload->angle_ki = angle_pid->ki;
  payload->angle_kd = angle_pid->kd;
  payload->target_position_mm = BallBeam_GetTarget();
  payload->angle_zero_deg = config->angle_zero_deg;
  payload->max_beam_angle_deg = config->max_beam_angle_deg;
  payload->max_motor_angle_deg = config->max_motor_angle_deg;
  payload->motor_pulses_per_degree = config->motor_pulses_per_degree;
  payload->motor_center_pulse = config->motor_center_pulse;
  payload->position_direction = (config->position_direction >= 0.0f) ? 1 : -1;
  payload->sensor_direction = (config->sensor_direction >= 0.0f) ? 1 : -1;
  payload->motor_direction = (config->motor_direction >= 0.0f) ? 1 : -1;
  payload->motor_positive_direction = config->motor_positive_direction;
}

static void ControlSettings_Apply(const ControlSettings_Payload_t *payload)
{
  PID_t *position_pid = BallBeam_GetPositionPID();
  PID_t *angle_pid = BallBeam_GetAnglePID();
  BallBeam_Config_t *config = BallBeam_GetConfig();

  PID_SetTunings(position_pid,
                 payload->position_kp,
                 payload->position_ki,
                 payload->position_kd);
  PID_SetTunings(angle_pid,
                 payload->angle_kp,
                 payload->angle_ki,
                 payload->angle_kd);
  config->angle_zero_deg = payload->angle_zero_deg;
  config->max_beam_angle_deg = payload->max_beam_angle_deg;
  config->max_motor_angle_deg = payload->max_motor_angle_deg;
  config->motor_pulses_per_degree = payload->motor_pulses_per_degree;
  config->motor_center_pulse = payload->motor_center_pulse;
  config->position_direction = (float)payload->position_direction;
  config->sensor_direction = (float)payload->sensor_direction;
  config->motor_direction = (float)payload->motor_direction;
  config->motor_positive_direction = payload->motor_positive_direction;
  config->motor_speed_rpm = 15U;
  config->motor_acceleration = 5U;
  PID_SetOutputLimit(position_pid,
                     -config->max_beam_angle_deg,
                     config->max_beam_angle_deg);
  PID_SetOutputLimit(angle_pid,
                     -config->max_motor_angle_deg,
                     config->max_motor_angle_deg);
  BallBeam_SetTarget(payload->target_position_mm);
  BallBeam_SetRun(0U);
  BallBeam_Reset();
}

static HAL_StatusTypeDef ControlSettings_WriteRecord(
    uint32_t page_address,
    const ControlSettings_Record_t *record)
{
  FLASH_EraseInitTypeDef erase;
  uint32_t page_error = 0U;
  const uint16_t *source = (const uint16_t *)record;
  uint32_t halfword_count = sizeof(*record) / 2U;
  uint32_t i;
  HAL_StatusTypeDef result;

  result = HAL_FLASH_Unlock();
  if (result != HAL_OK) return result;

  erase.TypeErase = FLASH_TYPEERASE_PAGES;
  erase.Banks = FLASH_BANK_1;
  erase.PageAddress = page_address;
  erase.NbPages = 1U;
  result = HAL_FLASHEx_Erase(&erase, &page_error);

  for (i = 2U; (result == HAL_OK) && (i < halfword_count); i++)
  {
    result = HAL_FLASH_Program(FLASH_TYPEPROGRAM_HALFWORD,
                               page_address + (i * 2U),
                               source[i]);
  }
  if (result == HAL_OK)
  {
    result = HAL_FLASH_Program(FLASH_TYPEPROGRAM_HALFWORD,
                               page_address,
                               source[0]);
  }
  if (result == HAL_OK)
  {
    result = HAL_FLASH_Program(FLASH_TYPEPROGRAM_HALFWORD,
                               page_address + 2U,
                               source[1]);
  }

  (void)HAL_FLASH_Lock();
  return result;
}

static void ControlSettings_SetStatus(ControlSettings_Status_t status)
{
  settings_status = status;
  g_settings_status_debug = (uint8_t)status;
}

void ControlSettings_Init(void)
{
  const ControlSettings_Record_t *record_a =
      (const ControlSettings_Record_t *)SETTINGS_PAGE_A;
  const ControlSettings_Record_t *record_b =
      (const ControlSettings_Record_t *)SETTINGS_PAGE_B;
  const ControlSettings_Record_t *selected = 0;
  uint8_t valid_a = ControlSettings_RecordValid(record_a);
  uint8_t valid_b = ControlSettings_RecordValid(record_b);

  settings_active_page = 0U;
  settings_sequence = 0U;
  settings_dirty_tick_ms = 0U;
  settings_dirty = 0U;
  settings_have_saved_payload = 0U;
  g_settings_sequence_debug = 0U;
  g_settings_save_count_debug = 0U;
  g_settings_error_debug = 0U;

  if ((valid_a != 0U) && (valid_b != 0U))
  {
    selected = (ControlSettings_SequenceNewer(record_b->sequence,
                                               record_a->sequence) != 0U) ?
               record_b : record_a;
  }
  else if (valid_a != 0U)
  {
    selected = record_a;
  }
  else if (valid_b != 0U)
  {
    selected = record_b;
  }

  if (selected != 0)
  {
    ControlSettings_Apply(&selected->payload);
    settings_saved_payload = selected->payload;
    settings_have_saved_payload = 1U;
    settings_sequence = selected->sequence;
    settings_active_page = (selected == record_a) ?
                           SETTINGS_PAGE_A : SETTINGS_PAGE_B;
    g_settings_sequence_debug = settings_sequence;
    ControlSettings_SetStatus(CONTROL_SETTINGS_STATUS_LOADED);
  }
  else
  {
    ControlSettings_SetStatus(CONTROL_SETTINGS_STATUS_DEFAULTS);
  }
}

void ControlSettings_MarkDirty(void)
{
  settings_dirty = 1U;
  settings_dirty_tick_ms = HAL_GetTick();
  ControlSettings_SetStatus(CONTROL_SETTINGS_STATUS_DIRTY);
}

ControlSettings_Status_t ControlSettings_SaveNow(void)
{
  ControlSettings_Record_t record;
  uint32_t target_page;
  HAL_StatusTypeDef result;

  if (BallBeam_GetRun() != 0U)
  {
    ControlSettings_SetStatus(CONTROL_SETTINGS_STATUS_BUSY);
    return settings_status;
  }

  memset(&record, 0, sizeof(record));
  record.magic = SETTINGS_MAGIC;
  record.sequence = settings_sequence + 1U;
  record.version = SETTINGS_VERSION;
  record.payload_length = sizeof(record.payload);
  ControlSettings_Capture(&record.payload);

  if ((settings_have_saved_payload != 0U) &&
      (memcmp(&record.payload,
              &settings_saved_payload,
              sizeof(record.payload)) == 0))
  {
    settings_dirty = 0U;
    ControlSettings_SetStatus(CONTROL_SETTINGS_STATUS_SAVED);
    return settings_status;
  }

  if (ControlSettings_ValidatePayload(&record.payload) == 0U)
  {
    g_settings_error_debug++;
    ControlSettings_SetStatus(CONTROL_SETTINGS_STATUS_ERROR);
    return settings_status;
  }

  record.crc32 = ControlSettings_Crc32((const uint8_t *)&record.payload,
                                       sizeof(record.payload));
  target_page = (settings_active_page == SETTINGS_PAGE_A) ?
                SETTINGS_PAGE_B : SETTINGS_PAGE_A;
  result = ControlSettings_WriteRecord(target_page, &record);
  if ((result != HAL_OK) ||
      (ControlSettings_RecordValid(
          (const ControlSettings_Record_t *)target_page) == 0U))
  {
    g_settings_error_debug++;
    ControlSettings_SetStatus(CONTROL_SETTINGS_STATUS_ERROR);
    return settings_status;
  }

  settings_active_page = target_page;
  settings_sequence = record.sequence;
  settings_saved_payload = record.payload;
  settings_have_saved_payload = 1U;
  settings_dirty = 0U;
  g_settings_sequence_debug = settings_sequence;
  g_settings_save_count_debug++;
  ControlSettings_SetStatus(CONTROL_SETTINGS_STATUS_SAVED);
  return settings_status;
}

void ControlSettings_Task(void)
{
  if ((settings_dirty != 0U) &&
      (BallBeam_GetRun() == 0U) &&
      ((HAL_GetTick() - settings_dirty_tick_ms) >= SETTINGS_SAVE_DELAY_MS))
  {
    (void)ControlSettings_SaveNow();
  }
}

ControlSettings_Status_t ControlSettings_GetStatus(void)
{
  return settings_status;
}
