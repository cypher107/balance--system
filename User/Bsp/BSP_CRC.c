#include "BSP_CRC.h"

uint16_t GetCRC16(const uint8_t *data, uint8_t length)
{
  uint16_t crc = 0xFFFFU;
  uint8_t bit;

  if (data == 0)
  {
    return crc;
  }

  while (length != 0U)
  {
    crc ^= *data;
    data++;
    length--;

    for (bit = 0U; bit < 8U; bit++)
    {
      if ((crc & 0x0001U) != 0U)
      {
        crc = (uint16_t)((crc >> 1U) ^ 0xA001U);
      }
      else
      {
        crc >>= 1U;
      }
    }
  }

  /* Match the byte order returned by the original MSPM0 table implementation. */
  return (uint16_t)((crc << 8U) | (crc >> 8U));
}
