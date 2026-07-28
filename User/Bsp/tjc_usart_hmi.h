#ifndef __TJC_USART_HMI_H
#define __TJC_USART_HMI_H

#include <stdint.h>
#include "usart.h"

#define TJC_UART       huart4
#define TJC_UART_INS   UART4
#define RINGBUFFER_LEN 500U

#define usize      getRingBufferLength()
#define code_c()   initRingBuffer()
#define udelete(x) deleteRingBuffer(x)
#define u(x)       read1ByteFromRingBuffer(x)

extern volatile uint8_t RxBuffer[1];
extern volatile uint32_t tjc_ring_overflow_count;
extern volatile uint8_t tjc_tx_data_debug[132];
extern volatile uint16_t tjc_tx_length_debug;
extern volatile uint8_t tjc_tx_status_debug;
extern volatile uint32_t tjc_tx_count_debug;

void TJC_UART_Init(void);
void TJC_UART_RxCpltCallback(UART_HandleTypeDef *huart);

void tjc_send_string(const char *str);
void tjc_send_txt(const char *objname, const char *attribute, const char *txt);
void tjc_send_val(const char *objname, const char *attribute, int val);
void tjc_send_nstring(const char *str, uint8_t str_length);

void initRingBuffer(void);
void write1ByteToRingBuffer(uint8_t data);
void deleteRingBuffer(uint16_t size);
uint16_t getRingBufferLength(void);
uint8_t read1ByteFromRingBuffer(uint16_t position);
uint8_t tjc_read_byte(uint8_t *data);
uint8_t isRingBufferOverflow(void);

#endif
