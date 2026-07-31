/**
 * @file    oled.h
 * @brief   0.96寸 OLED 驱动头文件 (SSD1306, I2C接口, 128x64)
 *
 * 移植说明：
 *   1. 修改 oled.h 中 OLED_I2C_HANDLE 指向实际使用的 I2C 句柄
 *   2. 修改 OLED_I2C_ADDR 确认 OLED 的 I2C 地址（0x78 或 0x7A）
 *   3. 其余代码无需修改，直接编译即可
 */

#ifndef BSP_OLED_H
#define BSP_OLED_H

#include "main.h"

/* -----------------------------------------------------------------------
 * 移植配置区 —— 只需修改这里
 * ----------------------------------------------------------------------- */
#include "i2c.h"
#define OLED_I2C_HANDLE     hi2c1      /* 使用的 I2C 句柄 */
#define OLED_I2C_ADDR       0x78U      /* OLED I2C 地址: SA0=GND->0x78, SA0=VCC->0x7A */
#define OLED_I2C_TIMEOUT    20U        /* HAL I2C 超时时间 (ms) */
/* ----------------------------------------------------------------------- */

#define OLED_WIDTH          128U
#define OLED_HEIGHT         64U
#define OLED_PAGES          (OLED_HEIGHT / 8U)

/* OLED 颜色定义 */
typedef enum {
    OLED_COLOR_BLACK = 0,   /* 像素关闭（黑色） */
    OLED_COLOR_WHITE = 1,   /* 像素点亮（白色） */
    OLED_COLOR_INVERT = 2,  /* 反转当前像素 */
} OLED_Color_t;

extern volatile uint32_t g_oled_i2c_tx_count_debug;
extern volatile uint32_t g_oled_i2c_error_count_debug;
extern volatile uint32_t g_oled_i2c_last_error_debug;
extern volatile uint32_t g_oled_refresh_page_count_debug;

/* ====================== 基础控制 ====================== */

/**
 * @brief  初始化 OLED，清屏并开启显示
 * @retval 0: 成功, 非0: I2C通信失败
 */
int OLED_Init(void);

/**
 * @brief  将显存（GRAM）内容刷新到屏幕
 */
void OLED_Refresh(void);

/**
 * @brief  清除显存（不立即刷新屏幕）
 */
void OLED_Clear(void);

/**
 * @brief  设置整屏显示开关
 * @param  enable: 1=开显示, 0=关显示
 */
void OLED_SetDisplay(uint8_t enable);

/* ====================== 绘图 API ====================== */

/**
 * @brief  在 (x, y) 处绘制一个像素点
 * @param  x: 列坐标 [0, 127]
 * @param  y: 行坐标 [0, 63]
 * @param  color: OLED_COLOR_WHITE / BLACK / INVERT
 */
void OLED_DrawPixel(uint8_t x, uint8_t y, OLED_Color_t color);

/**
 * @brief  画直线（Bresenham）
 */
void OLED_DrawLine(uint8_t x0, uint8_t y0, uint8_t x1, uint8_t y1, OLED_Color_t color);

/**
 * @brief  画矩形（空心）
 */
void OLED_DrawRect(uint8_t x, uint8_t y, uint8_t w, uint8_t h, OLED_Color_t color);

/**
 * @brief  画填充矩形
 */
void OLED_FillRect(uint8_t x, uint8_t y, uint8_t w, uint8_t h, OLED_Color_t color);

/**
 * @brief  画圆（空心）
 */
void OLED_DrawCircle(uint8_t cx, uint8_t cy, uint8_t r, OLED_Color_t color);

/* ====================== 文字 API ====================== */

/**
 * @brief  在 (x, y) 处显示 ASCII 字符（6x8 点阵）
 * @param  x:    列坐标 [0, 127]
 * @param  y:    行坐标 [0, 63]
 * @param  ch:   ASCII 字符 (0x20~0x7F)
 * @param  color: 前景色
 */
void OLED_DrawChar(uint8_t x, uint8_t y, char ch, OLED_Color_t color);

/**
 * @brief  在 (x, y) 处显示字符串（6x8 点阵，超出屏幕自动换行）
 */
void OLED_DrawString(uint8_t x, uint8_t y, const char *str, OLED_Color_t color);

/**
 * @brief  在 (x, y) 处显示整数（十进制）
 */
void OLED_DrawInt(uint8_t x, uint8_t y, int32_t val, OLED_Color_t color);

/**
 * @brief  在 (x, y) 处显示浮点数
 * @param  decimals: 小数位数
 */
void OLED_DrawFloat(uint8_t x, uint8_t y, float val, uint8_t decimals, OLED_Color_t color);

/* Compatibility API used by the existing menu code. Drawing only changes
 * GRAM; call OLED_Refresh() once after composing a complete frame. */
void OLED_Fill(uint8_t fill_data);
void OLED_ShowStr(uint8_t x, uint8_t page, unsigned char text[], uint8_t text_size);

#endif /* BSP_OLED_H */
