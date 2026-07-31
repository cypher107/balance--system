/**
 * @file    oled.c
 * @brief   0.96寸 OLED 驱动实现 (SSD1306, I2C接口, 128x64)
 */

#include "OLED.h"
#include "OLEDFont.h"
#include <string.h>
#include <stdio.h>
#include <stdlib.h>

/* ====================== 内部显存 ====================== */
/* 128列 x 8页，每页8行，共128x64像素 */
static uint8_t s_gram[OLED_PAGES][OLED_WIDTH];
static uint8_t s_sent_gram[OLED_PAGES][OLED_WIDTH];
static uint8_t s_sent_page_mask;

volatile uint32_t g_oled_i2c_tx_count_debug;
volatile uint32_t g_oled_i2c_error_count_debug;
volatile uint32_t g_oled_i2c_last_error_debug;
volatile uint32_t g_oled_refresh_page_count_debug;

/* ====================== SSD1306 命令表 ====================== */
static const uint8_t OLED_INIT_CMDS[] = {
    0xAE,       /* 关闭显示 */
    0xD5, 0x80, /* 设置时钟分频/振荡频率 */
    0xA8, 0x3F, /* 设置多路比 (64-1) */
    0xD3, 0x00, /* 设置显示偏移 */
    0x40,       /* 设置起始行 = 0 */
    0x8D, 0x14, /* 开启电荷泵 */
    0x20, 0x02, /* 页寻址模式，与 OLED_Refresh 的分页命令一致 */
    0xA1,       /* 列地址映射：反转 (SEG127->COL0) */
    0xC8,       /* COM 扫描方向：反转 */
    0xDA, 0x12, /* COM 引脚配置 */
    0x81, 0xCF, /* 对比度 */
    0xD9, 0xF1, /* 预充电周期 */
    0xDB, 0x40, /* VCOMH 取消选择级别 */
    0xA4,       /* 全局显示 ON（跟随 GRAM）*/
    0xA6,       /* 正常显示（非反色）*/
    0xAF,       /* 开启显示 */
};

/* ====================== 内部 I2C 发送函数 ====================== */

/**
 * @brief 通过 I2C 发送一个命令字节
 */
static int oled_write_cmd(uint8_t cmd)
{
    uint8_t buf[2] = {0x00, cmd}; /* Co=0, D/C#=0 表示命令 */
    HAL_StatusTypeDef status;

    status = HAL_I2C_Master_Transmit(&OLED_I2C_HANDLE, OLED_I2C_ADDR,
                                     buf, 2, OLED_I2C_TIMEOUT);
    g_oled_i2c_tx_count_debug++;
    g_oled_i2c_last_error_debug = HAL_I2C_GetError(&OLED_I2C_HANDLE);
    if (status != HAL_OK) {
        g_oled_i2c_error_count_debug++;
        return -1;
    }
    return 0;
}

/**
 * @brief 通过 I2C 发送数据缓冲区（一整页显示数据）
 */
static int oled_write_data(uint8_t *data, uint16_t len)
{
    /* SSD1306 数据传输：控制字节 0x40 + 数据 */
    /* 由于 HAL 传输不能直接在数据前插入控制字节，使用临时缓冲 */
    static uint8_t buf[OLED_WIDTH + 1];
    HAL_StatusTypeDef status;

    if (len > OLED_WIDTH) {
        return -1;
    }
    buf[0] = 0x40; /* Co=0, D/C#=1 表示数据 */
    memcpy(&buf[1], data, len);
    status = HAL_I2C_Master_Transmit(&OLED_I2C_HANDLE, OLED_I2C_ADDR,
                                     buf, len + 1U, OLED_I2C_TIMEOUT);
    g_oled_i2c_tx_count_debug++;
    g_oled_i2c_last_error_debug = HAL_I2C_GetError(&OLED_I2C_HANDLE);
    if (status != HAL_OK) {
        g_oled_i2c_error_count_debug++;
        return -1;
    }
    return 0;
}

/* ====================== 基础控制实现 ====================== */

int OLED_Init(void)
{
    uint8_t i;

    s_sent_page_mask = 0U;
    HAL_Delay(100); /* 上电等待 OLED 稳定 */

    for (i = 0U; i < (uint8_t)sizeof(OLED_INIT_CMDS); i++) {
        if (oled_write_cmd(OLED_INIT_CMDS[i]) != 0) {
            return -1;
        }
    }

    OLED_Clear();
    OLED_Refresh();
    return 0;
}

void OLED_Refresh(void)
{
    uint8_t page;
    uint8_t page_bit;
    int result;

    for (page = 0U; page < OLED_PAGES; page++) {
        page_bit = (uint8_t)(1U << page);
        if (((s_sent_page_mask & page_bit) != 0U) &&
            (memcmp(s_gram[page], s_sent_gram[page], OLED_WIDTH) == 0)) {
            continue;
        }

        result = oled_write_cmd((uint8_t)(0xB0U + page));
        result |= oled_write_cmd(0x00U);
        result |= oled_write_cmd(0x10U);
        result |= oled_write_data(s_gram[page], OLED_WIDTH);
        if (result == 0) {
            memcpy(s_sent_gram[page], s_gram[page], OLED_WIDTH);
            s_sent_page_mask |= page_bit;
            g_oled_refresh_page_count_debug++;
        }
    }
}

void OLED_Clear(void)
{
    memset(s_gram, 0, sizeof(s_gram));
}

void OLED_SetDisplay(uint8_t enable)
{
    oled_write_cmd(enable ? 0xAF : 0xAE);
}

/* ====================== 绘图实现 ====================== */

void OLED_DrawPixel(uint8_t x, uint8_t y, OLED_Color_t color)
{
    uint8_t page;
    uint8_t bit;

    if (x >= OLED_WIDTH || y >= OLED_HEIGHT) {
        return;
    }

    page = y / 8U;
    bit = y % 8U;

    switch (color) {
        case OLED_COLOR_WHITE:  s_gram[page][x] |=  (1U << bit); break;
        case OLED_COLOR_BLACK:  s_gram[page][x] &= ~(1U << bit); break;
        case OLED_COLOR_INVERT: s_gram[page][x] ^=  (1U << bit); break;
        default: break;
    }
}

void OLED_DrawLine(uint8_t x0, uint8_t y0, uint8_t x1, uint8_t y1, OLED_Color_t color)
{
    int16_t dx = (int16_t)abs((int16_t)x1 - (int16_t)x0);
    int16_t dy = (int16_t)abs((int16_t)y1 - (int16_t)y0);
    int16_t sx = (x0 < x1) ? 1 : -1;
    int16_t sy = (y0 < y1) ? 1 : -1;
    int16_t err = dx - dy;
    int16_t e2;

    while (1) {
        OLED_DrawPixel(x0, y0, color);
        if (x0 == x1 && y0 == y1) break;
        e2 = 2 * err;
        if (e2 > -dy) { err -= dy; x0 = (uint8_t)((int16_t)x0 + sx); }
        if (e2 <  dx) { err += dx; y0 = (uint8_t)((int16_t)y0 + sy); }
    }
}

void OLED_DrawRect(uint8_t x, uint8_t y, uint8_t w, uint8_t h, OLED_Color_t color)
{
    OLED_DrawLine(x,         y,         x + w - 1U, y,         color);
    OLED_DrawLine(x,         y + h - 1U, x + w - 1U, y + h - 1U, color);
    OLED_DrawLine(x,         y,         x,           y + h - 1U, color);
    OLED_DrawLine(x + w - 1U, y,        x + w - 1U,  y + h - 1U, color);
}

void OLED_FillRect(uint8_t x, uint8_t y, uint8_t w, uint8_t h, OLED_Color_t color)
{
    uint8_t col;
    uint8_t row;

    for (col = x; col < x + w; col++) {
        for (row = y; row < y + h; row++) {
            OLED_DrawPixel(col, row, color);
        }
    }
}

void OLED_DrawCircle(uint8_t cx, uint8_t cy, uint8_t r, OLED_Color_t color)
{
    int16_t x = 0;
    int16_t y = (int16_t)r;
    int16_t d = 3 - 2 * (int16_t)r;

    while (x <= y) {
        OLED_DrawPixel((uint8_t)(cx + x), (uint8_t)(cy + y), color);
        OLED_DrawPixel((uint8_t)(cx - x), (uint8_t)(cy + y), color);
        OLED_DrawPixel((uint8_t)(cx + x), (uint8_t)(cy - y), color);
        OLED_DrawPixel((uint8_t)(cx - x), (uint8_t)(cy - y), color);
        OLED_DrawPixel((uint8_t)(cx + y), (uint8_t)(cy + x), color);
        OLED_DrawPixel((uint8_t)(cx - y), (uint8_t)(cy + x), color);
        OLED_DrawPixel((uint8_t)(cx + y), (uint8_t)(cy - x), color);
        OLED_DrawPixel((uint8_t)(cx - y), (uint8_t)(cy - x), color);
        if (d < 0) {
            d += 4 * x + 6;
        } else {
            d += 4 * (x - y) + 10;
            y--;
        }
        x++;
    }
}

/* ====================== 文字实现 ====================== */

void OLED_DrawChar(uint8_t x, uint8_t y, char ch, OLED_Color_t color)
{
    const uint8_t *glyph;
    uint8_t col;
    uint8_t row;
    uint8_t line;

    if (ch < 0x20 || ch > 0x7F) ch = '?';
    glyph = OLED_Font6x8[(uint8_t)(ch - 0x20)];

    for (col = 0U; col < 6U; col++) {
        line = glyph[col];
        for (row = 0U; row < 8U; row++) {
            if (line & (1U << row)) {
                OLED_DrawPixel(x + col, y + row, color);
            } else {
                /* 绘制背景（清除旧像素），使文字可覆盖 */
                OLED_DrawPixel(x + col, y + row,
                               (color == OLED_COLOR_WHITE) ? OLED_COLOR_BLACK : OLED_COLOR_WHITE);
            }
        }
    }
}

void OLED_DrawString(uint8_t x, uint8_t y, const char *str, OLED_Color_t color)
{
    uint8_t cur_x = x;
    uint8_t cur_y = y;

    while (*str) {
        if (cur_x + 6U > OLED_WIDTH) {
            cur_x = 0;
            cur_y += 8U;
        }
        if (cur_y + 8U > OLED_HEIGHT) break;

        OLED_DrawChar(cur_x, cur_y, *str, color);
        cur_x += 6U;
        str++;
    }
}

void OLED_DrawInt(uint8_t x, uint8_t y, int32_t val, OLED_Color_t color)
{
    char buf[12];
    snprintf(buf, sizeof(buf), "%ld", (long)val);
    OLED_DrawString(x, y, buf, color);
}

void OLED_DrawFloat(uint8_t x, uint8_t y, float val, uint8_t decimals, OLED_Color_t color)
{
    char buf[16];
    /* 按小数位数格式化，最多6位小数 */
    if (decimals > 6U) decimals = 6U;
    snprintf(buf, sizeof(buf), "%.*f", (int)decimals, (double)val);
    OLED_DrawString(x, y, buf, color);
}

void OLED_Fill(uint8_t fill_data)
{
    memset(s_gram, fill_data, sizeof(s_gram));
}

void OLED_ShowStr(uint8_t x, uint8_t page, unsigned char text[], uint8_t text_size)
{
    (void)text_size;
    OLED_DrawString(x, (uint8_t)(page * 8U), (const char *)text,
                    OLED_COLOR_WHITE);
}
