#include "ws2812_screen.h"
#include <string.h>

/* ========================= 亮度限制函数 ======================== */

/**
 * @brief 对RGB颜色应用亮度限制（所有通道/2）
 * @param color 输入的RGB颜色
 * @return 亮度限制后的RGB颜色
 */
RGB_t Screen_ApplyBrightnessLimit(RGB_t color)
{
    RGB_t limited;
    limited.R = color.R / BRIGHTNESS_SCALE;  /* 除以2 */
    limited.G = color.G / BRIGHTNESS_SCALE;  /* 除以2 */
    limited.B = color.B / BRIGHTNESS_SCALE;  /* 除以2 */
    return limited;
}

/* ========================= 坐标转换函数 ======================== */

/**
 * @brief 坐标(x,y)转换为LED索引
 * 
 * 蛇形排列映射：
 *   第0行(y=0)：左→右  index = 0,1,2,3
 *   第1行(y=1)：右←左  index = 7,6,5,4
 *   第2行(y=2)：左→右  index = 8,9,10,11
 *   第3行(y=3)：右←左  index = 15,14,13,12
 * 
 * @param x 列号 (0-3)
 * @param y 行号 (0-3)
 * @return LED索引 (0-15)
 */
uint8_t Screen_XY_to_Index(uint8_t x, uint8_t y)
{
    /* 边界检查 */
    if (x >= SCREEN_COLS || y >= SCREEN_ROWS) {
        return 0;
    }

    /* 奇数行（y=1,3）：从右往左 */
    if (y & 1) {
        return y * SCREEN_COLS + (SCREEN_COLS - 1 - x);
    }
    /* 偶数行（y=0,2）：从左往右 */
    else {
        return y * SCREEN_COLS + x;
    }
}

/* ========================= 屏幕操作函数 ======================== */

/**
 * @brief 初始化屏幕控制器
 */
HAL_StatusTypeDef Screen_Init(WS2812_Screen_t *screen, WS2812_t *ws2812)
{
    if (screen == NULL || ws2812 == NULL) {
        return HAL_ERROR;
    }

    screen->ws2812 = ws2812;
    screen->brightness_enabled = 1;  /* 默认启用亮度限制 */
    Screen_Clear(screen);

    return HAL_OK;
}

/**
 * @brief 启用/禁用亮度限制
 */
void Screen_SetBrightnessLimit(WS2812_Screen_t *screen, uint8_t enable)
{
    if (screen == NULL) {
        return;
    }
    screen->brightness_enabled = enable ? 1 : 0;
}

/**
 * @brief 获取亮度限制状态
 */
uint8_t Screen_GetBrightnessLimit(WS2812_Screen_t *screen)
{
    if (screen == NULL) {
        return 0;
    }
    return screen->brightness_enabled;
}

/**
 * @brief 设置屏幕上某个像素的颜色
 */
void Screen_SetPixel(WS2812_Screen_t *screen, uint8_t x, uint8_t y, RGB_t color)
{
    if (x >= SCREEN_COLS || y >= SCREEN_ROWS || screen == NULL) {
        return;
    }

    /* 如果启用了亮度限制，应用之 */
    if (screen->brightness_enabled) {
        color = Screen_ApplyBrightnessLimit(color);
    }

    screen->screen_buffer[y][x] = color;
}

/**
 * @brief 获取屏幕上某个像素的颜色
 */
RGB_t Screen_GetPixel(WS2812_Screen_t *screen, uint8_t x, uint8_t y)
{
    RGB_t black = {0, 0, 0};

    if (x >= SCREEN_COLS || y >= SCREEN_ROWS || screen == NULL) {
        return black;
    }

    return screen->screen_buffer[y][x];
}

/**
 * @brief 清空屏幕（所有像素关闭）
 */
void Screen_Clear(WS2812_Screen_t *screen)
{
    if (screen == NULL) {
        return;
    }

    RGB_t black = {0, 0, 0};
    Screen_Fill(screen, black);
}

/**
 * @brief 填充屏幕（所有像素设为同一颜色）
 */
void Screen_Fill(WS2812_Screen_t *screen, RGB_t color)
{
    uint8_t x, y;

    if (screen == NULL) {
        return;
    }

    /* 如果启用了亮度限制，应用之 */
    if (screen->brightness_enabled) {
        color = Screen_ApplyBrightnessLimit(color);
    }

    for (y = 0; y < SCREEN_ROWS; y++) {
        for (x = 0; x < SCREEN_COLS; x++) {
            screen->screen_buffer[y][x] = color;
        }
    }
}

/**
 * @brief 刷新屏幕（将缓冲区内容发送到LED）
 */
void Screen_Flush(WS2812_Screen_t *screen)
{
    uint8_t x, y;
    uint8_t led_index;

    if (screen == NULL || screen->ws2812 == NULL) {
        return;
    }

    /* 将2D缓冲区映射到1D LED数组 */
    for (y = 0; y < SCREEN_ROWS; y++) {
        for (x = 0; x < SCREEN_COLS; x++) {
            led_index = Screen_XY_to_Index(x, y);
            WS2812_SetColor(screen->ws2812, led_index, screen->screen_buffer[y][x]);
        }
    }

    /* 发送到硬件 */
    WS2812_Send(screen->ws2812);
}

/**
 * @brief 设置屏幕某一行的颜色
 */
void Screen_SetRow(WS2812_Screen_t *screen, uint8_t y, RGB_t color)
{
    uint8_t x;

    if (y >= SCREEN_ROWS || screen == NULL) {
        return;
    }

    /* 如果启用了亮度限制，应用之 */
    if (screen->brightness_enabled) {
        color = Screen_ApplyBrightnessLimit(color);
    }

    for (x = 0; x < SCREEN_COLS; x++) {
        screen->screen_buffer[y][x] = color;
    }
}

/**
 * @brief 设置屏幕某一列的颜色
 */
void Screen_SetColumn(WS2812_Screen_t *screen, uint8_t x, RGB_t color)
{
    uint8_t y;

    if (x >= SCREEN_COLS || screen == NULL) {
        return;
    }

    /* 如果启用了亮度限制，应用之 */
    if (screen->brightness_enabled) {
        color = Screen_ApplyBrightnessLimit(color);
    }

    for (y = 0; y < SCREEN_ROWS; y++) {
        screen->screen_buffer[y][x] = color;
    }
}

/**
 * @brief 绘制水平线
 */
void Screen_DrawHLine(WS2812_Screen_t *screen, uint8_t x1, uint8_t x2, uint8_t y, RGB_t color)
{
    uint8_t x;
    uint8_t min_x, max_x;

    if (y >= SCREEN_ROWS || screen == NULL) {
        return;
    }

    /* 如果启用了亮度限制，应用之 */
    if (screen->brightness_enabled) {
        color = Screen_ApplyBrightnessLimit(color);
    }

    /* 确保x1 <= x2 */
    min_x = (x1 <= x2) ? x1 : x2;
    max_x = (x1 <= x2) ? x2 : x1;

    /* 边界检查 */
    if (min_x >= SCREEN_COLS) min_x = SCREEN_COLS - 1;
    if (max_x >= SCREEN_COLS) max_x = SCREEN_COLS - 1;

    for (x = min_x; x <= max_x; x++) {
        screen->screen_buffer[y][x] = color;
    }
}

/**
 * @brief 绘制竖直线
 */
void Screen_DrawVLine(WS2812_Screen_t *screen, uint8_t x, uint8_t y1, uint8_t y2, RGB_t color)
{
    uint8_t y;
    uint8_t min_y, max_y;

    if (x >= SCREEN_COLS || screen == NULL) {
        return;
    }

    /* 如果启用了亮度限制，应用之 */
    if (screen->brightness_enabled) {
        color = Screen_ApplyBrightnessLimit(color);
    }

    /* 确保y1 <= y2 */
    min_y = (y1 <= y2) ? y1 : y2;
    max_y = (y1 <= y2) ? y2 : y1;

    /* 边界检查 */
    if (min_y >= SCREEN_ROWS) min_y = SCREEN_ROWS - 1;
    if (max_y >= SCREEN_ROWS) max_y = SCREEN_ROWS - 1;

    for (y = min_y; y <= max_y; y++) {
        screen->screen_buffer[y][x] = color;
    }
}

/**
 * @brief 绘制矩形边框
 */
void Screen_DrawRect(WS2812_Screen_t *screen, uint8_t x1, uint8_t y1, uint8_t x2, uint8_t y2, RGB_t color)
{
    uint8_t min_x, max_x, min_y, max_y;
    uint8_t x, y;

    if (screen == NULL) {
        return;
    }

    /* 如果启用了亮度限制，应用之 */
    if (screen->brightness_enabled) {
        color = Screen_ApplyBrightnessLimit(color);
    }

    /* 确定最小和最大坐标 */
    min_x = (x1 <= x2) ? x1 : x2;
    max_x = (x1 <= x2) ? x2 : x1;
    min_y = (y1 <= y2) ? y1 : y2;
    max_y = (y1 <= y2) ? y2 : y1;

    /* 边界检查 */
    if (min_x >= SCREEN_COLS) min_x = SCREEN_COLS - 1;
    if (max_x >= SCREEN_COLS) max_x = SCREEN_COLS - 1;
    if (min_y >= SCREEN_ROWS) min_y = SCREEN_ROWS - 1;
    if (max_y >= SCREEN_ROWS) max_y = SCREEN_ROWS - 1;

    /* 绘制四条边 */
    /* 上下边 */
    for (x = min_x; x <= max_x; x++) {
        screen->screen_buffer[min_y][x] = color;
        screen->screen_buffer[max_y][x] = color;
    }
    
    /* 左右边 */
    for (y = min_y; y <= max_y; y++) {
        screen->screen_buffer[y][min_x] = color;
        screen->screen_buffer[y][max_x] = color;
    }
}

/**
 * @brief 填充矩形
 */
void Screen_FillRect(WS2812_Screen_t *screen, uint8_t x1, uint8_t y1, uint8_t x2, uint8_t y2, RGB_t color)
{
    uint8_t x, y;
    uint8_t min_x, max_x, min_y, max_y;

    if (screen == NULL) {
        return;
    }

    /* 如果启用了亮度限制，应用之 */
    if (screen->brightness_enabled) {
        color = Screen_ApplyBrightnessLimit(color);
    }

    /* 确定最小和最大坐标 */
    min_x = (x1 <= x2) ? x1 : x2;
    max_x = (x1 <= x2) ? x2 : x1;
    min_y = (y1 <= y2) ? y1 : y2;
    max_y = (y1 <= y2) ? y2 : y1;

    /* 边界检查 */
    if (min_x >= SCREEN_COLS) min_x = SCREEN_COLS - 1;
    if (max_x >= SCREEN_COLS) max_x = SCREEN_COLS - 1;
    if (min_y >= SCREEN_ROWS) min_y = SCREEN_ROWS - 1;
    if (max_y >= SCREEN_ROWS) max_y = SCREEN_ROWS - 1;

    /* 填充所有像素 */
    for (y = min_y; y <= max_y; y++) {
        for (x = min_x; x <= max_x; x++) {
            screen->screen_buffer[y][x] = color;
        }
    }
}
