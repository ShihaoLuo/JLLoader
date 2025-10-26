#ifndef __WS2812_SCREEN_H
#define __WS2812_SCREEN_H

#include "ws2812.h"

/* ========================= 屏幕配置 ======================== */
/**
 * 灯珠排列说明（4x4网格，蛇形排列）：
 * 
 * 逻辑坐标系（用户友好）：
 *   (0,0)  (1,0)  (2,0)  (3,0)
 *   (0,1)  (1,1)  (2,1)  (3,1)
 *   (0,2)  (1,2)  (2,2)  (3,2)
 *   (0,3)  (1,3)  (2,3)  (3,3)
 * 
 * 物理排列（蛇形）：
 *   第0行：左→右  灯珠 0   1   2   3
 *   第1行：右←左  灯珠 7   6   5   4
 *   第2行：左→右  灯珠 8   9  10  11
 *   第3行：右←左  灯珠15  14  13  12
 * 
 * 转换规则：
 *   偶数行(0,2)：从左往右 index = row*4 + x
 *   奇数行(1,3)：从右往左 index = row*4 + (3-x)
 */

#define SCREEN_ROWS     4   /* 屏幕行数 */
#define SCREEN_COLS     4   /* 屏幕列数 */
#define SCREEN_SIZE     16  /* 总像素数 */

/* ========================= 亮度限制配置 ======================== */
/**
 * 最大亮度限制：
 * - MAX_BRIGHTNESS = 125 (最大值256的约50%)
 * - 所有RGB通道自动除以2
 * - 防止过度发热和电流过大
 */
#define MAX_BRIGHTNESS      125     /* 最大亮度值 */
#define BRIGHTNESS_SCALE    2       /* 亮度缩放因子 (256/MAX_BRIGHTNESS) */

/* ========================= 屏幕数据结构 ======================== */

/**
 * @brief 屏幕控制器结构体
 */
typedef struct {
    WS2812_t *ws2812;                           /* WS2812控制器指针 */
    RGB_t screen_buffer[SCREEN_ROWS][SCREEN_COLS];  /* 屏幕显示缓冲 */
    uint8_t brightness_enabled;                 /* 是否启用亮度限制 (1=启用, 0=禁用) */
} WS2812_Screen_t;

/* ========================= 函数声明 ======================== */

/**
 * @brief 初始化屏幕控制器
 * @param screen 屏幕控制器指针
 * @param ws2812 WS2812控制器指针
 * @return HAL_OK 成功
 */
HAL_StatusTypeDef Screen_Init(WS2812_Screen_t *screen, WS2812_t *ws2812);

/**
 * @brief 启用/禁用亮度限制
 * @param screen 屏幕控制器指针
 * @param enable 1=启用亮度限制, 0=禁用亮度限制
 */
void Screen_SetBrightnessLimit(WS2812_Screen_t *screen, uint8_t enable);

/**
 * @brief 获取亮度限制状态
 * @param screen 屏幕控制器指针
 * @return 1=启用, 0=禁用
 */
uint8_t Screen_GetBrightnessLimit(WS2812_Screen_t *screen);

/**
 * @brief 对RGB颜色应用亮度限制
 * @param color 输入的RGB颜色
 * @return 亮度限制后的RGB颜色
 */
RGB_t Screen_ApplyBrightnessLimit(RGB_t color);

/**
 * @brief 坐标(x,y)转换为LED索引
 * @param x 水平坐标 (0-3)
 * @param y 竖直坐标 (0-3)
 * @return LED索引 (0-15)
 */
uint8_t Screen_XY_to_Index(uint8_t x, uint8_t y);

/**
 * @brief 设置屏幕上某个像素的颜色
 * @param screen 屏幕控制器指针
 * @param x 水平坐标 (0-3)
 * @param y 竖直坐标 (0-3)
 * @param color RGB颜色
 */
void Screen_SetPixel(WS2812_Screen_t *screen, uint8_t x, uint8_t y, RGB_t color);

/**
 * @brief 获取屏幕上某个像素的颜色
 * @param screen 屏幕控制器指针
 * @param x 水平坐标 (0-3)
 * @param y 竖直坐标 (0-3)
 * @return RGB颜色
 */
RGB_t Screen_GetPixel(WS2812_Screen_t *screen, uint8_t x, uint8_t y);

/**
 * @brief 清空屏幕（所有像素关闭）
 * @param screen 屏幕控制器指针
 */
void Screen_Clear(WS2812_Screen_t *screen);

/**
 * @brief 填充屏幕（所有像素设为同一颜色）
 * @param screen 屏幕控制器指针
 * @param color RGB颜色
 */
void Screen_Fill(WS2812_Screen_t *screen, RGB_t color);

/**
 * @brief 刷新屏幕（将缓冲区内容发送到LED）
 * @param screen 屏幕控制器指针
 */
void Screen_Flush(WS2812_Screen_t *screen);

/**
 * @brief 设置屏幕某一行的颜色
 * @param screen 屏幕控制器指针
 * @param y 行号 (0-3)
 * @param color RGB颜色
 */
void Screen_SetRow(WS2812_Screen_t *screen, uint8_t y, RGB_t color);

/**
 * @brief 设置屏幕某一列的颜色
 * @param screen 屏幕控制器指针
 * @param x 列号 (0-3)
 * @param color RGB颜色
 */
void Screen_SetColumn(WS2812_Screen_t *screen, uint8_t x, RGB_t color);

/**
 * @brief 绘制水平线
 * @param screen 屏幕控制器指针
 * @param x1 起始x坐标
 * @param x2 结束x坐标
 * @param y 行号
 * @param color RGB颜色
 */
void Screen_DrawHLine(WS2812_Screen_t *screen, uint8_t x1, uint8_t x2, uint8_t y, RGB_t color);

/**
 * @brief 绘制竖直线
 * @param screen 屏幕控制器指针
 * @param x 列号
 * @param y1 起始y坐标
 * @param y2 结束y坐标
 * @param color RGB颜色
 */
void Screen_DrawVLine(WS2812_Screen_t *screen, uint8_t x, uint8_t y1, uint8_t y2, RGB_t color);

/**
 * @brief 绘制矩形边框
 * @param screen 屏幕控制器指针
 * @param x1 左上角x坐标
 * @param y1 左上角y坐标
 * @param x2 右下角x坐标
 * @param y2 右下角y坐标
 * @param color RGB颜色
 */
void Screen_DrawRect(WS2812_Screen_t *screen, uint8_t x1, uint8_t y1, uint8_t x2, uint8_t y2, RGB_t color);

/**
 * @brief 填充矩形
 * @param screen 屏幕控制器指针
 * @param x1 左上角x坐标
 * @param y1 左上角y坐标
 * @param x2 右下角x坐标
 * @param y2 右下角y坐标
 * @param color RGB颜色
 */
void Screen_FillRect(WS2812_Screen_t *screen, uint8_t x1, uint8_t y1, uint8_t x2, uint8_t y2, RGB_t color);

#endif /* __WS2812_SCREEN_H */
