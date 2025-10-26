#ifndef __WS2812_H
#define __WS2812_H

#include "stm32f1xx_hal.h"

/* ========================= WS2812 Protocol Parameters ======================== */
/*
 * WS2812 使用单线通信协议，基于时间的manchester编码：
 * 
 * 频率：800KHz (1bit = 1.25us)
 * 
 * 逻辑0: HIGH 0.4us + LOW 0.85us = 1.25us (占空比: 32%)
 * 逻辑1: HIGH 0.8us + LOW 0.45us = 1.25us (占空比: 64%)
 * 
 * 使用PWM编码方案：
 * - PWM频率设置为800KHz (PSC=89, ARR=89, 系统时钟72MHz)
 * - 实际周期 = 72000000 / (89+1) / (89+1) = 800KHz ✓
 * - 占空比50% => CCR=45时为逻辑0
 * - 占空比100% => CCR=89时为逻辑1
 * 
 * 或者采用更精确的方案：
 * - PWM频率为800KHz (PSC=0, ARR=89)
 * - 频率 = 72MHz / 90 = 800KHz ✓
 * - 占空比约32% (CCR=28~29) => 逻辑0
 * - 占空比约64% (CCR=57~58) => 逻辑1
 */

/* PWM频率800KHz的配置 */
#define WS2812_PWM_PSC      89      /* 预分频 */
#define WS2812_PWM_ARR      89      /* 自动重装值 */
#define WS2812_PWM_FREQ     800000  /* 目标频率800KHz */

/* PWM编码方案 (改进方案: PSC=0, ARR=89) */
#define WS2812_PWM_PSC_V2      0       /* 预分频 */
#define WS2812_PWM_ARR_V2      89      /* 自动重装值 */
#define WS2812_PWM_BIT_0       29      /* 逻辑0: 32.2%占空比 (29/90) - 对应0.4us高 + 0.85us低 */
#define WS2812_PWM_BIT_1       58      /* 逻辑1: 64.4%占空比 (58/90) - 对应0.8us高 + 0.45us低 */

#define WS2812_LED_COUNT    16      /* 灯珠数量（共16个灯珠） */
#define WS2812_BYTES_PER_LED 3      /* 每个灯珠3个字节(GRB) */
#define WS2812_BITS_PER_BYTE 8      /* 每字节8bit */
#define WS2812_DATA_SIZE    (WS2812_LED_COUNT * WS2812_BYTES_PER_LED * WS2812_BITS_PER_BYTE)
#define WS2812_RESET_BYTES  100     /* 复位信号：至少50us，800KHz下需要>62字节，使用100确保充分 */
#define WS2812_BUFFER_SIZE  (WS2812_DATA_SIZE + WS2812_RESET_BYTES)  /* 总缓冲大小：384+100=484个半字 */

/* RES(复位)信号要求 */
#define WS2812_RESET_TIME   50   /* 至少50us的低电平 */
#define WS2812_PWM_PERIOD   1.25 /* us */

/* ========================= RGB 颜色结构体 ======================== */
typedef struct {
    uint8_t R;   /* 红色分量 0-255 */
    uint8_t G;   /* 绿色分量 0-255 */
    uint8_t B;   /* 蓝色分量 0-255 */
} RGB_t;

/* ========================= WS2812 LED数组 ======================== */
typedef struct {
    RGB_t colors[WS2812_LED_COUNT];  /* 每个灯珠的颜色 */
    TIM_HandleTypeDef *tim_handle;   /* 定时器句柄 */
    uint32_t tim_channel;             /* 定时器通道 */
} WS2812_t;

/* ========================= 函数声明 ======================== */

/**
 * @brief 初始化WS2812控制器
 * @param ws2812 控制器指针
 * @param tim_handle 定时器句柄指针(TIM1)
 * @param tim_channel 定时器通道(TIM_CHANNEL_1)
 * @return HAL_OK 成功
 */
HAL_StatusTypeDef WS2812_Init(WS2812_t *ws2812, TIM_HandleTypeDef *tim_handle, uint32_t tim_channel);

/**
 * @brief 设置单个LED的颜色
 * @param ws2812 控制器指针
 * @param index LED索引(0-7)
 * @param color RGB颜色指针
 */
void WS2812_SetColor(WS2812_t *ws2812, uint8_t index, RGB_t color);

/**
 * @brief 设置整个LED板的颜色
 * @param ws2812 控制器指针
 * @param color RGB颜色
 */
void WS2812_SetAllColors(WS2812_t *ws2812, RGB_t color);

/**
 * @brief 调整整个LED板的亮度
 * @param ws2812 控制器指针
 * @param brightness 亮度百分比(0-100)
 */
void WS2812_SetBrightness(WS2812_t *ws2812, uint8_t brightness);

/**
 * @brief 生成PWM编码缓冲区
 * @param ws2812 控制器指针
 */
void WS2812_UpdateBuffer(WS2812_t *ws2812);

/**
 * @brief 通过DMA+PWM发送数据到LED
 * @param ws2812 控制器指针
 * @return HAL_OK 成功
 */
HAL_StatusTypeDef WS2812_Send(WS2812_t *ws2812);

/**
 * @brief 关闭所有LED
 * @param ws2812 控制器指针
 */
void WS2812_Clear(WS2812_t *ws2812);

/**
 * @brief 创建RGB颜色
 * @param r 红色分量(0-255)
 * @param g 绿色分量(0-255)
 * @param b 蓝色分量(0-255)
 * @return RGB颜色结构体
 */
static inline RGB_t WS2812_RGB(uint8_t r, uint8_t g, uint8_t b)
{
    RGB_t color = {r, g, b};
    return color;
}

#endif /* __WS2812_H */
