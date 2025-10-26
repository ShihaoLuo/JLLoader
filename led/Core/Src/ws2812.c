#include "ws2812.h"
#include <string.h>

/* ========================= 外部变量 ======================== */
extern TIM_HandleTypeDef htim1;  /* TIM1 handle from main.c */

/* ========================= 内部变量 ======================== */
DMA_HandleTypeDef hdma_tim1_ch1;  /* DMA handle - accessible to interrupt handler */
static uint8_t dma_transfer_complete = 0;  /* DMA传输完成标志 - 初始为0（未完成） */

/* PWM缓冲区：存储CCR值（16位半字），用于DMA直接写入CCR1寄存器 */
static uint16_t pwm_dma_buffer[WS2812_BUFFER_SIZE];

/* ========================= DMA回调函数 ======================== */

/**
 * @brief DMA传输完成回调
 * @note 此函数由HAL库调用（在DMA中断中）
 */
void HAL_DMA_XferCpltCallback(DMA_HandleTypeDef *hdma)
{
    if (hdma == &hdma_tim1_ch1) {
        /* 立即停止DMA */
        HAL_DMA_Abort(&hdma_tim1_ch1);
        
        /* 禁用PWM DMA输出 */
        __HAL_TIM_DISABLE_DMA(&htim1, TIM_DMA_CC1);
        
        /* 设置CCR1为0 - 确保PWM输出为LOW */
        htim1.Instance->CCR1 = 0;
        
        /* 标记传输完成 */
        dma_transfer_complete = 1;
    }
}

/* ========================= 内部函数 ======================== */

/**
 * @brief 配置DMA用于TIM1通道1
 * @param ws2812 控制器指针
 */
static void WS2812_DMA_Init(WS2812_t *ws2812)
{
    /* 启用DMA1时钟 */
    __HAL_RCC_DMA1_CLK_ENABLE();

    /* DMA初始化 */
    hdma_tim1_ch1.Instance = DMA1_Channel2;  /* TIM1_CH1对应DMA1_Channel2 */
    hdma_tim1_ch1.Init.Direction = DMA_MEMORY_TO_PERIPH;
    hdma_tim1_ch1.Init.PeriphInc = DMA_PINC_DISABLE;
    hdma_tim1_ch1.Init.MemInc = DMA_MINC_ENABLE;
    hdma_tim1_ch1.Init.PeriphDataAlignment = DMA_PDATAALIGN_HALFWORD;  /* CCR1是16位寄存器 */
    hdma_tim1_ch1.Init.MemDataAlignment = DMA_MDATAALIGN_HALFWORD;     /* 内存也用半字 */
    hdma_tim1_ch1.Init.Mode = DMA_NORMAL;
    hdma_tim1_ch1.Init.Priority = DMA_PRIORITY_VERY_HIGH;  /* 设置为最高优先级 */

    HAL_DMA_Init(&hdma_tim1_ch1);
    
    /* 注册DMA完成回调 */
    HAL_DMA_RegisterCallback(&hdma_tim1_ch1, HAL_DMA_XFER_CPLT_CB_ID, HAL_DMA_XferCpltCallback);
    
    /* 启用DMA中断 */
    HAL_NVIC_SetPriority(DMA1_Channel2_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(DMA1_Channel2_IRQn);

    /* 配置DMA与TIM1的联系 */
    __HAL_LINKDMA(ws2812->tim_handle, hdma[TIM_DMA_ID_CC1], hdma_tim1_ch1);
}

/**
 * @brief 将RGB颜色转换为PWM编码
 * @param byte 字节值(0-255)
 * @param buffer PWM缓冲区指针（16位半字数组）
 * @param offset 缓冲区偏移（以半字计）
 */
static void WS2812_EncodeGRB(uint8_t byte, uint16_t *buffer, uint16_t offset)
{
    uint8_t i;
    
    for (i = 0; i < 8; i++) {
        if (byte & (0x80 >> i)) {
            /* 逻辑1: 占空比64% -> CCR值58 */
            buffer[offset + i] = WS2812_PWM_BIT_1;
        } else {
            /* 逻辑0: 占空比32% -> CCR值29 */
            buffer[offset + i] = WS2812_PWM_BIT_0;
        }
    }
}

/**
 * @brief 初始化WS2812控制器
 */
HAL_StatusTypeDef WS2812_Init(WS2812_t *ws2812, TIM_HandleTypeDef *tim_handle, uint32_t tim_channel)
{
    if (ws2812 == NULL || tim_handle == NULL) {
        return HAL_ERROR;
    }

    /* 保存定时器信息 */
    ws2812->tim_handle = tim_handle;
    ws2812->tim_channel = tim_channel;

    /* 初始化LED颜色为黑色 */
    memset(ws2812->colors, 0, sizeof(ws2812->colors));

    /* 配置DMA */
    WS2812_DMA_Init(ws2812);

    return HAL_OK;
}

/**
 * @brief 设置单个LED的颜色
 */
void WS2812_SetColor(WS2812_t *ws2812, uint8_t index, RGB_t color)
{
    if (index >= WS2812_LED_COUNT) {
        return;
    }

    ws2812->colors[index] = color;
}

/**
 * @brief 设置整个LED板的颜色
 */
void WS2812_SetAllColors(WS2812_t *ws2812, RGB_t color)
{
    uint8_t i;

    for (i = 0; i < WS2812_LED_COUNT; i++) {
        ws2812->colors[i] = color;
    }
}

/**
 * @brief 调整整个LED板的亮度
 */
void WS2812_SetBrightness(WS2812_t *ws2812, uint8_t brightness)
{
    uint8_t i;
    uint16_t factor;

    if (brightness > 100) {
        brightness = 100;
    }

    factor = (uint16_t)brightness * 255 / 100;

    for (i = 0; i < WS2812_LED_COUNT; i++) {
        ws2812->colors[i].R = (uint16_t)ws2812->colors[i].R * factor / 255;
        ws2812->colors[i].G = (uint16_t)ws2812->colors[i].G * factor / 255;
        ws2812->colors[i].B = (uint16_t)ws2812->colors[i].B * factor / 255;
    }
}

/**
 * @brief 生成PWM编码缓冲区
 * 
 * WS2812采用GRB格式，每个灯珠24bit数据：
 * [绿色8bit] [红色8bit] [蓝色8bit]
 * 
 * 缓冲区结构：
 * [192个半字CCR值] [100个半字复位信号(全0)]
 * 总计292个半字 = 584字节
 */
void WS2812_UpdateBuffer(WS2812_t *ws2812)
{
    uint8_t led_index;
    uint16_t buffer_pos = 0;
    uint16_t i;
    RGB_t *color;

    /* 清空DMA缓冲区 */
    memset(pwm_dma_buffer, 0, sizeof(pwm_dma_buffer));

    /* 编码LED数据 */
    for (led_index = 0; led_index < WS2812_LED_COUNT; led_index++) {
        color = &ws2812->colors[led_index];

        /* GRB顺序编码 - 编码到16位DMA缓冲区 */
        WS2812_EncodeGRB(color->G, pwm_dma_buffer, buffer_pos);
        buffer_pos += 8;

        WS2812_EncodeGRB(color->R, pwm_dma_buffer, buffer_pos);
        buffer_pos += 8;

        WS2812_EncodeGRB(color->B, pwm_dma_buffer, buffer_pos);
        buffer_pos += 8;
    }

    /* 
     * 添加复位信号（低电平）
     * WS2812需要至少50us的低电平
     * 在800KHz频率下：50us需要40个周期
     * 使用100个半字（100 * 1.25us = 125us）确保充分低电平
     */
    for (i = 0; i < WS2812_RESET_BYTES; i++) {
        pwm_dma_buffer[buffer_pos + i] = 0;
    }
}

/**
 * @brief 发送LED数据到WS2812（改进版）
 * @note 此函数不阻塞，立即返回。DMA在后台传输。
 */
HAL_StatusTypeDef WS2812_Send(WS2812_t *ws2812)
{
    HAL_StatusTypeDef status;

    if (ws2812 == NULL) {
        return HAL_ERROR;
    }

    /* 更新PWM缓冲区到DMA缓冲区 */
    WS2812_UpdateBuffer(ws2812);

    /* 禁用DMA和PWM输出 */
    __HAL_TIM_DISABLE_DMA(&htim1, TIM_DMA_CC1);
    __HAL_TIM_DISABLE(&htim1);
    HAL_DMA_Abort(&hdma_tim1_ch1);

    /* 清除DMA标志 */
    dma_transfer_complete = 0;

    /* 启动DMA传输（中断模式）
     * 源：pwm_dma_buffer（16位数组）
     * 目标：CCR1寄存器（16位）
     * 大小：缓冲区中的半字数量
     */
    status = HAL_DMA_Start_IT(
        &hdma_tim1_ch1,
        (uint32_t)pwm_dma_buffer,           /* 源地址：DMA缓冲区 */
        (uint32_t)&(htim1.Instance->CCR1), /* 目标地址：CCR1 */
        WS2812_BUFFER_SIZE                  /* 传输大小：半字数量 */
    );

    if (status != HAL_OK) {
        return status;
    }

    /* 启用定时器 */
    __HAL_TIM_ENABLE(&htim1);

    /* 启用PWM + DMA */
    __HAL_TIM_ENABLE_DMA(&htim1, TIM_DMA_CC1);

    /* 立即返回 - 不阻塞等待！ */
    return HAL_OK;
}

/**
 * @brief 关闭所有LED
 */
void WS2812_Clear(WS2812_t *ws2812)
{
    RGB_t black = {0, 0, 0};
    WS2812_SetAllColors(ws2812, black);
    WS2812_Send(ws2812);
}
