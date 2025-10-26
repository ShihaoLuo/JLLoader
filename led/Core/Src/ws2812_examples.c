/**
  ******************************************************************************
  * @file    ws2812_examples.c
  * @brief   WS2812 LED 控制示例程序
  * 
  * 这个文件包含多个WS2812 LED控制的实际示例
  * 可以根据需要复制到main.c中使用
  ******************************************************************************
  */

#include "main.h"
#include "ws2812.h"

/* ======================== 示例1: 基础颜色设置 ======================== */
/**
 * @brief 示例1: 设置整个LED板为固定颜色
 * 
 * 将8个LED灯珠全部设置为红色，然后发送
 * 复制此函数到main函数中调用：example_1_fixed_color();
 */
void example_1_fixed_color(WS2812_t *ws2812)
{
    /* 创建红色 */
    RGB_t red = WS2812_RGB(255, 0, 0);
    
    /* 设置所有LED为红色 */
    WS2812_SetAllColors(ws2812, red);
    
    /* 发送数据到LED */
    WS2812_Send(ws2812);
}

/* ======================== 示例2: 彩虹渐变 ======================== */
/**
 * @brief 示例2: 彩虹颜色循环显示
 * 
 * 依次显示7种基本颜色，每500ms切换一次
 * 复制此函数到main函数中调用：
 *   while (1) {
 *       example_2_rainbow_cycle(&ws2812_controller);
 *   }
 */
void example_2_rainbow_cycle(WS2812_t *ws2812)
{
    static int color_index = 0;
    
    RGB_t rainbow[] = {
        WS2812_RGB(255, 0, 0),      // 红
        WS2812_RGB(255, 127, 0),    // 橙
        WS2812_RGB(255, 255, 0),    // 黄
        WS2812_RGB(0, 255, 0),      // 绿
        WS2812_RGB(0, 0, 255),      // 蓝
        WS2812_RGB(75, 0, 130),     // 靛
        WS2812_RGB(148, 0, 211),    // 紫
    };
    
    WS2812_SetAllColors(ws2812, rainbow[color_index]);
    WS2812_Send(ws2812);
    
    color_index++;
    if (color_index >= 7) {
        color_index = 0;
    }
}

/* ======================== 示例3: 亮度渐变 ======================== */
/**
 * @brief 示例3: 单一颜色的亮度渐变
 * 
 * 红色从最暗渐变到最亮，然后反向
 * 复制此函数到main函数中调用：
 *   while (1) {
 *       example_3_brightness_fade(&ws2812_controller);
 *   }
 */
void example_3_brightness_fade(WS2812_t *ws2812)
{
    static int brightness = 0;
    static int direction = 1;  // 1: 增加, -1: 减少
    
    RGB_t red = WS2812_RGB(255, 0, 0);
    
    WS2812_SetAllColors(ws2812, red);
    WS2812_SetBrightness(ws2812, brightness);
    WS2812_Send(ws2812);
    
    brightness += direction * 5;
    
    if (brightness >= 100) {
        brightness = 100;
        direction = -1;
    } else if (brightness <= 0) {
        brightness = 0;
        direction = 1;
    }
}

/* ======================== 示例4: 独立LED控制 ======================== */
/**
 * @brief 示例4: 每个LED显示不同颜色
 * 
 * 依次设置每个LED为不同颜色，形成彩虹效果
 */
void example_4_individual_leds(WS2812_t *ws2812)
{
    RGB_t colors[] = {
        WS2812_RGB(255, 0, 0),      // LED0: 红
        WS2812_RGB(255, 127, 0),    // LED1: 橙
        WS2812_RGB(255, 255, 0),    // LED2: 黄
        WS2812_RGB(0, 255, 0),      // LED3: 绿
        WS2812_RGB(0, 0, 255),      // LED4: 蓝
        WS2812_RGB(75, 0, 130),     // LED5: 靛
        WS2812_RGB(148, 0, 211),    // LED6: 紫
        WS2812_RGB(255, 255, 255),  // LED7: 白
    };
    
    for (int i = 0; i < 8; i++) {
        WS2812_SetColor(ws2812, i, colors[i]);
    }
    
    WS2812_Send(ws2812);
}

/* ======================== 示例5: 流动效果 ======================== */
/**
 * @brief 示例5: LED流动效果
 * 
 * 一个红色LED在8个位置间循环移动
 * 复制此函数到main函数中调用：
 *   while (1) {
 *       example_5_flowing_light(&ws2812_controller);
 *       HAL_Delay(100);
 *   }
 */
void example_5_flowing_light(WS2812_t *ws2812)
{
    static int position = 0;
    RGB_t black = WS2812_RGB(0, 0, 0);
    RGB_t red = WS2812_RGB(255, 0, 0);
    
    /* 清除所有LED */
    WS2812_SetAllColors(ws2812, black);
    
    /* 点亮当前位置 */
    WS2812_SetColor(ws2812, position, red);
    
    WS2812_Send(ws2812);
    
    /* 移动到下一个位置 */
    position++;
    if (position >= 8) {
        position = 0;
    }
}

/* ======================== 示例6: 呼吸灯效果 ======================== */
/**
 * @brief 示例6: 呼吸灯效果（缓慢亮度变化）
 * 
 * 复制此函数到main函数中调用：
 *   while (1) {
 *       example_6_breathing_light(&ws2812_controller);
 *       HAL_Delay(50);
 *   }
 */
void example_6_breathing_light(WS2812_t *ws2812)
{
    static int brightness = 0;
    static int direction = 1;
    
    RGB_t green = WS2812_RGB(0, 255, 0);
    
    WS2812_SetAllColors(ws2812, green);
    WS2812_SetBrightness(ws2812, brightness);
    WS2812_Send(ws2812);
    
    brightness += direction;
    
    if (brightness >= 100) {
        brightness = 100;
        direction = -1;
    } else if (brightness <= 0) {
        brightness = 0;
        direction = 1;
    }
}

/* ======================== 示例7: 彩虹流动 ======================== */
/**
 * @brief 示例7: 彩虹颜色依次流动
 * 
 * 创建一个彩虹流动效果，7种颜色在8个LED间循环
 * 复制此函数到main函数中调用：
 *   while (1) {
 *       example_7_rainbow_flow(&ws2812_controller);
 *       HAL_Delay(100);
 *   }
 */
void example_7_rainbow_flow(WS2812_t *ws2812)
{
    static int offset = 0;
    
    RGB_t rainbow[] = {
        WS2812_RGB(255, 0, 0),      // 红
        WS2812_RGB(255, 127, 0),    // 橙
        WS2812_RGB(255, 255, 0),    // 黄
        WS2812_RGB(0, 255, 0),      // 绿
        WS2812_RGB(0, 0, 255),      // 蓝
        WS2812_RGB(75, 0, 130),     // 靛
        WS2812_RGB(148, 0, 211),    // 紫
    };
    
    for (int i = 0; i < 8; i++) {
        int color_index = (i + offset) % 7;
        WS2812_SetColor(ws2812, i, rainbow[color_index]);
    }
    
    WS2812_Send(ws2812);
    
    offset++;
    if (offset >= 7) {
        offset = 0;
    }
}

/* ======================== 示例8: 追踪光 ======================== */
/**
 * @brief 示例8: 追踪光效果（尾部逐渐淡出）
 * 
 * 复制此函数到main函数中调用：
 *   while (1) {
 *       example_8_chasing_light(&ws2812_controller);
 *       HAL_Delay(100);
 *   }
 */
void example_8_chasing_light(WS2812_t *ws2812)
{
    static int position = 0;
    
    RGB_t colors[8];
    
    /* 初始化颜色数组 */
    for (int i = 0; i < 8; i++) {
        colors[i] = WS2812_RGB(0, 0, 0);  /* 黑色 */
    }
    
    /* 设置追踪点和尾部 */
    colors[position] = WS2812_RGB(255, 0, 0);           /* 头部: 红色 */
    
    int prev1 = (position - 1 + 8) % 8;
    int prev2 = (position - 2 + 8) % 8;
    
    RGB_t tail_bright = WS2812_RGB(255, 0, 0);
    RGB_t tail_dim = WS2812_RGB(128, 0, 0);
    
    /* 这里需要修改颜色值，需要增强支持 */
    colors[prev1] = tail_bright;
    colors[prev2] = tail_dim;
    
    for (int i = 0; i < 8; i++) {
        WS2812_SetColor(ws2812, i, colors[i]);
    }
    
    WS2812_Send(ws2812);
    
    position++;
    if (position >= 8) {
        position = 0;
    }
}

/* ======================== 示例9: 自定义调色板 ======================== */
/**
 * @brief 示例9: 使用自定义调色板
 * 
 * 定义一个调色板，依次循环显示调色板中的颜色
 */
void example_9_custom_palette(WS2812_t *ws2812)
{
    static int palette_index = 0;
    
    /* 自定义调色板 */
    const RGB_t palette[] = {
        {255, 0, 0},      /* 红 */
        {255, 255, 0},    /* 黄 */
        {0, 255, 0},      /* 绿 */
        {0, 255, 255},    /* 青 */
        {0, 0, 255},      /* 蓝 */
        {255, 0, 255},    /* 洋红 */
        {255, 127, 0},    /* 橙 */
        {192, 192, 192},  /* 银 */
    };
    
    const int palette_size = sizeof(palette) / sizeof(palette[0]);
    
    WS2812_SetAllColors(ws2812, palette[palette_index]);
    WS2812_Send(ws2812);
    
    palette_index++;
    if (palette_index >= palette_size) {
        palette_index = 0;
    }
}

/* ======================== 在main.c中使用示例 ======================== */
/*

int main(void)
{
    HAL_Init();
    SystemClock_Config();
    MX_GPIO_Init();
    MX_TIM1_Init();
    
    // 初始化WS2812控制器
    WS2812_Init(&ws2812_controller, &htim1, TIM_CHANNEL_1);
    
    // 选择你想要的示例效果：
    
    // 主循环示例1: 固定红色
    // example_1_fixed_color(&ws2812_controller);
    
    // 主循环示例2-9: 需要在while循环中持续调用
    while (1)
    {
        // 彩虹循环
        example_2_rainbow_cycle(&ws2812_controller);
        HAL_Delay(500);
        
        // 或 呼吸灯
        // example_6_breathing_light(&ws2812_controller);
        // HAL_Delay(50);
        
        // 或 流动效果
        // example_5_flowing_light(&ws2812_controller);
        // HAL_Delay(100);
    }
}

*/

/* ======================== 颜色常量定义 ======================== */
/**
 * @brief 常用颜色定义（可复制到头文件中）
 */

#define COLOR_RED           WS2812_RGB(255, 0, 0)
#define COLOR_GREEN         WS2812_RGB(0, 255, 0)
#define COLOR_BLUE          WS2812_RGB(0, 0, 255)
#define COLOR_WHITE         WS2812_RGB(255, 255, 255)
#define COLOR_BLACK         WS2812_RGB(0, 0, 0)
#define COLOR_YELLOW        WS2812_RGB(255, 255, 0)
#define COLOR_CYAN          WS2812_RGB(0, 255, 255)
#define COLOR_MAGENTA       WS2812_RGB(255, 0, 255)
#define COLOR_ORANGE        WS2812_RGB(255, 127, 0)
#define COLOR_PURPLE        WS2812_RGB(128, 0, 128)

