# WS2812 LED 控制器实现指南

## 概述

基于 **PWM + DMA** 的WS2812 LED控制器，支持8个RGB LED灯珠的独立颜色控制和整体亮度调节。

## 系统架构

```
STM32F103 主控
    ↓
TIM1 定时器 (800KHz PWM)
    ↓
DMA1_Channel2 (内存→PWM)
    ↓
PA8 输出引脚
    ↓
WS2812 LED灯板 (8个灯珠)
```

## WS2812 通信协议

### 时序规格

| 参数 | 值 | 描述 |
|------|-----|------|
| **通信频率** | 800 KHz | 单线通信 |
| **Bit周期** | 1.25 μs | 每个比特的时间 |
| **逻辑0** | 0.4us(HIGH) + 0.85us(LOW) | 占空比 ~32% |
| **逻辑1** | 0.8us(HIGH) + 0.45us(LOW) | 占空比 ~64% |
| **复位信号** | ≥50 μs LOW | 芯片复位 |

### 数据帧格式

每个LED灯珠使用 **GRB** (绿-红-蓝) 格式，共24bit：

```
┌─────────────┬─────────────┬─────────────┐
│ 绿色 8bit   │ 红色 8bit   │ 蓝色 8bit   │
│ (G7~G0)     │ (R7~R0)     │ (B7~B0)     │
└─────────────┴─────────────┴─────────────┘

8个LED灯珠 = 8 × 24bit = 192bit
```

### PWM编码规则

使用占空比来表示逻辑值：

```c
逻辑0: PWM占空比 = 32%  (CCR值 = 28, ARR = 89)
逻辑1: PWM占空比 = 64%  (CCR值 = 58, ARR = 89)
```

## 硬件配置

### TIM1 PWM配置

| 参数 | 值 | 计算 |
|------|-----|------|
| **系统时钟** | 72 MHz | - |
| **预分频 (PSC)** | 0 | 无分频 |
| **自动重装值 (ARR)** | 89 | - |
| **PWM频率** | 800 KHz | 72MHz / (0+1) / (89+1) = 800KHz |
| **周期** | 1.25 μs | 1 / 800KHz |

### DMA配置

| 参数 | 值 | 说明 |
|------|-----|------|
| **DMA通道** | DMA1_Channel2 | TIM1_CH1对应 |
| **数据方向** | 内存→外设 | 从RAM传输到CCR1 |
| **外设地址** | TIM1→CCR1 | PWM占空比寄存器 |
| **传输大小** | 193字节 | 192bit + 1bit复位信号 |
| **优先级** | 高 | DMA_PRIORITY_HIGH |

### GPIO配置

| 引脚 | 功能 | 模式 |
|------|------|------|
| PA8 | TIM1_CH1 | 复用推挽输出 (50MHz) |

## 软件实现

### 文件结构

```
led/Core/Inc/
├── main.h              (包含WS2812头文件)
├── ws2812.h            (WS2812驱动头文件)
└── stm32f1xx_hal_conf.h (启用TIM和DMA模块)

led/Core/Src/
├── main.c              (主程序+TIM1初始化)
├── ws2812.c            (WS2812驱动实现)
└── system_stm32f1xx.c  (系统初始化)
```

### 核心数据结构

```c
typedef struct {
    RGB_t colors[8];              // 8个LED的颜色值
    uint8_t pwm_buffer[193];      // PWM编码缓冲区
    TIM_HandleTypeDef *tim_handle;// 定时器句柄
    uint32_t tim_channel;         // 定时器通道
} WS2812_t;

typedef struct {
    uint8_t R;  // 红色 (0-255)
    uint8_t G;  // 绿色 (0-255)
    uint8_t B;  // 蓝色 (0-255)
} RGB_t;
```

## API 使用指南

### 初始化

```c
WS2812_Init(&ws2812_controller, &htim1, TIM_CHANNEL_1);
```

- 初始化DMA和PWM
- 设置所有LED为黑色
- 准备传输缓冲区

### 设置单个LED颜色

```c
RGB_t color = WS2812_RGB(255, 0, 0);    // 红色
WS2812_SetColor(&ws2812_controller, 0, color);  // LED0设为红色
WS2812_Send(&ws2812_controller);        // 发送到LED
```

### 设置整个LED板颜色

```c
RGB_t green = WS2812_RGB(0, 255, 0);
WS2812_SetAllColors(&ws2812_controller, green);
WS2812_Send(&ws2812_controller);
```

### 亮度调节

```c
// 设置颜色
RGB_t blue = WS2812_RGB(0, 0, 255);
WS2812_SetAllColors(&ws2812_controller, blue);

// 调整亮度为50%
WS2812_SetBrightness(&ws2812_controller, 50);

// 发送到LED
WS2812_Send(&ws2812_controller);
```

### 关闭所有LED

```c
WS2812_Clear(&ws2812_controller);
```

## 实现细节

### PWM编码过程

```c
/*
 * 将一个字节编码成8个PWM值
 * 例如: 字节值 = 0xA5 (10100101)
 */

uint8_t byte = 0xA5;
uint8_t pwm_values[8];

for (int i = 0; i < 8; i++) {
    if (byte & (0x80 >> i)) {
        // 第i位为1
        pwm_values[i] = 58;  // 占空比64%
    } else {
        // 第i位为0
        pwm_values[i] = 28;  // 占空比32%
    }
}
// 结果: [58, 28, 58, 28, 28, 58, 28, 58]
```

### DMA传输流程

```
1. 更新PWM缓冲区 (WS2812_UpdateBuffer)
   ↓
2. 启动DMA传输 (WS2812_Send)
   ↓
3. DMA从内存读取缓冲区数据
   ↓
4. DMA逐字节写入TIM1→CCR1
   ↓
5. PWM输出动态改变占空比
   ↓
6. LED接收PWM信号并解码
```

### GRB编码示例

```
LED0: 红色 R=100, G=200, B=150
      ↓
GRB编码顺序: [G=200] [R=100] [B=150]
      ↓
二进制: [11001000] [01100100] [10010110]
      ↓
PWM编码 (每位32%或64%占空比)
```

## 应用示例

### 示例1: 彩虹渐变

```c
// 循环显示7种基本颜色
RGB_t rainbow[] = {
    WS2812_RGB(255, 0, 0),      // 红
    WS2812_RGB(255, 127, 0),    // 橙
    WS2812_RGB(255, 255, 0),    // 黄
    WS2812_RGB(0, 255, 0),      // 绿
    WS2812_RGB(0, 0, 255),      // 蓝
    WS2812_RGB(75, 0, 130),     // 靛
    WS2812_RGB(148, 0, 211),    // 紫
};

while (1) {
    for (int i = 0; i < 7; i++) {
        WS2812_SetAllColors(&ws2812_controller, rainbow[i]);
        WS2812_Send(&ws2812_controller);
        HAL_Delay(500);
    }
}
```

### 示例2: 亮度渐变

```c
while (1) {
    RGB_t color = WS2812_RGB(255, 0, 0);  // 红色
    
    // 亮度从0%增加到100%
    for (int brightness = 0; brightness <= 100; brightness += 10) {
        WS2812_SetAllColors(&ws2812_controller, color);
        WS2812_SetBrightness(&ws2812_controller, brightness);
        WS2812_Send(&ws2812_controller);
        HAL_Delay(100);
    }
}
```

### 示例3: 独立LED控制

```c
// 设置每个LED不同的颜色
RGB_t colors[] = {
    WS2812_RGB(255, 0, 0),    // LED0: 红
    WS2812_RGB(0, 255, 0),    // LED1: 绿
    WS2812_RGB(0, 0, 255),    // LED2: 蓝
    WS2812_RGB(255, 255, 0),  // LED3: 黄
    WS2812_RGB(255, 0, 255),  // LED4: 洋红
    WS2812_RGB(0, 255, 255),  // LED5: 青
    WS2812_RGB(255, 255, 255),// LED6: 白
    WS2812_RGB(128, 128, 128),// LED7: 灰
};

for (int i = 0; i < 8; i++) {
    WS2812_SetColor(&ws2812_controller, i, colors[i]);
}
WS2812_Send(&ws2812_controller);
```

## 常见问题

### Q1: LED不亮
- 检查PA8引脚连接
- 验证系统时钟是否正确配置为72MHz
- 检查DMA是否正确初始化
- 确认WS2812_Send()已被调用

### Q2: LED显示颜色不对
- WS2812使用GRB格式，不是RGB格式
- 检查PWM占空比计算是否正确
- 使用示波器验证PA8的PWM波形

### Q3: LED随机闪烁
- 检查DMA缓冲区是否溢出
- 确认DMA优先级设置为HIGH
- 增加PWM缓冲区大小

### Q4: 如何实现实时颜色更新
```c
// 在主循环中动态更新颜色
while (1) {
    // 获取新颜色值 (例如从UART或传感器)
    RGB_t new_color = GetColorFromSensor();
    
    // 更新LED
    WS2812_SetAllColors(&ws2812_controller, new_color);
    WS2812_Send(&ws2812_controller);
    
    // 控制更新频率
    HAL_Delay(50);
}
```

## 性能指标

| 指标 | 值 |
|------|-----|
| **通信速率** | 800 KHz |
| **8个LED传输时间** | 192bit / 800kHz ≈ 240 μs |
| **复位信号** | ≥50 μs |
| **总传输时间** | ≈290 μs |
| **刷新率** | 最高 ~3400 Hz |
| **DMA缓冲大小** | 193 字节 |

## 编译配置

### stm32f1xx_hal_conf.h 需启用

```c
#define HAL_MODULE_ENABLED
#define HAL_TIM_MODULE_ENABLED      // 定时器
#define HAL_GPIO_MODULE_ENABLED     // GPIO
#define HAL_DMA_MODULE_ENABLED      // DMA
#define HAL_RCC_MODULE_ENABLED      // RCC
#define HAL_CORTEX_MODULE_ENABLED   // Cortex
#define HAL_EXTI_MODULE_ENABLED     // EXTI
#define HAL_PWR_MODULE_ENABLED      // PWR
```

## 总结

这个WS2812控制器实现了：
✓ 高效的PWM+DMA传输方案
✓ 支持8个独立LED的颜色控制
✓ 整体亮度调节功能
✓ 简单易用的API接口
✓ 精准的WS2812协议时序

