# WS2812 LED 控制器 - 快速开始指南

## 文件清单

已为你创建以下文件：

| 文件 | 说明 |
|------|------|
| `Core/Inc/ws2812.h` | WS2812驱动头文件（API定义） |
| `Core/Src/ws2812.c` | WS2812驱动实现（核心逻辑） |
| `Core/Src/ws2812_examples.c` | 9个完整应用示例 |
| `Core/Src/main.c` | 已修改支持WS2812 |
| `Core/Inc/main.h` | 已添加必要的头文件 |
| `WS2812_CONTROLLER_GUIDE.md` | 详细技术文档 |

## 硬件连接

```
STM32F103 开发板
    PA8 (TIM1_CH1) ----→ WS2812 灯板 DATA 引脚
    GND            ----→ WS2812 灯板 GND 引脚
    3.3V(可选)     ----→ WS2812 灯板 5V 引脚
                        (如果灯板有3.3V输入)
```

## 编译步骤

### 1. 确保HAL模块已启用

打开 `Core/Inc/stm32f1xx_hal_conf.h`，确保以下宏已定义：

```c
#define HAL_TIM_MODULE_ENABLED      // 定时器
#define HAL_GPIO_MODULE_ENABLED     // GPIO
#define HAL_DMA_MODULE_ENABLED      // DMA
```

### 2. 添加源文件到编译

根据你的开发环境（MDK-ARM、IAR、STM32CubeIDE等），添加以下源文件：
- `Core/Src/ws2812.c`
- `Core/Src/main.c` (已修改)

### 3. 编译和烧写

按照正常的STM32项目编译流程编译和烧写代码。

## 最简单的使用方式

### 方式1: 固定红色（复制到main.c的while循环前）

```c
int main(void)
{
    HAL_Init();
    SystemClock_Config();
    MX_GPIO_Init();
    MX_TIM1_Init();
    
    // 初始化
    WS2812_Init(&ws2812_controller, &htim1, TIM_CHANNEL_1);
    
    // 设置所有LED为红色
    RGB_t red = WS2812_RGB(255, 0, 0);
    WS2812_SetAllColors(&ws2812_controller, red);
    WS2812_Send(&ws2812_controller);
    
    while (1)
    {
        // 主循环
    }
}
```

### 方式2: 彩虹循环（复制到main.c的while循环中）

```c
int main(void)
{
    HAL_Init();
    SystemClock_Config();
    MX_GPIO_Init();
    MX_TIM1_Init();
    
    WS2812_Init(&ws2812_controller, &htim1, TIM_CHANNEL_1);
    
    while (1)
    {
        // 插入下面的代码
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
        
        WS2812_SetAllColors(&ws2812_controller, rainbow[color_index]);
        WS2812_Send(&ws2812_controller);
        
        color_index++;
        if (color_index >= 7) color_index = 0;
        
        HAL_Delay(500);  // 500ms切换一次颜色
    }
}
```

### 方式3: 使用示例文件中的函数

在 `ws2812_examples.c` 中有9个完整的示例，可以直接复制使用：

```c
// 在main.c中包含
#include "ws2812_examples.c"

// 然后在main中调用示例函数
int main(void)
{
    // ... 初始化代码 ...
    
    WS2812_Init(&ws2812_controller, &htim1, TIM_CHANNEL_1);
    
    while (1)
    {
        example_2_rainbow_cycle(&ws2812_controller);
        HAL_Delay(500);
        // 或使用其他示例
    }
}
```

## 常用API速查表

### 创建颜色
```c
RGB_t color = WS2812_RGB(红值, 绿值, 蓝值);
// 例如：WS2812_RGB(255, 0, 0) 是红色
```

### 初始化
```c
WS2812_Init(&ws2812_controller, &htim1, TIM_CHANNEL_1);
```

### 设置所有LED为同一颜色
```c
RGB_t green = WS2812_RGB(0, 255, 0);
WS2812_SetAllColors(&ws2812_controller, green);
WS2812_Send(&ws2812_controller);
```

### 设置单个LED
```c
RGB_t blue = WS2812_RGB(0, 0, 255);
WS2812_SetColor(&ws2812_controller, 3, blue);  // 设置第3个LED为蓝色
WS2812_Send(&ws2812_controller);
```

### 调整亮度
```c
WS2812_SetAllColors(&ws2812_controller, some_color);
WS2812_SetBrightness(&ws2812_controller, 50);  // 50% 亮度
WS2812_Send(&ws2812_controller);
```

### 关闭所有LED
```c
WS2812_Clear(&ws2812_controller);
```

## 颜色值参考

| 颜色 | RGB值 | 代码 |
|------|-------|------|
| 红 | (255, 0, 0) | `WS2812_RGB(255, 0, 0)` |
| 绿 | (0, 255, 0) | `WS2812_RGB(0, 255, 0)` |
| 蓝 | (0, 0, 255) | `WS2812_RGB(0, 0, 255)` |
| 黄 | (255, 255, 0) | `WS2812_RGB(255, 255, 0)` |
| 青 | (0, 255, 255) | `WS2812_RGB(0, 255, 255)` |
| 洋红 | (255, 0, 255) | `WS2812_RGB(255, 0, 255)` |
| 白 | (255, 255, 255) | `WS2812_RGB(255, 255, 255)` |
| 黑 | (0, 0, 0) | `WS2812_RGB(0, 0, 0)` |
| 橙 | (255, 127, 0) | `WS2812_RGB(255, 127, 0)` |
| 紫 | (128, 0, 128) | `WS2812_RGB(128, 0, 128)` |

## 故障排除

### LED不亮
1. 检查PA8引脚是否正确连接到LED灯板
2. 确认系统时钟设置为72MHz（代码已配置）
3. 检查是否调用了 `WS2812_Init()` 和 `WS2812_Send()`
4. 用示波器测量PA8引脚，应该看到800KHz的PWM波形

### LED显示随机颜色
- WS2812使用 **GRB** 格式（绿-红-蓝），不是RGB格式
- 确保在调用 `WS2812_Send()` 后等待足够的时间（约290μs）再发送下一条命令

### LED颜色不准确
- 这是正常的，WS2812的驱动芯片本身误差范围较大
- 可以通过调整RGB值进行颜色校准

## 性能指标

- **刷新速率**: 最高3400 Hz
- **传输延迟**: 约290 μs（8个LED）
- **同时控制LED数**: 8个
- **颜色深度**: 24bit（RGB各8bit）

## 下一步

1. **编译并烧写代码**
2. **连接硬件** - PA8 → WS2812 DATA
3. **观察效果** - LED应该显示设定的颜色
4. **修改代码** - 尝试不同的颜色和效果
5. **参考文档** - 查看 `WS2812_CONTROLLER_GUIDE.md` 了解更多细节

## 技术支持

- 详细的技术文档见: `WS2812_CONTROLLER_GUIDE.md`
- 完整示例代码见: `Core/Src/ws2812_examples.c`
- API头文件见: `Core/Inc/ws2812.h`

