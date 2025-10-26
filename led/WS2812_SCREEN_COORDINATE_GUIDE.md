# WS2812屏幕坐标系统指南

## 硬件排列与逻辑坐标映射

### 物理排列（蛇形/S形）

您的16个灯珠采用蛇形排列，串联连接：

```
第0行（从左往右）：
  灯珠0 → 灯珠1 → 灯珠2 → 灯珠3

第1行（从右往左）：
  灯珠4 ← 灯珠5 ← 灯珠6 ← 灯珠7

第2行（从左往右）：
  灯珠8 → 灯珠9 → 灯珠10 → 灯珠11

第3行（从右往左）：
  灯珠12 ← 灯珠13 ← 灯珠14 ← 灯珠15
```

### 逻辑坐标系（用户友好）

```
  列：  0      1      2      3
行0  (0,0)  (1,0)  (2,0)  (3,0)
行1  (0,1)  (1,1)  (2,1)  (3,1)
行2  (0,2)  (1,2)  (2,2)  (3,2)
行3  (0,3)  (1,3)  (2,3)  (3,3)
```

## 坐标转换算法

```c
uint8_t Screen_XY_to_Index(uint8_t x, uint8_t y)
{
    /* 奇数行（y=1,3）：从右往左 */
    if (y & 1) {
        return y * 4 + (3 - x);
    }
    /* 偶数行（y=0,2）：从左往右 */
    else {
        return y * 4 + x;
    }
}
```

### 转换示例

| 逻辑坐标(x,y) | 计算方法 | LED索引 |
|---|---|---|
| (0,0) | 0*4+0 | 0 |
| (1,0) | 0*4+1 | 1 |
| (3,0) | 0*4+3 | 3 |
| (0,1) | 1*4+(3-0) | 7 |
| (1,1) | 1*4+(3-1) | 6 |
| (3,1) | 1*4+(3-3) | 4 |
| (0,2) | 2*4+0 | 8 |
| (3,3) | 3*4+(3-3) | 12 |

## API使用示例

### 1. 初始化

```c
WS2812_t ws2812_controller;
WS2812_Screen_t screen_controller;

// 初始化LED控制器
WS2812_Init(&ws2812_controller, &htim1, TIM_CHANNEL_1);

// 初始化屏幕控制器
Screen_Init(&screen_controller, &ws2812_controller);
```

### 2. 基础操作

#### 设置单个像素
```c
RGB_t red = WS2812_RGB(255, 0, 0);
Screen_SetPixel(&screen_controller, 0, 0, red);  // 左上角设为红色
Screen_Flush(&screen_controller);                 // 刷新显示
```

#### 填充整个屏幕
```c
RGB_t blue = WS2812_RGB(0, 0, 255);
Screen_Fill(&screen_controller, blue);
Screen_Flush(&screen_controller);
```

#### 清空屏幕
```c
Screen_Clear(&screen_controller);
Screen_Flush(&screen_controller);
```

### 3. 高级绘图

#### 绘制矩形边框
```c
RGB_t green = WS2812_RGB(0, 255, 0);
Screen_DrawRect(&screen_controller, 0, 0, 3, 3, green);
Screen_Flush(&screen_controller);
```

#### 填充矩形
```c
RGB_t yellow = WS2812_RGB(255, 255, 0);
Screen_FillRect(&screen_controller, 1, 1, 2, 2, yellow);
Screen_Flush(&screen_controller);
```

#### 绘制线条
```c
// 水平线
RGB_t cyan = WS2812_RGB(0, 255, 255);
Screen_DrawHLine(&screen_controller, 0, 3, 1, cyan);

// 竖直线
Screen_DrawVLine(&screen_controller, 1, 0, 3, cyan);

Screen_Flush(&screen_controller);
```

#### 填充行/列
```c
RGB_t white = WS2812_RGB(255, 255, 255);
Screen_SetRow(&screen_controller, 0, white);     // 第0行全白
Screen_SetColumn(&screen_controller, 1, white);  // 第1列全白
Screen_Flush(&screen_controller);
```

### 4. 棋盘图案示例
```c
Screen_Clear(&screen_controller);
RGB_t yellow = WS2812_RGB(255, 255, 0);

for (uint8_t y = 0; y < 4; y++) {
    for (uint8_t x = 0; x < 4; x++) {
        if ((x + y) % 2 == 0) {
            Screen_SetPixel(&screen_controller, x, y, yellow);
        }
    }
}
Screen_Flush(&screen_controller);
```

### 5. 彩虹列示例
```c
RGB_t rainbow[] = {
    WS2812_RGB(255, 0, 0),      // 红
    WS2812_RGB(255, 127, 0),    // 橙
    WS2812_RGB(0, 255, 0),      // 绿
    WS2812_RGB(0, 0, 255),      // 蓝
};

for (uint8_t x = 0; x < 4; x++) {
    Screen_SetColumn(&screen_controller, x, rainbow[x]);
}
Screen_Flush(&screen_controller);
```

## 常用RGB颜色

```c
// 基础色
WS2812_RGB(255, 0, 0)      // 红
WS2812_RGB(0, 255, 0)      // 绿
WS2812_RGB(0, 0, 255)      // 蓝

// 混合色
WS2812_RGB(255, 255, 0)    // 黄
WS2812_RGB(255, 0, 255)    // 洋红
WS2812_RGB(0, 255, 255)    // 青

// 其他
WS2812_RGB(255, 255, 255)  // 白
WS2812_RGB(0, 0, 0)        // 黑（关闭）
WS2812_RGB(255, 165, 0)    // 橙
WS2812_RGB(128, 0, 128)    // 紫
WS2812_RGB(255, 192, 203)  // 粉红
WS2812_RGB(128, 128, 128)  // 灰
```

## 调试技巧

### 验证坐标映射是否正确

```c
// 点亮左上角 (0,0) - 应该亮灯珠0
Screen_Clear(&screen_controller);
Screen_SetPixel(&screen_controller, 0, 0, WS2812_RGB(255, 0, 0));
Screen_Flush(&screen_controller);

// 点亮右下角 (3,3) - 应该亮灯珠12
Screen_Clear(&screen_controller);
Screen_SetPixel(&screen_controller, 3, 3, WS2812_RGB(0, 255, 0));
Screen_Flush(&screen_controller);
```

### 顺序测试所有像素

```c
RGB_t color = WS2812_RGB(255, 255, 255);

for (uint8_t y = 0; y < 4; y++) {
    for (uint8_t x = 0; x < 4; x++) {
        Screen_Clear(&screen_controller);
        Screen_SetPixel(&screen_controller, x, y, color);
        Screen_Flush(&screen_controller);
        HAL_Delay(500);
    }
}
```

如果所有16个灯依次亮起，说明坐标系统工作正确。

## 性能提示

1. **缓冲机制**：所有绘图操作首先写入屏幕缓冲区，需要调用 `Screen_Flush()` 才能发送到LED
2. **减少刷新**：尽量在一次循环中完成所有绘图操作后再调用 `Screen_Flush()`
3. **内存占用**：屏幕缓冲区占用 16 × RGB_t = 16 × 3 = 48 字节

## 相关文件

- `ws2812_screen.h` - 屏幕API头文件
- `ws2812_screen.c` - 屏幕API实现
- `ws2812.h` / `ws2812.c` - 底层LED驱动
- `main.c` - 测试示例代码
