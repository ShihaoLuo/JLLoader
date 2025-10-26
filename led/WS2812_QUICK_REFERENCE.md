# WS2812屏幕系统 - 快速参考

## 灯珠排列映射表

### 物理灯珠索引 → 逻辑坐标

```
索引 0 1 2 3   →   (0,0) (1,0) (2,0) (3,0)
索引 4 5 6 7   →   (0,1) (1,1) (2,1) (3,1)
索引 8 9 ...   →   (0,2) (1,2) (2,2) (3,2)
索引 12 13...  →   (0,3) (1,3) (2,3) (3,3)
```

## 最常用的API

### 初始化（main函数开始）
```c
WS2812_Init(&ws2812_controller, &htim1, TIM_CHANNEL_1);
Screen_Init(&screen_controller, &ws2812_controller);
```

### 设置颜色
```c
// 定义颜色
RGB_t red = WS2812_RGB(255, 0, 0);
RGB_t green = WS2812_RGB(0, 255, 0);
RGB_t blue = WS2812_RGB(0, 0, 255);
RGB_t white = WS2812_RGB(255, 255, 255);
RGB_t black = WS2812_RGB(0, 0, 0);
```

### 基础操作（最常用）
```c
// 设置单个像素
Screen_SetPixel(&screen, x, y, color);

// 填充整屏
Screen_Fill(&screen, color);

// 清空屏幕
Screen_Clear(&screen);

// 刷新显示（必须调用才能看到效果）
Screen_Flush(&screen);
```

### 行/列操作
```c
Screen_SetRow(&screen, y, color);      // 填充某一行
Screen_SetColumn(&screen, x, color);   // 填充某一列
```

### 绘图函数
```c
Screen_DrawHLine(&screen, x1, x2, y, color);          // 水平线
Screen_DrawVLine(&screen, x, y1, y2, color);          // 竖直线
Screen_DrawRect(&screen, x1, y1, x2, y2, color);      // 矩形边框
Screen_FillRect(&screen, x1, y1, x2, y2, color);      // 填充矩形
```

## 常用代码片段

### 片段1：显示棋盘
```c
Screen_Clear(&screen_controller);
for (int y = 0; y < 4; y++) {
    for (int x = 0; x < 4; x++) {
        if ((x + y) % 2 == 0) {
            Screen_SetPixel(&screen_controller, x, y, WS2812_RGB(255, 255, 0));
        }
    }
}
Screen_Flush(&screen_controller);
```

### 片段2：显示彩虹
```c
RGB_t rainbow[] = {
    WS2812_RGB(255, 0, 0),      // 红
    WS2812_RGB(255, 127, 0),    // 橙
    WS2812_RGB(0, 255, 0),      // 绿
    WS2812_RGB(0, 0, 255),      // 蓝
};
for (int x = 0; x < 4; x++) {
    Screen_SetColumn(&screen_controller, x, rainbow[x]);
}
Screen_Flush(&screen_controller);
```

### 片段3：显示边框
```c
Screen_Clear(&screen_controller);
Screen_DrawRect(&screen_controller, 0, 0, 3, 3, WS2812_RGB(255, 255, 255));
Screen_Flush(&screen_controller);
```

### 片段4：闪烁测试
```c
while(1) {
    Screen_Fill(&screen_controller, WS2812_RGB(255, 0, 0));
    Screen_Flush(&screen_controller);
    HAL_Delay(500);
    
    Screen_Clear(&screen_controller);
    Screen_Flush(&screen_controller);
    HAL_Delay(500);
}
```

### 片段5：逐个点亮所有灯珠（测试）
```c
for (int y = 0; y < 4; y++) {
    for (int x = 0; x < 4; x++) {
        Screen_Clear(&screen_controller);
        Screen_SetPixel(&screen_controller, x, y, WS2812_RGB(255, 255, 255));
        Screen_Flush(&screen_controller);
        HAL_Delay(300);
    }
}
```

## 颜色快速参考

| 颜色 | RGB值 | 代码 |
|---|---|---|
| 红 | (255,0,0) | `WS2812_RGB(255, 0, 0)` |
| 绿 | (0,255,0) | `WS2812_RGB(0, 255, 0)` |
| 蓝 | (0,0,255) | `WS2812_RGB(0, 0, 255)` |
| 黄 | (255,255,0) | `WS2812_RGB(255, 255, 0)` |
| 洋红 | (255,0,255) | `WS2812_RGB(255, 0, 255)` |
| 青 | (0,255,255) | `WS2812_RGB(0, 255, 255)` |
| 白 | (255,255,255) | `WS2812_RGB(255, 255, 255)` |
| 黑 | (0,0,0) | `WS2812_RGB(0, 0, 0)` |
| 橙 | (255,165,0) | `WS2812_RGB(255, 165, 0)` |
| 紫 | (128,0,128) | `WS2812_RGB(128, 0, 128)` |
| 灰 | (128,128,128) | `WS2812_RGB(128, 128, 128)` |

## 重要提示

1. **一定要调用 `Screen_Flush()`**：屏幕的所有改变只在缓冲区中，需要 Flush 才能发送到LED
2. **坐标范围**：x 和 y 都是 0-3 的范围
3. **DMA传输**：Flush 后 DMA 会后台传输数据，不会阻塞程序

## 故障排查

### 问题1：只有部分灯珠亮
- 检查坐标系统是否正确：运行"逐个点亮测试"
- 检查 Screen_Flush() 是否被调用

### 问题2：颜色不对
- 确认使用的是 RGB 颜色（而不是 BGR）
- 检查 LED 供电是否充分

### 问题3：灯珠闪烁或显示错误
- 增加复位信号等待时间
- 检查 PWM 波形是否正确（用示波器观察PA8）
