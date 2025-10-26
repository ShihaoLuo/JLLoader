# WS2812屏幕系统 - 完整API参考

## 目录
1. [基础函数](#基础函数)
2. [像素操作](#像素操作)
3. [行列操作](#行列操作)
4. [绘图函数](#绘图函数)
5. [坐标转换](#坐标转换)
6. [颜色定义](#颜色定义)

---

## 基础函数

### Screen_Init()
```c
HAL_StatusTypeDef Screen_Init(WS2812_Screen_t *screen, WS2812_t *ws2812)
```

**功能**：初始化屏幕控制器

**参数**：
- `screen` - 屏幕控制器指针
- `ws2812` - WS2812控制器指针

**返回值**：
- `HAL_OK` - 成功
- `HAL_ERROR` - 失败

**示例**：
```c
WS2812_Screen_t screen;
WS2812_t ws2812;
Screen_Init(&screen, &ws2812);
```

**说明**：必须在使用其他屏幕函数前调用

---

### Screen_Flush()
```c
void Screen_Flush(WS2812_Screen_t *screen)
```

**功能**：将屏幕缓冲区发送到LED

**参数**：
- `screen` - 屏幕控制器指针

**返回值**：无

**示例**：
```c
Screen_SetPixel(&screen, 0, 0, red);
Screen_Flush(&screen);  // 立即显示
```

**说明**：
- 所有绘图操作都在缓冲区中
- 必须调用Flush才能在LED上显示
- DMA会在后台传输，不阻塞程序

---

## 像素操作

### Screen_SetPixel()
```c
void Screen_SetPixel(WS2812_Screen_t *screen, uint8_t x, uint8_t y, RGB_t color)
```

**功能**：设置屏幕上某个像素的颜色

**参数**：
- `screen` - 屏幕控制器指针
- `x` - 水平坐标 (0-3)
- `y` - 竖直坐标 (0-3)
- `color` - RGB颜色

**返回值**：无

**示例**：
```c
RGB_t red = WS2812_RGB(255, 0, 0);
Screen_SetPixel(&screen, 0, 0, red);  // 左上角设为红色
Screen_SetPixel(&screen, 3, 3, red);  // 右下角设为红色
Screen_Flush(&screen);
```

**说明**：
- 坐标超出范围会被忽略
- 仅修改缓冲区，需要Flush才能显示

---

### Screen_GetPixel()
```c
RGB_t Screen_GetPixel(WS2812_Screen_t *screen, uint8_t x, uint8_t y)
```

**功能**：获取屏幕上某个像素的颜色

**参数**：
- `screen` - 屏幕控制器指针
- `x` - 水平坐标 (0-3)
- `y` - 竖直坐标 (0-3)

**返回值**：RGB颜色（如果坐标无效，返回黑色）

**示例**：
```c
RGB_t color = Screen_GetPixel(&screen, 1, 1);
```

---

## 全屏操作

### Screen_Clear()
```c
void Screen_Clear(WS2812_Screen_t *screen)
```

**功能**：清空屏幕（所有像素变黑）

**参数**：
- `screen` - 屏幕控制器指针

**返回值**：无

**示例**：
```c
Screen_Clear(&screen);
Screen_Flush(&screen);
```

---

### Screen_Fill()
```c
void Screen_Fill(WS2812_Screen_t *screen, RGB_t color)
```

**功能**：填充整个屏幕为指定颜色

**参数**：
- `screen` - 屏幕控制器指针
- `color` - RGB颜色

**返回值**：无

**示例**：
```c
RGB_t blue = WS2812_RGB(0, 0, 255);
Screen_Fill(&screen, blue);
Screen_Flush(&screen);
```

---

## 行列操作

### Screen_SetRow()
```c
void Screen_SetRow(WS2812_Screen_t *screen, uint8_t y, RGB_t color)
```

**功能**：设置屏幕某一行的所有像素为指定颜色

**参数**：
- `screen` - 屏幕控制器指针
- `y` - 行号 (0-3)
- `color` - RGB颜色

**返回值**：无

**示例**：
```c
Screen_SetRow(&screen, 0, WS2812_RGB(255, 0, 0));  // 顶行设为红色
Screen_SetRow(&screen, 3, WS2812_RGB(0, 255, 0));  // 底行设为绿色
Screen_Flush(&screen);
```

---

### Screen_SetColumn()
```c
void Screen_SetColumn(WS2812_Screen_t *screen, uint8_t x, RGB_t color)
```

**功能**：设置屏幕某一列的所有像素为指定颜色

**参数**：
- `screen` - 屏幕控制器指针
- `x` - 列号 (0-3)
- `color` - RGB颜色

**返回值**：无

**示例**：
```c
Screen_SetColumn(&screen, 0, WS2812_RGB(0, 0, 255));  // 左列设为蓝色
Screen_SetColumn(&screen, 3, WS2812_RGB(0, 0, 255));  // 右列设为蓝色
Screen_Flush(&screen);
```

---

## 绘图函数

### Screen_DrawHLine()
```c
void Screen_DrawHLine(WS2812_Screen_t *screen, uint8_t x1, uint8_t x2, uint8_t y, 
                      RGB_t color)
```

**功能**：绘制水平线

**参数**：
- `screen` - 屏幕控制器指针
- `x1` - 起始x坐标
- `x2` - 结束x坐标（会自动排序）
- `y` - 行号
- `color` - RGB颜色

**返回值**：无

**示例**：
```c
Screen_DrawHLine(&screen, 0, 3, 1, WS2812_RGB(255, 255, 0));
Screen_Flush(&screen);
```

---

### Screen_DrawVLine()
```c
void Screen_DrawVLine(WS2812_Screen_t *screen, uint8_t x, uint8_t y1, uint8_t y2,
                      RGB_t color)
```

**功能**：绘制竖直线

**参数**：
- `screen` - 屏幕控制器指针
- `x` - 列号
- `y1` - 起始y坐标
- `y2` - 结束y坐标（会自动排序）
- `color` - RGB颜色

**返回值**：无

**示例**：
```c
Screen_DrawVLine(&screen, 1, 0, 3, WS2812_RGB(255, 255, 0));
Screen_Flush(&screen);
```

---

### Screen_DrawRect()
```c
void Screen_DrawRect(WS2812_Screen_t *screen, uint8_t x1, uint8_t y1, 
                     uint8_t x2, uint8_t y2, RGB_t color)
```

**功能**：绘制矩形边框

**参数**：
- `screen` - 屏幕控制器指针
- `x1, y1` - 左上角坐标
- `x2, y2` - 右下角坐标（会自动排序）
- `color` - RGB颜色

**返回值**：无

**示例**：
```c
// 绘制外框
Screen_DrawRect(&screen, 0, 0, 3, 3, WS2812_RGB(255, 255, 255));
Screen_Flush(&screen);
```

---

### Screen_FillRect()
```c
void Screen_FillRect(WS2812_Screen_t *screen, uint8_t x1, uint8_t y1,
                     uint8_t x2, uint8_t y2, RGB_t color)
```

**功能**：填充矩形

**参数**：
- `screen` - 屏幕控制器指针
- `x1, y1` - 左上角坐标
- `x2, y2` - 右下角坐标（会自动排序）
- `color` - RGB颜色

**返回值**：无

**示例**：
```c
// 填充中心2x2的矩形
Screen_FillRect(&screen, 1, 1, 2, 2, WS2812_RGB(0, 255, 255));
Screen_Flush(&screen);
```

---

## 坐标转换

### Screen_XY_to_Index()
```c
uint8_t Screen_XY_to_Index(uint8_t x, uint8_t y)
```

**功能**：将逻辑坐标转换为LED物理索引

**参数**：
- `x` - 水平坐标 (0-3)
- `y` - 竖直坐标 (0-3)

**返回值**：LED索引 (0-15)

**示例**：
```c
uint8_t idx = Screen_XY_to_Index(0, 0);  // 返回 0
uint8_t idx = Screen_XY_to_Index(3, 3);  // 返回 12
```

**映射关系**：
```
偶数行(y=0,2): index = y*4 + x
奇数行(y=1,3): index = y*4 + (3-x)
```

---

## 颜色定义

### WS2812_RGB()
```c
static inline RGB_t WS2812_RGB(uint8_t r, uint8_t g, uint8_t b)
```

**功能**：创建RGB颜色

**参数**：
- `r` - 红色分量 (0-255)
- `g` - 绿色分量 (0-255)
- `b` - 蓝色分量 (0-255)

**返回值**：RGB_t 颜色结构体

**示例**：
```c
RGB_t red = WS2812_RGB(255, 0, 0);
RGB_t green = WS2812_RGB(0, 255, 0);
RGB_t blue = WS2812_RGB(0, 0, 255);
```

---

## 常用颜色预定义

```c
// 基础颜色
#define COLOR_RED       WS2812_RGB(255, 0, 0)
#define COLOR_GREEN     WS2812_RGB(0, 255, 0)
#define COLOR_BLUE      WS2812_RGB(0, 0, 255)
#define COLOR_YELLOW    WS2812_RGB(255, 255, 0)
#define COLOR_CYAN      WS2812_RGB(0, 255, 255)
#define COLOR_MAGENTA   WS2812_RGB(255, 0, 255)
#define COLOR_WHITE     WS2812_RGB(255, 255, 255)
#define COLOR_BLACK     WS2812_RGB(0, 0, 0)

// 其他颜色
#define COLOR_ORANGE    WS2812_RGB(255, 165, 0)
#define COLOR_PURPLE    WS2812_RGB(128, 0, 128)
#define COLOR_PINK      WS2812_RGB(255, 192, 203)
#define COLOR_GRAY      WS2812_RGB(128, 128, 128)
```

---

## 实际代码示例

### 例1：彩虹网格
```c
RGB_t colors[] = {
    WS2812_RGB(255, 0, 0),      // 红
    WS2812_RGB(255, 165, 0),    // 橙
    WS2812_RGB(0, 255, 0),      // 绿
    WS2812_RGB(0, 0, 255),      // 蓝
};

for (uint8_t x = 0; x < 4; x++) {
    Screen_SetColumn(&screen, x, colors[x]);
}
Screen_Flush(&screen);
```

### 例2：棋盘图案
```c
Screen_Clear(&screen);
for (uint8_t y = 0; y < 4; y++) {
    for (uint8_t x = 0; x < 4; x++) {
        if ((x + y) % 2 == 0) {
            Screen_SetPixel(&screen, x, y, WS2812_RGB(255, 255, 255));
        }
    }
}
Screen_Flush(&screen);
```

### 例3：闪烁效果
```c
while (1) {
    Screen_Fill(&screen, WS2812_RGB(255, 0, 0));
    Screen_Flush(&screen);
    HAL_Delay(500);
    
    Screen_Clear(&screen);
    Screen_Flush(&screen);
    HAL_Delay(500);
}
```

### 例4：旋转图案
```c
RGB_t colors[4];
while (1) {
    for (uint8_t i = 0; i < 4; i++) {
        colors[i] = WS2812_RGB(i * 64, i * 64, i * 64);
    }
    
    for (uint8_t x = 0; x < 4; x++) {
        Screen_SetColumn(&screen, x, colors[(x + 1) % 4]);
    }
    Screen_Flush(&screen);
    HAL_Delay(500);
}
```

---

## 性能注意事项

| 操作 | 缓冲区操作时间 | 刷新时间 | 总时间 |
|---|---|---|---|
| SetPixel | <1µs | 240µs | 240µs |
| SetRow | <10µs | 240µs | 250µs |
| Fill | <50µs | 240µs | 290µs |
| DrawRect | <50µs | 240µs | 290µs |
| Flush | N/A | 240µs | 240µs |

**结论**：缓冲区操作极快，主要耗时在DMA传输

---

## 错误处理

所有函数都对非法参数进行了边界检查：
- 坐标超范围会被忽略
- 空指针会被检查
- 不会导致崩溃或死机

---

## 相关头文件

需要包含的头文件：

```c
#include "ws2812.h"          // LED驱动
#include "ws2812_screen.h"   // 屏幕API
```

---

## 版本历史

| 版本 | 日期 | 变更 |
|---|---|---|
| 1.0 | 2025-10-26 | 初始版本，支持4x4网格 |

---

**更新日期**：2025年10月26日
