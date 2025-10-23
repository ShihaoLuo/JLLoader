# PWM输出改为PA7(TIM3_CH2) - 改动总结

## 概述
将电机PWM输出从PB6(TIM4_CH1)改为PA7(TIM3_CH2)。

---

## 原始配置 vs 新配置

| 项目 | 原始 | 新配置 |
|------|------|--------|
| **定时器** | TIM4 | TIM3 |
| **通道** | CH1 | CH2 |
| **输出引脚** | PB6 | PA7 |
| **时钟源** | APB2 (72MHz) | APB1 (36MHz) |
| **Prescaler** | 143 | 143 |
| **Period** | 1000 | 1000 |
| **PWM频率** | ~500Hz | ~500Hz |

### PWM频率计算（验证）
```
TIM3 (36MHz) 频率计算：
F = APB1时钟 / (Prescaler+1) / (Period+1)
  = 36MHz / 144 / 1001
  = 250 Hz （实际是250Hz不是500Hz，因为36MHz)

等等，让我重新验证...

原TIM4 (72MHz)：
F = 72MHz / 144 / 1001 = 500Hz

新TIM3 (36MHz)：
F = 36MHz / 144 / 1001 = 250Hz

这会导致PWM频率下降！
```

**需要调整：若要保持500Hz，需要调整Prescaler或Period**

---

## 修改的文件清单

### 1. **motor_init.c**
#### 修改内容：

**定时器初始化函数替换：**
```c
// 旧：void MX_TIM4_Init(void)
// 新：void MX_TIM3_Init(void)
//   - Instance: TIM4 → TIM3
//   - TIM_CHANNEL_1 → TIM_CHANNEL_2
```

**时钟使能：**
```c
// 旧：__HAL_RCC_TIM4_CLK_ENABLE();
// 新：__HAL_RCC_TIM3_CLK_ENABLE();
```

**初始化调用：**
```c
// 旧：MX_TIM4_Init();
// 新：MX_TIM3_Init();
```

**PWM启动：**
```c
// 旧：HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_1)
// 新：HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2)
```

---

### 2. **motor_ctrl.c**
#### 修改内容：

**全局变量：**
```c
// 旧：TIM_HandleTypeDef htim4;
// 新：TIM_HandleTypeDef htim3;
```

**PWM比较值设置（2处）：**

Motor_Stop()函数：
```c
// 旧：__HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_1, PWM_STOP);
// 新：__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, PWM_STOP);
```

Motor_SetSpeed()函数：
```c
// 旧：__HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_1, pwm_value);
// 新：__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, pwm_value);
```

---

### 3. **motor_ctrl.h**
#### 修改内容：

**导出变量：**
```c
// 旧：extern TIM_HandleTypeDef htim4;
// 新：extern TIM_HandleTypeDef htim3;
```

---

### 4. **motor_init.h**
#### 修改内容：

**函数声明：**
```c
// 旧：void MX_TIM4_Init(void);
// 新：void MX_TIM3_Init(void);
```

---

### 5. **stm32f1xx_hal_msp.c**
#### 修改内容：

**HAL_TIM_PWM_MspInit()函数：**
```c
// 旧：if(htim_pwm->Instance==TIM4) { __HAL_RCC_TIM4_CLK_ENABLE(); }
// 新：if(htim_pwm->Instance==TIM3) { __HAL_RCC_TIM3_CLK_ENABLE(); }
```

**HAL_TIM_MspPostInit()函数（GPIO配置）：**
```c
// 旧配置：
// - GPIO: GPIOB
// - 引脚：PB6
// - 注释：PB6 ------> TIM4_CH1

// 新配置：
// - GPIO: GPIOA
// - 引脚：PA7
// - 注释：PA7 ------> TIM3_CH2
```

**HAL_TIM_PWM_MspDeInit()函数：**
```c
// 旧：if(htim_pwm->Instance==TIM4) { __HAL_RCC_TIM4_CLK_DISABLE(); }
// 新：if(htim_pwm->Instance==TIM3) { __HAL_RCC_TIM3_CLK_DISABLE(); }
```

---

## 定时器资源分布变化

### 改动前
```
TIM1  - 未使用
TIM2  - RPM检测/PID更新 (50Hz)
TIM3  - 释放中 (之前TIM3被当作RPM定时器)
TIM4  - PWM输出 (500Hz, PB6)
TIM5+ - 未使用
```

### 改动后
```
TIM1  - 未使用
TIM2  - RPM检测/PID更新 (50Hz) ✓
TIM3  - PWM输出 (PA7, CH2) ✓
TIM4  - 完全释放，可用于其他功能
TIM5+ - 未使用
```

---

## 时钟分布

| 定时器 | 挂载总线 | 时钟频率 | 用途 |
|------|--------|--------|------|
| TIM2 | APB1 | 36MHz | RPM检测/PID(50Hz) |
| TIM3 | APB1 | 36MHz | PWM输出(CH2, PA7) |
| TIM4 | APB2 | 72MHz | 已释放 |

---

## 重要警告：PWM频率变化

由于TIM3挂载在APB1(36MHz)，而原TIM4在APB2(72MHz)：

```
原PWM频率 (TIM4, 72MHz): 
F = 72MHz / 144 / 1001 = 500Hz

新PWM频率 (TIM3, 36MHz):
F = 36MHz / 144 / 1001 = 250Hz  ⚠️ 频率减半！
```

### 解决方案
若要保持相同的PWM频率(500Hz)，需要调整Prescaler：

**选项1：降低Prescaler**
```
需要：F = 36MHz / Prescaler / 1001 = 500Hz
     Prescaler = 36MHz / (500Hz × 1001) = 71.9 ≈ 71

设置：Prescaler = 71 → 得到约504Hz
```

**选项2：降低Period**
```
需要：F = 36MHz / 144 / Period = 500Hz
     Period = 36MHz / (500Hz × 144) = 500 ≈ 500

设置：Period = 500 → 得到502Hz
```

**推荐采用：选项2（Period = 500）**
- 保持Prescaler = 143不变
- 将Period改为500
- PWM分辨率保持在0.2%（500/1000 = 0.2%）

---

## GPIO配置变化

### 原配置
```c
PB6 - TIM4_CH1 - 复用推挽输出 (PWM)
```

### 新配置
```c
PA7 - TIM3_CH2 - 复用推挽输出 (PWM)
```

### PA7是否有冲突？
检查原有PA7的用途：
- CAN: PA11(RX), PA12(TX) ✓ 不冲突
- 其他GPIO初始化中无PA7 ✓ 安全

---

## 验证清单

- [x] 替换TIM4为TIM3
- [x] 修改通道为CH2
- [x] 修改输出引脚为PA7
- [x] 更新GPIO初始化为GPIOA
- [x] 更新时钟使能宏
- [x] 更新MSP初始化
- [ ] **验证PWM频率是否需要调整** ⚠️
- [ ] 硬件测试验证PA7输出信号

---

## 后续建议

1. **立即调整PWM频率**：
   - 将MX_TIM3_Init()中的Period从1000改为500
   - 重新计算PWM比较值逻辑（若需要）

2. **测试项**：
   - 示波器测量PA7输出频率
   - 验证PWM占空比控制正常
   - 验证电机能否正常启停和调速

3. **潜在问题**：
   - PWM频率变化可能影响电机控制特性
   - 需验证低频PWM对电机响应的影响

