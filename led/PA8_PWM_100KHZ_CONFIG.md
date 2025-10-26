# PA8 PWM 100KHz 配置说明

## 概述
配置PA8引脚为TIM1通道1的PWM输出，频率为100KHz。

## 硬件配置

| 参数 | 值 | 说明 |
|------|-----|------|
| **引脚** | PA8 | STM32F103 Port A Pin 8 |
| **定时器** | TIM1 | Advanced Control Timer 1 |
| **通道** | CH1 | Channel 1 |
| **GPIO Mode** | AF_PP | Alternate Function Push-Pull |

## 时钟配置

| 参数 | 值 | 说明 |
|------|-----|------|
| **系统时钟** | 72 MHz | HSE × 9 (8MHz × 9) |
| **APB2时钟** | 72 MHz | APB2分频系数为1 |
| **TIM1时钟** | 72 MHz | APB2上的定时器 |

## PWM参数配置

### 频率计算
- **目标频率**: 100 KHz
- **周期**: T = 1/100K = 10 μs
- **系统时钟**: 72 MHz
- **预分频 (PSC)**: 0 (无分频)
- **自动重装值 (ARR)**: 720 - 1 = 719

**验证计算**:
```
频率 = 系统时钟 / (PSC + 1) / (ARR + 1)
     = 72,000,000 / 1 / 720
     = 100,000 Hz ✓
```

### 计数周期
- **ARR (Auto-Reload Register)**: 719
- **计数周期**: 720 ≥ 250 ✓

### PWM输出参数
| 参数 | 值 | 说明 |
|------|-----|------|
| **PSC** | 0 | 预分频（无分频） |
| **ARR** | 719 | 自动重装值(周期-1) |
| **CCR1** | 360 | 比较值(占空比50%) |
| **占空比** | 50% | CCR1/ARR = 360/720 = 50% |

## 代码实现

### 1. GPIO配置 (main.c 中的 MX_GPIO_Init)
```c
GPIO_InitStruct.Pin = GPIO_PIN_8;
GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;        // 复用推挽输出
GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;  // 高速
HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
```

### 2. TIM1 PWM初始化 (main.c 中的 MX_TIM1_Init)
```c
// 启用TIM1时钟
__HAL_RCC_TIM1_CLK_ENABLE();

// 配置定时器参数
htim1.Instance = TIM1;
htim1.Init.Prescaler = 0;           // PSC = 0
htim1.Init.Period = 719;            // ARR = 719
htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
HAL_TIM_PWM_Init(&htim1);

// 配置输出比较
sConfigOC.OCMode = TIM_OCMODE_PWM1;
sConfigOC.Pulse = 360;              // CCR = 360 (50%占空比)
sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_1);

// 启动PWM
HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
```

### 3. 主函数调用 (main.c)
```c
int main(void)
{
  HAL_Init();
  SystemClock_Config();
  MX_GPIO_Init();
  MX_TIM1_Init();      // ← 初始化PA8 PWM
  
  while (1) {
    // 主循环
  }
}
```

## 占空比调整

如需修改占空比，更改 **CCR值** (Pulse参数):

| 占空比 | CCR值 | 计算 |
|--------|-------|------|
| 25% | 180 | 720 × 0.25 = 180 |
| 50% | 360 | 720 × 0.50 = 360 |
| 75% | 540 | 720 × 0.75 = 540 |

## 输出波形特性

| 特性 | 值 |
|------|-----|
| **频率** | 100 KHz |
| **周期** | 10 μs |
| **高电平宽度** | 5 μs (50%占空比) |
| **低电平宽度** | 5 μs (50%占空比) |
| **峰值电压** | 3.3V (STM32 I/O) |

## 验证方法

1. **示波器测量**: 连接示波器到PA8，应观察到100KHz的方波信号
2. **频率测试**: 使用频率计测量PA8引脚频率，应显示~100kHz
3. **占空比验证**: 使用示波器或逻辑分析仪测量高低电平宽度

## 注意事项

1. PA8是TIM1的CH1输出，TIM1是高级定时器
2. TIM1支持死区配置(用于补偿电源管理电路的延迟)
3. 当改变占空比时，应在PWM运行时直接更改 `htim1.Instance->CCR1` 值
4. 确保系统时钟已正确配置为72MHz (8MHz × 9)

## 动态修改占空比

在运行时修改PWM占空比:
```c
// 修改占空比为75%
__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, 540);
```

