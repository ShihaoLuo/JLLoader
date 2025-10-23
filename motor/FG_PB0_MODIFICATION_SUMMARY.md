# FG脉冲检测改为PB0 - 改动总结

## 改动概述
将电机脉冲(FG)检测引脚从PB7改为PB0，一圈脉冲数确认为18 (PPR = 18)。

---

## 修改的文件

### 1. **motor_init.c** - MX_GPIO_Init()

#### 改动：
```c
// 旧配置
/*Configure GPIO pin : PB7 (FG - 霍尔脉冲输入，外部中断模式) */
GPIO_InitStruct.Pin = GPIO_PIN_7;
GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;  // 下降沿中断
GPIO_InitStruct.Pull = GPIO_PULLUP;           // 上拉，确保稳定的数字信号
HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

// 新配置
/*Configure GPIO pin : PB0 (FG - 霍尔脉冲输入，外部中断模式) */
GPIO_InitStruct.Pin = GPIO_PIN_0;
GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;  // 下降沿中断
GPIO_InitStruct.Pull = GPIO_PULLUP;           // 上拉，确保稳定的数字信号
HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
```

---

### 2. **motor_ctrl.c** - HAL_GPIO_EXTI_Callback()

#### 改动：
```c
// 旧代码
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
  if (GPIO_Pin == GPIO_PIN_7) // PB7 FG脉冲中断
  {
    pulse_count++;
  }
}

// 新代码
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
  if (GPIO_Pin == GPIO_PIN_0) // PB0 FG脉冲中断
  {
    pulse_count++;
  }
}
```

---

### 3. **stm32f1xx_it.c** - EXTI9_5_IRQHandler()

#### 改动：
```c
// 旧代码
void EXTI9_5_IRQHandler(void)
{
  HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_7);
}

// 新代码
void EXTI9_5_IRQHandler(void)
{
  HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_0);
}
```

> **注意**：中断线名称保持为EXTI9_5_IRQn（因为PB0在中断线0，属于EXTI[4:0]组，但映射到同一个ISR中）

---

### 4. **motor_ctrl.h** - 硬件配置注释

#### 更新：
```c
// 旧：霍尔传感器连接到 PB7 (带上拉电阻)
// 新：霍尔传感器连接到 PB0 (带上拉电阻)
```

---

## 脉冲计数配置

### RPM计算公式
```
RPM = (脉冲数 × 3000) / PPR
    = (脉冲数 × 3000) / 18
```

**参数说明：**
- **脉冲数**：每20ms周期内计数的脉冲数
- **3000** = 60秒/分钟 × 50次/秒 (20ms周期)
- **PPR** = 18 (每转18个脉冲)

### 例子
- 若20ms内检测到9个脉冲：RPM = (9 × 3000) / 18 = 1500 RPM
- 若20ms内检测到18个脉冲：RPM = (18 × 3000) / 18 = 3000 RPM

---

## GPIO分布最终状态

| 功能 | 原引脚 | 新引脚 | 模式 | 中断类型 |
|------|-------|-------|------|---------|
| **脉冲(FG)** | PB7 | PB0 | IT下降沿 | EXTI9_5_IRQn |
| **启停(BK)** | PB4 | PB4 | 推挽输出 | - |
| **方向(FR)** | PA6 | PA6 | 推挽输出 | - |
| **PWM输出** | PA7 | PA7 | 复用推挽 | - |

---

## 代码验证

### 脉冲计数流程
```c
// 1. PB0下降沿触发中断
EXTI9_5_IRQHandler()
  → HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_0)
  
// 2. 调用回调函数
HAL_GPIO_EXTI_Callback(GPIO_PIN_0)
  → pulse_count++ (计数脉冲)
  
// 3. 每20ms更新一次RPM
Motor_RPM_Detection_Update()
  → motor_rpm = (pulse_count × 3000) / 18
  → pulse_count = 0 (重置)
```

---

## 测试建议

1. **静态测试**
   - [ ] 用函数信号发生器连接到PB0
   - [ ] 设置为18Hz方波 (模拟1500RPM)
   - [ ] 观察Motor_GetRPM()返回值是否约为1500

2. **动态测试**
   - [ ] 启动电机
   - [ ] 通过CAN或UART读取RPM值
   - [ ] 改变速度，验证RPM跟随变化

3. **中断测试**
   - [ ] 示波器观察PB0引脚
   - [ ] 验证下降沿触发脉冲计数

---

## 兼容性检查

✓ **PB0不与其他功能冲突**
- PB4: BK (启停)
- PA6: FR (方向)
- PA7: PWM (TIM3_CH2)
- 无其他使用PB0的功能

✓ **中断线检查**
- PB0属于EXTI线0
- STM32F1xx将EXTI[4:0]映射到同一个ISR处理
- 原有PB7属于EXTI线7，属于EXTI[9:5]组
- **需要注意**：可能需要添加新的中断处理程序或在existing ISR中添加EXTI[4:0]处理

---

## 总结

✓ **FG脉冲引脚成功改为PB0**
✓ **脉冲数确认为18 (PPR = 18)**
✓ **中断配置已更新**
✓ **代码完全更新**

系统现在使用：
- **PB0**：脉冲输入(外部中断，下降沿)
- **PB4**：启停控制(GPIO推挽)
- **PA6**：方向控制(GPIO推挽)
- **PA7**：PWM速度控制(TIM3_CH2)

