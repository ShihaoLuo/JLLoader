# FG脉冲RPM为0的问题分析与解决

## 问题症状

```
目标RPM: 200
当前RPM: 0
脉冲频率: 120-140 Hz (示波器测量)
信号质量: 2.5V方波, 0-2.5V范围
```

---

## 问题分析

### 1. 脉冲频率计算

**示波器显示120-140Hz** 意味着：
```
每20ms内的脉冲数 = 120Hz ÷ 50Hz = 2.4个脉冲/20ms
```

预期的RPM应该是：
```
RPM = (脉冲数 × 3000) / PPR
    = (2.4 × 3000) / 18
    ≈ 400 RPM
```

但 **目标设置是200RPM**，这表示预期脉冲频率应该是：
```
频率 = (200 RPM × 18 PPR) / 60 = 60 Hz
```

**结论**：脉冲频率正常（只是高于预期），问题不在脉冲本身

---

### 2. RPM检测为0的根本原因

**✗ 错误的中断线配置**

PB0接入中断时存在线路配置问题：

```
PB0 → EXTI Line 0 (应该用EXTI0_IRQHandler)
PB7 → EXTI Line 7 (原代码用EXTI9_5_IRQHandler)
```

**原代码问题**：
```c
// motor_init.c - 错误配置
HAL_NVIC_SetPriority(EXTI9_5_IRQn, 0, 0);      // ❌ 错误的中断线
HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);

// stm32f1xx_it.c - 使用了错误的ISR
void EXTI9_5_IRQHandler(void)                   // ❌ PB0不会触发这个
{
  HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_0);
}
```

**为什么RPM一直为0**：
1. PB0下降沿中断触发 → 需要EXTI0_IRQHandler处理
2. 但只配置了EXTI9_5_IRQn中断 → 中断从未执行
3. pulse_count始终为0 → Motor_RPM_Detection_Update()返回0
4. motor_rpm显示为0

---

### 3. STM32F1 EXTI线路映射

```
PB0 → EXTI0 IRQn   ← 正确
PB1 → EXTI1 IRQn
...
PB5 → EXTI9_5 IRQn  (共享ISR，PB5-PB9)
PB6 → EXTI9_5 IRQn  (共享ISR)
PB7 → EXTI9_5 IRQn  (共享ISR)
...
PB9 → EXTI9_5 IRQn  (共享ISR)
```

---

## 解决方案

### 修改1: motor_init.c - 更新中断优先级配置

```c
/* 旧代码 */
HAL_NVIC_SetPriority(EXTI9_5_IRQn, 0, 0);
HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);

/* 新代码 */
HAL_NVIC_SetPriority(EXTI0_IRQn, 0, 0);   // ✓ 正确的中断线
HAL_NVIC_EnableIRQ(EXTI0_IRQn);
```

### 修改2: stm32f1xx_it.c - 添加EXTI0中断处理程序

```c
/* 新增 */
void EXTI0_IRQHandler(void)
{
  /* USER CODE BEGIN EXTI0_IRQn 0 */

  /* USER CODE END EXTI0_IRQn 0 */
  HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_0);   // ✓ 处理PB0中断
  /* USER CODE BEGIN EXTI0_IRQn 1 */

  /* USER CODE END EXTI0_IRQn 1 */
}

/* 保留旧的EXTI9_5处理器以防其他引脚使用 */
void EXTI9_5_IRQHandler(void)
{
  HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_0);
}
```

---

## 验证步骤

### 1. 编译检查
```
确保编译通过，没有EXTI0_IRQn未定义的错误
```

### 2. 调试验证
```
在motor_ctrl.c的HAL_GPIO_EXTI_Callback()中添加调试变量：

uint32_t exti_trigger_count = 0;

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
  if (GPIO_Pin == GPIO_PIN_0)
  {
    pulse_count++;
    exti_trigger_count++;  // ✓ 调试计数
  }
}
```

### 3. 示波器验证
- [ ] PB0脉冲: 120-140Hz ✓ (正常)
- [ ] 信号幅度: 2.5V ✓ (正常)

### 4. Debug数值验证
```
预期值（对于120Hz脉冲，目标RPM=200）:
- exti_trigger_count: 持续增加 (原来是0)
- pulse_count: 每20ms递增约2-3次后重置
- motor_rpm: 从0变为 ~400 RPM
```

---

## 期望效果

修复后：
```
脉冲频率: 120-140 Hz (不变)
  ↓
每20ms脉冲数: 2.4-2.8个
  ↓
计算RPM: (2.4 × 3000) / 18 ≈ 400 RPM ✓
  ↓
motor_rpm显示: 400 (而不是0)
  ↓
PID闭环控制根据400 RPM调节速度
  ↓
最终收敛到目标200RPM (通过PWM调节)
```

---

## 关键要点

| 项目 | 旧配置 | 新配置 |
|------|--------|--------|
| **中断源** | PB7 | PB0 |
| **EXTI线** | 7 (属于EXTI9_5组) | 0 (独立EXTI0) |
| **ISR名称** | EXTI9_5_IRQHandler | EXTI0_IRQHandler |
| **IRQn** | EXTI9_5_IRQn | EXTI0_IRQn |
| **问题** | 中断从不触发 | ✓ 正确触发 |

---

## 深层原因

STM32F1系列EXTI设计：
- EXTI[0:4]: 每个都有独立的IRQHandler
- EXTI[5:9]: 共享一个EXTI9_5_IRQHandler
- EXTI[10:15]: 共享一个EXTI15_10_IRQHandler

从PB7改为PB0需要同时更改：
1. ✓ GPIO初始化（已修改）
2. ✓ 中断回调检查（已修改）
3. ❌ 中断优先级和使能（现已修复）
4. ❌ 中断处理程序（现已添加）

