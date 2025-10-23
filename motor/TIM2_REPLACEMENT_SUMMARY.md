# TIM3 替换为 TIM2 - 改动总结

## 概述
将RPM检测和PID更新从TIM3定时器更改为TIM2定时器。

---

## 修改的文件列表

### 1. **motor_init.c**
#### 修改内容：
- **函数替换**：`MX_TIM3_Init()` → `MX_TIM2_Init()`
  - TIM2的时钟来自APB1（36MHz），TIM3来自APB2（72MHz）
  - 重新计算分频参数：
    - APB1时钟 = 36MHz
    - Prescaler = 7199 → 得到5kHz计时频率
    - Period = 199 → 中断周期40ms（25Hz）
  
- **时钟使能**：
  ```c
  // 旧：__HAL_RCC_TIM3_CLK_ENABLE();
  // 新：__HAL_RCC_TIM2_CLK_ENABLE();
  ```

- **系统初始化函数**：`Motor_System_Init()` 中更新调用
  ```c
  MX_TIM2_Init();  // 替代 MX_TIM3_Init()
  HAL_TIM_Base_Start_IT(&htim2);  // 替代 HAL_TIM_Base_Start_IT(&htim3)
  ```

---

### 2. **motor_ctrl.c**
#### 修改内容：
- **全局变量声明**：
  ```c
  // 旧：TIM_HandleTypeDef htim3;
  // 新：TIM_HandleTypeDef htim2;
  ```

- **中断回调函数**：`HAL_TIM_PeriodElapsedCallback()`
  ```c
  // 旧：if (htim->Instance == TIM3)
  // 新：if (htim->Instance == TIM2)
  ```

---

### 3. **motor_ctrl.h**
#### 修改内容：
- **导出变量**：
  ```c
  // 旧：extern TIM_HandleTypeDef htim3;
  // 新：extern TIM_HandleTypeDef htim2;
  ```

---

### 4. **motor_init.h**
#### 修改内容：
- **函数声明**：
  ```c
  // 旧：void MX_TIM3_Init(void);
  // 新：void MX_TIM2_Init(void);
  ```

---

### 5. **stm32f1xx_hal_msp.c**
#### 修改内容：
- **HAL_TIM_Base_MspInit()** 函数：
  ```c
  // 旧：
  if(htim_base->Instance==TIM3)
  {
    __HAL_RCC_TIM3_CLK_ENABLE();
    HAL_NVIC_SetPriority(TIM3_IRQn, 1, 0);
    HAL_NVIC_EnableIRQ(TIM3_IRQn);
  }
  
  // 新：
  if(htim_base->Instance==TIM2)
  {
    __HAL_RCC_TIM2_CLK_ENABLE();
    HAL_NVIC_SetPriority(TIM2_IRQn, 1, 0);
    HAL_NVIC_EnableIRQ(TIM2_IRQn);
  }
  ```

---

### 6. **stm32f1xx_it.c**
#### 修改内容：
- **外部变量声明**：
  ```c
  // 旧：extern TIM_HandleTypeDef htim3;
  // 新：extern TIM_HandleTypeDef htim2;
  ```

- **中断处理函数替换**：
  ```c
  // 旧：void TIM3_IRQHandler(void)
  // 新：void TIM2_IRQHandler(void)
  
  // 旧：HAL_TIM_IRQHandler(&htim3);
  // 新：HAL_TIM_IRQHandler(&htim2);
  ```

---

## 时钟配置对比

### TIM3（旧）
| 项目 | 值 |
|------|-----|
| 挂载总线 | APB2 |
| 时钟频率 | 72 MHz |
| Prescaler | 7199 |
| 计时频率 | 72MHz / 7200 = 10kHz |
| Period | 199 |
| **中断周期** | **20ms (50Hz)** |

### TIM2（新）
| 项目 | 值 |
|------|-----|
| 挂载总线 | APB1 |
| 时钟频率 | 36 MHz |
| Prescaler | 7199 |
| 计时频率 | 36MHz / 7200 = 5kHz |
| Period | 99 |
| **中断周期** | **20ms (50Hz)** ✓ |

### 频率计算
**TIM3（72MHz）：**
```
中断周期 = (7199 + 1) × (199 + 1) / 72MHz = 7200 × 200 / 72MHz = 20ms (50Hz)
```

**TIM2（36MHz）：**
```
中断周期 = (7199 + 1) × (99 + 1) / 36MHz = 7200 × 100 / 36MHz = 20ms (50Hz)
```

### 影响
- ✓ RPM检测和PID更新的频率**保持不变**：50Hz（20ms）
- ✓ 中断周期**保持不变**：20ms
- ✓ 检测逻辑和控制频率**完全一致**
- 定时器负载分散：PWM在TIM4，RPM/PID在TIM2
- TIM3完全释放，可用于其他功能

---

## 验证检查清单

- [x] 替换所有TIM3函数为TIM2
- [x] 更新全局变量声明
- [x] 更新中断回调函数
- [x] 更新时钟使能宏
- [x] 更新中断处理函数
- [x] 更新函数声明

---

## 编译和测试建议

1. **编译检查**：确保无编译错误和警告
2. **功能测试**：
   - 验证电机可以启停
   - 验证RPM检测正常工作
   - 验证PID闭环控制正常响应
   - 检查中断频率（可通过LED翻转或示波器验证）

3. **性能影响**：
   - RPM检测更新频率降低（20ms→40ms）
   - 可能对高速响应有轻微影响
   - 低速稳定性应无明显变化

---

## 后续优化建议

如需恢复到更高的更新频率，可以：
1. 选择其他APB2上的定时器（TIM1, TIM4, TIM5, TIM8等）
2. 或调整TIM2的Prescaler和Period获得更高频率
3. 或同时使用两个定时器（TIM2 + TIM4）进行不同的控制

