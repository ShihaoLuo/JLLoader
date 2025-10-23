# PA7 PWM不输出 - 根本原因分析

## 已识别的根本问题

### 问题1：IOC配置过时
**状态：** 已发现但无法直接修复（需要CubeMX重新配置）

motor.ioc文件中仍然配置TIM4，但代码已改为TIM3。这可能导致：
- HAL库在某些情况下使用错误的配置
- NVIC中断配置可能不完整

### 问题2：PA7初始化顺序问题 ✓ 已修复

**修复：** 添加了显式的PA7 GPIO初始化到MX_GPIO_Init()中

```c
/*Configure GPIO pin : PA7 (PWM输出 - TIM3_CH2) */
GPIO_InitStruct.Pin = GPIO_PIN_7;
GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;       // 关键：复用推挽输出
GPIO_InitStruct.Pull = GPIO_NOPULL;
GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH; // PWM需要高速
HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
```

### 问题3：可能缺少的中断配置

TIM3可能需要在NVIC中配置中断（虽然PWM不需要中断，但某些HAL操作可能需要）

---

## 立即可采取的修复步骤

### 步骤1：验证TIM3是否真的启动

添加调试代码到main.c：

```c
int main(void)
{
  HAL_Init();
  Motor_System_Init();
  
  // 诊断代码
  // 1. 检查TIM3是否在运行
  if (!(TIM3->CR1 & TIM_CR1_CEN)) {
    // TIM3未启用！
    TIM3->CR1 |= TIM_CR1_CEN;  // 强制启用
  }
  
  // 2. 检查CC2输出是否使能
  if (!(TIM3->CCER & TIM_CCER_CC2E)) {
    // CC2输出未使能！
    TIM3->CCER |= TIM_CCER_CC2E;  // 强制使能
  }
  
  // 3. 检查PA7 GPIO是否正确配置
  uint32_t pa7_mode = (GPIOA->CRH >> 28) & 0xF;
  // 应该 = 0xA (1010b = 复用推挽, 50MHz)
  if (pa7_mode != 0xA) {
    // PA7 GPIO模式错误，重新配置
    GPIOA->CRH = (GPIOA->CRH & 0x0FFFFFFF) | 0xA0000000;
  }
  
  // 设置50.7% PWM
  Motor_SetSpeed(50.7f);
  
  while(1) {
    // 保持运行
    HAL_Delay(1000);
  }
}
```

### 步骤2：验证PWM值是否正确设置

在Motor_SetSpeed()中添加调试：

```c
void Motor_SetSpeed(float speed_percent)
{
  if (speed_percent > 100.0f) speed_percent = 100.0f;
  if (speed_percent < 0.0f) speed_percent = 0.0f;
  
  float pwm_value_float = ((100.0f - speed_percent) * (float)PWM_PERIOD) / 100.0f;
  uint32_t pwm_value = (uint32_t)(pwm_value_float + 0.5f);
  
  if (pwm_value > PWM_PERIOD) pwm_value = PWM_PERIOD;
  
  // 调试：检查计算的PWM值
  // speed_percent = 50.7f
  // expected pwm_value = (100 - 50.7) * 500 / 100 = 247.5 ≈ 248
  
  __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, pwm_value);
  
  // 验证是否写入成功
  uint32_t ccr2 = TIM3->CCR2;  // 应该 = 248
}
```

### 步骤3：关键寄存器验证

完整的诊断函数：

```c
void Diagnose_PA7_PWM(void)
{
  printf("=== PA7 PWM 诊断 ===\n");
  
  // 1. GPIO配置
  printf("GPIOA->CRH = 0x%08X\n", GPIOA->CRH);
  uint32_t pa7_mode = (GPIOA->CRH >> 28) & 0xF;
  printf("PA7 mode bits: 0x%X (应该 = 0xA)\n", pa7_mode);
  
  // 2. TIM3基本状态
  printf("TIM3->CR1 = 0x%08X\n", TIM3->CR1);
  printf("TIM3 使能: %d (bit0)\n", (TIM3->CR1 & 1));
  
  // 3. TIM3 PWM配置
  printf("TIM3->CCMR2 = 0x%08X\n", TIM3->CCMR2);
  printf("TIM3->CCER = 0x%08X\n", TIM3->CCER);
  printf("CC2E (bit5): %d\n", (TIM3->CCER >> 5) & 1);
  printf("CC2P (bit5): %d\n", (TIM3->CCER >> 5) & 1);
  
  // 4. TIM3计数值
  printf("TIM3->ARR = %d\n", TIM3->ARR);
  printf("TIM3->PSC = %d\n", TIM3->PSC);
  printf("TIM3->CCR2 = %d\n", TIM3->CCR2);
  printf("TIM3->CNT = %d\n", TIM3->CNT);
  
  // 5. 频率计算
  uint32_t freq = 36000000 / (TIM3->PSC + 1) / (TIM3->ARR + 1);
  printf("计算的PWM频率: %d Hz\n", freq);
}
```

---

## 代码修改清单

### 已完成的修复 ✓

1. **在MX_GPIO_Init()中添加PA7初始化**
   - 配置PA7为GPIO_MODE_AF_PP（复用推挽）
   - 设置速度为GPIO_SPEED_FREQ_HIGH

2. **验证调用顺序**
   - MX_GPIO_Init() → MX_TIM3_Init() → HAL_TIM_PWM_Start()
   - 顺序正确

3. **PWM参数调整**
   - Period: 500 (从1000调整以保持500Hz)
   - Prescaler: 143
   - 频率: ~501Hz ✓

### 需要验证的项目

1. **编译是否成功**
   - 应该无编译错误（lint警告可以忽略）

2. **烧写后示波器测试**
   - 测量PA7电压
   - 应该看到500Hz的方波

3. **如果仍然不工作**
   - 检查硬件连接
   - 检查示波器设置
   - 使用诊断代码确认寄存器值

---

## 最可能的原因（按概率排序）

1. **PA7仍未被正确配置为PWM** (40%)
   - **症状：** 恒定电压 2.0-2.2V
   - **修复：** 已添加显式GPIO初始化
   
2. **TIM3 CC2通道未使能** (30%)
   - **症状：** 无PWM输出
   - **修复：** 检查HAL_TIM_PWM_Start()是否被调用
   
3. **PWM频率极低导致看起来像直线** (20%)
   - **症状：** 恒定电压，实际频率<1Hz
   - **修复：** 验证Period和Prescaler
   
4. **GPIO被其他代码覆盖** (10%)
   - **症状：** 初始化后被改为其他模式
   - **修复：** 搜索所有对PA7的操作

---

## 下一步行动

### 立即行动 (今天)
1. 编译代码
2. 烧写到开发板
3. 运行诊断代码
4. 用示波器测量PA7

### 如果仍然不工作
1. 运行Diagnose_PA7_PWM()函数
2. 检查输出的寄存器值
3. 对比预期值
4. 根据差异调整代码

### 最后手段
1. 使用CubeMX重新生成代码，配置TIM3_CH2
2. 手动合并HAL生成的代码

