# PA7 PWM 快速修复检查清单

## 问题症状回顾
- ✗ PA7 无方波输出
- ✗ 输出恒定电压 2.0-2.2V
- ✗ 没有脉冲信号

## 已完成的代码修复 ✓

### 1. motor_init.c - MX_GPIO_Init()
**位置：** 第201-207行

```c
/*Configure GPIO pin : PA7 (PWM输出 - TIM3_CH2) */
GPIO_InitStruct.Pin = GPIO_PIN_7;
GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;       // ✓ 复用推挽输出
GPIO_InitStruct.Pull = GPIO_NOPULL;
GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH; // ✓ 高速
HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
```

**检查：** 代码已添加 ✓

### 2. motor_init.c - MX_TIM3_Init()
**位置：** 第79-115行

```c
htim3.Init.Period = 500;    // ✓ 已调整为500
htim3.Init.Prescaler = 143; // ✓ 正确值
sConfigOC.Pulse = 500;      // ✓ 初始值已调整
```

**检查：** 参数已正确 ✓

### 3. motor_init.h
**位置：** 函数声明

```c
void MX_TIM3_Init(void);  // ✓ 已改为TIM3
```

**检查：** 已正确 ✓

### 4. motor_ctrl.c
**全局变量：**
```c
TIM_HandleTypeDef htim3;  // ✓ 已改为TIM3（不是TIM4）
```

**PWM设置：**
```c
__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, pwm_value);  // ✓ 正确
```

**检查：** 已正确 ✓

### 5. stm32f1xx_hal_msp.c
**HAL_TIM_MspPostInit()：**
```c
GPIO_InitStruct.Pin = GPIO_PIN_7;  // ✓ PA7
HAL_GPIO_Init(GPIOA, &GPIO_InitStruct); // ✓ GPIOA
```

**检查：** 已正确 ✓

---

## 诊断步骤（按顺序执行）

### Step 1: 确认代码编译
```bash
# 在项目目录执行编译
# 应该无编译错误
```
**结果：** ✓ / ✗

### Step 2: 烧写并重启
1. 编译生成hex文件
2. 使用STM32CubeProgrammer烧写
3. 复位开发板
4. 上电

**结果：** ✓ / ✗

### Step 3: 用示波器测量PA7
1. 示波器探针接到PA7脚
2. 接地到GND
3. 直流耦合模式
4. 量程：1V/div或2V/div

**期望看到：**
- 频率：~500Hz
- 占空比：~50%（对于50.7% PWM）
- 幅值：0V-3.3V

**实际看到：** ________________

### Step 4: 对比寄存器值
创建诊断函数并在main中调用：

```c
void Check_PA7_Registers(void)
{
  // GPIO配置
  uint32_t crh = GPIOA->CRH;
  uint32_t pa7_config = (crh >> 28) & 0xF;
  printf("PA7 GPIO config: 0x%X (expect 0xA)\n", pa7_config);
  
  // TIM3状态
  printf("TIM3->CR1.CEN (enable): %d (expect 1)\n", (TIM3->CR1 >> 0) & 1);
  printf("TIM3->CCER.CC2E (CH2 enable): %d (expect 1)\n", (TIM3->CCER >> 5) & 1);
  printf("TIM3->ARR: %d (expect 500)\n", TIM3->ARR);
  printf("TIM3->PSC: %d (expect 143)\n", TIM3->PSC);
  printf("TIM3->CCR2: %d\n", TIM3->CCR2);
  
  // 频率计算
  uint32_t pclk1 = 36000000; // APB1时钟
  uint32_t freq = pclk1 / (TIM3->PSC + 1) / (TIM3->ARR + 1);
  printf("Calculated frequency: %d Hz\n", freq);
}
```

**寄存器检查结果：**
```
PA7 GPIO config: ____ (expect 0xA)
TIM3->CR1.CEN: _____ (expect 1)
TIM3->CCER.CC2E: _____ (expect 1)
TIM3->ARR: _____ (expect 500)
TIM3->PSC: _____ (expect 143)
TIM3->CCR2: _____
Frequency: _____ Hz
```

---

## 可能的问题和解决方案

### 问题A：PA7 GPIO配置不是0xA
**症状：** PA7_config = 0x0 或其他值

**原因：** GPIO模式配置失败

**解决方案：**
```c
// 在Motor_System_Init()后立即执行
GPIOA->CRH = (GPIOA->CRH & 0x0FFFFFFF) | 0xA0000000;
```

### 问题B：TIM3未启用（CR1.CEN = 0）
**症状：** 没有PWM输出

**原因：** HAL_TIM_PWM_Start()未被正确调用

**解决方案：**
```c
// 检查Motor_System_Init()中该行是否存在
if (HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2) != HAL_OK)
{
  Error_Handler();
}

// 或强制启用
TIM3->CR1 |= 0x01;
```

### 问题C：CC2E = 0
**症状：** TIM3运行但CH2无输出

**原因：** PWM通道未启用

**解决方案：**
```c
// 强制启用CC2输出
TIM3->CCER |= 0x10; // bit4 = CC2E
```

### 问题D：频率不对
**症状：** 频率不是500Hz

**原因：** PSC或ARR值错误

**解决方案：** 检查MX_TIM3_Init()中的值

---

## 快速诊断表

| 检查项 | 期望值 | 实际值 | 状态 |
|-------|-------|-------|------|
| PA7 GPIO模式 | 0xA | ____ | ✓/✗ |
| TIM3 CEN | 1 | ____ | ✓/✗ |
| TIM3 CC2E | 1 | ____ | ✓/✗ |
| TIM3 ARR | 500 | ____ | ✓/✗ |
| TIM3 PSC | 143 | ____ | ✓/✗ |
| PWM频率 | ~500Hz | ____ Hz | ✓/✗ |
| PA7示波器波形 | 方波 | ____ | ✓/✗ |

---

## 如果以上都检查正常但还是不工作

### 可能的硬件问题
1. 示波器探针接触不良
   - **修复：** 重新连接探针，确保接触良好
   
2. PA7引脚损坏
   - **修复：** 尝试使用PA6或其他GPIO测试
   
3. 驱动电路问题
   - **修复：** 检查驱动电路是否连接正确

### 可能的软件问题
1. CubeMX生成的代码与手动修改冲突
   - **修复：** 重新用CubeMX生成TIM3配置
   
2. AFIO重映射问题
   - **修复：** 添加AFIO配置（通常不需要）

### 最后手段
使用CubeMX重新配置：
1. 打开motor.ioc
2. 配置TIM3_CH2为PWM输出
3. 配置PA7引脚
4. 重新生成代码
5. 手动保留我们的修改

---

## 成功标志

当你看到以下现象时，修复成功 ✓
- ✓ 示波器上PA7有清晰的500Hz方波
- ✓ 50.7% PWM时占空比约50%
- ✓ 频率稳定，无抖动
- ✓ 幅值0V-3.3V

