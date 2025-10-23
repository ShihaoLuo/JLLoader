# PA7 PWM输出问题诊断和修复指南

## 问题现象
- PA7输出不是方波，而是一条直线
- 最大值：2.2V
- 最小值：2.0V
- 跨度仅0.2V，不是正常的3.3V～0V

## 根本原因分析

### 可能原因1：GPIO模式配置错误（最可能）
**症状：** 输出保持恒定电压而非PWM波形

**原因：** PA7可能被配置为普通GPIO输出或模拟输入，而不是复用推挽输出

**解决方案：** 已在motor_init.c中添加了显式的PA7 GPIO初始化

### 可能原因2：TIM3 PWM模块未启动
**症状：** 无PWM输出

**原因：** HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2) 失败或未调用

**解决方案：** 检查Motor_System_Init()中该函数是否被正确调用

### 可能原因3：TIM3初始化失败
**症状：** PWM参数错误或定时器不工作

**原因：** MX_TIM3_Init() 中的配置有误

**解决方案：** 验证Period=500, Prescaler=143

### 可能原因4：AFIO重映射
**症状：** GPIO被其他功能复用

**原因：** STM32F1xx某些引脚可能需要AFIO重映射

**解决方案：** 检查是否需要AFIO配置

---

## 修复清单

### 修复1：确保PA7在MX_GPIO_Init()中初始化 ✓ 已完成

在motor_init.c的MX_GPIO_Init()函数中添加了PA7初始化：
```c
/*Configure GPIO pin : PA7 (PWM输出 - TIM3_CH2) */
GPIO_InitStruct.Pin = GPIO_PIN_7;
GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;       // 复用推挽输出
GPIO_InitStruct.Pull = GPIO_NOPULL;
GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH; // PWM需要高速
HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
```

### 修复2：验证调用顺序

在Motor_System_Init()中的顺序应该是：
```
1. SystemClock_Config()        // 系统时钟
2. MX_GPIO_Init()              // GPIO初始化（包括PA7）
3. MX_TIM3_Init()              // TIM3初始化（调用HAL_TIM_MspPostInit）
4. MX_TIM2_Init()              // TIM2初始化
5. HAL_TIM_PWM_Start()         // 启动PWM
6. HAL_TIM_Base_Start_IT()     // 启动定时器中断
```

✓ **顺序正确**

### 修复3：调试技巧

#### 方法1：检查GPIO配置寄存器
添加调试代码检查PA7的GPIO设置：
```c
// 在Motor_System_Init()之后添加
uint32_t crh = GPIOA->CRH;    // PA7在CRH中（引脚7）
// CRH bit [31:28] = PA7的模式配置
// 应该 = 0xA (1010b) 表示复用推挽输出
```

#### 方法2：检查TIM3寄存器
验证TIM3是否在运行：
```c
// 在HAL_TIM_PWM_Start()之后添加
uint32_t cr1 = TIM3->CR1;      // 应该 & 0x01 = 1（使能位）
uint32_t ccmr2 = TIM3->CCMR2;  // CH2模式配置
uint32_t ccer = TIM3->CCER;    // CC2 输出使能位
```

#### 方法3：示波器测量
1. 测量TIM3的其他通道（如PA6 = TIM3_CH1）确认定时器工作
2. 如果其他通道工作而PA7不工作，说明是PA7配置问题
3. 测量PA7电压变化速度

---

## 推荐的完整诊断步骤

### 步骤1：编译和烧写
- [ ] 编译代码，确保无编译错误
- [ ] 烧写到STM32F103C8
- [ ] 重新上电

### 步骤2：检查电源
- [ ] 用万用表检查PA7管脚的静态电压
- [ ] 应该接近0V或3.3V（取决于初始值）

### 步骤3：示波器测量
- [ ] 用示波器探针接到PA7
- [ ] 设置范围：直流耦合，1V/div
- [ ] 运行你的测试代码（50.7% PWM）
- [ ] 观察是否有脉冲信号

### 步骤4：对比测试
**测试A：测试PA6（如果可用）**
```c
// 在motor_init.c中临时添加
GPIO_InitStruct.Pin = GPIO_PIN_6;
GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

// 在Motor_System_Init()中
// 尝试启动TIM3 CH1看是否工作
```

**测试B：检查Motor_SetSpeed()**
```c
// 在main.c中
Motor_Start();
Motor_SetSpeed(50.7f);
// 然后用示波器观察PA7
```

### 步骤5：内核查看
使用STM32 CubeMX或直接阅读IoC文件验证：
- [ ] PA7是否真的被配置为TIM3_CH2
- [ ] 是否有端口重映射冲突

---

## 可能的额外问题

### 问题A：输出为恒定低电平（0V）
**原因：** GPIO可能被配置为推挽输出且输出0
**解决：** 检查是否有其他代码覆盖PA7

### 问题B：输出为恒定高电平（3.3V）
**原因：** GPIO可能被配置为推挽输出且输出1
**解决：** 同上

### 问题C：输出2.0-2.2V之间摆动（你的现象）
**原因：** PWM可能在工作，但频率极低或占空比极端
**可能性：** 
- TIM3定时器频率极低
- Period参数错误
- 上拉/下拉电阻的影响
- 驱动电路阻抗高

**诊断：**
```
期望：50.7% PWM，频率 ~500Hz
  - 高电平时间：~1ms
  - 低电平时间：~1ms
  
观察到：恒定2.0-2.2V
  - 表明PWM频率可能 < 1Hz 或根本没启动
```

---

## 立即行动项

### 今天必须做的：
1. **添加PA7初始化代码** ✓ 已完成
2. **编译并烧写**
3. **用示波器测量PA7** 
4. **测量TIM3寄存器值验证工作状态**

### 代码验证检查

在Motor_System_Init()完成后添加诊断代码：
```c
void Motor_System_Init(void)
{
  /* ... 所有初始化代码 ... */
  
  /* 诊断代码 */
  #ifdef DEBUG_PA7
  // 检查GPIO模式
  volatile uint32_t crhval = GPIOA->CRH;  // PA7配置应在bit[31:28]
  volatile uint32_t crh_pa7 = (crhval >> 28) & 0xF;
  // 0xA = 1010 = 复用推挽输出 ✓
  // 0x0 = 0000 = 模拟输入 ✗
  // 0x1 = 0001 = 开漏输出 ✗
  // 0x3 = 0011 = 普通推挽输出 ✗
  
  // 检查TIM3寄存器
  volatile uint32_t tim3_cr1 = TIM3->CR1;   // bit0应 = 1
  volatile uint32_t tim3_ccmr2 = TIM3->CCMR2; // CH2模式
  volatile uint32_t tim3_ccer = TIM3->CCER;   // bit5应 = 1（CC2E）
  volatile uint32_t tim3_arr = TIM3->ARR;     // 应 = 500
  volatile uint32_t tim3_psc = TIM3->PSC;     // 应 = 143
  #endif
}
```

---

## 预期结果

### 修复成功的征兆
- ✓ 示波器观察到方波信号
- ✓ 频率 ~500Hz
- ✓ 50.7% PWM占空比（~503μs高+497μs低）
- ✓ 幅值 0V-3.3V

### 如果仍然不工作
- 可能需要检查硬件连接
- 检查示波器探针是否接触良好
- 检查PA7是否被其他硬件占用

