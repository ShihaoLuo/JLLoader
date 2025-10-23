# Motor 项目代码结构分析

## 一、定时器配置概览

### 使用的定时器
项目使用了 **TIM3** 和 **TIM4** 两个定时器：

| 定时器 | 用途 | 频率 | 中断 | 备注 |
|------|------|------|------|------|
| **TIM4** | PWM输出（电机速度控制） | ~500Hz | 否 | 输出通道：CH1（PB6） |
| **TIM3** | RPM检测定时 + PID更新 | 50Hz（20ms） | 是 | 中断触发RPM计算和PID控制 |

---

## 二、TIM4 - PWM速度控制

### 配置参数
```c
// 来自 motor_init.c: MX_TIM4_Init()
htim4.Init.Prescaler = 143;          // 分频值
htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
htim4.Init.Period = 1000;            // PWM周期值
htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
```

### 频率计算
- **系统时钟**：72 MHz
- **APB2时钟**：72 MHz（TIM4挂载在APB2）
- **频率计算**：
  ```
  PWM频率 = 系统时钟 / (Prescaler + 1) / (Period + 1)
          = 72MHz / (143 + 1) / (1000 + 1)
          = 72MHz / 144 / 1001
          ≈ 494 Hz (~500Hz)
  ```

### PWM输出引脚
- **引脚**：**PB6**（GPIO Port B, Pin 6）
- **模式**：复用推挽输出（GPIO_MODE_AF_PP）
- **功能**：TIM4_CH1

### PWM占空比控制
```c
// PWM_PERIOD = 1000
// 正向逻辑（从API看）：100% → 最高速
// 硬件反向逻辑：PWM占空比越小 → 速度越快

Motor_SetSpeed(100.0f)  // 100% → PWM比较值 = 0   → 0% 占空比   → 最高速
Motor_SetSpeed(50.0f)   // 50%  → PWM比较值 = 500 → 50% 占空比  → 中速
Motor_SetSpeed(0.0f)    // 0%   → PWM比较值 = 1000→ 100% 占空比 → 停止
```

**精度**：0.1%（因为Period=1000，每个计数=0.1%）

---

## 三、TIM3 - RPM检测与PID控制定时

### 配置参数
```c
// 来自 motor_init.c: MX_TIM3_Init()
htim3.Init.Prescaler = 7199;         // 分频值
htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
htim3.Init.Period = 199;             // 自动重装值
htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
```

### 中断频率计算
- **系统时钟**：72 MHz
- **APB1时钟**：36 MHz（TIM3挂载在APB1，时钟分频÷2）
- **频率计算**：
  ```
  TIM3频率 = APB1时钟 / (Prescaler + 1)
           = 36MHz / (7199 + 1)
           = 36MHz / 7200
           = 5 kHz
  
  中断频率 = 5kHz / (Period + 1)
           = 5kHz / (199 + 1)
           = 5kHz / 200
           = 25 Hz (40ms)
  
  注：代码注释中说是50Hz(20ms)，但实际计算是25Hz(40ms)
  可能是注释过时或预期与实现不符
  ```

### 中断处理
- **中断源**：EXTI9_5_IRQn（外部中断9-5）→ TIM3_IRQn（TIM3全局中断）
- **优先级**：1（子优先级0）
- **回调函数**：`HAL_TIM_PeriodElapsedCallback()`

### TIM3用途
1. **RPM检测**：每中断一次，计算当前转速
   ```
   RPM = (pulse_count × 3000) / PPR
   其中 pulse_count = 20ms时间窗口内计数的脉冲数
        PPR = 18（每转脉冲数）
   ```

2. **PID更新**：每中断一次，更新PID控制参数

---

## 四、GPIO 引脚配置

### 电机控制相关引脚

| 引脚 | 端口 | 功能 | 模式 | 用途 | 逻辑 |
|------|------|------|------|------|------|
| **PB6** | GPIO B | TIM4_CH1 | 复用推挽 | **PWM速度控制** | 反向：占空比↓→速度↑ |
| **PB4** | GPIO B | GPIO输出 | 推挽输出 | **BK启停控制** | 1=运行，0=停止 |
| **PB5** | GPIO B | GPIO输出 | 推挽输出 | **FR方向控制** | 1=顺时针，0=逆时针 |
| **PB7** | GPIO B | GPIO输入 | IT下降沿 | **FG脉冲输入（霍尔传感器）** | 下降沿触发计数 |
| **PB3** | GPIO B | GPIO输出 | 推挽输出 | 备用输出 | - |

### GPIO初始化代码
```c
// 来自 motor_init.c: MX_GPIO_Init()

// PB4 (BK - 启停控制)
GPIO_InitStruct.Pin = GPIO_PIN_4;
GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;  // 推挽输出
GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

// PB5 (FR - 方向控制)
GPIO_InitStruct.Pin = GPIO_PIN_5;
GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;  // 推挽输出
GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

// PB7 (FG - 霍尔脉冲输入)
GPIO_InitStruct.Pin = GPIO_PIN_7;
GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;  // 下降沿中断
GPIO_InitStruct.Pull = GPIO_PULLUP;           // 上拉
HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

// 使能中断
HAL_NVIC_SetPriority(EXTI9_5_IRQn, 0, 0);  // 优先级0（最高）
HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);
```

### 初始状态
```c
HAL_GPIO_WritePin(GPIOB, GPIO_PIN_3, GPIO_PIN_SET);  // PB3 = 1
HAL_GPIO_WritePin(GPIOB, GPIO_PIN_4, GPIO_PIN_SET);  // PB4 = 1 (默认运行)
HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5, GPIO_PIN_SET);  // PB5 = 1 (默认顺时针)
```

---

## 五、关键功能函数

### 电机速度控制
```c
void Motor_SetSpeed(float speed_percent)
// 输入：0.0-100.0%
// 内部转换为PWM比较值（反向逻辑处理）
// 精度：0.1%
```

### RPM检测
```c
void Motor_RPM_Detection_Update(void)
// 在TIM3中断中调用（每20ms或40ms）
// 计算当前转速：RPM = (pulse_count × 3000) / 18
```

### PID闭环控制
```c
void Motor_SetTargetRPM(uint16_t rpm)
// 设置目标转速，启用PID闭环控制
// 自适应PID参数根据目标转速自动调整

void Motor_PID_Update(void)
// 在TIM3中断中调用
// 根据当前RPM和目标RPM计算PID输出
```

### 启停和方向控制
```c
void Motor_Start(void)
// BK = 1，启动电机

void Motor_Stop(void)
// BK = 0，停止电机

void Motor_SetDirection(uint8_t clockwise)
// FR = 1 顺时针
// FR = 0 逆时针
```

---

## 六、中断系统概览

### 外部中断 - EXTI9_5 (PB7 FG脉冲)
```c
// 中断处理函数：stm32f1xx_it.c: EXTI9_5_IRQHandler()
// 回调函数：motor_ctrl.c: HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
  if (GPIO_Pin == GPIO_PIN_7) // PB7 FG脉冲
  {
    pulse_count++;  // 最小化处理，快速返回
  }
}

优先级：0（最高）
特点：快速响应，仅做计数
```

### 定时器中断 - TIM3 (RPM计算和PID更新)
```c
// 中断处理函数：stm32f1xx_it.c: TIM3_IRQHandler()
// 回调函数：motor_ctrl.c: HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  if (htim->Instance == TIM3)
  {
    Motor_RPM_Detection_Update();  // 更新RPM计算
    Motor_PID_Update();             // 更新PID控制
  }
}

优先级：1
周期：~40ms（实际）
特点：定时更新RPM和PID，控制周期稳定
```

### CAN中断 (接收消息)
```c
// 中断处理函数：USB_LP_CAN1_RX0_IRQn
// 回调函数：motor_init.c: HAL_CAN_RxFifo0MsgPendingCallback()

优先级：1
特点：接收CAN消息后立即处理
```

---

## 七、系统时钟配置

```c
// 来自 motor_init.c: SystemClock_Config()
// 振荡器配置
RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;  // 高速外部振荡器
RCC_OscInitStruct.HSEState = RCC_HSE_ON;
RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;  // ×9倍频

// 时钟分配
RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;  // PLL输出
RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;         // AHB = 72MHz
RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;          // APB1 = 36MHz
RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;          // APB2 = 72MHz

最终系统时钟：72 MHz
```

---

## 八、代码结构图

```
motor_init.c / motor_init.h
├── SystemClock_Config()          // 系统时钟初始化 (72MHz)
├── MX_GPIO_Init()                // GPIO初始化
│   ├── PB4 - BK启停
│   ├── PB5 - FR方向
│   ├── PB6 - PWM输出(TIM4_CH1)
│   └── PB7 - FG脉冲输入(EXTI)
├── MX_TIM4_Init()                // PWM定时器初始化
│   ├── 频率：~500Hz
│   ├── 精度：0.1%（Period=1000）
│   └── CH1输出到PB6
├── MX_TIM3_Init()                // RPM定时器初始化
│   ├── 中断频率：50Hz理论(25Hz实际)
│   └── 用于RPM计算和PID更新
└── Motor_System_Init()           // 系统总初始化

motor_ctrl.c / motor_ctrl.h
├── Motor_Start()                 // 启动电机(BK=1)
├── Motor_Stop()                  // 停止电机(BK=0)
├── Motor_SetDirection()          // 设置方向(FR=0/1)
├── Motor_SetSpeed()              // PWM速度控制(0-100%)
├── Motor_SetTargetRPM()          // PID闭环转速控制
├── Motor_RPM_Detection_Update()  // RPM计算(TIM3中断)
├── Motor_PID_Update()            // PID更新(TIM3中断)
├── Motor_PID_AdaptiveUpdate()    // 自适应PID参数
└── HAL_GPIO_EXTI_Callback()      // 脉冲计数(EXTI中断)

main.c
└── main()
    ├── Motor_System_Init()
    ├── MotorCANProtocol_Init()   // CAN协议初始化
    └── 主循环处理CAN命令
```

---

## 九、控制流程总结

### 1. 启动流程
```
main() 
  → Motor_System_Init()
    → SystemClock_Config()      (72MHz)
    → MX_GPIO_Init()            (初始化PB3-7)
    → MX_TIM4_Init()            (PWM初始化)
    → MX_TIM3_Init()            (RPM定时初始化)
    → HAL_TIM_PWM_Start()       (启动PWM)
    → HAL_TIM_Base_Start_IT()   (启动TIM3中断)
    → Motor_Stop()              (电机默认停止)
    → Motor_RPM_Detection_Init()
    → Motor_PID_Init()
    → Motor_CAN_Init()
```

### 2. 实时控制流程（通过CAN命令）
```
主循环
  → MotorCANProtocol_Task()      (处理CAN协议)
  → 根据CAN命令：
    ├── Motor_SetSpeed(pwm%)     (直接PWM控制)
    └── Motor_SetTargetRPM(rpm)  (闭环转速控制)

TIM3中断 (每40ms)
  → Motor_RPM_Detection_Update() (计算当前RPM)
  → Motor_PID_Update()           (更新PID，调整PWM)

EXTI中断 (每次FG脉冲)
  → pulse_count++                (实时计数)
```

### 3. PWM信号流程
```
Motor_SetSpeed(50%) 
  → 计算PWM值 = (100-50) * 10 = 500
  → __HAL_TIM_SET_COMPARE(&htim4, CH1, 500)
  → TIM4_CH1 输出50%占空比PWM信号到PB6
  → 驱动电路根据反向逻辑控制电机速度
```

---

## 十、关键参数速查表

| 参数 | 值 | 单位 | 说明 |
|------|---|----|------|
| 系统时钟 | 72 | MHz | HSE+PLL×9 |
| APB1时钟 | 36 | MHz | 用于TIM3 |
| APB2时钟 | 72 | MHz | 用于TIM4 |
| TIM4频率 | ~500 | Hz | PWM频率 |
| TIM4周期 | 1000 | counts | PWM分辨率0.1% |
| TIM3中断 | 40 | ms | RPM更新周期（实际） |
| TIM3中断理论 | 20 | ms | 代码注释值 |
| PPR(脉冲/转) | 18 | - | 霍尔传感器参数 |
| RPM计算公式 | pulse×3000/18 | - | 20ms时间窗口 |
| 电机启停 | PB4 | - | BK信号 |
| 电机方向 | PB5 | - | FR信号 |
| PWM输出 | PB6 | - | TIM4_CH1 |
| 脉冲输入 | PB7 | - | EXTI中断 |

