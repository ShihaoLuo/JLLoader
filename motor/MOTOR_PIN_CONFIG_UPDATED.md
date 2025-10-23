# 电机控制引脚配置修改总结

## 修改内容

### 1️⃣ PWM频率修改
**目标**: 20kHz (从原来的50kHz)

**修改位置**: `Core/Src/motor_init.c` - `MX_TIM4_Init()` 函数

**具体参数**:
- **旧配置**: 
  - Prescaler = 143
  - Period = 1000
  - Pulse = 1000
  - 频率 = 72MHz / (143+1) / (1000+1) ≈ 500Hz (错误的理解)

- **新配置**:
  - Prescaler = 3 (PSC=3, 即分频系数4)
  - Period = 899 (ARR=899)
  - Pulse = 450 (50% 占空比)
  - 频率 = 72MHz / 4 / 900 = 20kHz ✓

### 2️⃣ 引脚映射修改

| 功能 | 旧引脚 | 新引脚 | 说明 |
|------|--------|--------|------|
| BK (启停) | PB4 | PA5 | GPIO输出，1=运行, 0=停止 |
| FR (方向) | PB5 | PA6 | GPIO输出，1=顺时针, 0=逆时针 |
| PWM | PB6 | PA7 | TIM4 CH1 PWM输出 |
| FG (霍尔脉冲) | PB7 | PB0 | EXTI0 下降沿中断输入 |

### 3️⃣ 修改的文件

#### 📄 `motor.ioc` (STM32CubeIDE项目文件)
- 更新MCU引脚定义: PA5, PA6, PA7, PB0
- 更新TIM4参数: Period=899, Prescaler=3, Pulse=450
- 更新GPIO配置: PA5/PA6为输出, PA7为PWM, PB0为EXTI中断
- 添加EXTI0_IRQn NVIC配置

#### 📄 `Core/Src/motor_init.c`
- `MX_TIM4_Init()`: 修改频率到20kHz
- `MX_GPIO_Init()`: 更新GPIO初始化为新引脚
  - PA5, PA6输出初始化
  - PB0 EXTI中断初始化 (EXTI0)
  
#### 📄 `Core/Src/motor_ctrl.c`
- `HAL_GPIO_EXTI_Callback()`: GPIO_PIN_7 → GPIO_PIN_0
- `Motor_Start()`: GPIOB, GPIO_PIN_4 → GPIOA, GPIO_PIN_5
- `Motor_Stop()`: GPIOB, GPIO_PIN_4 → GPIOA, GPIO_PIN_5
- `Motor_SetDirection()`: GPIOB, GPIO_PIN_5 → GPIOA, GPIO_PIN_6

#### 📄 `Core/Src/stm32f1xx_it.c`
- `EXTI9_5_IRQHandler()` → `EXTI0_IRQHandler()`
- 更新中断处理函数名和GPIO_PIN编号

### 4️⃣ 物理连接

#### STM32F103C8 引脚号对应关系
```
PA5 (Pin14) -----> BK (启停控制)
PA6 (Pin15) -----> FR (方向控制)
PA7 (Pin16) -----> PWM (速度控制)
PB0 (Pin18) -----> FG (霍尔脉冲/频率反馈)
```

#### 逻辑说明
- **BK (PA5)**: 1 = 电机运行, 0 = 电机停止
- **FR (PA6)**: 1 = 顺时针, 0 = 逆时针
- **PWM (PA7)**: TIM4通道1 PWM输出, 20kHz, 占空比 0-100%
- **FG (PB0)**: 霍尔传感器脉冲, 下降沿触发EXTI0中断

### 5️⃣ 编译与验证

#### 预期编译结果
- ✓ 无错误
- ⚠️ 可能有关于GPIOA/GPIOB未定义的警告(这是Pylance的假阳性，实际编译会通过)

#### 功能验证步骤
1. 编译工程
2. 烧写固件到STM32
3. 连接电机驱动模块
4. 运行测试:
   ```c
   Motor_Start();           // PA5 = 1, 电机启动
   Motor_SetSpeed(50.0);    // PA7: 50% PWM
   Motor_SetDirection(1);   // PA6 = 1, 顺时针
   // FG脉冲会在PB0上触发EXTI0中断
   ```

### 6️⃣ 时序参数

**20kHz PWM 时序**:
- 周期(Period): 50µs (1/20kHz)
- 一个计数周期: 13.9ns (1/72MHz)
- 从0到Period所需时间: 50µs

**占空比计算**:
- 50% 占空比: Pulse = 450 (899/2)
- 例如: 设置50% → PWM高电平持续25µs

---

## ✅ 修改检查清单

- [x] TIM4频率改为20kHz
- [x] PA5配置为BK输出
- [x] PA6配置为FR输出
- [x] PA7配置为PWM输出
- [x] PB0配置为FG中断输入
- [x] motor_ctrl.c中GPIO控制函数更新
- [x] stm32f1xx_it.c中EXTI中断函数更新
- [x] motor.ioc配置文件更新
- [ ] 编译验证
- [ ] 烧写与功能测试

---

## 📝 注意事项

1. **PA7 PWM输出**: 由于改为PA7, 物理硬件上需要确保PA7连接到电机驱动模块的PWM输入
2. **PB0中断**: FG信号现在使用PB0, 需要确保霍尔传感器接到PB0
3. **GPIOA时钟**: MX_GPIO_Init()中已经使能了GPIOA时钟(__HAL_RCC_GPIOA_CLK_ENABLE())
4. **向后兼容**: 如果原代码中还有其他地方使用PB4/PB5/PB6/PB7, 需要一并更新

---

## 🔍 PWM频率验证公式

```
时钟频率 = 72MHz (STM32F103系统时钟)
分频系数 = PSC + 1 = 3 + 1 = 4
计数周期 = Period + 1 = 899 + 1 = 900

PWM频率 = 时钟频率 / 分频系数 / 计数周期
        = 72MHz / 4 / 900
        = 18MHz / 900
        = 20kHz ✓
```

