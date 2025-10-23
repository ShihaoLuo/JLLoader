# RPM计算频率和PID控制频率分析

## 当前配置总结

### RPM计算频率

```
定时器: TIM2
触发条件: TIM2周期中断
中断处理: HAL_TIM_PeriodElapsedCallback → Motor_RPM_Detection_Update()

配置:
├─ 时钟源: APB1 = 36 MHz
├─ 预分频器 (Prescaler): 7199
│  └─ 计数频率 = 36MHz / (7199+1) = 36MHz / 7200 = 5 kHz
├─ 自动重装值 (Period): 99
│  └─ 中断周期 = 100 / 5kHz = 20 ms
└─ 中断频率 = 1000ms / 20ms = 50 Hz

结论: RPM计算频率 = 50 Hz (每20ms更新一次)
```

### PID控制频率

```
调用点: Motor_PID_Update()
调用位置: Motor_RPM_Detection_Update() 内部
调用频率: 与RPM更新频率相同

结论: PID控制频率 = 50 Hz (每20ms更新一次)
```

---

## 详细计算过程

### TIM2的频率计算

```
基本公式:
  f_out = f_clk / ((PSC + 1) × (ARR + 1))

其中:
  f_clk = 36 MHz (APB1时钟)
  PSC (Prescaler) = 7199
  ARR (Auto Reload Register / Period) = 99

计算:
  f_out = 36,000,000 / ((7199 + 1) × (99 + 1))
        = 36,000,000 / (7200 × 100)
        = 36,000,000 / 720,000
        = 50 Hz

中断周期 T = 1 / 50 = 0.02 s = 20 ms
```

### RPM检测的时间窗口

```
在每个TIM2中断中:
  Motor_RPM_Detection_Update()被调用
  ↓
  计算当前20ms内收到的脉冲数 (pulse_count)
  ↓
  使用移动平均滤波 (RPM_FILTER_SIZE = 5个周期)
  ↓
  计算平均脉冲数: avg_pulses = sum(最近5个周期) / 5
  ↓
  RPM = (avg_pulses × 3000) / 18
  ↓
  每20ms重置pulse_count = 0

时间轴:
周期1(0-20ms):   pulse_count = ?  → history[0] = ?
周期2(20-40ms):  pulse_count = ?  → history[1] = ?
周期3(40-60ms):  pulse_count = ?  → history[2] = ?
周期4(60-80ms):  pulse_count = ?  → history[3] = ?
周期5(80-100ms): pulse_count = ?  → history[4] = ?
周期6(100-120ms):pulse_count = ?  → history[0] = ? (覆盖第1个)
```

### RPM计算公式

```
原始脉冲计数 (单个20ms周期):
  pulse_count: 单周期内(20ms)检测到的脉冲数

移动平均 (5周期 = 100ms):
  avg_pulses = (history[0] + history[1] + ... + history[4]) / 5
             = (最近100ms的脉冲总数) / 5

转速计算:
  RPM = (avg_pulses × 3000) / 18
  
  其中:
    3000 = 60秒/分钟 × 50次/秒 (50Hz周期)
    18 = PPR (每转18个脉冲)
```

### PID控制更新

```
时序:
  TIM2中断 (每20ms)
    ↓
  HAL_TIM_PeriodElapsedCallback()
    ↓
  Motor_RPM_Detection_Update()
    ├─ 更新RPM (使用移动平均)
    └─ 调用 Motor_PID_Update()
         ↓
       计算误差: error = target_rpm - motor_rpm
         ↓
       计算PID输出:
         p_term = Kp × error
         i_term = Ki × integral
         d_term = Kd × (error - prev_error)
         output = p_term + i_term + d_term
         ↓
       应用到PWM:
         Motor_SetSpeed(pid_output)
         ↓
         设置TIM3的比较值
```

---

## 关键参数表

| 参数 | 值 | 单位 | 说明 |
|------|-----|------|------|
| **RPM更新周期** | 20 | ms | 1/50Hz |
| **RPM更新频率** | 50 | Hz | TIM2中断频率 |
| **PID更新周期** | 20 | ms | 同RPM |
| **PID更新频率** | 50 | Hz | 同RPM |
| **RPM滤波器大小** | 5 | 个周期 | 100ms窗口 |
| **时间窗口** | 100 | ms | 5×20ms |
| **脉冲频率** | 120-140 | Hz | 实测 |
| **每周期脉冲数** | 2.4-2.8 | 个 | 120-140Hz ÷ 50Hz |

---

## 信号流程图

```
脉冲信号 (PB0)
   ↓
EXTI0_IRQHandler
   ↓
HAL_GPIO_EXTI_Callback()
   ├─ pulse_count++ (实时计数)
   ↓
[每20ms]
   ↓
TIM2中断 (50Hz)
   ↓
HAL_TIM_PeriodElapsedCallback()
   ↓
Motor_RPM_Detection_Update()
   ├─ 保存pulse_count到history
   ├─ 计算avg_pulses (移动平均)
   ├─ 计算motor_rpm
   ├─ 调用Motor_PID_Update()
   │  ├─ 计算误差
   │  ├─ 计算PID输出
   │  └─ 调用Motor_SetSpeed()
   │     ├─ 计算PWM值
   │     ├─ 设置pwm_value_debug
   │     └─ __HAL_TIM_SET_COMPARE() → TIM3
   │
   └─ pulse_count = 0 (重置)
```

---

## 时间关键路径

```
脉冲检测到PWM输出的最大延迟:

脉冲到达 (t=0)
  ↓
EXTI中断 (几个CPU周期) ≈ 1-2 us
  ↓
pulse_count++ 更新 ≈ 1 us
  ↓
[等待TIM2中断]
  ↓
TIM2中断触发 (最晚) ≈ 20 ms
  ↓
Motor_PID_Update() 计算 ≈ 100-200 us
  ↓
Motor_SetSpeed() 调用 ≈ 10 us
  ↓
__HAL_TIM_SET_COMPARE() 设置 ≈ 1 us
  ↓
PWM更新到电机 (下一个PWM周期)
  ↓
总延迟: 20 ms (由TIM2中断决定)
```

---

## 频率相关的可能问题

### 问题1: RPM检测延迟

```
症状: RPM显示延迟，不能实时反映转速变化
原因: 20ms的检测周期
解决: 可以改为10ms周期 (Prescaler=7199, Period=49)
      但会增加CPU负载
```

### 问题2: PID控制响应速度

```
当前: 50Hz更新频率 (20ms周期)
影响: PID需要50-100ms才能对误差做出反应
      
提高频率方法:
  改为100Hz (10ms): Prescaler=7199, Period=49
  改为200Hz (5ms):  Prescaler=7199, Period=24
  
权衡:
  更高频率 → 更快响应 但 CPU负载增加
  更低频率 → CPU节省 但 响应变慢
```

### 问题3: 脉冲计数的量化误差

```
脉冲频率: 120-140 Hz (2.4-2.8个脉冲/20ms)
问题: 脉冲数是整数，导致每个周期计数不稳定
      周期1: 2个脉冲
      周期2: 3个脉冲
      周期3: 2个脉冲
      ...

解决: 使用移动平均滤波器 (RPM_FILTER_SIZE=5)
      消除短期波动，平滑长期趋势
```

---

## 优化建议

### 建议1: 保持当前50Hz

**理由**:
```
✓ 20ms周期是一个很好的平衡
✓ CPU负载合理 (~2-5%)
✓ 脉冲滤波窗口 (100ms) 足够消除抖动
✓ PID控制响应速度足够 (20ms × 2-3次 = 40-60ms)
```

### 建议2: 如需提升到100Hz

```c
// motor_init.c 中修改 MX_TIM2_Init()
htim2.Init.Prescaler = 7199;      // 保持不变
htim2.Init.Period = 49;           // 改为49 (从99)
// 新频率 = 36MHz / (7200 × 50) = 100 Hz (10ms周期)

// motor_ctrl.h 中修改
#define RPM_DETECTION_PERIOD_MS 10  // 改为10
#define RPM_FILTER_SIZE 10          // 改为10 (保持100ms窗口)
```

### 建议3: 监测CPU负载

```c
// 添加时间戳测量
uint32_t tim2_callback_start = 0;
uint32_t tim2_callback_duration = 0;

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  tim2_callback_start = HAL_GetTick();
  
  Motor_RPM_Detection_Update();
  
  tim2_callback_duration = HAL_GetTick() - tim2_callback_start;
  // tim2_callback_duration 应该 < 5ms (对于50Hz)
}
```

---

## 总结

```
当前配置:
┌─────────────────────────────────────────────────────────┐
│ RPM计算频率:  50 Hz (每20ms)                            │
│ PID控制频率:  50 Hz (每20ms)                            │
│ 脉冲检测:     实时 (EXTI中断)                          │
│ 滤波窗口:     100ms (5个周期)                          │
│ 时间延迟:     0-20ms (脉冲到RPM计算)                  │
│ CPU负载:      低 (~2-5%)                              │
└─────────────────────────────────────────────────────────┘

这个配置适合大多数应用场景:
✓ 提供足够的实时性
✓ CPU负载低
✓ 控制精度足够
✓ 成本-收益平衡好
```

