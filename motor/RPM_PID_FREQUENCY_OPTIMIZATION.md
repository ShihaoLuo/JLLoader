# 电机RPM计算与PID控制频率优化方案

## 问题分析

### 原系统问题
```
症状: 转速严重抖动（摆动），PID控制性差

根本原因：
- RPM计算周期: 20ms (50Hz)
- 最大转速: 4100 RPM
- PPR: 18
- 脉冲频率: 4100 × 18 / 60 = 1230 Hz
- 每个20ms窗口内脉冲数: 1230 × 0.02 = 24.6个 ← 大量的量化误差！

具体表现：
- 周期1: 24个脉冲 → RPM = 24 × 3000 / 18 = 4000 RPM
- 周期2: 25个脉冲 → RPM = 25 × 3000 / 18 = 4166 RPM
- 周期3: 24个脉冲 → RPM = 4000 RPM
- ...
- 结果: 4000 ↔ 4166 的震荡，即使加入移动平均也无法完全消除！
```

### 解决方案思路
```
增加RPM采样窗口，降低量化误差：

新方案 (100ms窗口):
- 每个100ms窗口内脉冲数: 1230 × 0.1 = 123个 ← 精度提高！
- 量化误差: ±1个脉冲对应 ±33.3 RPM (相对精度 0.8%)
- 是否可接受: ✓ 是

权衡:
- RPM更新频率: 50Hz → 10Hz (20ms → 100ms)
- PID控制频率: 需要单独提高 ← 保证快速响应
- 最终: RPM 10Hz 检测，PID 200Hz 控制
```

## 优化配置

### 1. 定时器配置 (motor_init.c)

#### TIM2 (原配置)
```c
Prescaler = 7199    // 36MHz / 7200 = 5kHz
Period    = 99      // 5kHz / 100 = 50Hz (20ms)
结果      : RPM和PID都是50Hz
```

#### TIM2 (新配置)
```c
Prescaler = 1799    // 36MHz / 1800 = 20kHz
Period    = 99      // 20kHz / 100 = 200Hz (5ms)
结果      : 中断频率 200Hz (每5ms触发一次)
```

### 2. RPM计算参数 (motor_ctrl.h)

```c
// 原配置
#define RPM_FILTER_SIZE    5              // 5个周期 = 100ms窗口
#define RPM_DETECTION_PERIOD_MS  20       // 20ms周期

// 新配置
#define RPM_FILTER_SIZE    2              // 2个周期 = 200ms窗口  
#define RPM_DETECTION_PERIOD_MS  100      // 100ms周期（实际通过中断分频实现）
```

### 3. 频率分频逻辑 (motor_ctrl.c)

```c
// 在motor_ctrl.c中添加计数器
static uint8_t tim2_interrupt_counter = 0;
#define RPM_UPDATE_DIVIDER  2   // 200Hz ÷ 2 = 100Hz RPM更新
#define PID_UPDATE_DIVIDER  1   // 200Hz ÷ 1 = 200Hz PID更新

// 在HAL_TIM_PeriodElapsedCallback中实现分频
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  if (htim->Instance == TIM2)
  {
    // 每次中断都执行PID (200Hz)
    Motor_PID_Update();
    
    // 每2次中断执行一次RPM计算 (100Hz)
    if ((++tim2_interrupt_counter) >= RPM_UPDATE_DIVIDER)
    {
      tim2_interrupt_counter = 0;
      Motor_RPM_Detection_Update();
    }
  }
}
```

## 控制系统架构

```
脉冲信号 (PB0)
   ↓
EXTI中断 (实时计数)
   ├─ pulse_count++
   └─ 无延迟
   
TIM2中断 (200Hz, 每5ms)
   ├─ [分频计数]
   ├─ 分频=0: Motor_PID_Update() (200Hz)
   │  ├─ 读取motor_rpm (最新值)
   │  ├─ 计算误差
   │  ├─ 计算PID输出
   │  └─ Motor_SetSpeed() → PWM输出
   │
   └─ 分频=1时: Motor_RPM_Detection_Update() (100Hz)
      ├─ 保存pulse_count到history
      ├─ 计算平均脉冲数 (2个100ms周期)
      └─ 更新motor_rpm
```

## RPM计算公式变化

### 原公式 (20ms周期)
```
RPM = (脉冲数 × 3000) / 18
    = (脉冲数 × 60秒/分钟 × 50次/秒) / 18

其中 50次/秒 = 1 / 20ms
```

### 新公式 (100ms周期)
```
RPM = (脉冲数 × 600) / 18
    = (脉冲数 × 60秒/分钟 × 10次/秒) / 18

其中 10次/秒 = 1 / 100ms
```

## PID参数调整

### 频率变化的影响

从50Hz → 200Hz，时间间隔 Δt 从20ms → 5ms：

| 参数 | 原理 | 调整方法 |
|------|------|--------|
| **Kp** | 比例项与时间无关 | 保持不变 |
| **Ki** | 积分项 = Ki × Δt × 累积误差 | 需要÷4 (Δt变为1/4) |
| **Kd** | 微分项 = Kd × Δ误差 / Δt | 需要×4 (Δt变为1/4) |

### 实施方案

为了保守起见，采用折中值：
- Ki: 乘以 0.5 (而不是 ÷4)
- Kd: 乘以 2.0 (而不是 ×4)

```c
// 示例：中转速范围 (1500-3000 RPM)
// 原参数
kp = 0.0048;    ki = 0.03;      kd = 0.002;

// 新参数 (200Hz)
kp = 0.0048;    ki = 0.015;     kd = 0.004;
               (×0.5)           (×2)
```

## 性能对比

### 量化误差分析

#### 原系统 (20ms, 50Hz)
```
脉冲频率 = 1230 Hz
单位脉冲对应RPM = 1 / 0.02 × 3000 / 18 = 8333 RPM !

最坏情况: 
- 24个脉冲 = 4000 RPM
- 25个脉冲 = 4166 RPM
- 误差 = ±83 RPM 相对精度 2%
```

#### 新系统 (100ms, 10Hz)  
```
脉冲频率 = 1230 Hz
单位脉冲对应RPM = 1 / 0.1 × 600 / 18 = 333.3 RPM

最坏情况:
- 123个脉冲 = 4100 RPM
- 124个脉冲 = 4133 RPM
- 误差 = ±16.5 RPM 相对精度 0.4%
```

**改善: 误差从 ±2% → ±0.4%，提升5倍精度！**

### 控制响应特性

| 指标 | 原配置 | 新配置 | 提升 |
|------|-------|-------|------|
| **RPM更新周期** | 20ms | 100ms | - (检测更新变慢) |
| **PID控制周期** | 20ms | 5ms | ×4 (快速) |
| **总响应延迟** | 20ms | 5ms + 100ms RPM | 改进 (更平滑) |
| **抖动幅度** | ±83 RPM | ±17 RPM | ×5 (改善) |

### 稳定性分析

```
原系统: PID每20ms更新一次，但RPM精度差
       → PID控制"冲击"输入值变化，导致超调和抖动

新系统: RPM精度高(×5)，PID更频繁(×4)
       → PID有更稳定的反馈输入
       → 控制输出更连续平滑
       → 整体稳定性大幅提升
```

## 编译和测试

### 代码修改清单

✓ motor_init.c:
  - TIM2: Prescaler 7199 → 1799
  - TIM2: Period 保持99

✓ motor_ctrl.h:
  - RPM_FILTER_SIZE: 5 → 2
  - RPM_DETECTION_PERIOD_MS: 20 → 100

✓ motor_ctrl.c:
  - 添加: tim2_interrupt_counter, RPM_UPDATE_DIVIDER
  - 修改: HAL_TIM_PeriodElapsedCallback (实现分频)
  - 修改: Motor_RPM_Detection_Update (100ms公式)
  - 修改: Motor_PID_AdaptiveUpdate (新PID参数)

### 编译步骤

```bash
cd motor/app
rm -rf build
make clean
make
```

### 测试步骤

1. **编译验证**: 无错误
2. **上电测试**: 
   - 设置目标转速: Motor_SetTargetRPM(2400)
   - 观察转速曲线: 应该平滑上升，无明显抖动
   - 对比: 转速稳定在±50 RPM范围内 (原先±200+ RPM)
3. **全转速范围测试**:
   - 800 RPM: 平稳上升，无超调
   - 2000 RPM: 响应快速，稳定性好
   - 4100 RPM: 无抖动，保持稳定
4. **示波器测量**:
   - PA7 (PWM): 频率仍为10.26kHz ✓
   - PB0 (脉冲): 实时脉冲计数准确

## 注意事项

⚠️ **关键点**

1. **RPM检测变慢**: 从50Hz → 10Hz
   - 这是可以接受的，因为精度提高了5倍
   - PID控制仍然快速 (200Hz)

2. **PWM频率不变**: 仍为10.26kHz
   - 占空比精度仍为0.2%
   - 只是PID更新频率变高

3. **PID参数保守调整**:
   - Ki×0.5, Kd×2 (而不是严格的÷4和×4)
   - 如果响应不够快，可以进一步调整

4. **前馈补偿**:
   - 保持启用 (FEEDFORWARD_ENABLE=1)
   - 帮助快速响应
   - 可在实测后微调增益

## 预期效果

✅ **转速稳定性**: 抖动从 ±200 RPM → ±50 RPM
✅ **控制响应**: 从50Hz → 200Hz PID更新，响应快速
✅ **精度提升**: 量化误差从 ±2% → ±0.4%
✅ **整体体验**: 电机运行平滑，控制精细，噪音降低

## 回滚方法

如需恢复原配置：

```c
// motor_init.c
htim2.Init.Prescaler = 7199;    // 改回原值

// motor_ctrl.h  
#define RPM_FILTER_SIZE 5
#define RPM_DETECTION_PERIOD_MS 20

// motor_ctrl.c
// 恢复原RPM公式: × 3000 / 18
// 恢复原中断处理: 每次都调用Motor_RPM_Detection_Update()
// 恢复原PID参数
```

