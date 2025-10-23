# PWM和PID核心问题诊断

## 问题症状汇总

```
症状1: pid_output_debug = 0 (永不变化)
症状2: PWM = 0 → 2400 RPM (最大速度)
症状3: 目标2400 RPM → 只能1800 RPM (无法达到)
症状4: 平滑后RPM仍为166 (滤波器未生效)
```

---

## 根本问题分析

### 问题1: PWM逻辑反向 ❌

**现象**: PWM=0时转速最高(2400RPM)，这说明:
- PWM=0 (0%占空比) → 电机全速 ✓
- PWM=500 (100%占空比) → 电机停止 ✓

**代码现状** (Motor_SetSpeed):
```c
float pwm_value_float = ((100.0f - speed_percent) * (float)PWM_PERIOD) / 100.0f;
// 输入100% → pwm_value = 0 (全速) ✓
// 输入0% → pwm_value = 500 (停止) ✓
```

**问题**: 这个反向逻辑是**故意的**（硬件就是这样设计的）

但是，**PWM占空比为0意味着电机输出是PWM_PERIOD的PWM**（方波在高电平）
而不是真正的0占空比！

实际上应该是：
```c
// TIM_CHANNEL_2的比较值决定了占空比
__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, pwm_value);

// 如果 pwm_value = 0:
//   占空比 = 0/500 = 0% (输出始终为低) → 错误的！
// 应该是:
//   占空比 = pwm_value/(PWM_PERIOD+1)
```

---

### 问题2: PID输出未被正确导出 ❌

**现象**: `pid_output_debug` 始终为0

**原因**: 
1. `pid_output_debug` 在代码中**未定义**
2. `speed_pid.output` 在Motor_PID_Update中计算，但未导出给调试

**应该添加**:
```c
// motor_ctrl.c 全局变量
float pid_output_debug = 0;

// Motor_PID_Update中
pid_output_debug = speed_pid.output;  // 在应用前保存
Motor_SetSpeed(speed_pid.output);
```

---

### 问题3: PWM应用范围不对 ❌

**现象**: 目标2400 RPM只能达到1800 RPM

**原因分析**:
```
PWM = 0 → 2400 RPM (实测最大)
PWM = 500 → 0 RPM (停止)

假设PWM与转速近似线性:
RPM ≈ 2400 - (PWM / 500) × 2400
    = 2400 × (1 - PWM/500)

要达到2400 RPM: 需要PWM = 0
要达到2000 RPM: 需要PWM ≈ 40
要达到1800 RPM: 需要PWM ≈ 120
要达到1200 RPM: 需要PWM ≈ 240
要达到0 RPM:   需要PWM = 500
```

**问题**: Motor_SetSpeed的PWM反向逻辑可能有问题

```c
// 当前代码
float pwm_value_float = ((100.0f - speed_percent) * PWM_PERIOD) / 100.0f;

// 输入speed_percent=100% → pwm_value = 0 → 应该是最高速
// 输入speed_percent=0% → pwm_value = 500 → 应该是停止

但实测显示PWM=0时转速最高，这说明逻辑**看起来**是对的
但为什么无法达到目标转速？
```

---

### 问题4: RPM滤波器未生效 ❌

**现象**: 平滑后RPM仍为166跳变

**可能原因**:
```
① Motor_RPM_Detection_Update 在TIM2中被调用
② Motor_RPM_Detection_Update 可能没有被正确调用
③ rpm_pulse_history 可能初始化有问题
④ 或者滤波计算本身有bug
```

**检查点**:
```c
// 是否motor_rpm真的在变化？
// 应该看到: 0 → 33 → 100 → 133 → 200 → ...
// 而不是: 166 → 333 → 166 → 333
```

---

## 真正的问题所在

### 核心问题: 达不到目标转速的原因

```
目标2400 RPM → 实际1800 RPM (达成率 75%)

分析:
如果PWM的影响是线性的，且PWM=0能达到2400RPM:
  RPM = 2400 - k × PWM
  1800 = 2400 - k × PWM_current
  k × PWM_current = 600
  PWM_current ≈ 250 (假设k=2.4)

PID应该输出 speed_percent = 100% → pwm_value = 0
但如果输出的是 speed_percent = 75% → pwm_value = 125

这说明PID输出(speed_percent)不够大！
```

**问题诊断**:
1. ❓ PID的输出范围是什么？ (应该0-100)
2. ❓ 当前PID输出多少？ (需要看pid_output_debug)
3. ❓ Motor_SetSpeed的逻辑是否真的反向？

---

## 快速修复方案

### 方案1: 添加PID debug输出

```c
// motor_ctrl.h
extern float pid_output_debug;
extern float pid_error_debug;

// motor_ctrl.c
float pid_output_debug = 0;
float pid_error_debug = 0;

void Motor_PID_Update(void)
{
  // ... 计算逻辑 ...
  
  float error = speed_pid.target - current;
  pid_error_debug = error;  // 保存误差
  
  // ... 更新PID ...
  
  pid_output_debug = speed_pid.output;  // 保存输出
  Motor_SetSpeed(speed_pid.output);
}
```

### 方案2: 验证PWM的真实行为

```c
// 测试代码
Motor_Start();
Motor_SetSpeed(0);    // PWM应该= 0, 转速= 2400 RPM
HAL_Delay(1000);
motor_rpm_test_1 = motor_rpm;

Motor_SetSpeed(50);   // PWM应该= 250, 转速= ?
HAL_Delay(1000);
motor_rpm_test_50 = motor_rpm;

Motor_SetSpeed(100);  // PWM应该= 500, 转速= 0 RPM
HAL_Delay(1000);
motor_rpm_test_100 = motor_rpm;

// 观察motor_rpm_test_1, motor_rpm_test_50, motor_rpm_test_100
// 应该看到大约 2400, 1200, 0
```

---

## 详细问题清单

| 问题 | 症状 | 原因 | 修复 |
|------|------|------|------|
| PWM逻辑 | 低PWM=高速 | ✓硬件设计 | 检查是否有反转 |
| PID输出 | pid_output_debug=0 | ❌未定义 | 添加debug变量 |
| RPM滤波 | 仍为166跳变 | ❌未生效 | 检查Motor_RPM_Detection_Update |
| 达不到目标 | 2400→1800 | ❌PID输出不足? | 检查pid_output_debug |

---

## 紧急排查步骤

### 第1步: 添加必要的Debug变量

```c
// motor_ctrl.h
extern float pid_output_debug;
extern float pid_error_debug;
extern uint16_t pwm_value_debug;

// motor_ctrl.c
float pid_output_debug = 0;
float pid_error_debug = 0;
uint16_t pwm_value_debug = 0;

void Motor_SetSpeed(float speed_percent)
{
  // ... 计算 ...
  float pwm_value_float = ((100.0f - speed_percent) * PWM_PERIOD) / 100.0f;
  uint32_t pwm_value = (uint32_t)(pwm_value_float + 0.5f);
  
  pwm_value_debug = pwm_value;  // 保存debug
  __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, pwm_value);
}
```

### 第2步: 观察Debug数据

```
当目标RPM = 2400时:
pid_error_debug:      应该从 ~2400 减少到接近0
pid_output_debug:     应该从 ~100 减少到 ~1-5
pwm_value_debug:      应该从 ~0 增加到 ~480-500

当目标RPM = 1800时:
pid_error_debug:      应该收敛到接近0
pid_output_debug:     应该收敛到 ~25-30 (对应75%速度)
pwm_value_debug:      应该收敛到 ~125-150
```

### 第3步: 检查RPM滤波

```
观察这些值:
pulse_count:       应该 1,2,1,2,... 或 2,2,2,2,...
motor_rpm_raw:     应该 166,333,... 或 333,333,...
motor_rpm:         应该 平滑增长 (不是166,333)
```

---

## 最可能的原因

根据现象，**最可能是PID控制根本没有启用**:

```c
void Motor_PID_Update(void)
{
  if (!pid_control_enable || !speed_pid.enable) return;  // ← 这里返回了！
  
  // 如果pid_control_enable或speed_pid.enable为0，就不会进行任何计算
}
```

**检查**:
1. `pid_control_enable` 是否被正确设置为1?
2. `speed_pid.enable` 是否被正确设置为1?
3. Motor_SetTargetRPM是否被正确调用？

