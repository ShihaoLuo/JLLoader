# PID和PWM调试检查清单

## 新增的Debug变量

```c
// motor_ctrl.h和motor_ctrl.c中已添加以下debug变量:

float pid_output_debug;         // PID输出 (0-100% speed)
float pid_error_debug;          // PID误差 (target_rpm - current_rpm)
uint16_t pwm_value_debug;       // PWM比较值 (0-500)
float pid_control_enable_debug; // PID是否启用 (0/1)
float pid_pid_enable_debug;     // speed_pid.enable状态 (0/1)
```

---

## 调试步骤 - 一步一步排查

### 第1步: 验证PID是否启用

**操作**:
```c
// 在main.c中调用以下测试代码
Motor_Start();                  // 启动电机
Motor_SetTargetRPM(2400);       // 设置目标转速
HAL_Delay(100);                 // 等待初始化
```

**观察Debug窗口**:
```
pid_control_enable_debug:  应该 = 1.0
pid_pid_enable_debug:      应该 = 1.0

如果任何一个为0.0，说明PID没有启用！
```

**检查代码**:
```c
// motor_ctrl.c Motor_SetTargetRPM函数中
void Motor_SetTargetRPM(uint16_t rpm)
{
  target_rpm = rpm;
  speed_pid.target = (float)rpm;
  
  if (rpm > 0) {
    pid_control_enable = 1;       // ← 这行很关键
    speed_pid.enable = 1;         // ← 这行也很关键
    Motor_PID_Reset();
    Motor_PID_AdaptiveUpdate(rpm);
    // ...
  }
}
```

---

### 第2步: 验证PID的误差计算

**操作**:
```c
Motor_Start();
Motor_SetTargetRPM(2400);
HAL_Delay(50);
```

**观察Debug数据** (每20ms刷新):
```
时刻    | target_rpm | motor_rpm | pid_error_debug | 状态
--------|------------|-----------|-----------------|--------
初始    | 2400       | 0         | 2400            | ✓正常
20ms    | 2400       | 166       | 2234            | ✓减少
40ms    | 2400       | 333       | 2067            | ✓继续减少
...     | 2400       | 1800      | 600             | ✓接近目标
...     | 2400       | 2350      | 50              | ✓非常接近

如果pid_error_debug始终不变，说明motor_rpm没有更新！
```

---

### 第3步: 验证PID输出

**操作**:
```c
Motor_Start();
Motor_SetTargetRPM(2400);
```

**观察Debug数据**:
```
pid_output_debug应该:
初始: 100 (最大输出，加速电机)
中期: 80-50 (根据误差减少)
最终: 1-5 (微调到稳定)

关键检查:
- 如果pid_output_debug = 0，说明PID被限幅或禁用了
- 如果pid_output_debug始终 = 100，说明误差太大或有bug
```

**可能的值范围**:
```
pid_output_debug = 0   → 电机停止
pid_output_debug = 50  → 电机运行在中速(1200 RPM左右)
pid_output_debug = 100 → 电机运行在高速(2400 RPM左右)
```

---

### 第4步: 验证PWM是否正确应用

**操作**:
```c
Motor_Start();
Motor_SetSpeed(0);      // 直接设置速度0
HAL_Delay(500);
```

**观察Debug窗口**:
```
pwm_value_debug:  应该 = 500 (反向逻辑)
motor_rpm:        应该 = 0 (电机停止)

然后测试:
Motor_SetSpeed(50);
```

**观察**:
```
pwm_value_debug:  应该 ≈ 250 (0到500的中点)
motor_rpm:        应该 ≈ 1200 (2400的一半)

然后测试:
Motor_SetSpeed(100);
```

**观察**:
```
pwm_value_debug:  应该 ≈ 0 (最小)
motor_rpm:        应该 ≈ 2400 (最大)
```

---

## 完整诊断流程

### 场景1: pid_output_debug始终为0

**症状**: PID输出不工作

**排查**:
```
1. 检查 pid_control_enable_debug 是否为1
   - 如果为0，说明Motor_SetTargetRPM没有被调用
   - 或者Motor_SetTargetRPM中的pid_control_enable设置失败
   
2. 检查 pid_pid_enable_debug 是否为1
   - 如果为0，说明speed_pid.enable没有被正确初始化
   - 可能是Motor_PID_Reset中有问题
   
3. 检查 Motor_PID_Update 是否被调用
   - 在Motor_RPM_Detection_Update中调用
   - 如果rpm_update_count不增加，说明这个函数未被调用
```

**修复**:
```c
// 手动测试PID是否工作
void test_pid(void)
{
  Motor_Start();
  target_rpm = 2400;
  speed_pid.target = 2400.0f;
  pid_control_enable = 1;
  speed_pid.enable = 1;
  
  // 现在pid_output_debug应该不为0
}
```

---

### 场景2: pid_output_debug有值，但电机不加速

**症状**: 
```
pid_output_debug = 80
motor_rpm = 0 (不增长)
```

**原因**: PWM可能没有正确应用

**检查**:
```
1. pwm_value_debug的值是否在变化?
   - 如果不变，说明Motor_SetSpeed没有被正确调用
   
2. pwm_value_debug是否与pid_output_debug成反比?
   - 当pid_output_debug = 100时，pwm_value_debug应该 ≈ 0
   - 当pid_output_debug = 50时，pwm_value_debug应该 ≈ 250
   - 当pid_output_debug = 0时，pwm_value_debug应该 ≈ 500
```

**公式验证**:
```
pwm_value = (100 - pid_output) * 500 / 100
          = (100 - pid_output) * 5

例:
pid_output = 100 → pwm_value = 0 (最高速)
pid_output = 50  → pwm_value = 250 (中速)
pid_output = 0   → pwm_value = 500 (停止)
```

---

### 场景3: 可以达到2400但无法达到目标转速

**症状**:
```
目标: 2400 RPM
实际: 1800 RPM
pid_output_debug: 75
```

**分析**:
```
PWM与RPM应该线性关系:
pwm_value = 500 * (1 - pid_output/100)

pid_output = 75% → pwm_value ≈ 125 → RPM ≈ 1800
pid_output = 100% → pwm_value = 0 → RPM ≈ 2400

问题可能在于:
1. PWM在max_output时没有达到真正的0？
2. 或者电机特性不是线性的？
3. 或者PID还未充分加速？
```

**修复方法**:
```c
// 在Motor_PID_Update中检查max output是否真的输出
if (speed_pid.output >= PID_OUTPUT_LIMIT) {
  // 强制设置为100%
  Motor_SetSpeed(100.0f);
}
```

---

## 快速检查表

在Debug窗口添加这些变量并运行：

### 启动电机，设置目标2400RPM

```
检查项                     | 预期值范围  | 实际值 | 状态
--------------------------|------------|--------|-------
pid_control_enable_debug   | 1.0        |        | [ ]
pid_pid_enable_debug       | 1.0        |        | [ ]
motor_rpm (初始)           | 0          |        | [ ]
pid_error_debug (初始)     | ~2400      |        | [ ]
pid_output_debug (初始)    | ~100       |        | [ ]
pwm_value_debug (初始)     | ~0         |        | [ ]
motor_rpm (1秒后)          | 1500-2300  |        | [ ]
pid_error_debug (1秒后)    | 100-500    |        | [ ]
pid_output_debug (最终)    | 1-10       |        | [ ]
motor_rpm (最终)           | 2350-2400  |        | [ ]
```

---

## 如果所有都正常但仍无法达到目标

**检查PWM的实际行为**:

```c
// 添加这段测试代码到main.c
void test_pwm_response(void)
{
  Motor_Start();
  
  // 测试1: PWM=0
  Motor_SetSpeed(100);
  HAL_Delay(1000);
  rpm_at_pwm_0 = motor_rpm;
  
  // 测试2: PWM=50%
  Motor_SetSpeed(50);
  HAL_Delay(1000);
  rpm_at_pwm_50 = motor_rpm;
  
  // 测试3: PWM=100% (停止)
  Motor_SetSpeed(0);
  HAL_Delay(1000);
  rpm_at_pwm_100 = motor_rpm;
}
```

**预期结果**:
```
rpm_at_pwm_0:   ~2400 (最高速)
rpm_at_pwm_50:  ~1200 (中速)
rpm_at_pwm_100: ~0    (停止)

如果不符合这个线性关系，可能:
1. 电机硬件特性非线性
2. PWM在某个范围没有应用
3. 需要重新标定电机特性
```

---

## 总结 - 调试优先级

1. **最关键**: 检查 `pid_control_enable_debug` 和 `pid_pid_enable_debug` 是否为1
   
2. **第二关键**: 检查 `pid_output_debug` 是否在变化
   
3. **第三关键**: 检查 `pwm_value_debug` 是否正确响应 `pid_output_debug`
   
4. **最后**: 检查 `motor_rpm` 是否正确响应 `pwm_value_debug`

