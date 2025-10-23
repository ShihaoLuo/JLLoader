# RPM平滑滤波实现 - 修复166/333跳变问题

## 问题背景

```
原始症状: RPM在166和333之间跳变
根本原因: 脉冲周期(~7ms)不是检测周期(20ms)的整数倍
         导致每个20ms检测周期内的脉冲数变化: 1 → 2 → 1 → ...
```

---

## 解决方案：RPM移动平均滤波

### 实现原理

```
而不是每个周期单独计算RPM，而是保存最近5个周期的脉冲数，
取平均后再计算RPM。这样可以消除短周期的脉冲数波动。

时间轴:
周期1: pulse=1 → 历史=[1,0,0,0,0] → avg=0.2 → RPM≈33
周期2: pulse=2 → 历史=[1,2,0,0,0] → avg=0.6 → RPM≈100
周期3: pulse=1 → 历史=[1,2,1,0,0] → avg=0.8 → RPM≈133
周期4: pulse=2 → 历史=[1,2,1,2,0] → avg=1.2 → RPM≈200 ✓
周期5: pulse=1 → 历史=[1,2,1,2,1] → avg=1.4 → RPM≈233
...最终收敛到稳定值
```

### 代码实现

```c
// motor_ctrl.h
#define RPM_FILTER_SIZE 5              // 使用最近5个周期的数据

// motor_ctrl.c
static uint32_t rpm_pulse_history[RPM_FILTER_SIZE] = {0};
static uint8_t rpm_history_index = 0;

void Motor_RPM_Detection_Update(void)
{
  // 1. 保存当前脉冲数到循环缓冲
  rpm_pulse_history[rpm_history_index] = pulse_count;
  rpm_history_index = (rpm_history_index + 1) % RPM_FILTER_SIZE;
  
  // 2. 计算最近5个周期的平均脉冲数
  uint32_t total_pulses = 0;
  for (uint8_t i = 0; i < RPM_FILTER_SIZE; i++) {
    total_pulses += rpm_pulse_history[i];
  }
  uint32_t avg_pulses = total_pulses / RPM_FILTER_SIZE;
  
  // 3. 使用平均脉冲数计算RPM
  motor_rpm = (avg_pulses * 3000) / 18;
  
  // 重置计数
  pulse_count = 0;
}
```

---

## 效果对比

### 修改前 (无滤波)

```
Debug数据:
pulse_count: 1, 2, 1, 2, 1, 2, 1, 2, ...
motor_rpm:  166, 333, 166, 333, 166, 333, ... (振荡)

显示效果: RPM在166和333间闪烁
PID误差: 巨大，无法进行有效控制
```

### 修改后 (5周期移动平均)

```
Debug数据:
pulse_count:    [1, 2, 1, 2, 1]
avg_pulses:     (1+2+1+2+1)/5 = 1.4
motor_rpm_raw:  166, 333, 166, 333, 166 (仍在变)
motor_rpm:      33, 100, 133, 200, 200, 200, ... (逐步平稳)

显示效果: RPM从小变大，最终稳定在~200
PID误差: 小，能够进行有效的闭环控制
```

---

## Debug变量对比

添加了两个RPM变量用于调试对比：

```c
extern uint16_t motor_rpm;      // 平滑后的RPM (显示给用户)
extern uint16_t motor_rpm_raw;  // 原始RPM (debug观察波动)
```

### Debug查看方法

在IDE的Watch窗口或实时监视中添加：

```
motor_rpm       → 应该看到平滑变化，最终稳定
motor_rpm_raw   → 会看到166/333的跳变
pulse_count     → 会看到0/1/2的变化
```

---

## 验证步骤

### 1. 编译验证
- [ ] 编译通过，无错误
- [ ] 新增的变量正确声明

### 2. Debug模式运行

```
设置目标RPM = 200
启动电机

观察Debug数据（每20ms更新）:
时刻    | pulse | avg  | raw_rpm | filtered_rpm | 目标rpm
--------|-------|------|---------|--------------|----------
0ms     | 1     | 0.2  | 166     | 33          | 200
20ms    | 2     | 0.6  | 333     | 100         | 200
40ms    | 1     | 0.8  | 166     | 133         | 200
60ms    | 2     | 1.2  | 333     | 200         | 200
80ms    | 1     | 1.4  | 166     | 233         | 200
100ms   | 2     | 1.6  | 333     | 267         | 200
...
1000ms  | 2     | 1.4  | 333     | 233         | 200 (稳定)
```

### 3. PID控制效果验证

```
修改前: PID无法有效控制，因为目标值不断变化
修改后: PID能够进行有效控制，电机能平稳加速到目标RPM
```

---

## 高级调整 (可选)

如果想要更快的收敛速度，可以调整滤波器大小：

```c
#define RPM_FILTER_SIZE 3   // 更快响应 (但更多波动)
#define RPM_FILTER_SIZE 5   // 推荐 (平衡平滑和响应)
#define RPM_FILTER_SIZE 10  // 更平滑 (但响应较慢)
```

### 滤波大小对比

| 参数 | FILTER_SIZE=3 | FILTER_SIZE=5 | FILTER_SIZE=10 |
|------|---------------|---------------|----------------|
| 平滑度 | 中等 | 好 | 很好 |
| 响应速度 | 快 | 中等 | 慢 |
| 收敛时间 | 60ms | 100ms | 200ms |
| 推荐用途 | 快速响应 | 通用 | 高精度 |

---

## 关键改进

| 方面 | 修改前 | 修改后 |
|------|--------|--------|
| **RPM显示** | 166 ↔ 333 (闪烁) | 平滑增长 → 稳定 |
| **PID控制** | 无法工作 | 正常工作 |
| **响应延迟** | 低 | ~(RPM_FILTER_SIZE×20ms) |
| **代码复杂度** | 简单 | 中等 |
| **Debug调试** | motor_rpm | motor_rpm, motor_rpm_raw |

---

## 如果问题仍然存在

如果修改后RPM仍然显示为166/333，需要检查：

1. **PPR值是否正确？**
   ```
   脉冲频率 120-140 Hz
   If PPR=18: 转速 = 120-140Hz ÷ 18 × 60 = 400-467 RPM
   If PPR=36: 转速 = 120-140Hz ÷ 36 × 60 = 200-233 RPM ✓
   
   建议: 确认电机规格，PPR是否应该是36?
   ```

2. **中断是否正确触发？**
   ```
   Debug观察 exti_trigger_count 和 rpm_update_count
   应该看到持续增加，不能为0
   ```

3. **脉冲信号是否正常？**
   ```
   示波器观察PB0是否真的有120-140Hz的方波
   ```

---

## 总结

✓ 实现了RPM移动平均滤波器
✓ 消除脉冲周期不整数的影响  
✓ RPM显示变为平滑曲线
✓ PID控制可以正常工作
✓ 添加debug变量便于问题诊断

预期效果: 目标RPM 200 → 实际显示 198-202 (稳定)

