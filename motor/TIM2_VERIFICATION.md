# TIM2替换验证文档

## 问题描述
原始实现中，使用TIM3（APB2, 72MHz）进行RPM检测和PID更新。
要求改用TIM2（APB1, 36MHz），但需要**保持检测频率不变**。

---

## 解决方案

### 参数计算

#### TIM3（原始配置，72MHz）
```
系统时钟：72MHz
Prescaler：7199
Period：199

中断周期计算：
T = (Prescaler + 1) × (Period + 1) / 时钟频率
  = (7199 + 1) × (199 + 1) / 72MHz
  = 7200 × 200 / 72MHz
  = 1,440,000 / 72,000,000
  = 0.02秒 = 20ms ✓

中断频率：
F = 1 / T = 1 / 0.02 = 50Hz ✓
```

#### TIM2（新配置，36MHz）
```
系统时钟：36MHz
Prescaler：7199
Period：99  <- 关键修改

中断周期计算：
T = (Prescaler + 1) × (Period + 1) / 时钟频率
  = (7199 + 1) × (99 + 1) / 36MHz
  = 7200 × 100 / 36MHz
  = 720,000 / 36,000,000
  = 0.02秒 = 20ms ✓

中断频率：
F = 1 / T = 1 / 0.02 = 50Hz ✓
```

### 验证结果
| 参数 | TIM3(旧) | TIM2(新) | 一致性 |
|------|---------|---------|-------|
| 时钟频率 | 72MHz | 36MHz | 差异 |
| Prescaler | 7199 | 7199 | ✓ |
| Period | 199 | 99 | 调整 |
| 计时频率 | 10kHz | 5kHz | 差异 |
| **中断周期** | **20ms** | **20ms** | **✓ 一致** |
| **中断频率** | **50Hz** | **50Hz** | **✓ 一致** |

---

## 修改的代码文件

### 1. motor_init.c - MX_TIM2_Init()
```c
htim2.Init.Prescaler = 7199;  // 36MHz / 7200 = 5kHz
htim2.Init.Period = 99;       // 5kHz / 100 = 50Hz (20ms)
```

### 2. motor_ctrl.c - HAL_TIM_PeriodElapsedCallback()
```c
if (htim->Instance == TIM2)  // 从TIM3改为TIM2
{
  /* TIM2每20ms触发一次RPM检测更新（50Hz，与原TIM3频率一致） */
  Motor_RPM_Detection_Update();
}
```

### 3. stm32f1xx_hal_msp.c - HAL_TIM_Base_MspInit()
```c
if(htim_base->Instance==TIM2)  // 从TIM3改为TIM2
{
  __HAL_RCC_TIM2_CLK_ENABLE();
  HAL_NVIC_SetPriority(TIM2_IRQn, 1, 0);
  HAL_NVIC_EnableIRQ(TIM2_IRQn);
}
```

### 4. stm32f1xx_it.c - TIM2_IRQHandler()
```c
void TIM2_IRQHandler(void)  // 从TIM3_IRQHandler改为
{
  HAL_TIM_IRQHandler(&htim2);  // 从&htim3改为
}
```

---

## 功能验证清单

### RPM检测频率
- [x] TIM2中断频率 = 50Hz
- [x] 中断周期 = 20ms（与原TIM3相同）
- [x] 在此周期内计算RPM，逻辑不变

### PID更新频率
- [x] PID在TIM2中断中更新
- [x] 更新频率 = 50Hz（与原TIM3相同）
- [x] PID控制性能应无变化

### 系统时钟分布
- [x] TIM4(PWM) - APB2 72MHz（不变）
- [x] TIM2(RPM/PID) - APB1 36MHz（替换TIM3）
- [x] TIM3 - 已释放，可用于其他功能

---

## 关键要点总结

1. **时钟树差异被参数补偿**
   - TIM3: 72MHz → Prescaler/Period = 7200/200
   - TIM2: 36MHz → Prescaler/Period = 7200/100
   - 结果：中断周期相同 = 20ms

2. **检测逻辑完全相同**
   - RPM检测使用的pulse_count计数不变
   - RPM计算公式不变：RPM = (pulse_count × 3000) / PPR
   - PID参数和算法不变

3. **性能无劣化**
   - RPM检测频率保持50Hz
   - PID更新周期保持20ms
   - 系统响应时间不变

4. **资源优化**
   - TIM3完全释放，可用于其他功能
   - PWM(TIM4)和RPM/PID(TIM2)分别在两条总线上
   - 减少单个总线负载

---

## 测试建议

### 硬件验证
1. 示波器测量TIM2_IRQHandler中断间隔，应为20ms
2. 示波器测量PWM输出频率，应保持~500Hz
3. 观察LED闪烁频率（如用于中断指示）

### 软件验证
1. 检查motor_rpm变量更新频率
2. 验证PID输出的响应性
3. 对比原TIM3和新TIM2的控制曲线

### 长期稳定性
1. 运行时间：≥1小时
2. 监测RPM计算精度
3. 检查PID错误积分是否正常衰减

