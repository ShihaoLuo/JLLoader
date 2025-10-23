# FR引脚改为PA6 - 改动总结 (已修正逻辑)

## 改动概述
将电机方向控制(FR)引脚从PB5改为PA6，经过实测发现逻辑为：
- **PA6 = 低电平(0)** → 顺时针
- **PA6 = 高电平(1)** → 逆时针

> **注意**：初始文档有误，已根据实测结果修正

---

## 修改的文件
## 总结

✓ **FR引脚成功改为PA6**
✓ **逻辑定义为低=顺时针，高=逆时针**（已根据实测修正）
✓ **不影响其他功能**
✓ **代码已完全更新**

系统现在使用：
- **PA7**：PWM速度控制(TIM3_CH2)
- **PA6**：方向控制(GPIO推挽) - 低电平顺时针，高电平逆时针
- **PB4**：启停控制(GPIO推挽)
- **PB7**：脉冲输入(EXTI中断)tor_init.c** - MX_GPIO_Init()

#### 移除：
```c
// 旧配置
HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5, GPIO_PIN_SET);  // FR: 1=顺时针, 0=逆时针

/*Configure GPIO pin : PB5 (FR - 方向控制) */
GPIO_InitStruct.Pin = GPIO_PIN_5;
GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
GPIO_InitStruct.Pull = GPIO_NOPULL;
GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
```

#### 添加：
```c
// 新配置
HAL_GPIO_WritePin(GPIOA, GPIO_PIN_6, GPIO_PIN_SET);  // FR: 1=正转, 0=反转

/*Configure GPIO pin : PA6 (FR - 方向控制) */
GPIO_InitStruct.Pin = GPIO_PIN_6;
GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
GPIO_InitStruct.Pull = GPIO_NOPULL;
GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
```

---

### 2. **motor_ctrl.c** - Motor_SetDirection()

#### 修改：
```c
// 旧代码（已修正）
void Motor_SetDirection(uint8_t clockwise)
{
  motor_direction = clockwise;
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5, clockwise ? GPIO_PIN_SET : GPIO_PIN_RESET);
}

// 新代码（实测修正后）
void Motor_SetDirection(uint8_t clockwise)
{
  motor_direction = clockwise;
  // 反转逻辑：clockwise=1时输出低电平，clockwise=0时输出高电平
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_6, clockwise ? GPIO_PIN_RESET : GPIO_PIN_SET);
}
```

#### 注释更新：
```c
// 旧：@param  clockwise: 1=正转(高电平), 0=反转(低电平)
// 新：@param  clockwise: 1=顺时针(低电平), 0=逆时针(高电平)
```

---

### 3. **motor_ctrl.h** - 硬件配置注释

#### 更新：
```c
// 旧
 *    - PWM输出：PB6 (TIM4_CH1，反向逻辑控制)
 *    - 控制信号：PB4(BK启停), PB5(FR方向)

// 新
 *    - PWM输出：PA7 (TIM3_CH2，反向逻辑控制)
 *    - 控制信号：PB4(BK启停), PA6(FR方向)
```

---

## 引脚配置最终状态

| 功能 | 原引脚 | 新引脚 | 模式 | 逻辑 |
|------|-------|-------|------|------|
| **PWM输出** | PB6 | PA7 | 复用推挽 | 占空比控制速度 |
| **启停(BK)** | PB4 | PB4 | 推挽输出 | 1=运行, 0=停止 |
| **方向(FR)** | PB5 | PA6 | 推挽输出 | 0=顺时针, 1=逆时针 |
| **脉冲(FG)** | PB7 | PB7 | IT下降沿 | 霍尔传感器 |

---

## GPIO分布图

### 端口A
```
PA6  ← FR (方向控制) - 新增
PA7  ← PWM (TIM3_CH2) - 已有
```

### 端口B
```
PB3  ← 备用输出
PB4  ← BK (启停控制)
PB5  ✗ (已释放)
PB7  ← FG (脉冲输入)
```

---

## 兼容性检查

✓ **PA6不与其他功能冲突**
- CAN使用：PA11, PA12
- PWM使用：PA7
- 其他GPIO：无冲突

✓ **PA7 PWM已正常工作**
- 已配置为TIM3_CH2输出
- 已测试成功输出方波
- 不影响FR功能

---

## 代码验证

### 调用示例

```c
// 顺时针
Motor_SetDirection(1);  // PA6 = 低电平

// 逆时针
Motor_SetDirection(0);  // PA6 = 高电平
```

### 实现逻辑

```c
// 设置顺时针
Motor_Start();
Motor_SetDirection(1);  // PA6 = 0 (低电平)
Motor_SetSpeed(50.0f);  // 50% PWM

// 设置逆时针
Motor_SetDirection(0);  // PA6 = 1 (高电平)
Motor_SetSpeed(50.0f);  // 同样50% PWM，但方向反转
```

---

## 测试建议

1. **静态测试**
   - [ ] PA6接示波器，观察GPIO电平变化
   - [ ] 调用Motor_SetDirection(1)，PA6应为高电平
   - [ ] 调用Motor_SetDirection(0)，PA6应为低电平

2. **功能测试**
   - [ ] 启动电机，设置正转
   - [ ] 启动电机，设置反转
   - [ ] 观察电机实际转向是否符合预期

3. **集成测试**
   - [ ] CAN协议能否正确控制方向
   - [ ] 方向切换时是否平稳
   - [ ] 与速度控制配合是否正常

---

## 总结

✓ **FR引脚成功改为PA6**
✓ **逻辑定义为高=正转，低=反转**
✓ **不影响其他功能**
✓ **代码已完全更新**

系统现在使用：
- **PA7**：PWM速度控制(TIM3_CH2)
- **PA6**：方向控制(GPIO推挽)
- **PB4**：启停控制(GPIO推挽)
- **PB7**：脉冲输入(EXTI中断)

