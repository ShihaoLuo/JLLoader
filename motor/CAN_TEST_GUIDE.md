# Motor CAN功能测试指南

## 功能概述
在motor项目中添加了CAN1外设初始化，作为第二个CAN节点与app项目进行双节点CAN通信测试。

## 硬件配置

### CAN引脚配置
- **PA11**: CAN1_RX（接收）
- **PA12**: CAN1_TX（发送）

### CAN参数
- **波特率**: 500kbps
- **时钟源**: APB1总线（36MHz）
- **预分频器**: 8
- **位时序**: 
  - SJW = 1TQ
  - BS1 = 6TQ
  - BS2 = 2TQ
  - 总位时间 = 1 + 6 + 2 = 9TQ

### CAN模式
- **运行模式**: CAN_MODE_NORMAL（正常模式，需要双节点）
- **过滤器**: 接收所有标准ID（0x000 - 0x7FF）
- **FIFO**: 使用FIFO0接收消息

## 硬件连接

### 双节点CAN总线连接
```
App板 (STM32F103)          Motor板 (STM32F103)
    PA12 (CAN_TX) ----+-------- PA12 (CAN_TX)
    PA11 (CAN_RX) ----+-------- PA11 (CAN_RX)
         GND ---------+-------- GND
```

### 完整CAN总线拓扑
```
     App板                         Motor板
   (节点1)                        (节点2)
      |                              |
    [CAN                          [CAN
  Transceiver]                 Transceiver]
      |                              |
  CAN_H |------ 120Ω ------+------ 120Ω ------| CAN_H
  CAN_L |------------------+-------------------| CAN_L
                           |
                          GND
```

**重要说明**：
1. **必须使用CAN收发器**（如TJA1050、MCP2551）
2. **必须在总线两端添加120Ω终端电阻**
3. **GND必须共地**

## 软件实现

### 1. 已修改的文件

#### motor/Core/Inc/stm32f1xx_hal_conf.h
```c
#define HAL_CAN_MODULE_ENABLED  // 启用CAN HAL模块
```

#### motor/Core/Inc/motor_init.h
```c
extern CAN_HandleTypeDef hcan1;       // CAN1句柄
void Motor_CAN_Init(void);            // CAN初始化函数
void Motor_CAN_ReceiveTask(void);     // CAN接收任务
```

#### motor/Core/Src/motor_init.c
新增功能：
1. **CAN_HandleTypeDef hcan1** - CAN句柄定义
2. **Motor_CAN_Init()** - 完整的CAN初始化函数
3. **Motor_CAN_ReceiveTask()** - CAN消息接收和处理函数

CAN初始化在`Motor_System_Init()`中调用，作为系统初始化的最后一步。

#### motor/Core/Src/main.c
在主循环中添加了CAN接收任务调用：
```c
while (1)
{
    // ...电机控制代码...
    
    /* CAN接收任务 - 接收来自App节点的CAN消息 */
    Motor_CAN_ReceiveTask();
    
    HAL_Delay(10);
}
```

### 2. CAN接收功能
Motor_CAN_ReceiveTask()功能：
- 检查CAN FIFO0是否有接收到的消息
- 接收标准ID为0x123的消息（来自App）
- 收到消息后翻转PB3 LED指示灯
- 可扩展：添加响应消息发送功能

## 测试步骤

### 第1步：编译和下载
1. 编译motor项目（确保无错误）
2. 编译app项目（确保无错误）
3. 分别下载到两块STM32板

### 第2步：硬件连接
1. 连接两块板的CAN收发器：
   - CAN_H to CAN_H
   - CAN_L to CAN_L
   - GND to GND
2. 在总线两端各接一个120Ω电阻（CAN_H到CAN_L之间）
3. 确保电源正常供电

### 第3步：运行测试
1. 上电两块板
2. App板每秒发送一次CAN消息（ID=0x123）
3. Motor板接收到消息后应该翻转LED（PB3）
4. 使用示波器观察CAN_H和CAN_L波形

### 第4步：预期结果
**正常CAN波形特征**：
- CAN_H和CAN_L差分信号
- 显性位（0）：CAN_H=3.5V, CAN_L=1.5V，差值约2V
- 隐性位（1）：CAN_H=2.5V, CAN_L=2.5V，差值约0V
- 完整的ACK位（不再是错误标志）
- 无错误帧重传

**LED指示**：
- Motor板的PB3 LED应该以1Hz频率闪烁

## 故障排查

### 问题1：CAN总线无通信
**检查项**：
- [ ] 两块板是否都正确初始化了CAN
- [ ] 硬件连接是否正确
- [ ] 是否使用了CAN收发器
- [ ] 终端电阻是否正确连接（120Ω×2）
- [ ] GND是否共地

### 问题2：编译错误
**可能原因**：
- HAL_CAN_MODULE_ENABLED未定义
- 缺少CAN相关头文件包含

**解决方法**：
1. 检查stm32f1xx_hal_conf.h中的HAL_CAN_MODULE_ENABLED
2. 确保main.h包含了stm32f1xx_hal.h
3. 清理工程后重新编译

### 问题3：Motor不接收消息
**检查项**：
- [ ] CAN过滤器配置是否正确（应接收所有ID）
- [ ] FIFO是否正确配置
- [ ] Motor_CAN_ReceiveTask()是否在主循环中调用
- [ ] App发送的消息ID是否为0x123

## 对比App项目配置

| 参数 | App配置 | Motor配置 | 说明 |
|------|---------|-----------|------|
| CAN引脚 | PA11/PA12 | PA11/PA12 | 相同 |
| 波特率 | 500kbps | 500kbps | 必须相同 |
| 运行模式 | NORMAL | NORMAL | 双节点通信 |
| 过滤器 | 接收所有 | 接收所有 | 相同 |
| 发送功能 | ✅ 有 | ⚠️ 可选 | App主动发送，Motor接收 |
| 接收功能 | ⚠️ 可选 | ✅ 有 | Motor主要接收 |

## 扩展功能

### 添加Motor响应功能
在Motor_CAN_ReceiveTask()中可以添加响应代码：
```c
/* 发送响应消息回App */
CAN_TxHeaderTypeDef TxHeader;
uint8_t TxData[8] = {0x55, 0xAA, 0x12, 0x34, 0x56, 0x78, 0x9A, 0xBC};
uint32_t TxMailbox;

TxHeader.StdId = 0x456;              // 响应消息ID
TxHeader.ExtId = 0x00;
TxHeader.RTR = CAN_RTR_DATA;
TxHeader.IDE = CAN_ID_STD;
TxHeader.DLC = 8;
TxHeader.TransmitGlobalTime = DISABLE;

if (HAL_CAN_AddTxMessage(&hcan1, &TxHeader, TxData, &TxMailbox) != HAL_OK)
{
    // 发送失败处理
}
```

### 基于CAN的电机控制
可以通过CAN接收电机控制指令：
```c
if (RxHeader.StdId == 0x200)  // 电机控制命令ID
{
    uint16_t rpm = (RxData[0] << 8) | RxData[1];
    uint8_t direction = RxData[2];
    
    Motor_SetTargetRPM(rpm);
    if (direction == 0) {
        Motor_SetDirection(MOTOR_DIR_CW);
    } else {
        Motor_SetDirection(MOTOR_DIR_CCW);
    }
}
```

## 技术参考

### CAN波特率计算
```
CAN时钟 = APB1时钟 / Prescaler = 36MHz / 8 = 4.5MHz
位时间 = 1 + TimeSeg1 + TimeSeg2 = 1 + 6 + 2 = 9TQ
波特率 = CAN时钟 / 位时间 = 4.5MHz / 9 = 500kbps
```

### CAN采样点位置
```
采样点 = (1 + TimeSeg1) / 位时间 = (1 + 6) / 9 = 77.8%
```
这是CAN总线推荐的采样点位置（75%-80%）。

## 相关文档
- [CAN协议详解](../i2c_protocol.md)
- [App CAN初始化](../app/Core/Src/app_init.c)
- [App Main函数](../app/Core/Src/main.c)

## 更新日志
- 2025-01-XX: 创建Motor CAN功能
  - 添加CAN初始化函数
  - 添加CAN接收任务
  - 集成到主循环
