# APP CAN Protocol - 使用说明

## 概述

本文档介绍了在 `app` 项目中实现的主机端CAN协议系统，包括设备管理、心跳发现、以及多设备类型控制功能。

## 文件结构

```
app/Core/
├── Inc/
│   ├── app_can_protocol.h    # CAN协议头文件
│   ├── app_init.h             # 应用初始化头文件
│   └── main.h                 # 主程序头文件
└── Src/
    ├── app_can_protocol.c     # CAN协议实现文件
    ├── app_init.c             # 应用初始化实现
    └── main.c                 # 主程序（集成协议调用）
```

## 功能特性

### 1. 设备管理服务
- **自动设备发现**：上电后延迟1秒自动发送设备发现广播
- **设备表管理**：自动维护所有在线设备信息
- **设备类型识别**：支持电机、传感器、IO设备等多种类型
- **在线状态跟踪**：实时监控设备在线/离线状态

### 2. 心跳机制
- **心跳广播**：主机每500ms发送一次心跳广播
- **心跳响应**：从机接收到心跳后响应状态信息
- **超时检测**：3秒内未收到心跳则标记设备离线
- **自动恢复**：设备重新上线后自动恢复

### 3. 电机控制
- 启动/停止电机
- 设置转速和方向
- 查询电机状态
- 紧急停止所有电机
- 实时监控电机参数（转速、电流、温度、故障）

### 4. 传感器支持
- 查询传感器数据
- 支持多种传感器类型（温度、湿度、压力、距离）
- 数据质量评估

### 5. IO设备控制
- 端口控制
- 状态查询
- 位掩码操作

## 协议架构

### CAN ID分配方案
```
 10  9  8  7  6  5  4  3  2  1  0
[      功能码      ][   节点ID   ]
  5位(0-31)           6位(0-63)
```

**功能码定义：**
- `0x00`: 系统管理
- `0x01`: 心跳
- `0x02`: 设备发现
- `0x04`: 电机控制命令
- `0x05`: 电机控制响应
- `0x06`: 电机状态查询
- `0x07`: 电机状态上报
- `0x10`: 传感器查询
- `0x11`: 传感器数据上报
- `0x14`: IO控制
- `0x15`: IO状态上报
- `0x1F`: 广播命令

**节点ID分配：**
- `0x00`: 主机（APP）
- `0x01-0x14`: 电机节点（1-20）
- `0x15-0x28`: 传感器节点（21-40）
- `0x29-0x3C`: IO设备节点（41-60）
- `0x3F`: 广播地址

## API使用指南

### 初始化

```c
#include "app_can_protocol.h"

// 在main函数中初始化
AppCANProtocol_Init(&hcan1);
```

### 启动设备发现

```c
// 延迟1秒后启动发现
HAL_Delay(1000);
AppCANProtocol_StartDiscovery();
```

### 协议定时任务

```c
// 在主循环中调用
while (1) {
    AppCANProtocol_Task();  // 处理心跳和超时检查
    HAL_Delay(10);
}
```

### 电机控制示例

#### 启动电机
```c
// 启动电机1，正转1000 RPM
AppCANProtocol_StartMotor(1, 1000, MOTOR_DIR_CW);

// 启动电机2，反转500 RPM
AppCANProtocol_StartMotor(2, 500, MOTOR_DIR_CCW);
```

#### 停止电机
```c
// 停止电机1
AppCANProtocol_StopMotor(1);

// 紧急停止所有电机
AppCANProtocol_EmergencyStopAll();
```

#### 设置电机速度
```c
// 设置电机1速度为1500 RPM
AppCANProtocol_SetMotorSpeed(1, 1500);
```

#### 查询电机状态
```c
// 发送查询命令
AppCANProtocol_QueryMotorStatus(1);

// 延迟等待响应
HAL_Delay(50);

// 获取状态
MotorStatus_t *status = AppCANProtocol_GetMotorStatus(1);
if (status != NULL) {
    printf("Speed: %d RPM\n", status->actual_speed);
    printf("Direction: %d\n", status->direction);
    printf("Current: %d mA\n", status->current);
    printf("Temperature: %d°C\n", status->temperature);
    printf("Fault: 0x%02X\n", status->fault);
}
```

#### 高级控制
```c
// 使用完整的控制参数
MotorControl_t control = {
    .command = MOTOR_CMD_SET_BOTH,
    .target_speed = 2000,
    .direction = MOTOR_DIR_CW,
    .accel = 100,  // 加速度 100 RPM/s
    .decel = 50    // 减速度 50 RPM/s
};
AppCANProtocol_ControlMotor(1, &control);
```

### 传感器控制示例

#### 查询传感器
```c
// 查询传感器1的所有数据
AppCANProtocol_QuerySensor(1, SENSOR_TYPE_ALL);

// 查询传感器2的温度数据
AppCANProtocol_QuerySensor(2, SENSOR_TYPE_TEMPERATURE);
```

#### 获取传感器数据
```c
SensorData_t *data = AppCANProtocol_GetSensorData(1);
if (data != NULL) {
    printf("Type: %d\n", data->sensor_type);
    printf("Value: %.2f\n", data->value);
    printf("Quality: %d%%\n", data->quality);
}
```

### IO设备控制示例

#### 控制IO端口
```c
// 控制IO设备1的端口0，设置所有位为0xAA
AppCANProtocol_ControlIO(1, 0, 0xFF, 0xAA);

// 控制IO设备2的端口3，只设置低4位为0x0F
AppCANProtocol_ControlIO(2, 3, 0x0F, 0x0F);
```

#### 获取IO状态
```c
IOStatus_t *io_status = AppCANProtocol_GetIOStatus(1);
if (io_status != NULL) {
    for (uint8_t i = 0; i < 8; i++) {
        printf("Port %d: 0x%02X\n", i, io_status->port_status[i]);
    }
}
```

### 设备管理示例

#### 获取在线设备数量
```c
uint8_t count = AppCANProtocol_GetOnlineNodeCount();
printf("Online devices: %d\n", count);
```

#### 获取节点信息
```c
NodeInfo_t *node = AppCANProtocol_GetNodeInfo(0x01);
if (node != NULL) {
    printf("Node ID: 0x%02X\n", node->node_id);
    printf("Device Type: %d\n", node->device_type);
    printf("HW Version: %d.%d\n", 
           node->hw_version >> 4, 
           node->hw_version & 0x0F);
    printf("SW Version: %d.%d\n", 
           node->sw_version >> 8, 
           node->sw_version & 0xFF);
}
```

#### 列出所有设备
```c
// 打印所有在线设备的详细信息
AppCANProtocol_ListDevices();
```

## 工作流程

### 1. 系统启动流程

```
时间    主机APP                         从机节点
--------|-------------------------------|---------------------------
0ms     上电，初始化CAN
100ms   初始化协议管理器
1000ms  发送设备发现广播 ------------>
1100ms                                  收到发现请求
1200ms  <---------- 响应设备信息        节点1发送设备信息
1300ms  记录节点1到设备表
1400ms  <---------- 响应设备信息        节点2发送设备信息
1500ms  记录节点2到设备表
...
3000ms  发现完成，列出所有设备
3500ms  开始定时心跳维护 ------------>
        每500ms发送心跳
        每100ms检查超时
```

### 2. 心跳维护流程

```
主机                                    从机
1. 每500ms发送心跳广播 ----------->
                                        2. 收到心跳，更新状态
                <-------- 3. 响应心跳   发送运行时间、错误码
4. 更新节点心跳时间
5. 每100ms检查超时
   - 如果3秒未收到心跳
   - 标记节点离线
   - 电机节点自动停止
```

### 3. 电机控制流程

```
主机                                    电机节点
1. 发送控制命令 (0x104) ---------->
                                        2. 接收命令，解析参数
                                        3. 执行控制（启动/停止/调速）
                <-------- 4. 响应结果   发送成功/失败状态
5. 收到响应，更新状态
6. 发送状态查询 (0x184) ---------->
                                        7. 读取实际状态
                <-------- 8. 上报状态   发送转速/电流/温度
9. 更新电机状态表
10. 应用层可随时查询状态
```

## 注意事项

### 1. 初始化顺序
必须按以下顺序初始化：
```c
App_Init();                      // 初始化HAL、CAN等
AppCANProtocol_Init(&hcan1);     // 初始化协议
HAL_Delay(1000);                 // 延迟1秒
AppCANProtocol_StartDiscovery(); // 启动发现
```

### 2. 主循环调用
在主循环中必须调用：
```c
while (1) {
    AppCANProtocol_Task();       // 协议定时任务（必须）
    App_CAN_ProcessUARTOutput(); // UART输出处理（可选）
    HAL_Delay(10);               // 短暂延迟
}
```

### 3. 设备ID范围
- 电机ID: 1-20
- 传感器ID: 1-20
- IO设备ID: 1-20
- 超出范围会返回错误

### 4. 数据有效性
获取设备状态时，需检查返回值：
```c
MotorStatus_t *status = AppCANProtocol_GetMotorStatus(1);
if (status == NULL) {
    // 设备不存在或数据过期
}
```

### 5. 超时处理
- 心跳超时：3秒
- 发现超时：2秒
- 电机状态有效期：3秒
- 传感器数据有效期：5秒

### 6. 中断安全
CAN接收在中断中处理，会自动调用协议回调。UART输出在中断中缓冲，主循环中发送，避免阻塞。

## 调试功能

### UART日志输出
系统会通过UART输出详细的调试信息：
- CAN发送/接收日志
- 设备发现日志
- 心跳维护日志
- 命令执行日志
- 状态更新日志

### 设备列表查看
```c
AppCANProtocol_ListDevices();
```
输出示例：
```
========== Device List ==========
Total Nodes: 2
  [0] ID:0x01 Type:MOTOR Status:ONLINE HW:1.0 SW:1.0
  [1] ID:0x02 Type:MOTOR Status:ONLINE HW:1.0 SW:1.0
================================
```

## 测试示例

参考 `main.c` 中的 `CAN_TestMotorControl()` 函数，包含完整的测试流程：
1. 启动电机（正转1000 RPM）
2. 查询状态
3. 改变速度（1500 RPM）
4. 反转电机（500 RPM）
5. 停止电机
6. 列出所有设备
7. 测试传感器
8. 测试IO设备

测试每10秒执行一个阶段，可以观察完整的控制流程。

## 扩展开发

### 添加新的设备类型
1. 在 `app_can_protocol.h` 中添加新的功能码
2. 在 `DeviceType_t` 中添加新的设备类型
3. 实现对应的控制函数和状态结构
4. 在 `AppCANProtocol_RxCallback()` 中添加消息处理

### 自定义控制命令
参考 `AppCANProtocol_ControlMotor()` 的实现方式：
1. 定义命令数据格式
2. 构造CAN消息
3. 调用 `AppCANProtocol_SendMessage()` 发送
4. 实现响应处理函数

## 性能指标

- **波特率**: 500 kbps
- **最大节点数**: 62个从机 + 1个主机
- **心跳周期**: 500ms
- **超时判断**: 3秒
- **控制响应时间**: < 10ms
- **状态更新周期**: 可配置

## 故障排除

### 问题1：发现不到设备
- 检查CAN总线连接
- 确认从机ID配置正确
- 检查从机是否响应发现广播

### 问题2：设备频繁掉线
- 检查心跳超时设置
- 确认CAN总线干扰情况
- 检查从机心跳响应实现

### 问题3：控制命令无响应
- 确认设备在线状态
- 检查命令格式和校验和
- 查看UART日志确认发送成功

## 相关文档

- `can_protocol.md` - CAN协议详细规范
- `protocol.md` - 通用协议说明
- `i2c_protocol.md` - I2C协议说明（如适用）

---

**版本**: v1.0  
**日期**: 2025-10-14  
**作者**: Generated  
**项目**: JLLoader - CAN主机端协议实现
