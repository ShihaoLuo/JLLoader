# APP CAN Protocol - 快速参考

## 常用API速查

### 初始化
```c
AppCANProtocol_Init(&hcan1);              // 初始化协议
AppCANProtocol_StartDiscovery();          // 启动设备发现
AppCANProtocol_Task();                    // 主循环任务（必须调用）
```

### 电机控制
```c
// 启动/停止
AppCANProtocol_StartMotor(id, speed, dir);     // 启动电机
AppCANProtocol_StopMotor(id);                  // 停止电机
AppCANProtocol_EmergencyStopAll();             // 紧急停止所有

// 速度控制
AppCANProtocol_SetMotorSpeed(id, speed);       // 设置速度

// 状态查询
AppCANProtocol_QueryMotorStatus(id);           // 发送查询
MotorStatus_t* status = AppCANProtocol_GetMotorStatus(id);  // 获取状态
```

### 传感器
```c
AppCANProtocol_QuerySensor(id, type);          // 查询数据
SensorData_t* data = AppCANProtocol_GetSensorData(id);  // 获取数据
```

### IO设备
```c
AppCANProtocol_ControlIO(id, port, mask, val); // 控制IO
IOStatus_t* io = AppCANProtocol_GetIOStatus(id);  // 获取状态
```

### 设备管理
```c
uint8_t count = AppCANProtocol_GetOnlineNodeCount();  // 在线设备数
NodeInfo_t* node = AppCANProtocol_GetNodeInfo(id);    // 获取节点信息
AppCANProtocol_ListDevices();                         // 列出所有设备
```

## CAN ID计算

```c
// 计算CAN ID
#define CAN_ID(func, addr)  (((func) << 6) | (addr))

// 示例
CAN_ID(CAN_FUNC_MOTOR_CMD, 0x01)     // 0x101 - 控制电机1
CAN_ID(CAN_FUNC_HEARTBEAT, 0x3F)     // 0x07F - 心跳广播
CAN_ID(CAN_FUNC_DISCOVERY, 0x3F)     // 0x0BF - 发现广播
```

## 设备ID范围

| 设备类型 | ID范围 | 节点地址范围 | 示例 |
|---------|--------|------------|------|
| 主机     | -      | 0x00       | 主机APP |
| 电机     | 1-20   | 0x01-0x14  | 电机1=0x01 |
| 传感器   | 1-20   | 0x15-0x28  | 传感器1=0x15 |
| IO设备   | 1-20   | 0x29-0x3C  | IO1=0x29 |
| 广播     | -      | 0x3F       | 广播地址 |

## 功能码列表

| 功能码 | 名称 | 说明 |
|-------|------|-----|
| 0x00  | SYSTEM | 系统管理 |
| 0x01  | HEARTBEAT | 心跳 |
| 0x02  | DISCOVERY | 设备发现 |
| 0x04  | MOTOR_CMD | 电机控制命令 |
| 0x05  | MOTOR_RESP | 电机控制响应 |
| 0x06  | MOTOR_QUERY | 电机状态查询 |
| 0x07  | MOTOR_STATUS | 电机状态上报 |
| 0x10  | SENSOR_QUERY | 传感器查询 |
| 0x11  | SENSOR_DATA | 传感器数据上报 |
| 0x14  | IO_CMD | IO控制 |
| 0x15  | IO_STATUS | IO状态上报 |
| 0x1F  | BROADCAST | 广播命令 |

## 电机命令

```c
typedef enum {
    MOTOR_CMD_STOP          = 0x00,  // 停止
    MOTOR_CMD_START         = 0x01,  // 启动
    MOTOR_CMD_EMERGENCY_STOP = 0x02, // 急停
    MOTOR_CMD_SET_SPEED     = 0x03,  // 设置速度
    MOTOR_CMD_SET_DIRECTION = 0x04,  // 设置方向
    MOTOR_CMD_SET_BOTH      = 0x05   // 同时设置速度和方向
} MotorCommand_t;
```

## 电机方向

```c
typedef enum {
    MOTOR_DIR_STOP  = 0x00,  // 停止
    MOTOR_DIR_CW    = 0x01,  // 顺时针
    MOTOR_DIR_CCW   = 0x02   // 逆时针
} MotorDirection_t;
```

## 传感器类型

```c
typedef enum {
    SENSOR_TYPE_TEMPERATURE = 0x01,  // 温度传感器
    SENSOR_TYPE_HUMIDITY    = 0x02,  // 湿度传感器
    SENSOR_TYPE_PRESSURE    = 0x03,  // 压力传感器
    SENSOR_TYPE_DISTANCE    = 0x04,  // 距离传感器
    SENSOR_TYPE_ALL         = 0xFF   // 所有传感器
} SensorType_t;
```

## 时间参数

| 参数 | 值 | 说明 |
|-----|---|------|
| HEARTBEAT_INTERVAL | 500ms | 心跳发送间隔 |
| HEARTBEAT_TIMEOUT | 3000ms | 心跳超时判断 |
| DISCOVERY_DELAY | 1000ms | 启动后延迟发现 |
| DISCOVERY_TIMEOUT | 2000ms | 发现响应超时 |
| HEARTBEAT_CHECK_INTERVAL | 100ms | 心跳检查间隔 |

## 完整示例

### 基本控制流程
```c
int main(void) {
    // 1. 初始化
    App_Init();
    AppCANProtocol_Init(&hcan1);
    
    // 2. 延迟后启动发现
    HAL_Delay(1000);
    AppCANProtocol_StartDiscovery();
    
    // 3. 主循环
    while (1) {
        AppCANProtocol_Task();  // 必须调用
        
        // 4. 发现完成后控制设备
        if (g_device_manager.discovery_done) {
            // 启动电机1
            AppCANProtocol_StartMotor(1, 1000, MOTOR_DIR_CW);
            HAL_Delay(5000);
            
            // 查询状态
            AppCANProtocol_QueryMotorStatus(1);
            HAL_Delay(100);
            
            // 获取状态
            MotorStatus_t *status = AppCANProtocol_GetMotorStatus(1);
            if (status != NULL) {
                // 使用状态数据
            }
            
            // 停止电机
            AppCANProtocol_StopMotor(1);
        }
        
        HAL_Delay(10);
    }
}
```

### 高级控制示例
```c
// 完整的电机控制参数
MotorControl_t control = {
    .command = MOTOR_CMD_SET_BOTH,
    .target_speed = 2000,
    .direction = MOTOR_DIR_CW,
    .accel = 100,  // 100 RPM/s
    .decel = 50    // 50 RPM/s
};
AppCANProtocol_ControlMotor(1, &control);

// 查询所有在线设备
uint8_t count = AppCANProtocol_GetOnlineNodeCount();
for (uint8_t i = 0; i < MAX_NODES; i++) {
    NodeInfo_t *node = &g_device_manager.nodes[i];
    if (node->is_valid && node->status == NODE_STATUS_ONLINE) {
        // 处理在线设备
    }
}

// 传感器数据读取
AppCANProtocol_QuerySensor(1, SENSOR_TYPE_TEMPERATURE);
HAL_Delay(50);
SensorData_t *data = AppCANProtocol_GetSensorData(1);
if (data != NULL && data->quality > 80) {
    // 数据质量良好，使用数据
    float temp = data->value;
}

// IO端口批量控制
for (uint8_t port = 0; port < 8; port++) {
    AppCANProtocol_ControlIO(1, port, 0xFF, port * 0x10);
    HAL_Delay(10);
}
```

## 错误处理

### 检查返回值
```c
if (!AppCANProtocol_StartMotor(1, 1000, MOTOR_DIR_CW)) {
    // 失败处理
    UART_Printf("Failed to start motor 1\n");
}
```

### 检查设备在线
```c
NodeInfo_t *node = AppCANProtocol_GetNodeInfo(motor_node_id);
if (node != NULL && node->status == NODE_STATUS_ONLINE) {
    // 设备在线，可以控制
    AppCANProtocol_StartMotor(motor_id, speed, dir);
} else {
    // 设备离线
    UART_Printf("Motor %d is offline\n", motor_id);
}
```

### 数据有效性检查
```c
MotorStatus_t *status = AppCANProtocol_GetMotorStatus(1);
if (status == NULL) {
    // 数据不存在或已过期
    // 重新查询
    AppCANProtocol_QueryMotorStatus(1);
    HAL_Delay(100);
    status = AppCANProtocol_GetMotorStatus(1);
}
```

## 调试技巧

### 1. 启用详细日志
所有关键操作都会通过UART输出日志，连接串口监视器查看。

### 2. 查看设备列表
```c
AppCANProtocol_ListDevices();
```

### 3. 监控心跳
主机每500ms发送心跳，设备响应。检查UART日志确认通信正常。

### 4. 检查CAN总线
- 确认波特率：500kbps
- 检查终端电阻：120Ω
- 测量CAN_H和CAN_L电压差

## 常见问题

### Q: 发现不到设备？
A: 
1. 检查从机是否上电
2. 确认从机ID配置
3. 检查CAN总线连接
4. 延长发现超时时间

### Q: 设备频繁掉线？
A:
1. 检查心跳响应实现
2. 确认从机定时发送心跳
3. 检查总线干扰
4. 增加超时时间

### Q: 控制命令无效？
A:
1. 确认设备在线
2. 检查命令参数
3. 查看响应消息
4. 验证校验和

### Q: 如何添加新设备？
A:
1. 分配节点ID（注意范围）
2. 实现设备发现响应
3. 实现心跳响应
4. 实现对应的控制命令

---

**版本**: v1.0  
**最后更新**: 2025-10-14
