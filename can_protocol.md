# CAN总线多节点管理协议设计

## 1. 协议概述

### 1.1 系统架构
```
         CAN总线 (500kbps)
              |
    +---------+---------+---------+
    |         |         |         |
  APP      Motor1   Motor2   Sensor1  IO1
 (主机)    (从机)   (从机)    (从机)  (从机)
```

### 1.2 设计目标
- ✅ 主机自动发现从机节点
- ✅ 维护从机心跳，检测离线
- ✅ 支持多种设备类型（电机、传感器、IO等）
- ✅ 电机控制：转速、方向、启停
- ✅ 电机状态读取：实际转速、方向、故障
- ✅ 可扩展性：支持最多127个节点

## 2. CAN标识符分配

### 2.1 标准11位ID分配方案
```
 10  9  8  7  6  5  4  3  2  1  0
[      功能码      ][   节点ID   ]
  5位(0-31)           6位(0-63)
```

**功能码（5位）：0-31**
- `0x00`: 系统管理
- `0x01`: 心跳
- `0x02`: 设备发现
- `0x03`: 设备信息查询
- `0x04-0x0F`: 电机控制
- `0x10-0x13`: 传感器
- `0x14-0x17`: IO设备
- `0x1F`: 广播命令

**节点ID（6位）：0-63**
- `0x00`: 主机（APP）
- `0x01-0x14`: 电机节点（1-20）
- `0x15-0x28`: 传感器节点（21-40）
- `0x29-0x3C`: IO设备节点（41-60）
- `0x3D-0x3E`: 预留（61-62）
- `0x3F`: 广播地址

### 2.2 设备ID分配规则

#### 设备类型与ID范围映射
```
设备类型    ID范围          数量    说明
----------------------------------------------
主机        0x00            1       APP主控制器
电机        0x01-0x14       20      电机节点（Motor 1-20）
传感器      0x15-0x28       20      传感器节点（Sensor 1-20）
IO设备      0x29-0x3C       20      IO节点（IO 1-20）
预留        0x3D-0x3E       2       未来扩展
广播        0x3F            1       广播地址
```

#### ID编号规则
- **电机编号**: ID = 0x01 + (电机序号 - 1)
  - 电机1 → ID=0x01, 电机2 → ID=0x02, ..., 电机20 → ID=0x14

- **传感器编号**: ID = 0x15 + (传感器序号 - 1)  
  - 传感器1 → ID=0x15, 传感器2 → ID=0x16, ..., 传感器20 → ID=0x28

- **IO设备编号**: ID = 0x29 + (IO序号 - 1)
  - IO1 → ID=0x29, IO2 → ID=0x2A, ..., IO20 → ID=0x3C

### 2.3 CAN ID计算公式
```c
CAN_ID = (功能码 << 6) | 节点ID

例如：
- 主机发送心跳广播: (0x01 << 6) | 0x3F = 0x07F
- 电机1的心跳响应: (0x01 << 6) | 0x01 = 0x041
- 主机控制电机5: (0x04 << 6) | 0x05 = 0x105
- 传感器1数据上报: (0x11 << 6) | 0x15 = 0x455
- IO1状态上报: (0x15 << 6) | 0x29 = 0x569
```

### 2.4 设备类型识别
```c
// 根据节点ID判断设备类型
DeviceType_t GetDeviceTypeFromID(uint8_t node_id)
{
    if (node_id >= 0x01 && node_id <= 0x14)
        return DEV_TYPE_MOTOR;      // 电机
    else if (node_id >= 0x15 && node_id <= 0x28)
        return DEV_TYPE_SENSOR;     // 传感器
    else if (node_id >= 0x29 && node_id <= 0x3C)
        return DEV_TYPE_IO;         // IO设备
    else
        return DEV_TYPE_UNKNOWN;
}
```

## 3. 消息定义

### 3.1 系统管理类 (功能码 0x00)

#### 3.1.1 主机广播发现请求
```
ID: 0x03F (功能码0x00, 广播0x3F)
DLC: 2
Data: [0x01] [0x00]
      CMD    保留
```

#### 3.1.2 从机响应设备信息
```
ID: 0x000 + 节点ID (功能码0x00, 节点自己的ID)
DLC: 8
Data: [DevType] [HW_Ver] [SW_Ver_H] [SW_Ver_L] [Status] [Capability] [保留] [保留]
      设备类型  硬件版本  软件版本高   软件版本低   状态    能力位

DevType设备类型:
  0x01 = 电机
  0x02 = 传感器
  0x03 = IO设备
  0x04 = 混合设备
  
Status状态:
  Bit0: 0=离线, 1=在线
  Bit1: 0=正常, 1=故障
  Bit2-7: 保留
  
Capability能力位:
  电机设备:
    Bit0: 支持速度控制
    Bit1: 支持位置控制
    Bit2: 支持扭矩控制
    Bit3: 支持编码器反馈
```

### 3.2 心跳类 (功能码 0x01)

#### 3.2.1 主机心跳广播
```
ID: 0x07F (功能码0x01, 广播0x3F)
DLC: 4
Data: [0xAA] [SEQ_H] [SEQ_L] [0x55]
      魔数    序列号高  序列号低  魔数
      
说明：主机每500ms广播一次心跳
```

#### 3.2.2 从机心跳响应
```
ID: 0x040 + 节点ID (功能码0x01, 节点自己的ID)
DLC: 4
Data: [Status] [Error] [Uptime_H] [Uptime_L]
      状态码    错误码   运行时间高  运行时间低
      
Status状态码:
  0x00 = 正常运行
  0x01 = 初始化中
  0x02 = 警告状态
  0x03 = 错误状态
  0xFF = 严重故障
  
Error错误码:
  0x00 = 无错误
  0x01 = 通信超时
  0x02 = 硬件故障
  0x03 = 参数错误
  0x04 = 过载保护
  0x05 = 温度过高
```

### 3.3 电机控制类 (功能码 0x04-0x0F)

#### 3.3.1 电机控制命令 (0x04)
```
ID: 0x100 + 节点ID (功能码0x04, 目标节点ID)
DLC: 8
Data: [CMD] [Speed_H] [Speed_L] [Direction] [Accel] [Decel] [保留] [Checksum]
      命令  目标转速高 目标转速低  方向      加速度   减速度          校验和

CMD命令:
  0x00 = 停止
  0x01 = 启动
  0x02 = 急停
  0x03 = 设置速度
  0x04 = 设置方向
  0x05 = 速度+方向同时设置
  
Speed: 0-800 (RPM)
  
Direction方向:
  0x00 = 停止
  0x01 = 正转 (CW)
  0x02 = 反转 (CCW)
  
Accel/Decel: 加速度/减速度 (RPM/s, 0=使用默认值)

Checksum: 简单校验和 = (CMD + Speed_H + Speed_L + Direction + Accel + Decel) & 0xFF
```

#### 3.3.2 电机控制响应 (0x05)
```
ID: 0x140 + 节点ID (功能码0x05, 响应节点ID)
DLC: 2
Data: [Result] [Error]
      结果码   错误码
      
Result结果码:
  0x00 = 成功
  0x01 = 拒绝（正在执行其他命令）
  0x02 = 参数错误
  0x03 = 硬件故障
  0xFF = 未知错误
```

#### 3.3.3 电机状态查询 (0x06)
```
主机发送:
ID: 0x180 + 节点ID (功能码0x06, 目标节点ID)
DLC: 1
Data: [QueryType]
      查询类型
      
QueryType:
  0x00 = 查询所有状态
  0x01 = 仅查询转速
  0x02 = 仅查询方向
  0x03 = 仅查询故障状态
```

#### 3.3.4 电机状态上报 (0x07)
```
从机响应:
ID: 0x1C0 + 节点ID (功能码0x07, 响应节点ID)
DLC: 8
Data: [Status] [Speed_H] [Speed_L] [Direction] [Current_H] [Current_L] [Temp] [Fault]
      状态    实际转速高 实际转速低  当前方向   电流高      电流低     温度   故障码

Status状态:
  Bit0: 0=停止, 1=运行
  Bit1: 0=正常, 1=故障
  Bit2: 0=空载, 1=负载
  Bit3: 0=正转, 1=反转
  Bit4-7: 保留
  
Speed: 实际转速 (RPM)
Direction: 当前方向 (0x00=停止, 0x01=正转, 0x02=反转)
Current: 电流 (mA)
Temp: 温度 (°C, -40~215, offset=40)
Fault: 故障位图
  Bit0: 过流
  Bit1: 过压
  Bit2: 欠压
  Bit3: 过温
  Bit4: 堵转
  Bit5: 编码器故障
  Bit6-7: 保留
```

#### 3.3.5 电机参数设置 (0x08)
```
ID: 0x200 + 节点ID (功能码0x08, 目标节点ID)
DLC: 8
Data: [ParamID] [Value0] [Value1] [Value2] [Value3] [Save] [保留] [Checksum]
      参数ID    值字节0  值字节1  值字节2  值字节3  保存标志        校验和

ParamID参数ID:
  0x01 = 最大转速 (RPM, uint32)
  0x02 = 加速时间 (ms, uint32)
  0x03 = 减速时间 (ms, uint32)
  0x04 = PID-Kp (float32)
  0x05 = PID-Ki (float32)
  0x06 = PID-Kd (float32)
  0x10 = 电流限制 (mA, uint32)
  0x11 = 温度限制 (°C, uint8)
  
Save保存标志:
  0x00 = 仅RAM
  0x01 = 保存到Flash
```

### 3.4 传感器类 (功能码 0x10-0x13)

#### 3.4.1 传感器数据查询 (0x10)
```
ID: 0x400 + 节点ID (功能码0x10, 目标节点ID)
DLC: 2
Data: [SensorType] [Reserved]
      传感器类型   保留
      
SensorType:
  0x01 = 温度传感器
  0x02 = 湿度传感器
  0x03 = 压力传感器
  0x04 = 距离传感器
  0xFF = 所有传感器
```

#### 3.4.2 传感器数据上报 (0x11)
```
ID: 0x440 + 节点ID (功能码0x11, 响应节点ID)
DLC: 8
Data: [SensorType] [Status] [Data0] [Data1] [Data2] [Data3] [Unit] [Quality]
      传感器类型   状态    数据字节0 数据字节1 数据字节2 数据字节3 单位  质量

Status:
  0x00 = 正常
  0x01 = 校准中
  0x02 = 超出范围
  0x03 = 传感器故障
  
Quality: 数据质量 (0-100%)
```

### 3.5 IO设备类 (功能码 0x14-0x17)

#### 3.5.1 IO控制命令 (0x14)
```
ID: 0x500 + 节点ID (功能码0x14, 目标节点ID)
DLC: 4
Data: [Port] [Mask] [Value] [Reserved]
      端口号  掩码    值      保留
      
Port: 端口号 (0-7)
Mask: 位掩码 (0xFF = 所有位)
Value: 输出值
```

#### 3.5.2 IO状态上报 (0x15)
```
ID: 0x540 + 节点ID (功能码0x15, 响应节点ID)
DLC: 8
Data: [Port0] [Port1] [Port2] [Port3] [Port4] [Port5] [Port6] [Port7]
      端口0状态 端口1状态 ... 端口7状态
```

### 3.6 广播命令 (功能码 0x1F)

#### 3.6.1 紧急停止广播
```
ID: 0x7FF (功能码0x1F, 广播0x3F)
DLC: 2
Data: [0xE5] [0xE5]
      紧急停止魔数
      
说明：所有设备收到后立即停止运动
```

## 4. 通信流程

### 4.1 系统启动流程
```
时间  主机APP                    从机节点
------|---------------------------|---------------------------
0ms   上电，初始化CAN
100ms 
200ms 发送设备发现广播 -------->
300ms                            收到发现请求
400ms <-------- 响应设备信息     发送设备信息
500ms 记录节点1信息
600ms <-------- 响应设备信息     节点2发送信息
700ms 记录节点2信息
...   
1000ms 启动心跳定时器
1500ms 发送心跳广播 ----------->
1600ms <-------- 心跳响应        所有节点发送心跳
```

### 4.2 电机控制流程
```
主机                                电机节点
1. 发送控制命令 (0x104) --------> 
                                    2. 接收命令，解析
                                    3. 执行命令
                <-------- 4. 发送响应 (0x144)
5. 收到成功响应
6. 发送状态查询 (0x184) -------->
                                    7. 读取实际状态
                <-------- 8. 上报状态 (0x1C4)
9. 显示实际转速/方向
```

### 4.3 心跳超时处理
```
主机维护节点表:
struct {
    uint8_t node_id;
    uint8_t device_type;
    uint8_t status;
    uint32_t last_heartbeat_time;
} node_table[MAX_NODES];

心跳检测逻辑:
每100ms检查一次:
  for each node:
    if (current_time - last_heartbeat_time > 3000ms):
      标记节点为离线
      触发报警
      if (device_type == MOTOR):
        发送紧急停止命令
```

## 5. 数据结构定义

### 5.1 C语言结构体定义

```c
/* CAN ID定义 */
#define CAN_FUNC_SYSTEM         0x00
#define CAN_FUNC_HEARTBEAT      0x01
#define CAN_FUNC_DISCOVERY      0x02
#define CAN_FUNC_INFO           0x03
#define CAN_FUNC_MOTOR_CMD      0x04
#define CAN_FUNC_MOTOR_RESP     0x05
#define CAN_FUNC_MOTOR_QUERY    0x06
#define CAN_FUNC_MOTOR_STATUS   0x07
#define CAN_FUNC_MOTOR_PARAM    0x08
#define CAN_FUNC_SENSOR_QUERY   0x10
#define CAN_FUNC_SENSOR_DATA    0x11
#define CAN_FUNC_IO_CMD         0x14
#define CAN_FUNC_IO_STATUS      0x15
#define CAN_FUNC_BROADCAST      0x1F

#define CAN_ADDR_MASTER         0x00
#define CAN_ADDR_BROADCAST      0x3F

#define CAN_ID(func, addr)      (((func) << 6) | (addr))

/* 设备类型 */
typedef enum {
    DEV_TYPE_UNKNOWN    = 0x00,
    DEV_TYPE_MOTOR      = 0x01,
    DEV_TYPE_SENSOR     = 0x02,
    DEV_TYPE_IO         = 0x03,
    DEV_TYPE_HYBRID     = 0x04
} DeviceType_t;

/* 节点状态 */
typedef enum {
    NODE_STATUS_OFFLINE = 0,
    NODE_STATUS_ONLINE  = 1,
    NODE_STATUS_ERROR   = 2,
    NODE_STATUS_WARNING = 3
} NodeStatus_t;

/* 电机命令 */
typedef enum {
    MOTOR_CMD_STOP          = 0x00,
    MOTOR_CMD_START         = 0x01,
    MOTOR_CMD_EMERGENCY_STOP = 0x02,
    MOTOR_CMD_SET_SPEED     = 0x03,
    MOTOR_CMD_SET_DIRECTION = 0x04,
    MOTOR_CMD_SET_BOTH      = 0x05
} MotorCommand_t;

/* 电机方向 */
typedef enum {
    MOTOR_DIR_STOP  = 0x00,
    MOTOR_DIR_CW    = 0x01,  // 顺时针
    MOTOR_DIR_CCW   = 0x02   // 逆时针
} MotorDirection_t;

/* 节点信息 */
typedef struct {
    uint8_t node_id;
    DeviceType_t device_type;
    uint8_t hw_version;
    uint16_t sw_version;
    NodeStatus_t status;
    uint8_t capability;
    uint32_t last_heartbeat_time;
    uint32_t uptime;
    uint8_t error_code;
} NodeInfo_t;

/* 电机状态 */
typedef struct {
    uint8_t status;           // 状态位
    uint16_t actual_speed;    // 实际转速 (RPM)
    MotorDirection_t direction; // 当前方向
    uint16_t current;         // 电流 (mA)
    uint8_t temperature;      // 温度 (°C)
    uint8_t fault;            // 故障位图
} MotorStatus_t;

/* 电机控制参数 */
typedef struct {
    MotorCommand_t command;
    uint16_t target_speed;    // RPM
    MotorDirection_t direction;
    uint8_t accel;            // 加速度
    uint8_t decel;            // 减速度
} MotorControl_t;
```

## 6. 协议特性

### 6.1 优势
- ✅ **自动发现**：主机可自动检测所有在线设备
- ✅ **心跳机制**：及时检测设备离线
- ✅ **可扩展**：最多支持62个从机节点
- ✅ **多设备类型**：电机、传感器、IO统一管理
- ✅ **实时性好**：CAN总线延迟低，适合实时控制
- ✅ **容错性**：校验和、超时检测
- ✅ **分层设计**：功能码+节点ID清晰分层

### 6.2 性能指标
- **波特率**: 500kbps
- **最大节点数**: 62个从机 + 1个主机
- **心跳周期**: 500ms
- **超时判断**: 3秒无心跳标记离线
- **控制响应时间**: < 10ms
- **状态更新周期**: 100ms (可配置)

### 6.3 安全机制
- **校验和**: 关键命令带校验和
- **序列号**: 心跳带序列号防重放
- **超时保护**: 通信超时自动停机
- **紧急停止**: 广播紧急停止所有设备
- **故障上报**: 实时上报设备故障

## 7. 实现建议

### 7.1 主机实现要点
```c
// 节点表维护
NodeInfo_t node_table[62];
uint8_t node_count = 0;

// 定期任务
void CAN_Protocol_Task_500ms(void) {
    // 发送心跳广播
    CAN_SendHeartbeat();
}

void CAN_Protocol_Task_100ms(void) {
    // 检查节点超时
    CAN_CheckNodeTimeout();
}

// 接收处理
void CAN_RxCallback(uint32_t id, uint8_t* data, uint8_t len) {
    uint8_t func = (id >> 6) & 0x1F;
    uint8_t addr = id & 0x3F;
    
    switch(func) {
        case CAN_FUNC_SYSTEM:
            HandleDeviceInfo(addr, data);
            break;
        case CAN_FUNC_HEARTBEAT:
            UpdateNodeHeartbeat(addr, data);
            break;
        case CAN_FUNC_MOTOR_STATUS:
            UpdateMotorStatus(addr, data);
            break;
        // ...
    }
}
```

### 7.2 从机实现要点
```c
// 节点ID配置 (DIP开关或Flash存储)
uint8_t my_node_id = 1;
DeviceType_t my_device_type = DEV_TYPE_MOTOR;

// 心跳响应
void CAN_HeartbeatHandler(void) {
    uint8_t data[4];
    data[0] = device_status;
    data[1] = error_code;
    data[2] = (uptime >> 8) & 0xFF;
    data[3] = uptime & 0xFF;
    
    CAN_Send(CAN_ID(CAN_FUNC_HEARTBEAT, my_node_id), data, 4);
}

// 命令处理
void CAN_MotorCommandHandler(uint8_t* data) {
    MotorCommand_t cmd = data[0];
    uint16_t speed = (data[1] << 8) | data[2];
    MotorDirection_t dir = data[3];
    
    // 执行命令
    Motor_SetSpeed(speed);
    Motor_SetDirection(dir);
    
    // 发送响应
    uint8_t resp[2] = {0x00, 0x00}; // 成功
    CAN_Send(CAN_ID(CAN_FUNC_MOTOR_RESP, my_node_id), resp, 2);
}
```

## 8. 测试验证

### 8.1 测试用例
1. **设备发现测试**: 上电后主机发现所有从机
2. **心跳测试**: 断电一个节点，主机3秒内检测到
3. **电机控制测试**: 设置转速1000RPM，实际转速误差<5%
4. **紧急停止测试**: 发送紧急停止，所有电机立即停止
5. **多节点并发测试**: 同时控制10个电机节点
6. **通信异常测试**: 模拟总线干扰，协议容错性

### 8.2 调试工具
- **CAN分析仪**: 查看总线消息
- **串口日志**: 主机打印节点状态
- **LED指示**: 从机LED显示状态（运行/故障/通信）

## 9. 扩展性

### 9.1 未来扩展
- 支持CAN FD（更高速率）
- 增加固件升级功能
- 支持参数批量配置
- 增加数据记录功能（黑匣子）
- 支持无线网关（CAN转WiFi/蓝牙）

### 9.2 兼容性
- 向下兼容：新版本主机可识别旧版本从机
- 协议版本号：设备信息中包含协议版本

---

**文档版本**: v1.0
**创建日期**: 2025-10-14
**适用项目**: JLLoader CAN总线多节点管理系统
