/**
  ******************************************************************************
  * @file    can_protocol.h
  * @brief   CAN多节点管理协议头文件
  ******************************************************************************
  */

#ifndef __CAN_PROTOCOL_H
#define __CAN_PROTOCOL_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include <stdint.h>
#include <stdbool.h>

/* CAN协议配置 ----------------------------------------------------------------*/
#define CAN_PROTOCOL_VERSION        0x0100      // v1.0
#define CAN_MAX_NODES               62          // 最大从机节点数
#define CAN_MAX_MOTORS              20          // 最大电机节点数
#define CAN_MAX_SENSORS             20          // 最大传感器节点数
#define CAN_MAX_IOS                 20          // 最大IO节点数
#define CAN_HEARTBEAT_INTERVAL_MS   500         // 心跳间隔
#define CAN_HEARTBEAT_TIMEOUT_MS    3000        // 心跳超时
#define CAN_RESPONSE_TIMEOUT_MS     100         // 命令响应超时

/* CAN功能码定义 -------------------------------------------------------------*/
#define CAN_FUNC_SYSTEM             0x00        // 系统管理
#define CAN_FUNC_HEARTBEAT          0x01        // 心跳
#define CAN_FUNC_DISCOVERY          0x02        // 设备发现
#define CAN_FUNC_INFO               0x03        // 设备信息查询
#define CAN_FUNC_MOTOR_CMD          0x04        // 电机控制命令
#define CAN_FUNC_MOTOR_RESP         0x05        // 电机控制响应
#define CAN_FUNC_MOTOR_QUERY        0x06        // 电机状态查询
#define CAN_FUNC_MOTOR_STATUS       0x07        // 电机状态上报
#define CAN_FUNC_MOTOR_PARAM        0x08        // 电机参数设置
#define CAN_FUNC_SENSOR_QUERY       0x10        // 传感器数据查询
#define CAN_FUNC_SENSOR_DATA        0x11        // 传感器数据上报
#define CAN_FUNC_IO_CMD             0x14        // IO控制命令
#define CAN_FUNC_IO_STATUS          0x15        // IO状态上报
#define CAN_FUNC_BROADCAST          0x1F        // 广播命令

/* CAN地址定义 ---------------------------------------------------------------*/
#define CAN_ADDR_MASTER             0x00        // 主机地址
#define CAN_ADDR_MOTOR_START        0x01        // 电机起始地址
#define CAN_ADDR_MOTOR_END          0x14        // 电机结束地址 (1-20)
#define CAN_ADDR_SENSOR_START       0x15        // 传感器起始地址
#define CAN_ADDR_SENSOR_END         0x28        // 传感器结束地址 (21-40)
#define CAN_ADDR_IO_START           0x29        // IO设备起始地址
#define CAN_ADDR_IO_END             0x3C        // IO设备结束地址 (41-60)
#define CAN_ADDR_BROADCAST          0x3F        // 广播地址

/* 设备ID编号宏 ---------------------------------------------------------------*/
#define MOTOR_ID(n)                 (CAN_ADDR_MOTOR_START + (n) - 1)    // 电机编号转ID
#define SENSOR_ID(n)                (CAN_ADDR_SENSOR_START + (n) - 1)   // 传感器编号转ID
#define IO_ID(n)                    (CAN_ADDR_IO_START + (n) - 1)       // IO编号转ID

/* 设备类型判断宏 -------------------------------------------------------------*/
#define IS_MOTOR_ID(id)             ((id) >= CAN_ADDR_MOTOR_START && (id) <= CAN_ADDR_MOTOR_END)
#define IS_SENSOR_ID(id)            ((id) >= CAN_ADDR_SENSOR_START && (id) <= CAN_ADDR_SENSOR_END)
#define IS_IO_ID(id)                ((id) >= CAN_ADDR_IO_START && (id) <= CAN_ADDR_IO_END)
#define IS_VALID_NODE_ID(id)        ((id) >= CAN_ADDR_MOTOR_START && (id) <= CAN_ADDR_IO_END)

/* CAN ID宏定义 --------------------------------------------------------------*/
#define CAN_ID(func, addr)          (((func) << 6) | (addr))
#define CAN_GET_FUNC(id)            (((id) >> 6) & 0x1F)
#define CAN_GET_ADDR(id)            ((id) & 0x3F)

/* 设备类型定义 --------------------------------------------------------------*/
typedef enum {
    DEV_TYPE_UNKNOWN    = 0x00,
    DEV_TYPE_MOTOR      = 0x01,
    DEV_TYPE_SENSOR     = 0x02,
    DEV_TYPE_IO         = 0x03,
    DEV_TYPE_HYBRID     = 0x04
} DeviceType_t;

/* 节点状态定义 --------------------------------------------------------------*/
typedef enum {
    NODE_STATUS_OFFLINE = 0,
    NODE_STATUS_ONLINE  = 1,
    NODE_STATUS_ERROR   = 2,
    NODE_STATUS_WARNING = 3
} NodeStatus_t;

/* 电机命令定义 --------------------------------------------------------------*/
typedef enum {
    MOTOR_CMD_STOP              = 0x00,
    MOTOR_CMD_START             = 0x01,
    MOTOR_CMD_EMERGENCY_STOP    = 0x02,
    MOTOR_CMD_SET_SPEED         = 0x03,
    MOTOR_CMD_SET_DIRECTION     = 0x04,
    MOTOR_CMD_SET_BOTH          = 0x05
} MotorCommand_t;

/* 电机方向定义 --------------------------------------------------------------*/
typedef enum {
    MOTOR_DIR_STOP  = 0x00,
    MOTOR_DIR_CW    = 0x01,     // 顺时针 (Clockwise)
    MOTOR_DIR_CCW   = 0x02      // 逆时针 (Counter-Clockwise)
} MotorDirection_t;

/* 电机状态位定义 ------------------------------------------------------------*/
#define MOTOR_STATUS_RUNNING        (1 << 0)    // 运行中
#define MOTOR_STATUS_FAULT          (1 << 1)    // 故障
#define MOTOR_STATUS_LOADED         (1 << 2)    // 有负载
#define MOTOR_STATUS_REVERSE        (1 << 3)    // 反转

/* 电机故障位定义 ------------------------------------------------------------*/
#define MOTOR_FAULT_OVERCURRENT     (1 << 0)    // 过流
#define MOTOR_FAULT_OVERVOLTAGE     (1 << 1)    // 过压
#define MOTOR_FAULT_UNDERVOLTAGE    (1 << 2)    // 欠压
#define MOTOR_FAULT_OVERTEMP        (1 << 3)    // 过温
#define MOTOR_FAULT_STALL           (1 << 4)    // 堵转
#define MOTOR_FAULT_ENCODER         (1 << 5)    // 编码器故障

/* 数据结构定义 --------------------------------------------------------------*/

/**
 * @brief 节点信息结构体
 */
typedef struct {
    uint8_t node_id;                    // 节点ID (1-62)
    DeviceType_t device_type;           // 设备类型
    uint8_t hw_version;                 // 硬件版本
    uint16_t sw_version;                // 软件版本
    NodeStatus_t status;                // 节点状态
    uint8_t capability;                 // 能力位
    uint32_t last_heartbeat_time;       // 最后心跳时间 (ms)
    uint32_t uptime;                    // 运行时间 (s)
    uint8_t error_code;                 // 错误码
    bool discovered;                    // 是否已发现
} NodeInfo_t;

/**
 * @brief 电机状态结构体
 */
typedef struct {
    uint8_t status;                     // 状态位
    uint16_t actual_speed;              // 实际转速 (RPM)
    MotorDirection_t direction;         // 当前方向
    uint16_t current;                   // 电流 (mA)
    uint8_t temperature;                // 温度 (°C)
    uint8_t fault;                      // 故障位图
} MotorStatus_t;

/**
 * @brief 电机控制参数结构体
 */
typedef struct {
    MotorCommand_t command;             // 命令
    uint16_t target_speed;              // 目标转速 (RPM)
    MotorDirection_t direction;         // 方向
    uint8_t accel;                      // 加速度 (RPM/s)
    uint8_t decel;                      // 减速度 (RPM/s)
} MotorControl_t;

/**
 * @brief 传感器数据结构体
 */
typedef struct {
    uint8_t sensor_type;                // 传感器类型
    uint8_t status;                     // 状态
    uint32_t data;                      // 数据
    uint8_t unit;                       // 单位
    uint8_t quality;                    // 质量 (0-100%)
} SensorData_t;

/* 主机端API ----------------------------------------------------------------*/
#ifdef CAN_PROTOCOL_MASTER

/**
 * @brief  初始化CAN协议（主机端）
 * @retval None
 */
void CANProtocol_Master_Init(void);

/**
 * @brief  发送设备发现广播
 * @retval None
 */
void CANProtocol_SendDiscovery(void);

/**
 * @brief  发送心跳广播
 * @retval None
 */
void CANProtocol_SendHeartbeat(void);

/**
 * @brief  发送电机控制命令
 * @param  node_id: 节点ID
 * @param  control: 控制参数
 * @retval true=成功, false=失败
 */
bool CANProtocol_MotorControl(uint8_t node_id, MotorControl_t* control);

/**
 * @brief  查询电机状态
 * @param  node_id: 节点ID
 * @param  status: 状态结构体指针
 * @retval true=成功, false=失败
 */
bool CANProtocol_MotorQueryStatus(uint8_t node_id, MotorStatus_t* status);

/**
 * @brief  获取节点信息
 * @param  node_id: 节点ID
 * @retval NodeInfo_t* 节点信息指针，NULL=不存在
 */
NodeInfo_t* CANProtocol_GetNodeInfo(uint8_t node_id);

/**
 * @brief  获取在线节点数量
 * @retval uint8_t 在线节点数
 */
uint8_t CANProtocol_GetOnlineNodeCount(void);

/**
 * @brief  检查节点超时（在定时任务中调用）
 * @retval None
 */
void CANProtocol_CheckNodeTimeout(void);

/**
 * @brief  CAN接收处理（在CAN中断回调中调用）
 * @param  id: CAN消息ID
 * @param  data: 数据指针
 * @param  len: 数据长度
 * @retval None
 */
void CANProtocol_Master_RxHandler(uint32_t id, uint8_t* data, uint8_t len);

/**
 * @brief  发送紧急停止广播
 * @retval None
 */
void CANProtocol_EmergencyStop(void);

#endif // CAN_PROTOCOL_MASTER

/* 从机端API ----------------------------------------------------------------*/
#ifdef CAN_PROTOCOL_SLAVE

/**
 * @brief  初始化CAN协议（从机端）
 * @param  node_id: 本机节点ID
 * @param  device_type: 设备类型
 * @retval None
 */
void CANProtocol_Slave_Init(uint8_t node_id, DeviceType_t device_type);

/**
 * @brief  发送心跳响应
 * @param  status: 状态码
 * @param  error: 错误码
 * @retval None
 */
void CANProtocol_SendHeartbeatResponse(uint8_t status, uint8_t error);

/**
 * @brief  发送设备信息响应
 * @retval None
 */
void CANProtocol_SendDeviceInfo(void);

/**
 * @brief  发送电机状态
 * @param  status: 电机状态结构体指针
 * @retval None
 */
void CANProtocol_SendMotorStatus(MotorStatus_t* status);

/**
 * @brief  发送命令响应
 * @param  func: 功能码
 * @param  result: 结果码
 * @param  error: 错误码
 * @retval None
 */
void CANProtocol_SendResponse(uint8_t func, uint8_t result, uint8_t error);

/**
 * @brief  CAN接收处理（在CAN中断回调中调用）
 * @param  id: CAN消息ID
 * @param  data: 数据指针
 * @param  len: 数据长度
 * @retval None
 */
void CANProtocol_Slave_RxHandler(uint32_t id, uint8_t* data, uint8_t len);

/**
 * @brief  更新运行时间（1秒调用一次）
 * @retval None
 */
void CANProtocol_UpdateUptime(void);

#endif // CAN_PROTOCOL_SLAVE

/* 通用工具函数 --------------------------------------------------------------*/

/**
 * @brief  计算校验和
 * @param  data: 数据指针
 * @param  len: 数据长度
 * @retval uint8_t 校验和
 */
uint8_t CANProtocol_Checksum(uint8_t* data, uint8_t len);

/**
 * @brief  验证校验和
 * @param  data: 数据指针
 * @param  len: 数据长度（包含校验和）
 * @retval true=正确, false=错误
 */
bool CANProtocol_VerifyChecksum(uint8_t* data, uint8_t len);

/**
 * @brief  根据节点ID获取设备类型
 * @param  node_id: 节点ID
 * @retval DeviceType_t 设备类型
 */
DeviceType_t CANProtocol_GetDeviceTypeFromID(uint8_t node_id);

/**
 * @brief  获取电机的节点ID
 * @param  motor_num: 电机编号 (1-20)
 * @retval uint8_t 节点ID
 */
uint8_t CANProtocol_GetMotorID(uint8_t motor_num);

/**
 * @brief  获取传感器的节点ID
 * @param  sensor_num: 传感器编号 (1-20)
 * @retval uint8_t 节点ID
 */
uint8_t CANProtocol_GetSensorID(uint8_t sensor_num);

/**
 * @brief  获取IO设备的节点ID
 * @param  io_num: IO编号 (1-20)
 * @retval uint8_t 节点ID
 */
uint8_t CANProtocol_GetIOID(uint8_t io_num);

#ifdef __cplusplus
}
#endif

#endif /* __CAN_PROTOCOL_H */
