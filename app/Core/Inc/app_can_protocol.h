/**
  ******************************************************************************
  * @file    app_can_protocol.h
  * @author  Generated
  * @brief   Host-side CAN protocol implementation with device management
  *          支持设备自动发现、心跳维护、多设备类型控制
  ******************************************************************************
  */

#ifndef __APP_CAN_PROTOCOL_H
#define __APP_CAN_PROTOCOL_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include <stdint.h>
#include <stdbool.h>

/* Exported constants --------------------------------------------------------*/

/* CAN协议功能码定义 (5位: 0-31) */
#define CAN_FUNC_SYSTEM         0x00    // 系统管理
#define CAN_FUNC_HEARTBEAT      0x01    // 心跳
#define CAN_FUNC_DISCOVERY      0x02    // 设备发现
#define CAN_FUNC_INFO           0x03    // 设备信息查询
#define CAN_FUNC_MOTOR_CMD      0x04    // 电机控制命令
#define CAN_FUNC_MOTOR_RESP     0x05    // 电机控制响应
#define CAN_FUNC_MOTOR_QUERY    0x06    // 电机状态查询
#define CAN_FUNC_MOTOR_STATUS   0x07    // 电机状态上报
#define CAN_FUNC_MOTOR_PARAM    0x08    // 电机参数设置
#define CAN_FUNC_SENSOR_QUERY   0x10    // 传感器数据查询
#define CAN_FUNC_SENSOR_DATA    0x11    // 传感器数据上报
#define CAN_FUNC_IO_CMD         0x14    // IO控制命令
#define CAN_FUNC_IO_STATUS      0x15    // IO状态上报
#define CAN_FUNC_BROADCAST      0x1F    // 广播命令

/* CAN节点地址定义 (6位: 0-63) */
#define CAN_ADDR_MASTER         0x00    // 主机地址
#define CAN_ADDR_MOTOR_BASE     0x01    // 电机基地址 (0x01-0x14, 共20个)
#define CAN_ADDR_SENSOR_BASE    0x15    // 传感器基地址 (0x15-0x28, 共20个)
#define CAN_ADDR_IO_BASE        0x29    // IO设备基地址 (0x29-0x3C, 共20个)
#define CAN_ADDR_BROADCAST      0x3F    // 广播地址

/* 设备数量限制 */
#define MAX_NODES               62      // 最大节点数 (不含主机)
#define MAX_MOTORS              20      // 最大电机数量
#define MAX_SENSORS             20      // 最大传感器数量
#define MAX_IO_DEVICES          20      // 最大IO设备数量

/* 电机转速约束 */
#define MOTOR_MAX_SPEED         800     // 电机最大转速 800 RPM
#define MOTOR_MIN_SPEED         0       // 电机最小转速 0 RPM

/* 超时时间定义 (ms) */
#define HEARTBEAT_INTERVAL      500     // 心跳发送间隔
#define HEARTBEAT_TIMEOUT       3000    // 心跳超时判断
#define DISCOVERY_DELAY         1000    // 启动后延迟发现时间
#define DISCOVERY_TIMEOUT       2000    // 发现响应超时

/* CAN ID计算宏 */
#define CAN_ID(func, addr)      (((func) << 6) | (addr))
#define CAN_GET_FUNC(id)        (((id) >> 6) & 0x1F)
#define CAN_GET_ADDR(id)        ((id) & 0x3F)

/* Exported types ------------------------------------------------------------*/

/**
 * @brief 设备类型枚举
 */
typedef enum {
    DEV_TYPE_UNKNOWN    = 0x00,
    DEV_TYPE_MOTOR      = 0x01,
    DEV_TYPE_SENSOR     = 0x02,
    DEV_TYPE_IO         = 0x03,
    DEV_TYPE_HYBRID     = 0x04
} DeviceType_t;

/**
 * @brief 节点状态枚举
 */
typedef enum {
    NODE_STATUS_OFFLINE = 0,    // 离线
    NODE_STATUS_ONLINE  = 1,    // 在线
    NODE_STATUS_ERROR   = 2,    // 错误
    NODE_STATUS_WARNING = 3     // 警告
} NodeStatus_t;

/**
 * @brief 电机命令枚举
 */
typedef enum {
    MOTOR_CMD_STOP          = 0x00,     // 停止
    MOTOR_CMD_START         = 0x01,     // 启动
    MOTOR_CMD_EMERGENCY_STOP = 0x02,    // 急停
    MOTOR_CMD_SET_SPEED     = 0x03,     // 设置速度
    MOTOR_CMD_SET_DIRECTION = 0x04,     // 设置方向
    MOTOR_CMD_SET_BOTH      = 0x05      // 同时设置速度和方向
} MotorCommand_t;

/**
 * @brief 电机方向枚举
 */
typedef enum {
    MOTOR_DIR_STOP  = 0x00,     // 停止
    MOTOR_DIR_CW    = 0x01,     // 顺时针
    MOTOR_DIR_CCW   = 0x02      // 逆时针
} MotorDirection_t;

/**
 * @brief 传感器类型枚举
 */
typedef enum {
    SENSOR_TYPE_TEMPERATURE = 0x01,     // 温度传感器
    SENSOR_TYPE_HUMIDITY    = 0x02,     // 湿度传感器
    SENSOR_TYPE_PRESSURE    = 0x03,     // 压力传感器
    SENSOR_TYPE_DISTANCE    = 0x04,     // 距离传感器
    SENSOR_TYPE_ALL         = 0xFF      // 所有传感器
} SensorType_t;

/**
 * @brief 节点信息结构体
 */
typedef struct {
    uint8_t node_id;                    // 节点ID
    DeviceType_t device_type;           // 设备类型
    uint8_t hw_version;                 // 硬件版本
    uint16_t sw_version;                // 软件版本
    NodeStatus_t status;                // 节点状态
    uint8_t capability;                 // 能力位
    uint32_t last_heartbeat_time;       // 最后心跳时间
    uint32_t uptime;                    // 运行时间(秒)
    uint8_t error_code;                 // 错误码
    bool is_valid;                      // 是否有效
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
    uint32_t last_update_time;          // 最后更新时间
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
    SensorType_t sensor_type;           // 传感器类型
    uint8_t status;                     // 状态
    float value;                        // 数据值 (根据类型解释)
    uint8_t unit;                       // 单位
    uint8_t quality;                    // 数据质量 (0-100%)
    uint32_t last_update_time;          // 最后更新时间
} SensorData_t;

/**
 * @brief IO状态结构体
 */
typedef struct {
    uint8_t port_status[8];             // 8个端口的状态
    uint32_t last_update_time;          // 最后更新时间
} IOStatus_t;

/**
 * @brief 设备管理器结构体
 */
typedef struct {
    NodeInfo_t nodes[MAX_NODES];        // 节点信息表
    uint8_t node_count;                 // 在线节点数量
    
    MotorStatus_t motors[MAX_MOTORS];   // 电机状态表
    SensorData_t sensors[MAX_SENSORS];  // 传感器数据表
    IOStatus_t io_devices[MAX_IO_DEVICES]; // IO设备状态表
    
    uint16_t heartbeat_seq;             // 心跳序列号
    uint32_t last_heartbeat_time;       // 最后心跳时间
    uint32_t last_check_time;           // 最后检查时间
    
    bool discovery_done;                // 发现完成标志
    uint32_t discovery_start_time;      // 发现开始时间
} DeviceManager_t;

/* Exported variables --------------------------------------------------------*/
extern DeviceManager_t g_device_manager;
extern CAN_HandleTypeDef hcan1;

/* Exported function prototypes ---------------------------------------------*/

/**
 * @brief  初始化CAN协议和设备管理器
 * @param  hcan: CAN句柄指针
 * @retval None
 */
void AppCANProtocol_Init(CAN_HandleTypeDef *hcan);

/**
 * @brief  启动设备发现流程（上电后延迟1秒调用）
 * @retval None
 */
void AppCANProtocol_StartDiscovery(void);

/**
 * @brief  协议定时任务 - 在主循环中调用
 * @retval None
 */
void AppCANProtocol_Task(void);

/**
 * @brief  CAN接收回调处理函数
 * @param  rx_id: 接收到的CAN ID
 * @param  rx_data: 接收到的数据
 * @param  rx_len: 数据长度
 * @retval None
 */
void AppCANProtocol_RxCallback(uint32_t rx_id, uint8_t *rx_data, uint8_t rx_len);

/* ========== 设备管理函数 ========== */

/**
 * @brief  获取在线节点数量
 * @retval 在线节点数量
 */
uint8_t AppCANProtocol_GetOnlineNodeCount(void);

/**
 * @brief  根据节点ID获取节点信息
 * @param  node_id: 节点ID
 * @retval 节点信息指针，未找到返回NULL
 */
NodeInfo_t* AppCANProtocol_GetNodeInfo(uint8_t node_id);

/**
 * @brief  根据节点ID判断设备类型
 * @param  node_id: 节点ID
 * @retval 设备类型
 */
DeviceType_t AppCANProtocol_GetDeviceType(uint8_t node_id);

/**
 * @brief  列出所有在线设备（调试用）
 * @retval None
 */
void AppCANProtocol_ListDevices(void);

/* ========== 电机控制函数 ========== */

/**
 * @brief  控制电机
 * @param  motor_id: 电机ID (1-20)
 * @param  control: 控制参数
 * @retval true: 成功, false: 失败
 */
bool AppCANProtocol_ControlMotor(uint8_t motor_id, MotorControl_t *control);

/**
 * @brief  启动电机
 * @param  motor_id: 电机ID (1-20)
 * @param  speed: 目标转速 (RPM)
 * @param  direction: 方向
 * @retval true: 成功, false: 失败
 */
bool AppCANProtocol_StartMotor(uint8_t motor_id, uint16_t speed, MotorDirection_t direction);

/**
 * @brief  停止电机
 * @param  motor_id: 电机ID (1-20)
 * @retval true: 成功, false: 失败
 */
bool AppCANProtocol_StopMotor(uint8_t motor_id);

/**
 * @brief  设置电机速度
 * @param  motor_id: 电机ID (1-20)
 * @param  speed: 目标转速 (RPM)
 * @retval true: 成功, false: 失败
 */
bool AppCANProtocol_SetMotorSpeed(uint8_t motor_id, uint16_t speed);

/**
 * @brief  查询电机状态
 * @param  motor_id: 电机ID (1-20)
 * @retval true: 成功, false: 失败
 */
bool AppCANProtocol_QueryMotorStatus(uint8_t motor_id);

/**
 * @brief  获取电机状态
 * @param  motor_id: 电机ID (1-20)
 * @retval 电机状态指针，未找到返回NULL
 */
MotorStatus_t* AppCANProtocol_GetMotorStatus(uint8_t motor_id);

/**
 * @brief  紧急停止所有电机
 * @retval true: 成功, false: 失败
 */
bool AppCANProtocol_EmergencyStopAll(void);

/* ========== 传感器控制函数 ========== */

/**
 * @brief  查询传感器数据
 * @param  sensor_id: 传感器ID (1-20)
 * @param  sensor_type: 传感器类型
 * @retval true: 成功, false: 失败
 */
bool AppCANProtocol_QuerySensor(uint8_t sensor_id, SensorType_t sensor_type);

/**
 * @brief  获取传感器数据
 * @param  sensor_id: 传感器ID (1-20)
 * @retval 传感器数据指针，未找到返回NULL
 */
SensorData_t* AppCANProtocol_GetSensorData(uint8_t sensor_id);

/* ========== IO设备控制函数 ========== */

/**
 * @brief  控制IO端口
 * @param  io_id: IO设备ID (1-20)
 * @param  port: 端口号 (0-7)
 * @param  mask: 位掩码
 * @param  value: 输出值
 * @retval true: 成功, false: 失败
 */
bool AppCANProtocol_ControlIO(uint8_t io_id, uint8_t port, uint8_t mask, uint8_t value);

/**
 * @brief  获取IO状态
 * @param  io_id: IO设备ID (1-20)
 * @retval IO状态指针，未找到返回NULL
 */
IOStatus_t* AppCANProtocol_GetIOStatus(uint8_t io_id);

/* ========== 内部辅助函数 ========== */

/**
 * @brief  发送心跳广播
 * @retval None
 */
void AppCANProtocol_SendHeartbeat(void);

/**
 * @brief  检查节点超时
 * @retval None
 */
void AppCANProtocol_CheckNodeTimeout(void);

/**
 * @brief  发送CAN消息
 * @param  can_id: CAN ID
 * @param  data: 数据指针
 * @param  len: 数据长度
 * @retval true: 成功, false: 失败
 */
bool AppCANProtocol_SendMessage(uint32_t can_id, uint8_t *data, uint8_t len);

/**
 * @brief  计算简单校验和
 * @param  data: 数据指针
 * @param  len: 数据长度
 * @retval 校验和
 */
uint8_t AppCANProtocol_CalcChecksum(uint8_t *data, uint8_t len);

#ifdef __cplusplus
}
#endif

#endif /* __APP_CAN_PROTOCOL_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
