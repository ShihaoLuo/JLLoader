/**
  ******************************************************************************
  * @file    motor_can_protocol.h
  * @author  Generated
  * @brief   Motor slave-side CAN protocol implementation
  *          电机从机端CAN协议实现 - 被动响应模式
  ******************************************************************************
  */

#ifndef __MOTOR_CAN_PROTOCOL_H
#define __MOTOR_CAN_PROTOCOL_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include <stdint.h>
#include <stdbool.h>

/* Exported constants --------------------------------------------------------*/

/* CAN协议功能码定义 (5位: 0-31) - 与主机保持一致 */
#define CAN_FUNC_SYSTEM         0x00    // 系统管理（设备发现）
#define CAN_FUNC_HEARTBEAT      0x01    // 心跳
#define CAN_FUNC_DISCOVERY      0x02    // 设备发现
#define CAN_FUNC_INFO           0x03    // 设备信息查询
#define CAN_FUNC_MOTOR_CMD      0x04    // 电机控制命令
#define CAN_FUNC_MOTOR_RESP     0x05    // 电机控制响应
#define CAN_FUNC_MOTOR_QUERY    0x06    // 电机状态查询
#define CAN_FUNC_MOTOR_STATUS   0x07    // 电机状态上报
#define CAN_FUNC_MOTOR_PARAM    0x08    // 电机参数设置
#define CAN_FUNC_BROADCAST      0x1F    // 广播命令

/* CAN节点地址定义 */
#define CAN_ADDR_MASTER         0x00    // 主机地址
#define CAN_ADDR_BROADCAST      0x3F    // 广播地址

/* 电机节点ID配置 - 可通过DIP开关或配置修改 */
#ifndef MOTOR_NODE_ID
#define MOTOR_NODE_ID           0x01    // 默认为电机1 (可修改为0x01-0x14)
#endif

/* 电机转速约束 */
#define MOTOR_MAX_SPEED         800     // 最大转速 800 RPM
#define MOTOR_MIN_SPEED         0       // 最小转速 0 RPM

/* 设备信息 */
#define DEVICE_TYPE             0x01    // 电机设备
#define HW_VERSION              0x10    // 硬件版本 1.0
#define SW_VERSION_H            0x01    // 软件版本高字节
#define SW_VERSION_L            0x00    // 软件版本低字节
#define DEVICE_CAPABILITY       0x0F    // 能力位：支持速度/位置/扭矩/编码器

/* CAN ID计算宏 */
#define CAN_ID(func, addr)      (((func) << 6) | (addr))
#define CAN_GET_FUNC(id)        (((id) >> 6) & 0x1F)
#define CAN_GET_ADDR(id)        ((id) & 0x3F)

/* Exported types ------------------------------------------------------------*/

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
 * @brief 电机状态结构体
 */
typedef struct {
    bool is_running;            // 是否运行中
    uint16_t target_speed;      // 目标转速 (RPM)
    uint16_t actual_speed;      // 实际转速 (RPM)
    MotorDirection_t direction; // 当前方向
    uint16_t current;           // 电流 (mA)
    uint8_t temperature;        // 温度 (°C)
    uint8_t fault;              // 故障位图
    uint32_t uptime;            // 运行时间 (秒)
} MotorStatus_t;

/**
 * @brief 命令响应结果枚举
 */
typedef enum {
    CMD_RESULT_SUCCESS      = 0x00,     // 成功
    CMD_RESULT_REJECTED     = 0x01,     // 拒绝（正在执行其他命令）
    CMD_RESULT_PARAM_ERROR  = 0x02,     // 参数错误
    CMD_RESULT_HW_FAULT     = 0x03,     // 硬件故障
    CMD_RESULT_UNKNOWN      = 0xFF      // 未知错误
} CommandResult_t;

/**
 * @brief 设备状态枚举
 */
typedef enum {
    DEVICE_STATUS_NORMAL    = 0x00,     // 正常运行
    DEVICE_STATUS_INIT      = 0x01,     // 初始化中
    DEVICE_STATUS_WARNING   = 0x02,     // 警告状态
    DEVICE_STATUS_ERROR     = 0x03,     // 错误状态
    DEVICE_STATUS_FAULT     = 0xFF      // 严重故障
} DeviceStatus_t;

/* Exported variables --------------------------------------------------------*/
extern MotorStatus_t g_motor_status;
extern CAN_HandleTypeDef hcan1;

/* Exported function prototypes ---------------------------------------------*/

/**
 * @brief  初始化电机CAN协议
 * @param  hcan: CAN句柄指针
 * @param  node_id: 节点ID (0x01-0x14)
 * @retval None
 */
void MotorCANProtocol_Init(CAN_HandleTypeDef *hcan, uint8_t node_id);

/**
 * @brief  CAN接收回调处理函数（在中断中调用）
 * @param  rx_id: 接收到的CAN ID
 * @param  rx_data: 接收到的数据
 * @param  rx_len: 数据长度
 * @retval None
 */
void MotorCANProtocol_RxCallback(uint32_t rx_id, uint8_t *rx_data, uint8_t rx_len);

/**
 * @brief  定时任务 - 在主循环中调用（用于更新状态和运行时间）
 * @retval None
 */
void MotorCANProtocol_Task(void);

/**
 * @brief  更新电机状态（应用层调用）
 * @param  speed: 实际转速 (RPM)
 * @param  current: 电流 (mA)
 * @param  temperature: 温度 (°C)
 * @param  fault: 故障位图
 * @retval None
 */
void MotorCANProtocol_UpdateStatus(uint16_t speed, uint16_t current, 
                                    uint8_t temperature, uint8_t fault);

/**
 * @brief  获取目标速度（供电机控制使用）
 * @retval 目标转速 (RPM)
 */
uint16_t MotorCANProtocol_GetTargetSpeed(void);

/**
 * @brief  获取目标方向（供电机控制使用）
 * @retval 方向
 */
MotorDirection_t MotorCANProtocol_GetTargetDirection(void);

/**
 * @brief  获取运行状态（供电机控制使用）
 * @retval true: 运行中, false: 停止
 */
bool MotorCANProtocol_IsRunning(void);

/**
 * @brief  设置故障标志
 * @param  fault: 故障位图
 * @retval None
 */
void MotorCANProtocol_SetFault(uint8_t fault);

/* ========== 内部函数（私有） ========== */

/**
 * @brief  响应设备发现请求
 * @retval None
 */
void MotorCANProtocol_ResponseDiscovery(void);

/**
 * @brief  响应心跳请求
 * @retval None
 */
void MotorCANProtocol_ResponseHeartbeat(void);

/**
 * @brief  处理电机控制命令
 * @param  data: 命令数据
 * @param  len: 数据长度
 * @retval None
 */
void MotorCANProtocol_HandleMotorCommand(uint8_t *data, uint8_t len);

/**
 * @brief  响应电机状态查询
 * @param  query_type: 查询类型
 * @retval None
 */
void MotorCANProtocol_ResponseMotorStatus(uint8_t query_type);

/**
 * @brief  发送命令响应
 * @param  result: 结果码
 * @param  error: 错误码
 * @retval None
 */
void MotorCANProtocol_SendCommandResponse(CommandResult_t result, uint8_t error);

/**
 * @brief  发送CAN消息
 * @param  can_id: CAN ID
 * @param  data: 数据指针
 * @param  len: 数据长度
 * @retval true: 成功, false: 失败
 */
bool MotorCANProtocol_SendMessage(uint32_t can_id, uint8_t *data, uint8_t len);

/**
 * @brief  计算校验和
 * @param  data: 数据指针
 * @param  len: 数据长度
 * @retval 校验和
 */
uint8_t MotorCANProtocol_CalcChecksum(uint8_t *data, uint8_t len);

/**
 * @brief  验证校验和
 * @param  data: 数据指针（包含校验和）
 * @param  len: 总长度（包含校验和）
 * @retval true: 校验成功, false: 校验失败
 */
bool MotorCANProtocol_VerifyChecksum(uint8_t *data, uint8_t len);

#ifdef __cplusplus
}
#endif

#endif /* __MOTOR_CAN_PROTOCOL_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
