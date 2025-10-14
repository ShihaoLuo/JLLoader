/**
  ******************************************************************************
  * @file    can_protocol_slave.c
  * @brief   CAN协议从机端实现（电机节点）
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32f1xx_hal.h"
#include "stm32f1xx_hal_can.h"
#define CAN_PROTOCOL_SLAVE
#include "can_protocol.h"
#include <string.h>

/* External variables --------------------------------------------------------*/
extern CAN_HandleTypeDef hcan1;

/* Private variables ---------------------------------------------------------*/
static uint8_t my_node_id = MOTOR_ID(1);  // 默认为电机1
static DeviceType_t my_device_type = DEV_TYPE_MOTOR;
static uint32_t uptime_seconds = 0;
static uint8_t device_status = 0x00;  // 0x00=正常运行
static uint8_t error_code = 0x00;

/* 电机状态 */
static MotorStatus_t current_motor_status = {
    .status = 0x00,
    .actual_speed = 0,
    .direction = MOTOR_DIR_STOP,
    .current = 0,
    .temperature = 25,  // 初始温度25°C
    .fault = 0x00
};

/* 电机控制目标 */
static uint16_t target_speed = 0;
static MotorDirection_t target_direction = MOTOR_DIR_STOP;
static bool motor_running = false;

/* Private function prototypes -----------------------------------------------*/
static void HandleDiscoveryRequest(void);
static void HandleHeartbeatRequest(void);
static void HandleMotorCommand(uint8_t* data, uint8_t len);
static void HandleMotorQuery(void);
static bool SendCANMessage(uint32_t id, uint8_t* data, uint8_t len);
static void Motor_UpdateStatus(void);

/* Public functions ----------------------------------------------------------*/

/**
 * @brief  初始化CAN协议（从机端）
 * @param  node_id: 本机节点ID
 * @param  device_type: 设备类型
 * @retval None
 */
void CANProtocol_Slave_Init(uint8_t node_id, DeviceType_t device_type)
{
    my_node_id = node_id;
    my_device_type = device_type;
    uptime_seconds = 0;
    
    // 初始化电机状态
    current_motor_status.status = 0x00;
    current_motor_status.actual_speed = 0;
    current_motor_status.direction = MOTOR_DIR_STOP;
    current_motor_status.current = 0;
    current_motor_status.temperature = 25;
    current_motor_status.fault = 0x00;
    
    motor_running = false;
    
    // 从机不使用串口调试
}

/**
 * @brief  发送心跳响应
 * @param  status: 状态码
 * @param  error: 错误码
 * @retval None
 */
void CANProtocol_SendHeartbeatResponse(uint8_t status, uint8_t error)
{
    uint8_t data[4];
    data[0] = status;
    data[1] = error;
    data[2] = (uptime_seconds >> 8) & 0xFF;
    data[3] = uptime_seconds & 0xFF;
    
    uint32_t id = CAN_ID(CAN_FUNC_HEARTBEAT, my_node_id);
    SendCANMessage(id, data, 4);
}

/**
 * @brief  发送设备信息响应
 * @retval None
 */
void CANProtocol_SendDeviceInfo(void)
{
    uint8_t data[8];
    data[0] = my_device_type;
    data[1] = 0x01;  // 硬件版本 v1
    data[2] = 0x01;  // 软件版本高字节 v1.0
    data[3] = 0x00;  // 软件版本低字节
    data[4] = 0x01;  // 状态：在线
    data[5] = 0x0F;  // 能力位：支持速度/方向/扭矩/编码器
    data[6] = 0x00;  // 保留
    data[7] = 0x00;  // 保留
    
    uint32_t id = CAN_ID(CAN_FUNC_SYSTEM, my_node_id);
    SendCANMessage(id, data, 8);
    
    // 从机不使用串口调试
}

/**
 * @brief  发送电机状态
 * @param  status: 电机状态结构体指针
 * @retval None
 */
void CANProtocol_SendMotorStatus(MotorStatus_t* status)
{
    uint8_t data[8];
    
    if (status == NULL)
    {
        status = &current_motor_status;
    }
    
    data[0] = status->status;
    data[1] = (status->actual_speed >> 8) & 0xFF;
    data[2] = status->actual_speed & 0xFF;
    data[3] = status->direction;
    data[4] = (status->current >> 8) & 0xFF;
    data[5] = status->current & 0xFF;
    data[6] = status->temperature;
    data[7] = status->fault;
    
    uint32_t id = CAN_ID(CAN_FUNC_MOTOR_STATUS, my_node_id);
    SendCANMessage(id, data, 8);
}

/**
 * @brief  发送命令响应
 * @param  func: 功能码
 * @param  result: 结果码
 * @param  error: 错误码
 * @retval None
 */
void CANProtocol_SendResponse(uint8_t func, uint8_t result, uint8_t error)
{
    uint8_t data[2];
    data[0] = result;
    data[1] = error;
    
    uint32_t id = CAN_ID(func, my_node_id);
    SendCANMessage(id, data, 2);
}

/**
 * @brief  CAN接收处理（在CAN中断回调中调用）
 * @param  id: CAN消息ID
 * @param  data: 数据指针
 * @param  len: 数据长度
 * @retval None
 */
void CANProtocol_Slave_RxHandler(uint32_t id, uint8_t* data, uint8_t len)
{
    uint8_t func = CAN_GET_FUNC(id);
    uint8_t addr = CAN_GET_ADDR(id);
    
    // 检查是否是发给本节点或广播的消息
    if (addr != my_node_id && addr != CAN_ADDR_BROADCAST)
    {
        return;
    }
    
    switch (func)
    {
        case CAN_FUNC_DISCOVERY:
        case CAN_FUNC_SYSTEM:
            if (addr == CAN_ADDR_BROADCAST)
            {
                HandleDiscoveryRequest();
            }
            break;
            
        case CAN_FUNC_HEARTBEAT:
            if (addr == CAN_ADDR_BROADCAST)
            {
                HandleHeartbeatRequest();
            }
            break;
            
        case CAN_FUNC_MOTOR_CMD:
            HandleMotorCommand(data, len);
            break;
            
        case CAN_FUNC_MOTOR_QUERY:
            HandleMotorQuery();
            break;
            
        case CAN_FUNC_BROADCAST:
            // 紧急停止
            if (len >= 2 && data[0] == 0xE5 && data[1] == 0xE5)
            {
                motor_running = false;
                target_speed = 0;
                target_direction = MOTOR_DIR_STOP;
                current_motor_status.status = 0x00;
                current_motor_status.actual_speed = 0;
                
                // 从机不使用串口调试
            }
            break;
            
        default:
            break;
    }
}

/**
 * @brief  更新运行时间（1秒调用一次）
 * @retval None
 */
void CANProtocol_UpdateUptime(void)
{
    uptime_seconds++;
}

/* Private functions ---------------------------------------------------------*/

/**
 * @brief  处理设备发现请求
 * @retval None
 */
static void HandleDiscoveryRequest(void)
{
    // 延迟响应，避免总线冲突
    HAL_Delay(my_node_id % 10);
    
    CANProtocol_SendDeviceInfo();
}

/**
 * @brief  处理心跳请求
 * @retval None
 */
static void HandleHeartbeatRequest(void)
{
    CANProtocol_SendHeartbeatResponse(device_status, error_code);
}

/**
 * @brief  处理电机控制命令
 * @param  data: 数据指针
 * @param  len: 数据长度
 * @retval None
 */
static void HandleMotorCommand(uint8_t* data, uint8_t len)
{
    if (len < 7)
    {
        CANProtocol_SendResponse(CAN_FUNC_MOTOR_RESP, 0x02, 0x03);  // 参数错误
        return;
    }
    
    // 验证校验和
    if (!CANProtocol_VerifyChecksum(data, len))
    {
        CANProtocol_SendResponse(CAN_FUNC_MOTOR_RESP, 0x02, 0x04);  // 校验和错误
        return;
    }
    
    MotorCommand_t cmd = (MotorCommand_t)data[0];
    uint16_t speed = (data[1] << 8) | data[2];
    MotorDirection_t dir = (MotorDirection_t)data[3];
    
    // 从机不使用串口调试
    
    switch (cmd)
    {
        case MOTOR_CMD_STOP:
            motor_running = false;
            target_speed = 0;
            target_direction = MOTOR_DIR_STOP;
            current_motor_status.status &= ~MOTOR_STATUS_RUNNING;
            
            CANProtocol_SendResponse(CAN_FUNC_MOTOR_RESP, 0x00, 0x00);
            break;
            
        case MOTOR_CMD_START:
            motor_running = true;
            current_motor_status.status |= MOTOR_STATUS_RUNNING;
            
            CANProtocol_SendResponse(CAN_FUNC_MOTOR_RESP, 0x00, 0x00);
            break;
            
        case MOTOR_CMD_EMERGENCY_STOP:
            motor_running = false;
            target_speed = 0;
            target_direction = MOTOR_DIR_STOP;
            current_motor_status.status = 0x00;
            
            CANProtocol_SendResponse(CAN_FUNC_MOTOR_RESP, 0x00, 0x00);
            break;
            
        case MOTOR_CMD_SET_SPEED:
            target_speed = speed;
            
            CANProtocol_SendResponse(CAN_FUNC_MOTOR_RESP, 0x00, 0x00);
            break;
            
        case MOTOR_CMD_SET_DIRECTION:
            target_direction = dir;
            
            CANProtocol_SendResponse(CAN_FUNC_MOTOR_RESP, 0x00, 0x00);
            break;
            
        case MOTOR_CMD_SET_BOTH:
            target_speed = speed;
            target_direction = dir;
            
            CANProtocol_SendResponse(CAN_FUNC_MOTOR_RESP, 0x00, 0x00);
            break;
            
        default:
            CANProtocol_SendResponse(CAN_FUNC_MOTOR_RESP, 0x02, 0x03);
            break;
    }
    
    // 更新状态
    Motor_UpdateStatus();
}

/**
 * @brief  处理电机状态查询
 * @retval None
 */
static void HandleMotorQuery(void)
{
    Motor_UpdateStatus();
    CANProtocol_SendMotorStatus(&current_motor_status);
    
    // 从机不使用串口调试
}

/**
 * @brief  更新电机状态（模拟）
 * @retval None
 */
static void Motor_UpdateStatus(void)
{
    // 模拟电机状态更新
    if (motor_running)
    {
        // 模拟速度逐渐达到目标速度
        if (current_motor_status.actual_speed < target_speed)
        {
            current_motor_status.actual_speed += 10;
            if (current_motor_status.actual_speed > target_speed)
            {
                current_motor_status.actual_speed = target_speed;
            }
        }
        else if (current_motor_status.actual_speed > target_speed)
        {
            current_motor_status.actual_speed -= 10;
            if (current_motor_status.actual_speed < target_speed)
            {
                current_motor_status.actual_speed = target_speed;
            }
        }
        
        current_motor_status.direction = target_direction;
        current_motor_status.status |= MOTOR_STATUS_RUNNING;
        
        // 模拟电流（速度越高电流越大）
        current_motor_status.current = 100 + (current_motor_status.actual_speed / 10);
        
        // 模拟温度（运行时温度升高）
        if (current_motor_status.temperature < 40)
        {
            current_motor_status.temperature++;
        }
    }
    else
    {
        // 停止时速度降为0
        if (current_motor_status.actual_speed > 0)
        {
            current_motor_status.actual_speed -= 20;
            if (current_motor_status.actual_speed < 0)
            {
                current_motor_status.actual_speed = 0;
            }
        }
        
        current_motor_status.status &= ~MOTOR_STATUS_RUNNING;
        current_motor_status.current = 0;
        
        // 停止时温度下降
        if (current_motor_status.temperature > 25)
        {
            current_motor_status.temperature--;
        }
    }
}

/**
 * @brief  发送CAN消息
 * @param  id: CAN消息ID
 * @param  data: 数据指针
 * @param  len: 数据长度
 * @retval true=成功, false=失败
 */
static bool SendCANMessage(uint32_t id, uint8_t* data, uint8_t len)
{
    CAN_TxHeaderTypeDef TxHeader;
    uint32_t TxMailbox;
    
    TxHeader.StdId = id;
    TxHeader.ExtId = 0x00;
    TxHeader.RTR = CAN_RTR_DATA;
    TxHeader.IDE = CAN_ID_STD;
    TxHeader.DLC = len;
    TxHeader.TransmitGlobalTime = DISABLE;
    
    if (HAL_CAN_AddTxMessage(&hcan1, &TxHeader, data, &TxMailbox) == HAL_OK)
    {
        return true;
    }
    
    return false;
}

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
