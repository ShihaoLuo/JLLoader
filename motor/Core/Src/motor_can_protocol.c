/**
  ******************************************************************************
  * @file    motor_can_protocol.c
  * @author  Generated
  * @brief   Motor slave-side CAN protocol implementation
  *          电机从机端CAN协议 - 被动响应模式
  *          重要：从机不主动发送数据，仅响应主机命令
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "motor_can_protocol.h"
#include "stm32f1xx_hal.h"
#include <string.h>

/* Private typedef -----------------------------------------------------------*/

/* Private define ------------------------------------------------------------*/
#define STATUS_UPDATE_INTERVAL  1000    // 状态更新间隔 (ms)

/* Private macro -------------------------------------------------------------*/

/* Private variables ---------------------------------------------------------*/
MotorStatus_t g_motor_status = {0};
static CAN_HandleTypeDef *phcan = NULL;
static uint8_t my_node_id = MOTOR_NODE_ID;
static DeviceStatus_t device_status = DEVICE_STATUS_INIT;
static uint8_t error_code = 0x00;
static uint32_t last_status_update_time = 0;

/* Private function prototypes -----------------------------------------------*/
static void ExecuteMotorCommand(MotorCommand_t cmd, uint16_t speed, 
                               MotorDirection_t direction, uint8_t accel, uint8_t decel);

/* Exported functions --------------------------------------------------------*/

/**
 * @brief  初始化电机CAN协议
 * @param  hcan: CAN句柄指针
 * @param  node_id: 节点ID (0x01-0x14)
 * @retval None
 */
void MotorCANProtocol_Init(CAN_HandleTypeDef *hcan, uint8_t node_id)
{
    phcan = hcan;
    my_node_id = node_id;
    
    // 初始化电机状态
    memset(&g_motor_status, 0, sizeof(MotorStatus_t));
    g_motor_status.is_running = false;
    g_motor_status.target_speed = 0;
    g_motor_status.actual_speed = 0;
    g_motor_status.direction = MOTOR_DIR_STOP;
    g_motor_status.current = 0;
    g_motor_status.temperature = 25;  // 默认室温
    g_motor_status.fault = 0x00;
    g_motor_status.uptime = 0;
    
    device_status = DEVICE_STATUS_NORMAL;
    error_code = 0x00;
    last_status_update_time = HAL_GetTick();
}

/**
 * @brief  CAN接收回调处理函数（在中断中调用）
 * @param  rx_id: 接收到的CAN ID
 * @param  rx_data: 接收到的数据
 * @param  rx_len: 数据长度
 * @retval None
 * @note   被动响应模式：仅在收到命令后才发送响应
 */
void MotorCANProtocol_RxCallback(uint32_t rx_id, uint8_t *rx_data, uint8_t rx_len)
{
    uint8_t func_code = CAN_GET_FUNC(rx_id);
    uint8_t target_addr = CAN_GET_ADDR(rx_id);
    
    // 检查消息是否发给本节点或广播
    if (target_addr != my_node_id && target_addr != CAN_ADDR_BROADCAST) {
        return;  // 不是发给本节点的消息，忽略
    }
    
    // 根据功能码处理并响应
    switch (func_code) {
        case CAN_FUNC_DISCOVERY:
            // 设备发现请求 - 响应设备信息
            MotorCANProtocol_ResponseDiscovery();
            break;
            
        case CAN_FUNC_HEARTBEAT:
            // 心跳请求 - 响应心跳
            MotorCANProtocol_ResponseHeartbeat();
            break;
            
        case CAN_FUNC_MOTOR_CMD:
            // 电机控制命令 - 处理命令并响应
            MotorCANProtocol_HandleMotorCommand(rx_data, rx_len);
            break;
            
        case CAN_FUNC_MOTOR_QUERY:
            // 电机状态查询 - 响应状态
            if (rx_len >= 1) {
                MotorCANProtocol_ResponseMotorStatus(rx_data[0]);
            }
            break;
            
        case CAN_FUNC_BROADCAST:
            // 广播命令 - 紧急停止
            if (rx_len >= 2 && rx_data[0] == 0xE5 && rx_data[1] == 0xE5) {
                ExecuteMotorCommand(MOTOR_CMD_EMERGENCY_STOP, 0, MOTOR_DIR_STOP, 0, 0);
            }
            break;
            
        default:
            // 未知命令，忽略
            break;
    }
}

/**
 * @brief  定时任务 - 在主循环中调用
 * @retval None
 * @note   仅更新内部状态，不主动发送数据
 */
void MotorCANProtocol_Task(void)
{
    uint32_t current_time = HAL_GetTick();
    
    // 每秒更新运行时间
    if ((current_time - last_status_update_time) >= STATUS_UPDATE_INTERVAL) {
        if (g_motor_status.is_running) {
            g_motor_status.uptime++;
        }
        last_status_update_time = current_time;
    }
}

/**
 * @brief  更新电机状态（应用层调用）
 * @param  speed: 实际转速 (RPM)
 * @param  current: 电流 (mA)
 * @param  temperature: 温度 (°C)
 * @param  fault: 故障位图
 * @retval None
 */
void MotorCANProtocol_UpdateStatus(uint16_t speed, uint16_t current, 
                                    uint8_t temperature, uint8_t fault)
{
    g_motor_status.actual_speed = speed;
    g_motor_status.current = current;
    g_motor_status.temperature = temperature;
    g_motor_status.fault = fault;
    
    // 根据故障更新设备状态
    if (fault != 0x00) {
        device_status = DEVICE_STATUS_ERROR;
        error_code = fault;
    } else {
        device_status = DEVICE_STATUS_NORMAL;
        error_code = 0x00;
    }
}

/**
 * @brief  获取目标速度（供电机控制使用）
 * @retval 目标转速 (RPM)
 */
uint16_t MotorCANProtocol_GetTargetSpeed(void)
{
    return g_motor_status.target_speed;
}

/**
 * @brief  获取目标方向（供电机控制使用）
 * @retval 方向
 */
MotorDirection_t MotorCANProtocol_GetTargetDirection(void)
{
    return g_motor_status.direction;
}

/**
 * @brief  获取运行状态（供电机控制使用）
 * @retval true: 运行中, false: 停止
 */
bool MotorCANProtocol_IsRunning(void)
{
    return g_motor_status.is_running;
}

/**
 * @brief  设置故障标志
 * @param  fault: 故障位图
 * @retval None
 */
void MotorCANProtocol_SetFault(uint8_t fault)
{
    g_motor_status.fault = fault;
    if (fault != 0x00) {
        device_status = DEVICE_STATUS_ERROR;
        error_code = fault;
    }
}

/* ========== 响应函数实现 ========== */

/**
 * @brief  响应设备发现请求
 * @retval None
 */
void MotorCANProtocol_ResponseDiscovery(void)
{
    uint8_t data[8];
    uint32_t can_id = CAN_ID(CAN_FUNC_SYSTEM, my_node_id);
    
    // 构造设备信息响应
    data[0] = DEVICE_TYPE;              // 设备类型：电机
    data[1] = HW_VERSION;               // 硬件版本
    data[2] = SW_VERSION_H;             // 软件版本高字节
    data[3] = SW_VERSION_L;             // 软件版本低字节
    data[4] = 0x01;                     // 状态：在线
    data[5] = DEVICE_CAPABILITY;        // 能力位
    data[6] = 0x00;                     // 保留
    data[7] = 0x00;                     // 保留
    
    MotorCANProtocol_SendMessage(can_id, data, 8);
}

/**
 * @brief  响应心跳请求
 * @retval None
 */
void MotorCANProtocol_ResponseHeartbeat(void)
{
    uint8_t data[4];
    uint32_t can_id = CAN_ID(CAN_FUNC_HEARTBEAT, my_node_id);
    
    // 构造心跳响应
    data[0] = device_status;            // 状态码
    data[1] = error_code;               // 错误码
    data[2] = (g_motor_status.uptime >> 8) & 0xFF;  // 运行时间高字节
    data[3] = g_motor_status.uptime & 0xFF;         // 运行时间低字节
    
    MotorCANProtocol_SendMessage(can_id, data, 4);
}

/**
 * @brief  处理电机控制命令
 * @param  data: 命令数据
 * @param  len: 数据长度
 * @retval None
 */
void MotorCANProtocol_HandleMotorCommand(uint8_t *data, uint8_t len)
{
    if (len < 8) {
        // 数据长度错误
        MotorCANProtocol_SendCommandResponse(CMD_RESULT_PARAM_ERROR, 0x01);
        return;
    }
    
    // 验证校验和
    if (!MotorCANProtocol_VerifyChecksum(data, 8)) {
        // 校验和错误
        MotorCANProtocol_SendCommandResponse(CMD_RESULT_PARAM_ERROR, 0x02);
        return;
    }
    
    // 解析命令
    MotorCommand_t cmd = (MotorCommand_t)data[0];
    uint16_t speed = (data[1] << 8) | data[2];
    MotorDirection_t direction = (MotorDirection_t)data[3];
    uint8_t accel = data[4];
    uint8_t decel = data[5];
    
    // 参数验证
    if (direction > MOTOR_DIR_CCW) {
        MotorCANProtocol_SendCommandResponse(CMD_RESULT_PARAM_ERROR, 0x03);
        return;
    }
    
    // 检查故障状态
    if (g_motor_status.fault != 0x00 && cmd != MOTOR_CMD_STOP && 
        cmd != MOTOR_CMD_EMERGENCY_STOP) {
        MotorCANProtocol_SendCommandResponse(CMD_RESULT_HW_FAULT, g_motor_status.fault);
        return;
    }
    
    // 执行命令
    ExecuteMotorCommand(cmd, speed, direction, accel, decel);
    
    // 发送成功响应
    MotorCANProtocol_SendCommandResponse(CMD_RESULT_SUCCESS, 0x00);
}

/**
 * @brief  响应电机状态查询
 * @param  query_type: 查询类型 (0x00=全部, 0x01=转速, 0x02=方向, 0x03=故障)
 * @retval None
 */
void MotorCANProtocol_ResponseMotorStatus(uint8_t query_type)
{
    uint8_t data[8];
    uint32_t can_id = CAN_ID(CAN_FUNC_MOTOR_STATUS, my_node_id);
    
    // 构造状态数据
    uint8_t status_byte = 0x00;
    status_byte |= (g_motor_status.is_running ? 0x01 : 0x00);           // Bit0: 运行状态
    status_byte |= (g_motor_status.fault != 0x00 ? 0x02 : 0x00);       // Bit1: 故障状态
    status_byte |= (g_motor_status.current > 1000 ? 0x04 : 0x00);      // Bit2: 负载状态
    status_byte |= (g_motor_status.direction == MOTOR_DIR_CCW ? 0x08 : 0x00);  // Bit3: 方向
    
    data[0] = status_byte;
    data[1] = (g_motor_status.actual_speed >> 8) & 0xFF;    // 实际转速高字节
    data[2] = g_motor_status.actual_speed & 0xFF;           // 实际转速低字节
    data[3] = g_motor_status.direction;                     // 当前方向
    data[4] = (g_motor_status.current >> 8) & 0xFF;         // 电流高字节
    data[5] = g_motor_status.current & 0xFF;                // 电流低字节
    data[6] = g_motor_status.temperature;                   // 温度
    data[7] = g_motor_status.fault;                         // 故障位图
    
    MotorCANProtocol_SendMessage(can_id, data, 8);
}

/**
 * @brief  发送命令响应
 * @param  result: 结果码
 * @param  error: 错误码
 * @retval None
 */
void MotorCANProtocol_SendCommandResponse(CommandResult_t result, uint8_t error)
{
    uint8_t data[2];
    uint32_t can_id = CAN_ID(CAN_FUNC_MOTOR_RESP, my_node_id);
    
    data[0] = result;
    data[1] = error;
    
    MotorCANProtocol_SendMessage(can_id, data, 2);
}

/**
 * @brief  发送CAN消息
 * @param  can_id: CAN ID
 * @param  data: 数据指针
 * @param  len: 数据长度
 * @retval true: 成功, false: 失败
 */
bool MotorCANProtocol_SendMessage(uint32_t can_id, uint8_t *data, uint8_t len)
{
    if (phcan == NULL || data == NULL || len > 8) {
        return false;
    }
    
    CAN_TxHeaderTypeDef TxHeader;
    uint32_t TxMailbox;
    
    TxHeader.StdId = can_id;
    TxHeader.ExtId = 0x00;
    TxHeader.RTR = CAN_RTR_DATA;
    TxHeader.IDE = CAN_ID_STD;
    TxHeader.DLC = len;
    TxHeader.TransmitGlobalTime = DISABLE;
    
    HAL_StatusTypeDef status = HAL_CAN_AddTxMessage(phcan, &TxHeader, data, &TxMailbox);
    
    return (status == HAL_OK);
}

/**
 * @brief  计算校验和
 * @param  data: 数据指针
 * @param  len: 数据长度
 * @retval 校验和
 */
uint8_t MotorCANProtocol_CalcChecksum(uint8_t *data, uint8_t len)
{
    uint8_t sum = 0;
    for (uint8_t i = 0; i < len; i++) {
        sum += data[i];
    }
    return sum & 0xFF;
}

/**
 * @brief  验证校验和
 * @param  data: 数据指针（包含校验和）
 * @param  len: 总长度（包含校验和）
 * @retval true: 校验成功, false: 校验失败
 */
bool MotorCANProtocol_VerifyChecksum(uint8_t *data, uint8_t len)
{
    if (len < 2) {
        return false;
    }
    
    uint8_t calculated = MotorCANProtocol_CalcChecksum(data, len - 1);
    uint8_t received = data[len - 1];
    
    return (calculated == received);
}

/* ========== 内部辅助函数 ========== */

/**
 * @brief  执行电机命令
 * @param  cmd: 命令
 * @param  speed: 目标转速
 * @param  direction: 方向
 * @param  accel: 加速度
 * @param  decel: 减速度
 * @retval None
 * @note   这里仅更新状态，实际电机控制由应用层实现
 */
static void ExecuteMotorCommand(MotorCommand_t cmd, uint16_t speed, 
                               MotorDirection_t direction, uint8_t accel, uint8_t decel)
{
    // 转速约束：限制在0-800 RPM范围内
    if (speed > MOTOR_MAX_SPEED) {
        speed = MOTOR_MAX_SPEED;
    }
    
    switch (cmd) {
        case MOTOR_CMD_STOP:
        case MOTOR_CMD_EMERGENCY_STOP:
            g_motor_status.is_running = false;
            g_motor_status.target_speed = 0;
            g_motor_status.direction = MOTOR_DIR_STOP;
            break;
            
        case MOTOR_CMD_START:
            g_motor_status.is_running = true;
            // 保持当前目标速度和方向
            break;
            
        case MOTOR_CMD_SET_SPEED:
            g_motor_status.target_speed = speed;
            if (speed > 0 && !g_motor_status.is_running) {
                g_motor_status.is_running = true;
            }
            break;
            
        case MOTOR_CMD_SET_DIRECTION:
            g_motor_status.direction = direction;
            break;
            
        case MOTOR_CMD_SET_BOTH:
            g_motor_status.target_speed = speed;
            g_motor_status.direction = direction;
            if (speed > 0) {
                g_motor_status.is_running = true;
            } else {
                g_motor_status.is_running = false;
            }
            break;
            
        default:
            break;
    }
    
    // 注意：accel和decel参数可用于实现更平滑的加减速控制
    // 这里暂时未使用，可在实际电机控制中实现
}

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
