/**
  ******************************************************************************
  * @file    app_can_protocol.c
  * @author  Generated
  * @brief   Host-side CAN protocol implementation
  *          主机端CAN协议实现 - 设备管理、心跳发现、设备控制
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "app_can_protocol.h"
#include "uart.h"
#include "stm32f1xx_hal.h"
#include <stdio.h>
#include <string.h>

/* Private typedef -----------------------------------------------------------*/

/* Private define ------------------------------------------------------------*/
#define HEARTBEAT_CHECK_INTERVAL    100     // 心跳检查间隔 (ms)

/* Private macro -------------------------------------------------------------*/

/* Private variables ---------------------------------------------------------*/
DeviceManager_t g_device_manager = {0};
static CAN_HandleTypeDef *phcan = NULL;

/* Private function prototypes -----------------------------------------------*/
static void HandleSystemMessage(uint8_t node_id, uint8_t *data, uint8_t len);
static void HandleHeartbeatMessage(uint8_t node_id, uint8_t *data, uint8_t len);
static void HandleMotorResponse(uint8_t node_id, uint8_t *data, uint8_t len);
static void HandleMotorStatus(uint8_t node_id, uint8_t *data, uint8_t len);
static void HandleSensorData(uint8_t node_id, uint8_t *data, uint8_t len);
static void HandleIOStatus(uint8_t node_id, uint8_t *data, uint8_t len);
static NodeInfo_t* FindOrCreateNode(uint8_t node_id);
static uint8_t GetMotorIndex(uint8_t motor_id);
static uint8_t GetSensorIndex(uint8_t sensor_id);
static uint8_t GetIOIndex(uint8_t io_id);

/* Exported functions --------------------------------------------------------*/

/**
 * @brief  初始化CAN协议和设备管理器
 * @param  hcan: CAN句柄指针
 * @retval None
 */
void AppCANProtocol_Init(CAN_HandleTypeDef *hcan)
{
    phcan = hcan;
    
    // 初始化设备管理器
    memset(&g_device_manager, 0, sizeof(DeviceManager_t));
    g_device_manager.heartbeat_seq = 0;
    g_device_manager.node_count = 0;
    g_device_manager.discovery_done = false;
    g_device_manager.last_heartbeat_time = HAL_GetTick();
    g_device_manager.last_check_time = HAL_GetTick();
    
    // 初始化所有节点为无效
    for (uint8_t i = 0; i < MAX_NODES; i++) {
        g_device_manager.nodes[i].is_valid = false;
        g_device_manager.nodes[i].status = NODE_STATUS_OFFLINE;
    }
    
    UART_Printf("CAN Protocol Initialized\r\n");
}

/**
 * @brief  启动设备发现流程（上电后延迟1秒调用）
 * @retval None
 */
void AppCANProtocol_StartDiscovery(void)
{
    uint8_t data[2] = {0x01, 0x00};  // 发现命令
    uint32_t can_id = CAN_ID(CAN_FUNC_DISCOVERY, CAN_ADDR_BROADCAST);
    
    g_device_manager.discovery_start_time = HAL_GetTick();
    g_device_manager.discovery_done = false;
    
    UART_Printf("Starting device discovery...\r\n");
    
    // 发送设备发现广播
    if (AppCANProtocol_SendMessage(can_id, data, 2)) {
        UART_Printf("Discovery broadcast sent (ID: 0x%03X)\r\n", can_id);
    } else {
        UART_Printf("Discovery broadcast failed!\r\n");
    }
}

/**
 * @brief  协议定时任务 - 在主循环中调用
 * @retval None
 */
void AppCANProtocol_Task(void)
{
    uint32_t current_time = HAL_GetTick();
    
    // 检查发现是否完成（仅执行一次）
    if (!g_device_manager.discovery_done && 
        g_device_manager.discovery_start_time > 0 &&
        (current_time - g_device_manager.discovery_start_time) >= DISCOVERY_TIMEOUT) {
        g_device_manager.discovery_done = true;
        UART_Printf("Device discovery completed. Found %d devices.\r\n", 
                    g_device_manager.node_count);
        AppCANProtocol_ListDevices();
    }
    
    // 定期发送心跳 (500ms)
    if ((current_time - g_device_manager.last_heartbeat_time) >= HEARTBEAT_INTERVAL) {
        AppCANProtocol_SendHeartbeat();
        g_device_manager.last_heartbeat_time = current_time;
    }
    
    // 定期检查节点超时 (100ms)
    if ((current_time - g_device_manager.last_check_time) >= HEARTBEAT_CHECK_INTERVAL) {
        AppCANProtocol_CheckNodeTimeout();
        g_device_manager.last_check_time = current_time;
    }
}

/**
 * @brief  CAN接收回调处理函数
 * @param  rx_id: 接收到的CAN ID
 * @param  rx_data: 接收到的数据
 * @param  rx_len: 数据长度
 * @retval None
 */
void AppCANProtocol_RxCallback(uint32_t rx_id, uint8_t *rx_data, uint8_t rx_len)
{
    uint8_t func_code = CAN_GET_FUNC(rx_id);
    uint8_t node_id = CAN_GET_ADDR(rx_id);
    
    // 忽略来自主机地址的消息
    if (node_id == CAN_ADDR_MASTER || node_id == CAN_ADDR_BROADCAST) {
        return;
    }
    
    // 根据功能码分发处理
    switch (func_code) {
        case CAN_FUNC_SYSTEM:
            HandleSystemMessage(node_id, rx_data, rx_len);
            break;
            
        case CAN_FUNC_HEARTBEAT:
            HandleHeartbeatMessage(node_id, rx_data, rx_len);
            break;
            
        case CAN_FUNC_MOTOR_RESP:
            HandleMotorResponse(node_id, rx_data, rx_len);
            break;
            
        case CAN_FUNC_MOTOR_STATUS:
            HandleMotorStatus(node_id, rx_data, rx_len);
            break;
            
        case CAN_FUNC_SENSOR_DATA:
            HandleSensorData(node_id, rx_data, rx_len);
            break;
            
        case CAN_FUNC_IO_STATUS:
            HandleIOStatus(node_id, rx_data, rx_len);
            break;
            
        default:
            UART_Printf("Unknown function code: 0x%02X from node 0x%02X\r\n", 
                       func_code, node_id);
            break;
    }
}

/* ========== 设备管理函数实现 ========== */

/**
 * @brief  获取在线节点数量
 * @retval 在线节点数量
 */
uint8_t AppCANProtocol_GetOnlineNodeCount(void)
{
    uint8_t count = 0;
    for (uint8_t i = 0; i < MAX_NODES; i++) {
        if (g_device_manager.nodes[i].is_valid && 
            g_device_manager.nodes[i].status == NODE_STATUS_ONLINE) {
            count++;
        }
    }
    return count;
}

/**
 * @brief  根据节点ID获取节点信息
 * @param  node_id: 节点ID
 * @retval 节点信息指针，未找到返回NULL
 */
NodeInfo_t* AppCANProtocol_GetNodeInfo(uint8_t node_id)
{
    for (uint8_t i = 0; i < MAX_NODES; i++) {
        if (g_device_manager.nodes[i].is_valid && 
            g_device_manager.nodes[i].node_id == node_id) {
            return &g_device_manager.nodes[i];
        }
    }
    return NULL;
}

/**
 * @brief  根据节点ID判断设备类型
 * @param  node_id: 节点ID
 * @retval 设备类型
 */
DeviceType_t AppCANProtocol_GetDeviceType(uint8_t node_id)
{
    if (node_id >= CAN_ADDR_MOTOR_BASE && node_id < CAN_ADDR_MOTOR_BASE + MAX_MOTORS) {
        return DEV_TYPE_MOTOR;
    } else if (node_id >= CAN_ADDR_SENSOR_BASE && node_id < CAN_ADDR_SENSOR_BASE + MAX_SENSORS) {
        return DEV_TYPE_SENSOR;
    } else if (node_id >= CAN_ADDR_IO_BASE && node_id < CAN_ADDR_IO_BASE + MAX_IO_DEVICES) {
        return DEV_TYPE_IO;
    }
    return DEV_TYPE_UNKNOWN;
}

/**
 * @brief  列出所有在线设备（调试用）
 * @retval None
 */
void AppCANProtocol_ListDevices(void)
{
    UART_Printf("\r\n========== Device List ==========\r\n");
    UART_Printf("Total Nodes: %d\r\n", g_device_manager.node_count);
    
    for (uint8_t i = 0; i < MAX_NODES; i++) {
        if (g_device_manager.nodes[i].is_valid) {
            NodeInfo_t *node = &g_device_manager.nodes[i];
            const char *type_str = "UNKNOWN";
            const char *status_str = "OFFLINE";
            
            // 设备类型字符串
            switch (node->device_type) {
                case DEV_TYPE_MOTOR: type_str = "MOTOR"; break;
                case DEV_TYPE_SENSOR: type_str = "SENSOR"; break;
                case DEV_TYPE_IO: type_str = "IO"; break;
                case DEV_TYPE_HYBRID: type_str = "HYBRID"; break;
                default: break;
            }
            
            // 状态字符串
            switch (node->status) {
                case NODE_STATUS_ONLINE: status_str = "ONLINE"; break;
                case NODE_STATUS_OFFLINE: status_str = "OFFLINE"; break;
                case NODE_STATUS_ERROR: status_str = "ERROR"; break;
                case NODE_STATUS_WARNING: status_str = "WARNING"; break;
            }
            
            UART_Printf("  [%d] ID:0x%02X Type:%s Status:%s HW:%d.%d SW:%d.%d\r\n",
                       i, node->node_id, type_str, status_str,
                       node->hw_version >> 4, node->hw_version & 0x0F,
                       node->sw_version >> 8, node->sw_version & 0xFF);
        }
    }
    UART_Printf("================================\r\n\r\n");
}

/* ========== 电机控制函数实现 ========== */

/**
 * @brief  控制电机
 * @param  motor_id: 电机ID (1-20)
 * @param  control: 控制参数
 * @retval true: 成功, false: 失败
 */
bool AppCANProtocol_ControlMotor(uint8_t motor_id, MotorControl_t *control)
{
    if (motor_id < 1 || motor_id > MAX_MOTORS || control == NULL) {
        return false;
    }
    
    // 转速约束：限制在0-800 RPM范围内
    if (control->target_speed > MOTOR_MAX_SPEED) {
        control->target_speed = MOTOR_MAX_SPEED;
    }
    
    uint8_t node_id = CAN_ADDR_MOTOR_BASE + (motor_id - 1);
    uint32_t can_id = CAN_ID(CAN_FUNC_MOTOR_CMD, node_id);
    
    // 构造命令数据
    uint8_t data[8];
    data[0] = control->command;
    data[1] = (control->target_speed >> 8) & 0xFF;
    data[2] = control->target_speed & 0xFF;
    data[3] = control->direction;
    data[4] = control->accel;
    data[5] = control->decel;
    data[6] = 0x00;  // 保留
    data[7] = AppCANProtocol_CalcChecksum(data, 6);
    
    return AppCANProtocol_SendMessage(can_id, data, 8);
}

/**
 * @brief  启动电机
 * @param  motor_id: 电机ID (1-20)
 * @param  speed: 目标转速 (RPM)
 * @param  direction: 方向
 * @retval true: 成功, false: 失败
 */
bool AppCANProtocol_StartMotor(uint8_t motor_id, uint16_t speed, MotorDirection_t direction)
{
    MotorControl_t control = {
        .command = MOTOR_CMD_SET_BOTH,
        .target_speed = speed,
        .direction = direction,
        .accel = 0,  // 使用默认加速度
        .decel = 0   // 使用默认减速度
    };
    
    UART_Printf("Starting Motor %d: Speed=%d RPM, Dir=%d\r\n", 
               motor_id, speed, direction);
    
    return AppCANProtocol_ControlMotor(motor_id, &control);
}

/**
 * @brief  停止电机
 * @param  motor_id: 电机ID (1-20)
 * @retval true: 成功, false: 失败
 */
bool AppCANProtocol_StopMotor(uint8_t motor_id)
{
    MotorControl_t control = {
        .command = MOTOR_CMD_STOP,
        .target_speed = 0,
        .direction = MOTOR_DIR_STOP,
        .accel = 0,
        .decel = 0
    };
    
    UART_Printf("Stopping Motor %d\r\n", motor_id);
    
    return AppCANProtocol_ControlMotor(motor_id, &control);
}

/**
 * @brief  设置电机速度
 * @param  motor_id: 电机ID (1-20)
 * @param  speed: 目标转速 (RPM)
 * @retval true: 成功, false: 失败
 */
bool AppCANProtocol_SetMotorSpeed(uint8_t motor_id, uint16_t speed)
{
    MotorControl_t control = {
        .command = MOTOR_CMD_SET_SPEED,
        .target_speed = speed,
        .direction = MOTOR_DIR_CW,  // 保持当前方向
        .accel = 0,
        .decel = 0
    };
    
    UART_Printf("Setting Motor %d Speed: %d RPM\r\n", motor_id, speed);
    
    return AppCANProtocol_ControlMotor(motor_id, &control);
}

/**
 * @brief  查询电机状态
 * @param  motor_id: 电机ID (1-20)
 * @retval true: 成功, false: 失败
 */
bool AppCANProtocol_QueryMotorStatus(uint8_t motor_id)
{
    if (motor_id < 1 || motor_id > MAX_MOTORS) {
        return false;
    }
    
    uint8_t node_id = CAN_ADDR_MOTOR_BASE + (motor_id - 1);
    uint32_t can_id = CAN_ID(CAN_FUNC_MOTOR_QUERY, node_id);
    
    uint8_t data[1] = {0x00};  // 查询所有状态
    
    return AppCANProtocol_SendMessage(can_id, data, 1);
}

/**
 * @brief  获取电机状态
 * @param  motor_id: 电机ID (1-20)
 * @retval 电机状态指针，未找到返回NULL
 */
MotorStatus_t* AppCANProtocol_GetMotorStatus(uint8_t motor_id)
{
    if (motor_id < 1 || motor_id > MAX_MOTORS) {
        return NULL;
    }
    
    uint8_t index = motor_id - 1;
    
    // 检查数据是否有效（最近3秒内更新过）
    uint32_t current_time = HAL_GetTick();
    if ((current_time - g_device_manager.motors[index].last_update_time) < 3000) {
        return &g_device_manager.motors[index];
    }
    
    return NULL;
}

/**
 * @brief  紧急停止所有电机
 * @retval true: 成功, false: 失败
 */
bool AppCANProtocol_EmergencyStopAll(void)
{
    uint8_t data[2] = {0xE5, 0xE5};  // 紧急停止魔数
    uint32_t can_id = CAN_ID(CAN_FUNC_BROADCAST, CAN_ADDR_BROADCAST);
    
    UART_Printf("EMERGENCY STOP ALL!\r\n");
    
    return AppCANProtocol_SendMessage(can_id, data, 2);
}

/* ========== 传感器控制函数实现 ========== */

/**
 * @brief  查询传感器数据
 * @param  sensor_id: 传感器ID (1-20)
 * @param  sensor_type: 传感器类型
 * @retval true: 成功, false: 失败
 */
bool AppCANProtocol_QuerySensor(uint8_t sensor_id, SensorType_t sensor_type)
{
    if (sensor_id < 1 || sensor_id > MAX_SENSORS) {
        return false;
    }
    
    uint8_t node_id = CAN_ADDR_SENSOR_BASE + (sensor_id - 1);
    uint32_t can_id = CAN_ID(CAN_FUNC_SENSOR_QUERY, node_id);
    
    uint8_t data[2] = {sensor_type, 0x00};
    
    UART_Printf("Querying Sensor %d, Type: 0x%02X\r\n", sensor_id, sensor_type);
    
    return AppCANProtocol_SendMessage(can_id, data, 2);
}

/**
 * @brief  获取传感器数据
 * @param  sensor_id: 传感器ID (1-20)
 * @retval 传感器数据指针，未找到返回NULL
 */
SensorData_t* AppCANProtocol_GetSensorData(uint8_t sensor_id)
{
    if (sensor_id < 1 || sensor_id > MAX_SENSORS) {
        return NULL;
    }
    
    uint8_t index = sensor_id - 1;
    
    // 检查数据是否有效（最近5秒内更新过）
    uint32_t current_time = HAL_GetTick();
    if ((current_time - g_device_manager.sensors[index].last_update_time) < 5000) {
        return &g_device_manager.sensors[index];
    }
    
    return NULL;
}

/* ========== IO设备控制函数实现 ========== */

/**
 * @brief  控制IO端口
 * @param  io_id: IO设备ID (1-20)
 * @param  port: 端口号 (0-7)
 * @param  mask: 位掩码
 * @param  value: 输出值
 * @retval true: 成功, false: 失败
 */
bool AppCANProtocol_ControlIO(uint8_t io_id, uint8_t port, uint8_t mask, uint8_t value)
{
    if (io_id < 1 || io_id > MAX_IO_DEVICES || port > 7) {
        return false;
    }
    
    uint8_t node_id = CAN_ADDR_IO_BASE + (io_id - 1);
    uint32_t can_id = CAN_ID(CAN_FUNC_IO_CMD, node_id);
    
    uint8_t data[4] = {port, mask, value, 0x00};
    
    UART_Printf("Controlling IO %d, Port: %d, Mask: 0x%02X, Value: 0x%02X\r\n",
               io_id, port, mask, value);
    
    return AppCANProtocol_SendMessage(can_id, data, 4);
}

/**
 * @brief  获取IO状态
 * @param  io_id: IO设备ID (1-20)
 * @retval IO状态指针，未找到返回NULL
 */
IOStatus_t* AppCANProtocol_GetIOStatus(uint8_t io_id)
{
    if (io_id < 1 || io_id > MAX_IO_DEVICES) {
        return NULL;
    }
    
    uint8_t index = io_id - 1;
    
    // 检查数据是否有效（最近3秒内更新过）
    uint32_t current_time = HAL_GetTick();
    if ((current_time - g_device_manager.io_devices[index].last_update_time) < 3000) {
        return &g_device_manager.io_devices[index];
    }
    
    return NULL;
}

/* ========== 内部辅助函数实现 ========== */

/**
 * @brief  发送心跳广播
 * @retval None
 */
void AppCANProtocol_SendHeartbeat(void)
{
    uint8_t data[4];
    data[0] = 0xAA;  // 魔数
    data[1] = (g_device_manager.heartbeat_seq >> 8) & 0xFF;
    data[2] = g_device_manager.heartbeat_seq & 0xFF;
    data[3] = 0x55;  // 魔数
    
    uint32_t can_id = CAN_ID(CAN_FUNC_HEARTBEAT, CAN_ADDR_BROADCAST);
    
    if (AppCANProtocol_SendMessage(can_id, data, 4)) {
        g_device_manager.heartbeat_seq++;
    }
}

/**
 * @brief  检查节点超时
 * @retval None
 */
void AppCANProtocol_CheckNodeTimeout(void)
{
    uint32_t current_time = HAL_GetTick();
    
    for (uint8_t i = 0; i < MAX_NODES; i++) {
        if (g_device_manager.nodes[i].is_valid && 
            g_device_manager.nodes[i].status == NODE_STATUS_ONLINE) {
            
            // 检查心跳超时
            if ((current_time - g_device_manager.nodes[i].last_heartbeat_time) > HEARTBEAT_TIMEOUT) {
                g_device_manager.nodes[i].status = NODE_STATUS_OFFLINE;
                g_device_manager.node_count--;
                
                UART_Printf("Node 0x%02X timeout! Marking as OFFLINE\r\n", 
                           g_device_manager.nodes[i].node_id);
                
                // 如果是电机节点，发送紧急停止
                if (g_device_manager.nodes[i].device_type == DEV_TYPE_MOTOR) {
                    uint8_t motor_id = g_device_manager.nodes[i].node_id - CAN_ADDR_MOTOR_BASE + 1;
                    AppCANProtocol_StopMotor(motor_id);
                }
            }
        }
    }
}

/**
 * @brief  发送CAN消息
 * @param  can_id: CAN ID
 * @param  data: 数据指针
 * @param  len: 数据长度
 * @retval true: 成功, false: 失败
 */
bool AppCANProtocol_SendMessage(uint32_t can_id, uint8_t *data, uint8_t len)
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
 * @brief  计算简单校验和
 * @param  data: 数据指针
 * @param  len: 数据长度
 * @retval 校验和
 */
uint8_t AppCANProtocol_CalcChecksum(uint8_t *data, uint8_t len)
{
    uint8_t sum = 0;
    for (uint8_t i = 0; i < len; i++) {
        sum += data[i];
    }
    return sum & 0xFF;
}

/* ========== 消息处理函数实现 ========== */

/**
 * @brief  处理系统消息（设备发现响应）
 * @param  node_id: 节点ID
 * @param  data: 数据指针
 * @param  len: 数据长度
 * @retval None
 */
static void HandleSystemMessage(uint8_t node_id, uint8_t *data, uint8_t len)
{
    if (len < 6) {
        return;
    }
    
    NodeInfo_t *node = FindOrCreateNode(node_id);
    if (node == NULL) {
        return;
    }
    
    // 解析设备信息
    node->device_type = (DeviceType_t)data[0];
    node->hw_version = data[1];
    node->sw_version = (data[2] << 8) | data[3];
    node->status = (data[4] & 0x01) ? NODE_STATUS_ONLINE : NODE_STATUS_OFFLINE;
    node->capability = data[5];
    node->last_heartbeat_time = HAL_GetTick();
    
    if (node->status == NODE_STATUS_ONLINE) {
        g_device_manager.node_count++;
    }
    
    UART_Printf("Device discovered: ID=0x%02X Type=%d HW=%d.%d SW=%d.%d\r\n",
               node_id, node->device_type,
               node->hw_version >> 4, node->hw_version & 0x0F,
               node->sw_version >> 8, node->sw_version & 0xFF);
}

/**
 * @brief  处理心跳消息
 * @param  node_id: 节点ID
 * @param  data: 数据指针
 * @param  len: 数据长度
 * @retval None
 */
static void HandleHeartbeatMessage(uint8_t node_id, uint8_t *data, uint8_t len)
{
    if (len < 4) {
        return;
    }
    
    NodeInfo_t *node = AppCANProtocol_GetNodeInfo(node_id);
    if (node == NULL) {
        return;
    }
    
    // 更新心跳时间
    node->last_heartbeat_time = HAL_GetTick();
    
    // 解析状态
    uint8_t status_code = data[0];
    node->error_code = data[1];
    node->uptime = (data[2] << 8) | data[3];
    
    // 更新状态
    if (node->status == NODE_STATUS_OFFLINE && status_code == 0x00) {
        node->status = NODE_STATUS_ONLINE;
        g_device_manager.node_count++;
        UART_Printf("Node 0x%02X back online!\r\n", node_id);
    }
}

/**
 * @brief  处理电机控制响应
 * @param  node_id: 节点ID
 * @param  data: 数据指针
 * @param  len: 数据长度
 * @retval None
 */
static void HandleMotorResponse(uint8_t node_id, uint8_t *data, uint8_t len)
{
    if (len < 2) {
        return;
    }
    
    uint8_t result = data[0];
    uint8_t error = data[1];
    uint8_t motor_id = node_id - CAN_ADDR_MOTOR_BASE + 1;
    
    if (result == 0x00) {
        UART_Printf("Motor %d command SUCCESS\r\n", motor_id);
    } else {
        UART_Printf("Motor %d command FAILED: Result=0x%02X Error=0x%02X\r\n",
                   motor_id, result, error);
    }
}

/**
 * @brief  处理电机状态上报
 * @param  node_id: 节点ID
 * @param  data: 数据指针
 * @param  len: 数据长度
 * @retval None
 */
static void HandleMotorStatus(uint8_t node_id, uint8_t *data, uint8_t len)
{
    if (len < 8) {
        return;
    }
    
    uint8_t motor_id = node_id - CAN_ADDR_MOTOR_BASE + 1;
    uint8_t index = GetMotorIndex(motor_id);
    
    if (index >= MAX_MOTORS) {
        return;
    }
    
    // 解析电机状态
    MotorStatus_t *status = &g_device_manager.motors[index];
    status->status = data[0];
    status->actual_speed = (data[1] << 8) | data[2];
    status->direction = (MotorDirection_t)data[3];
    status->current = (data[4] << 8) | data[5];
    status->temperature = data[6];
    status->fault = data[7];
    status->last_update_time = HAL_GetTick();
    
    UART_Printf("Motor %d Status: Speed=%d RPM, Dir=%d, Current=%d mA, Temp=%d°C, Fault=0x%02X\r\n",
               motor_id, status->actual_speed, status->direction, 
               status->current, status->temperature, status->fault);
}

/**
 * @brief  处理传感器数据上报
 * @param  node_id: 节点ID
 * @param  data: 数据指针
 * @param  len: 数据长度
 * @retval None
 */
static void HandleSensorData(uint8_t node_id, uint8_t *data, uint8_t len)
{
    if (len < 8) {
        return;
    }
    
    uint8_t sensor_id = node_id - CAN_ADDR_SENSOR_BASE + 1;
    uint8_t index = GetSensorIndex(sensor_id);
    
    if (index >= MAX_SENSORS) {
        return;
    }
    
    // 解析传感器数据
    SensorData_t *sensor = &g_device_manager.sensors[index];
    sensor->sensor_type = (SensorType_t)data[0];
    sensor->status = data[1];
    
    // 将4字节数据解析为浮点数（简化处理，实际应用中可能需要转换）
    uint32_t raw_value = (data[2] << 24) | (data[3] << 16) | (data[4] << 8) | data[5];
    memcpy(&sensor->value, &raw_value, sizeof(float));
    
    sensor->unit = data[6];
    sensor->quality = data[7];
    sensor->last_update_time = HAL_GetTick();
    
    UART_Printf("Sensor %d Data: Type=%d, Value=%.2f, Quality=%d%%\r\n",
               sensor_id, sensor->sensor_type, sensor->value, sensor->quality);
}

/**
 * @brief  处理IO状态上报
 * @param  node_id: 节点ID
 * @param  data: 数据指针
 * @param  len: 数据长度
 * @retval None
 */
static void HandleIOStatus(uint8_t node_id, uint8_t *data, uint8_t len)
{
    if (len < 8) {
        return;
    }
    
    uint8_t io_id = node_id - CAN_ADDR_IO_BASE + 1;
    uint8_t index = GetIOIndex(io_id);
    
    if (index >= MAX_IO_DEVICES) {
        return;
    }
    
    // 解析IO状态
    IOStatus_t *io_status = &g_device_manager.io_devices[index];
    memcpy(io_status->port_status, data, 8);
    io_status->last_update_time = HAL_GetTick();
    
    UART_Printf("IO %d Status: ", io_id);
    for (uint8_t i = 0; i < 8; i++) {
        UART_Printf("P%d=0x%02X ", i, io_status->port_status[i]);
    }
    UART_Printf("\r\n");
}

/* ========== 内部查找/创建函数 ========== */

/**
 * @brief  查找或创建节点
 * @param  node_id: 节点ID
 * @retval 节点信息指针，失败返回NULL
 */
static NodeInfo_t* FindOrCreateNode(uint8_t node_id)
{
    // 先查找是否已存在
    for (uint8_t i = 0; i < MAX_NODES; i++) {
        if (g_device_manager.nodes[i].is_valid && 
            g_device_manager.nodes[i].node_id == node_id) {
            return &g_device_manager.nodes[i];
        }
    }
    
    // 创建新节点
    for (uint8_t i = 0; i < MAX_NODES; i++) {
        if (!g_device_manager.nodes[i].is_valid) {
            g_device_manager.nodes[i].node_id = node_id;
            g_device_manager.nodes[i].is_valid = true;
            g_device_manager.nodes[i].device_type = AppCANProtocol_GetDeviceType(node_id);
            return &g_device_manager.nodes[i];
        }
    }
    
    UART_Printf("ERROR: Node table full!\r\n");
    return NULL;
}

/**
 * @brief  获取电机索引
 * @param  motor_id: 电机ID (1-20)
 * @retval 索引 (0-19)
 */
static uint8_t GetMotorIndex(uint8_t motor_id)
{
    if (motor_id >= 1 && motor_id <= MAX_MOTORS) {
        return motor_id - 1;
    }
    return 0xFF;
}

/**
 * @brief  获取传感器索引
 * @param  sensor_id: 传感器ID (1-20)
 * @retval 索引 (0-19)
 */
static uint8_t GetSensorIndex(uint8_t sensor_id)
{
    if (sensor_id >= 1 && sensor_id <= MAX_SENSORS) {
        return sensor_id - 1;
    }
    return 0xFF;
}

/**
 * @brief  获取IO索引
 * @param  io_id: IO设备ID (1-20)
 * @retval 索引 (0-19)
 */
static uint8_t GetIOIndex(uint8_t io_id)
{
    if (io_id >= 1 && io_id <= MAX_IO_DEVICES) {
        return io_id - 1;
    }
    return 0xFF;
}

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
