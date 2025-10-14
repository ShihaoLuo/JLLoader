/**
  ******************************************************************************
  * @file    can_protocol_common.c
  * @brief   CAN协议通用工具函数实现
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "can_protocol.h"

/* Public functions ----------------------------------------------------------*/

/**
 * @brief  计算校验和
 * @param  data: 数据指针
 * @param  len: 数据长度
 * @retval uint8_t 校验和
 */
uint8_t CANProtocol_Checksum(uint8_t* data, uint8_t len)
{
    uint8_t sum = 0;
    for (uint8_t i = 0; i < len; i++)
    {
        sum += data[i];
    }
    return sum;
}

/**
 * @brief  验证校验和
 * @param  data: 数据指针
 * @param  len: 数据长度（包含校验和）
 * @retval true=正确, false=错误
 */
bool CANProtocol_VerifyChecksum(uint8_t* data, uint8_t len)
{
    if (len < 2)
    {
        return false;
    }
    
    uint8_t calculated = CANProtocol_Checksum(data, len - 1);
    return (calculated == data[len - 1]);
}

/**
 * @brief  根据节点ID获取设备类型
 * @param  node_id: 节点ID
 * @retval DeviceType_t 设备类型
 */
DeviceType_t CANProtocol_GetDeviceTypeFromID(uint8_t node_id)
{
    if (IS_MOTOR_ID(node_id))
    {
        return DEV_TYPE_MOTOR;
    }
    else if (IS_SENSOR_ID(node_id))
    {
        return DEV_TYPE_SENSOR;
    }
    else if (IS_IO_ID(node_id))
    {
        return DEV_TYPE_IO;
    }
    else
    {
        return DEV_TYPE_UNKNOWN;
    }
}

/**
 * @brief  获取电机的节点ID
 * @param  motor_num: 电机编号 (1-20)
 * @retval uint8_t 节点ID
 */
uint8_t CANProtocol_GetMotorID(uint8_t motor_num)
{
    if (motor_num >= 1 && motor_num <= CAN_MAX_MOTORS)
    {
        return MOTOR_ID(motor_num);
    }
    return 0;
}

/**
 * @brief  获取传感器的节点ID
 * @param  sensor_num: 传感器编号 (1-20)
 * @retval uint8_t 节点ID
 */
uint8_t CANProtocol_GetSensorID(uint8_t sensor_num)
{
    if (sensor_num >= 1 && sensor_num <= CAN_MAX_SENSORS)
    {
        return SENSOR_ID(sensor_num);
    }
    return 0;
}

/**
 * @brief  获取IO设备的节点ID
 * @param  io_num: IO编号 (1-20)
 * @retval uint8_t 节点ID
 */
uint8_t CANProtocol_GetIOID(uint8_t io_num)
{
    if (io_num >= 1 && io_num <= CAN_MAX_IOS)
    {
        return IO_ID(io_num);
    }
    return 0;
}

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
