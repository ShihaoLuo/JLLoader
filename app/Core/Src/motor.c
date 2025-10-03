/**
  ******************************************************************************
  * @file    motor.c
  * @brief   Motor control via I2C
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "motor.h"

/* Private variables ---------------------------------------------------------*/

/* Private function prototypes -----------------------------------------------*/

/* Exported functions -------------------------------------------------------*/

/**
 * @brief  Start motor rotation with specified speed
 * @param  speed: Target speed in RPM (0-65535)
 * @param  direction: MOTOR_FORWARD or MOTOR_REVERSE
 * @retval HAL status
 */
HAL_StatusTypeDef Motor_Start(uint16_t speed, uint8_t direction)
{
    uint8_t data[3];
    HAL_StatusTypeDef status;

    /* Prepare command data */
    data[0] = CMD_MOTOR_CONTROL;  // Command byte
    data[1] = MOTOR_START | direction;  // Control flags: Start + Direction
    data[2] = (uint8_t)(speed >> 8);   // Speed high byte
    data[3] = (uint8_t)(speed);        // Speed low byte

    /* Send command via I2C */
    status = HAL_I2C_Master_Transmit(&hi2c2, (MOTOR_I2C_ADDR << 1), data, 4, HAL_MAX_DELAY);
    
    return status;
}

/**
 * @brief  Stop motor
 * @retval HAL status
 */
HAL_StatusTypeDef Motor_Stop(void)
{
    uint8_t data[2];
    HAL_StatusTypeDef status;

    /* Prepare command data */
    data[0] = CMD_MOTOR_CONTROL;  // Command byte
    data[1] = MOTOR_STOP;         // Control flags: Stop

    /* Send command via I2C */
    status = HAL_I2C_Master_Transmit(&hi2c2, (MOTOR_I2C_ADDR << 1), data, 2, HAL_MAX_DELAY);
    
    return status;
}

/**
 * @brief  Read motor status
 * @param  status: Pointer to store status byte
 * @param  speed: Pointer to store current speed (RPM)
 * @retval HAL status
 */
HAL_StatusTypeDef Motor_GetStatus(uint8_t* status, uint16_t* speed)
{
    uint8_t cmd = CMD_READ_STATUS;
    uint8_t data[3];
    HAL_StatusTypeDef ret;

    if (status == NULL || speed == NULL)
    {
        return HAL_ERROR;
    }

    /* Send read status command */
    ret = HAL_I2C_Master_Transmit(&hi2c2, (MOTOR_I2C_ADDR << 1), &cmd, 1, HAL_MAX_DELAY);
    if (ret != HAL_OK)
    {
        return ret;
    }

    /* Receive status data */
    ret = HAL_I2C_Master_Receive(&hi2c2, (MOTOR_I2C_ADDR << 1), data, 3, HAL_MAX_DELAY);
    if (ret != HAL_OK)
    {
        return ret;
    }

    /* Store received data */
    *status = data[0];
    *speed = ((uint16_t)data[1] << 8) | data[2];

    return HAL_OK;
}