#ifndef __MOTOR_H
#define __MOTOR_H

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32f1xx_hal.h"
#include "app_init.h"

/* Motor I2C slave address */
#define MOTOR_I2C_ADDR          0x30

/* Motor Commands */
#define CMD_MOTOR_CONTROL       0x10
#define CMD_READ_STATUS         0x40

/* Control flags */
#define MOTOR_START             0x80
#define MOTOR_STOP              0x00
#define MOTOR_FORWARD           0x40
#define MOTOR_REVERSE           0x00

/* Status flags */
#define STATUS_RUNNING          0x80
#define STATUS_STOPPED          0x00
#define STATUS_FORWARD          0x40
#define STATUS_REVERSE          0x00
#define STATUS_ERROR            0x20

/**
 * @brief  Start motor rotation with specified speed
 * @param  speed: Target speed in RPM (0-65535)
 * @param  direction: MOTOR_FORWARD or MOTOR_REVERSE
 * @retval HAL status
 */
HAL_StatusTypeDef Motor_Start(uint16_t speed, uint8_t direction);

/**
 * @brief  Stop motor
 * @retval HAL status
 */
HAL_StatusTypeDef Motor_Stop(void);

/**
 * @brief  Read motor status
 * @param  status: Pointer to store status byte
 * @param  speed: Pointer to store current speed (RPM)
 * @retval HAL status
 */
HAL_StatusTypeDef Motor_GetStatus(uint8_t* status, uint16_t* speed);

#ifdef __cplusplus
}
#endif

#endif /* __MOTOR_H */