/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : motor_init.h
  * @brief          : Motor initialization functions header
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

#ifndef __MOTOR_INIT_H__
#define __MOTOR_INIT_H__

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Exported types ------------------------------------------------------------*/

/* Exported constants --------------------------------------------------------*/

/* Exported macro ------------------------------------------------------------*/

/* Exported variables --------------------------------------------------------*/
extern CAN_HandleTypeDef hcan1;

/* Exported functions prototypes ---------------------------------------------*/
void SystemClock_Config(void);
void MX_GPIO_Init(void);
void MX_TIM4_Init(void);
void MX_TIM3_Init(void);
void Motor_System_Init(void);
void Motor_CAN_Init(void);
void Motor_CAN_ReceiveTask(void);

#ifdef __cplusplus
}
#endif

#endif /* __MOTOR_INIT_H__ */