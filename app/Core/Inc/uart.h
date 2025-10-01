/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : uart.h
  * @brief          : Header for uart.c file.
  *                   This file contains UART communication functions.
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

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __UART_H
#define __UART_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include <stdint.h>
#include <stdbool.h>

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private defines -----------------------------------------------------------*/
/* USER CODE BEGIN Private defines */

/* UART Configuration */
#define UART_BAUD_RATE          115200
#define UART_TIMEOUT_DEFAULT    1000
#define UART_RX_BUFFER_SIZE     256
#define UART_TX_BUFFER_SIZE     256
#define UART_DATA_COMPLETE_DELAY 1  // 1ms延迟确保数据完整

/* USER CODE END Private defines */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

// UART接收缓冲区结构
typedef struct {
    uint8_t buffer[UART_RX_BUFFER_SIZE];
    volatile uint16_t head;
    volatile uint16_t tail;
    volatile uint16_t count;
    volatile bool data_ready;
    volatile uint32_t last_receive_time;
} UART_RxBuffer_t;

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

/* Exported variables --------------------------------------------------------*/
extern UART_HandleTypeDef huart1;

/* Exported functions prototypes ---------------------------------------------*/

/* UART Initialization */
void MX_USART1_UART_Init(void);

/* UART Communication Functions - Raw Data Only */
bool UART_SendData(const uint8_t* data, uint16_t size);
HAL_StatusTypeDef UART_ReceiveByte(uint8_t* data, uint32_t timeout);
HAL_StatusTypeDef UART_ReceiveData(uint8_t* data, uint16_t size, uint32_t timeout);

/* UART Interrupt Functions */
void UART_StartInterruptReceive(void);
void UART_StopInterruptReceive(void);
void UART_IRQHandler(void);
bool UART_IsDataReady(void);
uint16_t UART_GetReceivedData(uint8_t* data, uint16_t max_size);
// Note: UART_ProcessReceivedData() is now called automatically in interrupt

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

#ifdef __cplusplus
}
#endif

#endif /* __UART_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/