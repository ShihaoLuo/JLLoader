/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : uart.c
  * @brief          : UART communication functions for the application project.
  *                   This file contains all UART related initialization and communication.
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

/* Includes ------------------------------------------------------------------*/
#include "uart.h"
#include "main.h"
#include "protocol.h"
#include "stm32f1xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
UART_HandleTypeDef huart1;

/* USER CODE BEGIN PV */
// 中断接收相关变量
static UART_RxBuffer_t rx_buffer = {0};
static uint8_t rx_byte = 0; // 单字节接收缓冲区
static volatile bool interrupt_enabled = false;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN PFP */
static void UART_AddByteToBuffer(uint8_t byte);
static void UART_ProcessCompleteData(void);
static void UART_ProcessReceivedData(void);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
void MX_USART1_UART_Init(void)
{
  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = UART_BAUD_RATE;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */
}

/**
  * @brief  Send data buffer via UART1
  * @param  data: pointer to data buffer
  * @param  size: size of data to send
  * @retval true if successful, false otherwise
  */
bool UART_SendData(const uint8_t* data, uint16_t size)
{
  if (data != NULL && size > 0)
  {
    HAL_StatusTypeDef status = HAL_UART_Transmit(&huart1, (uint8_t*)data, size, UART_TIMEOUT_DEFAULT);
    return (status == HAL_OK);
  }
  return false;
}

/**
  * @brief  Receive byte via UART1 with timeout
  * @param  data: pointer to received data
  * @param  timeout: timeout in ms
  * @retval HAL status
  */
HAL_StatusTypeDef UART_ReceiveByte(uint8_t* data, uint32_t timeout)
{
  if (data != NULL)
  {
    return HAL_UART_Receive(&huart1, data, 1, timeout);
  }
  return HAL_ERROR;
}

/**
  * @brief  Receive data buffer via UART1 with timeout
  * @param  data: pointer to data buffer
  * @param  size: size of data to receive
  * @param  timeout: timeout in ms
  * @retval HAL status
  */
HAL_StatusTypeDef UART_ReceiveData(uint8_t* data, uint16_t size, uint32_t timeout)
{
  if (data != NULL && size > 0)
  {
    return HAL_UART_Receive(&huart1, data, size, timeout);
  }
  return HAL_ERROR;
}

/* USER CODE BEGIN 4 */

/**
  * @brief  添加接收到的字节到缓冲区
  * @param  byte: 接收到的字节
  * @retval None
  */
static void UART_AddByteToBuffer(uint8_t byte)
{
  // 添加字节到环形缓冲区
  rx_buffer.buffer[rx_buffer.head] = byte;
  rx_buffer.head = (rx_buffer.head + 1) % UART_RX_BUFFER_SIZE;
  
  // 如果缓冲区满了，移动tail指针（丢弃最旧的数据）
  if (rx_buffer.count >= UART_RX_BUFFER_SIZE) {
    rx_buffer.tail = (rx_buffer.tail + 1) % UART_RX_BUFFER_SIZE;
  } else {
    rx_buffer.count++;
  }
  
  // 记录最后接收时间
  rx_buffer.last_receive_time = HAL_GetTick();
  rx_buffer.data_ready = true;
}

/**
  * @brief  处理接收完成的数据（智能版本：保留不完整数据）
  * @param  None
  * @retval None
  */
static void UART_ProcessCompleteData(void)
{
  if (rx_buffer.count > 0) {
    uint8_t temp_buffer[UART_RX_BUFFER_SIZE];
    uint16_t data_length = 0;
    
    // 复制数据到临时缓冲区（不立即修改原缓冲区）
    uint16_t temp_tail = rx_buffer.tail;
    uint16_t temp_count = rx_buffer.count;
    while (temp_count > 0) {
      temp_buffer[data_length++] = rx_buffer.buffer[temp_tail];
      temp_tail = (temp_tail + 1) % UART_RX_BUFFER_SIZE;
      temp_count--;
    }
    
    // 尝试处理数据
    if (data_length > 0) {
      bool processed = Protocol_ProcessReceivedData(temp_buffer, data_length);
      
      if (processed) {
        // 成功处理了至少一帧，清空缓冲区
        rx_buffer.tail = rx_buffer.head;
        rx_buffer.count = 0;
        rx_buffer.data_ready = false;
      }
      // 如果没有处理任何完整帧，保留数据在缓冲区中
    }
  }
}

/**
  * @brief  启动UART中断接收
  * @param  None
  * @retval None
  */
void UART_StartInterruptReceive(void)
{
  if (!interrupt_enabled) {
    // 清空接收缓冲区 - 直接赋值初始化，无需string.h
    rx_buffer.head = 0;
    rx_buffer.tail = 0;
    rx_buffer.count = 0;
    rx_buffer.data_ready = false;
    rx_buffer.last_receive_time = 0;
    
    // 启动中断接收
    HAL_UART_Receive_IT(&huart1, &rx_byte, 1);
    interrupt_enabled = true;
  }
}

/**
  * @brief  停止UART中断接收
  * @param  None
  * @retval None
  */
void UART_StopInterruptReceive(void)
{
  if (interrupt_enabled) {
    HAL_UART_AbortReceive_IT(&huart1);
    interrupt_enabled = false;
  }
}

/**
  * @brief  UART中断回调函数
  * @param  None
  * @retval None
  */
void UART_IRQHandler(void)
{
  if (interrupt_enabled) {
    // 将接收到的字节添加到缓冲区
    UART_AddByteToBuffer(rx_byte);
    
    // 重新启动下一字节的接收
    HAL_UART_Receive_IT(&huart1, &rx_byte, 1);
    
    // 每收到一个字节都尝试处理，让协议解析器自己判断数据完整性
    UART_ProcessReceivedData();
  }
}

/**
  * @brief  检查是否有数据准备好处理
  * @param  None
  * @retval true if data ready, false otherwise
  */
bool UART_IsDataReady(void)
{
  // 简化：只要有数据就认为准备好了，不等1ms
  return (rx_buffer.data_ready && rx_buffer.count > 0);
}

/**
  * @brief  获取接收到的数据
  * @param  data: 输出缓冲区
  * @param  max_size: 最大缓冲区大小
  * @retval 实际读取的数据长度
  */
uint16_t UART_GetReceivedData(uint8_t* data, uint16_t max_size)
{
  if (data == NULL || max_size == 0) {
    return 0;
  }
  
  uint16_t copied = 0;
  
  // 从环形缓冲区复制数据
  while (rx_buffer.count > 0 && copied < max_size) {
    data[copied++] = rx_buffer.buffer[rx_buffer.tail];
    rx_buffer.tail = (rx_buffer.tail + 1) % UART_RX_BUFFER_SIZE;
    rx_buffer.count--;
  }
  
  if (rx_buffer.count == 0) {
    rx_buffer.data_ready = false;
  }
  
  return copied;
}

/**
  * @brief  处理接收到的数据（检查并立即处理数据，改进版）
  * @param  None
  * @retval None
  */
static void UART_ProcessReceivedData(void)
{
  // 只有当有足够数据时才尝试处理
  if (rx_buffer.count >= 5 && UART_IsDataReady()) {
    UART_ProcessCompleteData();
  }
  
  // 超时保护：如果数据在缓冲区停留超过100ms，强制清空
  if (rx_buffer.count > 0 && 
      (HAL_GetTick() - rx_buffer.last_receive_time) > 100) {
    rx_buffer.tail = rx_buffer.head;
    rx_buffer.count = 0;
    rx_buffer.data_ready = false;
  }
}

/**
  * @brief  HAL UART接收完成回调函数
  * @param  huart: UART句柄指针
  * @retval None
  */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
  if (huart->Instance == USART1) {
    UART_IRQHandler();
  }
}

/* USER CODE END 4 */