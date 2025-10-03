/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
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
#include "main.h"
#include "init.h"
#include "uart.h"
#include "flash_config.h"
#include "bootloader_check.h"
#include "protocol.h"
#include "bootloader_jump.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdint.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
  /* USER CODE BEGIN PD */
#define BOOTLOADER_TIMEOUT_MS  2000  // 500ms timeout for firmware update check
/* USER CODE END PD *//* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
static uint32_t bootloader_start_time = 0;
static uint32_t led_last_toggle_time = 0;
/* USER CODE END PV */

/* Global variables ---------------------------------------------------------*/
bool firmware_update_requested = false;  // Flag to indicate if firmware update command received

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/* External variables --------------------------------------------------------*/
extern TIM_HandleTypeDef htim4;

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_USART1_UART_Init();
  MX_TIM4_Init();

  /* USER CODE BEGIN 2 */
  
  // Only start timeout counter if we're in bootloader mode
  if (Protocol_GetRunningMode() == MODE_BOOTLOADER) {
    bootloader_start_time = HAL_GetTick();
  } else {
    // If we're in application mode, disable the timeout
    bootloader_start_time = 0xFFFFFFFF;
  }
  
  // Turn off LED (PC13 is active-low, so SET turns it off)
  LED_TurnOff();
  
  // Initialize protocol module
  Protocol_Init();
  
  // Start TIM4 for timebase
  HAL_TIM_Base_Start(&htim4);
  
  // Start UART interrupt reception
  UART_StartInterruptReceive();
  
  // 强制重新配置关键中断（解决跳转后中断失效问题）
  __enable_irq();  // 确保全局中断开启
  
  // Record bootloader start time
  bootloader_start_time = HAL_GetTick();
  
  // // Send system ready status
  // Protocol_SendStatus(STATUS_READY);
  
  // // Send initial system and memory information
  // Protocol_SendSystemInfo();
  // Protocol_SendMemoryInfo();
  
  // Verify bootloader constraints
  if (!Flash_CheckBootloaderConstraints())
  {
    Protocol_SendStatus(MEM_CONSTRAINT_ERR);
  }

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    uint32_t current_time = HAL_GetTick();
    
    // Check if bootloader timeout has elapsed (500ms) and no firmware update is requested
    if (!firmware_update_requested && (current_time - bootloader_start_time) >= BOOTLOADER_TIMEOUT_MS)
    {
        // Check if application is valid before jumping
        if (Bootloader_CheckApplication(APPLICATION_ADDRESS))
        {
            // Jump to application using the modular function
            Bootloader_JumpToApplication(APPLICATION_ADDRESS);
        }
    }
    
    // Check for pending jump requests from serial protocol
    Protocol_CheckPendingJump();
    
    // Process any incoming protocol commands
    // Add your protocol handling here if needed
    
    // Small delay to prevent excessive CPU usage
    HAL_Delay(100);
    
    /* USER CODE END WHILE */
  }
  /* USER CODE END 3 */
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

