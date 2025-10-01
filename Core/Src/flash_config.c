/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : flash_config.c
  * @brief          : Flash memory configuration implementation for bootloader.
  *                   This file implements memory layout and validation functions.
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
#include "flash_config.h"

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

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  Get bootloader start address
  * @retval Bootloader start address
  */
uint32_t Flash_GetBootloaderStartAddress(void)
{
  return BOOTLOADER_START_ADDRESS;
}

/**
  * @brief  Get bootloader end address
  * @retval Bootloader end address
  */
uint32_t Flash_GetBootloaderEndAddress(void)
{
  return BOOTLOADER_END_ADDRESS;
}

/**
  * @brief  Get bootloader size in bytes
  * @retval Bootloader size
  */
uint32_t Flash_GetBootloaderSize(void)
{
  return BOOTLOADER_SIZE;
}

/**
  * @brief  Get application start address
  * @retval Application start address
  */
uint32_t Flash_GetApplicationStartAddress(void)
{
  return APPLICATION_START_ADDRESS;
}

/**
  * @brief  Get maximum application size in bytes
  * @retval Maximum application size
  */
uint32_t Flash_GetApplicationMaxSize(void)
{
  return APPLICATION_MAX_SIZE;
}

/**
  * @brief  Check if address is within bootloader range
  * @param  address: Address to check
  * @retval 1 if address is in bootloader range, 0 otherwise
  */
uint8_t Flash_IsBootloaderAddress(uint32_t address)
{
  return (address >= BOOTLOADER_START_ADDRESS && address <= BOOTLOADER_END_ADDRESS) ? 1 : 0;
}

/**
  * @brief  Check if address is within application range
  * @param  address: Address to check
  * @retval 1 if address is in application range, 0 otherwise
  */
uint8_t Flash_IsApplicationAddress(uint32_t address)
{
  return (address >= APPLICATION_START_ADDRESS && address <= APPLICATION_END_ADDRESS) ? 1 : 0;
}

/**
  * @brief  Check if address is valid Flash address
  * @param  address: Address to check
  * @retval 1 if address is valid, 0 otherwise
  */
uint8_t Flash_IsValidFlashAddress(uint32_t address)
{
  return (address >= FLASH_BASE_ADDRESS && address <= APPLICATION_END_ADDRESS) ? 1 : 0;
}

/**
  * @brief  Check if bootloader size is within limits
  * @param  size: Size to check
  * @retval 1 if size is valid, 0 otherwise
  */
uint8_t Flash_IsBootloaderSizeValid(uint32_t size)
{
  return (size <= BOOTLOADER_SIZE) ? 1 : 0;
}

/**
  * @brief  Check bootloader memory constraints
  * @retval 1 if all constraints are satisfied, 0 otherwise
  */
uint8_t Flash_CheckBootloaderConstraints(void)
{
  /* Check bootloader start address */
  if (BOOTLOADER_START_ADDRESS != FLASH_BASE_ADDRESS)
  {
    return 0;
  }
  
  /* Check bootloader size */
  if (BOOTLOADER_SIZE > 0x2800)  /* 10KB */
  {
    return 0;
  }
  
  /* Check address alignment */
  if ((BOOTLOADER_START_ADDRESS % FLASH_PAGE_SIZE) != 0)
  {
    return 0;
  }
  
  /* Check application start address */
  if (APPLICATION_START_ADDRESS <= BOOTLOADER_END_ADDRESS)
  {
    return 0;
  }
  
  return 1;
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */