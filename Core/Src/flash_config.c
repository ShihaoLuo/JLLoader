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
#include "stm32f1xx_hal_flash.h"
#include "stm32f1xx_hal_flash_ex.h"

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
  if (BOOTLOADER_SIZE > 0x4000)  /* 10KB */
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

/**
 * @brief Erase a specified number of pages starting from a given address.
 * @param start_address The starting address of the first page to erase. Must be page-aligned.
 * @param page_count The number of pages to erase.
 * @return HAL_StatusTypeDef: HAL_OK on success, HAL_ERROR or HAL_BUSY on failure.
 */
HAL_StatusTypeDef Flash_ErasePages(uint32_t start_address, uint16_t page_count)
{
    if (page_count == 0) {
        return HAL_OK; // Nothing to erase is considered a success.
    }

    // Security check: Do not allow erasing the bootloader itself or outside the application area.
    uint32_t end_erase_addr = start_address + page_count * FLASH_PAGE_SIZE - 1;
    if (start_address < APPLICATION_START_ADDRESS || end_erase_addr > APPLICATION_END_ADDRESS) {
        return HAL_ERROR; // Address range is outside the application area.
    }

    // Ensure the start address is page-aligned.
    if ((start_address % FLASH_PAGE_SIZE) != 0) {
        return HAL_ERROR; // Address is not page-aligned.
    }

    HAL_StatusTypeDef status;

    // Unlock the Flash to enable the flash control register access.
    if (HAL_FLASH_Unlock() != HAL_OK) {
        return HAL_ERROR;
    }

    // Clear any pending flags before the operation.
    __HAL_FLASH_CLEAR_FLAG(FLASH_FLAG_EOP | FLASH_FLAG_WRPERR | FLASH_FLAG_PGERR);

    FLASH_EraseInitTypeDef EraseInitStruct;
    uint32_t PageError = 0;

    EraseInitStruct.TypeErase   = FLASH_TYPEERASE_PAGES;
    EraseInitStruct.PageAddress = start_address;
    EraseInitStruct.NbPages     = page_count;

    // Perform the erase operation.
    status = HAL_FLASHEx_Erase(&EraseInitStruct, &PageError);

    // Lock the Flash to disable the flash control register access.
    // It's important to lock the flash even if the erase operation fails.
    HAL_FLASH_Lock();

    return status;
}