/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : flash_config.h
  * @brief          : Flash memory configuration for bootloader.
  *                   This file defines memory layout and size constraints.
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
#ifndef __FLASH_CONFIG_H
#define __FLASH_CONFIG_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f1xx_hal.h"
#include <stdint.h>

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

/* Flash Memory Layout Configuration -----------------------------------------*/

/* STM32F103C8T6 Flash Memory Map:
 * Total Flash: 64KB (0x08000000 - 0x0800FFFF)
 * 
 * Bootloader Layout:
 * - Bootloader Code: 0x08000000 - 0x08003FFF (16KB)
 * - Reserved Area:   0x08004000 - 0x0800FFFF (48KB for application)
 */

/* Bootloader Memory Configuration */
#define FLASH_BASE_ADDRESS              0x08000000UL    /*!< Flash base address */
#define BOOTLOADER_START_ADDRESS        0x08000000UL    /*!< Bootloader start address */
#define BOOTLOADER_SIZE                 0x4000UL        /*!< Bootloader size: 16KB */
#define BOOTLOADER_END_ADDRESS          (BOOTLOADER_START_ADDRESS + BOOTLOADER_SIZE - 1)

/* Application Memory Configuration */
#define APPLICATION_START_ADDRESS       (BOOTLOADER_START_ADDRESS + BOOTLOADER_SIZE)
#define APPLICATION_MAX_SIZE            (FLASH_SIZE - BOOTLOADER_SIZE)
#define APPLICATION_END_ADDRESS         (FLASH_BASE_ADDRESS + FLASH_SIZE - 1)

/* Memory Size Definitions */
#define FLASH_SIZE                      0x10000UL       /*!< Total Flash size: 64KB */
#define BOOTLOADER_SIZE_KB              (BOOTLOADER_SIZE / 1024)
#define APPLICATION_SIZE_KB             (APPLICATION_MAX_SIZE / 1024)

/* Flash Page Configuration (STM32F103C8T6) */
/* Note: FLASH_PAGE_SIZE is already defined in stm32f1xx_hal_flash_ex.h as 0x400U */
#define BOOTLOADER_PAGE_COUNT           (BOOTLOADER_SIZE / FLASH_PAGE_SIZE)
#define APPLICATION_PAGE_COUNT          (APPLICATION_MAX_SIZE / FLASH_PAGE_SIZE)

/* Bootloader Version and Info */
#define BOOTLOADER_VERSION_MAJOR        1
#define BOOTLOADER_VERSION_MINOR        0
#define BOOTLOADER_VERSION_PATCH        0
#define BOOTLOADER_BUILD_DATE           __DATE__
#define BOOTLOADER_BUILD_TIME           __TIME__

/* Exported functions prototypes ---------------------------------------------*/

/* Memory Layout Information Functions */
uint32_t Flash_GetBootloaderStartAddress(void);
uint32_t Flash_GetBootloaderEndAddress(void);
uint32_t Flash_GetBootloaderSize(void);
uint32_t Flash_GetApplicationStartAddress(void);
uint32_t Flash_GetApplicationMaxSize(void);

/* Flash Operation Functions */
HAL_StatusTypeDef Flash_ErasePages(uint32_t start_address, uint16_t page_count);
HAL_StatusTypeDef Flash_WriteData(uint32_t address, const uint8_t* data, uint8_t length);

/* Address Validation Functions */
uint8_t Flash_IsBootloaderAddress(uint32_t address);
uint8_t Flash_IsApplicationAddress(uint32_t address);
uint8_t Flash_IsValidFlashAddress(uint32_t address);

/* Size Validation Functions */
uint8_t Flash_IsBootloaderSizeValid(uint32_t size);
uint8_t Flash_CheckBootloaderConstraints(void);
uint8_t Flash_EraseApplicationPages(void);
uint8_t Flash_ErasePages(uint32_t start_address, uint16_t page_count);

#ifdef __cplusplus
}
#endif

#endif /* __FLASH_CONFIG_H */