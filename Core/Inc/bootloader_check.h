/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : bootloader_check.h
  * @brief          : Compile-time checks for bootloader constraints.
  *                   This file ensures bootloader meets size and address requirements.
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
#ifndef __BOOTLOADER_CHECK_H
#define __BOOTLOADER_CHECK_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "flash_config.h"

/* Compile-time Size and Address Checks -------------------------------------*/

/* Check bootloader size constraint */
#if (BOOTLOADER_SIZE > 0x4000)
    #error "ERROR: Bootloader size exceeds 16KB (0x4000 bytes) limit!"
    #error "Current size would be larger than allowed constraint."
    #error "Please reduce code size or increase size limit."
#endif

/* Check Flash base address */
#if (BOOTLOADER_START_ADDRESS != 0x08000000UL)
    #error "ERROR: Bootloader must start at Flash base address 0x08000000!"
    #error "Current start address does not match STM32F103C8T6 Flash base."
#endif

/* Check address alignment */
#if ((BOOTLOADER_START_ADDRESS % FLASH_PAGE_SIZE) != 0)
    #error "ERROR: Bootloader start address must be page-aligned!"
    #error "STM32F103C8T6 Flash page size is 1KB (0x400 bytes)."
#endif

/* Check application start address */
#if (APPLICATION_START_ADDRESS <= BOOTLOADER_END_ADDRESS)
    #error "ERROR: Application start address overlaps with bootloader!"
    #error "There must be no overlap between bootloader and application memory."
#endif

/* Check total Flash size */
#if (FLASH_SIZE != 0x10000UL)
    #warning "WARNING: Flash size may not match STM32F103C8T6 specification!"
    #warning "Expected Flash size is 64KB (0x10000 bytes)."
#endif

/* Runtime Memory Layout Validation -----------------------------------------*/

/**
  * @brief  Validate memory layout at compile time
  * @note   This function should be called during initialization
  * @retval 1 if all checks pass, 0 otherwise
  */
static inline uint8_t Bootloader_ValidateMemoryLayout(void)
{
    /* All compile-time checks passed if we reach here */
    return 1;
}

/**
  * @brief  Print bootloader constraints summary
  * @retval None
  */
void Bootloader_PrintConstraints(void);

/* Exported macros -----------------------------------------------------------*/

/* Memory layout information macros */
#define BOOTLOADER_SIZE_KB_STR      "16"
#define BOOTLOADER_START_ADDR_STR   "0x08000000"
#define BOOTLOADER_END_ADDR_STR     "0x08004000"
#define APPLICATION_START_ADDR_STR  "0x08004000"

/* Version information */
#define BOOTLOADER_CONSTRAINT_VERSION "1.0"

#ifdef __cplusplus
}
#endif

#endif /* __BOOTLOADER_CHECK_H */