/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : build_check.c
  * @brief          : Build verification file for Keil project configuration.
  *                   This file helps verify all includes and dependencies.
  ******************************************************************************
  * @attention
  *
  * This file is used only for build verification and can be excluded from
  * the final build if needed.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Build Verification Includes -----------------------------------------------*/
#include "main.h"
#include "init.h"
#include "uart.h"
#include "flash_config.h"
#include "bootloader_check.h"
#include "protocol.h"

/* STM32 HAL Includes for verification */
#include "stm32f1xx_hal.h"
#include "stm32f1xx_hal_uart.h"
#include "stm32f1xx_hal_tim.h"
#include "stm32f1xx_hal_gpio.h"
#include "stm32f1xx_hal_rcc.h"
#include "stm32f1xx_hal_flash.h"

/* C Standard Library Includes */
#include <string.h>
#include <stdio.h>
#include <stdarg.h>
#include <stdint.h>

/**
  * @brief  Build verification function
  * @note   This function tests compilation of all modules
  * @retval Build status (always returns 1 if compilation succeeds)
  */
uint8_t Build_VerifyConfiguration(void)
{
    /* Test function pointer assignments to verify all functions exist */
    void (*test_uart_init)(void) = MX_USART1_UART_Init;
    void (*test_gpio_init)(void) = MX_GPIO_Init;
    void (*test_tim_init)(void) = MX_TIM4_Init;
    void (*test_clock_config)(void) = SystemClock_Config;
    
    /* Test UART function pointers */
    bool (*test_uart_send)(const uint8_t*, uint16_t) = UART_SendData;
    HAL_StatusTypeDef (*test_uart_recv)(uint8_t*, uint32_t) = UART_ReceiveByte;
    
    /* Test Flash configuration functions */
    uint32_t (*test_flash_start)(void) = Flash_GetBootloaderStartAddress;
    uint8_t (*test_flash_check)(void) = Flash_CheckBootloaderConstraints;
    
    /* Prevent compiler warnings for unused variables */
    (void)test_uart_init;
    (void)test_gpio_init;
    (void)test_tim_init;
    (void)test_clock_config;
    (void)test_uart_send;
    (void)test_uart_recv;
    (void)test_flash_start;
    (void)test_flash_check;
    
    /* Test compile-time constants */
    #if defined(BOOTLOADER_START_ADDRESS) && defined(BOOTLOADER_SIZE)
        /* Bootloader configuration constants are defined */
    #else
        #error "Bootloader configuration constants not properly defined!"
    #endif
    
    #if defined(UART_BAUD_RATE) && defined(UART_TIMEOUT_DEFAULT)
        /* UART configuration constants are defined */
    #else
        #error "UART configuration constants not properly defined!"
    #endif
    
    return 1; /* Build verification passed */
}

/**
  * @brief  Print build configuration summary
  * @retval None
  */
void Build_PrintConfiguration(void)
{
    /* This function can be called to print build info */
    /* Implementation would use UART_Printf if needed */
}

/* Build verification passed if this file compiles successfully */