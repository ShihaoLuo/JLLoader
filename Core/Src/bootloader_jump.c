/**
  ******************************************************************************
  * @file    bootloader_jump.c
  * @author  Generated
  * @brief   Bootloader jump functionality implementation
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "bootloader_jump.h"
#include "protocol.h"

/* Private defines -----------------------------------------------------------*/
#define NVIC_ICPR_COUNT        8

/* Private function prototypes -----------------------------------------------*/
static void ResetSystemPeripherals(void);
static void DisableInterrupts(void);
static void ResetRCC(void);

/**
  * @brief  Check if valid application exists at specified address
  * @param  app_address: Application start address
  * @retval true if valid application exists, false otherwise
  */
bool Bootloader_CheckApplication(uint32_t app_address)
{
    /* Check if the stack pointer is valid (pointing to RAM) */
    uint32_t stack_pointer = *((volatile uint32_t*)app_address);
    
    if ((stack_pointer & STACK_POINTER_MASK) == STACK_POINTER_VALID)
    {
        return true;
    }
    
    return false;
}

/**
  * @brief  Prepare system for jump (disable peripherals, reset states)
  * @retval None
  */
void Bootloader_PrepareJump(void)
{
    /* Disable all interrupts */
    DisableInterrupts();
    
    /* Reset all peripherals */
    ResetSystemPeripherals();
    
    /* Reset RCC to default state */
    ResetRCC();
}

/**
  * @brief  Jump to application at specified address
  * @param  app_address: Application start address
  * @retval None (this function should not return)
  */
void Bootloader_JumpToApplication(uint32_t app_address)
{
    uint32_t jump_address;
    pFunction jump_to_application;
    
    /* Verify application is valid before jumping */
    if (!Bootloader_CheckApplication(app_address))
    {
        return; /* Invalid application, don't jump */
    }
    
    /* Prepare system for jump */
    Bootloader_PrepareJump();
    
    /* Remap interrupt vector table to application address */
    SCB->VTOR = app_address;
    
    /* Set new stack pointer from application address */
    __set_MSP(*((volatile uint32_t*)app_address));
    
    /* Get reset handler address from application address + 4 */
    jump_address = *((volatile uint32_t*)(app_address + 4));
    
    /* Clear the LSB to ensure proper function address */
    jump_to_application = (pFunction)jump_address;
    
    /* Jump to application */
    jump_to_application();
}

/**
  * @brief  Disable all interrupts and SysTick
  * @retval None
  */
static void DisableInterrupts(void)
{
    /* Disable all interrupts */
    __disable_irq();
    
    /* Disable SysTick completely */
    SysTick->CTRL = 0;
    SysTick->LOAD = 0;
    SysTick->VAL = 0;
    
    /* Clear all pending interrupts */
    for (int i = 0; i < NVIC_ICPR_COUNT; i++)
    {
        NVIC->ICPR[i] = 0xFFFFFFFF;
    }
}

/**
  * @brief  Reset all system peripherals
  * @retval None
  */
static void ResetSystemPeripherals(void)
{
    /* Reset all peripherals including RCC configuration */
    __HAL_RCC_APB1_FORCE_RESET();
    __HAL_RCC_APB1_RELEASE_RESET();
    __HAL_RCC_APB2_FORCE_RESET();
    __HAL_RCC_APB2_RELEASE_RESET();
}

/**
  * @brief  Reset RCC to default state (HSI enabled, PLL/HSE disabled)
  * @retval None
  */
static void ResetRCC(void)
{
    /* Enable HSI */
    RCC->CR |= RCC_CR_HSION;
    
    /* Wait for HSI ready */
    while(!(RCC->CR & RCC_CR_HSIRDY));
    
    /* Reset CFGR register */
    RCC->CFGR = 0x00000000;
    
    /* Disable HSE, CSS, PLL */
    RCC->CR &= ~(RCC_CR_HSEON | RCC_CR_CSSON | RCC_CR_PLLON);
}