/**
  ******************************************************************************
  * @file    app_init.c
  * @author  Generated
  * @brief   Application initialization functions
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "app_init.h"

/* Private variables ---------------------------------------------------------*/
static GPIO_PinState led_state = GPIO_PIN_SET;  // 初始状态：熄灭

/* Private function prototypes -----------------------------------------------*/

/* Exported functions --------------------------------------------------------*/

/**
 * @brief  完整的应用程序初始化
 * @retval None
 */
void App_Init(void)
{
    /* 初始化HAL库 */
    HAL_Init();

    /* 配置系统时钟 */
    SystemClock_Config();
    
    /* 重新配置SysTick */
    App_SysTick_Reconfig();
    
    /* 全局使能中断 */
    __enable_irq();

    /* 初始化GPIO */
    App_GPIO_Init();

    /* 显示启动指示 */
    App_LED_StartupIndication();
}

/**
 * @brief  系统时钟配置（HSE 8MHz + PLL = 72MHz）
 * @retval None
 */
void SystemClock_Config(void)
{
    RCC_OscInitTypeDef RCC_OscInitStruct = {0};
    RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

    /* 配置振荡器：启用HSE和PLL */
    RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE | RCC_OSCILLATORTYPE_HSI;
    RCC_OscInitStruct.HSEState = RCC_HSE_ON;           // 启用外部8MHz晶振
    RCC_OscInitStruct.HSIState = RCC_HSI_ON;           // 保持HSI开启作为备用
    RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;       // 启用PLL
    RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE; // PLL时钟源选择HSE
    RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;       // PLL倍频系数: 8MHz * 9 = 72MHz
    
    if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
    {
        Error_Handler();
    }

    /* 配置系统时钟 */
    RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK |
                                  RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
    RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
    RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
    RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
    RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

    if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
    {
        Error_Handler();
    }
}

/**
 * @brief  SysTick重新配置函数
 * @retval None
 */
void App_SysTick_Reconfig(void)
{
    /* 重新配置SysTick - 在SystemClock_Config后进行 */
    /* 计算SysTick重载值: 72MHz / 1000 - 1 = 71999 (1ms中断) */
    uint32_t systick_reload = 72000 - 1;  
    
    /* 直接配置SysTick寄存器 */
    *((volatile uint32_t*)0xE000E014) = systick_reload;     /* SysTick->LOAD */
    *((volatile uint32_t*)0xE000E018) = 0;                  /* SysTick->VAL = 0 */
    *((volatile uint32_t*)0xE000E010) = 0x07;               /* SysTick->CTRL = CLKSOURCE|TICKINT|ENABLE */
}

/**
 * @brief  GPIO初始化函数
 * @retval None
 */
void App_GPIO_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};

    /* 使能GPIOC时钟 */
    __HAL_RCC_GPIOC_CLK_ENABLE();

    /* 配置LED初始状态为熄灭 */
    HAL_GPIO_WritePin(LED_PORT, LED_PIN, GPIO_PIN_SET);

    /* 配置GPIO PC13 */
    GPIO_InitStruct.Pin = LED_PIN;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;      // 推挽输出
    GPIO_InitStruct.Pull = GPIO_NOPULL;              // 无上下拉
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;     // 低速

    HAL_GPIO_Init(LED_PORT, &GPIO_InitStruct);
}

/**
 * @brief  LED启动指示
 * @retval None
 */
void App_LED_StartupIndication(void)
{
    /* 立即点亮LED表示APP已成功启动 */
    HAL_GPIO_WritePin(LED_PORT, LED_PIN, GPIO_PIN_RESET);  // 点亮LED (低电平有效)
    led_state = GPIO_PIN_RESET;
}

/**
 * @brief  切换LED状态
 * @retval None
 */
void App_LED_Toggle(void)
{
    if (led_state == GPIO_PIN_SET)
    {
        /* 点亮LED (STM32F103C8T6板载LED低电平点亮) */
        HAL_GPIO_WritePin(LED_PORT, LED_PIN, GPIO_PIN_RESET);
        led_state = GPIO_PIN_RESET;
    }
    else
    {
        /* 熄灭LED */
        HAL_GPIO_WritePin(LED_PORT, LED_PIN, GPIO_PIN_SET);
        led_state = GPIO_PIN_SET;
    }
}

/**
 * @brief  获取LED状态
 * @retval GPIO_PinState LED当前状态
 */
GPIO_PinState App_LED_GetState(void)
{
    return led_state;
}

/**
 * @brief  错误处理函数
 * @retval None
 */
void Error_Handler(void)
{
    /* 禁用中断 */
    __disable_irq();
    
    /* 无限循环 */
    while (1)
    {
        /* 可以在这里添加错误指示，比如快速闪烁LED */
    }
}

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/