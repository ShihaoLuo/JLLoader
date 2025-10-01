/**
 * @file    main.c
 * @brief   STM32F103C8T6 LED闪烁应用程序
 *          PC13端口LED每2秒闪烁一次
 */

#include "main.h"
#include <stdint.h>

/* LED配置宏定义 */
#define LED_PIN          GPIO_PIN_13
#define LED_PORT         GPIOC
#define LED_BLINK_DELAY  2000     // 2秒延时

/* 私有变量 */
static uint32_t last_blink_time = 0;
static GPIO_PinState led_state = GPIO_PIN_SET;  // 初始状态：熄灭

/* 函数声明 */
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void LED_Toggle(void);
static void Simple_Delay(uint32_t delay_count);

/**
 * @brief  主函数
 * @retval int
 */
int main(void)
{   
    /* 初始化HAL库 */
    HAL_Init();

    /* 配置系统时钟 */
    SystemClock_Config();
    
    /* 重新配置SysTick - 在SystemClock_Config后进行 */
    /* 计算SysTick重载值: 72MHz / 1000 - 1 = 71999 (1ms中断) */
    uint32_t systick_reload = 72000 - 1;  
    
    /* 直接配置SysTick寄存器 */
    *((volatile uint32_t*)0xE000E014) = systick_reload;     /* SysTick->LOAD */
    *((volatile uint32_t*)0xE000E018) = 0;                  /* SysTick->VAL = 0 */
    *((volatile uint32_t*)0xE000E010) = 0x07;               /* SysTick->CTRL = CLKSOURCE|TICKINT|ENABLE */
    
    /* 全局使能中断 */
    __enable_irq();

    /* 初始化GPIO */
    MX_GPIO_Init();

    /* 立即点亮LED表示APP已成功启动 */
    HAL_GPIO_WritePin(LED_PORT, LED_PIN, GPIO_PIN_RESET);  // 点亮LED (低电平有效)

    /* 获取初始时间 */
    last_blink_time = HAL_GetTick();
    
    /* 主循环 - 使用HAL_GetTick()进行2秒间隔闪烁 */
    while (1)
    {
        uint32_t current_time = HAL_GetTick();
        
        /* 检查是否到达闪烁时间(2秒) */
        if ((current_time - last_blink_time) >= LED_BLINK_DELAY)
        {
            LED_Toggle();
            last_blink_time = current_time;
        }
    }
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
 * @brief  GPIO初始化函数
 * @param  None
 * @retval None
 */
static void MX_GPIO_Init(void)
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
 * @brief  切换LED状态
 * @param  None
 * @retval None
 */
static void LED_Toggle(void)
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

/**
 * @brief  简单的软件延时函数
 * @param  delay_count: 延时计数
 * @retval None
 */
static void Simple_Delay(uint32_t delay_count)
{
    volatile uint32_t i;
    for (i = 0; i < delay_count; i++)
    {
        __NOP();  // 空操作
    }
}

#ifdef USE_FULL_ASSERT
/**
 * @brief  断言失败时调用的函数
 * @param  file: 源文件名指针
 * @param  line: 断言失败的行号
 * @retval None
 */
void assert_failed(uint8_t *file, uint32_t line)
{
    /* 用户可以添加自己的实现来报告文件名和行号 */
}
#endif /* USE_FULL_ASSERT */