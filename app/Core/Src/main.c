/**
 * @file    main.c
 * @brief   STM32F103C8T6 LED闪烁应用程序
 *          PC13端口LED每2秒闪烁一次
 */

#include "main.h"
#include "app_init.h"
#include <stdint.h>

/* 私有变量 */
static uint32_t last_blink_time = 0;

/**
 * @brief  主函数
 * @retval int
 */
int main(void)
{   
    /* 完整的应用程序初始化 */
    App_Init();

    /* 获取初始时间 */
    last_blink_time = HAL_GetTick();
    
    /* 主循环 - 使用HAL_GetTick()进行2秒间隔闪烁 */
    while (1)
    {
        uint32_t current_time = HAL_GetTick();
        
        /* 检查是否到达闪烁时间(2秒) */
        if ((current_time - last_blink_time) >= LED_BLINK_DELAY)
        {
            App_LED_Toggle();
            last_blink_time = current_time;
        }
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