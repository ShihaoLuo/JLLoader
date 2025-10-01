/* STM32F103C8T6 简单LED闪烁示例
 * PC13端口，每2秒闪烁一次
 * 这是一个简化版本，可以作为参考
 */

#include "main.h"

// LED配置
#define LED_PIN GPIO_PIN_13
#define LED_PORT GPIOC
#define LED_BLINK_INTERVAL 2000  // 2秒

// 全局变量
static uint32_t last_blink_time = 0;
static uint8_t led_state = 0;

// LED控制函数
void LED_Toggle(void)
{
    if (led_state == 0) {
        // 点亮LED (低电平点亮)
        HAL_GPIO_WritePin(LED_PORT, LED_PIN, GPIO_PIN_RESET);
        led_state = 1;
    } else {
        // 熄灭LED
        HAL_GPIO_WritePin(LED_PORT, LED_PIN, GPIO_PIN_SET);
        led_state = 0;
    }
}

void LED_Update(void)
{
    uint32_t current_time = HAL_GetTick();
    
    if ((current_time - last_blink_time) >= LED_BLINK_INTERVAL) {
        LED_Toggle();
        last_blink_time = current_time;
    }
}

// GPIO初始化
void LED_GPIO_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};
    
    // 使能GPIOC时钟
    __HAL_RCC_GPIOC_CLK_ENABLE();
    
    // 配置PC13为输出
    GPIO_InitStruct.Pin = LED_PIN;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(LED_PORT, &GPIO_InitStruct);
    
    // 初始状态：熄灭
    HAL_GPIO_WritePin(LED_PORT, LED_PIN, GPIO_PIN_SET);
}

// 主函数
int main(void)
{
    // 初始化HAL库
    HAL_Init();
    
    // 配置系统时钟（使用内部RC振荡器）
    SystemClock_Config();
    
    // 初始化GPIO
    LED_GPIO_Init();
    
    // 初始化定时变量
    last_blink_time = HAL_GetTick();
    led_state = 0;
    
    // 主循环
    while (1) {
        LED_Update();
    }
}