/**
  ******************************************************************************
  * @file    app_init.c
  * @author  Generated
  * @brief   Application initialization functions
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "app_init.h"
#include "uart.h"
#include "protocol.h"
#include <stdio.h>
#include <string.h>
#include <stdbool.h>

/* Private variables ---------------------------------------------------------*/
static GPIO_PinState led_state = GPIO_PIN_SET;  // 初始状态：熄灭

/* CAN Handle */
CAN_HandleTypeDef hcan1;

/* External variables --------------------------------------------------------*/
extern UART_HandleTypeDef huart1;

/* CAN接收数据缓冲 - 用于在中断和主循环间传递数据 */
typedef struct {
    char message[256];
    uint16_t length;
    volatile bool pending;
} CAN_UART_Buffer_t;

static CAN_UART_Buffer_t uart_tx_buffer = {0};

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

    /* 初始化UART通信 */
    App_UART_Init();
    
    /* 初始化CAN1 */
    App_CAN_Init();
    
    /* 初始化Protocol */
    App_Protocol_Init();

    /* 显示启动指示 */
    App_LED_StartupIndication();
    
    // /* 发送启动完成系统信息到PC端 */
    // App_SendStartupInfo();
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
 * @brief  UART初始化函数
 * @retval None
 */
void App_UART_Init(void)
{
    /* 初始化UART1 */
    MX_USART1_UART_Init();
    
    /* 启动UART中断接收 */
    UART_StartInterruptReceive();
}

/**
 * @brief  Protocol初始化函数
 * @retval None
 */
void App_Protocol_Init(void)
{
    /* 初始化协议模块 */
    Protocol_Init();
}

/**
 * @brief  CAN1初始化函数
 * @retval None
 * @note   配置CAN1为500kbps波特率，使用PA11(RX)和PA12(TX)
 *         STM32F103C8T6 CAN1引脚：
 *         - PA11: CAN1_RX
 *         - PA12: CAN1_TX
 */
void App_CAN_Init(void)
{
    CAN_FilterTypeDef sFilterConfig;
    GPIO_InitTypeDef GPIO_InitStruct = {0};
    
    /* 使能CAN1和GPIOA时钟 */
    __HAL_RCC_CAN1_CLK_ENABLE();
    __HAL_RCC_GPIOA_CLK_ENABLE();
    
    /* 配置CAN1 GPIO引脚: PA11(RX)和PA12(TX) */
    /* PA12 CAN1_TX */
    GPIO_InitStruct.Pin = GPIO_PIN_12;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;         // 复用推挽输出
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
    
    /* PA11 CAN1_RX */
    GPIO_InitStruct.Pin = GPIO_PIN_11;
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;         // 输入模式
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
    
    /* CAN1基本配置 */
    hcan1.Instance = CAN1;
    hcan1.Init.Prescaler = 8;                       // 预分频器
    /* 注意：CAN1挂载在APB1总线上，APB1时钟 = 72MHz / 2 = 36MHz */
    /* CAN时钟 = APB1时钟 / Prescaler = 36MHz / 8 = 4.5MHz */
    hcan1.Init.Mode = CAN_MODE_NORMAL;              // 正常模式
    hcan1.Init.SyncJumpWidth = CAN_SJW_1TQ;         // 同步跳转宽度
    hcan1.Init.TimeSeg1 = CAN_BS1_6TQ;              // 时间段1：6个时间量子
    hcan1.Init.TimeSeg2 = CAN_BS2_2TQ;              // 时间段2：2个时间量子
    /* 波特率计算: 
     * 1个位时间 = 1 + TimeSeg1 + TimeSeg2 = 1 + 6 + 2 = 9TQ
     * 波特率 = CAN时钟 / (Prescaler × 位时间)
     *        = 36MHz / (8 × 9) = 4.5MHz / 9 = 500kbps 
     */
    
    hcan1.Init.TimeTriggeredMode = DISABLE;         // 禁用时间触发模式
    hcan1.Init.AutoBusOff = DISABLE;                // 禁用自动离线管理
    hcan1.Init.AutoWakeUp = DISABLE;                // 禁用自动唤醒
    hcan1.Init.AutoRetransmission = ENABLE;         // 使能自动重传
    hcan1.Init.ReceiveFifoLocked = DISABLE;         // 接收FIFO不锁定
    hcan1.Init.TransmitFifoPriority = DISABLE;      // 发送FIFO优先级由标识符决定
    
    if (HAL_CAN_Init(&hcan1) != HAL_OK)
    {
        Error_Handler();
    }
    
    /* 配置CAN过滤器 - 接收所有消息 */
    sFilterConfig.FilterBank = 0;                   // 过滤器组0
    sFilterConfig.FilterMode = CAN_FILTERMODE_IDMASK;
    sFilterConfig.FilterScale = CAN_FILTERSCALE_32BIT;
    sFilterConfig.FilterIdHigh = 0x0000;
    sFilterConfig.FilterIdLow = 0x0000;
    sFilterConfig.FilterMaskIdHigh = 0x0000;        // 接收所有ID
    sFilterConfig.FilterMaskIdLow = 0x0000;
    sFilterConfig.FilterFIFOAssignment = CAN_RX_FIFO0;
    sFilterConfig.FilterActivation = ENABLE;
    sFilterConfig.SlaveStartFilterBank = 14;
    
    if (HAL_CAN_ConfigFilter(&hcan1, &sFilterConfig) != HAL_OK)
    {
        Error_Handler();
    }
    
    /* 启动CAN */
    if (HAL_CAN_Start(&hcan1) != HAL_OK)
    {
        Error_Handler();
    }
    
    /* 使能CAN接收中断 - FIFO0消息挂起中断 */
    if (HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING) != HAL_OK)
    {
        Error_Handler();
    }
    
    /* 配置CAN中断优先级 - 设置为较低优先级，避免阻塞UART */
    HAL_NVIC_SetPriority(USB_LP_CAN1_RX0_IRQn, 3, 0);  // 优先级3（低于UART的0），子优先级0
    HAL_NVIC_EnableIRQ(USB_LP_CAN1_RX0_IRQn);          // 使能CAN RX0中断
}

/**
 * @brief  处理CAN接收到的UART发送任务
 * @retval None
 * @note   在主循环中调用，发送CAN中断中准备好的数据
 */
void App_CAN_ProcessUARTOutput(void)
{
    /* 检查是否有待发送的数据 */
    if (uart_tx_buffer.pending)
    {
        /* 发送数据 */
        if (UART_SendData((uint8_t*)uart_tx_buffer.message, uart_tx_buffer.length))
        {
            /* 发送成功，清除标志 */
            uart_tx_buffer.pending = false;
        }
    }
}

/**
 * @brief  CAN接收任务 - 已废弃，改用中断方式
 * @retval None
 * @note   此函数已不再使用，现在使用CAN中断接收 + App_CAN_ProcessUARTOutput
 */
void App_CAN_ReceiveTask(void)
{
    // 此函数已废弃，现在使用 HAL_CAN_RxFifo0MsgPendingCallback() 中断回调
}

/**
 * @brief  CAN接收FIFO0消息挂起中断回调函数
 * @param  hcan: CAN句柄指针
 * @retval None
 * @note   当FIFO0接收到消息时自动调用此函数
 */
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
    CAN_RxHeaderTypeDef RxHeader;
    uint8_t RxData[8];
    static uint32_t receive_count = 0;
    static uint32_t match_count = 0;
    static uint32_t mismatch_count = 0;
    
    /* 检查是否是CAN1 */
    if (hcan->Instance == CAN1)
    {
        /* 接收消息 */
        if (HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &RxHeader, RxData) == HAL_OK)
        {
            receive_count++;
            
            /* 验证消息ID - 来自Motor的回传消息 */
            if (RxHeader.StdId == 0x456)
            {
                char uart_buffer[256];
                int len;
                uint8_t* last_sent = App_GetLastSentData();
                bool data_match = true;
                
                /* 验证数据是否与发送的一致 */
                for (int i = 0; i < 8; i++)
                {
                    if (RxData[i] != last_sent[i])
                    {
                        data_match = false;
                        break;
                    }
                }
                
                if (data_match)
                {
                    match_count++;
                    /* 数据匹配 - 打印成功信息 */
                    len = sprintf(uart_buffer, 
                        "[ECHO OK #%lu] Data Match! TX=%lu RX=%lu Match=%lu Err=%lu | Data: %02X %02X %02X %02X %02X %02X %02X %02X\r\n",
                        receive_count,
                        App_GetSendCount(),
                        receive_count,
                        match_count,
                        mismatch_count,
                        RxData[0], RxData[1], RxData[2], RxData[3],
                        RxData[4], RxData[5], RxData[6], RxData[7]
                    );
                }
                else
                {
                    mismatch_count++;
                    /* 数据不匹配 - 打印错误信息 */
                    len = sprintf(uart_buffer, 
                        "[ECHO ERROR #%lu] Data Mismatch!\r\n"
                        "  Sent: %02X %02X %02X %02X %02X %02X %02X %02X\r\n"
                        "  Recv: %02X %02X %02X %02X %02X %02X %02X %02X\r\n",
                        receive_count,
                        last_sent[0], last_sent[1], last_sent[2], last_sent[3],
                        last_sent[4], last_sent[5], last_sent[6], last_sent[7],
                        RxData[0], RxData[1], RxData[2], RxData[3],
                        RxData[4], RxData[5], RxData[6], RxData[7]
                    );
                }
                
                /* 准备UART发送数据 - 不在中断中直接发送，避免阻塞 */
                if (len > 0 && !uart_tx_buffer.pending)
                {
                    /* 复制数据到缓冲区 */
                    memcpy(uart_tx_buffer.message, uart_buffer, len);
                    uart_tx_buffer.length = len;
                    uart_tx_buffer.pending = true;  // 设置待发送标志
                }
                
                /* 翻转LED指示接收成功 */
                if (data_match)
                {
                    App_LED_Toggle();
                }
            }
        }
    }
}

/**
 * @brief  发送启动完成系统信息
 * @retval None
 */
void App_SendStartupInfo(void)
{
    /* 小延时确保UART完全初始化 */
    HAL_Delay(10);
    
    /* 发送系统信息到PC端，表示APP启动完成 */
    Protocol_SendSystemInfo();
    
    /* 小延时确保数据发送完成 */
    HAL_Delay(10);
    
    /* 发送内存信息 */
    Protocol_SendMemoryInfo();
    
    /* 小延时确保数据发送完成 */
    HAL_Delay(10);
    
    /* 发送状态报告，表示系统就绪 */
    Protocol_SendStatusReport(STATUS_READY);
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