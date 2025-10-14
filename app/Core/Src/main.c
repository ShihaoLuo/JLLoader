/**
 * @file    main.c
 * @brief   STM32F103C8T6 LED闪烁应用程序
 *          PC13端口LED每2秒闪烁一次
 */

#include "main.h"
#include "app_init.h"
#include "protocol.h"
#include <stdint.h>

/* 私有变量 */
static uint32_t last_blink_time = 0;
static uint32_t last_can_send_time = 0;
void CAN_SendMessage(void);

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
        
        /* 检查挂起的跳转请求 */
        Protocol_CheckPendingJump();
        
        /* CAN发送消息 - 每1秒发送一次，避免占用过多CPU时间 */
        if ((current_time - last_can_send_time) >= 1)
        {
            CAN_SendMessage();
            last_can_send_time = current_time;
        }

        /* 检查是否到达闪烁时间(2秒) */
        if ((current_time - last_blink_time) >= LED_BLINK_DELAY)
        {
            App_LED_Toggle();
            last_blink_time = current_time;
        }
        
        /* 短暂延迟 */
        HAL_Delay(20);
    }
}

void CAN_SendMessage(void)
{
    CAN_TxHeaderTypeDef TxHeader;
    uint8_t TxData[8];
    uint32_t TxMailbox;
    HAL_StatusTypeDef status;
    static uint32_t error_count = 0;
    
    // 配置发送消息头
    TxHeader.StdId = 0x123;              // 标准ID
    TxHeader.ExtId = 0x00;               // 扩展ID（未使用）
    TxHeader.RTR = CAN_RTR_DATA;         // 数据帧
    TxHeader.IDE = CAN_ID_STD;           // 使用标准ID
    TxHeader.DLC = 8;                    // 数据长度8字节
    TxHeader.TransmitGlobalTime = DISABLE;
    
    // 准备数据
    TxData[0] = 0x01;
    TxData[1] = 0x02;
    TxData[2] = 0x03;
    TxData[3] = 0x04;
    TxData[4] = 0x05;
    TxData[5] = 0x06;
    TxData[6] = 0x07;
    TxData[7] = 0x08;
    
    // 发送消息 - 不要因为CAN错误而挂起系统
    status = HAL_CAN_AddTxMessage(&hcan1, &TxHeader, TxData, &TxMailbox);
    if (status != HAL_OK)
    {
        // 发送失败 - 仅计数，不挂起系统
        error_count++;
        // 可选：如果错误次数过多，可以考虑禁用CAN发送
        // 但不要调用Error_Handler()，避免影响UART等其他功能
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