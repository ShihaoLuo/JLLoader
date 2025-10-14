/**
 * @file    main.c
 * @brief   STM32F103C8T6 LED闪烁应用程序
 *          PC13端口LED每2秒闪烁一次
 */

#include "main.h"
#include "app_init.h"
#include "protocol.h"
#include "uart.h"
#include <stdint.h>
#include <stdio.h>
#include <string.h>

/* 私有变量 */
static uint32_t last_blink_time = 0;
static uint32_t last_can_send_time = 0;
static uint8_t last_sent_data[8] = {0};  // 保存最后发送的数据用于验证
static uint32_t send_count = 0;
void CAN_SendMessage(void);
uint8_t* App_GetLastSentData(void);
uint32_t App_GetSendCount(void);

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
        
        /* CAN接收处理已改为中断方式，但UART输出在主循环中处理 */
        /* 中断回调函数: HAL_CAN_RxFifo0MsgPendingCallback() */
        App_CAN_ProcessUARTOutput();
        
        /* CAN发送消息 - 每500ms发送一次，等待Motor回传 */
        if ((current_time - last_can_send_time) >= 500)
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
        HAL_Delay(2);
    }
}

void CAN_SendMessage(void)
{
    CAN_TxHeaderTypeDef TxHeader;
    uint8_t TxData[8];
    uint32_t TxMailbox;
    HAL_StatusTypeDef status;
    static uint32_t error_count = 0;
    char uart_buffer[128];
    int len;
    
    // 配置发送消息头
    TxHeader.StdId = 0x123;              // App使用ID 0x123
    TxHeader.ExtId = 0x00;
    TxHeader.RTR = CAN_RTR_DATA;
    TxHeader.IDE = CAN_ID_STD;
    TxHeader.DLC = 8;
    TxHeader.TransmitGlobalTime = DISABLE;
    
    // 准备测试数据 - 包含计数器和固定模式
    TxData[0] = 0xBB;                        // App起始标志
    TxData[1] = 0x66;                        // App起始标志
    TxData[2] = (send_count >> 8) & 0xFF;    // 计数器高字节
    TxData[3] = send_count & 0xFF;           // 计数器低字节
    TxData[4] = 0xAB;                        // 测试数据
    TxData[5] = 0xCD;                        // 测试数据
    TxData[6] = 0xEF;                        // 测试数据
    TxData[7] = 0x90;                        // 测试数据
    
    // 保存发送的数据用于后续验证
    for (int i = 0; i < 8; i++) {
        last_sent_data[i] = TxData[i];
    }
    
    // 发送消息
    status = HAL_CAN_AddTxMessage(&hcan1, &TxHeader, TxData, &TxMailbox);
    if (status == HAL_OK)
    {
        send_count++;
        
        // 打印发送信息
        len = sprintf(uart_buffer, 
            "[CAN TX #%lu] ID:0x%03X Data: %02X %02X %02X %02X %02X %02X %02X %02X\r\n",
            send_count,
            TxHeader.StdId,
            TxData[0], TxData[1], TxData[2], TxData[3],
            TxData[4], TxData[5], TxData[6], TxData[7]
        );
        if (len > 0) {
            UART_SendData((uint8_t*)uart_buffer, len);
        }
    }
    else
    {
        error_count++;
    }
}

// 获取最后发送的数据
uint8_t* App_GetLastSentData(void)
{
    return last_sent_data;
}

// 获取发送计数
uint32_t App_GetSendCount(void)
{
    return send_count;
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