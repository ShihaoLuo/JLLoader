/**
 * @file    main.c
 * @brief   STM32F103C8T6 CAN主机应用程序
 *          集成CAN协议、设备管理、心跳发现
 */

#include "main.h"
#include "app_init.h"
#include "protocol.h"
#include "uart.h"
#include "app_can_protocol.h"
#include <stdint.h>
#include <stdio.h>
#include <string.h>

/* 私有变量 */
static uint32_t last_blink_time = 0;
static bool discovery_started = false;
static uint32_t last_motor_test_time = 0;
static uint8_t test_phase = 0;

/* 私有函数声明 */
void CAN_TestMotorControl(void);
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
    
    /* 初始化CAN协议管理器 */
    AppCANProtocol_Init(&hcan1);

    /* 获取初始时间 */
    static uint32_t test_start_time = 0;
    
    UART_Printf("\r\n========================================\r\n");
    UART_Printf("  CAN Protocol Host Application Started\r\n");
    UART_Printf("========================================\r\n\r\n");
    
    /* 延迟1秒后启动设备发现（仅执行一次） */
    UART_Printf("Waiting 1 second before device discovery...\r\n");
    HAL_Delay(DISCOVERY_DELAY);
    AppCANProtocol_StartDiscovery();
    discovery_started = true;
    
    /* 立即启动电机 */
    UART_Printf("\r\n[Phase 1] Starting Motor 1 at %d RPM\r\n", 300);
    MotorControl_t control = {
        .command = MOTOR_CMD_SET_BOTH,
        .target_speed = 300, // 设置速度为300rpm
        .direction = MOTOR_DIR_CW,  // 顺时针方向
        .accel = 0,
        .decel = 0
    };
    AppCANProtocol_ControlMotor(1, &control);
    test_phase = 1; // 更新阶段标记
    test_start_time = HAL_GetTick(); // 记录启动时间
    
    /* 主循环 */
    while (1)
    {
        uint32_t current_time = HAL_GetTick();
        
        /* 检查挂起的跳转请求 */
        Protocol_CheckPendingJump();
        
        /* CAN接收处理已改为中断方式，但UART输出在主循环中处理 */
        App_CAN_ProcessUARTOutput();
        
        /* 协议定时任务（心跳发送、超时检查） */
        AppCANProtocol_Task();

        /* 电机使用演示 - 运行5秒后停止 */
        if (test_phase == 1 && (current_time - test_start_time) >= 5000) {
            // 5秒后停止电机
            UART_Printf("\r\n[Phase 2] Stopping Motor 1\r\n");
            MotorControl_t control = {
                .command = MOTOR_CMD_SET_BOTH,
                .target_speed = 0, // 停止电机
                .direction = MOTOR_DIR_CW,  // 保持当前方向
                .accel = 0,
                .decel = 0
            };
            AppCANProtocol_ControlMotor(1, &control);
            test_phase = 2; // 更新阶段标记
            
            HAL_Delay(100);
            AppCANProtocol_QueryMotorStatus(1);
            
            // 显示状态
            HAL_Delay(100);
            MotorStatus_t *status = AppCANProtocol_GetMotorStatus(1);
            if (status != NULL) {
                UART_Printf("Motor 1 Current Status:\r\n");
                UART_Printf("  Speed: %d RPM\r\n", status->actual_speed);
                UART_Printf("  Direction: %d\r\n", status->direction);
                UART_Printf("  Current: %d mA\r\n", status->current);
                UART_Printf("  Temperature: %d°C\r\n", status->temperature);
            }
        }
        
        /* 短暂延迟 */
        HAL_Delay(10);
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
