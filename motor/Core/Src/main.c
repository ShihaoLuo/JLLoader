/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "motor_init.h"
#include "motor_ctrl.h"
#include "motor_can_protocol.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */
uint16_t current_rpm_debug = 0;     // 当前转速（调试用）
uint16_t target_rpm_debug = 0;      // 目标转速（调试用）
float pid_output_debug = 0;         // PID输出（调试用）
int16_t rpm_error_debug = 0;        // 转速误差（调试用）
float current_kp_debug = 0;         // 当前Kp值（调试用）
float current_ki_debug = 0;         // 当前Ki值（调试用）
uint16_t target_speed = 0;        // 目标速度（调试用）
/**
  * @brief  电机控制系统主程序
  * @retval int 程序退出状态（通常不返回）
  * 
  * 程序功能：
  * - 初始化电机控制系统（时钟、GPIO、PWM、中断）
  * - 提供完整的电机控制API（启停、方向、速度）
  * - 实时检测和计算电机转速（18脉冲/转）
  * - 支持直观的速度控制（0%=停止，100%=最高速）
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Initialize all configured peripherals */
  /* USER CODE BEGIN 2 */
  
  /* ========== 电机系统完整初始化 ========== */
  Motor_System_Init();
  
  /* ========== 初始化CAN协议（从机模式） ========== */
  /* 注意：节点ID可以通过编译宏或DIP开关配置 */
  MotorCANProtocol_Init(&hcan1, MOTOR_NODE_ID);  // 使用默认ID 0x01

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  
  /* 注意：电机现在由CAN协议控制，不再使用本地测试代码 */
  /* 电机将等待主机通过CAN总线发送控制命令 */
  HAL_Delay(1000);           // 等待1秒，让系统稳定
  
  /* 0.1%精度PWM控制演示（可选测试）
   * 以下代码展示了如何使用0.1%精度控制：
   * Motor_SetSpeedFine(50.7f);  // 50.7% PWM，对应PWM计数值493
   * Motor_SetSpeedFine(25.3f);  // 25.3% PWM，对应PWM计数值747
   * Motor_SetSpeedFine(75.1f);  // 75.1% PWM，对应PWM计数值249
   * 
   * PWM_PERIOD=1000，所以：
   * - 1个计数 = 0.1% PWM精度
   * - 50.7% → PWM计数 = (100-50.7)*10 = 493
   * - 精度范围：0.0% - 100.0%，分辨率0.1%
   */
  
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    
    /* ========== 主循环 - CAN协议处理和电机控制 ========== */
    
    /* 协议定时任务（更新运行时间等） */
    MotorCANProtocol_Task();
    
    /* 根据CAN协议设置的目标参数控制电机 */
    if (MotorCANProtocol_IsRunning()) {
        // 电机应该运行
        target_speed = MotorCANProtocol_GetTargetSpeed();
        MotorDirection_t target_dir = MotorCANProtocol_GetTargetDirection();
        
        // 设置电机方向
        if (target_dir == MOTOR_DIR_CW) {
            Motor_SetDirection(true);   // 正转
        } else if (target_dir == MOTOR_DIR_CCW) {
            Motor_SetDirection(false);  // 反转
        }
        
        // 设置目标转速
        Motor_SetTargetRPM(target_speed);
        
        // 确保电机启动（无论当前转速是多少）
        if (target_speed > 0) {
            Motor_Start();
        }
    } else {
        // 电机应该停止
        Motor_Stop();
        Motor_SetTargetRPM(0);
    }
    
    /* 更新电机状态到协议层（供CAN响应使用） */
    current_rpm_debug = Motor_GetRPM();
    target_rpm_debug = Motor_GetTargetRPM();
    pid_output_debug = Motor_GetPIDOutput();
    rpm_error_debug = (int16_t)target_rpm_debug - (int16_t)current_rpm_debug;
    current_kp_debug = Motor_GetCurrentKp();
    current_ki_debug = Motor_GetCurrentKi();
    
    // 模拟电流和温度读取（实际应用中应从传感器读取）
    uint16_t current_ma = (uint16_t)(pid_output_debug * 100);  // 简化计算
    uint8_t temperature = 25 + (current_ma / 1000);  // 简化计算
    uint8_t fault = 0x00;  // 无故障
    
    // 更新协议状态
    MotorCANProtocol_UpdateStatus(current_rpm_debug, current_ma, temperature, fault);
    
    /* CAN接收处理已改为中断方式 */
    /* 中断回调函数: HAL_CAN_RxFifo0MsgPendingCallback() */
    
    /* 短暂延时降低CPU占用 */
    HAL_Delay(10);
  }
  /* USER CODE END 3 */
}

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

