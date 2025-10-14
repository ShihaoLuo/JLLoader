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

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  
  /* 测试电机控制函数（示例） */
  HAL_Delay(1000);           // 等待1秒
  Motor_Start();             // 启动电机
  
  /* 测试自适应PID转速控制 - 验证平衡优化效果 */
  Motor_SetTargetRPM(0);   // 测试3000 RPM - 验证稳定性+响应性平衡
  
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
    
    /* ========== 主循环 - 调试信息监控 ========== */
    current_rpm_debug = Motor_GetRPM();         // 当前实际转速
    target_rpm_debug = Motor_GetTargetRPM();    // 目标转速
    pid_output_debug = Motor_GetPIDOutput();    // PID输出值
    rpm_error_debug = (int16_t)target_rpm_debug - (int16_t)current_rpm_debug; // 转速误差
    current_kp_debug = Motor_GetCurrentKp();    // 当前Kp值
    current_ki_debug = Motor_GetCurrentKi();    // 当前Ki值
    
    /* 在调试器中可以观察：
     * - current_rpm_debug: 当前转速
     * - target_rpm_debug: 目标转速
     * - pid_output_debug: PID输出（PWM百分比，0-100）
     * - rpm_error_debug: 转速误差（应该趋向于0）
     * - current_kp_debug: 自适应Kp值
     * - current_ki_debug: 自适应Ki值
     */
    
    /* CAN接收处理已改为中断方式，无需在主循环中调用 */
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

