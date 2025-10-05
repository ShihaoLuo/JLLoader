/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : motor_ctrl.c
  * @brief          : Motor control and RPM detection functions
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
#include "motor_ctrl.h"
#include <stdlib.h>  // for abs()

/* Private includes ----------------------------------------------------------*/

/* Private typedef -----------------------------------------------------------*/

/* Private define ------------------------------------------------------------*/

/* Private macro -------------------------------------------------------------*/

/* Private variables ---------------------------------------------------------*/
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim4;

// 电机控制变量
uint8_t motor_enable = 1;      // 电机使能状态 (默认启用)
uint8_t motor_direction = 1;   // 电机方向 (1=顺时针, 0=逆时针)

// 转速计算变量
uint32_t pulse_count = 0;              // 脉冲计数（频率法）
uint32_t rpm_calc_time = 0;            // RPM计算时间基准(毫秒)
uint16_t motor_rpm = 0;                // 当前转速 (RPM)

// 转速检测状态变量
uint8_t rpm_detection_active = 1;      // 转速检测激活标志
uint32_t rpm_update_time = 0;          // 转速更新时间戳
uint8_t rpm_valid = 0;                 // 转速数据有效标志

// 毫秒级测量变量
uint32_t last_pulse_time_ms = 0;       // 毫秒级时间戳

/* Private function prototypes -----------------------------------------------*/

/* Private user code ---------------------------------------------------------*/

/**
 * @brief  外部中断回调函数 - 处理霍尔传感器脉冲
 * @param  GPIO_Pin: 中断引脚
 * @retval None
 */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
  if (GPIO_Pin == GPIO_PIN_7) // PB7 FG脉冲中断
  {
    // 最简化的脉冲计数，避免任何可能的干扰
    pulse_count++;
  }
}

/**
 * @brief  启动电机
 * @retval None
 */
void Motor_Start(void)
{
  motor_enable = 1;
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_4, GPIO_PIN_SET);  // BK=1 启动电机
}

/**
 * @brief  停止电机
 * @retval None
 */
void Motor_Stop(void)
{
  motor_enable = 0;
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_4, GPIO_PIN_RESET); // BK=0 停止电机
  __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_1, PWM_STOP); // 100% PWM停止
}

/**
 * @brief  设置电机旋转方向
 * @param  clockwise: 1=顺时针, 0=逆时针
 * @retval None
 */
void Motor_SetDirection(uint8_t clockwise)
{
  motor_direction = clockwise;
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5, clockwise ? GPIO_PIN_SET : GPIO_PIN_RESET);
}

/**
 * @brief  设置电机速度百分比
 * @param  speed_percent: 速度百分比 (0-100%)
 *         0% = 停止/最低速
 *         100% = 最高速
 * @retval None
 * @note   由于硬件是反向PWM控制逻辑，内部会自动转换
 */
void Motor_SetSpeed(uint8_t speed_percent)
{
  if (speed_percent > 100) speed_percent = 100;
  
  // 转换为正向逻辑：输入100%应该得到最高速（PWM占空比0%）
  // 因为硬件反向逻辑：PWM占空比越小，速度越快
  uint32_t pwm_value = ((100 - speed_percent) * PWM_PERIOD) / 100;
  __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_1, pwm_value);
}

/**
 * @brief  获取电机当前转速
 * @retval 转速值 (RPM)
 */
uint16_t Motor_GetRPM(void)
{
  return motor_rpm;
}

/**
 * @brief  检查电机是否运行
 * @retval 1=运行中, 0=已停止
 */
uint8_t Motor_IsRunning(void)
{
  return motor_enable;
}

/**
 * @brief  初始化RPM检测系统
 * @retval None
 */
void Motor_RPM_Detection_Init(void)
{
  /* 重置计数器和状态 */
  pulse_count = 0;
  motor_rpm = 0;
  rpm_valid = 0;
  last_pulse_time_ms = 0;
  rpm_detection_active = 1;
}

/**
 * @brief  更新RPM检测（定时器中断中调用）
 * @retval None
 */
void Motor_RPM_Detection_Update(void)
{
  if (!rpm_detection_active) return;
  
  /* 每100ms更新一次转速计算（频率法） */
  // 频率法计算，避免溢出
  if (pulse_count > 0) {
    // 使用32位计算避免溢出：RPM = (脉冲数 × 600) / 18
    uint32_t rpm_calc = ((uint32_t)pulse_count * 600) / 18;
    
    // 限制在uint16_t范围内
    if (rpm_calc > 65535) {
      motor_rpm = 65535;
    } else {
      motor_rpm = (uint16_t)rpm_calc;
    }
    
    rpm_valid = 1;
  } else {
    motor_rpm = 0;
    rpm_valid = 0;
  }
  
  // 重置计数
  pulse_count = 0;
}

/**
 * @brief  定时器中断回调函数
 * @param  htim: 定时器句柄
 * @retval None
 */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  if (htim->Instance == TIM3)
  {
    /* TIM3每100ms触发一次RPM检测更新 */
    Motor_RPM_Detection_Update();
  }
}