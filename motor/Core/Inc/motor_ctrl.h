/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : motor_ctrl.h
  * @brief          : Motor control and RPM detection functions header
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
  * 
  * RPM测量系统说明：
  * ================
  * 
  * 1. 硬件配置：
  *    - 霍尔传感器连接到 PB7 (带上拉电阻)
  *    - 电机参数：18脉冲/转 (PPR = 18)
  *    - PWM输出：PB6 (TIM4_CH1，反向逻辑控制)
  *    - 控制信号：PB4(BK启停), PB5(FR方向)
  * 
  * 2. 测量原理：
  *    - 使用频率法：在固定时间窗口(100ms)内计数脉冲
  *    - 公式：RPM = (脉冲数 × 600) / PPR
  *    - 其中 600 = 60秒/分钟 × 10次/秒(100ms周期)
  * 
  * 3. 系统架构：
  *    - GPIO中断(EXTI9_5)：实时计数脉冲，响应快速
  *    - 定时器中断(TIM3)：每100ms计算RPM，避免主循环阻塞
  *    - 双缓冲设计：pulse_count累加，定期转移到计算缓冲区
  * 
  * 4. 关键技术要点：
  *    - TIM3时钟使能：__HAL_RCC_TIM3_CLK_ENABLE()
  *    - MSP初始化：HAL_TIM_Base_MspInit()配置中断
  *    - 32位计算防溢出：(uint32_t)pulse_count * 600
  *    - PWM反向逻辑：Motor_SetSpeed(100) = 最高速度
  * 
  * 5. 性能特点：
  *    - 测量范围：0-65535 RPM
  *    - 更新频率：10 Hz (每100ms)
  *    - 精度：±1 RPM (理论值)
  *    - 响应时间：<100ms
  * 
  ******************************************************************************
  */
/* USER CODE END Header */

#ifndef __MOTOR_CTRL_H__
#define __MOTOR_CTRL_H__

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include <stdint.h>

/* Exported types ------------------------------------------------------------*/

/* Exported constants --------------------------------------------------------*/
#define MAX_RPM_LIMIT 15000            // 最大转速限制(RPM)，防止计算错误

// 电机PWM控制参数（反向逻辑：0%=最高速，100%=最低速）
#define PWM_PERIOD 1000                // PWM周期值
#define PWM_MAX_SPEED 0                // 最高速度对应的PWM值（0%占空比）
#define PWM_MIN_SPEED PWM_PERIOD       // 最低速度对应的PWM值（100%占空比）
#define PWM_STOP PWM_PERIOD            // 停止对应的PWM值（100%占空比）

/* Exported macro ------------------------------------------------------------*/

/* Exported variables --------------------------------------------------------*/
extern TIM_HandleTypeDef htim3;
extern TIM_HandleTypeDef htim4;

// 电机控制变量
extern uint8_t motor_enable;
extern uint8_t motor_direction;

// 转速计算变量
extern uint32_t pulse_count;
extern uint32_t rpm_calc_time;
extern uint16_t motor_rpm;

// 转速检测状态变量
extern uint8_t rpm_detection_active;
extern uint32_t rpm_update_time;
extern uint8_t rpm_valid;

// 毫秒级测量变量
extern uint32_t last_pulse_time_ms;

/* Exported functions prototypes ---------------------------------------------*/
// 电机控制函数
void Motor_Start(void);                    // 启动电机
void Motor_Stop(void);                     // 停止电机
void Motor_SetDirection(uint8_t clockwise); // 设置方向（1=顺时针，0=逆时针）
void Motor_SetSpeed(uint8_t speed_percent); // 设置速度百分比（0-100%，100=最高速）
uint16_t Motor_GetRPM(void);               // 获取当前转速
uint8_t Motor_IsRunning(void);             // 检查电机是否运行

// RPM检测函数
void Motor_RPM_Detection_Init(void);       // 初始化RPM检测
void Motor_RPM_Detection_Update(void);     // 更新RPM检测（定时器中断中调用，每100ms）

// 中断回调函数
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin);
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim);

#ifdef __cplusplus
}
#endif

#endif /* __MOTOR_CTRL_H__ */