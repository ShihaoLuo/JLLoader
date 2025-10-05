/* USER CODE BEGIN Header */
/**
  ****************** * 2. 自适应PID参数（6档位精确控制，根据系统特性细分）：
 *    - 超低转速(0-800 RPM): Kp=0.0024, Ki=0.006, Kd=0.001
 *    - 低转速(800-1500 RPM): Kp=0.004, Ki=0.018, Kd=0.0012
 *    - 中转速(1500-3000 RPM): Kp=0.0048, Ki=0.03, Kd=0.002  
 *    - 中高转速(3000-4500 RPM): Kp=0.0056, Ki=0.039, Kd=0.0025
 *    - 高转速(4500-6000 RPM): Kp=0.0064, Ki=0.048, Kd=0.0028
 *    - 超高转速(>6000 RPM): Kp=0.0064, Ki=0.051, Kd=0.0025
 *    - 死区 = 12 RPM，最小PWM = 1%******************************************************
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
  *    - 使用频率法：在固定时间窗口(20ms)内计数脉冲
  *    - 公式：RPM = (脉冲数 × 3000) / PPR  
  *    - 其中 3000 = 60秒/分钟 × 50次/秒(20ms周期)
  * 
  * 3. 系统架构：
  *    - GPIO中断(EXTI9_5)：实时计数脉冲，响应快速
  *    - 定时器中断(TIM3)：每20ms计算RPM和PID，高频控制
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
  *    - 更新频率：50 Hz (每20ms) - 高精度控制
  *    - 精度：±1 RPM (理论值)
  *    - 响应时间：<20ms - 快速响应
  * 
  * PID闭环控制系统说明：
  * ====================
  * 
  * 1. 控制模式：
  *    - 开环控制：Motor_SetSpeed(percent) - 直接PWM百分比控制
  *    - 闭环控制：Motor_SetTargetRPM(rpm) - 自适应PID自动调节到目标转速
  * 
  * 2. 自适应PID参数（根据目标转速自动调整）：
  *    - 低转速(0-1500 RPM): Kp=0.012, Ki=0.03, Kd=0.001
  *    - 中转速(1500-3000 RPM): Kp=0.0096, Ki=0.054, Kd=0.0012  
  *    - 高转速(3000-6000 RPM): Kp=0.008, Ki=0.075, Kd=0.0015
  *    - 超高转速(>6000 RPM): Kp=0.0064, Ki=0.09, Kd=0.002
  *    - 死区 = 8 RPM，最小PWM = 15%
  * 
  * 3. 使用方法：
  *    - Motor_SetTargetRPM(1500)     // 自动选择适合1500RPM的PID参数
  *    - Motor_SetTargetRPM(3500)     // 自动选择适合3500RPM的PID参数(3000-4500档)
  *    - Motor_SetTargetRPM(5000)     // 自动选择适合5000RPM的PID参数(4500-6000档)
  *    - Motor_GetCurrentKp/Ki()      // 获取当前使用的PID参数
  *    - Motor_GetRPM()               // 获取当前实际转速
  *    - Motor_GetTargetRPM()         // 获取目标转速
  * 
  * 4. 自适应控制性能（6档位细分优化）：
  *    - 全转速范围稳定：0-8000+ RPM
  *    - 各转速段针对性优化：根据实测系统特性细分
  *    - 3000-4500 RPM：第一系统特性区间，独立调优
  *    - 4500-6000 RPM：第二系统特性区间，独立调优
  *    - 自动参数切换：设置目标转速时自动优化
  *    - 消除静态误差：针对性积分系数调整
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
// PID控制器结构体
typedef struct {
    float kp;                          // 比例系数
    float ki;                          // 积分系数
    float kd;                          // 微分系数
    float target;                      // 目标值
    float integral;                    // 积分累积
    float prev_error;                  // 上次误差
    float output;                      // 输出值
    uint8_t enable;                    // PID使能标志
} PID_TypeDef;

/* Exported constants --------------------------------------------------------*/
#define MAX_RPM_LIMIT 15000            // 最大转速限制(RPM)，防止计算错误

// 电机PWM控制参数（反向逻辑：0%=最高速，100%=最低速）
#define PWM_PERIOD 1000                // PWM周期值
#define PWM_MAX_SPEED 0                // 最高速度对应的PWM值（0%占空比）
#define PWM_MIN_SPEED PWM_PERIOD       // 最低速度对应的PWM值（100%占空比）
#define PWM_STOP PWM_PERIOD            // 停止对应的PWM值（100%占空比）

// PID控制参数 (基础参数，将自适应调整)
#define PID_KP_BASE 0.008f             // 基础比例系数
#define PID_KI_BASE 0.03f              // 基础积分系数
#define PID_KD_BASE 0.001f             // 基础微分系数
#define PID_OUTPUT_LIMIT 100.0f        // PID输出限制（对应PWM百分比）
#define PID_INTEGRAL_LIMIT 500.0f      // 积分限幅（适度增加，平衡稳定性和响应性）
#define PID_DEADBAND 12                // 转速误差死区(RPM)，适度减小提高精度
#define PWM_MIN_OUTPUT 1               // 电机启动最小PWM值（%），降低以解决低速控制饱和

// 前馈补偿参数定义
#define FEEDFORWARD_ENABLE 1           // 前馈补偿使能标志（重新启用）
#define FEEDFORWARD_GAIN 0.008f        // 前馈增益 (PWM% per RPM，适度降低)
#define FEEDFORWARD_OFFSET 3.0f        // 前馈偏移补偿 (PWM%，适度降低)

// 自适应PID参数范围定义
#define ADAPTIVE_ENABLE 1              // 自适应PID使能标志
#define RPM_RANGE_LOW 1500             // 低转速范围上限
#define RPM_RANGE_MID 3000             // 中转速范围上限
#define RPM_RANGE_HIGH_MID 4500        // 中高转速范围上限 (新增细分档位)
#define RPM_RANGE_HIGH 6000            // 高转速范围上限

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

// PID控制变量
extern PID_TypeDef speed_pid;
extern uint16_t target_rpm;
extern uint8_t pid_control_enable;

/* Exported functions prototypes ---------------------------------------------*/
// 电机控制函数
void Motor_Start(void);                    // 启动电机
void Motor_Stop(void);                     // 停止电机
void Motor_SetDirection(uint8_t clockwise); // 设置方向（1=顺时针，0=逆时针）
void Motor_SetSpeed(float speed_percent);   // 设置速度百分比（0.0-100.0%，支持0.1%分辨率，100=最高速）
void Motor_SetSpeedFine(float speed_percent); // 精细速度设置（支持小数点精度演示）
void Motor_SetTargetRPM(uint16_t rpm);     // 设置目标转速（RPM）- PID控制
void Motor_SetTargetRPMGradual(uint16_t rpm, uint16_t step_rpm); // 渐变设置目标转速
void Motor_EnablePIDControl(uint8_t enable); // 使能/禁用PID控制
uint16_t Motor_GetTargetRPM(void);         // 获取目标转速
uint16_t Motor_GetRPM(void);               // 获取当前转速
float Motor_GetPIDOutput(void);            // 获取PID输出值（调试用）
float Motor_GetPIDIntegral(void);          // 获取PID积分项（调试用）
float Motor_GetCurrentKp(void);            // 获取当前Kp值（调试用）
float Motor_GetCurrentKi(void);            // 获取当前Ki值（调试用）
uint8_t Motor_IsRunning(void);             // 检查电机是否运行

// RPM检测函数
void Motor_RPM_Detection_Init(void);       // 初始化RPM检测
void Motor_RPM_Detection_Update(void);     // 更新RPM检测（定时器中断中调用，每100ms）

// PID控制函数
void Motor_PID_Init(void);                 // 初始化PID控制器
void Motor_PID_Update(void);               // 更新PID控制（定时器中断中调用）
void Motor_PID_Reset(void);                // 重置PID控制器
void Motor_PID_SetParameters(float kp, float ki, float kd); // 设置PID参数
void Motor_PID_AdaptiveUpdate(uint16_t target_rpm); // 自适应PID参数更新

// 中断回调函数
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin);
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim);

#ifdef __cplusplus
}
#endif

#endif /* __MOTOR_CTRL_H__ */