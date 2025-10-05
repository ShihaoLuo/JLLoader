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
#include "stm32f1xx_hal.h"  // for TIM3 and other HAL definitions
#include <stdlib.h>  // for abs()
#include <math.h>    // for fabs()

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

// PID控制变量
PID_TypeDef speed_pid = {0};           // PID控制器
uint16_t target_rpm = 0;               // 目标转速
uint8_t pid_control_enable = 0;        // PID控制使能标志

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
 * @brief  设置电机速度百分比 (支持0.1%精度)
 * @param  speed_percent: 速度百分比 (0.0-100.0%)
 *         0.0% = 停止/最低速
 *         100.0% = 最高速
 * @retval None
 * @note   由于硬件是反向PWM控制逻辑，内部会自动转换
 */
void Motor_SetSpeed(float speed_percent)
{
  if (speed_percent > 100.0f) speed_percent = 100.0f;
  if (speed_percent < 0.0f) speed_percent = 0.0f;
  
  // 转换为正向逻辑：输入100%应该得到最高速（PWM占空比0%）
  // 因为硬件反向逻辑：PWM占空比越小，速度越快
  // 使用浮点运算获得0.1%精度：PWM_PERIOD=1000，所以1个计数=0.1%
  float pwm_value_float = ((100.0f - speed_percent) * (float)PWM_PERIOD) / 100.0f;
  uint32_t pwm_value = (uint32_t)(pwm_value_float + 0.5f); // 四舍五入
  
  // 确保PWM值在有效范围内
  if (pwm_value > PWM_PERIOD) pwm_value = PWM_PERIOD;
  
  __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_1, pwm_value);
}

/**
 * @brief  精细速度设置演示函数 (展示0.1%分辨率)
 * @param  speed_percent: 精确速度百分比 (支持0.1%分辨率，如50.7%)
 * @retval None
 * @note   此函数演示0.1%精度PWM控制能力
 */
void Motor_SetSpeedFine(float speed_percent)
{
  // 直接调用支持浮点精度的Motor_SetSpeed函数
  Motor_SetSpeed(speed_percent);
  
  // 可选：输出调试信息显示精确的PWM值
  if (speed_percent > 100.0f) speed_percent = 100.0f;
  if (speed_percent < 0.0f) speed_percent = 0.0f;
  
  float pwm_value_float = ((100.0f - speed_percent) * (float)PWM_PERIOD) / 100.0f;
  uint32_t pwm_value = (uint32_t)(pwm_value_float + 0.5f);
  
  // 调试信息：实际PWM计数值 (0-1000)
  // PWM计数值可以在调试器中观察
  volatile uint32_t debug_pwm_count = pwm_value;
  volatile float debug_actual_percent = (float)(PWM_PERIOD - pwm_value) / (float)PWM_PERIOD * 100.0f;
  
  // 防止编译器优化掉调试变量
  (void)debug_pwm_count;
  (void)debug_actual_percent;
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
 * @brief  初始化PID控制器
 * @retval None
 */
void Motor_PID_Init(void)
{
  speed_pid.kp = PID_KP_BASE;
  speed_pid.ki = PID_KI_BASE;
  speed_pid.kd = PID_KD_BASE;
  speed_pid.target = 0;
  speed_pid.integral = 0;
  speed_pid.prev_error = 0;
  speed_pid.output = 0;
  speed_pid.enable = 0;
  
  target_rpm = 0;
  pid_control_enable = 0;
}

/**
 * @brief  重置PID控制器
 * @retval None
 */
void Motor_PID_Reset(void)
{
  speed_pid.integral = 0;
  speed_pid.prev_error = 0;
  speed_pid.output = 0;
}

/**
 * @brief  设置PID参数
 * @param  kp: 比例系数
 * @param  ki: 积分系数  
 * @param  kd: 微分系数
 * @retval None
 */
void Motor_PID_SetParameters(float kp, float ki, float kd)
{
  speed_pid.kp = kp;
  speed_pid.ki = ki;
  speed_pid.kd = kd;
}

/**
 * @brief  自适应PID参数更新 - 细分档位版本（6档位精确控制）
 * @param  target_rpm: 目标转速
 * @retval None
 */
void Motor_PID_AdaptiveUpdate(uint16_t target_rpm)
{
  if (!ADAPTIVE_ENABLE) return;
  
  float kp, ki, kd;
  
  if (target_rpm <= 800) {
    // 超低转速范围 (0-800 RPM) - 防止超调
    kp = PID_KP_BASE * 0.1f;      // 0.0024
    ki = PID_KI_BASE * 0.1f;      // 0.006
    kd = PID_KD_BASE * 1.0f;      // 0.001
  }
  else if (target_rpm <= RPM_RANGE_LOW) {
    // 低转速范围 (800-1500 RPM) - 平稳控制
    kp = PID_KP_BASE * 0.4f;      // 0.004
    ki = PID_KI_BASE * 0.3f;      // 0.018
    kd = PID_KD_BASE * 1.2f;      // 0.0012
  } 
  else if (target_rpm <= RPM_RANGE_MID) {
    // 中转速范围 (1500-3000 RPM) - 平衡控制
    kp = PID_KP_BASE * 0.6f;      // 0.0048
    ki = PID_KI_BASE * 0.9f;      // 0.03
    kd = PID_KD_BASE * 2.0f;      // 0.002
  }
  else if (target_rpm <= RPM_RANGE_HIGH_MID) {
    // 中高转速范围 (3000-4500 RPM) - 第一个系统特性区间
    kp = PID_KP_BASE * 0.7f;      // 0.0056
    ki = PID_KI_BASE * 1.3f;      // 0.039
    kd = PID_KD_BASE * 2.5f;      // 0.0025
  }
  else if (target_rpm <= RPM_RANGE_HIGH) {
    // 高转速范围 (4500-6000 RPM) - 第二个系统特性区间
    kp = PID_KP_BASE * 0.75;      // 0.0064
    ki = PID_KI_BASE * 1.5f;      // 0.048
    kd = PID_KD_BASE * 2.5f;      // 0.0028
  }
  else {
    // 超高转速范围 (>6000 RPM) - 强化控制
    kp = PID_KP_BASE * 0.8f;      // 0.0064
    ki = PID_KI_BASE * 1.7f;      // 0.051
    kd = PID_KD_BASE * 2.5f;      // 0.0025
  }
  
  // 应用新的PID参数
  Motor_PID_SetParameters(kp, ki, kd);
}

/**
 * @brief  PID控制器更新计算
 * @retval None
 */
void Motor_PID_Update(void)
{
  if (!pid_control_enable || !speed_pid.enable) return;
  
  // 计算误差
  float current = (float)motor_rpm;
  float error = speed_pid.target - current;
  
  // 目标转速为0时，直接停止
  if (speed_pid.target <= 0) {
    speed_pid.output = 0;
    speed_pid.integral = 0;
    speed_pid.prev_error = 0;
    Motor_SetSpeed(0);
    return;
  }
  
  // 改进的误差死区处理 - 只在非常接近目标且变化很小时才应用
  if (fabs(error) < PID_DEADBAND && current > (speed_pid.target * 0.95f)) {
    // 在死区内，缓慢减少积分项，但继续比例和微分控制
    speed_pid.integral *= 0.99f;
    // 继续执行PID计算而不是直接返回
  }
  
  // 比例项
  float p_term = speed_pid.kp * error;
  
  // 积分项优化 - 针对稳态精度调整
  float error_abs = fabs(error);
  if (error_abs > 50) {
    // 大误差时，积分稍快
    speed_pid.integral += error * 1.2f;
  } else if (error_abs > 10) {
    // 中等误差时，正常积分
    speed_pid.integral += error;
  } else {
    // 小误差时，也要保持积分作用以消除稳态误差
    speed_pid.integral += error * 0.8f;
  }
  
  // 积分限幅
  if (speed_pid.integral > PID_INTEGRAL_LIMIT) {
    speed_pid.integral = PID_INTEGRAL_LIMIT;
  } else if (speed_pid.integral < -PID_INTEGRAL_LIMIT) {
    speed_pid.integral = -PID_INTEGRAL_LIMIT;
  }
  float i_term = speed_pid.ki * speed_pid.integral;
  
  // 微分项（带滤波）
  float d_term = speed_pid.kd * (error - speed_pid.prev_error);
  speed_pid.prev_error = error;
  
  // PID输出计算
  float pid_output = p_term + i_term + d_term;
  
  // 前馈补偿计算
  float feedforward_output = 0;
  if (FEEDFORWARD_ENABLE && speed_pid.target > 0) {
    // 基于目标转速的前馈补偿：PWM = 目标转速 × 增益 + 偏移
    feedforward_output = speed_pid.target * FEEDFORWARD_GAIN + FEEDFORWARD_OFFSET;
  }
  
  // 总输出 = PID输出 + 前馈补偿
  speed_pid.output = pid_output + feedforward_output;
  
  // 输出限幅和死区处理
  if (speed_pid.output < PWM_MIN_OUTPUT && speed_pid.target > 0) {
    speed_pid.output = PWM_MIN_OUTPUT; // 确保电机能启动
  } else if (speed_pid.output > PID_OUTPUT_LIMIT) {
    speed_pid.output = PID_OUTPUT_LIMIT;
  } else if (speed_pid.output < 0) {
    speed_pid.output = 0;
  }
  
  // 将PID输出应用到PWM控制 (保持浮点精度，支持0.1%分辨率)
  Motor_SetSpeed(speed_pid.output);
}

/**
 * @brief  设置目标转速（RPM）
 * @param  rpm: 目标转速值
 * @retval None
 */
void Motor_SetTargetRPM(uint16_t rpm)
{
  target_rpm = rpm;
  speed_pid.target = (float)rpm;
  
  if (rpm > 0) {
    // 启用PID控制
    pid_control_enable = 1;
    speed_pid.enable = 1;
    Motor_PID_Reset(); // 重置PID状态
    
    // 自适应调整PID参数
    Motor_PID_AdaptiveUpdate(rpm);
    
    // 对于低转速，给一个初始PWM值帮助启动
    if (rpm < 1000 && motor_rpm < 100) {
      Motor_SetSpeed(PWM_MIN_OUTPUT);
      HAL_Delay(100); // 给电机一点启动时间
    }
  } else {
    // 禁用PID控制
    pid_control_enable = 0;
    speed_pid.enable = 0;
    Motor_SetSpeed(0); // 停止电机
  }
}

/**
 * @brief  使能/禁用PID控制
 * @param  enable: 1=使能，0=禁用
 * @retval None
 */
void Motor_EnablePIDControl(uint8_t enable)
{
  pid_control_enable = enable;
  speed_pid.enable = enable;
  
  if (!enable) {
    Motor_PID_Reset();
  }
}

/**
 * @brief  获取目标转速
 * @retval 目标转速值 (RPM)
 */
uint16_t Motor_GetTargetRPM(void)
{
  return target_rpm;
}

/**
 * @brief  获取PID输出值（调试用）
 * @retval PID输出值 (0-100)
 */
float Motor_GetPIDOutput(void)
{
  return speed_pid.output;
}

/**
 * @brief  获取PID积分项值（调试用）
 * @retval PID积分项值
 */
float Motor_GetPIDIntegral(void)
{
  return speed_pid.integral;
}

/**
 * @brief  获取当前Kp值（调试用）
 * @retval 当前Kp值
 */
float Motor_GetCurrentKp(void)
{
  return speed_pid.kp;
}

/**
 * @brief  获取当前Ki值（调试用）
 * @retval 当前Ki值
 */
float Motor_GetCurrentKi(void)
{
  return speed_pid.ki;
}

/**
 * @brief  渐变设置目标转速（减少突变）
 * @param  rpm: 最终目标转速值
 * @param  step_rpm: 每步增加的转速（建议200-500）
 * @retval None
 */
void Motor_SetTargetRPMGradual(uint16_t rpm, uint16_t step_rpm)
{
  uint16_t current_target = target_rpm;
  
  if (rpm > current_target) {
    // 逐步增加转速
    while (current_target < rpm) {
      current_target += step_rpm;
      if (current_target > rpm) current_target = rpm;
      Motor_SetTargetRPM(current_target);
      HAL_Delay(200); // 每步间隔200ms
    }
  } else if (rpm < current_target) {
    // 逐步减少转速
    while (current_target > rpm) {
      if (current_target > step_rpm) {
        current_target -= step_rpm;
      } else {
        current_target = 0;
      }
      if (current_target < rpm) current_target = rpm;
      Motor_SetTargetRPM(current_target);
      HAL_Delay(200); // 每步间隔200ms
    }
  }
}

/**
 * @brief  更新RPM检测（定时器中断中调用）
 * @retval None
 */
void Motor_RPM_Detection_Update(void)
{
  if (!rpm_detection_active) return;
  
  /* 每20ms更新一次转速计算（频率法） */
  // 频率法计算，避免溢出
  if (pulse_count > 0) {
    // 使用32位计算避免溢出：RPM = (脉冲数 × 3000) / 18
    // 3000 = 60秒/分钟 × 50次/秒(20ms周期)
    uint32_t rpm_calc = ((uint32_t)pulse_count * 3000) / 18;
    
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
  
  // 更新PID控制
  Motor_PID_Update();
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