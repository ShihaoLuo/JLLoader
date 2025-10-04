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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdint.h>
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
TIM_HandleTypeDef htim4;

  /* USER CODE BEGIN PV */
  uint32_t pwm_value = 0;
  uint32_t last_tick = 0;
  uint8_t direction = 1; // 1 for increasing, 0 for decreasing
  
  // 电机控制变量
  uint8_t motor_enable = 1;      // 电机使能状态
  uint8_t motor_direction = 1;   // 电机方向 (1=顺时针, 0=逆时针)
  
  // 转速计算变量
  uint32_t pulse_count = 0;      // 脉冲计数
  uint32_t last_pulse_time = 0;  // 上次脉冲时间
  uint32_t rpm_calc_time = 0;    // RPM计算时间基准
  uint16_t motor_rpm = 0;        // 当前转速 (RPM)
  uint8_t last_fg_state = 1;     // 上次FG引脚状态
  /* USER CODE END PV *//* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_TIM4_Init(void);
/* USER CODE BEGIN PFP */
void Motor_SetEnable(uint8_t enable);
void Motor_SetDirection(uint8_t clockwise);
uint16_t Motor_GetRPM(void);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
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

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_TIM4_Init();
  /* USER CODE BEGIN 2 */
  
  /* Start PWM generation on TIM4 Channel 1 (PB6) */
  if (HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  last_tick = HAL_GetTick();
  rpm_calc_time = HAL_GetTick();
  last_fg_state = HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_7);
  
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    
    uint32_t current_tick = HAL_GetTick();
    
    // 读取霍尔传感器脉冲 (PB7-FG)
    uint8_t current_fg_state = HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_7);
    if (last_fg_state == 1 && current_fg_state == 0) // 下降沿检测
    {
      pulse_count++;
      last_pulse_time = current_tick;
    }
    last_fg_state = current_fg_state;
    
    // 每1秒计算一次转速
    if (current_tick - rpm_calc_time >= 1000)
    {
      // 假设电机每转产生2个脉冲（根据实际电机调整）
      motor_rpm = (pulse_count * 60) / 2;  // RPM = (脉冲数 * 60秒) / 每转脉冲数
      pulse_count = 0;
      rpm_calc_time = current_tick;
    }
    
    // PWM斜坡控制 (每50ms更新)
    if (current_tick - last_tick >= 50)
    {
      last_tick = current_tick;
      
      // 只有在电机使能时才更新PWM
      if (motor_enable)
      {
        // Update PWM value (0-1000 corresponds to 0-100% duty cycle)
        // 注意：电机驱动器逻辑 - PWM 0%=最高速, PWM 100%=最低速
        // 所以 pwm_value=0 表示最高速，pwm_value=1000 表示最低速
        if (direction == 1)
        {
          pwm_value += 10; // Increase by 1% every 50ms (降低转速)
          if (pwm_value >= 1000)
          {
            pwm_value = 1000;  // 最低速
            direction = 0; // Start decreasing
            // 在最低速(PWM=100%)时切换电机方向，避免冲击
            motor_direction = !motor_direction;
            Motor_SetDirection(motor_direction);
          }
        }
        else
        {
          // 安全地减少PWM值，避免无符号整数下溢
          if (pwm_value > 10)
          {
            pwm_value -= 10; // Decrease by 1% every 50ms (提高转速)
          }
          else
          {
            pwm_value = 0;   // 最高速
            direction = 1;   // Start increasing
          }
        }
        
        // Update PWM duty cycle
        __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_1, pwm_value);
      }
      else
      {
        // 电机停止时PWM设为0
        pwm_value = 0;
        __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_1, 0);
      }
      
      // 更新控制信号
      HAL_GPIO_WritePin(GPIOB, GPIO_PIN_4, motor_enable ? GPIO_PIN_SET : GPIO_PIN_RESET);  // BK控制
      HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5, motor_direction ? GPIO_PIN_SET : GPIO_PIN_RESET); // FR控制
    }
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
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
  * @brief TIM4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM4_Init(void)
{

  /* USER CODE BEGIN TIM4_Init 0 */

  /* USER CODE END TIM4_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM4_Init 1 */

  /* USER CODE END TIM4_Init 1 */
  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 143;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 1000;
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_PWM_Init(&htim4) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 200;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM4_Init 2 */

  /* USER CODE END TIM4_Init 2 */
  HAL_TIM_MspPostInit(&htim4);

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_3, GPIO_PIN_SET);
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_4, GPIO_PIN_SET);  // BK: 1=运行, 0=刹停
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5, GPIO_PIN_SET);  // FR: 1=顺时针, 0=逆时针

  /*Configure GPIO pin : PB3 (spare output) */
  GPIO_InitStruct.Pin = GPIO_PIN_3;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : PB4 (BK - 启停控制) */
  GPIO_InitStruct.Pin = GPIO_PIN_4;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : PB5 (FR - 方向控制) */
  GPIO_InitStruct.Pin = GPIO_PIN_5;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : PB7 (FG - 霍尔脉冲输入) */
  GPIO_InitStruct.Pin = GPIO_PIN_7;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;  // 上拉，确保稳定的数字信号
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */

/**
 * @brief  设置电机使能状态
 * @param  enable: 1=使能电机, 0=停止电机
 * @retval None
 */
void Motor_SetEnable(uint8_t enable)
{
  motor_enable = enable;
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_4, enable ? GPIO_PIN_SET : GPIO_PIN_RESET);
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
 * @brief  获取电机当前转速
 * @retval 转速值 (RPM)
 */
uint16_t Motor_GetRPM(void)
{
  return motor_rpm;
}

/* USER CODE END 4 */

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

