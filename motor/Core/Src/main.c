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
#include <string.h>
#include "stm32f1xx.h"
#include "stm32f1xx_hal.h"
#include "stm32f1xx_hal_tim.h"
#include "stm32f1xx_hal_gpio.h"
#include "stm32f1xx_hal_rcc.h"
#include "stm32f1xx_hal_i2c.h"
#include "stm32f1xx_hal_def.h"
#include "stm32f1xx_ll_bus.h"
#include "stm32f1xx_ll_rcc.h"
#include "stm32f1xx_ll_system.h"
#include "stm32f1xx_ll_gpio.h"
#include "stm32f1xx_ll_i2c.h"
#include "stm32f1xx_hal_cortex.h"
#include "stm32f1xx_hal_flash.h"
#include "stm32f1xx_hal_flash_ex.h"

/* GPIO base definitions */
#ifndef GPIOA_BASE
#define GPIOA_BASE          (APB2PERIPH_BASE + 0x0800UL)
#endif

#ifndef GPIOB_BASE
#define GPIOB_BASE          (APB2PERIPH_BASE + 0x0C00UL)
#endif

#ifndef GPIOC_BASE
#define GPIOC_BASE          (APB2PERIPH_BASE + 0x1000UL)
#endif
#include "stm32f1xx_hal_i2c.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define MOTOR_FORWARD  GPIO_PIN_SET
#define MOTOR_REVERSE  GPIO_PIN_RESET
#define MOTOR_BRAKE_ON GPIO_PIN_SET
#define MOTOR_BRAKE_OFF GPIO_PIN_RESET

/* I2C Commands */
#define CMD_MOTOR_CONTROL    0x10
#define CMD_READ_STATUS      0x40

/* I2C Configuration moved to main.h */

/* Motor Control Flags */
#define MOTOR_FLAG_START    0x80
#define MOTOR_FLAG_FORWARD  0x40
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
TIM_HandleTypeDef htim4;
TIM_HandleTypeDef htim2;
I2C_HandleTypeDef hi2c1;

/* USER CODE BEGIN PV */
volatile uint32_t FG_CaptureValue = 0;
volatile uint32_t FG_LastCaptureValue = 0;
volatile uint32_t FG_Period = 0;
volatile uint32_t RPM = 0;
volatile uint8_t FG_FirstCapture = 1;

/* I2C communication buffers */
volatile uint8_t i2c_rx_buffer[4];
volatile uint8_t i2c_tx_buffer[4];
volatile uint8_t i2c_rx_index = 0;
volatile uint8_t motor_status = 0;
volatile uint16_t target_rpm = 0;

/* Error status */
volatile uint8_t error_status = 0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_TIM4_Init(void);
static void MX_TIM2_Init(void);
static void MX_I2C1_Init(void);
/* USER CODE BEGIN PFP */
static void Motor_Init(void);
static void Motor_SetSpeed(uint16_t speed);
static void Motor_SetDirection(GPIO_PinState direction);
static void Motor_SetBrake(GPIO_PinState brake_state);
static uint32_t Motor_GetSpeed(void);
static void Process_I2C_Command(void);
static void Update_Status_Buffer(void);
static void Motor_SetTargetRPM(uint16_t rpm);
static uint16_t RPM_To_PWM(uint16_t rpm);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
static void Motor_Init(void)
{
  /* Set initial motor states */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_4, MOTOR_BRAKE_OFF);  // Release brake
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5, MOTOR_FORWARD);    // Set forward direction
  Motor_SetSpeed(0);                                      // Set initial speed to 0
}

static void Motor_SetSpeed(uint16_t speed)
{
  if(speed > 999) speed = 999;  // Limit to max PWM value
  __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_1, speed);
}

static void Motor_SetDirection(GPIO_PinState direction)
{
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5, direction);  // FR pin
}

static void Motor_SetBrake(GPIO_PinState brake_state)
{
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_4, brake_state);  // BK pin
}

static uint32_t Motor_GetSpeed(void)
{
  return RPM;
}

static void Motor_SetTargetRPM(uint16_t rpm)
{
  target_rpm = rpm;
  uint16_t pwm = RPM_To_PWM(rpm);
  Motor_SetSpeed(pwm);
}

static uint16_t RPM_To_PWM(uint16_t rpm)
{
  // 简单线性映射，可以根据实际电机特性调整
  if (rpm > 10000) rpm = 10000;  // 限制最大转速
  return (rpm * 999) / 10000;    // 将RPM映射到0-999的PWM范围
}

static void Process_I2C_Command(void)
{
  if (i2c_rx_index > 0)
  {
    switch(i2c_rx_buffer[0])
    {
      case CMD_MOTOR_CONTROL:
        if (i2c_rx_index >= 4)  // 确保收到完整的控制命令
        {
          uint8_t control = i2c_rx_buffer[1];
          uint16_t rpm = (i2c_rx_buffer[2] << 8) | i2c_rx_buffer[3];
          
          // 设置电机方向
          Motor_SetDirection((control & MOTOR_FLAG_FORWARD) ? MOTOR_FORWARD : MOTOR_REVERSE);
          
          // 启动/停止电机
          if (control & MOTOR_FLAG_START)
          {
            Motor_SetBrake(MOTOR_BRAKE_OFF);
            Motor_SetTargetRPM(rpm);
            motor_status |= MOTOR_FLAG_START;
            motor_status |= (control & MOTOR_FLAG_FORWARD);
          }
          else
          {
            Motor_SetBrake(MOTOR_BRAKE_ON);
            Motor_SetTargetRPM(0);
            motor_status &= ~MOTOR_FLAG_START;
          }
        }
        break;
        
      case CMD_READ_STATUS:
        Update_Status_Buffer();
        break;
    }
    i2c_rx_index = 0;  // 重置接收索引
  }
}

static void Update_Status_Buffer(void)
{
  i2c_tx_buffer[0] = motor_status;
  uint32_t current_rpm = Motor_GetSpeed();
  i2c_tx_buffer[1] = (current_rpm >> 8) & 0xFF;
  i2c_tx_buffer[2] = current_rpm & 0xFF;
  i2c_tx_buffer[3] = error_status;
}

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{
  /* Enable I2C1 clock */
  __HAL_RCC_I2C1_CLK_ENABLE();

  /* Validate slave address */
  if (I2C_SLAVE_ADDRESS < 0x08 || I2C_SLAVE_ADDRESS > 0x77)
  {
    Error_Handler();  // 地址无效
  }

  hi2c1.Instance = I2C1;
  hi2c1.Init.ClockSpeed = I2C_CLOCK_SPEED;    // 400kHz
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;     // 50% 占空比
  hi2c1.Init.OwnAddress1 = I2C_SLAVE_ADDRESS << 1; // 7位地址左移1位
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;  // 禁用广播呼叫
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;     // 启用时钟延展
  
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }

  /* Enable I2C interrupt mode */
  HAL_I2C_Slave_Receive_IT(&hi2c1, (uint8_t*)i2c_rx_buffer, 4);
}

/**
  * @brief TIM2 Initialization Function for FG signal capture
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{
  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_IC_InitTypeDef sConfigIC = {0};

  /* Enable TIM2 clock */
  __HAL_RCC_TIM2_CLK_ENABLE();

  /* Initialize TIM2 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 71;        /* 72MHz/72 = 1MHz counting frequency */
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 0xFFFF;       /* Maximum period */
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }

  /* Configure Input Capture for PB7 (FG signal) */
  sConfigIC.ICPolarity = TIM_ICPOLARITY_RISING;
  sConfigIC.ICSelection = TIM_ICSELECTION_DIRECTTI;
  sConfigIC.ICPrescaler = TIM_ICPSC_DIV1;
  sConfigIC.ICFilter = 0x0;         /* No filter */
  
  if (HAL_TIM_IC_ConfigChannel(&htim2, &sConfigIC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }

  /* Start Input Capture */
  HAL_TIM_IC_Start_IT(&htim2, TIM_CHANNEL_2);
}

/**
  * @brief Input Capture callback for FG signal
  * @param htim TIM handle
  * @retval None
  */
void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim)
{
  if(htim->Instance == TIM2)
  {
    if(htim->Channel == HAL_TIM_ACTIVE_CHANNEL_2)
    {
      if(FG_FirstCapture)
      {
        FG_LastCaptureValue = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_2);
        FG_FirstCapture = 0;
      }
      else
      {
        FG_CaptureValue = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_2);
        
        /* Calculate period between two rising edges */
        if(FG_CaptureValue > FG_LastCaptureValue)
        {
          FG_Period = FG_CaptureValue - FG_LastCaptureValue;
        }
        else
        {
          FG_Period = (0xFFFF - FG_LastCaptureValue) + FG_CaptureValue;
        }
        
        /* Calculate RPM
         * Timer frequency = 1MHz
         * Time per count = 1µs
         * FG signal: 2 pulses per revolution
         * RPM = (60 * 1000000) / (FG_Period * 2)
         */
        RPM = (60000000UL) / (FG_Period * 2);
        
        FG_LastCaptureValue = FG_CaptureValue;
      }
    }
  }
}
/* USER CODE END 0 */

/**
  * @brief TIM4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM4_Init(void)
{
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* Enable TIM4 clock */
  __HAL_RCC_TIM4_CLK_ENABLE();

  /* Initialize TIM4 */
  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 71;  /* 72MHz/72 = 1MHz counting frequency */
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 999;    /* 1MHz/1000 = 1KHz PWM frequency */
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_PWM_Init(&htim4) != HAL_OK)
  {
    Error_Handler();
  }

  /* PWM configuration */
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;  /* Start with 0% duty cycle */
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }

  /* Start PWM */
  HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_1);
}

/**
  * @brief  The application entry point.
  * @retval int
  */
/**
  * @brief I2C Slave Rx Event callback
  * @param hi2c Pointer to I2C handle
  * @retval None
  */
void HAL_I2C_SlaveRxCpltCallback(I2C_HandleTypeDef *hi2c)
{
  if (hi2c->Instance == I2C1)
  {
    Process_I2C_Command();
    /* Restart I2C reception */
    HAL_I2C_Slave_Receive_IT(hi2c, (uint8_t*)i2c_rx_buffer, 4);
  }
}

/**
  * @brief I2C Slave Tx Event callback
  * @param hi2c Pointer to I2C handle
  * @retval None
  */
void HAL_I2C_SlaveTxCpltCallback(I2C_HandleTypeDef *hi2c)
{
  if (hi2c->Instance == I2C1)
  {
    Update_Status_Buffer();
  }
}

int main(void)
{
  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* Configure the system clock */
  SystemClock_Config();

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_TIM4_Init();
  MX_TIM2_Init();  // Initialize FG capture timer
  MX_I2C1_Init();  // Initialize I2C as slave
  
  /* Initialize motor control */
  Motor_Init();

  /* Initial motor state: stopped */
  Motor_SetDirection(MOTOR_FORWARD);   // Set default direction
  Motor_SetBrake(MOTOR_BRAKE_ON);     // Start with brake on
  Motor_SetSpeed(0);                  // Zero initial speed
  motor_status = 0;                   // Clear status flags

  /* Infinite loop */
  while (1)
  {
    /* USER CODE BEGIN 3 */
    // 主循环中不需要特别的处理，所有操作都是通过I2C中断处理
    HAL_Delay(10);  // 短暂延时防止空循环占用过多CPU
    /* USER CODE END 3 */
  }
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
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /* Configure GPIO pins : PB5(FR) PB4(BK) */
  GPIO_InitStruct.Pin = GPIO_PIN_5 | GPIO_PIN_4;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* Configure GPIO pin : PB6(PWM) */
  GPIO_InitStruct.Pin = GPIO_PIN_6;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* Configure GPIO pin : PB7(FG) */
  GPIO_InitStruct.Pin = GPIO_PIN_7;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* Configure GPIO pins : PB10(SCL) PB11(SDA) */
  GPIO_InitStruct.Pin = GPIO_PIN_10 | GPIO_PIN_11;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_OD;    // I2C需要开漏输出
  GPIO_InitStruct.Pull = GPIO_PULLUP;        // 启用上拉
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;  // 高速模式
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* Configure I2C1 pins : PB10(SCL) PB11(SDA) */
  GPIO_InitStruct.Pin = GPIO_PIN_10 | GPIO_PIN_11;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_OD;      // 开漏输出模式
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  GPIO_InitStruct.Pull = GPIO_PULLUP;          // 需要上拉
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* Set initial states */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_4 | GPIO_PIN_5 | GPIO_PIN_7, GPIO_PIN_RESET);
}

/* USER CODE BEGIN 4 */

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

