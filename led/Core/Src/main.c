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
#include "ws2812.h"
#include "ws2812_screen.h"

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
WS2812_t ws2812_controller;
WS2812_Screen_t screen_controller;
TIM_HandleTypeDef htim1 = {0};  /* TIM1 handle for WS2812 PWM */
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_TIM1_Init(void);
/* USER CODE BEGIN PFP */

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
  MX_TIM1_Init();
  /* USER CODE BEGIN 2 */

  /* Initialize WS2812 LED controller */
  WS2812_Init(&ws2812_controller, &htim1, TIM_CHANNEL_1);
  
  /* Initialize screen controller (4x4 LED grid) */
  Screen_Init(&screen_controller, &ws2812_controller);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    
    /* Test 1: Fill entire screen with colors */
    RGB_t red = WS2812_RGB(255, 0, 0);
    Screen_Fill(&screen_controller, red);
    Screen_Flush(&screen_controller);
    HAL_Delay(2000);
    
    RGB_t green = WS2812_RGB(0, 255, 0);
    Screen_Fill(&screen_controller, green);
    Screen_Flush(&screen_controller);
    HAL_Delay(2000);
    
    RGB_t blue = WS2812_RGB(0, 0, 255);
    Screen_Fill(&screen_controller, blue);
    Screen_Flush(&screen_controller);
    HAL_Delay(2000);
    
    /* Test 2: Draw checkerboard pattern */
    Screen_Clear(&screen_controller);
    RGB_t yellow = WS2812_RGB(255, 255, 0);
    for (uint8_t y = 0; y < 4; y++) {
      for (uint8_t x = 0; x < 4; x++) {
        if ((x + y) % 2 == 0) {
          Screen_SetPixel(&screen_controller, x, y, yellow);
        }
      }
    }
    Screen_Flush(&screen_controller);
    HAL_Delay(2000);
    
    /* Test 3: Draw border rectangle */
    Screen_Clear(&screen_controller);
    RGB_t cyan = WS2812_RGB(0, 255, 255);
    Screen_DrawRect(&screen_controller, 0, 0, 3, 3, cyan);
    Screen_Flush(&screen_controller);
    HAL_Delay(2000);
    
    /* Test 4: Fill rectangle */
    Screen_Clear(&screen_controller);
    RGB_t magenta = WS2812_RGB(255, 0, 255);
    Screen_FillRect(&screen_controller, 1, 1, 2, 2, magenta);
    Screen_Flush(&screen_controller);
    HAL_Delay(2000);
    
    /* Test 5: Draw vertical lines */
    Screen_Clear(&screen_controller);
    RGB_t white = WS2812_RGB(255, 255, 255);
    for (uint8_t x = 0; x < 4; x++) {
      Screen_DrawVLine(&screen_controller, x, 0, 3, white);
    }
    Screen_Flush(&screen_controller);
    HAL_Delay(2000);
    
    /* Test 6: Draw horizontal lines */
    Screen_Clear(&screen_controller);
    RGB_t orange = WS2812_RGB(255, 165, 0);
    for (uint8_t y = 0; y < 4; y++) {
      Screen_DrawHLine(&screen_controller, 0, 3, y, orange);
    }
    Screen_Flush(&screen_controller);
    HAL_Delay(2000);
    
    /* Test 7: Rainbow pattern - each column different color */
    RGB_t rainbow[] = {
      WS2812_RGB(255, 0, 0),      // 红
      WS2812_RGB(255, 127, 0),    // 橙
      WS2812_RGB(0, 255, 0),      // 绿
      WS2812_RGB(0, 0, 255),      // 蓝
    };
    for (uint8_t x = 0; x < 4; x++) {
      Screen_SetColumn(&screen_controller, x, rainbow[x]);
    }
    Screen_Flush(&screen_controller);
    HAL_Delay(2000);
    
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
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();

}

/**
  * @brief TIM1 Initialization Function (for WS2812 control)
  * @param None
  * @retval None
  * 
  * TIM1配置为800KHz PWM频率用于WS2812驱动：
  * PSC = 0 (无分频)
  * ARR = 89 (自动重装值)
  * 频率 = 72MHz / (0+1) / (89+1) = 800KHz ✓
  * 占空比由DMA动态控制
  */
static void MX_TIM1_Init(void)
{

  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* TIM1 clock enable */
  __HAL_RCC_TIM1_CLK_ENABLE();

  /* PA8 GPIO configuration for PWM output */
  GPIO_InitStruct.Pin = GPIO_PIN_8;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /* TIM1 parameter configuration for WS2812 (800KHz) */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 0;           /* PSC = 0: 无分频 */
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 89;             /* ARR = 89, 频率 = 72MHz/90 = 800KHz */
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  HAL_TIM_PWM_Init(&htim1);

  /* Configure output compare for DMA mode */
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 45;               /* 初始占空比50% (45/90) */
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_1);

  /* Break and Dead Time configuration */
  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = 0;
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
  HAL_TIMEx_ConfigBreakDeadTime(&htim1, &sBreakDeadTimeConfig);

  /* Start PWM generation (will use DMA for data transfer) */
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);

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

