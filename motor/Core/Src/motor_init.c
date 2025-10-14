/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : motor_init.c
  * @brief          : Motor initialization functions
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
#include "motor_init.h"
#include "motor_ctrl.h"
#include "motor_can_protocol.h"
#include "stm32f1xx_hal_can.h"

/* Private includes ----------------------------------------------------------*/

/* Private typedef -----------------------------------------------------------*/

/* Private define ------------------------------------------------------------*/

/* Private macro -------------------------------------------------------------*/

/* Private variables ---------------------------------------------------------*/
CAN_HandleTypeDef hcan1;

/* Private function prototypes -----------------------------------------------*/

/* Private user code ---------------------------------------------------------*/

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
void MX_TIM4_Init(void)
{
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

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
  sConfigOC.Pulse = 1000;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }

  HAL_TIM_MspPostInit(&htim4);
}

/**
  * @brief TIM3 Initialization Function (for RPM detection timing)
  * @param None
  * @retval None
  */
void MX_TIM3_Init(void)
{
  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 7199;           // 72MHz / 7200 = 10kHz
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 199;               // 10kHz / 200 = 50Hz (20ms)
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  
  /* Timer Clock Enable */
  __HAL_RCC_TIM3_CLK_ENABLE();  // 使能TIM3时钟（用于RPM检测定时）
  __HAL_RCC_TIM4_CLK_ENABLE();  // 使能TIM4时钟（用于PWM输出）

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

  /*Configure GPIO pin : PB7 (FG - 霍尔脉冲输入，外部中断模式) */
  GPIO_InitStruct.Pin = GPIO_PIN_7;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;  // 下降沿中断
  GPIO_InitStruct.Pull = GPIO_PULLUP;           // 上拉，确保稳定的数字信号
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* EXTI interrupt init */
  HAL_NVIC_SetPriority(EXTI9_5_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);
}

/**
  * @brief Motor System Complete Initialization
  * @retval None
  */
void Motor_System_Init(void)
{
  /* 1. 系统时钟配置 */
  SystemClock_Config();
  
  /* 2. GPIO初始化 */
  MX_GPIO_Init();
  
  /* 3. TIM4 PWM初始化 */
  MX_TIM4_Init();
  
  /* 4. TIM3定时器初始化（用于RPM检测） */
  MX_TIM3_Init();
  
  /* 5. 启动PWM输出 */
  if (HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  
  /* 6. 启动定时器中断（100ms周期） */
  if (HAL_TIM_Base_Start_IT(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  
  /* 7. 初始化电机为停止状态 */
  Motor_Stop();
  
  /* 8. 初始化RPM检测系统 */
  Motor_RPM_Detection_Init();
  
  /* 9. 初始化PID控制系统 */
  Motor_PID_Init();
  
  /* 10. 初始化CAN1 */
  Motor_CAN_Init();
}

/**
 * @brief  CAN1初始化函数（Motor作为第二节点用于测试）
 * @retval None
 * @note   配置CAN1为500kbps波特率，使用PA11(RX)和PA12(TX)
 *         与App项目的CAN配置完全相同，用于双节点通信测试
 */
void Motor_CAN_Init(void)
{
    CAN_FilterTypeDef sFilterConfig;
    GPIO_InitTypeDef GPIO_InitStruct = {0};
    
    /* 使能CAN1和GPIOA时钟 */
    __HAL_RCC_CAN1_CLK_ENABLE();
    __HAL_RCC_GPIOA_CLK_ENABLE();
    
    /* 配置CAN1 GPIO引脚: PA11(RX)和PA12(TX) */
    /* PA12 CAN1_TX */
    GPIO_InitStruct.Pin = GPIO_PIN_12;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;         // 复用推挽输出
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
    
    /* PA11 CAN1_RX */
    GPIO_InitStruct.Pin = GPIO_PIN_11;
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;         // 输入模式
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
    
    /* CAN1基本配置 - 与App完全相同 */
    hcan1.Instance = CAN1;
    hcan1.Init.Prescaler = 8;                       // 预分频器
    /* 注意：CAN1挂载在APB1总线上，APB1时钟 = 72MHz / 2 = 36MHz */
    /* CAN时钟 = APB1时钟 / Prescaler = 36MHz / 8 = 4.5MHz */
    hcan1.Init.Mode = CAN_MODE_NORMAL;              // 正常模式（双节点通信）
    hcan1.Init.SyncJumpWidth = CAN_SJW_1TQ;         // 同步跳转宽度
    hcan1.Init.TimeSeg1 = CAN_BS1_6TQ;              // 时间段1：6个时间量子
    hcan1.Init.TimeSeg2 = CAN_BS2_2TQ;              // 时间段2：2个时间量子
    /* 波特率计算: 
     * 1个位时间 = 1 + TimeSeg1 + TimeSeg2 = 1 + 6 + 2 = 9TQ
     * 波特率 = CAN时钟 / (Prescaler × 位时间)
     *        = 36MHz / (8 × 9) = 4.5MHz / 9 = 500kbps 
     */
    
    hcan1.Init.TimeTriggeredMode = DISABLE;         // 禁用时间触发模式
    hcan1.Init.AutoBusOff = DISABLE;                // 禁用自动离线管理
    hcan1.Init.AutoWakeUp = DISABLE;                // 禁用自动唤醒
    hcan1.Init.AutoRetransmission = ENABLE;         // 使能自动重传
    hcan1.Init.ReceiveFifoLocked = DISABLE;         // 接收FIFO不锁定
    hcan1.Init.TransmitFifoPriority = DISABLE;      // 发送FIFO优先级由标识符决定
    
    if (HAL_CAN_Init(&hcan1) != HAL_OK)
    {
        Error_Handler();
    }
    
    /* 配置CAN过滤器 - 接收所有消息 */
    sFilterConfig.FilterBank = 0;                   // 过滤器组0
    sFilterConfig.FilterMode = CAN_FILTERMODE_IDMASK;
    sFilterConfig.FilterScale = CAN_FILTERSCALE_32BIT;
    sFilterConfig.FilterIdHigh = 0x0000;
    sFilterConfig.FilterIdLow = 0x0000;
    sFilterConfig.FilterMaskIdHigh = 0x0000;        // 接收所有ID
    sFilterConfig.FilterMaskIdLow = 0x0000;
    sFilterConfig.FilterFIFOAssignment = CAN_RX_FIFO0;
    sFilterConfig.FilterActivation = ENABLE;
    sFilterConfig.SlaveStartFilterBank = 14;
    
    if (HAL_CAN_ConfigFilter(&hcan1, &sFilterConfig) != HAL_OK)
    {
        Error_Handler();
    }
    
    /* 启动CAN */
    if (HAL_CAN_Start(&hcan1) != HAL_OK)
    {
        Error_Handler();
    }
    
    /* 使能CAN接收中断 - FIFO0消息挂起中断 */
    if (HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING) != HAL_OK)
    {
        Error_Handler();
    }
    
    /* 配置CAN中断优先级 */
    HAL_NVIC_SetPriority(USB_LP_CAN1_RX0_IRQn, 1, 0);  // 优先级1，子优先级0
    HAL_NVIC_EnableIRQ(USB_LP_CAN1_RX0_IRQn);          // 使能CAN RX0中断
}

/**
 * @brief  CAN发送任务 - 每500ms发送一次数据给App (已弃用，改用回传模式)
 * @retval None
 * @note   此函数已不再使用，改为接收App数据后回传
 */
void Motor_CAN_SendTask(void)
{
    // 此函数已弃用，现在使用 Motor_CAN_ReceiveAndEchoTask() 进行回传
}

/**
 * @brief  CAN接收并回传任务 - 已废弃，改用中断方式
 * @retval None
 * @note   此函数已不再使用，现在使用CAN中断接收
 */
void Motor_CAN_ReceiveAndEchoTask(void)
{
    // 此函数已废弃，现在使用 HAL_CAN_RxFifo0MsgPendingCallback() 中断回调
}

/**
 * @brief  CAN接收FIFO0消息挂起中断回调函数
 * @param  hcan: CAN句柄指针
 * @retval None
 * @note   当FIFO0接收到消息时自动调用此函数
 */
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
    CAN_RxHeaderTypeDef RxHeader;
    uint8_t RxData[8];
    
    /* 检查是否是CAN1 */
    if (hcan->Instance == CAN1)
    {
        /* 接收消息 */
        if (HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &RxHeader, RxData) == HAL_OK)
        {
            /* 将CAN消息传递给电机协议处理 */
            MotorCANProtocol_RxCallback(RxHeader.StdId, RxData, RxHeader.DLC);
            
            /* 翻转LED指示接收到CAN消息 */
            HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_3);
        }
    }
}