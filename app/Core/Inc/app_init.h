/**
  ******************************************************************************
  * @file    app_init.h
  * @author  Generated
  * @brief   Header file for application initialization functions
  ******************************************************************************
  */

#ifndef __APP_INIT_H
#define __APP_INIT_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "uart.h"
#include "protocol.h"
#include <stdint.h>

/* Exported constants --------------------------------------------------------*/
#define LED_PIN          GPIO_PIN_13
#define LED_PORT         GPIOC
#define LED_BLINK_DELAY  2000     // 2秒延时

/* Exported types ------------------------------------------------------------*/

/* Exported variables --------------------------------------------------------*/
extern CAN_HandleTypeDef hcan1;

/* Exported function prototypes ---------------------------------------------*/

/**
 * @brief  完整的应用程序初始化
 * @retval None
 */
void App_Init(void);

/**
 * @brief  系统时钟配置（HSE 8MHz + PLL = 72MHz）
 * @retval None
 */
void SystemClock_Config(void);

/**
 * @brief  GPIO初始化函数
 * @retval None
 */
void App_GPIO_Init(void);

/**
 * @brief  SysTick重新配置函数
 * @retval None
 */
void App_SysTick_Reconfig(void);

/**
 * @brief  UART初始化函数
 * @retval None
 */
void App_UART_Init(void);

/**
 * @brief  CAN1初始化函数
 * @retval None
 */
void App_CAN_Init(void);

/**
 * @brief  Protocol初始化函数
 * @retval None
 */
void App_Protocol_Init(void);

/**
 * @brief  发送启动完成系统信息
 * @retval None
 */
void App_SendStartupInfo(void);

/**
 * @brief  LED相关函数
 * @retval None
 */
void App_LED_StartupIndication(void);
void App_LED_Toggle(void);

/**
 * @brief  获取LED状态
 * @retval GPIO_PinState LED当前状态
 */
GPIO_PinState App_LED_GetState(void);

/**
 * @brief  错误处理函数
 * @retval None
 */
void Error_Handler(void);

#ifdef __cplusplus
}
#endif

#endif /* __APP_INIT_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/