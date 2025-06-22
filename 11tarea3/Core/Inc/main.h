/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
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

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define LedB_Pin GPIO_PIN_13
#define LedB_GPIO_Port GPIOC
#define userLed_Pin GPIO_PIN_1
#define userLed_GPIO_Port GPIOH
#define segD_Pin GPIO_PIN_0
#define segD_GPIO_Port GPIOC
#define segE_Pin GPIO_PIN_1
#define segE_GPIO_Port GPIOC
#define LedR_Pin GPIO_PIN_2
#define LedR_GPIO_Port GPIOC
#define LedG_Pin GPIO_PIN_3
#define LedG_GPIO_Port GPIOC
#define dis1_Pin GPIO_PIN_0
#define dis1_GPIO_Port GPIOA
#define dis2_Pin GPIO_PIN_1
#define dis2_GPIO_Port GPIOA
#define USART_TX_Pin GPIO_PIN_2
#define USART_TX_GPIO_Port GPIOA
#define USART_RX_Pin GPIO_PIN_3
#define USART_RX_GPIO_Port GPIOA
#define dis4_Pin GPIO_PIN_4
#define dis4_GPIO_Port GPIOA
#define dis3_Pin GPIO_PIN_0
#define dis3_GPIO_Port GPIOB
#define segF_Pin GPIO_PIN_1
#define segF_GPIO_Port GPIOB
#define segB_Pin GPIO_PIN_10
#define segB_GPIO_Port GPIOB
#define segC_Pin GPIO_PIN_13
#define segC_GPIO_Port GPIOB
#define userData_Pin GPIO_PIN_8
#define userData_GPIO_Port GPIOC
#define userSw_Pin GPIO_PIN_9
#define userSw_GPIO_Port GPIOC
#define userSw_EXTI_IRQn EXTI9_5_IRQn
#define segA_Pin GPIO_PIN_10
#define segA_GPIO_Port GPIOA
#define userClk_Pin GPIO_PIN_12
#define userClk_GPIO_Port GPIOA
#define userClk_EXTI_IRQn EXTI15_10_IRQn
#define TMS_Pin GPIO_PIN_13
#define TMS_GPIO_Port GPIOA
#define TCK_Pin GPIO_PIN_14
#define TCK_GPIO_Port GPIOA
#define SWO_Pin GPIO_PIN_3
#define SWO_GPIO_Port GPIOB
#define segG_Pin GPIO_PIN_5
#define segG_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
