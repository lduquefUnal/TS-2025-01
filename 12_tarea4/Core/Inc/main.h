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
#include "main.h"
#include "string.h"
#include "stdio.h"
#include "math.h"
#include "stdlib.h"
#include "arm_math.h"
#include "i2c_lcd.h"
#include "stm32f4xx_hal_i2c.h"
#include "stm32f4xx_hal.h"
/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */
extern I2C_HandleTypeDef hi2c1;
extern TIM_HandleTypeDef htim2;
extern TIM_HandleTypeDef htim3;
extern TIM_HandleTypeDef htim4;
extern UART_HandleTypeDef huart2;
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
#define userLed_Pin GPIO_PIN_1
#define userLed_GPIO_Port GPIOH
#define USART_TX_Pin GPIO_PIN_2
#define USART_TX_GPIO_Port GPIOA
#define USART_RX_Pin GPIO_PIN_3
#define USART_RX_GPIO_Port GPIOA
#define userData_Pin GPIO_PIN_8
#define userData_GPIO_Port GPIOC
#define userSw_Pin GPIO_PIN_9
#define userSw_GPIO_Port GPIOC
#define userSw_EXTI_IRQn EXTI9_5_IRQn
#define userClk_Pin GPIO_PIN_12
#define userClk_GPIO_Port GPIOA
#define userClk_EXTI_IRQn EXTI15_10_IRQn
#define TMS_Pin GPIO_PIN_13
#define TMS_GPIO_Port GPIOA
#define TCK_Pin GPIO_PIN_14
#define TCK_GPIO_Port GPIOA
#define SWO_Pin GPIO_PIN_3
#define SWO_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */
#define FREQ_BUFFER_SIZE 16
#define FREQ_BUFFER_SIZE 16
#define TX_BUFFER_SIZE 256
#define FFT_SIZE_MAX 4096
extern uint8_t tx_buffer[TX_BUFFER_SIZE];
extern uint16_t adc_buffer[];
extern float32_t accel_buffer_z[FFT_SIZE_MAX];
extern float32_t fft_output[FFT_SIZE_MAX];
extern float32_t fft_magnitude[FFT_SIZE_MAX / 2];
extern volatile uint16_t fft_buffer_index;
extern volatile float32_t fundamental_freq;
extern arm_rfft_fast_instance_f32 fft_instance;
extern uint8_t tx_buffer[TX_BUFFER_SIZE];
extern uint16_t adc_buffer[];
extern uint16_t fft_size;
extern float freq_buffer[];
extern uint8_t freq_index;
extern uint8_t freq_full;
extern const uint32_t timer_clk;

extern UART_HandleTypeDef huart2;

extern TIM_HandleTypeDef htim2;
extern TIM_HandleTypeDef htim3;

extern uint16_t display_value;
extern uint8_t nextDigit_FSM;
extern float freq_buffer[];
extern uint8_t freq_index;
extern uint8_t freq_full;
extern const uint32_t timer_clk;

extern UART_HandleTypeDef huart2;
extern TIM_HandleTypeDef htim2;
extern TIM_HandleTypeDef htim3;

extern uint16_t display_value;
extern uint8_t nextDigit_FSM;
/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
