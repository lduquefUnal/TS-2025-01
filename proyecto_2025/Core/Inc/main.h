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

#ifndef __MAIN_H
#define __MAIN_H
#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include "arm_math.h"
#include "i2c_lcd.h"
#include "stm32f4xx_hal_i2c.h"
/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */
/** FSM states */
typedef enum {
    STATE_IDLE,
    STATE_UPDATE_LCD,
    STATE_PROCESS_FFT,
    STATE_PROCESS_DATA
} e_PosiblesStates;

/** FSM events */
typedef enum {
    EVENT_NONE,
    EVENT_USART_COMMAND,
    EVENT_IC_CAPTURE,
    EVENT_FFT_BUFFER_FULL,
    EVENT_DATA_READY,
    EVENT_PRINT_NEXT_DATA
} e_PosiblesEvents;
/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */
#define FFT_SIZE_MAX        4096
#define TX_BUFFER_SIZE      256
#define FREQ_BUFFER_SIZE    16
#define SWEEP_STEPS         10
#define FREQ_MIN_HZ         1.0f
#define FREQ_MAX_HZ         2500000UL
#define FREQ_START          1000UL
/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */
/* USER CODE END EM */

void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */
/* --- FSM control --- */
void     init_fsm(void);
e_PosiblesStates state_machine_action(e_PosiblesEvents event);
void     mostrar_LCD(void);

/* --- System control --- */
void     System_Init(void);
void     ADC_Set_Sampling_Freq(uint32_t sampling_freq);
void     StartImpedanceMeasurement(uint32_t frequency);
void     PWM_SetFrequency(uint32_t freq_hz);
int      UART_DMA_Enqueue(UART_HandleTypeDef *huart, uint8_t *data, uint16_t length);
void     PWMSweep(void);
void     LogSweep(void);
/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define userLed_Pin        GPIO_PIN_1
#define userLed_GPIO_Port  GPIOH
#define USART_TX_Pin       GPIO_PIN_2
#define USART_TX_GPIO_Port GPIOA
#define USART_RX_Pin       GPIO_PIN_3
#define USART_RX_GPIO_Port GPIOA
#define userClk_Pin        GPIO_PIN_12
#define userClk_GPIO_Port  GPIOA
#define userClk_EXTI_IRQn  EXTI15_10_IRQn
#define TMS_Pin            GPIO_PIN_13
#define TMS_GPIO_Port      GPIOA
#define TCK_Pin            GPIO_PIN_14
#define TCK_GPIO_Port      GPIOA
#define SWO_Pin            GPIO_PIN_3
#define SWO_GPIO_Port      GPIOB
#define FSYNC_Pin          GPIO_PIN_9
#define FSYNC_GPIO_Port    GPIOB

/* USER CODE BEGIN Private defines */
/* === Peripheral handles (defined in main.c) === */
extern ADC_HandleTypeDef    hadc1;
extern DMA_HandleTypeDef    hdma_adc1;
extern I2C_HandleTypeDef    hi2c1;
extern SPI_HandleTypeDef    hspi1;
extern TIM_HandleTypeDef    htim1;
extern TIM_HandleTypeDef    htim2;
extern TIM_HandleTypeDef    htim3;
extern TIM_HandleTypeDef    htim4;
extern UART_HandleTypeDef   huart2;
extern DMA_HandleTypeDef    hdma_usart2_tx;
extern I2C_LCD_HandleTypeDef hlcd;

/* === FSM / UART commands === */
extern volatile e_PosiblesEvents current_event;
extern char     rx_buffer[100];
extern uint8_t  rx_index;
extern char     current_mode_str[21];

/* === UART TX ring buffer === */
extern uint8_t   tx_ring[];
extern uint16_t  tx_head;
extern uint16_t  tx_tail;
extern uint8_t   tx_dma_busy;
extern uint16_t  tx_last_len;
extern uint8_t   tx_buffer[TX_BUFFER_SIZE];

/* === ADC capture / FFT === */
extern volatile uint16_t num_samples_to_capture;
extern volatile uint16_t capture_index;
extern uint16_t adc_dma_buffer[];
extern volatile uint16_t adc_max_ch1;
extern volatile uint16_t adc_max_ch2;
extern volatile uint8_t  is_capture_active;
extern float    phase_results[FFT_SIZE_MAX];
extern float    frequency_results[FFT_SIZE_MAX];

/* === Sweep & logic === */
extern float    freq_buffer[FREQ_BUFFER_SIZE];
extern uint8_t  freq_index;
extern uint8_t  freq_full;
extern uint32_t timer_clk;
extern uint32_t last_lcd_update;

/* === Generated / measured signals === */
extern volatile float frecuencia_generada;
extern volatile float frecuencia_medida;

extern volatile uint32_t period_ticks;
extern uint16_t fft_size;
extern uint32_t counterOverflow;
extern volatile uint8_t  capture_flags;
extern volatile uint32_t capture_ch1;
extern volatile uint32_t capture_ch2;

/* USER CODE END Private defines */
#ifdef __cplusplus
}
#endif
#endif /* __MAIN_H */
