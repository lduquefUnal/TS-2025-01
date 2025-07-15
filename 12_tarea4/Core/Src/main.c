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
#include "commands.h"
#include "fsm.h"
#include "stdint.h"
#include "stdio.h"
#include "string.h"
#include <stdlib.h>
#include "arm_math.h"
#include "main.h"


/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
#define MPU6050_I2C_ADDR_7BIT 0x68
#define MPU6050_ADDR (MPU6050_I2C_ADDR_7BIT << 1)
#define LCD_ADDR_7BIT 0x22
extern I2C_HandleTypeDef hi2c1;
#define INT_ENABLE_REG     0x38
#define WHO_AM_I_REG       0x75
I2C_LCD_HandleTypeDef lcd;
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;

TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim4;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
#define FFT_SIZE_MAX 4096
const uint32_t timer_clk = 84000000UL;
 uint16_t adc_buffer[FFT_SIZE_MAX];
 uint16_t fft_size = 1024;
 uint8_t nextDigit_FSM    =  1;
 uint8_t tx_buffer[TX_BUFFER_SIZE] = {0};

#define FREQ_BUFFER_SIZE 16
float freq_buffer[FREQ_BUFFER_SIZE] = {0};
uint8_t freq_index = 0;
uint8_t freq_full = 0;
char rx_buffer[100] = {0};
uint8_t rx_char = 0;
uint8_t rx_index=0;
float32_t accel_buffer_z[FFT_SIZE_MAX];
float32_t fft_output[FFT_SIZE_MAX];
float32_t fft_magnitude[FFT_SIZE_MAX / 2];

volatile uint16_t fft_buffer_index = 0;
volatile float32_t fundamental_freq = 0.0f;

// Instancia de la FFT
arm_rfft_fast_instance_f32 fft_instance;

// Handler del LCD
I2C_LCD_HandleTypeDef hlcd;
volatile e_PosiblesEvents current_event = EVENT_NONE;
volatile uint8_t data_snapshot = 0;

volatile uint8_t  flagCapture      = 0;
volatile uint32_t firstCapture     = 0;
volatile uint32_t secondCapture    = 0;
volatile uint32_t elapsedTicks     = 0;
volatile uint32_t counterOverflow  = 0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM4_Init(void);
static void MX_TIM3_Init(void);
static void MX_I2C1_Init(void);
/* USER CODE BEGIN PFP */
	void System_Init(void);

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

	void I2C_Scanner(void) {
	    char uart_buf[64];
	    HAL_StatusTypeDef res;

	    HAL_UART_Transmit(&huart2, (uint8_t*)"Escaneando bus I2C...\r\n", 24, HAL_MAX_DELAY);

	    for (uint16_t i = 1; i < 128; i++) {
	        // La función HAL espera la dirección de 8 bits (dirección de 7 bits << 1)
	        res = HAL_I2C_IsDeviceReady(&hi2c1, (i << 1), 1, 10);
	        if (res == HAL_OK) {
	            sprintf(uart_buf, "Dispositivo encontrado en la direccion 0x%X\r\n", i);
	            HAL_UART_Transmit(&huart2, (uint8_t*)uart_buf, strlen(uart_buf), HAL_MAX_DELAY);
	        }
	    }
	    HAL_UART_Transmit(&huart2, (uint8_t*)"Escaneo finalizado.\r\n", 20, HAL_MAX_DELAY);
	}
	void System_Init(void) {

	    HAL_TIM_Base_Start_IT(&htim2);
	    HAL_TIM_Base_Start_IT(&htim3);

	    HAL_UART_Receive_IT(&huart2, &rx_char, 1);
	    HAL_TIM_IC_Start_IT(&htim3, TIM_CHANNEL_2);

	    init_fsm();
	    arm_rfft_fast_init_f32(&fft_instance, fft_size);
	    Cmd_HandleFFTSizeCmd("1");
	    Cmd_HandleHelpCmd();
	}

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
  MX_USART2_UART_Init();
  MX_TIM2_Init();
  MX_TIM4_Init();
  MX_TIM3_Init();
  MX_I2C1_Init();
  /* USER CODE BEGIN 2 */
  /* Initialize all configured peripherals */
  HAL_Delay(100);

  HAL_UART_Transmit(&huart2, (uint8_t*)"Iniciando escaneo I2C simplificado...\r\n", 40, HAL_MAX_DELAY);
  I2C_Scanner();
  HAL_UART_Transmit(&huart2, (uint8_t*)"Fin del escaneo.\r\n", 18, HAL_MAX_DELAY);


  	  MPU6050_Init();
      System_Init();

      lcd.hi2c    = &hi2c1;
      lcd.address = LCD_ADDR_7BIT << 1;
      lcd_init(&lcd);
      lcd_gotoxy(&lcd, 0, 0);
      lcd_puts(&lcd, "LCD FUNCIONA");
      lcd_gotoxy(&lcd, 0, 3);
      lcd_puts(&lcd, "STM32 + I2C OK");
      MPU6050_Read_Accel();
      current_event = EVENT_ACCEL_READY;
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
	  while (1)
	  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

		if (current_event != EVENT_NONE){
			state_machine_action(current_event);
			current_event = EVENT_NONE;
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

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 16;
  RCC_OscInitStruct.PLL.PLLN = 336;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV4;
  RCC_OscInitStruct.PLL.PLLQ = 4;
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
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.ClockSpeed = 100000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 42000-1;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 500;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */

}

/**
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 0;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 65535;
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
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_UPDATE;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */
	  HAL_TIM_IC_Start_IT(&htim3, TIM_CHANNEL_2);
  /* USER CODE END TIM3_Init 2 */

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

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM4_Init 1 */

  /* USER CODE END TIM4_Init 1 */
  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 84-1;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 10000-1;
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim4) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim4, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM4_Init 2 */

  /* USER CODE END TIM4_Init 2 */

}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
/* USER CODE BEGIN MX_GPIO_Init_1 */
/* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(userLed_GPIO_Port, userLed_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin : userLed_Pin */
  GPIO_InitStruct.Pin = userLed_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(userLed_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : userData_Pin */
  GPIO_InitStruct.Pin = userData_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(userData_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : userSw_Pin */
  GPIO_InitStruct.Pin = userSw_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(userSw_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : PA8 userClk_Pin */
  GPIO_InitStruct.Pin = GPIO_PIN_8|userClk_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI9_5_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);

  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim){
	if(htim->Instance==TIM2){
		HAL_GPIO_TogglePin(userLed_GPIO_Port,userLed_Pin);
	}else if (htim->Instance==TIM3){

		counterOverflow++;
	}
}
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
	if (GPIO_Pin == GPIO_PIN_8) {
				// Si no hay otro evento en proceso, genera el evento de "datos listos"
				if (current_event == EVENT_NONE) {
					 current_event = EVENT_ACCEL_READY;
				}
}
}
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart){
    if(huart->Instance == USART2){

        HAL_UART_Transmit(&huart2, &rx_char, 1, 10);

        if(rx_index < sizeof(rx_buffer)-1){
            rx_buffer[rx_index++] = rx_char;
        }
        if (rx_char == '@') {
            rx_buffer[rx_index] = '\0';
            rx_index = 0;
            current_event = EVENT_USART_COMMAND;
        }
        HAL_UART_Receive_IT(huart, &rx_char, 1);
    }
}
void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim) {
	if (htim->Instance == TIM3 && htim->Channel == HAL_TIM_ACTIVE_CHANNEL_2) {
		static uint8_t ready = 0;

		if (ready == 0) {
			firstCapture = TIM3->CCR2;
			counterOverflow = 0;
			ready = 1;
		} else {
			secondCapture = TIM3->CCR2;

			if (secondCapture >= firstCapture) {
				elapsedTicks = secondCapture - firstCapture + (counterOverflow * 65536);
			} else {
				elapsedTicks = (0xFFFF - firstCapture + secondCapture + 1) + (counterOverflow * 65536);
			}

			if (elapsedTicks > 0 && current_event == EVENT_NONE) {
				current_event = EVENT_IC_CAPTURE;
			}

			ready = 0; // volver a medir siguiente ciclo
		}
	}
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
