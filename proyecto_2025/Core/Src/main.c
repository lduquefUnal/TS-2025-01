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
#include "fsm.h"
#include "AD9833.h"
#include "utils.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
#define LCD_ADDR_7BIT 0x22
extern I2C_HandleTypeDef hi2c1;
#define INT_ENABLE_REG     0x38
#define WHO_AM_I_REG       0x75
I2C_LCD_HandleTypeDef hlcd;
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;
DMA_HandleTypeDef hdma_adc1;

I2C_HandleTypeDef hi2c1;

SPI_HandleTypeDef hspi1;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim4;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */

I2C_LCD_HandleTypeDef hlcd;
// ==== Globals DMA-TX ====
uint8_t   tx_ring[TX_BUFFER_SIZE];
volatile uint16_t  tx_head = 0, tx_tail = 0;
volatile uint8_t   tx_dma_busy = 0;
volatile uint16_t  tx_last_len = 0;
uint32_t counterOverflow= 0;
// ==== Muestreo / FFT ====
volatile uint16_t num_samples_to_capture = 512;
volatile uint16_t capture_index        = 0;
volatile uint16_t adc_max_ch1 = 0, adc_max_ch2 = 0;
volatile uint8_t  is_capture_active    = 0;
static uint32_t start_tick = 0;
volatile uint16_t cycle_count = 0;
#define CYCLES_TO_MEASURE 10
float     phase_results[FFT_SIZE_MAX];
float     frequency_results[FFT_SIZE_MAX];
volatile uint8_t command_ready_flag = 0;
char command_buffer[100] = {0};
float     freq_buffer[FREQ_BUFFER_SIZE] = {0};
uint8_t   freq_index = 0, freq_full = 0;

volatile float frecuencia_generada = 1000.0f;
#define ADC_BUF_LEN 1024 // Tamaño del buffer, preferiblemente una potencia de 2
uint16_t adc_dma_buffer[ADC_BUF_LEN];

// Variables para guardar los resultados del procesamiento
volatile uint16_t adc_max = 0;
volatile uint16_t adc_min = 4095;
volatile uint8_t data_ready_flag = 0;
uint32_t  timer_clk       = 84000000UL;
uint32_t  last_lcd_update = 0;

// Variables temporales para recepción simple (pueden ser estáticas o globales)
static uint8_t rx_index_simple = 0;
static char rx_buffer_simple[100] = {0};
// --- Input Capture ---
volatile uint32_t capture_ch1   = 0;
volatile uint32_t 	capture_ch2 = 0;
volatile uint32_t period_ticks  = 0;
volatile uint8_t  capture_flags = 0;
uint32_t last_uart_activity = 0;
// --- UART RX buffer ---
uint8_t rx_index   = 0;
uint8_t  rx_char    = 0;
char    rx_buffer[100]     = {0};
// --- FSM event ---
volatile e_PosiblesEvents current_event = EVENT_NONE;
// FSM necesita esto
char current_mode_str[21] = {0};
// --- FFT instance ---
arm_rfft_fast_instance_f32 fft_instance;
uint16_t fft_size = 512;


uint8_t tx_buffer[TX_BUFFER_SIZE] = {0};

volatile uint32_t ic_ref      = 0;
volatile uint32_t ic_sig      = 0;
volatile uint32_t last_ref    = 0;
volatile uint8_t  got_ref     = 0;
volatile uint8_t  got_sig     = 0;


/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM4_Init(void);
static void MX_TIM3_Init(void);
static void MX_I2C1_Init(void);
static void MX_SPI1_Init(void);
static void MX_TIM1_Init(void);
static void MX_ADC1_Init(void);
/* USER CODE BEGIN PFP */
void System_Init(void);

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
	void ADC_Set_Sampling_Freq(uint32_t sampling_freq) {
	    if (sampling_freq == 0 || sampling_freq > 2000000) return; // Límite de 2 MHz

	    uint32_t timer_clk = 84000000;
	    uint32_t arr_value = (timer_clk / sampling_freq) - 1;
	    uint16_t psc_value = 0;

	    if (arr_value > 65535) {
	        psc_value = arr_value / 65536;
	        arr_value = (timer_clk / ((psc_value + 1) * sampling_freq)) - 1;
	    }

	    __HAL_TIM_SET_PRESCALER(&htim2, psc_value);
	    __HAL_TIM_SET_AUTORELOAD(&htim2, arr_value);
	}

	/**
	  * @brief Ajusta la frecuencia de la señal PWM generada por TIM1.
	  * @param freq_hz: Frecuencia deseada en Hertz.
	  * @retval None
	  */
	void PWM_SetFrequency(uint32_t freq_hz) {
	    if (freq_hz == 0 || freq_hz > 100000) {
	        return;
	    }

	    // El reloj para TIM1 en tu configuración es de 84MHz
	    uint32_t timer_clk = 84000000;
	    uint32_t arr_value;
	    uint32_t psc_value = 0;

	    // Fórmula: F_pwm = timer_clk / ((PSC+1) * (ARR+1))
	    // Primero, calculamos el valor de ARR asumiendo que el prescaler es 0
	    arr_value = (timer_clk / freq_hz) - 1;

	    // Si el valor de ARR excede el máximo de 16 bits (65535),
	    // debemos ajustar el prescaler para bajar la frecuencia del contador.
	    if (arr_value > 65535) {
	        psc_value = arr_value / 65536;
	        arr_value = (timer_clk / ((psc_value + 1) * freq_hz)) - 1;
	    }

	    // Actualiza los registros del timer en tiempo real
	    __HAL_TIM_SET_PRESCALER(&htim1, psc_value);
	    __HAL_TIM_SET_AUTORELOAD(&htim1, arr_value);

	    // Fija el ciclo de trabajo al 50%
	    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, arr_value / 2);

	    // Actualiza la variable global para mostrarla en el LCD
	    frecuencia_generada = (float)freq_hz;

	}



void System_Init(void) {
		HAL_ADC_Start_DMA(&hadc1, (uint32_t*)adc_dma_buffer, ADC_BUF_LEN);
    	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
	    HAL_TIM_Base_Start_IT(&htim2);
	    HAL_TIM_Base_Start_IT(&htim3);
	    HAL_TIM_Base_Start_IT(&htim4);
	    hlcd.hi2c    = &hi2c1;
	    hlcd.address = LCD_ADDR_7BIT << 1;
	    lcd_init(&hlcd);
	    init_fsm();
	    arm_rfft_fast_init_f32(&fft_instance, fft_size);
	    frecuencia_generada = 1000;
	    lcd_gotoxy(&hlcd, 0, 0);
	    lcd_puts(&hlcd, "   LCD FUNCIONA   ");
	    lcd_gotoxy(&hlcd, 0, 1);
	    lcd_puts(&hlcd, " STM32 + I2C OK ");
	    HAL_Delay(1500);
	    init_fsm();

	    HAL_TIM_Base_Start(&htim2);
	    HAL_UART_Receive_IT(&huart2, &rx_char, 1);
	    PWM_SetFrequency(1000);
	    strcpy(current_mode_str, "ESPERA");
	    mostrar_LCD();
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
  MX_DMA_Init();
  MX_USART2_UART_Init();
  MX_TIM2_Init();
  MX_TIM4_Init();
  MX_TIM3_Init();
  MX_I2C1_Init();
  MX_SPI1_Init();
  MX_TIM1_Init();
  MX_ADC1_Init();
  /* USER CODE BEGIN 2 */
  /* Initialize all configured peripherals */



  AD9833_Init(&hspi1);
  I2C_Scanner();
  SPI_Debug_AD9833();
  System_Init();



  //current_event = EVENT_IMP_SWEEP_START;



  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
	  while (1)
	  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

		  if (current_event != EVENT_NONE) {
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
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */

  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.ScanConvMode = DISABLE;
  hadc1.Init.ContinuousConvMode = ENABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 1;
  hadc1.Init.DMAContinuousRequests = ENABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_0;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_3CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

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
  hi2c1.Init.ClockSpeed = 400000;
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
  * @brief SPI1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI1_Init(void)
{

  /* USER CODE BEGIN SPI1_Init 0 */

  /* USER CODE END SPI1_Init 0 */

  /* USER CODE BEGIN SPI1_Init 1 */

  /* USER CODE END SPI1_Init 1 */
  /* SPI1 parameter configuration*/
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_HIGH;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI1_Init 2 */

  /* USER CODE END SPI1_Init 2 */

}

/**
  * @brief TIM1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM1_Init(void)
{

  /* USER CODE BEGIN TIM1_Init 0 */

  /* USER CODE END TIM1_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 0;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 65535;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = 0;
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim1, &sBreakDeadTimeConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */
  HAL_TIM_MspPostInit(&htim1);

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
  HAL_NVIC_SetPriority(TIM3_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(TIM3_IRQn);
	HAL_TIM_IC_Start_IT(&htim3, TIM_CHANNEL_3);
	HAL_TIM_IC_Start_IT(&htim3, TIM_CHANNEL_4);
  /* USER CODE END TIM3_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_IC_InitTypeDef sConfigIC = {0};
  TIM_SlaveConfigTypeDef sSlaveConfig = {0};

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
  if (HAL_TIM_IC_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_UPDATE;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigIC.ICPolarity = TIM_INPUTCHANNELPOLARITY_RISING;
  sConfigIC.ICSelection = TIM_ICSELECTION_DIRECTTI;
  sConfigIC.ICPrescaler = TIM_ICPSC_DIV1;
  sConfigIC.ICFilter = 0;
  if (HAL_TIM_IC_ConfigChannel(&htim3, &sConfigIC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_IC_ConfigChannel(&htim3, &sConfigIC, TIM_CHANNEL_4) != HAL_OK)
  {
    Error_Handler();
  }
  sSlaveConfig.SlaveMode = TIM_SLAVEMODE_RESET;
  sSlaveConfig.InputTrigger = TIM_TS_TI1FP1;
  sSlaveConfig.TriggerPolarity = TIM_INPUTCHANNELPOLARITY_RISING;
  sSlaveConfig.TriggerPrescaler = TIM_ICPSC_DIV1;
  sSlaveConfig.TriggerFilter = 0;
  if (HAL_TIM_SlaveConfigSynchro(&htim3, &sSlaveConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_IC_ConfigChannel(&htim3, &sConfigIC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigIC.ICPolarity = TIM_INPUTCHANNELPOLARITY_FALLING;
  sConfigIC.ICSelection = TIM_ICSELECTION_INDIRECTTI;
  if (HAL_TIM_IC_ConfigChannel(&htim3, &sConfigIC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */


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
  htim4.Init.Period = 2000-1;
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
  HAL_NVIC_SetPriority(USART2_IRQn, 2, 0); // Fija la prioridad a 2, como mencionaste
  HAL_NVIC_EnableIRQ(USART2_IRQn);
  /* USER CODE END USART2_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA2_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA2_Stream0_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream0_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream0_IRQn);

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

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(FSYNC_GPIO_Port, FSYNC_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin : userLed_Pin */
  GPIO_InitStruct.Pin = userLed_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(userLed_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : userClk_Pin */
  GPIO_InitStruct.Pin = userClk_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(userClk_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : FSYNC_Pin */
  GPIO_InitStruct.Pin = FSYNC_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  HAL_GPIO_Init(FSYNC_GPIO_Port, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */


void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc) {
    // El DMA ha llenado la segunda mitad del buffer (desde ADC_BUF_LEN/2 hasta el final).
    // Ahora podemos procesar esos datos de forma segura.
    uint16_t temp_max = 0;
    uint16_t temp_min = 4095;

    for (int i = ADC_BUF_LEN / 2; i < ADC_BUF_LEN; i++) {
        if (adc_dma_buffer[i] > temp_max) {
            temp_max = adc_dma_buffer[i];
        }
        if (adc_dma_buffer[i] < temp_min) {
            temp_min = adc_dma_buffer[i];
        }
    }
    // Actualizamos las variables globales de forma "atómica"
    adc_max = temp_max;
    adc_min = temp_min;

    // Indicamos que hay un nuevo set de datos de amplitud listos
    current_event = EVENT_DATA_READY;
}
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim){

	if (htim->Instance == TIM2) {
	        HAL_GPIO_TogglePin(userLed_GPIO_Port, userLed_Pin);
	    } else if (htim->Instance == TIM3) {
	        counterOverflow++;
	    }
}
void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart) {
	if (huart == &huart2) {
	        // --- LINEA DE DEPURACION TEMPORAL ---
	        HAL_UART_Transmit(&huart2, (uint8_t*)"[TxCplt]\r\n", 12,HAL_MAX_DELAY);
	        // --- FIN LINEA DE DEPURACION ---

	        tx_tail = (tx_tail + tx_last_len) % TX_BUFFER_SIZE;
	        tx_dma_busy = 0; // Marcar DMA libre

	        // Si hay más datos pendientes, reiniciar transmisión
	        if (tx_tail != tx_head) {
	            uint16_t remaining = (tx_head > tx_tail) ?
	                               (tx_head - tx_tail) :
	                               (TX_BUFFER_SIZE - tx_tail);
	            tx_last_len = remaining;
	            tx_dma_busy = 1;
	            HAL_UART_Transmit_DMA(huart, &tx_ring[tx_tail], remaining);
	        }
	    }
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
	if (huart->Instance == USART2) {
	        // Protección para evitar desbordamiento del buffer de recepción
	        if (rx_index_simple >= sizeof(rx_buffer_simple) - 1) {
	            rx_index_simple = 0;
	            memset(rx_buffer_simple, 0, sizeof(rx_buffer_simple));
	        }

	        // Si se recibe el terminador '@'
	        if (rx_char == '@') {
	            rx_buffer_simple[rx_index_simple] = '\0'; // Terminar la cadena de recepción

	            // Copiar de forma segura al buffer de comando global
	            strncpy(command_buffer, rx_buffer_simple, sizeof(command_buffer) - 1);
	            command_buffer[sizeof(command_buffer) - 1] = '\0';

	            // *** LA CLAVE: SOLO ESTABLECE EL EVENTO PARA LA FSM ***
	            current_event = EVENT_USART_COMMAND;

	            // Reiniciar el buffer de recepción para el siguiente comando
	            rx_index_simple = 0;
	            memset(rx_buffer_simple, 0, sizeof(rx_buffer_simple));

	        } else {
	            // Si no es el final, solo almacena el carácter
	            rx_buffer_simple[rx_index_simple++] = rx_char;
	        }

	        // Siempre reactivar la interrupción para el siguiente carácter
	        HAL_UART_Receive_IT(&huart2, &rx_char, 1);
	    }
}
/**
  * @brief  Callback de interrupción para la Captura de Entrada (Input Capture).
  * Esta función se llama cada vez que el timer detecta un flanco en un canal de IC.
  * @param  htim: Puntero a la estructura del timer.
  * @retval None
  */
void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim) {
    // 1. VERIFICACIÓN INICIAL
    // Solo procesar si hay una medición activa. Esto evita ejecuciones accidentales.
    if (!is_capture_active) {
        return;
    }

// Asegurarse de que la interrupción proviene del Timer 3.
if (htim->Instance == TIM3) {

	// 2. CAPTURA DE LA SEÑAL DE FASE (CANAL 4)
	// Se captura el tiempo de la señal que se desfasa.
	if (htim->Channel == HAL_TIM_ACTIVE_CHANNEL_4) {
		// Solo nos interesa el primer flanco que llega después de iniciar la medición.
		if (!got_sig) {
			ic_sig = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_4);
			got_sig = 1; // Marcar que ya tenemos la captura de fase.
		}
	}

	// 3. CAPTURA DE LA SEÑAL DE REFERENCIA (CANAL 3)
	// Usamos esta señal para medir la frecuencia y como referencia para la fase.
	if (htim->Channel == HAL_TIM_ACTIVE_CHANNEL_3) {

		// 3.1. INICIO DE LA MEDICIÓN
		// Si es el primer flanco (cycle_count es 0), guardamos el tiempo de inicio.
		if (cycle_count == 0) {
			start_tick = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_3);
			// Reiniciar contadores y flags para esta nueva medición.
			counterOverflow = 0;
			got_sig = 0;
			cycle_count++; // Incrementar para esperar los siguientes flancos.

		// 3.2. FIN DE LA MEDICIÓN
		// Si ya no es el primer flanco, contamos hasta completar los ciclos necesarios.
		} else {
			cycle_count++;
			// Cuando hemos capturado suficientes ciclos (ej. 10 flancos para 9 periodos).
			if (cycle_count >= CYCLES_TO_MEASURE) {
				uint32_t end_tick = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_3);
				uint32_t total_ticks;

				// Calcular ticks totales, manejando el desbordamiento (overflow) del timer.
				if (end_tick >= start_tick) {
					total_ticks = (end_tick - start_tick) + (counterOverflow * (htim->Init.Period + 1));
				} else {
					total_ticks = (htim->Init.Period + 1 - start_tick) + end_tick + (counterOverflow * (htim->Init.Period + 1));
				}

				// Si la medición es válida, proceder con los cálculos.
				if (total_ticks > 0) {
					// Obtener la frecuencia del reloj del timer (considerando el prescaler).
					uint32_t tim_prescaler = htim->Init.Prescaler;
					uint32_t timclk = (HAL_RCC_GetPCLK1Freq() * 2) / (tim_prescaler + 1);

					// Calcular el periodo promedio en ticks.
					float avg_period_ticks = (float)total_ticks / (CYCLES_TO_MEASURE - 1);

					// Calcular la frecuencia medida.
					frecuencia_medida = (float)timclk / avg_period_ticks;

					// Calcular el desfase.
					if (got_sig) {
						uint32_t diff_ticks;
						if (ic_sig >= start_tick) { // Caso simple: captura en el mismo ciclo del timer.
							diff_ticks = ic_sig - start_tick;
						} else { // Caso complejo: hubo un overflow entre la referencia y la fase.
							diff_ticks = (htim->Init.Period + 1 - start_tick) + ic_sig;
						}
						fase_medida = fmodf(((float)diff_ticks / avg_period_ticks) * 360.0f, 360.0f);
					} else {
						fase_medida = 0.0f; // No se detectó señal de fase.
					}

					// Guardar los resultados en los arrays.
					if (capture_index < num_samples_to_capture) {
						frequency_results[capture_index] = frecuencia_medida;
						phase_results[capture_index] = fase_medida;
						capture_index++;
					}

					// Indicar que los datos están listos y la medición ha terminado.
					is_capture_active = 0;
					current_event = EVENT_DATA_READY;
				}

				// Reiniciar el contador de ciclos para la próxima medición.
				cycle_count = 0;
			}
		}
	}
}
}

//void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
//{
//    if (GPIO_Pin == userClk_Pin) {
//        data_snapshot = HAL_GPIO_ReadPin(userData_GPIO_Port, userData_Pin);
//        if (pending_event == IDLE)
//            pending_event = EVENT_ENCODER;
//    } else if (GPIO_Pin == userSw_Pin) {
//        if (pending_event == IDLE)
//            pending_event = EVENT_SW;
//    }
//}

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
