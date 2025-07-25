/*
 * utils.c
 *
 *  Created on: Jul 23, 2025
 *      Author: luisduquefranco
 */
#include "utils.h"
#include "AD9833.h"
#include "fsm.h"
#include <stdio.h>
#include <string.h>
#include <math.h>

extern volatile float frecuencia_generada;
void LogSweep(void) {
	  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_9, GPIO_PIN_RESET);
	  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_9, GPIO_PIN_SET);
	    AD9833_SetWaveform(0);  // modo seno

	    float ratio = powf(FREQ_MAX_HZ  / FREQ_START, 1.0f / (SWEEP_STEPS - 1));

	    for (int i = 0; i < SWEEP_STEPS; i++) {
	        float f = FREQ_START * powf(ratio, (float)i);
	        AD9833_SetFrequency((uint32_t)f);
	        frecuencia_generada = f;
	        mostrar_LCD();
	        char buf[64];
	        int n = snprintf(buf, sizeof(buf), "Paso %2d: %.0f Hz\r\n", i+1, f);
	        HAL_UART_Transmit(&huart2, (uint8_t*)buf, n, HAL_MAX_DELAY);
	        HAL_Delay(3000);
	    }

	    // Vuelve a 1 kHz por defecto
	    AD9833_SetFrequency((uint32_t)FREQ_START);
	    frecuencia_generada = FREQ_START;
	    mostrar_LCD();
	}
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
	void SPI_Debug_AD9833(void) {
	    char uart_buf[64];
	    HAL_StatusTypeDef res;
	    uint8_t test_data[2] = {0x21, 0x00}; // Comando RESET para AD9833

	    HAL_UART_Transmit(&huart2, (uint8_t*)"Verificando AD9833 por SPI...\r\n", 32, HAL_MAX_DELAY);

	    HAL_GPIO_WritePin(FSYNC_GPIO_Port, FSYNC_Pin , GPIO_PIN_RESET);
	    res = HAL_SPI_Transmit(&hspi1, test_data, 2, HAL_MAX_DELAY);
	    HAL_GPIO_WritePin(FSYNC_GPIO_Port, FSYNC_Pin , GPIO_PIN_SET);

	    if (res == HAL_OK) {
	        sprintf(uart_buf, "AD9833 responde OK por SPI\r\n");
	    } else {
	        sprintf(uart_buf, "Fallo comunicacion SPI con AD9833\r\n");
	    }

	    HAL_UART_Transmit(&huart2, (uint8_t*)uart_buf, strlen(uart_buf), HAL_MAX_DELAY);
	}
	void PWMSweep(void) {
	    for (uint32_t f = 1000; f <= 20000; f += 500) {
	        PWM_SetFrequency(f);
	        frecuencia_generada = f;
	        mostrar_LCD();

	        char buf[64];
	        int n = snprintf(buf, sizeof(buf), "PWM Freq: %lu Hz\r\n", f);
	        HAL_UART_Transmit(&huart2, (uint8_t*)buf, n, HAL_MAX_DELAY);

	        HAL_Delay(3000); // como el blinky
	    }

	    PWM_SetFrequency(1000);
	    frecuencia_generada = 1000;
	    mostrar_LCD();
	}



