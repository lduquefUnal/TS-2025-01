/*
 * commands.c
 *
 *  Created on: Jul 4, 2025
 *      Author: luisduquefranco
 */

#include "main.h"
#include "MPU6050.h"
#include <string.h>
#include <stdio.h>
#include <stdlib.h>

// === Variables externas ===

extern UART_HandleTypeDef huart2;
extern uint16_t fft_size;
extern arm_rfft_fast_instance_f32 fft_instance;

// Función para verificar si un número es potencia de 2
static int is_power_of_two(uint16_t n) {
    return (n != 0) && ((n & (n - 1)) == 0);
}
// === Comando: led=<ms>@ ===
void Cmd_HandleLEDDelayCmd(const char *arg) {
    uint32_t nuevo = atoi(arg);
    __HAL_TIM_SET_AUTORELOAD(&htim2, nuevo);
    __HAL_TIM_SET_COUNTER(&htim2, 0);

    int len = snprintf((char*)tx_buffer, sizeof(tx_buffer),
                       "LED delay = %lu ms\r\n", (unsigned long)nuevo);
    HAL_UART_Transmit(&huart2, (uint8_t*)tx_buffer, len, 1000);
}

// === Comando: print@ ===
// Imprime Ax, Ay, Az en g (aceleración)
void Cmd_HandlePrintADC(void) {
    int len = snprintf((char*)tx_buffer, sizeof(tx_buffer),
        "AccelX: %.3f g\r\nAccelY: %.3f g\r\nAccelZ: %.3f g\r\n",
        Ax, Ay, Az);
    HAL_UART_Transmit(&huart2, (uint8_t*)tx_buffer, len, 1000);
}

// === Comando: fftSize=1 o fftSize=2 ===
// Cambia el tamaño de la FFT futura (por eje Z)
void Cmd_HandleFFTSizeCmd(const char *arg) {
    char option = arg[0];
    if (option == '1') {
        fft_size = 1024;
    } else if (option == '2') {
        fft_size = 2048;
    } else {
        const char *msg = "Opciones válidas para 'fftSize=' son:\r\n"
                          "1 -> 1024 puntos\r\n"
                          "2 -> 2048 puntos\r\n";
        HAL_UART_Transmit(&huart2, (uint8_t*)msg, strlen(msg), 1000);
        return;
    }

    int len = snprintf((char*)tx_buffer, sizeof(tx_buffer),
                       "FFT size set to %u\r\n", fft_size );
    HAL_UART_Transmit(&huart2, (uint8_t*)tx_buffer, len, 1000);
}

// === Comando: status@ ===
// Información del sistema
void Cmd_HandleStatusCmd(void) {
    // Obtenemos la frecuencia del reloj del sistema (HCLK) de forma dinámica
    uint32_t sys_clock_freq = HAL_RCC_GetHCLKFreq();

    // Frecuencia de muestreo del MPU6050, definida en la configuración del sensor
    const float sample_rate = 1000.0f;

    int len = snprintf((char*)tx_buffer, 256,
        "====== ESTADO DEL SISTEMA ======\r\n"
        "--- Reloj ---\r\n"
        "  CPU (HCLK): %lu Hz\r\n"
        "--- FFT ---\r\n"
        "  Tamano FFT: %u puntos\r\n"
        "  Sample Rate: %.1f Hz\r\n"
        "--- Timers ---\r\n"
        "  TIM2 (LED) Prescaler: %lu\r\n"
        "  TIM2 (LED) Period: %lu\r\n"
        "  TIM4 (LCD) Prescaler: %lu\r\n"
        "  TIM4 (LCD) Period: %lu\r\n"
        "================================\r\n",
        sys_clock_freq,
        fft_size,
        sample_rate,
        htim2.Init.Prescaler,
        htim2.Init.Period,
        htim4.Init.Prescaler,
        htim4.Init.Period
    );

    HAL_UART_Transmit(&huart2, (uint8_t*)tx_buffer, len, 1000);
}
// === Comando: help@ ===
// Lista de comandos disponibles
void Cmd_HandleHelpCmd(void) {
    const char *help_msg =
        "\r\n========== AYUDA ==========\r\n"
        "Comandos disponibles:\r\n"
        "  led=<ms>@        - Cambia la frecuencia del LED\r\n"
        "  print@           - Imprime datos del MPU6050 (Ax, Ay, Az)\r\n"
        "  fftSize=<1|2>@   - Cambia el tamaño de la FFT (1:1024, 2:2048)\r\n"
        "  status@          - Estado actual del sistema\r\n"
        "  help@            - Muestra esta ayuda\r\n"
        "===========================\r\n";
    HAL_UART_Transmit(&huart2, (uint8_t*)help_msg, strlen(help_msg), 1000);
}
