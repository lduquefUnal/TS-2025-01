/*
 * utils.c
 *
 * Created on: Jul 23, 2025
 * Author: luisduquefranco
 */
#include "utils.h"
#include "main.h"
#include "AD9833.h"
// No es necesario incluir fsm.h aquí si no usas sus tipos de FSM
#include <stdio.h>
#include <string.h>
#include <math.h>

// Declaraciones de variables externas que usas en este archivo


// Funciones que llamas pero están definidas en otro lugar
void PWM_SetFrequency(uint32_t freq_hz);
void ADC_Set_Sampling_Freq(uint32_t sampling_freq);
float phase(float frequency);
void SendStatusAsJSON(void);
// --- DEFINICIÓN DE FUNCIONES ---

void mostrar_LCD(void) {
    char linea[21];
    float voltage_ch2 = (adc_max_ch2 / 4095.0f) * 3.3f;
    lcd_clear(&hlcd);
    snprintf(linea, sizeof(linea), "MODO: %s", current_mode_str);
    lcd_gotoxy(&hlcd, 0, 0);
    lcd_puts(&hlcd, linea);
    snprintf(linea, sizeof(linea), "F_GEN: %.1f Hz", frecuencia_generada);
    lcd_gotoxy(&hlcd, 0, 1);
    lcd_puts(&hlcd, linea);
    snprintf(linea, sizeof(linea), "F_MED: %.1f Hz", frecuencia_medida);
    lcd_gotoxy(&hlcd, 0, 2);
    lcd_puts(&hlcd, linea);
    snprintf(linea, sizeof(linea), "V2:%.2fV P:%.1fdeg", voltage_ch2, fase_medida);
    lcd_gotoxy(&hlcd, 0, 3);
    lcd_puts(&hlcd, linea);
}

void StartImpedanceMeasurement(uint32_t frequency) {
    // Detener capturas previas
    HAL_TIM_Base_Stop(&htim3);
    HAL_TIM_IC_Stop_IT(&htim3, TIM_CHANNEL_3);
    HAL_TIM_IC_Stop_IT(&htim3, TIM_CHANNEL_4);

    // Reiniciar contadores y flags (asegúrate de que estas variables estén declaradas globalmente)
    __HAL_TIM_SET_COUNTER(&htim3, 0);
    counterOverflow = 0;
    cycle_count = 0;
    got_ref = got_sig = 0;
    capture_flags = 0;

    PWM_SetFrequency(frequency);

    // Configurar frecuencia de muestreo (opcional si no usas ADC para esto)
    // uint32_t adc_freq = frequency * 50;
    // ADC_Set_Sampling_Freq(adc_freq);

    // Pequeña pausa para estabilización
    HAL_Delay(2);

    // Iniciar captura
    HAL_TIM_Base_Start_IT(&htim3);
    HAL_TIM_IC_Start_IT(&htim3, TIM_CHANNEL_3);
    HAL_TIM_IC_Start_IT(&htim3, TIM_CHANNEL_4);
    is_capture_active = 1;
}
float Phase(float frequency) {
    const float R = 1000.0f;
    const float C = 0.000001f;
    float phase_rad = atan2(-2.0f * M_PI * frequency * R * C, 1.0f);
    return phase_rad * 180.0f / M_PI;
}
void RunBlockingSingle(uint32_t frequency_hz) {
    const uint32_t TIMEOUT_PER_SAMPLE_MS = 2000;

    PWM_SetFrequency(frequency_hz);
    frecuencia_generada = (float)frequency_hz;
    strcpy((char*)current_mode_str, "MIDIENDO");
    mostrar_LCD();

    capture_index = 0;

    for (uint16_t i = 0; i < num_samples_to_capture; i++) {
        StartImpedanceMeasurement(frequency_hz);
        uint32_t start_wait = HAL_GetTick();
        uint8_t sample_captured = 0;
        current_event = EVENT_NONE;

        while (HAL_GetTick() - start_wait < TIMEOUT_PER_SAMPLE_MS) {
            if (current_event == EVENT_DATA_READY) {
                current_event = EVENT_NONE;
                sample_captured = 1;
                break;
            }
        }
        if (!sample_captured) {
            HAL_UART_Transmit(&huart2, (uint8_t*)"Timeout en muestra, abortando.\r\n", 32, UART_TIMEOUT_MS );
            break;
        }
    }

    if (capture_index > 0) {
        frecuencia_medida = frequency_results[capture_index - 1];
        //fase_medida = Phase((float)frequency_hz);
        fase_medida = phase_results[capture_index - 1];
        strcpy((char*)current_mode_str, "COMPLETADO");
        mostrar_LCD();
        SendCaptureDataOverUART(capture_index);
    }
    HAL_Delay(500);
}


void RunBlockingSweep(void) {
    strcpy((char*)current_mode_str, "BARRIDO...");
    mostrar_LCD();
    HAL_Delay(100); // Pequeña pausa para que se actualice el LCD

    float current_freq = g_config.sweep_fstart;

    // Bucle para realizar el barrido de frecuencia
    for (uint16_t i = 0; i < g_config.sweep_steps; i++) {
        // Asegurarse de no exceder la frecuencia final
        if (current_freq > g_config.sweep_fend) {
            current_freq = g_config.sweep_fend;
        }

        char msg[64];
        snprintf(msg, sizeof(msg), "Barriendo -> %.1f Hz\r\n", current_freq);
        HAL_UART_Transmit(&huart2, (uint8_t*)msg, strlen(msg), UART_TIMEOUT_MS );

        // Ejecutar una medición única y bloqueante para la frecuencia actual
        RunBlockingSingle((uint32_t)current_freq);

        // Pequeña pausa entre mediciones
        HAL_Delay(500);

        // Calcular la siguiente frecuencia en la escala logarítmica
        current_freq *= imp_sweep_ratio;
    }

    // El barrido ha finalizado
    HAL_UART_Transmit(&huart2, (uint8_t*)"--- Barrido Completado ---\r\n", 28, UART_TIMEOUT_MS );
    strcpy((char*)current_mode_str, "ESPERA");
    frecuencia_generada = g_config.sweep_fstart; // Volver a una frecuencia por defecto
    PWM_SetFrequency((uint32_t)frecuencia_generada);
    mostrar_LCD();
}

void SendCaptureDataOverUART(uint16_t num_samples) {
	char json_buf[256];
	float voltage_ch2 = (adc_max_ch2 / 4095.0f) * 3.3f;
    int len = snprintf(json_buf, sizeof(json_buf),
	                       "{\"modo\":\"%s\",\"f_gen\":%.2f,\"f_med\":%.2f,\"V2\":%.3f,\"phase\":%.2f}\r\n",
	                       current_mode_str,
	                       frecuencia_generada,
	                       frecuencia_medida,
	                       voltage_ch2,
	                       fase_medida);

	    // Transmite la cadena JSON por el puerto UART.
	    if (len > 0 && len < sizeof(json_buf)) {
	        HAL_UART_Transmit(&huart2, (uint8_t*)json_buf, len, HAL_MAX_DELAY);
	    }
}

void I2C_Scanner(void) {

char uart_buf[64];

HAL_StatusTypeDef res;


HAL_UART_Transmit(&huart2, (uint8_t*)"Escaneando bus I2C...\r\n", 24, UART_TIMEOUT_MS );


for (uint16_t i = 1; i < 128; i++) {

// La función HAL espera la dirección de 8 bits (dirección de 7 bits << 1)

res = HAL_I2C_IsDeviceReady(&hi2c1, (i << 1), 1, 10);

if (res == HAL_OK) {

sprintf(uart_buf, "Dispositivo encontrado en la direccion 0x%X\r\n", i);

HAL_UART_Transmit(&huart2, (uint8_t*)uart_buf, strlen(uart_buf), UART_TIMEOUT_MS );

}

}

HAL_UART_Transmit(&huart2, (uint8_t*)"Escaneo finalizado.\r\n", 20, UART_TIMEOUT_MS );

}

void SPI_Debug_AD9833(void) {
	char uart_buf[64];
	HAL_StatusTypeDef res;
	uint8_t test_data[2] = {0x21, 0x00}; // Comando RESET para AD9833

	HAL_UART_Transmit(&huart2, (uint8_t*)"Verificando AD9833 por SPI...\r\n", 32, UART_TIMEOUT_MS );

	HAL_GPIO_WritePin(FSYNC_GPIO_Port, FSYNC_Pin , GPIO_PIN_RESET);

	res = HAL_SPI_Transmit(&hspi1, test_data, 2, UART_TIMEOUT_MS );
	HAL_GPIO_WritePin(FSYNC_GPIO_Port, FSYNC_Pin , GPIO_PIN_SET);
	if (res == HAL_OK) {
	sprintf(uart_buf, "AD9833 responde OK por SPI\r\n");
	} else {
	sprintf(uart_buf, "Fallo comunicacion SPI con AD9833\r\n");
	}

	HAL_UART_Transmit(&huart2, (uint8_t*)uart_buf, strlen(uart_buf), UART_TIMEOUT_MS );
}



