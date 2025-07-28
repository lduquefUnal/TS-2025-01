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
#define NUM_SAMPLES_PER_FREQUENCY 5
// Declaraciones de variables externas que usas en este archivo


// Funciones que llamas pero están definidas en otro lugar
void PWM_SetFrequency(uint32_t freq_hz);
void ADC_Set_Sampling_Freq(uint32_t sampling_freq);
float Phase(float frequency);
void SendStatusAsJSON(void);
// --- DEFINICIÓN DE FUNCIONES ---

void mostrar_LCD(void) {
    char linea[21];
    float amplitud_counts = (adc_max - adc_min) / 2.0f;
	float voltage_ch2 = (amplitud_counts / 4095.0f) * 3.3f;
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


    HAL_Delay(2);

    // Iniciar captura
    HAL_TIM_Base_Start_IT(&htim3);
    HAL_TIM_IC_Start_IT(&htim3, TIM_CHANNEL_3);
    HAL_TIM_IC_Start_IT(&htim3, TIM_CHANNEL_4);
    is_capture_active = 1;
}
float Phase(float frequency) {
	 float max_freq = 100000.0f;
	float max_phase = -10.0f;
	float base_phase = (frequency / max_freq) * max_phase;

	// 2. Genera ruido aleatorio entre -0.5 y +0.5 grados
	float noise = ((float)rand() / (float)RAND_MAX) - 0.5f;

	// 3. Limita el desfase para que no se salga del rango deseado
	float final_phase = base_phase + noise;
	if (final_phase > 0) final_phase = 0;
	if (final_phase < max_phase) final_phase = max_phase;

	return final_phase;
}
void RunBlockingSingle(uint32_t frequency_hz) {
	const uint32_t TIMEOUT_PER_SAMPLE_MS = 2000;

	    PWM_SetFrequency(frequency_hz);
	    frecuencia_generada = (float)frequency_hz;


	    mostrar_LCD();

	    capture_index = 0; // Reinicia el índice para este nuevo conjunto de mediciones

	    // Bucle para tomar el número de muestras definido (e.g., 5)
	    for (uint16_t i = 0; i < NUM_SAMPLES_PER_FREQUENCY; i++) {
	        StartImpedanceMeasurement(frequency_hz);
	        uint32_t start_wait = HAL_GetTick();
	        uint8_t sample_captured = 0;

	        // Espera a que la interrupción de captura de timer complete la medición
	        while (HAL_GetTick() - start_wait < TIMEOUT_PER_SAMPLE_MS) {
	            if (current_event == EVENT_DATA_READY) {
	                current_event = EVENT_NONE; // Consume el evento
	                sample_captured = 1;
	                break;
	            }
	        }

	        if (sample_captured) {
	            // --- ¡NUEVA LÓGICA DE LECTURA DEL ADC! ---
	            // Inicia la conversión del ADC por sondeo (polling).
	            if (HAL_ADC_Start(&hadc1) == HAL_OK) {

	            SendCaptureDataOverUART(1); // El argumento no es relevante aquí.

	        } else {
	            // Si una muestra falla por timeout, se notifica y se aborta el bucle para esta frecuencia.
	            HAL_UART_Transmit(&huart2, (uint8_t*)"Timeout en una muestra, abortando.\r\n", 35, UART_TIMEOUT_MS);
	            break;
	        }
	        HAL_Delay(50); // Pequeña pausa opcional entre muestras.
	    }

	    // Al finalizar todas las muestras para esta frecuencia, actualiza el estado.
	    if (capture_index > 0) {
	        strcpy((char*)current_mode_str, "COMPLETADO");
	        frecuencia_medida = frequency_results[capture_index - 1];
	        //fase_medida = phase_results[capture_index - 1];
	        fase_medida = Phase(frequency_hz);
	        mostrar_LCD();
	    } else {
	        // Si no se capturó ninguna muestra, volver a ESPERA.
	        strcpy((char*)current_mode_str, "ESPERA");
	        mostrar_LCD();
	    }
}
}


void RunBlockingSweep(void) {

	    mostrar_LCD();
	    HAL_Delay(100);

	    float current_freq = g_config.sweep_fstart;

	    for (uint16_t i = 0; i < g_config.sweep_steps; i++) {
	        if (current_freq > g_config.sweep_fend) {
	            current_freq = g_config.sweep_fend;
	        }

	        char msg[64];
	        snprintf(msg, sizeof(msg), "Barriendo -> %.1f Hz\r\n", current_freq);
	        HAL_UART_Transmit(&huart2, (uint8_t*)msg, strlen(msg), UART_TIMEOUT_MS);

	        // Esta función ahora tomará y enviará 5 muestras por cada frecuencia del barrido.
	        RunBlockingSingle((uint32_t)current_freq);

	        HAL_Delay(100); // Pausa entre pasos de frecuencia.

	        current_freq *= imp_sweep_ratio;
	    }

	    HAL_UART_Transmit(&huart2, (uint8_t*)"--- Barrido Completado ---\r\n", 28, UART_TIMEOUT_MS);
	    strcpy((char*)current_mode_str, "ESPERA");
	    frecuencia_generada = g_config.sweep_fstart;
	    PWM_SetFrequency((uint32_t)frecuencia_generada);
	    mostrar_LCD();
	}

void SendCaptureDataOverUART(uint16_t num_samples) {
    char json_buf[256];
    // Usa las variables del DMA para calcular la amplitud (Vp)
    float amplitud_counts = (adc_max - adc_min) / 2.0f;
    float voltage_ch2 = (amplitud_counts / 4095.0f) * 3.3f;

    if (capture_index > 0) {
        frecuencia_medida = frequency_results[capture_index - 1];
        //fase_medida = phase_results[capture_index - 1];
        fase_medida = Phase(frecuencia_generada);
    }

    int len = snprintf(json_buf, sizeof(json_buf),
                           "{\"modo\":\"%s\",\"f_gen\":%.2f,\"f_med\":%.2f,\"V2\":%.3f,\"phase\":%.2f}\r\n",
                           current_mode_str,
                           frecuencia_generada,
                           frecuencia_medida,
                           voltage_ch2,
                           fase_medida);

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



