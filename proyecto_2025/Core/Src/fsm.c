/*
 * fsm.c
 *
 *  Created on: Jul 4, 2025
 *      Author: luisduquefranco
 */

#include "main.h"
#include "fsm.h"
#include "i2c_lcd.h"
#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include "AD9833.h"
#include "utils.h"
/* ============================================================================
 * === VARIABLES EXTERNAS (Necesarias para que la FSM acceda a main.c) =======
 * ============================================================================*/


// Variables de la aplicación



// Variables del sistema de medición


/* ============================================================================
 * === VARIABLES PRIVADAS DE LA FSM ===========================================
 * ============================================================================*/

static volatile e_PosiblesStates current_state = STATE_IDLE;
static uint16_t print_index                  = 0;



/* ============================================================================
 * === FUNCIONES ==============================================================
 * ============================================================================*/

void mostrar_LCD(void) {
    char linea[21];
    float voltage_ch1, voltage_ch2; // Variables locales para el cálculo

    // --- MUEVE EL CÁLCULO AQUÍ ---
    // Convierte el valor máximo del ADC a voltaje (Vref = 3.3V, 12-bit)
    voltage_ch1 = (adc_max_ch1 / 4095.0f) * 3.3f;
    voltage_ch2 = (adc_max_ch2 / 4095.0f) * 3.3f;

    lcd_clear(&hlcd);

    snprintf(linea, sizeof(linea), "MODO: %s", current_mode_str);
    lcd_gotoxy(&hlcd, 0, 0);
    lcd_puts(&hlcd, linea);

    lcd_gotoxy(&hlcd, 0, 1);
    snprintf(linea, sizeof(linea), "F_GEN: %.1f Hz", frecuencia_generada);
    lcd_puts(&hlcd, linea);

    lcd_gotoxy(&hlcd, 0, 2);
    snprintf(linea, sizeof(linea), "F_MED: %.1f Hz", frecuencia_medida);
    lcd_puts(&hlcd, linea);

    lcd_gotoxy(&hlcd, 0, 3);
    snprintf(linea, sizeof(linea), "V1:%.2fV V2:%.2fV", voltage_ch1, voltage_ch2);
    lcd_puts(&hlcd, linea);
}

static void handle_uart_command(const char *cmd) {
    if (strncmp(cmd, "measure=", 8) == 0) {
        uint32_t freq = atoi(cmd + 8);
        if (freq > 0) {
        	strcpy(current_mode_str, "MIDIENDO");

            snprintf((char*)tx_buffer, sizeof(tx_buffer), "Iniciando medicion a %lu Hz...\r\n", freq);
            UART_DMA_Enqueue(&huart2, tx_buffer, strlen((char*)tx_buffer));
            StartImpedanceMeasurement(freq);
        }
    }
    else if (strncmp(cmd, "samples=", 8) == 0) {
        uint16_t size = atoi(cmd + 8);
        if (size > 0 && size <= FFT_SIZE_MAX) {
            num_samples_to_capture = size;
            snprintf((char*)tx_buffer, sizeof(tx_buffer), "Tamaño de muestra: %u\r\n", size);
            UART_DMA_Enqueue(&huart2, tx_buffer, strlen((char*)tx_buffer));
        } else {
            snprintf((char*)tx_buffer, sizeof(tx_buffer), "Tamaño invalido (1-%d)\r\n", FFT_SIZE_MAX);
            UART_DMA_Enqueue(&huart2, tx_buffer, strlen((char*)tx_buffer));
        }
    }
    else if (strncmp(cmd, "print_data@", 11) == 0) {
    	if (capture_index > 0) {
    	        char buf[100];
    	        // Convierte los valores maximos del ADC a voltaje
    	        float last_v1 = (adc_max_ch1 / 4095.0f) * 3.3f;
    	        float last_v2 = (adc_max_ch2 / 4095.0f) * 3.3f;
    	        float last_phase = phase_results[capture_index - 1];
    	        float last_freq = frequency_results[capture_index - 1];

    	        // Formato: freq,fase,V1,V2
    	        snprintf(buf, sizeof(buf), "%.2f,%.2f,%.2f,%.2f\r\n",
    	                 last_freq,
    	                 last_phase,
    	                 last_v1,
    	                 last_v2);
    	        UART_DMA_Enqueue(&huart2, (uint8_t*)buf, strlen(buf));
        } else {
            UART_DMA_Enqueue(&huart2, (uint8_t*)"No hay datos capturados.\r\n", 28);
        }
    }
    else if (strncmp(cmd, "help@", 5) == 0) {
    	const char *msg =
    	        "\r\n--- Comandos Disponibles ---\r\n"
    	        "measure=<Hz>@            : Inicia una medicion a la frecuencia dada\r\n"
    	        "samples=<num>@           : Fija el numero de muestras a capturar\r\n"
    	        "print_data@              : Imprime la ultima captura realizada\r\n"
    	        "ad9833_mode=0@           : Cambia a seno (0=seno, 1=cuadrada, 2=triangular)\r\n"
    	        "sweep_ad9833@            : Barrido de frecuencia con AD9833\r\n"
    	        "sweep_pwm@               : Barrido de frecuencia PWM en PA9\r\n"
    	        "help@                    : Muestra esta ayuda\r\n";
    	    UART_DMA_Enqueue(&huart2, (uint8_t*)msg, strlen(msg));
    }else if (strncmp(cmd, "ad9833_mode=", 13) == 0) {
        int mode = atoi(cmd + 13);
        if (mode >= 0 && mode <= 2) {
            AD9833_SetWaveform((uint8_t)mode);
            const char *modo_str[] = {"senoidal", "cuadrada", "triangular"};
            snprintf((char*)tx_buffer, sizeof(tx_buffer), "Modo AD9833 cambiado a %s (%d)\r\n", modo_str[mode], mode);
            UART_DMA_Enqueue(&huart2, tx_buffer, strlen((char*)tx_buffer));
        } else {
            UART_DMA_Enqueue(&huart2, (uint8_t*)"Modo invalido (0=seno, 1=cuadrada, 2=triangular)\r\n", 51);
        }
    }else if (strcmp(cmd, "sweep_ad9833@") == 0) {
    	strcpy(current_mode_str, "BARRIDO AD9833");
        UART_DMA_Enqueue(&huart2, (uint8_t*)"Barrido AD9833...\r\n", 20);
        LogSweep();
        strcpy(current_mode_str, "ESPERA");
    }
    else if (strcmp(cmd, "sweep_pwm@") == 0) {
    	strcpy(current_mode_str, "BARRIDO PWM");
        UART_DMA_Enqueue(&huart2, (uint8_t*)"Barrido PWM...\r\n", 16);
        PWMSweep();
        strcpy(current_mode_str, "ESPERA");
    }
    else {
        UART_DMA_Enqueue(&huart2, (uint8_t*)"Comando no reconocido. Usa help@\r\n", 34);

    }
}


void init_fsm(void) {
    current_state = STATE_IDLE;
}

e_PosiblesStates state_machine_action(e_PosiblesEvents event) {
    switch (event) {
        case EVENT_USART_COMMAND:
            handle_uart_command(rx_buffer);
            memset(rx_buffer, 0, sizeof(rx_buffer));
            rx_index = 0;
            break;

        case EVENT_DATA_READY:
        	strcpy(current_mode_str, "ESPERA");
            UART_DMA_Enqueue(&huart2, (uint8_t*)"FSM: Captura completa.\r\n", 26);
            current_state = STATE_IDLE;
            break;

        case EVENT_PRINT_NEXT_DATA:
            if (print_index < capture_index) {
                char buf[80];
                snprintf(buf, sizeof(buf), "Muestra[%04u]: Freq=%.2f Hz, Fase=%.2f deg\r\n",
                         print_index,
                         frequency_results[print_index],
                         phase_results[print_index]);
                UART_DMA_Enqueue(&huart2, (uint8_t*)buf, strlen(buf));
                print_index++;

                if (print_index < capture_index) {
                    if (current_event == EVENT_NONE) {
                        current_event = EVENT_PRINT_NEXT_DATA;
                    }
                } else {
                    UART_DMA_Enqueue(&huart2, (uint8_t*)"--- Fin de Datos ---\r\n", 22);
                    current_state = STATE_IDLE;
                }
            }
            break;

        default:
            current_state = STATE_IDLE;
            break;
    }
    return current_state;
}
