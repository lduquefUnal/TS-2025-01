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

// ============================================================================
// === VARIABLES EXTERNAS (Necesarias para que la FSM acceda a main.c) ========
// ============================================================================
static volatile e_PosiblesStates current_state = STATE_IDLE;
static uint16_t print_index                = 0;

static volatile uint32_t ic_ref, ic_sig, last_ref;
static volatile uint8_t  got_ref, got_sig;
volatile float frecuencia_medida = 0.0f;
volatile float fase_medida       = 0.0f;

#define IMP_SWEEP_STEPS   10
#define IMP_SWEEP_START_HZ   10.0f
#define IMP_SWEEP_END_HZ   100000.0f

static float  imp_sweep_ratio;
static uint8_t imp_sweep_index;
static void send_status(void);

#define IMP_SWEEP_STEPS   10
#define IMP_SWEEP_START_HZ   10.0f
#define IMP_SWEEP_END_HZ   100000.0f

static float  imp_sweep_ratio;
static uint8_t imp_sweep_index;
static void send_status(void);

// ============================================================================
// === FUNCIONES ==============================================================
// ============================================================================

void mostrar_LCD(void) {
    char linea[21];
    float voltage_ch1, voltage_ch2;

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

    snprintf(linea, sizeof(linea), "V2:%.2fV P:%.1fdeg", voltage_ch2, fase_medida);
    lcd_gotoxy(&hlcd, 0, 3);
    lcd_puts  (&hlcd, linea);
}

static void handle_uart_command(const char *cmd) {
    char local_buf[128];


    uint32_t start = HAL_GetTick();
    while (tx_dma_busy && (HAL_GetTick() - start) < 100) {
        // Timeout de 100ms
    }

    snprintf(local_buf, sizeof(local_buf), "Comando recibido: %s\r\n", cmd);
    if (HAL_UART_Transmit(&huart2, (uint8_t*)local_buf, strlen(local_buf), HAL_MAX_DELAY) != HAL_OK) {
        // Error al encolar, limpiar buffer DMA
        tx_head = tx_tail = 0;
        tx_dma_busy = 0;
    }

    if (strncmp(cmd, "measure=", 8) == 0) {
        uint32_t freq = atoi(cmd + 8);
        if (freq > 0) {
            strcpy(current_mode_str, "MIDIENDO");
            snprintf(local_buf, sizeof(local_buf), "Iniciando medicion a %lu Hz...\r\n", freq);
            HAL_UART_Transmit(&huart2, (uint8_t*)local_buf, strlen(local_buf), HAL_MAX_DELAY);
            StartImpedanceMeasurement(freq);

            fase_medida = 45.0f;         // Dato ficticio
             mostrar_LCD();
        }
    }
    else if (strncmp(cmd, "samples=", 8) == 0) {
        uint16_t size = atoi(cmd + 8);
        if (size > 0 && size <= FFT_SIZE_MAX) {
            num_samples_to_capture = size;
            snprintf(local_buf, sizeof(local_buf), "Tamaño de muestra: %u\r\n", size);
            HAL_UART_Transmit(&huart2, (uint8_t*)local_buf, strlen(local_buf), HAL_MAX_DELAY);
        } else {
            snprintf(local_buf, sizeof(local_buf), "Tamaño invalido (1-%d)\r\n", FFT_SIZE_MAX);
            HAL_UART_Transmit(&huart2, (uint8_t*)local_buf, strlen(local_buf), HAL_MAX_DELAY);
        }
    }
    else if (strcmp(cmd, "print_data") == 0) {
        if (capture_index > 0) {
            char buf[100];
            float last_v1 = (adc_max_ch1 / 4095.0f) * 3.3f;
            float last_v2 = (adc_max_ch2 / 4095.0f) * 3.3f;
            float last_phase = phase_results[capture_index - 1];
            float last_freq = frequency_results[capture_index - 1];

            snprintf(buf, sizeof(buf), "%.2f,%.2f,%.2f,%.2f\r\n",
                     last_freq,
                     last_phase,
                     last_v1,
                     last_v2);
            HAL_UART_Transmit(&huart2, (uint8_t*)buf, strlen(buf), HAL_MAX_DELAY);
        } else {
            HAL_UART_Transmit(&huart2, (uint8_t*)"No hay datos capturados.\r\n", 28, HAL_MAX_DELAY);
        }
    }
    else if (strcmp(cmd, "help") == 0) {
        const char *msg =
            "\r\n--- Comandos Disponibles ---\r\n"
            "measure=<Hz>@            : Inicia una medicion a la frecuencia dada\r\n"
            "samples=<num>@           : Fija el numero de muestras a capturar\r\n"
            "print_data@              : Imprime la ultima captura realizada\r\n"
            "ad9833_mode=0@           : Cambia a seno (0=seno, 1=cuadrada, 2=triangular)\r\n"
            "sweep_ad9833@            : Barrido de frecuencia con AD9833\r\n"
            "sweep_pwm@               : Barrido de frecuencia PWM en PA9\r\n"
            "status@                  : Información de interes\r\n"
            "help@                    : Muestra esta ayuda\r\n";
        HAL_UART_Transmit(&huart2, (uint8_t*)msg, strlen(msg), HAL_MAX_DELAY);
    }
    else if (strncmp(cmd, "ad9833_mode=", 13) == 0) {
        int mode = atoi(cmd + 13);
        if (mode >= 0 && mode <= 2) {
            AD9833_SetWaveform((uint8_t)mode);
            const char *modo_str[] = {"senoidal", "cuadrada", "triangular"};
            snprintf(local_buf, sizeof(local_buf), "Modo AD9833 cambiado a %s (%d)\r\n", modo_str[mode], mode);
            HAL_UART_Transmit(&huart2, (uint8_t*)local_buf, strlen(local_buf), HAL_MAX_DELAY);
        } else {
            HAL_UART_Transmit(&huart2, (uint8_t*)"Modo invalido (0=seno, 1=cuadrada, 2=triangular)\r\n", 51, HAL_MAX_DELAY);
        }
    }
    else if (strcmp(cmd, "sweep_ad9833") == 0) {
        strcpy(current_mode_str, "BARRIDO AD9833");
        HAL_UART_Transmit(&huart2, (uint8_t*)"Barrido AD9833...\r\n", 20, HAL_MAX_DELAY);
        LogSweep();
        strcpy(current_mode_str, "ESPERA");
    }
    else if (strcmp(cmd, "sweep_pwm") == 0) {
        strcpy(current_mode_str, "BARRIDO PWM");
        HAL_UART_Transmit(&huart2, (uint8_t*)"Barrido PWM...\r\n", 16, HAL_MAX_DELAY);
        PWMSweep();
        strcpy(current_mode_str, "ESPERA");
    }
    else if (strcmp(cmd, "sweep_imp") == 0) {
        current_event = EVENT_IMP_SWEEP_START;
    }
    else if (strcmp(cmd, "status") == 0) {
        send_status();
    }
    else {
        HAL_UART_Transmit(&huart2, (uint8_t*)"Comando no reconocido. Usa help@\r\n", 34, HAL_MAX_DELAY);
    }
}

void init_fsm(void) {
    current_state = STATE_IDLE;
    imp_sweep_ratio = powf(IMP_SWEEP_END_HZ/IMP_SWEEP_START_HZ,
                           1.0f/(IMP_SWEEP_STEPS-1));
    imp_sweep_index = 0;
}

e_PosiblesStates state_machine_action(e_PosiblesEvents event) {
    switch (current_state) {
        case STATE_IDLE:
            if (event == EVENT_USART_COMMAND) {
                // CAMBIO PRINCIPAL: Usar command_buffer en lugar de rx_buffer
                char cmd_copy[100];
                strncpy(cmd_copy, (char*)command_buffer, sizeof(cmd_copy));
                current_event = EVENT_NONE;
                memset((void*)command_buffer, 0, sizeof(command_buffer));

                if (strlen(cmd_copy) > 0) {
                    handle_uart_command(cmd_copy);
                }
            }
            else if (event == EVENT_IMP_SWEEP_START) {
                imp_sweep_index = 0;
                strcpy(current_mode_str, "SWEEP IMP");
                HAL_UART_Transmit(&huart2, (uint8_t*)"Iniciando sweep impedancia\r\n", 27, HAL_MAX_DELAY);
                float f = IMP_SWEEP_START_HZ * powf(imp_sweep_ratio, (float)imp_sweep_index);
                frecuencia_generada = f;
                PWM_SetFrequency((uint32_t)f);
                StartImpedanceMeasurement((uint32_t)f);
                current_state = STATE_IMP_SWEEP_RUNNING;
            }
            break;

        case STATE_IMP_SWEEP_RUNNING:
            if (event == EVENT_DATA_READY) {
                mostrar_LCD();
                char buf[64];
                int n = snprintf(buf, sizeof(buf),
                        "Paso %u/%u: F:%.1fHz P:%.1fdeg\r\n",
                        imp_sweep_index + 1, IMP_SWEEP_STEPS,
                        frecuencia_medida, fase_medida);
                HAL_UART_Transmit(&huart2, (uint8_t*)buf, n, HAL_MAX_DELAY);

                imp_sweep_index++;

                if (imp_sweep_index < IMP_SWEEP_STEPS) {
                    float f2 = IMP_SWEEP_START_HZ * powf(imp_sweep_ratio, (float)imp_sweep_index);
                    frecuencia_generada = f2;
                    PWM_SetFrequency((uint32_t)f2);
                    StartImpedanceMeasurement((uint32_t)f2);
                } else {
                    PWM_SetFrequency(1000);
                    frecuencia_generada = 1000.0f;
                    strcpy(current_mode_str, "ESPERA");
                    HAL_UART_Transmit(&huart2, (uint8_t*)"Sweep finalizado\r\n", 19, HAL_MAX_DELAY);
                    mostrar_LCD();
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

static void send_status(void) {
    char buf[256];

    // Lectura de registros de TIM1 (PWM)
    uint32_t pwm_psc = htim1.Instance->PSC;
    uint32_t pwm_arr = htim1.Instance->ARR;

    // Lectura de registros de TIM2 (trigger ADC)
    uint32_t adc_psc = htim2.Instance->PSC;
    uint32_t adc_arr = htim2.Instance->ARR;

    // Lectura de registros de TIM3 (Input Capture)
    uint32_t ic_psc  = htim3.Instance->PSC;
    uint32_t ic_arr  = htim3.Instance->ARR;

    int len = snprintf(buf, sizeof(buf),
        "\r\n--- STATUS SWEEP IMPEDANCIA ---\r\n"
        "Pasos sweep:    %u\r\n"
        "Fstart:         %.1fHz  Fend: %.1fHz\r\n"
        "Muestras:       %u\r\n"
        "PWM (TIM1 CH1): PSC=%lu ARR=%lu → Fout≈%.1fHz\r\n"
        "ADC trig (TIM2):PSC=%lu ARR=%lu\r\n"
        "IC (TIM3):      PSC=%lu ARR=%lu\r\n"
        "Última Fgen:    %.1fHz\r\n"
        "Última Fmed:    %.1fHz P:%.1fdeg\r\n\r\n",
        IMP_SWEEP_STEPS,
        IMP_SWEEP_START_HZ, IMP_SWEEP_END_HZ,
        num_samples_to_capture,
        pwm_psc, pwm_arr, frecuencia_generada,
        adc_psc, adc_arr,
        ic_psc,  ic_arr,
        frecuencia_generada,
        frecuencia_medida,
        fase_medida
    );
    HAL_UART_Transmit(&huart2, (uint8_t*)buf, len, HAL_MAX_DELAY);
}
