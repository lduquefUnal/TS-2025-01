/*
 * fsm.c
 *
 * Created on: Jul 4, 2025
 * Author: luisduquefranco
 */
#include "main.h"
#include "utils.h"
#include "fsm.h"
#include "i2c_lcd.h"
#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include "AD9833.h"

#include <math.h>

#define SAMPLES_PER_STEP 5 // Promediar 10 mediciones por punto

// --- Variables moved here for broader scope if needed, or keep static ---
static uint8_t imp_sweep_index = 0;
float   imp_sweep_ratio = 0;
// Variables for averaging (used for both sweep steps and single measurement if averaging is desired)
static uint16_t current_sample_count = 0;
static float    accumulated_phase = 0.0f;
static float    accumulated_freq = 0.0f;
// -------------------------------------------------------------------------

// ============================================================================
// === ESTRUCTURA DE CONFIGURACIÓN Y PARÁMETROS ===============================
// ============================================================================
 MeasurementConfig_t g_config;

// ============================================================================
// === VARIABLES DE ESTADO Y GLOBALES DEL MÓDULO ==============================
// ============================================================================
// --- Ensure these are declared appropriately, likely as extern in fsm.h if used elsewhere ---

extern volatile e_PosiblesEvents current_event; // Assuming this is defined elsewhere
// --- End of external assumptions ---

static volatile e_PosiblesStates current_state = STATE_IDLE;
volatile float frecuencia_medida = 0.0f;
volatile float fase_medida       = 0.0f;

// --- Remove duplicate declarations ---
// static float   imp_sweep_ratio; // Already declared above
// static uint8_t imp_sweep_index; // Already declared above
// -------------------------------------------------------------------------

// ============================================================================
// === DECLARACIONES DE FUNCIONES PRIVADAS ====================================
// ============================================================================
static void send_status(void);
static void handle_uart_command(const char *cmd);
static void send_sweep_point_json(uint8_t step, uint8_t total_steps, float freq, float phase);
// --- Add declaration for LCD function if not in header ---
void mostrar_LCD(void); // Assuming this is implemented elsewhere, declare if needed
// --- Add declaration for measurement function ---
void StartImpedanceMeasurement(uint32_t freq); // Assuming this is implemented elsewhere, declare if needed
void PWM_SetFrequency(uint32_t freq);          // Assuming this is implemented elsewhere, declare if needed
// -------------------------------------------------------------------------

// ============================================================================
// === IMPLEMENTACIÓN DE FUNCIONES PÚBLICAS ===================================
// ============================================================================

void init_fsm(void) {
    // Valores por defecto al iniciar el sistema
    g_config.samples = 512;
    g_config.sweep_steps = 10;
    g_config.sweep_fstart = 1000.0f;
    g_config.sweep_fend = 100000.0f;
    current_state = STATE_IDLE;
    // El ratio se calcula una vez en init o cada vez que cambien las frecuencias/pasos
    if (g_config.sweep_steps > 1) {
        imp_sweep_ratio = powf(g_config.sweep_fend / g_config.sweep_fstart, 1.0f / (g_config.sweep_steps - 1));
    } else {
        imp_sweep_ratio = 1.0f; // Avoid division by zero
    }
    imp_sweep_index = 0;
    current_sample_count = 0;
    accumulated_phase = 0.0f;
    accumulated_freq = 0.0f;
}

// Assuming mostrar_LCD is implemented elsewhere, kept for completeness


e_PosiblesStates state_machine_action(e_PosiblesEvents event) {
    // Con el enfoque de funciones bloqueantes, la FSM solo tiene UNA tarea.
    if (current_state == STATE_IDLE && event == EVENT_USART_COMMAND) {

        // 1. Copiar y limpiar el buffer de comando
        char cmd_copy[100];
        strncpy(cmd_copy, (const char*)command_buffer, sizeof(cmd_copy) - 1);
        cmd_copy[sizeof(cmd_copy) - 1] = '\0';
        current_event = EVENT_NONE; // Consumir el evento inmediatamente
        memset((void*)command_buffer, 0, sizeof(command_buffer));

        // 2. Llamar al manejador que hace todo el trabajo pesado
        if (strlen(cmd_copy) > 0) {
            handle_uart_command(cmd_copy);
        }
    }

    // La FSM nunca cambia de estado porque las funciones bloqueantes
    // se encargan de todo el proceso de principio a fin antes de devolver el control.
    return STATE_IDLE;
}

// ============================================================================
// === IMPLEMENTACIÓN DE FUNCIONES PRIVADAS ===================================
// ============================================================================

/**
 * @brief Procesa los comandos de texto recibidos por UART.
 * @param cmd La cadena de texto del comando (sin el '@' final).
 */
static void handle_uart_command(const char *cmd) {
    char response_buf[128];
    // --- Comando: Medición Única ("m <frecuencia>") ---
    if (strncmp(cmd, "m ", 2) == 0) {
        uint32_t freq = atoi(cmd + 2);
        if (freq > 0 && freq <= 20000) { // Limita la frecuencia si es necesario
        	strcpy((char*)current_mode_str, "simple");
            RunBlockingSingle(freq);
        } else {
             const char* invalid_freq_msg = "Error: Frecuencia invalida.\r\n";
             HAL_UART_Transmit(&huart2, (uint8_t*)invalid_freq_msg, strlen(invalid_freq_msg), HAL_MAX_DELAY);
             strcpy(current_mode_str, "ESPERA"); // Reset mode on error
             mostrar_LCD();
        }
    }
    // --- Comando: Configurar Parámetros ("config <param> <valor>") ---
    else if (strncmp(cmd, "config ", 7) == 0) {
        char param[20];
        char value_str[20];
        // sscanf is ideal for parsear "palabra valor"
        if (sscanf(cmd + 7, "%19s %19s", param, value_str) == 2) {
            if (strcmp(param, "samples") == 0) {
                const uint16_t valid_sizes[] = {256, 512, 1024, 2048, 4096};
                int index = atoi(value_str);
                if (index >= 0 && index < sizeof(valid_sizes)/sizeof(valid_sizes[0])) {
                    g_config.samples = valid_sizes[index];
                    snprintf(response_buf, sizeof(response_buf), "OK: Muestras -> %u\r\n", g_config.samples);
                } else {
                    snprintf(response_buf, sizeof(response_buf), "Error: Indice de muestras invalido (0-4).\r\n");
                }
            } else if (strcmp(param, "steps") == 0) {
                int steps = atoi(value_str);
                if (steps > 1 && steps <= 100) {
                    g_config.sweep_steps = steps;
                    // Recalculate ratio if needed immediately or let sweep start handle it
                     if (g_config.sweep_steps > 1) {
                         imp_sweep_ratio = powf(g_config.sweep_fend / g_config.sweep_fstart, 1.0f / (g_config.sweep_steps - 1));
                     } else {
                         imp_sweep_ratio = 1.0f;
                     }
                    snprintf(response_buf, sizeof(response_buf), "OK: Pasos del barrido -> %u\r\n", g_config.sweep_steps);
                } else {
                    snprintf(response_buf, sizeof(response_buf), "Error: Pasos invalidos (2-100).\r\n");
                }
            } else if (strcmp(param, "fstart") == 0) {
                g_config.sweep_fstart = atof(value_str);
                 if (g_config.sweep_steps > 1) {
                     imp_sweep_ratio = powf(g_config.sweep_fend / g_config.sweep_fstart, 1.0f / (g_config.sweep_steps - 1));
                 } else {
                     imp_sweep_ratio = 1.0f;
                 }
                snprintf(response_buf, sizeof(response_buf), "OK: Frecuencia de inicio -> %.1f Hz\r\n", g_config.sweep_fstart);
            } else if (strcmp(param, "fend") == 0) {
                g_config.sweep_fend = atof(value_str);
                 if (g_config.sweep_steps > 1) {
                     imp_sweep_ratio = powf(g_config.sweep_fend / g_config.sweep_fstart, 1.0f / (g_config.sweep_steps - 1));
                 } else {
                     imp_sweep_ratio = 1.0f;
                 }
                snprintf(response_buf, sizeof(response_buf), "OK: Frecuencia final -> %.1f Hz\r\n", g_config.sweep_fend);
            } else {
                snprintf(response_buf, sizeof(response_buf), "Error: Parametro de 'config' desconocido.\r\n");
            }
        } else {
            snprintf(response_buf, sizeof(response_buf), "Error: Formato 'config <param> <valor>' incorrecto.\r\n");
        }
        HAL_UART_Transmit(&huart2, (uint8_t*)response_buf, strlen(response_buf), HAL_MAX_DELAY);
    }
    // --- Comando: Iniciar Barrido de Impedancia ("sweep") ---
    else if (strcmp(cmd, "sweep") == 0) {
        char confirm[128]; // Increased buffer size slightly
        snprintf(confirm, sizeof(confirm),
                 "Iniciando barrido: %.1fHz a %.1fHz en %d pasos\r\n",
                 g_config.sweep_fstart, g_config.sweep_fend, g_config.sweep_steps);
        strcpy((char*)current_mode_str, "BARRIDO...");
        HAL_UART_Transmit(&huart2, (uint8_t*)confirm, strlen(confirm), HAL_MAX_DELAY);
        RunBlockingSweep();
    }
    else if (strcmp(cmd, "status") == 0) {
        send_status();
    }
    // --- Comando: Ayuda ("help") ---
    else if (strcmp(cmd, "help") == 0) {
        const char *msg =
            "\r\n--- Comandos Disponibles ---\r\n"
            "m <Hz>@            : Medicion unica a <Hz>.\r\n"
            "sweep@             : Inicia barrido de impedancia.\r\n"
            "config <p> <v>@    : Configura un parametro.\r\n"
            "  p=samples, v=0-4 (256,512,1024,2048,4096)\r\n"
            "  p=steps, v=2-100\r\n"
            "  p=fstart, v=<Hz>\r\n"
            "  p=fend, v=<Hz>\r\n"
            "status@            : Muestra estado actual y config.\r\n"
            "help@              : Muestra esta ayuda.\r\n";
        HAL_UART_Transmit(&huart2, (uint8_t*)msg, strlen(msg), UART_TIMEOUT_MS);
    }
    // --- Comando no reconocido ---
    else {
        HAL_UART_Transmit(&huart2, (uint8_t*)"Error: Comando no reconocido. Usa help@@\r\n", 42, HAL_MAX_DELAY);
        strcpy(current_mode_str, "ESPERA"); // Reset mode on error
        mostrar_LCD();
    }
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
    // Ahora el status muestra los valores de la configuración global
    int len = snprintf(buf, sizeof(buf),
        "\r\n--- STATUS Y CONFIGURACION ---\r\n"
        "--- Config ---\r\n"
        "Pasos sweep:    %u\r\n"
        "F. Inicio:      %.1f Hz\r\n"
        "F. Fin:         %.1f Hz\r\n"
        "Muestras/Punto: %u\r\n"
        "--- Hardware ---\r\n"
        "PWM (TIM1): PSC=%lu ARR=%lu\r\n"
        "--- Ultima Medicion ---\r\n"
        "F. Generada:    %.1f Hz\r\n"
        "F. Medida:      %.1f Hz\r\n"
        "Fase:           %.1f deg\r\n\r\n",
        g_config.sweep_steps,
        g_config.sweep_fstart,
        g_config.sweep_fend,
        g_config.samples,
        pwm_psc, pwm_arr,
        frecuencia_generada,
        frecuencia_medida,
        fase_medida
    );
    HAL_UART_Transmit(&huart2, (uint8_t*)buf, len, HAL_MAX_DELAY);
}


