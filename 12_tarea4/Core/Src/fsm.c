/*
 * fsm.c
 *
 *  Created on: Jul 4, 2025
 *      Author: luisduquefranco
 */

#include "main.h"
#include "fsm.h"
#include "MPU6050.h"
#include "i2c_lcd.h"
#include <string.h>
#include <stdio.h>

// Estado actual de la FSM
extern I2C_LCD_HandleTypeDef hlcd;
extern float Ax, Ay, Az;
extern volatile uint16_t fft_buffer_index;
extern uint16_t fft_size;
extern float32_t accel_buffer_z[];
extern float32_t fft_output[];
extern float32_t fft_magnitude[];
extern arm_rfft_fast_instance_f32 fft_instance;
extern volatile float32_t fundamental_freq;
extern volatile e_PosiblesEvents  event ;
static volatile e_PosiblesStates current_state = STATE_IDLE;

// UART
extern char rx_buffer[100];
extern uint8_t rx_index;
extern uint8_t tx_buffer[256];
extern UART_HandleTypeDef huart2;

// LCD externo
extern I2C_LCD_HandleTypeDef lcd;

// Comandos activos
extern void Cmd_HandleLEDDelayCmd(const char *arg);
extern void Cmd_HandleStatusCmd(void);
extern void Cmd_HandlePrintADC(void);

// Mostrar datos en LCD
void mostrar_LCD(void) {
    char linea1[20], linea2[20];

    lcd_gotoxy(&lcd, 0, 0);
    snprintf(linea1, sizeof(linea1), "X:  %.2f m/s2",Ax);
    lcd_puts(&lcd, linea1);
    lcd_gotoxy(&lcd, 0, 1);
    snprintf(linea1, sizeof(linea1), "X:   %.2f  m/s2", Ay);
    lcd_puts(&lcd, linea1);
    lcd_gotoxy(&lcd, 0, 2);
    snprintf(linea2, sizeof(linea2), "Z:   %.2f   m/s2", Az);
    lcd_puts(&lcd, linea2);
    lcd_gotoxy(&lcd, 0, 3); // Añadido: Se posiciona el cursor en la tercera línea del LCD.
	snprintf(linea2, sizeof(linea2), "FreqZ: %.2fHz", fundamental_freq); // Añadido: Muestra la frecuencia fundamental con unidades 'Hz'.
	lcd_puts(&lcd, linea2); // Añadido: Envía la cadena de frecuencia al LCD.

}
static void process_fft(void) {
    // 1. Aplicar la FFT a los datos del búfer
    // La salida es compleja y se almacena en `fft_output`
    arm_rfft_fast_f32(&fft_instance, accel_buffer_z, fft_output, 0);

    // 2. Calcular la magnitud de la salida de la FFT
    // El resultado tiene fft_size/2 puntos de magnitud
    arm_cmplx_mag_f32(fft_output, fft_magnitude, fft_size / 2);

    // 3. Encontrar la frecuencia con la máxima magnitud (frecuencia fundamental)
    // Ignoramos el primer valor (componente DC)
    float32_t max_magnitude = 0.0f;
    uint32_t max_index = 0;
    for (uint16_t i = 1; i < fft_size / 2; i++) {
        if (fft_magnitude[i] > max_magnitude) {
            max_magnitude = fft_magnitude[i];
            max_index = i;
        }
    }

    // 4. Calcular la frecuencia real
    // Frecuencia = (índice_del_pico * frecuencia_de_muestreo) / tamaño_fft
    // La frecuencia de muestreo es 1kHz según tu driver MPU6050.
    float sample_rate = 1000.0f;
    fundamental_freq = (float32_t)max_index * sample_rate / (float)fft_size;
}

// Inicialización de la FSM
void init_fsm(void) {
    current_state = STATE_IDLE;
}

// Evaluación de evento y transición de estado
e_PosiblesStates state_machine_action(e_PosiblesEvents event) {
    switch (event) {

        case EVENT_ACCEL_READY:
            // Leer acelerómetro al recibir interrupción de datos disponibles
            MPU6050_Read_Accel();
            if (fft_buffer_index < fft_size) {
				accel_buffer_z[fft_buffer_index++] = Az;
			}

			// Si el búfer se llenó, generar el evento para procesar la FFT
			if (fft_buffer_index >= fft_size) {
			event = EVENT_FFT_BUFFER_FULL; // Dispara el siguiente evento
				fft_buffer_index = 0; // Reiniciar para la próxima captura
				process_fft();
			}
			mostrar_LCD();
            current_state = STATE_UPDATE_LCD;
            break;

        case EVENT_USART_COMMAND:
            // Procesar comandos vía USART
            HAL_UART_Transmit(&huart2, tx_buffer,
                snprintf((char*)tx_buffer, sizeof(tx_buffer),
                         "Comando recibido: %s\r\n", rx_buffer), 1000);

            if      (strncmp(rx_buffer, "led=", 4) == 0)
                Cmd_HandleLEDDelayCmd(rx_buffer + 4);
            else if (strncmp(rx_buffer, "status", 6) == 0)
                Cmd_HandleStatusCmd();
            else if (strncmp(rx_buffer, "print", 5) == 0) {
                // Enviar datos actuales de acelerómetro por UART
                snprintf((char*)tx_buffer, sizeof(tx_buffer),
                         "AccelX:%.2f, AccelY:%.2f, AccelZ:%.2f\r\n",
						 Ax, Ay, Az);
                HAL_UART_Transmit(&huart2, tx_buffer, strlen((char*)tx_buffer), 1000);
            }

            // Limpiar el buffer de recepción
            memset(rx_buffer, 0, sizeof(rx_buffer));
            rx_index = 0;
                        break;

                    default:
                        // Ignora otros eventos
                        break;
                }

                current_state = STATE_IDLE; // La FSM siempre vuelve a IDLE a esperar el siguiente evento
                return current_state;
            }
