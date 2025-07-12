/*
 * fsm.c
 *
 *  Created on: Jul 4, 2025
 *      Author: luisduquefranco
 */

#include "main.h"
#include "fsm.h"
#include "commands.h"
// Variables externas usadas en acciones
extern uint16_t display_value;
extern uint8_t nextDigit_FSM;
extern char rx_buffer[100];
extern uint8_t rx_index;
extern uint8_t tx_buffer[256];
extern UART_HandleTypeDef huart2;

// Comandos
extern void displayNumber(uint8_t digitValue);
extern void Cmd_HandlePrintADC(void);
extern void Cmd_HandleLEDDelayCmd(const char *arg);
extern void Cmd_HandleSampleFreqCmd(const char *arg);
extern void Cmd_HandleRGBCmd(const char *arg);
extern void Cmd_HandleFFTSizeCmd(const char *arg);
extern void Cmd_HandleStatusCmd(void);
extern void Cmd_HandleFreqDisplayCmd(void);
extern void Cmd_HandlePrintFFT(void);
extern void Cmd_HandleFFTInfo(void);
extern void Cmd_HandleHelpCmd(void);
extern void Cmd_HandleClearCmd(void);

// Estado actual de la FSM
static e_PosiblesStates current_state = STATE_IDLE;

void init_fsm(void) {
    current_state = STATE_IDLE;
}

e_PosiblesStates state_machine_action(e_PosiblesEvents event) {
    switch (event) {
        case EVENT_ENCODER:
            // Aquí podrías verificar una bandera externa como data_snapshot
            // para decidir si aumentar o disminuir display_value
            display_value = (display_value == 4095) ? 0 : display_value + 1;
            current_state = STATE_UPDATE_DISP;
            break;

        case EVENT_SW:
            display_value = 0;
            nextDigit_FSM = 1;
            current_state = STATE_SW_PRESS;
            break;

        case EVENT_TIMER_TICK:
            // Multiplexado de display
            HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0, GPIO_PIN_RESET);
            HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1, GPIO_PIN_RESET);
            HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_RESET);
            HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_RESET);

            switch (nextDigit_FSM) {
                case 1:
                    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0, GPIO_PIN_SET);
                    displayNumber(display_value % 10);
                    break;
                case 2:
                    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1, GPIO_PIN_SET);
                    displayNumber((display_value / 10) % 10);
                    break;
                case 3:
                    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_SET);
                    displayNumber((display_value / 100) % 10);
                    break;
                case 4:
                    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_SET);
                    displayNumber((display_value / 1000) % 10);
                    break;
            }
            nextDigit_FSM = (nextDigit_FSM < 4) ? nextDigit_FSM + 1 : 1;
            current_state = STATE_UPDATE_DISP;
            break;

        case EVENT_USART: {
            HAL_UART_Transmit(&huart2, tx_buffer,
                snprintf((char*)tx_buffer, sizeof(tx_buffer),
                         "Comando recibido: %s\r\n", rx_buffer), 1000);

            if      (strncmp(rx_buffer,"led=",4)==0)      Cmd_HandleLEDDelayCmd(rx_buffer+4);
            else if (strncmp(rx_buffer,"fmuestreo=",10)==0) Cmd_HandleSampleFreqCmd(rx_buffer+10);
            else if (strncmp(rx_buffer,"rgb=",4)==0)      Cmd_HandleRGBCmd(rx_buffer+4);
            else if (strncmp(rx_buffer,"fftSize=",8)==0)  Cmd_HandleFFTSizeCmd(rx_buffer+8);
            else if (strncmp(rx_buffer,"status",6)==0)    Cmd_HandleStatusCmd();
            else if (strncmp(rx_buffer,"print",5)==0)     Cmd_HandlePrintADC();
            else if (strncmp(rx_buffer,"freq",4)==0)      Cmd_HandleFreqDisplayCmd();
            else if (strncmp(rx_buffer,"fft",3)==0)       Cmd_HandlePrintFFT();
            else if (strncmp(rx_buffer,"info",4)==0)      Cmd_HandleFFTInfo();
            else if (strncmp(rx_buffer,"help",4)==0)      Cmd_HandleHelpCmd();
            else if (strncmp(rx_buffer,"clear",5)==0)     Cmd_HandleClearCmd();

            memset(rx_buffer, 0, sizeof(rx_buffer));
            rx_index = 0;
            current_state = STATE_IDLE;
            break;
        }

        case EVENT_IC_CAPTURE:
            // Aquí iría tu lógica de frecuencia medida, si aplica
            current_state = STATE_IDLE;
            break;

        case IDLE:
        default:
            current_state = STATE_IDLE;
            break;
    }

    return current_state;
}
