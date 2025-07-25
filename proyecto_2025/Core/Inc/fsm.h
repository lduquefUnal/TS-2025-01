#ifndef FSM_H
#define FSM_H

#include <stdint.h>

typedef enum {
    STATE_IDLE,
    STATE_UPDATE_LCD,
	STATE_PROCESS_FFT,
	STATE_PROCESS_DATA
} e_PosiblesStates;

typedef enum {
    EVENT_NONE,
    EVENT_USART_COMMAND,
	EVENT_IC_CAPTURE,
	EVENT_FFT_BUFFER_FULL,
	EVENT_DATA_READY ,
	EVENT_PRINT_NEXT_DATA
} e_PosiblesEvents;


// Funciones en fsm.c que pueden ser llamadas desde otros archivos (como main.c)
void init_fsm(void);
e_PosiblesStates state_machine_action(e_PosiblesEvents event);
void mostrar_LCD(void);

// Funciones en main.c que la FSM necesita llamar
void StartImpedanceMeasurement(uint32_t frequency);
void PWM_SetFrequency(uint32_t freq_hz);
extern char current_mode_str[21];
extern arm_rfft_fast_instance_f32 fft_instance;
extern volatile e_PosiblesEvents current_event;
#endif // FSM_H
