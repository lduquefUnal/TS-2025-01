#ifndef FSM_H
#define FSM_H

#include <stdint.h>

typedef enum {
    STATE_IDLE,
    STATE_UPDATE_LCD,
	STATE_PROCESS_FFT
} e_PosiblesStates;

typedef enum {
    EVENT_NONE,
    EVENT_ACCEL_READY,
    EVENT_USART_COMMAND,
	EVENT_IC_CAPTURE,
	EVENT_FFT_BUFFER_FULL,
} e_PosiblesEvents;

void init_fsm(void);
e_PosiblesStates state_machine_action(e_PosiblesEvents event);

#endif // FSM_H
