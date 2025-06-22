#ifndef FSM_H
#define FSM_H

#include <stdint.h>

typedef enum {
    STATE_IDLE,
    STATE_UPDATE_DISP,
    STATE_SW_PRESS
} e_PosiblesStates;

typedef enum {
	IDLE,
    EVENT_ENCODER,
    EVENT_TIMER_TICK,
    EVENT_SW,
	EVENT_USART
} e_PosiblesEvents;

void init_fsm(void);
e_PosiblesStates state_machine_action(e_PosiblesEvents event);

#endif // FSM_H
