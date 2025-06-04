/**
 ******************************************************************************
 * @file           : main.c
 * @author         : lduquef
 * @brief          : tarea2
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2025 STMicroelectronics.
 * All rights reserved.
 *
 * This software is licensed under terms that can be found in the LICENSE file
 * in the root directory of this software component.
 * If no LICENSE file comes with this software, it is provided AS-IS.
 *
 ******************************************************************************
 */
#include <stdint.h>
#include <stm32f4xx.h>

#include "stm32_assert.h"
#include "gpio_driver_hal.h"
#include "exti_driver_hal.h"
#include "timer_driver_hal.h"
#include "sysTickDriver.h"
#include "fsm.h"

GPIO_Handler_t userLed    = {0};
GPIO_Handler_t userDis1   = {0};
GPIO_Handler_t userDis2   = {0};
GPIO_Handler_t userDis3   = {0};
GPIO_Handler_t userDis4   = {0};
GPIO_Handler_t userClk    = {0};
GPIO_Handler_t userData   = {0};
GPIO_Handler_t userSw     = {0};
GPIO_Handler_t userSegA   = {0};
GPIO_Handler_t userSegB   = {0};
GPIO_Handler_t userSegC   = {0};
GPIO_Handler_t userSegD   = {0};
GPIO_Handler_t userSegE   = {0};
GPIO_Handler_t userSegF   = {0};
GPIO_Handler_t userSegG   = {0};

Timer_Handler_t blinkyTimer  = {0};
Timer_Handler_t displayTimer = {0};
EXTI_Config_t extiClk       = {0};
EXTI_Config_t extiSw         = {0};

volatile uint8_t clk_snapshot = 0;
volatile uint8_t data_snapshot = 0;
volatile uint8_t swEventFlag = 0;
static e_PosiblesStates currentState;
static uint16_t display_value;
static uint8_t nextDigit_FSM;
static uint8_t MS_LED = 4;
void init_System(void);
void init_fsm(void);
e_PosiblesStates state_machine_action(e_PosiblesEvents event);
void displayNumber(uint8_t digitValue);
void Timer2_Callback(void);
void Timer3_Callback(void);
void callback_ExtInt12(void);
void callback_ExtInt9(void);

void init_fsm(void) {
    currentState  = STATE_IDLE;
    display_value = 0;
    nextDigit_FSM = 1;
}

int main(void) {
    init_System();
    init_fsm();
    state_machine_action(EVENT_TIMER_TICK);

    while (1) {
        state_machine_action(EVENT_NONE);
        // Pequeño delay para evitar consumo excesivo de CPU
        delay_ms(10);
    }
}

e_PosiblesStates state_machine_action(e_PosiblesEvents event) {
    switch (currentState) {
        case STATE_IDLE:
            if (event == EVENT_ENCODER) {
                if (data_snapshot== 1) {
                    display_value = (display_value == 4095) ? 0 : display_value + 1;
                } else {
                    display_value = (display_value == 0) ? 4095 : display_value - 1;
                }
                currentState = STATE_UPDATE_DISP;
            }
            else if (event == EVENT_TIMER_TICK) {
                currentState = STATE_UPDATE_DISP;
            }
            else if (event == EVENT_SW) {
                currentState = STATE_SW_PRESS;
            }
            break;

        case STATE_UPDATE_DISP:
            gpio_WritePin(&userDis1, RESET);
            gpio_WritePin(&userDis2, RESET);
            gpio_WritePin(&userDis3, RESET);
            gpio_WritePin(&userDis4, RESET);
            switch (nextDigit_FSM) {
                case 1:
                    gpio_WritePin(&userDis1, SET);
                    displayNumber(display_value % 10);
                    break;
                case 2:
                    gpio_WritePin(&userDis2, SET);
                    displayNumber((display_value / 10) % 10);
                    break;
                case 3:
                    gpio_WritePin(&userDis3, SET);
                    displayNumber((display_value / 100) % 10);
                    break;
                case 4:
                    gpio_WritePin(&userDis4, SET);
                    displayNumber((display_value / 1000) % 10);
                    break;
                default:
                    nextDigit_FSM = 1;
                    gpio_WritePin(&userDis1, SET);
                    displayNumber(display_value % 10);
            }
            nextDigit_FSM = (nextDigit_FSM < 4) ? nextDigit_FSM + 1 : 1;
            currentState = STATE_IDLE;
            break;

        case STATE_SW_PRESS:
            display_value = 0; // reset on switch press
            nextDigit_FSM    = 1;
            state_machine_action(EVENT_TIMER_TICK);
            currentState = STATE_UPDATE_DISP;
            break;

        default:
            currentState = STATE_IDLE;
    }
    return currentState;
}

void displayNumber(uint8_t digitValue) {
    gpio_WritePin(&userSegA, RESET);
    gpio_WritePin(&userSegB, RESET);
    gpio_WritePin(&userSegC, RESET);
    gpio_WritePin(&userSegD, RESET);
    gpio_WritePin(&userSegE, RESET);
    gpio_WritePin(&userSegF, RESET);
    gpio_WritePin(&userSegG, RESET);
    switch (digitValue) {
        case 0:
            gpio_WritePin(&userSegA, SET);
            gpio_WritePin(&userSegB, SET);
            gpio_WritePin(&userSegC, SET);
            gpio_WritePin(&userSegD, SET);
            gpio_WritePin(&userSegE, SET);
            gpio_WritePin(&userSegF, SET);
            break;
        case 1:
            gpio_WritePin(&userSegB, SET);
            gpio_WritePin(&userSegC, SET);
            break;
        case 2:
            gpio_WritePin(&userSegA, SET);
            gpio_WritePin(&userSegB, SET);
            gpio_WritePin(&userSegD, SET);
            gpio_WritePin(&userSegE, SET);
            gpio_WritePin(&userSegG, SET);
            break;
        case 3:
            gpio_WritePin(&userSegA, SET);
            gpio_WritePin(&userSegB, SET);
            gpio_WritePin(&userSegC, SET);
            gpio_WritePin(&userSegD, SET);
            gpio_WritePin(&userSegG, SET);
            break;
        case 4:
            gpio_WritePin(&userSegB, SET);
            gpio_WritePin(&userSegC, SET);
            gpio_WritePin(&userSegF, SET);
            gpio_WritePin(&userSegG, SET);
            break;
        case 5:
            gpio_WritePin(&userSegA, SET);
            gpio_WritePin(&userSegC, SET);
            gpio_WritePin(&userSegD, SET);
            gpio_WritePin(&userSegF, SET);
            gpio_WritePin(&userSegG, SET);
            break;
        case 6:
            gpio_WritePin(&userSegA, SET);
            gpio_WritePin(&userSegC, SET);
            gpio_WritePin(&userSegD, SET);
            gpio_WritePin(&userSegE, SET);
            gpio_WritePin(&userSegF, SET);
            gpio_WritePin(&userSegG, SET);
            break;
        case 7:
            gpio_WritePin(&userSegA, SET);
            gpio_WritePin(&userSegB, SET);
            gpio_WritePin(&userSegC, SET);
            break;
        case 8:
            gpio_WritePin(&userSegA, SET);
            gpio_WritePin(&userSegB, SET);
            gpio_WritePin(&userSegC, SET);
            gpio_WritePin(&userSegD, SET);
            gpio_WritePin(&userSegE, SET);
            gpio_WritePin(&userSegF, SET);
            gpio_WritePin(&userSegG, SET);
            break;
        case 9:
            gpio_WritePin(&userSegA, SET);
            gpio_WritePin(&userSegB, SET);
            gpio_WritePin(&userSegC, SET);
            gpio_WritePin(&userSegF, SET);
            gpio_WritePin(&userSegG, SET);
            break;
    }
}

void init_System(void){
	userLed.pGPIOx							= GPIOH;
	userLed.pinConfig.GPIO_PinNumber		= PIN_1;
	userLed.pinConfig.GPIO_PinMode			= GPIO_MODE_OUT;
	userLed.pinConfig.GPIO_PinOutputType	= GPIO_OTYPE_PUSHPULL;
	userLed.pinConfig.GPIO_PinOutputSpeed	= GPIO_OSPEED_MEDIUM;
	userLed.pinConfig.GPIO_PinPuPdControl	= GPIO_PUPDR_NOTHING;
	gpio_Config(&userLed);

	userDis1.pGPIOx							= GPIOA;
	userDis1.pinConfig.GPIO_PinNumber		= PIN_0;
	userDis1.pinConfig.GPIO_PinMode			= GPIO_MODE_OUT;
	userDis1.pinConfig.GPIO_PinOutputType	= GPIO_OTYPE_PUSHPULL;
	userDis1.pinConfig.GPIO_PinOutputSpeed	= GPIO_OSPEED_MEDIUM;
	userDis1.pinConfig.GPIO_PinPuPdControl	= GPIO_PUPDR_NOTHING;
	gpio_Config(&userDis1);

	userDis2.pGPIOx							= GPIOA;
	userDis2.pinConfig.GPIO_PinNumber		= PIN_1;
	userDis2.pinConfig.GPIO_PinMode			= GPIO_MODE_OUT;
	userDis2.pinConfig.GPIO_PinOutputType	= GPIO_OTYPE_PUSHPULL;
	userDis2.pinConfig.GPIO_PinOutputSpeed	= GPIO_OSPEED_MEDIUM;
	userDis2.pinConfig.GPIO_PinPuPdControl	= GPIO_PUPDR_NOTHING;
	gpio_Config(&userDis2);

	userDis3.pGPIOx							= GPIOB;
	userDis3.pinConfig.GPIO_PinNumber		= PIN_0;
	userDis3.pinConfig.GPIO_PinMode			= GPIO_MODE_OUT;
	userDis3.pinConfig.GPIO_PinOutputType	= GPIO_OTYPE_PUSHPULL;
	userDis3.pinConfig.GPIO_PinOutputSpeed	= GPIO_OSPEED_MEDIUM;
	userDis3.pinConfig.GPIO_PinPuPdControl	= GPIO_PUPDR_NOTHING;
	gpio_Config(&userDis3);

	userDis4.pGPIOx							= GPIOA;
	userDis4.pinConfig.GPIO_PinNumber		= PIN_4;
	userDis4.pinConfig.GPIO_PinMode			= GPIO_MODE_OUT;
	userDis4.pinConfig.GPIO_PinOutputType	= GPIO_OTYPE_PUSHPULL;
	userDis4.pinConfig.GPIO_PinOutputSpeed	= GPIO_OSPEED_MEDIUM;
	userDis4.pinConfig.GPIO_PinPuPdControl	= GPIO_PUPDR_NOTHING;
	gpio_Config(&userDis4);

	userSegA.pGPIOx							= GPIOA;
	userSegA.pinConfig.GPIO_PinNumber		= PIN_10;
	userSegA.pinConfig.GPIO_PinMode			= GPIO_MODE_OUT;
	userSegA.pinConfig.GPIO_PinOutputType	= GPIO_OTYPE_PUSHPULL;
	userSegA.pinConfig.GPIO_PinOutputSpeed	= GPIO_OSPEED_MEDIUM;
	userSegA.pinConfig.GPIO_PinPuPdControl	= GPIO_PUPDR_NOTHING;
	gpio_Config(&userSegA);

	userSegB.pGPIOx							= GPIOB;
	userSegB.pinConfig.GPIO_PinNumber		= PIN_10;
	userSegB.pinConfig.GPIO_PinMode			= GPIO_MODE_OUT;
	userSegB.pinConfig.GPIO_PinOutputType	= GPIO_OTYPE_PUSHPULL;
	userSegB.pinConfig.GPIO_PinOutputSpeed	= GPIO_OSPEED_MEDIUM;
	userSegB.pinConfig.GPIO_PinPuPdControl	= GPIO_PUPDR_NOTHING;
	gpio_Config(&userSegB);

	userSegC.pGPIOx							= GPIOB;
	userSegC.pinConfig.GPIO_PinNumber		= PIN_13;
	userSegC.pinConfig.GPIO_PinMode			= GPIO_MODE_OUT;
	userSegC.pinConfig.GPIO_PinOutputType	= GPIO_OTYPE_PUSHPULL;
	userSegC.pinConfig.GPIO_PinOutputSpeed	= GPIO_OSPEED_MEDIUM;
	userSegC.pinConfig.GPIO_PinPuPdControl	= GPIO_PUPDR_NOTHING;

	gpio_Config(&userSegC);
	userSegD.pGPIOx							= GPIOC;
	userSegD.pinConfig.GPIO_PinNumber		= PIN_0;
	userSegD.pinConfig.GPIO_PinMode			= GPIO_MODE_OUT;
	userSegD.pinConfig.GPIO_PinOutputType	= GPIO_OTYPE_PUSHPULL;
	userSegD.pinConfig.GPIO_PinOutputSpeed	= GPIO_OSPEED_MEDIUM;
	userSegD.pinConfig.GPIO_PinPuPdControl	= GPIO_PUPDR_NOTHING;
	gpio_Config(&userSegD);

	userSegE.pGPIOx							= GPIOC;
	userSegE.pinConfig.GPIO_PinNumber		= PIN_1;
	userSegE.pinConfig.GPIO_PinMode			= GPIO_MODE_OUT;
	userSegE.pinConfig.GPIO_PinOutputType	= GPIO_OTYPE_PUSHPULL;
	userSegE.pinConfig.GPIO_PinOutputSpeed	= GPIO_OSPEED_MEDIUM;
	userSegE.pinConfig.GPIO_PinPuPdControl	= GPIO_PUPDR_NOTHING;
	gpio_Config(&userSegE);

	userSegF.pGPIOx							= GPIOB;
	userSegF.pinConfig.GPIO_PinNumber		= PIN_1;
	userSegF.pinConfig.GPIO_PinMode			= GPIO_MODE_OUT;
	userSegF.pinConfig.GPIO_PinOutputType	= GPIO_OTYPE_PUSHPULL;
	userSegF.pinConfig.GPIO_PinOutputSpeed	= GPIO_OSPEED_MEDIUM;
	userSegF.pinConfig.GPIO_PinPuPdControl	= GPIO_PUPDR_NOTHING;
	gpio_Config(&userSegF);

	userSegG.pGPIOx							= GPIOB;
	userSegG.pinConfig.GPIO_PinNumber		= PIN_5;
	userSegG.pinConfig.GPIO_PinMode			= GPIO_MODE_OUT;
	userSegG.pinConfig.GPIO_PinOutputType	= GPIO_OTYPE_PUSHPULL;
	userSegG.pinConfig.GPIO_PinOutputSpeed	= GPIO_OSPEED_MEDIUM;
	userSegG.pinConfig.GPIO_PinPuPdControl	= GPIO_PUPDR_NOTHING;
	gpio_Config(&userSegG);

	userClk.pGPIOx							= GPIOA;
	userClk.pinConfig.GPIO_PinNumber		= PIN_12;
	userClk.pinConfig.GPIO_PinMode			= GPIO_MODE_IN;
	userClk.pinConfig.GPIO_PinPuPdControl	= GPIO_PUPDR_PULLDOWN;
	gpio_Config(&userClk);

	userData.pGPIOx							= GPIOC;
	userData.pinConfig.GPIO_PinNumber		= PIN_8;
	userData.pinConfig.GPIO_PinMode			= GPIO_MODE_IN;
	userData.pinConfig.GPIO_PinPuPdControl	= GPIO_PUPDR_PULLDOWN;
	gpio_Config(&userData);

	userSw.pGPIOx							= GPIOC;
	userSw.pinConfig.GPIO_PinNumber		    = PIN_9;
	userSw.pinConfig.GPIO_PinMode			= GPIO_MODE_IN;
	userSw.pinConfig.GPIO_PinPuPdControl	= GPIO_PUPDR_PULLUP;
	gpio_Config(&userSw);

	config_SysTick_ms(0);

	blinkyTimer.pTIMx								= TIM2;
	blinkyTimer.TIMx_Config.TIMx_mode				= TIMER_UP_COUNTER;
	blinkyTimer.TIMx_Config.TIMx_Prescaler  		= 16000; //1ms conversion
	blinkyTimer.TIMx_Config.TIMx_Period				= 500;
	blinkyTimer.TIMx_Config.TIMx_InterruptEnable 	= TIMER_INT_ENABLE;
	timer_Config(&blinkyTimer);
	timer_SetState(&blinkyTimer, TIMER_ON);

	displayTimer.pTIMx								= TIM3;
	displayTimer.TIMx_Config.TIMx_mode				= TIMER_UP_COUNTER;
	displayTimer.TIMx_Config.TIMx_Prescaler  		= 16000; //1ms conversion
	displayTimer.TIMx_Config.TIMx_Period			= MS_LED;
	displayTimer.TIMx_Config.TIMx_InterruptEnable 	= TIMER_INT_ENABLE;
	timer_Config(&displayTimer);
	timer_SetState(&displayTimer, TIMER_ON);

	extiClk.pGPIOHandler					= &userClk;
	extiClk.edgeType						= EXTERNAL_INTERRUPT_RISING_EDGE;
	exti_Config(&extiClk);

	extiSw.pGPIOHandler					= &userSw;
	extiSw.edgeType						= EXTERNAL_INTERRUPT_FALLING_EDGE;
	exti_Config(&extiSw);

	gpio_WritePin(&userLed, SET);

}

void Timer2_Callback(void) {
    gpio_TogglePin(&userLed);
}

void Timer3_Callback(void) {
    state_machine_action(EVENT_TIMER_TICK);

void callback_ExtInt12(void) {
    data_snapshot = gpio_ReadPin(&userData);
    state_machine_action(EVENT_ENCODER);
}

void callback_ExtInt9(void) {
    state_machine_action(EVENT_SW);
}

void assert_failed(uint8_t* file, uint32_t line) {
    while (1) {
    	}
    }
}
