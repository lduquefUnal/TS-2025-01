/**
 ******************************************************************************
 * @file           : main.c
 * @author         : lduquef
 * @brief          : Basic Config
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

#include <stdio.h>
#include <stdint.h>
#include "stm32f4xx.h"
#include "gpio_driver_hal.h"
#include "stm32_assert.h"
#include "exti_driver_hal.h"
#include "timer_driver_hal.h"
#include "sysTickDriver.h"

GPIO_Handler_t userLed      = {0}; // PinH1
GPIO_Handler_t userDis1 	= {0}; // PinA0
GPIO_Handler_t userDis2 	= {0}; // PinA1
GPIO_Handler_t userDis3 	= {0}; // PinB0
GPIO_Handler_t userDis4 	= {0}; // PinA4
GPIO_Handler_t userClk 		= {0}; // PinA12
GPIO_Handler_t userData 	= {0}; // PinC6
GPIO_Handler_t userSw 	    = {0}; // PinC9
GPIO_Handler_t userSegA     = {0}; // PinA10
GPIO_Handler_t userSegB     = {0}; // PinB10
GPIO_Handler_t userSegC     = {0}; // PinB13
GPIO_Handler_t userSegD     = {0}; // PinC0
GPIO_Handler_t userSegE     = {0}; // PinC1
GPIO_Handler_t userSegF     = {0}; // PinB1
GPIO_Handler_t userSegG     = {0}; // PinB5

Timer_Handler_t blinkyTimer = {0}; //TIM2
Timer_Handler_t displayTimer = {0};//TIM3
EXTI_Config_t extiData = {0};
EXTI_Config_t extiSw = {0};

uint8_t nextDigit = 0;
uint8_t data = 0;
uint8_t clock = 0;
uint8_t ms_delay = 4 ;
uint16_t estado= 0;
volatile uint8_t encoderEventFlag = 0 ;
volatile uint8_t swEventFlag = 0 ;
enum{
	unidades = 1,
	decenas,
	centenas,
	miles,
};

void init_System(void);
void displayNumber(uint8_t);

int main(void){
    init_System();
    displayNumber(8);
    config_SysTick_ms(0);
    delay_ms(20);
    while(1){
		gpio_WritePin(&userDis1, RESET);
		gpio_WritePin(&userDis2, RESET);
		gpio_WritePin(&userDis3, RESET);
		gpio_WritePin(&userDis4, RESET);
		gpio_WritePin(&userSegA, RESET);
		gpio_WritePin(&userSegB, RESET);
		gpio_WritePin(&userSegC, RESET);
		gpio_WritePin(&userSegD, RESET);
		gpio_WritePin(&userSegE, RESET);
		gpio_WritePin(&userSegF, RESET);
		gpio_WritePin(&userSegG, RESET);

		//if(swEventFlag){

		//}
		if(encoderEventFlag){
			encoderEventFlag=0;
			if(clock == 0){
				if(estado==0){
					estado = 4095;
				}else{
					estado--;
				}
			}else{
				if(estado==4095){
					estado = 0;
				}else {
					estado++;
				}
			}
		}
		switch(nextDigit){
		case unidades: {
			gpio_WritePin(&userDis1,SET);
			displayNumber(estado%10);
			delay_ms(ms_delay);
			break;
		}
		}
    }
}


void displayNumber(uint8_t estado) {
    // Apagar todos los segmentos inicialmente
    // Encender los segmentos necesarios según el número
    if (estado == 0) {
        gpio_WritePin(&userSegA, SET);
        gpio_WritePin(&userSegB, SET);
        gpio_WritePin(&userSegC, SET);
        gpio_WritePin(&userSegD, SET);
        gpio_WritePin(&userSegE, SET);
        gpio_WritePin(&userSegF, SET);
    } else if (estado == 1) {
        gpio_WritePin(&userSegB, SET);
        gpio_WritePin(&userSegC, SET);
    } else if (estado == 2) {
        gpio_WritePin(&userSegA, SET);
        gpio_WritePin(&userSegB, SET);
        gpio_WritePin(&userSegD, SET);
        gpio_WritePin(&userSegE, SET);
        gpio_WritePin(&userSegG, SET);
    } else if (estado == 3) {
        gpio_WritePin(&userSegA, SET);
        gpio_WritePin(&userSegB, SET);
        gpio_WritePin(&userSegC, SET);
        gpio_WritePin(&userSegD, SET);
        gpio_WritePin(&userSegG, SET);
    } else if (estado == 4) {
        gpio_WritePin(&userSegB, SET);
        gpio_WritePin(&userSegC, SET);
        gpio_WritePin(&userSegF, SET);
        gpio_WritePin(&userSegG, SET);
    } else if (estado == 5) {
        gpio_WritePin(&userSegA, SET);
        gpio_WritePin(&userSegC, SET);
        gpio_WritePin(&userSegD, SET);
        gpio_WritePin(&userSegF, SET);
        gpio_WritePin(&userSegG, SET);
    } else if (estado == 6) {
        gpio_WritePin(&userSegA, SET);
        gpio_WritePin(&userSegC, SET);
        gpio_WritePin(&userSegD, SET);
        gpio_WritePin(&userSegE, SET);
        gpio_WritePin(&userSegF, SET);
        gpio_WritePin(&userSegG, SET);
    } else if (estado == 7) {
        gpio_WritePin(&userSegA, SET);
        gpio_WritePin(&userSegB, SET);
        gpio_WritePin(&userSegC, SET);
    } else if (estado == 8) {
        gpio_WritePin(&userSegA, SET);
        gpio_WritePin(&userSegB, SET);
        gpio_WritePin(&userSegC, SET);
        gpio_WritePin(&userSegD, SET);
        gpio_WritePin(&userSegE, SET);
        gpio_WritePin(&userSegF, SET);
        gpio_WritePin(&userSegG, SET);
    } else if (estado == 9) {
        gpio_WritePin(&userSegA, SET);
        gpio_WritePin(&userSegB, SET);
        gpio_WritePin(&userSegC, SET);
        gpio_WritePin(&userSegF, SET);
        gpio_WritePin(&userSegG, SET);
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
	userClk.pinConfig.GPIO_PinPuPdControl	= GPIO_PUPDR_NOTHING;
	gpio_Config(&userClk);

	userData.pGPIOx							= GPIOC;
	userData.pinConfig.GPIO_PinNumber		= PIN_6;
	userData.pinConfig.GPIO_PinMode			= GPIO_MODE_IN;
	userData.pinConfig.GPIO_PinPuPdControl	= GPIO_PUPDR_NOTHING;
	gpio_Config(&userData);

	userSw.pGPIOx							= GPIOC;
	userSw.pinConfig.GPIO_PinNumber		    = PIN_9;
	userSw.pinConfig.GPIO_PinMode			= GPIO_MODE_IN;
	userSw.pinConfig.GPIO_PinPuPdControl	= GPIO_PUPDR_NOTHING;
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
	displayTimer.TIMx_Config.TIMx_Period			= ms_delay;
	displayTimer.TIMx_Config.TIMx_InterruptEnable 	= TIMER_INT_ENABLE;
	timer_Config(&displayTimer);
	timer_SetState(&displayTimer, TIMER_ON);

	extiData.pGPIOHandler					= &userData;
	extiData.edgeType						= EXTERNAL_INTERRUPT_RISING_EDGE;
	exti_Config(&extiData);

	extiSw.pGPIOHandler					= &userSw;
	extiSw.edgeType						= EXTERNAL_INTERRUPT_RISING_EDGE;
	exti_Config(&extiSw);

	gpio_WritePin(&userLed, SET);

}
// callbacks  llamadas fuera del bucle
void Timer2_Callback(void){
	gpio_TogglePin(&userLed);
}
void Timer3_Callback(void){
	nextDigit++;
}

void callback_ExtInt12(void){
	encoderEventFlag =1 ;
}

void callback_ExtInte9(void){
	swEventFlag = 1 ;
}

void assert_failed(uint8_t* file, uint32_t line){
	while(1){
		//Problems
	}
}

