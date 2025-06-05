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

#include <stdint.h>
#include <stm32f4xx.h>
#include "stm32_assert.h"
#include "gpio_driver_hal.h"
#include "exti_driver_hal.h"
#include "timer_driver_hal.h"
#include "sysTickDriver.h"

// Definición de pines
GPIO_Handler_t userLed    = {0};
GPIO_Handler_t userDis1   = {0};
GPIO_Handler_t userDis2   = {0};
GPIO_Handler_t userDis3   = {0};
GPIO_Handler_t userDis4   = {0};
GPIO_Handler_t userSegA   = {0};
GPIO_Handler_t userSegB   = {0};
GPIO_Handler_t userSegC   = {0};
GPIO_Handler_t userSegD   = {0};
GPIO_Handler_t userSegE   = {0};
GPIO_Handler_t userSegF   = {0};
GPIO_Handler_t userSegG   = {0};

// Temporizador para Blinky
Timer_Handler_t blinkyTimer  = {0};
Timer_Handler_t displayTimer = {0};

// Configuración de 8888 en el display
uint16_t display_value = 8888;

// Función para inicializar el sistema
void init_System(void) {
    // Inicialización del LED (Pin GPIOH, Pin 1)
    userLed.pGPIOx = GPIOH;
    userLed.pinConfig.GPIO_PinNumber = PIN_1;
    userLed.pinConfig.GPIO_PinMode = GPIO_MODE_OUT;
    userLed.pinConfig.GPIO_PinOutputType = GPIO_OTYPE_PUSHPULL;
    userLed.pinConfig.GPIO_PinOutputSpeed = GPIO_OSPEED_MEDIUM;
    userLed.pinConfig.GPIO_PinPuPdControl = GPIO_PUPDR_NOTHING;
    gpio_Config(&userLed);

    // Inicialización de los displays (segundos)
    userDis1.pGPIOx = GPIOA;
    userDis1.pinConfig.GPIO_PinNumber = PIN_0;
    userDis1.pinConfig.GPIO_PinMode = GPIO_MODE_OUT;
    gpio_Config(&userDis1);

    userDis2.pGPIOx = GPIOA;
    userDis2.pinConfig.GPIO_PinNumber = PIN_1;
    userDis2.pinConfig.GPIO_PinMode = GPIO_MODE_OUT;
    gpio_Config(&userDis2);

    userDis3.pGPIOx = GPIOB;
    userDis3.pinConfig.GPIO_PinNumber = PIN_0;
    userDis3.pinConfig.GPIO_PinMode = GPIO_MODE_OUT;
    gpio_Config(&userDis3);

    userDis4.pGPIOx = GPIOA;
    userDis4.pinConfig.GPIO_PinNumber = PIN_4;
    userDis4.pinConfig.GPIO_PinMode = GPIO_MODE_OUT;
    gpio_Config(&userDis4);

    // Configuración del display de segmentos
    userSegA.pGPIOx = GPIOA;
    userSegA.pinConfig.GPIO_PinNumber = PIN_10;
    userSegA.pinConfig.GPIO_PinMode = GPIO_MODE_OUT;
    gpio_Config(&userSegA);

    userSegB.pGPIOx = GPIOB;
    userSegB.pinConfig.GPIO_PinNumber = PIN_10;
    userSegB.pinConfig.GPIO_PinMode = GPIO_MODE_OUT;
    gpio_Config(&userSegB);

    userSegC.pGPIOx = GPIOB;
    userSegC.pinConfig.GPIO_PinNumber = PIN_13;
    userSegC.pinConfig.GPIO_PinMode = GPIO_MODE_OUT;
    gpio_Config(&userSegC);

    userSegD.pGPIOx = GPIOC;
    userSegD.pinConfig.GPIO_PinNumber = PIN_0;
    userSegD.pinConfig.GPIO_PinMode = GPIO_MODE_OUT;
    gpio_Config(&userSegD);

    userSegE.pGPIOx = GPIOC;
    userSegE.pinConfig.GPIO_PinNumber = PIN_1;
    userSegE.pinConfig.GPIO_PinMode = GPIO_MODE_OUT;
    gpio_Config(&userSegE);

    userSegF.pGPIOx = GPIOB;
    userSegF.pinConfig.GPIO_PinNumber = PIN_1;
    userSegF.pinConfig.GPIO_PinMode = GPIO_MODE_OUT;
    gpio_Config(&userSegF);

    userSegG.pGPIOx = GPIOB;
    userSegG.pinConfig.GPIO_PinNumber = PIN_5;
    userSegG.pinConfig.GPIO_PinMode = GPIO_MODE_OUT;
    gpio_Config(&userSegG);

    // Configuración de temporizadores
    blinkyTimer.pTIMx = TIM2;
    blinkyTimer.TIMx_Config.TIMx_mode = TIMER_UP_COUNTER;
    blinkyTimer.TIMx_Config.TIMx_Prescaler = 16000; // 1ms
    blinkyTimer.TIMx_Config.TIMx_Period = 500; // 500ms
    blinkyTimer.TIMx_Config.TIMx_InterruptEnable = TIMER_INT_ENABLE;
    timer_Config(&blinkyTimer);
    timer_SetState(&blinkyTimer, TIMER_ON);

    displayTimer.pTIMx = TIM3;
    displayTimer.TIMx_Config.TIMx_mode = TIMER_UP_COUNTER;
    displayTimer.TIMx_Config.TIMx_Prescaler = 16000; // 1ms
    displayTimer.TIMx_Config.TIMx_Period = 500;
    displayTimer.TIMx_Config.TIMx_InterruptEnable = TIMER_INT_ENABLE;
    timer_Config(&displayTimer);
    timer_SetState(&displayTimer, TIMER_ON);
}

// Función para mostrar números en el display
void displayNumber(uint8_t digitValue) {
    // Lógica simple para mostrar números 0-9
    gpio_WritePin(&userSegA, digitValue & 1 ? SET : RESET);
    gpio_WritePin(&userSegB, digitValue & 2 ? SET : RESET);
    gpio_WritePin(&userSegC, digitValue & 4 ? SET : RESET);
    gpio_WritePin(&userSegD, digitValue & 8 ? SET : RESET);
    gpio_WritePin(&userSegE, digitValue & 16 ? SET : RESET);
    gpio_WritePin(&userSegF, digitValue & 32 ? SET : RESET);
    gpio_WritePin(&userSegG, digitValue & 64 ? SET : RESET);
}

// Función de interrupción para parpadeo del LED
void Timer2_Callback(void) {
    gpio_TogglePin(&userLed); // Cambiar el estado del LED
}

// Función de interrupción para el temporizador del display
void Timer3_Callback(void) {
    static uint8_t nextDigit = 1;
    switch (nextDigit) {
        case 1:
            gpio_WritePin(&userDis1, SET);
            displayNumber(display_value / 1000);
            break;
        case 2:
            gpio_WritePin(&userDis2, SET);
            displayNumber((display_value / 100) % 10);
            break;
        case 3:
            gpio_WritePin(&userDis3, SET);
            displayNumber((display_value / 10) % 10);
            break;
        case 4:
            gpio_WritePin(&userDis4, SET);
            displayNumber(display_value % 10);
            break;
    }
    nextDigit = (nextDigit % 4) + 1; // Avanzar al siguiente dígito
}

int main(void) {
    init_System();

    while (1) {
        // Aquí no es necesario hacer nada más
        delay_ms(10); // Solo un pequeño delay para evitar sobrecargar la CPU
    }
}
