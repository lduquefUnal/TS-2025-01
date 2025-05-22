/*
 * systickDriver.c
 *
 *  Created on: May 22, 2025
 *      Author: luisduquefranco
 */
#include <stdint.h>
#include <stdbool.h>
#include <stdio.h>
#include <stm32f4xx.h>
#include "sysTickDriver.h"


uint64_t ticks = 0;
uint64_t ticks_start = 0;
uint64_t ticks_counting = 0;
void config_SysTick_ms(uint8_t systemClock){
	//Reiniciamos el valor de la variable que cuenta el tiempo
	ticks=0 ;
	// Cargando el valor del limite de incrementos que representan 1ms
	switch(systemClock){
	// Caso para el reloj HSI ->16MHz
	case 0:
		SysTick -> LOAD = SYSTICK_LOAD_VALUE_16MHz_1ms ;
		break;
	// Caso para el reloj HSE
	case 1:
		SysTick -> LOAD = SYSTICK_LOAD_VALUE_16MHz_1ms ;
		break;
	// Caso para el reloj PLL a 100 MHz
	case 2:
		SysTick -> LOAD = SYSTICK_LOAD_VALUE_100MHz_1ms ;
		break;
	// En caso de que se ingrese un valor diferente
	default:
		SysTick -> LOAD = SYSTICK_LOAD_VALUE_100MHz_1ms ;
	}
	// limpiamos el valor actual del SysTick
	SysTick -> VAL =0 ;
	//Configuramos el reloj interno como el reloj para el Timer
	SysTick -> CTRL = SysTick_CTRL_CLKSOURCE_Msk;
	//Desactivamos las interrupciones globales
	__disable_irq();
	//Matriculamos la interrupciones en el NVIC
	NVIC_EnableIRQ(SysTick_IRQn);
	// Activamos la interrupción debida al conteo a cero del SysTick
	SysTick->CTRL |= SysTick_CTRL_TICKINT_Msk;
	// Activamos el Timer
	SysTick->CTRL |= SysTick_CTRL_ENABLE_Msk;
	// Activamos de nuevo las interrupciones globales
	__enable_irq();
}

uint64_t getTicks_ms(void){
	return ticks;
}

void delay_ms(uint32_t wait_time_ms){
	// Captura el primer Valor de tiempo para comparar
	ticks_start = getTicks_ms();
	// Captura el segundo valor de tiempo para comparar
	ticks_counting = getTicks_ms();
	// Compara: si el valor "counting" es menor que el "start+wait"
	//Actualiza el valor counting
	// Repite está operación hasta que conting sea mayor (se cumple el tiempo de espera
	while(ticks_counting < (ticks_start)+(uint64_t)wait_time_ms){
		//actualizar el valor
		ticks_counting = getTicks_ms();
	}
}

void SysTick_Handler(void){
	// Verificamos que la interrupción se lanzo
	if(SysTick -> CTRL & SysTick_CTRL_COUNTFLAG_Msk){
		//limpiamos la bandera
		SysTick -> CTRL &= ~SysTick_CTRL_COUNTFLAG_Msk;
		//Incrementamos 1 en el contador
		ticks++;
	}

}


