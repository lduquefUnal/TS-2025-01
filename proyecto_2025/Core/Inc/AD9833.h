#ifndef INC_AD9833_H_
#define INC_AD9833_H_

#include "stm32f4xx_hal.h"



// Reloj maestro del AD9833 (típico: 25 MHz)
#define AD9833_MCLK 25000000UL

void AD9833_Init(SPI_HandleTypeDef *hspi);
void AD9833_SetFrequency(uint32_t freq_hz);
void AD9833_SetWaveform(uint8_t type); // 0=seno, 1=cuadrada, 2=triangular
void AD9833_SetPhase(float degrees);

#endif /* INC_AD9833_H_ */
