/*************************************************************************************
 Title	:   Analog Devices AD9833 DDS Wave Generator Library for STM32 Using HAL Libraries
 Author:    CHATGPT
 Software:  IAR Embedded Workbench for ARM
 Hardware:  Any STM32 device
*************************************************************************************/
#include <math.h>
#include "AD9833.h"
#include "main.h"
// ------------------- Variables ----------------
uint16_t FRQLW = 0;    // MSB of Frequency Tuning Word
uint16_t FRQHW = 0;    // LSB of Frequency Tuning Word
uint32_t  phaseVal=0;  // Phase Tuning Value
uint8_t WKNOWN=0;      // Flag Variable
// -------------------------------- Functions --------------------------------

// ------------------------------------------------ Software SPI Function



static SPI_HandleTypeDef *spi; // SPI handle interno
static uint16_t current_waveform = 0;

static void AD9833_WriteRegister(uint16_t reg) {
    uint8_t data[2];
    data[0] = (reg >> 8) & 0xFF;
    data[1] = reg & 0xFF;

    HAL_GPIO_WritePin(FSYNC_GPIO_Port, FSYNC_Pin , GPIO_PIN_RESET);
    HAL_SPI_Transmit(spi, data, 2, HAL_MAX_DELAY);
    HAL_GPIO_WritePin(FSYNC_GPIO_Port, FSYNC_Pin , GPIO_PIN_SET);
}

void AD9833_SetFrequency(uint32_t freq_hz) {
    uint32_t freq_word = (uint32_t)((float)freq_hz * 268435456.0f / AD9833_MCLK);
    uint16_t lsb = (uint16_t)(freq_word & 0x3FFF) | 0x4000; // FREQ0 LSB
    uint16_t msb = (uint16_t)((freq_word >> 14) & 0x3FFF) | 0x4000; // FREQ0 MSB

    // Reset while programming
    AD9833_WriteRegister(0x2100); // B28=1, RESET=1
    AD9833_WriteRegister(lsb);
    AD9833_WriteRegister(msb);
    AD9833_SetPhase(0); // default
    AD9833_WriteRegister(0x2000); // RESET=0, salida seno
    AD9833_SetWaveform(current_waveform);
}

void AD9833_SetWaveform(uint8_t type) {
    current_waveform = type;
    switch (type) {
        case 0: AD9833_WriteRegister(0x2000); break; // seno
        case 1: AD9833_WriteRegister(0x2028); break; // cuadrada
        case 2: AD9833_WriteRegister(0x2002); break; // triangular
    }
}

void AD9833_SetPhase(float degrees) {
    if (degrees < 0.0f) degrees = 0.0f;
    if (degrees > 360.0f) degrees = 360.0f;

    uint16_t word = (uint16_t)((degrees * 4096.0f / 360.0f)) & 0x0FFF;
    AD9833_WriteRegister(word | 0xC000); // PHASE0
}

void AD9833_Init(SPI_HandleTypeDef *hspi) {
    spi = hspi;
	// Default config
	HAL_GPIO_WritePin(FSYNC_GPIO_Port, FSYNC_Pin , GPIO_PIN_SET);
	HAL_Delay(10);

	AD9833_WriteRegister(0x2100); // RESET
	AD9833_SetFrequency(1000);   // 1 kHz
	AD9833_SetWaveform(0);       // seno
}

