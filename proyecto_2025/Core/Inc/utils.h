/*
 * utils.h
 *
 *  Created on: Jul 23, 2025
 *      Author: luisduquefranco
 */

#ifndef INC_UTILS_H_
#define INC_UTILS_H_

#include "main.h"


void RunBlockingSweep(void);
void I2C_Scanner(void);
void SPI_Debug_AD9833(void);
void mostrar_LCD(void);
void SendCaptureDataOverUART(uint16_t num_samples);
void RunBlockingSingle(uint32_t frequency_hz);
void LogSweep(void);
void RunBlockingSweep(void);

#ifdef __cplusplus
}
#endif
#endif /* INC_UTILS_H_ */
