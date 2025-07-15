/*
 * MPU6050.h
 *
 *  Created on: Oct 6, 2022
 *      Author: mac
 */

#ifndef INC_MPU6050_H_
#define INC_MPU6050_H_

#include <stdio.h>
#include "stm32f4xx.h"
#define MPU6050_I2C_ADDR_7BIT   0x68
#define MPU6050_ADDR            (MPU6050_I2C_ADDR_7BIT << 1)
#define ACCEL_XOUT_H_REG        0x3B
extern I2C_HandleTypeDef hi2c1;
extern UART_HandleTypeDef huart1;

void MPU6050_Init(void);
void MPU6050_Read_Accel(void);
void MPU6050_Read_Gyro(void);

extern int16_t Accel_X_RAW, Accel_Y_RAW, Accel_Z_RAW;
extern int16_t Gyro_X_RAW, Gyro_Y_RAW, Gyro_Z_RAW;
extern float Ax, Ay, Az;
extern float Gx, Gy, Gz;

#endif /* INC_MPU6050_H_ */
