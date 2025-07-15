#include "main.h"
#include "MPU6050.h"
#include <stdio.h>
#include <string.h>


// Si AD0 está en GND (0V), usa 0x68.
#define MPU6050_I2C_ADDR_7BIT 0x68

#define MPU6050_ADDR (MPU6050_I2C_ADDR_7BIT << 1)
#define CONFIG_REG 0x1A
// === FIN DE MODIFICACIÓN ===

#define SMPLRT_DIV_REG 0x19
#define GYRO_CONFIG_REG 0x1B
#define ACCEL_CONFIG_REG 0x1C
#define ACCEL_XOUT_H_REG 0x3B
#define TEMP_OUT_H_REG 0x41
#define GYRO_XOUT_H_REG 0x43
#define PWR_MGMT_1_REG 0x6B
#define WHO_AM_I_REG 0x75
#define INT_ENABLE_REG 0x38
#define INT_PIN_CFG_REG 0x37

int16_t Accel_X_RAW, Accel_Y_RAW, Accel_Z_RAW = 0;
int16_t Gyro_X_RAW, Gyro_Y_RAW, Gyro_Z_RAW = 0;
float Ax, Ay, Az = 0;
float Gx, Gy, Gz = 0;

extern I2C_HandleTypeDef hi2c1;  // MODIFICACIÓN: Declaración externa para los handles
extern UART_HandleTypeDef huart2; // MODIFICACIÓN: Usados en esta librería

void MPU6050_Init (void)
{
    HAL_StatusTypeDef ret;
    uint8_t check, Data;
    char uart_buf[100];

    // 1. VERIFICAR LA IDENTIDAD DEL DISPOSITIVO (WHO_AM_I)
    ret = HAL_I2C_Mem_Read(&hi2c1, MPU6050_ADDR, WHO_AM_I_REG, 1, &check, 1, 1000);
    if (ret != HAL_OK) {
        sprintf(uart_buf, "Error I2C leyendo WHO_AM_I. Codigo: %d\r\n", ret);
        HAL_UART_Transmit(&huart2, (uint8_t*)uart_buf, strlen(uart_buf), 1000);
        return;
    }

    if (check == 0x68) { // El valor por defecto es 0x68 [cite: 990]
        sprintf(uart_buf, "MPU6050 encontrado en 0x%X. WHO_AM_I = 0x%X\r\n", MPU6050_I2C_ADDR_7BIT, check);
        HAL_UART_Transmit(&huart2, (uint8_t*)uart_buf, strlen(uart_buf), 1000);

        Data = 0x01;
        ret = HAL_I2C_Mem_Write(&hi2c1, MPU6050_ADDR, PWR_MGMT_1_REG, 1, &Data, 1, 1000);
        if (ret != HAL_OK) {
            HAL_UART_Transmit(&huart2, (uint8_t*)"Error al configurar PWR_MGMT_1 (Reloj)\r\n", 40, 1000);
            return;
        }

        Data = 0x01;
        ret = HAL_I2C_Mem_Write(&hi2c1, MPU6050_ADDR, CONFIG_REG, 1, &Data, 1, 1000);
        if (ret != HAL_OK) {
            HAL_UART_Transmit(&huart2, (uint8_t*)"Error al escribir en CONFIG (DLPF)\r\n", 35, 1000);
            return;
        }


        Data = 0x00; // SMPLRT_DIV = 0 -> Sample Rate = 1kHz / (1 + 0)
        ret = HAL_I2C_Mem_Write(&hi2c1, MPU6050_ADDR, SMPLRT_DIV_REG, 1, &Data, 1, 1000);
        if (ret != HAL_OK) {
            HAL_UART_Transmit(&huart2, (uint8_t*)"Error al escribir en SMPLRT_DIV\r\n", 32, 1000);
            return;
        }

        // 4. CONFIGURAR RANGO DEL ACELERÓMETRO Y GIROSCOPIO
        Data = 0x00; // AFS_SEL = 0 -> ±2g [cite: 264]
        HAL_I2C_Mem_Write(&hi2c1, MPU6050_ADDR, ACCEL_CONFIG_REG, 1, &Data, 1, 1000);
        Data = 0x00; // FS_SEL = 0 -> ±250 °/s [cite: 229]
        HAL_I2C_Mem_Write(&hi2c1, MPU6050_ADDR, GYRO_CONFIG_REG, 1, &Data, 1, 1000);

        // 5. CONFIGURAR EL PIN DE INTERRUPCIÓN
        // Lógica activa alta, push-pull, pulso de 50us, limpiar al leer INT_STATUS. [cite: 547, 551, 553, 557]
        Data = 0x00;
        ret = HAL_I2C_Mem_Write(&hi2c1, MPU6050_ADDR, INT_PIN_CFG_REG, 1, &Data, 1, 1000);
        if (ret != HAL_OK) {
            HAL_UART_Transmit(&huart2, (uint8_t*)"Error al escribir en INT_PIN_CFG\r\n", 33, 1000);
            return;
        }

        // 6. HABILITAR LA INTERRUPCIÓN DE 'DATA READY'
        Data = 0x01; // Habilitar DATA_RDY_EN (bit 0) [cite: 577, 581]
        ret = HAL_I2C_Mem_Write(&hi2c1, MPU6050_ADDR, INT_ENABLE_REG, 1, &Data, 1, 1000);
        if (ret != HAL_OK) {
            HAL_UART_Transmit(&huart2, (uint8_t*)"Error al escribir en INT_ENABLE\r\n", 32, 1000);
            return;
        }

        HAL_UART_Transmit(&huart2, (uint8_t*)"MPU6050 inicializado y configurado para INT a 1kHz!\r\n", 53, 1000);
    } else {
        sprintf(uart_buf, "MPU6050 no encontrado. WHO_AM_I = 0x%X (Esperado 0x68)\r\n", check);
        HAL_UART_Transmit(&huart2, (uint8_t*)uart_buf, strlen(uart_buf), 1000);
    }
}

void MPU6050_Read_Accel (void)
{
  uint8_t Rec_Data[6];


  if (HAL_I2C_Mem_Read(&hi2c1, MPU6050_ADDR, ACCEL_XOUT_H_REG, 1, Rec_Data, 6, 100) != HAL_OK)
  {
      Ax = Ay = Az = 0;
      return;
  }
  // === FIN DE MODIFICACIÓN ===

  Accel_X_RAW = (int16_t)(Rec_Data[0] << 8 | Rec_Data[1]);
  Accel_Y_RAW = (int16_t)(Rec_Data[2] << 8 | Rec_Data[3]);
  Accel_Z_RAW = (int16_t)(Rec_Data[4] << 8 | Rec_Data[5]);

  Ax = Accel_X_RAW / 1638.4;
  Ay = Accel_Y_RAW / 1638.4;
  Az = Accel_Z_RAW / 1638.4;
}

void MPU6050_Read_Gyro (void)
{
  uint8_t Rec_Data[6];

  if (HAL_I2C_Mem_Read(&hi2c1, MPU6050_ADDR, GYRO_XOUT_H_REG, 1, Rec_Data, 6, 100) != HAL_OK)
  {
      Gx = Gy = Gz = 0;
      return;
  }
  // === FIN DE MODIFICACIÓN ===

  Gyro_X_RAW = (int16_t)(Rec_Data[0] << 8 | Rec_Data [1]);
  Gyro_Y_RAW = (int16_t)(Rec_Data[2] << 8 | Rec_Data [3]);
  Gyro_Z_RAW = (int16_t)(Rec_Data[4] << 8 | Rec_Data [5]);

  Gx = Gyro_X_RAW / 131.0;
  Gy = Gyro_Y_RAW / 131.0;
  Gz = Gyro_Z_RAW / 131.0;
}
