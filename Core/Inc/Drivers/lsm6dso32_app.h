#ifndef LSM6DSO32_APP_H
#define LSM6DSO32_APP_H

#include "main.h"

/* Function Prototypes */
HAL_StatusTypeDef LSM6DSO32_Rocket_Init(SPI_HandleTypeDef *hspi);
void LSM6DSO32_Read_Accel(float *accel_mg);
void LSM6DSO32_Read_Gyro(float *gyro_dps);

#endif