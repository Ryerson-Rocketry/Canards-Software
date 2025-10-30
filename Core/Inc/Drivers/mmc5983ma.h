#pragma once

#include "main.h"
#include "FreeRTOS.h"
#include "semphr.h"
#include "cmsis_os.h"
#include "stm32f4xx_hal_def.h"


#define MAG_ADDRESS 0x30
#define MAG_CTRL_REG_0 0x09
#define MAG_CTRL_REG_1 0x0A
#define MAG_CTRL_REG_2 0x0B
#define MAG_STATUS_REG 0x08

#define MAG_XOUT_0 0x00

#define MAG_PIN_INT_VAL GPIO_PIN_12

HAL_StatusTypeDef magInit(void);
HAL_StatusTypeDef magGetData(SemaphoreHandle_t magDataReadySemaphore, float magData[4]);
