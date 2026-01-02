#pragma once

#include "main.h"
#include "stm32f4xx_hal_def.h"

// subject to change
#define BARO_CS_PORT GPIOC
#define BARO_CS_PIN GPIO_PIN_0

// Register Hex Values
#define MS5611_RESET_REG 0x1E
#define MS5611_OSR_D1_4096_REG 0x48
#define MS5611_OSR_D2_4096_REG 0x58
#define MS5611_ADC_REG 0x00
#define MS5611_BASE_PROM_REG 0xA0

void ms5611Reset(void);
HAL_StatusTypeDef ms5611ReadPROM(uint16_t out[8]);
void ms5611GetPressureAndTemp(uint16_t prom[8], int32_t *pressure, int32_t *temperature);
