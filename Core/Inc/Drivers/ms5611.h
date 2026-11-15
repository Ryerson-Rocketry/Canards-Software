/*
 * ms5611.h
 *
 * Author: Protsenko
 *
 * Driver for the TE MS5611-01BA03 barometer over SPI.
 *
 */

#ifndef MS5611_H
#define MS5611_H

#include <stdint.h>
#include "stm32f4xx_hal.h"

#define MS5611_RESET_WAIT_MS 4

typedef enum
{
    MS5611_OK = 0,
    MS5611_ERR_SPI,  // HAL SPI transfer failed or timed out
    MS5611_ERR_CRC,  // PROM calibration data failed its CRC-4 check
    MS5611_ERR_RANGE // ADC returned 0 (read too early / no chip) or result outside datasheet limits
} MS5611_Status_t;


typedef enum
{
    MS5611_OSR_256 = 0,
    MS5611_OSR_512,
    MS5611_OSR_1024,
    MS5611_OSR_2048,
    MS5611_OSR_4096
} MS5611_OSR_t;

typedef struct
{
    SPI_HandleTypeDef *hspi;
    GPIO_TypeDef *csPort;
    uint16_t csPin;
    MS5611_OSR_t osr;
    uint16_t prom[8];
} MS5611_t;

/** @brief Send the reset command. Wait MS5611_RESET_WAIT_MS before MS5611_ReadPROM. */
MS5611_Status_t MS5611_Reset(const MS5611_t *dev);

/** @brief Read all 8 PROM words into dev->prom and verify the CRC-4. */
MS5611_Status_t MS5611_ReadPROM(MS5611_t *dev);

/** @brief Start a pressure (D1) / temperature (D2) conversion at dev->osr. */
MS5611_Status_t MS5611_StartPressure(const MS5611_t *dev);
MS5611_Status_t MS5611_StartTemperature(const MS5611_t *dev);

/** @brief Wait time (ms) to pass to osDelay after a Start call, including the 1-tick margin. */
uint32_t MS5611_ConvWaitMs(const MS5611_t *dev);

/**
 * @brief Read the 24-bit result of the last conversion.
 * @return MS5611_ERR_RANGE if the result is 0 (read before conversion finished, or no chip)
 */
MS5611_Status_t MS5611_ReadADC(const MS5611_t *dev, uint32_t *raw);

/**
 * @brief Convert raw D1/D2 into pressure and temperature using the datasheet
 *        first and second order compensation. No bus access.
 *
 * @param pressurePa  pressure in Pa (= mbar x 100)
 * @param tempCentiC  sensor die temperature in 0.01 degC (2007 = 20.07 degC). Reads above
 *                    air temperature on a powered board. Do not offset it: the pressure
 *                    compensation needs the die temperature.
 * @return MS5611_ERR_RANGE if outside 10-1200 mbar or -40..85 degC
 */
MS5611_Status_t MS5611_Compensate(const MS5611_t *dev, uint32_t d1, uint32_t d2,
                                  int32_t *pressurePa, int32_t *tempCentiC);

#endif /* MS5611_H */
