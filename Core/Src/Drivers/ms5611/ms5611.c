#include "ms5611.h"
#include "spi.h"
#include "math.h"
#include "cmsis_os2.h"
#include "FreeRTOS.h"

#define BARO_CS_GPIO_PORT BARO_CS_PORT
#define BARO_CS_GPIO_PIN BARO_CS_PIN

static inline void BARO_CS_LOW()
{
    HAL_GPIO_WritePin(BARO_CS_GPIO_PORT, BARO_CS_GPIO_PIN, GPIO_PIN_RESET);
}

static inline void BARO_CS_HIGH()
{
    HAL_GPIO_WritePin(BARO_CS_GPIO_PORT, BARO_CS_GPIO_PIN, GPIO_PIN_SET);
}

HAL_StatusTypeDef ms5611Write(uint8_t val)
{
    BARO_CS_LOW();
    HAL_StatusTypeDef status = HAL_SPI_Transmit(&hspi3, &val, sizeof(val), HAL_MAX_DELAY);
    BARO_CS_HIGH();
    return status;
}

void ms5611Read(uint8_t reg, uint8_t val, int length)
{
    BARO_CS_LOW();
    HAL_SPI_Transmit(&hspi3, &reg, 1, HAL_TIMEOUT);
    HAL_SPI_Receive(&hspi3, &val, length, HAL_MAX_DELAY);
    BARO_CS_LOW();
}

void ms5611Reset()
{
    BARO_CS_LOW();
    ms5611Write(MS5611_RESET_REG);
    BARO_CS_HIGH();
    osDelay(3);
}

uint32_t ms5611ReadADC()
{
    uint8_t tx = MS5611_ADC_REG;
    uint8_t rx[3];

    BARO_CS_LOW();
    HAL_SPI_TransmitReceive(&hspi3, &tx, rx, 3, HAL_MAX_DELAY);
    BARO_CS_HIGH();

    return ((uint32_t)(rx[0] << 16)) | ((uint32_t)rx[1] << 8) | rx[0];
}

void ms5611ReadPROM(uint16_t out[8])
{

    for (uint8_t addr = 0; addr < 8; addr++)
    {
        uint8_t cmd = MS5611_BASE_PROM_REG | (addr << 1);
        uint8_t rx[2];

        BARO_CS_LOW();
        ms5611Read(cmd, rx, 2);
        BARO_CS_HIGH();

        // MSB first
        out[addr] = (uint16_t)(rx[0] << 8) | rx[1];
    }
}

uint32_t ms5611ReadUncompensatedPressure(void)
{
    ms5611Write(MS5611_OSR_D1_4096_REG);
    osDelay(10);
    return ms5611ReadADC();
}

uint32_t ms5611ReadUncompensatedTemp(void)
{
    ms5611Write(MS5611_OSR_D2_4096_REG);
    osDelay(10);
    return ms5611ReadADC();
}

void ms5611GetPressureAndTemp(uint16_t prom[8], int32_t *pressure, int32_t *temperature)
{
    uint32_t D1 = ms5611ReadUncompensatedPressure();
    uint32_t D2 = ms5611ReadUncompensatedTemp();

    int32_t dT = (int32_t)D2 - ((int32_t)prom[5] << 8);
    int32_t TEMP = 2000 + ((int64_t)dT * prom[6]) / (1 << 23);

    int64_t OFF = ((int64_t)prom[2] << 16) + (((int64_t)prom[4] * dT) / (1 << 7));
    int64_t SENS = ((int64_t)prom[1] << 15) + (((int64_t)prom[3] * dT) / (1 << 8));

    if (TEMP < 2000)
    {
        int32_t T2 = ((int64_t)dT * dT) >> 31;
        int64_t OFF2 = 5 * ((TEMP - 2000) * (TEMP - 2000)) / 2;
        int64_t SENS2 = OFF2 / 2;

        TEMP -= T2;
        OFF -= OFF2;
        SENS -= SENS2;
    }

    int32_t P = (((D1 * SENS) >> 21) - OFF) >> 15;

    *pressure = P;
    *temperature = TEMP;
}
