#include "ms5611.h"
#include "spi.h"
#include "math.h"
#include "cmsis_os.h"
#include "FreeRTOS.h"
#include "stm32f4xx_hal_def.h"

#define BARO_CS_GPIO_PORT BARO_CS_PORT
#define BARO_CS_GPIO_PIN BARO_CS_PIN

static inline void BARO_CS_LOW(void)
{
    HAL_GPIO_WritePin(BARO_CS_GPIO_PORT, BARO_CS_GPIO_PIN, GPIO_PIN_RESET);
}

static inline void BARO_CS_HIGH(void)
{
    HAL_GPIO_WritePin(BARO_CS_GPIO_PORT, BARO_CS_GPIO_PIN, GPIO_PIN_SET);
}

static HAL_StatusTypeDef ms5611Write(uint8_t val)
{
    BARO_CS_LOW();
    HAL_StatusTypeDef status = HAL_SPI_Transmit(&hspi2, &val, sizeof(val), 100);
    BARO_CS_HIGH();
    return status;
}

static HAL_StatusTypeDef ms5611Read(uint8_t reg, uint8_t *val, int length)
{
    BARO_CS_LOW();
    HAL_StatusTypeDef status = HAL_SPI_Transmit(&hspi2, &reg, 1, 100);
    if (status == HAL_OK)
    {
        status = HAL_SPI_Receive(&hspi2, val, length, 100);
    }
    BARO_CS_HIGH();
    return status;
}

HAL_StatusTypeDef ms5611Reset(void)
{
    HAL_StatusTypeDef status = ms5611Write(MS5611_RESET_REG);
    osDelay(2);
    return status;
}

uint32_t ms5611ReadADC(void)
{
    uint8_t tx = MS5611_ADC_REG;
    uint8_t rx[3];

    ms5611Read(tx, rx, 3);

    return ((uint32_t)(rx[0] << 16)) | ((uint32_t)rx[1] << 8) | rx[2];
}

// what?? I can't even find the formula for this or I am just blind
uint8_t crc4(uint16_t n_prom[])
{
    uint16_t n_rem = 0; // CRC remainder

    // Make a copy of the last word and clear the 4 CRC bits
    uint16_t crc_read = n_prom[7];
    n_prom[7] = (n_prom[7] & 0xFFF0);

    for (uint8_t cnt = 0; cnt < 16; cnt++)
    { // Loop for all 16 bytes
        // Select MSB or LSB
        if (cnt % 2 == 1)
        {
            n_rem ^= (uint16_t)((n_prom[cnt >> 1]) & 0x00FF);
        }
        else
        {
            n_rem ^= (uint16_t)(n_prom[cnt >> 1] >> 8);
        }

        for (uint8_t n_bit = 8; n_bit > 0; n_bit--)
        {
            if (n_rem & 0x8000)
            {
                // The polynomial is 0x3000 (equivalent to 0x13 in 4-bit)
                n_rem = (n_rem << 1) ^ 0x3000;
            }
            else
            {
                n_rem = (n_rem << 1);
            }
        }
    }

    // Restore the original value of prom[7] (optional, but good practice)
    n_prom[7] = crc_read;

    // Final 4-bit remainder is the CRC
    n_rem = ((n_rem >> 12) & 0x000F);

    return (n_rem ^ 0x00);
}

HAL_StatusTypeDef ms5611ReadPROM(uint16_t out[8])
{

    for (uint8_t addr = 0; addr < 8; addr++)
    {
        uint8_t cmd = MS5611_BASE_PROM_REG | (addr << 1);
        uint8_t rx[2];

        ms5611Read(cmd, rx, 2);

        // MSB first
        out[addr] = (uint16_t)(rx[0] << 8) | rx[1];
    }

    if (out[5] == 0 || out[6] == 0)
    {
        return HAL_ERROR;
    }

    uint8_t crc_calculated = crc4(out);
    uint8_t crc_stored = (out[7] & 0x000F);

    if (crc_calculated == crc_stored)
    {
        return HAL_OK;
    }
    return HAL_ERROR;
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

HAL_StatusTypeDef ms5611GetPressureAndTemp(uint16_t prom[8], int32_t *pressure, int32_t *temperature)
{
    if (prom[1] == 0 && prom[2] == 0)
    {
        *pressure = 0;
        *temperature = 0;
        return HAL_ERROR;
    }

    // read digital pressure and temp
    uint32_t D1 = ms5611ReadUncompensatedPressure();
    uint32_t D2 = ms5611ReadUncompensatedTemp();

    // calc temp
    int32_t dT = (int32_t)D2 - ((int32_t)prom[5] << 8);
    int32_t TEMP = 2000 + (((int64_t)dT * (int64_t)prom[6]) >> 23);

    // calc temp compensated pressure
    int64_t OFF = ((int64_t)prom[2] << 16) + (((int64_t)prom[4] * dT) / (1 << 7));
    int64_t SENS = ((int64_t)prom[1] << 15) + (((int64_t)prom[3] * dT) / (1 << 8));

    if (TEMP < 2000)
    {
        int64_t T2 = ((int64_t)dT * (int64_t)dT) >> 31;
        int64_t OFF2 = (5 * ((int64_t)TEMP - 2000) * ((int64_t)TEMP - 2000)) >> 1;
        int64_t SENS2 = (5 * ((int64_t)TEMP - 2000) * ((int64_t)TEMP - 2000)) >> 2;

        if (TEMP < -1500)
        {
            int64_t diff = (int64_t)TEMP + 1500;
            OFF2 += 7 * (diff * diff);
            SENS2 += (11 * (diff * diff)) >> 1;
        }

        TEMP -= (int32_t)T2;
        OFF -= OFF2;
        SENS -= SENS2;
    }

    int32_t P = (int32_t)(((((int64_t)D1 * SENS) >> 21) - OFF) >> 15);

    *pressure = P;       // in Pa
    *temperature = TEMP; // in 0.01 C
    return HAL_OK;
}
