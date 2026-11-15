/*
 * ms5611.c
 *
 * Author: Protsenko
 *
 * Driver for the TE MS5611-01BA03 barometer over SPI (mode 0 or 3, up to 20 MHz).
 *
 */

#include "Drivers/ms5611.h"

#define SPI_TIMEOUT_MS 5 // microseconds at any prescaler

/* Commands */
#define CMD_RESET 0x1E
#define CMD_ADC_READ 0x00
#define CMD_CONV_D1 0x40 // + 2 * OSR index
#define CMD_CONV_D2 0x50 // + 2 * OSR index
#define CMD_PROM_READ 0xA0 // + 2 * PROM address (0..7)

/* Datasheet result limits */
#define P_MIN_PA 1000   // 10 mbar
#define P_MAX_PA 120000 // 1200 mbar
#define T_MIN_CC -4000  // -40.00 degC
#define T_MAX_CC 8500   // 85.00 degC

/* Max conversion time per OSR (0.60/1.17/2.28/4.54/9.04 ms), rounded up + 1 tick for osDelay */
static const uint8_t convWaitMs[] = {2, 3, 4, 6, 11};

static inline void csLow(const MS5611_t *dev)
{
    HAL_GPIO_WritePin(dev->csPort, dev->csPin, GPIO_PIN_RESET);
}

static inline void csHigh(const MS5611_t *dev)
{
    HAL_GPIO_WritePin(dev->csPort, dev->csPin, GPIO_PIN_SET);
}

/* Send a 1-byte command with no response (reset, start conversion) */
static MS5611_Status_t sendCommand(const MS5611_t *dev, uint8_t cmd)
{
    csLow(dev);
    HAL_StatusTypeDef hal = HAL_SPI_Transmit(dev->hspi, &cmd, 1, SPI_TIMEOUT_MS);
    csHigh(dev);
    return (hal == HAL_OK) ? MS5611_OK : MS5611_ERR_SPI;
}

/* Send a command, then clock out len-1 response bytes. rx[0] is junk (clocked in while sending cmd). */
static MS5611_Status_t readCommand(const MS5611_t *dev, uint8_t cmd, uint8_t *rx, uint16_t len)
{
    uint8_t tx[4] = {cmd, 0, 0, 0};

    csLow(dev);
    HAL_StatusTypeDef hal = HAL_SPI_TransmitReceive(dev->hspi, tx, rx, len, SPI_TIMEOUT_MS);
    csHigh(dev);
    return (hal == HAL_OK) ? MS5611_OK : MS5611_ERR_SPI;
}

/* CRC-4 over the 8 PROM words, as specified in TE application note AN520 */
static uint8_t crc4(const uint16_t prom[8])
{
    uint16_t rem = 0;

    for (uint8_t i = 0; i < 16; i++)
    {
        // The CRC itself (low byte of word 7) is treated as 0 while computing
        uint16_t word = (i >> 1 == 7) ? (prom[7] & 0xFF00) : prom[i >> 1];
        rem ^= (i & 1) ? (word & 0x00FF) : (word >> 8);

        for (uint8_t bit = 0; bit < 8; bit++)
        {
            rem = (rem & 0x8000) ? (uint16_t)((rem << 1) ^ 0x3000) : (uint16_t)(rem << 1);
        }
    }
    return (rem >> 12) & 0x000F;
}

MS5611_Status_t MS5611_Reset(const MS5611_t *dev)
{
    return sendCommand(dev, CMD_RESET);
}

MS5611_Status_t MS5611_ReadPROM(MS5611_t *dev)
{
    uint8_t rx[3];

    for (uint8_t addr = 0; addr < 8; addr++)
    {
        if (readCommand(dev, (uint8_t)(CMD_PROM_READ + (addr << 1)), rx, 3) != MS5611_OK)
            return MS5611_ERR_SPI;
        dev->prom[addr] = ((uint16_t)rx[1] << 8) | rx[2];
    }

    // A missing chip reads all 0x0000 (which passes CRC) or all 0xFFFF, so reject those too
    for (uint8_t i = 1; i <= 6; i++)
    {
        if (dev->prom[i] == 0x0000 || dev->prom[i] == 0xFFFF)
            return MS5611_ERR_CRC;
    }

    return (crc4(dev->prom) == (dev->prom[7] & 0x000F)) ? MS5611_OK : MS5611_ERR_CRC;
}

MS5611_Status_t MS5611_StartPressure(const MS5611_t *dev)
{
    return sendCommand(dev, (uint8_t)(CMD_CONV_D1 + (dev->osr << 1)));
}

MS5611_Status_t MS5611_StartTemperature(const MS5611_t *dev)
{
    return sendCommand(dev, (uint8_t)(CMD_CONV_D2 + (dev->osr << 1)));
}

uint32_t MS5611_ConvWaitMs(const MS5611_t *dev)
{
    return convWaitMs[dev->osr];
}

MS5611_Status_t MS5611_ReadADC(const MS5611_t *dev, uint32_t *raw)
{
    uint8_t rx[4];

    if (readCommand(dev, CMD_ADC_READ, rx, 4) != MS5611_OK)
        return MS5611_ERR_SPI;

    *raw = ((uint32_t)rx[1] << 16) | ((uint32_t)rx[2] << 8) | rx[3];

    // 0 means the read came before the conversion finished, or no chip answered
    return (*raw == 0) ? MS5611_ERR_RANGE : MS5611_OK;
}

MS5611_Status_t MS5611_Compensate(const MS5611_t *dev, uint32_t d1, uint32_t d2,
                                  int32_t *pressurePa, int32_t *tempCentiC)
{
    const uint16_t *C = dev->prom;

    // First order. Divisions by powers of two compile to shifts.
    int32_t dT = (int32_t)d2 - ((int32_t)C[5] << 8);
    int32_t temp = 2000 + (int32_t)(((int64_t)dT * C[6]) / 8388608); // 2^23
    int64_t off = ((int64_t)C[2] << 16) + ((int64_t)C[4] * dT) / 128;
    int64_t sens = ((int64_t)C[1] << 15) + ((int64_t)C[3] * dT) / 256;

    // Second order, below 20 degC
    if (temp < 2000)
    {
        int64_t lo = (int64_t)(temp - 2000) * (temp - 2000);
        int32_t t2 = (int32_t)(((int64_t)dT * dT) >> 31);
        int64_t off2 = (5 * lo) >> 1;
        int64_t sens2 = (5 * lo) >> 2;

        if (temp < -1500)
        {
            int64_t vlo = (int64_t)(temp + 1500) * (temp + 1500);
            off2 += 7 * vlo;
            sens2 += (11 * vlo) >> 1;
        }

        temp -= t2;
        off -= off2;
        sens -= sens2;
    }

    int32_t p = (int32_t)((((int64_t)d1 * sens) / 2097152 - off) / 32768); // 2^21, 2^15

    *pressurePa = p;
    *tempCentiC = temp;

    if (p < P_MIN_PA || p > P_MAX_PA || temp < T_MIN_CC || temp > T_MAX_CC)
        return MS5611_ERR_RANGE;
    return MS5611_OK;
}
