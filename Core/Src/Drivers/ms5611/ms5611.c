#include "ms5611.h"
#include "spi.h"
#include "math.h"
#include "cmsis_os2.h"
#include "FreeRTOS.h"

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

void ms5611Reset(void)
{
    BARO_CS_LOW();
    ms5611Write(MS5611_RESET_REG);
    BARO_CS_HIGH();
    osDelay(3);
}

uint32_t ms5611ReadADC(void)
{
    uint8_t tx = MS5611_ADC_REG;
    uint8_t rx[3];

    BARO_CS_LOW();
    ms5611Read(tx, rx, 3);
    BARO_CS_HIGH();

    return ((uint32_t)(rx[0] << 16)) | ((uint32_t)rx[1] << 8) | rx[2];
}

// what?? I can't even find the formula for this or I am just blind
uint8_t crc4(uint16_t n_prom[]) {
  uint16_t n_rem = 0; // CRC remainder
  
  // Make a copy of the last word and clear the 4 CRC bits
  uint16_t crc_read = n_prom[7];
  n_prom[7] = (n_prom[7] & 0xFFF0); 

  for (uint8_t cnt = 0; cnt < 16; cnt++) { // Loop for all 16 bytes
    // Select MSB or LSB
    if (cnt % 2 == 1) {
      n_rem ^= (uint16_t)((n_prom[cnt >> 1]) & 0x00FF);
    } else {
      n_rem ^= (uint16_t)(n_prom[cnt >> 1] >> 8);
    }

    for (uint8_t n_bit = 8; n_bit > 0; n_bit--) {
      if (n_rem & 0x8000) {
        // The polynomial is 0x3000 (equivalent to 0x13 in 4-bit)
        n_rem = (n_rem << 1) ^ 0x3000;
      } else {
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

    uint8_t crc_calculated = crc4(out);
    uint8_t crc_stored = (out[7] & 0x000F);
    
    if (crc_calculated == crc_stored) {
        return;
    } else {
        for (int i = 0; i < 8; i++) {
            out[i] = 0;
        }
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

    // read digital pressure and temp
    uint32_t D1 = ms5611ReadUncompensatedPressure();
    uint32_t D2 = ms5611ReadUncompensatedTemp();

    // calc temp
    int32_t dT = (int32_t)D2 - ((int32_t)prom[5] << 8);
    int32_t TEMP = 2000 + (((int64_t)dT * prom[6]) / (1 << 23));

    // calc temp compensated pressure
    int64_t OFF = ((int64_t)prom[2] << 16) + (((int64_t)prom[4] * dT) / (1 << 7));
    int64_t SENS = ((int64_t)prom[1] << 15) + (((int64_t)prom[3] * dT) / (1 << 8));

    int64_t T2 = 0, OFF2 = 0, SENS2 = 0;

    if (TEMP < 2000)
    {
        T2 = (int32_t)(((int64_t)(dT * dT)) >> 31);
        OFF2 = 5 * ((TEMP - 2000) * (TEMP - 2000)) / 2;
        SENS2 = OFF2 / 2;

        if (TEMP < -1500)
        {
            int64_t t3 = (int64_t)TEMP + 1500;
            int64_t t3s = t3 * t3;
            OFF2 += 7 * t3s;
            SENS2 += (11 * t3s) >> 1;
        }
    }

    TEMP -= T2;
    OFF -= OFF2;
    SENS -= SENS2;

    int32_t P = (int32_t)((((int64_t)D1 * SENS) >> 21) - OFF);

    *pressure = P;       // in Pa
    *temperature = TEMP; // in 0.01 C
}
