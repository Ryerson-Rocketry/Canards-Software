#include "i2c.h"
#include "main.h"
#include "mmc5983ma.h"
#include <stdio.h>
#include "FreeRTOS.h"
#include "semphr.h"
#include "cmsis_os.h"

extern I2C_HandleTypeDef hi2c1;
extern SemaphoreHandle_t gI2c1Mutex;

// Standard 18-bit midpoint
#define MAG_OFFSET_18BIT 131072.0f

/**
 * @brief Low-level I2C write.
 */
HAL_StatusTypeDef write(uint8_t regAddress, uint8_t data)
{
    HAL_StatusTypeDef status = HAL_ERROR;
    if (xSemaphoreTake(gI2c1Mutex, pdMS_TO_TICKS(100)) == pdTRUE)
    {
        status = HAL_I2C_Mem_Write(&hi2c1, MAG_ADDRESS << 1, regAddress, 1, &data, 1, 100);
        xSemaphoreGive(gI2c1Mutex);
    }
    return status;
}

/**
 * @brief Low-level I2C read.
 */
HAL_StatusTypeDef read(uint8_t regAddress, uint8_t *out, int length)
{
    HAL_StatusTypeDef status = HAL_ERROR;
    if (xSemaphoreTake(gI2c1Mutex, pdMS_TO_TICKS(100)) == pdTRUE)
    {
        status = HAL_I2C_Mem_Read(&hi2c1, MAG_ADDRESS << 1, regAddress, 1, out, length, 100);
        xSemaphoreGive(gI2c1Mutex);
    }
    return status;
}

/**
 * @brief Clears the Status Register (0x08) flags and flushes MCU semaphore.
 */
void prepareForHardwareAction(SemaphoreHandle_t sem)
{
    uint8_t dummy;
    read(MAG_STATUS_REG, &dummy, 1);
    write(MAG_STATUS_REG, 0xFF); // Write-1-to-clear all flags
    while (xSemaphoreTake(sem, 0) == pdTRUE)
        ; // Flush semaphore
}

/**
 * @brief Resets the sensor and configures it.
 */
HAL_StatusTypeDef magInit(void)
{
    write(MAG_CTRL_REG_1, 0x80); // Software Reset
    osDelay(10);

    write(MAG_CTRL_REG_1, 0x01); // 400Hz Bandwidth
    write(MAG_CTRL_REG_2, 0x00); // Manual Mode
    write(MAG_CTRL_REG_0, 0x04); // Enable Interrupts

    printf("[MAG]: Initialized and Reset.\r\n");
    osDelay(1);
    return HAL_OK;
}

/**
 * @brief Triggers a measurement and WAITS for hardware confirmation.
 */
HAL_StatusTypeDef triggerAndWait(SemaphoreHandle_t sem)
{
    // 1. Ensure control reg is clear before triggering
    write(MAG_CTRL_REG_0, 0x00);

    // 2. Trigger Take Measurement + Interrupt
    if (write(MAG_CTRL_REG_0, 0x01 | 0x04) != HAL_OK)
        return HAL_ERROR;

    // 3. Wait for DRDY Semaphore
    if (xSemaphoreTake(sem, pdMS_TO_TICKS(200)) != pdTRUE)
    {
        printf("[DEBUG]: Semaphore Timeout\r\n");
        return HAL_TIMEOUT;
    }

    // 4. VERIFY Hardware Status
    uint8_t status;
    read(MAG_STATUS_REG, &status, 1);
    if (!(status & 0x01))
    { // Check Meas_M_Done bit
        return HAL_BUSY;
    }

    return HAL_OK;
}

/**
 * @brief Reconstructs 18-bit data.
 */
HAL_StatusTypeDef getData(uint32_t res[3])
{
    uint8_t buf[7] = {0};
    if (read(MAG_XOUT_0, buf, 7) != HAL_OK)
        return HAL_ERROR;

    res[0] = ((uint32_t)buf[0] << 10) | ((uint32_t)buf[1] << 2) | (buf[6] >> 6);
    res[1] = ((uint32_t)buf[2] << 10) | ((uint32_t)buf[3] << 2) | ((buf[6] >> 4) & 0x03);
    res[2] = ((uint32_t)buf[4] << 10) | ((uint32_t)buf[5] << 2) | ((buf[6] >> 2) & 0x03);
    return HAL_OK;
}

/**
 * @brief Full SET/RESET measurement cycle.
 */
HAL_StatusTypeDef magGetData(SemaphoreHandle_t magDataReadySemaphore, float magData[3])
{
    uint32_t out1[3], out2[3];

    // --- PHASE 1: SET ---
    write(MAG_CTRL_REG_0, 0x08);
    osDelay(5);
    prepareForHardwareAction(magDataReadySemaphore);

    if (triggerAndWait(magDataReadySemaphore) != HAL_OK)
        return HAL_ERROR;
    getData(out1);

    // --- PHASE 2: RESET ---
    write(MAG_CTRL_REG_0, 0x10);
    osDelay(5);
    prepareForHardwareAction(magDataReadySemaphore);

    if (triggerAndWait(magDataReadySemaphore) != HAL_OK)
        return HAL_ERROR;
    getData(out2);

    // --- PHASE 3: DIFFERENTIAL CALC ---
    for (int i = 0; i < 3; i++)
    {
        if (out1[i] == out2[i])
        {
            printf("[CRITICAL]: out1[%d] and out2[%d] are identical (%lu)\r\n", i, i, out1[i]);
        }
        magData[i] = ((float)((int32_t)out1[i] - (int32_t)out2[i]) / 2.0f) / 16384.0f;
    }

    return HAL_OK;
}