#include "i2c.h"
#include "main.h"
#include "projdefs.h"
#include "mmc5983ma.h"
#include <stdbool.h>
#include "FreeRTOS.h"
#include "semphr.h"
#include "cmsis_os.h"

extern I2C_HandleTypeDef hi2c1;
extern SemaphoreHandle_t gI2c1Mutex;

/**
 * @brief  Low-level I2C write function with mutex protection.
 */
HAL_StatusTypeDef write(uint8_t regAddress, uint8_t data)
{
    HAL_StatusTypeDef status = HAL_ERROR;

    // Take the I2C Mutex, wait up to 100ms
    if (xSemaphoreTake(gI2c1Mutex, pdMS_TO_TICKS(100)) == pdTRUE)
    {
        status = HAL_I2C_Mem_Write(
            &hi2c1,
            MAG_ADDRESS << 1,
            regAddress,
            I2C_MEMADD_SIZE_8BIT,
            &data,
            1,
            500);

        xSemaphoreGive(gI2c1Mutex);
    }
    else
    {
        status = HAL_TIMEOUT;
    }

    return status;
}

/**
 * @brief  Low-level I2C read function with mutex protection.
 */
HAL_StatusTypeDef read(uint8_t regAddress, uint8_t *out, int length)
{
    HAL_StatusTypeDef status = HAL_ERROR;

    // Take the I2C Mutex, wait up to 100ms
    if (xSemaphoreTake(gI2c1Mutex, pdMS_TO_TICKS(100)) == pdTRUE)
    {
        status = HAL_I2C_Mem_Read(
            &hi2c1,
            MAG_ADDRESS << 1,
            regAddress,
            I2C_MEMADD_SIZE_8BIT,
            out,
            length,
            500);

        xSemaphoreGive(gI2c1Mutex);
    }
    else
    {
        status = HAL_TIMEOUT;
    }

    return status;
}

/**
 * @brief  Resets the magnetometer.
 * @retval HAL_StatusTypeDef: HAL_OK on success, HAL_ERROR/HAL_TIMEOUT on failure.
 */
HAL_StatusTypeDef reset(void)
{
    HAL_StatusTypeDef status = write(MAG_CTRL_REG_1, 0x80);
    if (status != HAL_OK)
    {
        return status;
    }
    osDelay(10);
    return HAL_OK;
}

/**
 * @brief  Initializes the magnetometer with default settings.
 * @retval HAL_StatusTypeDef: HAL_OK on success, HAL_ERROR/HAL_TIMEOUT on failure.
 */
HAL_StatusTypeDef magInit(void)
{
    HAL_StatusTypeDef status;

    // CHANGED: Check the status of reset()
    status = reset();
    if (status != HAL_OK)
    {
        return status;
    }

    // get internal control reg 0
    uint8_t ctrlReg0;
    status = read(MAG_CTRL_REG_0, &ctrlReg0, 1);
    if (status != HAL_OK)
    {
        return status;
    }

    // enable SR
    status = write(MAG_CTRL_REG_0, ctrlReg0 | 0x20);
    if (status != HAL_OK)
    {
        return status;
    }

    // get internal ctrl reg 1
    uint8_t ctrlReg1;
    status = read(MAG_CTRL_REG_1, &ctrlReg1, 1);
    if (status != HAL_OK)
    {
        return status;
    }

    status = write(MAG_CTRL_REG_1, ctrlReg1 | 0x01);
    if (status != HAL_OK)
    {
        return status;
    }

    // get internal ctrl reg 2
    uint8_t ctrlReg2;
    status = read(MAG_CTRL_REG_2, &ctrlReg2, 1);
    if (status != HAL_OK)
    {
        return status;
    }

    // Keep sensor in manual/singleshot
    status = write(MAG_CTRL_REG_2, ctrlReg2 | 0x08);
    if (status != HAL_OK)
    {
        return status;
    }

    osDelay(1);
    return HAL_OK;
}

/**
 * @brief  Performs a "SET" operation.
 * @retval HAL_StatusTypeDef: HAL_OK on success, HAL_ERROR/HAL_TIMEOUT on failure.
 */
HAL_StatusTypeDef setOperation(void)
{
    HAL_StatusTypeDef status;
    uint8_t ctrlReg;

    status = read(MAG_CTRL_REG_0, &ctrlReg, 1);
    if (status != HAL_OK)
        return status;

    status = write(MAG_CTRL_REG_0, ctrlReg | 0x08);
    if (status != HAL_OK)
        return status;

    // 500ns delay
    for (volatile int i = 0; i < 50; i++)
    {
        __NOP();
    }
    return HAL_OK;
}

/**
 * @brief  Performs a "RESET" operation.
 * @retval HAL_StatusTypeDef: HAL_OK on success, HAL_ERROR/HAL_TIMEOUT on failure.
 */
HAL_StatusTypeDef resetOperation(void)
{
    HAL_StatusTypeDef status;
    uint8_t ctrlReg;

    status = read(MAG_CTRL_REG_0, &ctrlReg, 1);
    if (status != HAL_OK)
        return status;

    status = write(MAG_CTRL_REG_0, ctrlReg | 0x10);
    if (status != HAL_OK)
        return status;

    // 500ns delay
    for (volatile int i = 0; i < 50; i++)
    {
        __NOP();
    }
    return HAL_OK;
}

/**
 * @brief  Starts a single measurement.
 * @retval HAL_StatusTypeDef: HAL_OK on success, HAL_ERROR/HAL_TIMEOUT on failure.
 */
HAL_StatusTypeDef startMeas(void)
{
    // CHANGED: Added status checking
    HAL_StatusTypeDef status;
    uint8_t ctrlReg;

    status = read(MAG_CTRL_REG_0, &ctrlReg, 1);
    if (status != HAL_OK)
        return status;

    status = write(MAG_CTRL_REG_0, ctrlReg | 0x01);
    return status;
}

/**
 * @brief  Checks if a measurement is complete via the status register.
 * @param  isComplete: Pointer to a bool to store the result.
 * @retval HAL_StatusTypeDef: HAL_OK on I2C success, HAL_ERROR/HAL_TIMEOUT on I2C failure.
 */
HAL_StatusTypeDef checkMeasComplete(bool *isComplete)
{
    uint8_t statusReg;
    *isComplete = false;

    HAL_StatusTypeDef status = read(MAG_STATUS_REG, &statusReg, 1);
    if (status != HAL_OK)
    {
        return status;
    }

    if (statusReg & 0x01)
    {
        *isComplete = true;
    }

    return HAL_OK;
}

/**
 * @brief  Gets the 18-bit data from the sensor.
 * @param  res: A 3-element array to store X, Y, Z raw data.
 * @retval HAL_StatusTypeDef: HAL_OK on success, HAL_ERROR/HAL_TIMEOUT on failure.
 */
HAL_StatusTypeDef getData(uint32_t res[3])
{
    uint8_t dataBuf[7];

    HAL_StatusTypeDef status = read(MAG_XOUT_0, dataBuf, 7);
    if (status != HAL_OK)
    {
        // Don't process garbage data
        res[0] = 0;
        res[1] = 0;
        res[2] = 0;
        return status;
    }

    uint8_t x2LSB = (dataBuf[6] >> 6) & 0x03;
    uint8_t y2LSB = (dataBuf[6] >> 4) & 0x03;
    uint8_t z2LSB = (dataBuf[6] >> 2) & 0x03;

    // Reconstruct 18-bit data
    res[0] = ((uint32_t)dataBuf[1] << 10) | ((uint32_t)dataBuf[0] << 2) | x2LSB;
    res[1] = ((uint32_t)dataBuf[3] << 10) | ((uint32_t)dataBuf[2] << 2) | y2LSB;
    res[2] = ((uint32_t)dataBuf[5] << 10) | ((uint32_t)dataBuf[4] << 2) | z2LSB;

    return HAL_OK;
}

/**
 * @brief  Performs a full SET/RESET measurement cycle.
 * @param  magDataReadySemaphore: Semaphore handle to wait on for DDRY interrupt.
 * @param  magData: A 3-element array [X, Y, Z] to store final data in Gauss.
 */
HAL_StatusTypeDef magGetData(SemaphoreHandle_t magDataReadySemaphore, float *magData)
{
    HAL_StatusTypeDef status;
    bool isDone;
    uint32_t out1[3];
    uint32_t out2[3];

    // 1. Perform Set Operation
    status = setOperation();
    if (status != HAL_OK)
        return status;

    // 2. Start Mag measurement
    status = startMeas();
    if (status != HAL_OK)
        return status;

    // 3. Wait for DDRY interrupt
    if (xSemaphoreTake(magDataReadySemaphore, 100) != pdTRUE)
    {
        return HAL_TIMEOUT;
    }

    // 4. Check Measurement is done
    status = checkMeasComplete(&isDone);
    if (status != HAL_OK)
        return status;
    if (!isDone)
    {
        return HAL_ERROR;
    }

    // 5. Get first measurement
    status = getData(out1);
    if (status != HAL_OK)
        return status;

    // 6. Perform reset op
    status = resetOperation();
    if (status != HAL_OK)
        return status;

    // 7. start mag meas
    status = startMeas();
    if (status != HAL_OK)
        return status;

    // 8. Wait for DDRY interrupt
    if (xSemaphoreTake(magDataReadySemaphore, 100) != pdTRUE)
    {
        return HAL_TIMEOUT;
    }

    // 9. Check Measurement is done
    status = checkMeasComplete(&isDone);
    if (status != HAL_OK)
        return status;
    if (!isDone)
    {
        return HAL_ERROR;
    }

    // 10. Get second measurement
    status = getData(out2);
    if (status != HAL_OK)
        return status;

    // 11. Subtract two meas. and divide by 2
    for (int i = 0; i < 3; i++)
    {
        magData[i] = (((int32_t)out1[i] - (int32_t)out2[i]) / 2.0f) / 16384.0f;
    }

    return HAL_OK;
}
