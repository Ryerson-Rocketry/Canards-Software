#include <stdbool.h>
#include "main.h"
#include "spi.h"
#include "bmi088.h"
#include "stm32f4xx_hal_def.h" // Good include, this defines HAL_StatusTypeDef
#include "string.h"
#include "stdint.h"
#include "math.h"
#include "cmsis_os.h"

#define ACCEL_CS_PORT ACCEL_CS_GPIO_PORT
#define ACCEL_CS_PIN ACCEL_CS_GPIO_PIN
#define GYRO_CS_PORT GYRO_CS_GPIO_PORT
#define GYRO_CS_PIN GYRO_CS_GPIO_PIN
#define GYRO_SOFTRESET true
#define ACCEL_SOFTRESET true

// ... CS Low/High functions are correct ...
static inline void ACCEL_CS_LOW(void)
{
    HAL_GPIO_WritePin(ACCEL_CS_PORT, ACCEL_CS_PIN, GPIO_PIN_RESET);
}
static inline void ACCEL_CS_HIGH(void)
{
    HAL_GPIO_WritePin(ACCEL_CS_PORT, ACCEL_CS_PIN, GPIO_PIN_SET);
}
static inline void GYRO_CS_LOW(void)
{
    HAL_GPIO_WritePin(GYRO_CS_PORT, GYRO_CS_PIN, GPIO_PIN_RESET);
}
static inline void GYRO_CS_HIGH(void)
{
    HAL_GPIO_WritePin(GYRO_CS_PORT, GYRO_CS_PIN, GPIO_PIN_SET);
}

// --- Base SPI Functions (Correct) ---

HAL_StatusTypeDef bmi088Write(uint8_t reg, uint8_t val)
{
    uint8_t tx[2] = {reg & 0x7F, val};
    return HAL_SPI_Transmit(&hspi2, tx, sizeof(tx), HAL_MAX_DELAY);
}

HAL_StatusTypeDef bmi088AccelRead(uint8_t reg, uint8_t *dataBuff, int length)
{
    if (length <= 0)
        return HAL_ERROR;
    HAL_StatusTypeDef status;
    uint8_t tx[length + 2];
    uint8_t rx[length + 2];
    tx[0] = reg | 0x80;
    memset(&tx[1], 0x00, length + 1);
    status = HAL_SPI_TransmitReceive(&hspi2, tx, rx, length + 2, HAL_MAX_DELAY);
    if (status == HAL_OK)
    {
        memcpy(dataBuff, &rx[2], length);
    }
    return status;
}

HAL_StatusTypeDef bmi088GyroRead(uint8_t reg, uint8_t *dataBuff, int length)
{
    if (length <= 0)
        return HAL_ERROR;
    HAL_StatusTypeDef status;
    uint8_t tx[length + 1];
    uint8_t rx[length + 1];
    tx[0] = reg | 0x80;
    memset(&tx[1], 0x00, length);
    status = HAL_SPI_TransmitReceive(&hspi2, tx, rx, length + 1, HAL_MAX_DELAY);
    if (status == HAL_OK)
    {
        memcpy(dataBuff, &rx[1], length);
    }
    return status;
}

// --- Configuration Functions (Refactored for Error Handling) ---

HAL_StatusTypeDef bmi088SoftReset(bool gyro, bool accel)
{
    if (!gyro && !accel)
        return HAL_ERROR;

    HAL_StatusTypeDef gyroStatus = HAL_OK;
    HAL_StatusTypeDef accelStatus = HAL_OK;

    if (gyro)
    {
        GYRO_CS_LOW();
        gyroStatus = bmi088Write(BMI088_GYR_SOFTRESET_REG, BMI088_SOFTRESET_VAL);
        GYRO_CS_HIGH();
        if (gyroStatus != HAL_OK)
            return gyroStatus;
        osDelay(30);
    }

    if (accel)
    {
        ACCEL_CS_LOW();
        accelStatus = bmi088Write(BMI088_ACC_SOFTRESET_REG, BMI088_SOFTRESET_VAL);
        ACCEL_CS_HIGH();
        if (accelStatus != HAL_OK)
            return accelStatus;
        osDelay(10);
    }

    return HAL_OK; // Both succeeded
}

HAL_StatusTypeDef bmi088SwitchGyroMode(uint8_t mode)
{
    if (mode != BMI088_GYRO_NORMAL_MODE &&
        mode != BMI088_GYRO_SUSPEND_MODE &&
        mode != BMI088_GYRO_DEEP_SUSPEND_MODE)
    {
        return HAL_ERROR; // Invalid parameter
    }

    uint8_t currentMode;
    HAL_StatusTypeDef status;

    GYRO_CS_LOW();
    status = bmi088GyroRead(BMI088_GYRO_PWR_REG, &currentMode, 1);
    GYRO_CS_HIGH();
    if (status != HAL_OK)
        return status;

    if (currentMode != BMI088_GYRO_NORMAL_MODE && mode != BMI088_GYRO_NORMAL_MODE)
    {
        // Per datasheet: to switch between suspend/deep_suspend, must go to normal first
        return HAL_OK; // Or you could handle this transition, but for now just returning is fine
    }

    GYRO_CS_LOW();
    status = bmi088Write(BMI088_GYRO_PWR_REG, mode);
    GYRO_CS_HIGH();
    if (status != HAL_OK)
        return status;

    osDelay(30);
    return HAL_OK;
}

HAL_StatusTypeDef bmi088AccelMode(uint8_t mode)
{
    if (mode != BMI088_ACC_PWR_CONF_ACTIVE_MODE && mode != BMI088_ACC_PWR_CONF_SUSPEND_MODE)
    {
        return HAL_ERROR;
    }

    HAL_StatusTypeDef status;
    osDelay(1); // Per datasheet, wait 1ms before writing to this register

    ACCEL_CS_LOW();
    status = bmi088Write(BMI088_ACC_PWR_CTRL, mode);
    ACCEL_CS_HIGH();

    if (status != HAL_OK)
        return status;

    osDelay(1); // Wait 1ms after
    return HAL_OK;
}

HAL_StatusTypeDef bmi088SetAccelFIFOMode(uint8_t mode)
{
    if (mode != BMI088_ACCEL_FIFO_MODE && mode != BMI088_ACCEL_STREAM_MODE)
    {
        return HAL_ERROR;
    }

    HAL_StatusTypeDef status;
    uint8_t regVal;

    ACCEL_CS_LOW();
    status = bmi088AccelRead(BMI088_FIFO_MODE_REG, &regVal, 1);
    ACCEL_CS_HIGH();
    if (status != HAL_OK)
        return status;

    regVal &= ~0x03;
    regVal |= (mode & 0x03);

    ACCEL_CS_LOW();
    status = bmi088Write(BMI088_FIFO_MODE_REG, regVal);
    ACCEL_CS_HIGH();

    return status;
}

HAL_StatusTypeDef bmi088SetAccelConfODRnBandwidth(void)
{
    uint8_t val = (BMI088_ACC_BWP_NORMAL << 4) | BMI088_ACC_ODR_VAL;
    ACCEL_CS_LOW();
    HAL_StatusTypeDef status = bmi088Write(BMI088_ACC_CONF_REG, val);
    ACCEL_CS_HIGH();
    return status;
}

HAL_StatusTypeDef bmi088SetAccelRange(void)
{
    uint8_t currRegVal;
    HAL_StatusTypeDef status;

    ACCEL_CS_LOW();
    status = bmi088AccelRead(BMI088_ACC_RANGE_REG, &currRegVal, 1);
    ACCEL_CS_HIGH();
    if (status != HAL_OK)
        return status;

    currRegVal &= ~0x03;
    currRegVal |= BMI088_ACC_RANGE_VAL;

    ACCEL_CS_LOW();
    status = bmi088Write(BMI088_ACC_RANGE_REG, currRegVal);
    ACCEL_CS_HIGH();
    return status;
}

HAL_StatusTypeDef bmi088SetGyroBandwidth(void)
{
    GYRO_CS_LOW();
    HAL_StatusTypeDef status = bmi088Write(BMI088_GYRO_BANDWIDTH_REG, BMI088_GYRO_BANDWIDTH_VAL);
    GYRO_CS_HIGH();
    return status;
}

HAL_StatusTypeDef bmi088SetGyroRange(void)
{
    GYRO_CS_LOW();
    HAL_StatusTypeDef status = bmi088Write(BMI088_GYRO_RANGE_REG, BMI088_GYRO_RANGE_VAL);
    GYRO_CS_HIGH();
    return status;
}

HAL_StatusTypeDef bmi088ConfigDataReadyInt(uint8_t gyroMode)
{
    if (gyroMode != BMI088_GYRO_EN_DATA_READY_INT && gyroMode != BMI088_GYRO_EN_FIFO_INT)
    {
        return HAL_ERROR;
    }

    HAL_StatusTypeDef status;
    uint8_t regVal;

    // --- Gyro Interrupt Config ---
    GYRO_CS_LOW();
    status = bmi088GyroRead(BMI088_GYRO_INT_CTRL_REG, &regVal, 1);
    GYRO_CS_HIGH();
    if (status != HAL_OK)
        return status;

    regVal &= ~0x0C;
    regVal |= gyroMode; // Use gyroMode directly

    GYRO_CS_LOW();
    status = bmi088Write(BMI088_GYRO_INT_CTRL_REG, regVal);
    GYRO_CS_HIGH();
    if (status != HAL_OK)
        return status;

    // --- Gyro Pin IO Config ---
    GYRO_CS_LOW();
    status = bmi088GyroRead(BMI088_GYRO_INT3_INT4_IO_CONF_REG, &regVal, 1);
    GYRO_CS_HIGH();
    if (status != HAL_OK)
        return status;

    regVal &= ~0x0F;                          // clear first 4 bits
    regVal |= BMI088_GYRO_INT_PIN_CONFIG_VAL; // push-pull and active high

    GYRO_CS_LOW();
    status = bmi088Write(BMI088_GYRO_INT3_INT4_IO_CONF_REG, regVal);
    GYRO_CS_HIGH();
    if (status != HAL_OK)
        return status;

    // --- Gyro Interrupt Pin Mapping ---
    GYRO_CS_LOW();
    status = bmi088GyroRead(BMI088_GYRO_INT3_INT4_IO_MAP_REG, &regVal, 1);
    GYRO_CS_HIGH();
    if (status != HAL_OK)
        return status;

    regVal &= ~0xA5; // 1010 0101
    regVal |= 0x81;  // enable data interrupt to INT3 and INT4 pins

    GYRO_CS_LOW();
    status = bmi088Write(BMI088_GYRO_INT3_INT4_IO_MAP_REG, regVal);
    GYRO_CS_HIGH();
    if (status != HAL_OK)
        return status;

    // --- Accel Interrupt Config ---

    // Config INT 1
    ACCEL_CS_LOW();
    status = bmi088AccelRead(BMI088_ACC_INT1_IO_CONF_REG, &regVal, 1);
    ACCEL_CS_HIGH();
    if (status != HAL_OK)
        return status;

    regVal &= ~0x1F;                         // set bit 0-5 to 0
    regVal |= BMI088_ACC_INT_PIN_CONFIG_VAL; // set INT1 as active high and output interrupt

    ACCEL_CS_LOW();
    status = bmi088Write(BMI088_ACC_INT1_IO_CONF_REG, regVal);
    ACCEL_CS_HIGH();
    if (status != HAL_OK)
        return status;

    // Config INT2
    ACCEL_CS_LOW();
    status = bmi088AccelRead(BMI088_ACC_INT2_IO_CONF_REG, &regVal, 1);
    ACCEL_CS_HIGH();
    if (status != HAL_OK)
        return status;

    regVal &= ~0x1F;                         // set bit 0-5 to 0
    regVal |= BMI088_ACC_INT_PIN_CONFIG_VAL; // set INT2 as active high and output interrupt

    ACCEL_CS_LOW();
    status = bmi088Write(BMI088_ACC_INT2_IO_CONF_REG, regVal);
    ACCEL_CS_HIGH();
    if (status != HAL_OK)
        return status;

    // map data ready interrupt to pins INT1/INT2
    ACCEL_CS_LOW();
    status = bmi088Write(BMI088_ACC_INT1_INT2_MAP_DATA, BMI088_ACC_INT_MAP_PIN_VAL);
    ACCEL_CS_HIGH();

    return status; // Return the status of the final operation
}

// --- Main Init Function (Now checks all errors) ---

HAL_StatusTypeDef bmi088Init(void)
{
    HAL_StatusTypeDef status;

    // 1. Reset both sensors
    status = bmi088SoftReset(GYRO_SOFTRESET, ACCEL_SOFTRESET);
    if (status != HAL_OK)
        return status;

    // 2. Change power modes to "normal"
    status = bmi088AccelMode(BMI088_ACC_PWR_CONF_ACTIVE_MODE);
    if (status != HAL_OK)
        return status;

    // Add required 50ms delay after turning on Accel
    osDelay(50);

    status = bmi088SwitchGyroMode(BMI088_GYRO_NORMAL_MODE);
    if (status != HAL_OK)
        return status;

    // 3. Select STREAM mode on Accelerometer
    status = bmi088SetAccelFIFOMode(BMI088_ACCEL_STREAM_MODE);
    if (status != HAL_OK)
        return status;

    // 4. Configure ODR and Bandwidth
    status = bmi088SetAccelConfODRnBandwidth();
    if (status != HAL_OK)
        return status;

    // 5. Set Acceleration range
    status = bmi088SetAccelRange();
    if (status != HAL_OK)
        return status;

    // 6. Set Gyroscope Bandwidth
    status = bmi088SetGyroBandwidth();
    if (status != HAL_OK)
        return status;

    // 7. Set Gyroscope Range
    status = bmi088SetGyroRange();
    if (status != HAL_OK)
        return status;

    // 8. Set up the data interrupts
    status = bmi088ConfigDataReadyInt(BMI088_GYRO_EN_DATA_READY_INT);
    if (status != HAL_OK)
        return status;

    return HAL_OK; // All steps succeeded
}

void bmi088ReadAccelerometer(float out[4])
{
    HAL_StatusTypeDef status;
    uint8_t accelRawData[6];
    memset(accelRawData, 0, 6);

    ACCEL_CS_LOW();
    status = bmi088AccelRead(BMI088_ACC_DATA_REG, accelRawData, 6);
    ACCEL_CS_HIGH();

    if (status != HAL_OK)
    {
        for (int i = 0; i < 4; i++)
            out[i] = -99.0f;
        return;
    }

    int16_t Accel_X_int16 = (int16_t)(accelRawData[0]) | (int16_t)(accelRawData[1] << 8);
    int16_t Accel_Y_int16 = (int16_t)(accelRawData[2]) | (int16_t)(accelRawData[3] << 8);
    int16_t Accel_Z_int16 = (int16_t)(accelRawData[4]) | (int16_t)(accelRawData[5] << 8);

    uint8_t accelRangeHex;
    ACCEL_CS_LOW();
    status = bmi088AccelRead(BMI088_ACC_RANGE_REG, &accelRangeHex, 1);
    ACCEL_CS_HIGH();

    if (status != HAL_OK)
    {
        for (int i = 0; i < 4; i++)
            out[i] = -99.0f;
        return;
    }

    float rangeVal = powf(2, (accelRangeHex & 0x03) + 1.0f) * 1.5;
    float Accel_X_in_mg = ((float)Accel_X_int16 / 32768.0f) * 1000.0f * rangeVal;
    float Accel_Y_in_mg = ((float)Accel_Y_int16 / 32768.0f) * 1000.0f * rangeVal;
    float Accel_Z_in_mg = ((float)Accel_Z_int16 / 32768.0f) * 1000.0f * rangeVal;

    out[0] = 0;
    out[1] = Accel_X_in_mg;
    out[2] = Accel_Y_in_mg;
    out[3] = Accel_Z_in_mg;
}

float bmi088GyroHexToResVal(uint8_t gyroRange)
{
    switch (gyroRange)
    {
    case 0x00:
        return 16.384;
    case 0x01:
        return 32.768;
    case 0x02:
        return 65.536;
    case 0x03:
        return 131.072;
    case 0x04:
        return 262.144;
    }
    return -99.0f;
}

void bmi088ReadGyroscope(float out[4])
{
    HAL_StatusTypeDef status;
    uint8_t gyroRawData[6];
    memset(gyroRawData, 0, 6);

    GYRO_CS_LOW();
    status = bmi088GyroRead(BMI088_GYRO_DATA_REG, gyroRawData, 6);
    GYRO_CS_HIGH();

    if (status != HAL_OK)
    {
        for (int i = 0; i < 4; i++)
            out[i] = -99.0f;
        return;
    }

    int16_t rateX = (int16_t)gyroRawData[0] | (int16_t)(gyroRawData[1] << 8);
    int16_t rateY = (int16_t)gyroRawData[2] | (int16_t)(gyroRawData[3] << 8);
    int16_t rateZ = (int16_t)gyroRawData[4] | (int16_t)(gyroRawData[5] << 8);

    uint8_t gyroRangeHex;
    GYRO_CS_LOW();
    status = bmi088GyroRead(BMI088_GYRO_RANGE_REG, &gyroRangeHex, 1);
    GYRO_CS_HIGH();

    if (status != HAL_OK)
    {
        for (int i = 0; i < 4; i++)
            out[i] = -99.0f;
        return;
    }

    float gyroRes = bmi088GyroHexToResVal(gyroRangeHex);
    if (gyroRes <= 0)
    {
        for (int i = 0; i < 4; i++)
            out[i] = -99.0f;
        return;
    }

    out[0] = 0;
    out[1] = rateX / gyroRes;
    out[2] = rateY / gyroRes;
    out[3] = rateZ / gyroRes;
}
