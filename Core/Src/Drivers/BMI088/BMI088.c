#include <stdbool.h>
#include "main.h"
#include "spi.h"
#include "BMI088.h"
#include "string.h"
#include "stdint.h"

#define ACCEL_CS_PORT ACCEL_CS_GPIO_PORT
#define ACCEL_CS_PIN ACCEL_CS_GPIO_PIN
#define GYRO_CS_PORT GYRO_CS_GPIO_PORT
#define GYRO_CS_PIN GYRO_CS_GPIO_PIN
#define GYRO_SOFTRESET true
#define ACCEL_SOFTRESET true

// to choose sensor, when you want to R/W then CS_LOW -> SPI -> CS_HIGH
static inline void ACCEL_CS_LOW()
{
    HAL_GPIO_WritePin(ACCEL_CS_PORT, ACCEL_CS_PIN, GPIO_PIN_RESET);
}
static inline ACCEL_CS_HIGH()
{
    HAL_GPIO_WritePin(ACCEL_CS_PORT, ACCEL_CS_PIN, GPIO_PIN_SET);
}
static inline GYRO_CS_LOW()
{
    HAL_GPIO_WritePin(GYRO_CS_PORT, GYRO_CS_PIN, GPIO_PIN_RESET);
}
static inline GYRO_CS_HIGH()
{
    HAL_GPIO_WritePin(GYRO_CS_PORT, GYRO_CS_PIN, GPIO_PIN_SET);
}

// write to bmi088
HAL_StatusTypeDef bmi088Write(uint8_t reg, uint8_t val)
{
    if (reg == NULL || val == NULL)
    {
        return;
    }

    // set up 16 bit protocol
    uint8_t tx[2] = {reg << 1 | 0x00, val};
    return HAL_SPI_Transmit(&hspi2, tx, sizeof(tx), HAL_MAX_DELAY);
}

// read to bmi088
HAL_StatusTypeDef bmi088Read(uint8_t reg, uint8_t *dataBuff, int length)
{
    if (reg == NULL || length < 0)
    {
        return;
    }

    HAL_StatusTypeDef status;

    // Reading requires a dummy byte then actual data will be sent
    uint8_t tx[length + 1];
    uint8_t rx[length + 1];

    // set first bit to read bit
    tx[0] = reg << 1 | 0x01;

    // fill tx[1:] with 0s
    memset(&tx[1], 0x00, length);
    status = HAL_SPI_TransmitReceive(&hspi2, tx, rx, length + 1, HAL_MAX_DELAY);

    if (status)
    {
        memcpy(dataBuff, &rx[1], length);
    }
    return status;
}

void bmi088SoftReset(bool gyro, bool accel)
{
    if (!gyro && !accel)
    {
        return;
    }

    HAL_StatusTypeDef ret;

    if (gyro)
    {
        GYRO_CS_LOW();
        ret = bmi088Write(BMI088_ACC_SOFTRESET_REG, BMI088_SOFTRESET_VAL);
        if (ret != HAL_OK)
        {
            print("Gyro Soft Reset Failed");
        }
        osDelay(30);
        GYRO_CS_HIGH();
    }

    if (accel)
    {
        ACCEL_CS_LOW();
        ret = bmi088Write(BMI088_ACC_SOFTRESET_REG, BMI088_SOFTRESET_VAL);
        if (ret != HAL_OK)
        {
            print("Gyro Soft Reset Failed");
        }
        ACCEL_CS_HIGH();
        osDelay(10);
    }
}
/**
 * Switch the power modes of the Gyroscope
 * param int mode:
 *      0 = normal mode
 *      1 = suspend mode
 *      2 = deep suspend mode
 */
void bmi088SwitchGyroMode(int mode)
{
    if (mode < 0 || mode > 2)
    {
        return;
    }

    uint8_t currentMode;
    uint8_t modeVal;
    HAL_StatusTypeDef status;

    // read current gyro mode
    GYRO_CS_LOW();
    status = bmi088Read(BMI088_GYRO_PWR_REG, &currentMode, 1);

    if (status != HAL_OK)
    {
        GYRO_CS_HIGH();
        return;
    }

    modeVal = (mode == 0)   ? BMI088_GYRO_NORMAL_MODE
              : (mode == 1) ? BMI088_GYRO_SUSPEND_MODE
                            : BMI088_GYRO_DEEP_SUSPEND_MODE;

    // if current mode != normal and we are not switching to normal
    if (currentMode != BMI088_GYRO_NORMAL_MODE && modeVal != 0)
    {
        return;
    }

    status = bmi088Write(BMI088_GYRO_PWR_REG, modeVal);

    if (status != HAL_OK)
    {
        GYRO_CS_HIGH();
        return;
    }

    GYRO_CS_HIGH();
    osDelay(30);
}
/**
 * Switch the power modes of the Accelerometer
 * param int mode:
 *      0 = normal mode
 *      1 = suspend mode
 */
void bmi088AccelMode(int mode)
{
    if (mode != 0 || mode != 1)
    {
        return;
    }

    uint8_t val;
    HAL_StatusTypeDef status;
    val = (mode == 0) ? BMI088_ACC_PWR_CONF_ACTIVE_MODE : BMI088_ACC_PWR_CONF_SUSPEND_MODE;

    osDelay(1);

    ACCEL_CS_LOW();
    bmi088Write(BMI088_ACC_PWR_CTRL, val);
    if (status != HAL_OK)
    {
        print("Unable to switch Accelerometer's mode");
    }
    ACCEL_CS_HIGH();
    osDelay(1);
}

void bmi088AccelSetPower(bool on)
{
    if (on == NULL)
    {
        return;
    }

    uint8_t powerMode;
    HAL_StatusTypeDef status;
    powerMode = (on) ? BMI088_ACC_ON : BMI088_ACC_OFF;

    ACCEL_CS_LOW();

    status = bmi088Write(BMI088_ACC_PWR_CTRL, powerMode);
    if (status != HAL_OK)
    {
        print("Unable to power ON/OFF accelerometer");
    }

    ACCEL_CS_HIGH();
}



void bmi088Init()
{
    // reset both sensors
    bmi088SoftReset(GYRO_SOFTRESET, ACCEL_SOFTRESET);

    // turn on accelerometer
    bmi088AccelSetPower(true);

    // change power modes to "normal"
    bmi088AccelMode(0);
    bmi088SwitchGyroMode(0);

    // select STREAM mode on Accelerometer
}