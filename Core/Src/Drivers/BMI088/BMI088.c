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
static inline void ACCEL_CS_HIGH()
{
    HAL_GPIO_WritePin(ACCEL_CS_PORT, ACCEL_CS_PIN, GPIO_PIN_SET);
}
static inline void GYRO_CS_LOW()
{
    HAL_GPIO_WritePin(GYRO_CS_PORT, GYRO_CS_PIN, GPIO_PIN_RESET);
}
static inline void GYRO_CS_HIGH()
{
    HAL_GPIO_WritePin(GYRO_CS_PORT, GYRO_CS_PIN, GPIO_PIN_SET);
}

// write to bmi088
HAL_StatusTypeDef bmi088Write(uint8_t reg, uint8_t val)
{
    // set up 16 bit protocol
    uint8_t tx[2] = {reg << 1 | 0x00, val};
    return HAL_SPI_Transmit(&hspi2, tx, sizeof(tx), HAL_MAX_DELAY);
}

// read to bmi088
HAL_StatusTypeDef bmi088Read(uint8_t reg, uint8_t *dataBuff, int length)
{
    if (length < 0)
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

    if (status == HAL_OK)
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
        ret = bmi088Write(BMI088_GYR_SOFTRESET_REG, BMI088_SOFTRESET_VAL);
        GYRO_CS_HIGH();
        osDelay(30);

        if (ret != HAL_OK)
        {
            print("Gyro Soft Reset Failed");
        }
    }

    if (accel)
    {
        ACCEL_CS_LOW();
        ret = bmi088Write(BMI088_ACC_SOFTRESET_REG, BMI088_SOFTRESET_VAL);
        ACCEL_CS_HIGH();

        if (ret != HAL_OK)
        {
            print("Accel Soft Reset Failed");
        }

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
void bmi088SwitchGyroMode(uint8_t mode)
{
    if (mode != BMI088_GYRO_NORMAL_MODE &&
        mode != BMI088_GYRO_SUSPEND_MODE &&
        mode != BMI088_GYRO_DEEP_SUSPEND_MODE)
    {
        return;
    }

    uint8_t currentMode;
    HAL_StatusTypeDef status;

    // read current gyro mode
    GYRO_CS_LOW();
    status = bmi088Read(BMI088_GYRO_PWR_REG, &currentMode, 1);
    GYRO_CS_HIGH();

    // if current mode != normal and we are not switching to normal
    if (currentMode != BMI088_GYRO_NORMAL_MODE && mode != BMI088_GYRO_NORMAL_MODE)
    {
        return;
    }

    GYRO_CS_LOW();
    status = bmi088Write(BMI088_GYRO_PWR_REG, mode);
    GYRO_CS_HIGH();

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
void bmi088AccelMode(uint8_t mode)
{
    if (mode != BMI088_ACC_PWR_CONF_ACTIVE_MODE && mode != BMI088_ACC_PWR_CONF_SUSPEND_MODE)
    {
        return;
    }

    uint8_t val;
    HAL_StatusTypeDef status;
    val = (mode == 0) ? BMI088_ACC_PWR_CONF_ACTIVE_MODE : BMI088_ACC_PWR_CONF_SUSPEND_MODE;

    osDelay(1);

    ACCEL_CS_LOW();
    status = bmi088Write(BMI088_ACC_PWR_CTRL, val);
    ACCEL_CS_HIGH();

    if (status != HAL_OK)
    {
        print("Unable to switch Accelerometer's mode");
    }
    osDelay(1);
}

void bmi088AccelSetPower(bool on)
{
    uint8_t powerMode;
    HAL_StatusTypeDef status;
    powerMode = (on) ? BMI088_ACC_ON : BMI088_ACC_OFF;

    ACCEL_CS_LOW();
    status = bmi088Write(BMI088_ACC_PWR_CTRL, powerMode);
    ACCEL_CS_HIGH();

    if (status != HAL_OK)
    {
        print("Unable to power ON/OFF accelerometer");
    }
}

void bmi088SetAccelFIFOMode(uint8_t mode)
{
    if (mode != BMI088_ACCEL_FIFO_MODE && mode != BMI088_ACCEL_STREAM_MODE)
    {
        return;
    }

    uint8_t regVal;
    ACCEL_CS_LOW();
    bmi088Read(BMI088_FIFO_MODE_REG, &regVal, 1);
    ACCEL_CS_HIGH();

    regVal &= ~0x03;
    regVal |= (mode & 0x03);

    ACCEL_CS_LOW();
    bmi088Write(BMI088_FIFO_MODE_REG, regVal);
    ACCEL_CS_HIGH();
}

void bmi088SetAccelConfODRnBandwidth()
{
    // need to change these values to match PID loop
    // um too lazy to make this changeable so I will be using 400hz and normal 😀
    uint8_t val = (BMI088_ACC_BWP_NORMAL << 4) | BMI088_ACC_ODR_VAL;

    ACCEL_CS_LOW();
    bmi088Write(BMI088_ACC_CONF_REG, val);
    ACCEL_CS_HIGH();
}

void bmi088SetAccelRange()
{
    uint8_t currRegVal;

    ACCEL_CS_LOW();
    bmi088Read(BMI088_ACC_RANGE_REG, &currRegVal, 1);
    ACCEL_CS_HIGH();

    currRegVal &= ~0x03;
    currRegVal |= BMI088_ACC_RANGE_VAL;

    ACCEL_CS_LOW();
    bmi088Write(BMI088_ACC_RANGE_REG, currRegVal);
    ACCEL_CS_HIGH();
}

void bmi088SetGyroBandwidth()
{
    GYRO_CS_LOW();
    bmi088Write(BMI088_GYRO_BANDWIDTH_REG, BMI088_GYRO_BANDWIDTH_VAL);
    GYRO_CS_HIGH();
}

void bmi088SetGyroRange()
{
    GYRO_CS_LOW();
    bmi088Write(BMI088_GYRO_RANGE_REG, BMI088_GYRO_RANGE_VAL);
    GYRO_CS_HIGH();
}

void bmi088ConfigDataReadyInt(uint8_t gyroMode)
{
    if (gyroMode != BMI088_GYRO_EN_DATA_READY_INT && gyroMode != BMI088_GYRO_EN_FIFO_INT)
    {
        return;
    }

    // enable data ready interrupts

    uint8_t gyroCtrlRegVal;
    GYRO_CS_LOW();
    bmi088Read(BMI088_GYRO_INT_CTRL_REG, &gyroCtrlRegVal, 1);
    GYRO_CS_HIGH();

    gyroCtrlRegVal &= ~0x0C;

    if (gyroMode == BMI088_GYRO_EN_DATA_READY_INT)
    {
        gyroCtrlRegVal |= BMI088_GYRO_EN_DATA_READY_INT;
    }
    else
    {
        gyroCtrlRegVal |= BMI088_GYRO_EN_FIFO_INT;
    }

    GYRO_CS_LOW();
    bmi088Write(BMI088_GYRO_INT_CTRL_REG, gyroCtrlRegVal);
    GYRO_CS_HIGH();

    // config the electrical and logical properties of int pins
    // we will be using push pull cuz I do not want to add pull ups (if possible we can do direct conecnt to gpio)
    // we will be using active high (rising edge 🙏🙏🙏)

    uint8_t gyroIntConfigRegVal;

    GYRO_CS_LOW();
    bmi088Read(BMI088_GYRO_INT3_INT4_IO_CONF_REG, &gyroIntConfigRegVal, 1);
    GYRO_CS_HIGH();

    // clear first 4 bits
    gyroIntConfigRegVal &= ~0x0F;

    // push-pull and active high
    gyroIntConfigRegVal |= BMI088_GYRO_INT_PIN_CONFIG_VAL;

    GYRO_CS_LOW();
    bmi088Write(BMI088_GYRO_INT3_INT4_IO_CONF_REG, gyroIntConfigRegVal);
    GYRO_CS_HIGH();

    // Map the data ready interrupt pin to one of the interrupt pins INT3 and/or INT4.

    uint8_t gyroIntToPinMapVal;
    GYRO_CS_LOW();
    bmi088Read(BMI088_GYRO_INT3_INT4_IO_MAP_REG, &gyroIntToPinMapVal, 1);
    GYRO_CS_HIGH();

    // 1010 0101 -> 0xA5
    gyroIntToPinMapVal &= ~0xA5;

    // enable data interrupt to INT3 and INT4 pins
    gyroIntToPinMapVal |= 0x81;

    GYRO_CS_LOW();
    bmi088Write(BMI088_GYRO_INT3_INT4_IO_MAP_REG, gyroIntToPinMapVal);
    GYRO_CS_HIGH();

    // Config Accelerometer Data Ready Interrupts

    // Config INT 1
    uint8_t accelInt1ConfigVal;
    ACCEL_CS_LOW();
    bmi088Read(BMI088_ACC_INT1_IO_CONF_REG, &accelInt1ConfigVal, 1);
    ACCEL_CS_HIGH();

    // set bit 0-5 to 0
    accelInt1ConfigVal &= ~0x1F;
    // set INT1 as active high and output interrupt
    accelInt1ConfigVal |= BMI088_ACC_INT_PIN_CONFIG_VAL;

    ACCEL_CS_LOW();
    bmi088Write(BMI088_ACC_INT1_IO_CONF_REG, accelInt1ConfigVal);
    ACCEL_CS_HIGH();

    // Config INT2
    uint8_t accelInt2ConfigVal;
    ACCEL_CS_LOW();
    bmi088Read(BMI088_ACC_INT2_IO_CONF_REG, &accelInt2ConfigVal, 1);
    ACCEL_CS_HIGH();

    // set bit 0-5 to 0
    accelInt2ConfigVal &= ~0x1F;
    // set INT1 as active high and output interrupt
    accelInt2ConfigVal |= BMI088_ACC_INT_PIN_CONFIG_VAL;

    ACCEL_CS_LOW();
    bmi088Write(BMI088_ACC_INT2_IO_CONF_REG, accelInt2ConfigVal);
    ACCEL_CS_HIGH();

    // map data ready interrupt to pins INT1/INT2
    ACCEL_CS_LOW();
    bmi088Write(BMI088_ACC_INT1_INT2_MAP_DATA, BMI088_ACC_INT_MAP_PIN_VAL);
    ACCEL_CS_HIGH();
}

void bmi088Init()
{
    // reset both sensors
    bmi088SoftReset(GYRO_SOFTRESET, ACCEL_SOFTRESET);

    // turn on accelerometer
    bmi088AccelSetPower(true);

    // change power modes to "normal"
    bmi088AccelMode(BMI088_ACC_PWR_CONF_ACTIVE_MODE);
    bmi088SwitchGyroMode(BMI088_GYRO_NORMAL_MODE);

    // select STREAM mode on Accelerometer
    bmi088SetAccelFIFOMode(BMI088_ACCEL_STREAM_MODE);

    // configure ODR and Bandwidth
    bmi088SetAccelConfODRnBandwidth();

    // set Acceleration range
    bmi088SetAccelRange();

    // set Gyroscope Bandwidth
    bmi088SetGyroBandwidth();

    // set Gyroscope Range
    bmi088SetGyroRange();

    // set up the data interrupts
    bmi088ConfigDataReadyInt(BMI088_ACCEL_STREAM_MODE);
}