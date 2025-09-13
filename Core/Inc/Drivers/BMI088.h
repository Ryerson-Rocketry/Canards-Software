#pragma once
#include <stdbool.h>

// Register ADDRESSES
#define BMI088_ACC_SOFTRESET_REG 0x7E
#define BMI088_ACC_PWR_CONF_REG 0x7C
#define BMI088_ACC_PWR_CTRL 0x7D
#define BMI088_GYRO_PWR_REG 0x11
#define BMI088_GYR_SOFTRESET_REG 0x14
#define BMI088_FIFO_MODE_REG 0x48
#define BMI088_ACC_CONF_REG 0x40
#define BMI088_ACC_RANGE_REG 0x41
#define BMI088_GYRO_BANDWIDTH_REG 0x10
#define BMI088_GYRO_RANGE_REG 0x0F
#define BMI088_GYRO_INT_CTRL_REG 0x15
#define BMI088_GYRO_INT3_INT4_IO_CONF_REG 0x16
#define BMI088_GYRO_INT3_INT4_IO_MAP_REG 0x18

// reg values
#define BMI088_SOFTRESET_VAL 0xB6
#define BMI088_ACC_PWR_CONF_SUSPEND_MODE 0x03
#define BMI088_ACC_PWR_CONF_ACTIVE_MODE 0x00
#define BMI088_ACC_OFF 0x00
#define BMI088_ACC_ON 0x04
#define BMI088_GYRO_NORMAL_MODE 0x00
#define BMI088_GYRO_SUSPEND_MODE 0x80
#define BMI088_GYRO_DEEP_SUSPEND_MODE 0x20
#define BMI088_ACCEL_FIFO_MODE 0x01
#define BMI088_ACCEL_STREAM_MODE 0x00
#define BMI088_ACC_ODR_VAL 0x0A // 400hz
#define BMI088_ACC_BWP_NORMAL 0x0A
#define BMI088_ACC_RANGE_VAL 0x03      // 24g
#define BMI088_GYRO_BANDWIDTH_VAL 0x03 // 400 hz
#define BMI088_GYRO_RANGE_VAL 0x02     // +- 500 deg/s
#define BMI088_GYRO_EN_DATA_READY_INT 0x08
#define BMI088_GYRO_EN_FIFO_INT 0x04
#define BMI088_GYRO_INT_PIN_CONFIG_VAL 0x05

// CS Pins (need to be changed)
#define ACCEL_CS_GPIO_PORT GPIOA
#define ACCEL_CS_GPIO_PIN GPIO_PIN_5
#define GYRO_CS_GPIO_PORT GPIOA
#define GYRO_CS_GPIO_PIN GPIO_PIN_0

#define GYRO_SOFTRESET true
#define ACCEL_SOFTRESET true

void bmi088SoftReset(bool gyro, bool accel);
HAL_StatusTypeDef bmi088Write(uint8_t reg, uint8_t val);
HAL_StatusTypeDef bmi088Read(uint8_t reg, uint8_t *dataBuff, int length);
void bmi088AccelSetPower(bool on);
void bmi088Init();
void bmi088AccelMode(uint8_t mode);
void bmi088SwitchGyroMode(uint8_t mode);
void bmi088SetAccelConfODRnBandwidth();
void bmi088SetAccelFIFOMode(uint8_t mode);
void bmi088SetAccelRange();
void bmi088SetGyroRange();
void bmi088SetGyroBandwidth();
