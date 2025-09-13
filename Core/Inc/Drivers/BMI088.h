#pragma once
#include <stdbool.h>

// Register ADDRESSES
#define BMI088_ACC_SOFTRESET_REG 0x7E
#define BMI088_ACC_PWR_CONF_REG 0x7C
#define BMI088_ACC_PWR_CTRL 0x7D
#define BMI088_GYRO_PWR_REG 0x11
#define BMI088_GYR_SOFTRESET_REG 0x14

// reg values
#define BMI088_SOFTRESET_VAL 0xB6
#define BMI088_ACC_PWR_CONF_SUSPEND_MODE 0x03
#define BMI088_ACC_PWR_CONF_ACTIVE_MODE 0x00
#define BMI088_ACC_OFF 0x00
#define BMI088_ACC_ON 0x04
#define BMI088_GYRO_NORMAL_MODE 0x00
#define BMI088_GYRO_SUSPEND_MODE 0x80
#define BMI088_GYRO_DEEP_SUSPEND_MODE 0x20

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
void bmi088AccelMode(int mode);
void bmi088SwitchGyroMode(int mode);
