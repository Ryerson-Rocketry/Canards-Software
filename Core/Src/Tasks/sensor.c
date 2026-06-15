#include "Tasks/sensor.h"
#include "FreeRTOS.h"
#include "spi.h"
#include "Drivers/ms5611.h"
#include "Drivers/lsm6dso32_app.h"
#include "Drivers/mmc5983ma.h"
#include "stm32f4xx_hal_def.h"
#include "task.h"
#include "semphr.h"
#include "states.h"
#include "cmsis_os.h"
#include <stdbool.h>
#include "math.h"

extern SPI_HandleTypeDef hspi1;
extern SemaphoreHandle_t gSpi1Mutex;
extern SemaphoreHandle_t gSpi2Mutex;
extern SemaphoreHandle_t gI2c1Mutex;
extern SemaphoreHandle_t xMagDataReadySemaphore;
extern SemaphoreHandle_t xImuAccelReadySemaphore;
extern SemaphoreHandle_t xImuGyroReadySemaphore;
extern TaskHandle_t launchDetTaskHandle;
extern volatile bool readSensorTask;

extern Rocket_States_t Rocket;

static float accel_filt[3] = {0.0f, 0.0f, 1000.0f};
const float alpha_accel = 0.1f;

// Accel zero-offset calibration. Board is flat on the pad at boot, so average the first
// ACCEL_CAL_SAMPLES raw readings; X/Y should read 0 and Z should read +1 g, so the Z term
// subtracts the offset while keeping gravity.
#define ACCEL_CAL_SAMPLES 200
static float accelBias[3] = {0.0f, 0.0f, 0.0f};
static float accelBiasAccum[3] = {0.0f, 0.0f, 0.0f};
static uint32_t accelCalCount = 0;

static float gyro_filt[3] = {0.0f, 0.0f, 0.0f};
const float alpha_gyro = 0.2f;

// Gyro zero-rate bias calibration. The board is stationary on the pad at boot, so
// average the first GYRO_CAL_SAMPLES raw readings to capture the per-axis offset
// (e.g. Z ~ -1 dps) and subtract it from every read after that.
#define GYRO_CAL_SAMPLES 200
static float gyroBias[3] = {0.0f, 0.0f, 0.0f};
static float gyroBiasAccum[3] = {0.0f, 0.0f, 0.0f};
static uint32_t gyroCalCount = 0;

void sensor_HardwareInit()
{
    magInit();
    LSM6DSO32_Rocket_Init(&hspi1);

    if (xSemaphoreTake(gSpi2Mutex, portMAX_DELAY) == pdTRUE)
    {
        SPI2_Switch_Settings(SPI_BAUDRATEPRESCALER_8, SPI_POLARITY_LOW, SPI_PHASE_1EDGE);
        Barometer_init();
        xSemaphoreGive(gSpi2Mutex);
    }

    osDelay(1000);

    HAL_NVIC_EnableIRQ(EXTI0_IRQn);
    HAL_NVIC_EnableIRQ(EXTI2_IRQn);
    HAL_NVIC_EnableIRQ(EXTI3_IRQn);
}

// I2C1: Magnetometer
void sensor_ReadMagnetometer(void)
{
    magGetData(xMagDataReadySemaphore, Rocket.rawData.mag);
}

// SPI2: Barometer
void sensor_ReadBarometer(void)
{
    if (xSemaphoreTake(gSpi2Mutex, pdMS_TO_TICKS(10)) == pdTRUE)
    {
        SPI2_Switch_Settings(SPI_BAUDRATEPRESCALER_8, SPI_POLARITY_LOW, SPI_PHASE_1EDGE);
        int32_t p_mbar_x100 = Barometer_getPressure(true);
        int32_t t_centiC = Barometer_getTemp(false);

        /* Convert units */
        Rocket.rawData.pressure = (float)p_mbar_x100; // because mbar*100 = Pa
        Rocket.rawData.temperature = t_centiC / 100.0f;
        xSemaphoreGive(gSpi2Mutex);
    }
}

// SPI1: IMU - accelerometer
void sensor_ReadIMUAccelerometer(void)
{
    if (xSemaphoreTake(xImuAccelReadySemaphore, 0) != pdTRUE)
        return;

    if (xSemaphoreTake(gSpi1Mutex, pdMS_TO_TICKS(10)) == pdTRUE)
    {
        LSM6DSO32_Read_Accel(Rocket.rawData.accel);

        // Capture the accel zero-offset from the first samples while flat on the pad.
        if (accelCalCount < ACCEL_CAL_SAMPLES)
        {
            for (int i = 0; i < 3; i++)
                accelBiasAccum[i] += Rocket.rawData.accel[i];

            if (++accelCalCount == ACCEL_CAL_SAMPLES)
            {
                accelBias[0] = accelBiasAccum[0] / (float)ACCEL_CAL_SAMPLES;
                accelBias[1] = accelBiasAccum[1] / (float)ACCEL_CAL_SAMPLES;
                accelBias[2] = accelBiasAccum[2] / (float)ACCEL_CAL_SAMPLES - 1000.0f;
            }
        }

        for (int i = 0; i < 3; i++)
        {
            // Subtract the captured offset (0 until cal completes), then low-pass.
            float a = Rocket.rawData.accel[i] - accelBias[i];
            accel_filt[i] = (alpha_accel * a) + ((1.0f - alpha_accel) * accel_filt[i]);
            Rocket.rawData.accel[i] = accel_filt[i];
        }
        xSemaphoreGive(gSpi1Mutex);
    }
}

// SPI1: IMU - gyro
void sensor_ReadIMUGyro(void)
{
    if (xSemaphoreTake(xImuGyroReadySemaphore, 0) != pdTRUE)
        return;

    if (xSemaphoreTake(gSpi1Mutex, pdMS_TO_TICKS(10)) == pdTRUE)
    {
        LSM6DSO32_Read_Gyro(Rocket.rawData.gyro);

        // Collect the zero-rate bias from the first samples while still on the pad.
        if (gyroCalCount < GYRO_CAL_SAMPLES)
        {
            for (int i = 0; i < 3; i++)
                gyroBiasAccum[i] += Rocket.rawData.gyro[i];

            if (++gyroCalCount == GYRO_CAL_SAMPLES)
                for (int i = 0; i < 3; i++)
                    gyroBias[i] = gyroBiasAccum[i] / (float)GYRO_CAL_SAMPLES;

            // Report zero rate while calibrating so the estimator doesn't integrate the bias.
            for (int i = 0; i < 3; i++)
                Rocket.rawData.gyro[i] = 0.0f;

            xSemaphoreGive(gSpi1Mutex);
            return;
        }

        for (int i = 0; i < 3; i++)
        {
            // Subtract the captured bias, then low-pass + deadband.
            float rate = Rocket.rawData.gyro[i] - gyroBias[i];
            gyro_filt[i] = (alpha_gyro * rate) + ((1.0f - alpha_gyro) * gyro_filt[i]);
            Rocket.rawData.gyro[i] = fabsf(gyro_filt[i]) < 0.05f ? 0.0f : gyro_filt[i];
        }
        xSemaphoreGive(gSpi1Mutex);
    }
}

void sensor_GroundReference(void)
{
    static bool groundCaptured = false;
    if (!groundCaptured && Rocket.flightState == STATE_PAD && Rocket.rawData.pressure > 0.0f)
    {
        setGroundPressure(Rocket.rawData.pressure);
        groundCaptured = true;
    }
    xTaskNotifyGive(launchDetTaskHandle);
}