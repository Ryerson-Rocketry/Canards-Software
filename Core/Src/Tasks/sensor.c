#include "Tasks/sensor.h"
#include "FreeRTOS.h"
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
static float gyro_filt[3] = {0.0f, 0.0f, 0.0f};
const float alpha_gyro = 0.2f;

void sensor_HardwareInit()
{
    magInit();
    LSM6DSO32_Rocket_Init(&hspi1);
    Barometer_init();

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
        for (int i = 0; i < 3; i++)
        {
            accel_filt[i] = (alpha_accel * Rocket.rawData.accel[i]) + ((1.0f - alpha_accel) * accel_filt[i]);
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
        for (int i = 0; i < 3; i++)
        {
            // Formula: Smooth Value = (New * Alpha) + (Old * (1 - Alpha))
            gyro_filt[i] = (alpha_gyro * Rocket.rawData.gyro[i]) + ((1.0f - alpha_gyro) * gyro_filt[i]);
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