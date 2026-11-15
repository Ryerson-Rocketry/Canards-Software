#include "Tasks/sensor.h"
#include "FreeRTOS.h"
#include "spi.h"
#include "Drivers/ms5611.h"
#include "Drivers/lsm6dso32_app.h"
#include "Drivers/mmc5983ma.h"
#include "StateEstimation/altitude_estimation.h"
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

// SPI2 shared with GPS: each user sets its own speed before every access. 18 MHz (chip max 20 MHz).
#define BARO_SPI_PRESCALER SPI_BAUDRATEPRESCALER_2
#define BARO_INIT_ATTEMPTS 3

static MS5611_t baro = {
    .hspi = &hspi2,
    .csPort = SPI2__CS_GPIO_Port,
    .csPin = SPI2__CS_Pin,
    .osr = MS5611_OSR_1024,
};
static bool baroReady = false; // PROM read and CRC passed

// Runs one barometer command on the shared SPI2 bus. Returns false if the bus
// mutex wasn't free in time (skip this cycle — not a sensor fault).
static bool baroBusCall(MS5611_Status_t (*cmd)(const MS5611_t *), MS5611_Status_t *status)
{
    if (xSemaphoreTake(gSpi2Mutex, pdMS_TO_TICKS(10)) != pdTRUE)
        return false;
    SPI2_Switch_Settings(BARO_SPI_PRESCALER, SPI_POLARITY_LOW, SPI_PHASE_1EDGE);
    *status = cmd(&baro);
    xSemaphoreGive(gSpi2Mutex);
    return true;
}

// Start a conversion, sleep while it runs (bus released), then read the 24-bit result.
static bool baroConvert(MS5611_Status_t (*start)(const MS5611_t *), uint32_t *raw, MS5611_Status_t *status)
{
    if (!baroBusCall(start, status))
        return false;
    if (*status != MS5611_OK)
        return true;

    osDelay(MS5611_ConvWaitMs(&baro));

    if (xSemaphoreTake(gSpi2Mutex, pdMS_TO_TICKS(10)) != pdTRUE)
        return false;
    SPI2_Switch_Settings(BARO_SPI_PRESCALER, SPI_POLARITY_LOW, SPI_PHASE_1EDGE);
    *status = MS5611_ReadADC(&baro, raw);
    xSemaphoreGive(gSpi2Mutex);
    return true;
}

void sensor_HardwareInit()
{
    magInit();
    LSM6DSO32_Rocket_Init(&hspi1);

    for (int attempt = 0; attempt < BARO_INIT_ATTEMPTS && !baroReady; attempt++)
    {
        MS5611_Status_t status;
        if (!baroBusCall(MS5611_Reset, &status) || status != MS5611_OK)
            continue;
        osDelay(MS5611_RESET_WAIT_MS); // PROM reload, bus released

        if (xSemaphoreTake(gSpi2Mutex, portMAX_DELAY) == pdTRUE)
        {
            SPI2_Switch_Settings(BARO_SPI_PRESCALER, SPI_POLARITY_LOW, SPI_PHASE_1EDGE);
            baroReady = (MS5611_ReadPROM(&baro) == MS5611_OK);
            xSemaphoreGive(gSpi2Mutex);
        }
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

// SPI2: Barometer. Pressure first, then temperature.
// Bad readings never reach Rocket.rawData; they only clear baroHealthy.
void sensor_ReadBarometer(void)
{
    uint32_t d1, d2;
    int32_t pressurePa, tempCentiC;
    MS5611_Status_t status;

    if (!baroReady)
        return;

    if (!baroConvert(MS5611_StartPressure, &d1, &status))
        return;
    if (status == MS5611_OK && !baroConvert(MS5611_StartTemperature, &d2, &status))
        return;
    if (status == MS5611_OK)
        status = MS5611_Compensate(&baro, d1, d2, &pressurePa, &tempCentiC);

    if (status == MS5611_OK)
    {
        Rocket.rawData.pressure = (float)pressurePa;
        // Barometer die temperature, not air temperature. Heat from the MCU, regulator and
        // other parts spreads through the PCB, so after warm-up this reads several degC
        // above ambient (~+6 degC on the bench). Only logged; not used by flight code.
        Rocket.rawData.temperature = tempCentiC / 100.0f;
        Rocket.rawData.baroHealthy = true;
    }
    else
    {
        Rocket.rawData.baroHealthy = false;
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