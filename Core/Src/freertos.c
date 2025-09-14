#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"
#include "semphr.h"
#include "BMI088.h"
#include "usart.h"
#include "string.h"
#include "stdio.h"

#define GPIO_PIN_ACC_INT GPIO_PIN_ACC_INT_VAL
#define GPIO_PIN_GYRO_INT GPIO_PIN_GYRO_INT_VAL

// Semaphores
SemaphoreHandle_t xImuDataReadySemaphore;

// Task handle and attributes
osThreadId_t defaultTaskHandle;
osThreadId_t retrieveOrientationTaskHandle;

const osThreadAttr_t defaultTask_attributes = {
    .name = "defaultTask",
    .stack_size = 128 * 4,
    .priority = (osPriority_t)osPriorityNormal,
};

const osThreadAttr_t retrieveOrientation_attributes = {
    .name = "getOrientation",
    .stack_size = 128 * 4,
    .priority = (osPriority_t)osPriorityRealtime,
};

void StartDefaultTask();
void calculateOrientation();

// FreeRTOS initialization
void MX_FREERTOS_Init(void)
{

    bmi088Init();
    osDelay(1);

    xImuDataReadySemaphore = xSemaphoreCreateBinary();
    configASSERT(xImuDataReadySemaphore);

    defaultTaskHandle = osThreadNew(StartDefaultTask, NULL, &defaultTask_attributes);
}

// Default task function
void StartDefaultTask()
{
    for (;;)
    {
        osDelay(1);
    }
}

void calculateOrientation()
{

    // needs to be changed to input it into Madgwick filter

    float accelData[4];
    float gyroData[4];

    char uartBuf[100];

    for (;;)
    {

        // retrieve sensor data here
        if (xSemaphoreTake(xImuDataReadySemaphore, portMAX_DELAY) != pdTRUE)
        {
            continue;
        }

        bmi088ReadAccelerometer(accelData);
        bmi088ReadGyroscope(gyroData);

        snprintf(uartBuf, sizeof(uartBuf),
                 "Accel: %.2f, %.2f, %.2f\r\nGyro: %.2f, %.2f, %.2f\r\n",
                 accelData[1], accelData[2], accelData[3], gyroData[1], gyroData[2], gyroData[3]);
        HAL_UART_Transmit(&huart2, (uint8_t *)uartBuf, strlen(uartBuf), HAL_MAX_DELAY);

        osDelay(1);
    }
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
    // flag to see if there is a higher priority task
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    if (GPIO_Pin == GPIO_PIN_ACC_INT || GPIO_Pin == GPIO_PIN_GYRO_INT)
    {
        // if there is a higher priority task, xHigherPriorityTaskWoken becomes true
        xSemaphoreGiveFromISR(xImuDataReadySemaphore, &xHigherPriorityTaskWoken);
        // if higher priority task, switch to it after interrupt is done
        portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
    }
}
