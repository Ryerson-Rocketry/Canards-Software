#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"
#include "madgwick.h"

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

void StartDefaultTask(void *argument);
void calculateOrientation(void *argument);

// FreeRTOS initialization
void MX_FREERTOS_Init(void)
{
    defaultTaskHandle = osThreadNew(StartDefaultTask, NULL, &defaultTask_attributes);
    retrieveOrientationTaskHandle = osThreadNew(calculateOrientation, NULL, &retrieveOrientation_attributes);
}

// Default task function
void StartDefaultTask(void *argument)
{
    for (;;)
    {
        osDelay(1);
    }
}

// Retrieve Orientation
void calculateOrientation(void *argument)
{

    Madgwick madgwick = {
        .mag_sensor = {0, 0, 0, 0},
        .accel_sensor = {0, 0, 0, 0},
        .gyro_sensor = {0, 0, 0, 0},
        .compensatedGyroMeasurements = {1, 0, 0, 0},
        .gyroBias = {1, 0, 0, 0},
        .dt = 0.01f,
        .estOrientation = {1, 0, 0, 0},
        .direcErrorQuat = {1, 0, 0, 0},
        .quatEstDeriv = {1, 0, 0, 0},
        .gravity = {0, 0, 0, 1},
        .beta = 0.001,                 // needs tuning
        .integralGain = 1,             // needs tuning
        .augmentationOfStepSize = 1.5, // needs tuning
        .euler = {0, 0, 0},
    };

    for (;;)
    {

        // retrieve sensor data here

        // update madgwick here
        updateMadgwick(&madgwick);

        osDelay(1);
    }
}
