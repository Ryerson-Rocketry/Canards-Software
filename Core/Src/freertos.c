#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os2.h"
#include "madgwick.h"
#include "queue.h"
#include <string.h>
#include "ms5611.h"
#include "altitudeEstimator.h"

typedef struct
{
    float accel[4];
    float orientation[4]; // quaternion (w,x,y,z)
} VelAltMsg;

// Task handle and attributes
osThreadId_t defaultTaskHandle;
osThreadId_t retrieveOrientationTaskHandle;
osThreadId_t retrieveVelAndAltitudeTaskHandle;
QueueHandle_t calcVelAndHeightQueue;

uint16_t prom[8];

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

const osThreadAttr_t retrieveVelAndAltitude_attributes = {
    .name = "getVelAndHeight",
    .stack_size = 128 * 4,
    .priority = (osPriority_t)osPriorityAboveNormal7,
};

void StartDefaultTask(void *argument);
void calculateOrientation(void *argument);
void calcVelAndHeight(void *argument);

// FreeRTOS initialization
void MX_FREERTOS_Init(void)
{
    xQueueHandle calcVelAndHeightQueue = xQueueCreate(8, sizeof(VelAltMsg));
    configASSERT(calcVelAndHeightQueue != NULL);

    defaultTaskHandle = osThreadNew(StartDefaultTask, NULL, &defaultTask_attributes);
    retrieveOrientationTaskHandle = osThreadNew(calculateOrientation, NULL, &retrieveOrientation_attributes);
    retrieveVelAndAltitudeTaskHandle = osThreadNew(calcVelAndHeight, NULL, &retrieveVelAndAltitude_attributes);
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
    float yaw = 0;
    float pitch = 0;
    float roll = 0;
    VelAltMsg msg;

    Madgwick madgwick = {
        .mag_sensor = {0, 0, 0, 0},
        .accel_sensor = {0, 0, 0, 0},
        .gyro_sensor = {0, 0, 0, 0},
        .compensatedGyroMeasurements = {1, 0, 0, 0},
        .gyroBias = {1, 0, 0, 0},
        .dt = 0.01f, // 0.01f is a placeholder for now,
                     // it can be faster and if we want to be fancy we can dynamically find this val
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

        yaw = madgwick.euler[0];
        pitch = madgwick.euler[1];
        roll = madgwick.euler[2];

        // send yaw pitch roll to actuator

        // send madgwick quaternian and accel data
        memcpy(msg.accel, madgwick.accel_sensor, sizeof(msg.accel));
        memcpy(msg.orientation, madgwick.estOrientation, sizeof(msg.orientation));
        xQueueSend(calcVelAndHeightQueue, &msg, portMAX_DELAY);

        osDelay(1);
    }
}

void calcVelAndHeight(void *argument)
{
    ms5611Reset();
    ms5611ReadPROM(prom);

    VelAltMsg rx;
    float K[2] = {0.10f, 0.05f}; // Tune as needed
    float x[2] = {0.0f, 0.0f};   // [height, velocity]
    int32_t baroPressure, temperature;
    float initialPressure;
    float baroAltitude, filteredBaroAlt;
    float vertLinAccel;
    float alphaBaro = 0.2f;

    // Get the first message to set the filter baseline
    xQueueReceive(calcVelAndHeightQueue, &rx, portMAX_DELAY);

    // Read barometer for initial pressure
    ms5611GetPressureAndTemp(prom, &baroPressure, &temperature);
    initialPressure = (float)baroPressure;

    // Compute initial altitude and set filter to that
    baroAltitude = pressureToAltitude((float)baroPressure, initialPressure);
    filteredBaroAlt = baroAltitude;

    TickType_t prevTick = xTaskGetTickCount();

    for (;;)
    {
        if (xQueueReceive(calcVelAndHeightQueue, &rx, portMAX_DELAY) != pdPASS)
            continue;

        TickType_t currTick = xTaskGetTickCount();
        float timeSampled = (float)(currTick - prevTick) / (float)configTICK_RATE_HZ;
        if (timeSampled <= 0.0f)
        {
            timeSampled = 1.0f / (float)configTICK_RATE_HZ;
        }

        prevTick = currTick;

        // Read barometer
        ms5611GetPressureAndTemp(prom, &baroPressure, &temperature);
        baroAltitude = pressureToAltitude((float)baroPressure, initialPressure);

        // IIR filter for baro altitude
        filteredBaroAlt = iirFilter(baroAltitude, filteredBaroAlt, alphaBaro);

        vertLinAccel = calcVerticalLinearAccel(rx.accel, rx.orientation);

        // Fusion
        float posError = calcPositionError(filteredBaroAlt, x[0]);
        float velCorr = timeSampled * vertLinAccel;

        float out[2];
        complementaryFilter(timeSampled, x, K, posError, velCorr, out);

        x[0] = out[0]; // Altitude
        x[1] = out[1]; // Velocity
    }
}
