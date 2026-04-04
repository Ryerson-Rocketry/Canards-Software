#pragma once
#include "main.h"
#include "stdbool.h"
#include <stdint.h>

typedef enum
{
    STATE_PAD,
    STATE_BOOST,
    STATE_BURNOUT,
    STATE_CANARDS_ACTIVATE,
    STATE_DESCENT,
} FlightState_t;

typedef enum
{
    ROLL = 0,
    PITCH = 1,
    YAW = 2
} RPY_t;

typedef struct
{
    float accel[3];
    float gyro[3];
    float mag[3];
    float pressure;
    float temperature;
    uint32_t timestamp;
} RawSensorData_t;

typedef struct
{
    float position;
    float velocity;
    float acceleration;
    float rpy[3];
    float tilt_angle;
    uint32_t launch_tick;
    uint32_t timestamp;
} RocketState_t;

// 4-byte aligned for DMA
typedef struct __attribute__((packed))
{
    float accel[3];
    float gyro[3];
    float mag[3];
    float position, velocity, pressure, tiltAngle;
    float rpy[3];

    float rollError;
    float pwmAngle;
    float pitchError;
    FlightState_t flightState;
    uint32_t timestamp;
} SDCardDataFormat_t;

typedef struct
{
    float rollError;
    float pitchError;
    float pwmAngle;
    float setPoint;
} ControlData_t;

typedef struct
{
    FlightState_t flightState;
    RawSensorData_t rawData;
    RocketState_t estimate;
    ControlData_t control;
    SDCardDataFormat_t snapshot;
} Rocket_States_t;