#pragma once

#include "main.h"
#include <stdbool.h>
#include <stdint.h>

// ─── Enumerations ─────────────────────────────────────────────────────────────

typedef enum
{
    STATE_PAD,
    STATE_BOOSTER_BURNOUT,
    STATE_SEPARATION,
    STATE_SUSTAINER_IGNITION,
    STATE_CANARDS_ACTIVATE,
    STATE_DESCENT
} FlightState_t;

typedef enum
{
    ROLL = 0,
    PITCH = 1,
    YAW = 2
} RPY_t;

// ─── Sensor Data ──────────────────────────────────────────────────────────────

typedef struct
{
    float accel[3];
    float gyro[3];
    float mag[3];
    float pressure;
    float temperature;
    uint32_t timestamp;
} RawSensorData_t;

// ─── State Estimate ───────────────────────────────────────────────────────────

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

// ─── GPS Fix ──────────────────────────────────────────────────────────────────

typedef struct
{
    bool   has_fix;
    double latitude;    // decimal degrees ($GNRMC)
    double longitude;   // decimal degrees ($GNRMC)
    float  altitude_m;  // metres ASL   ($GNGGA)
    float  speed_kmh;   // ground speed  ($GNRMC)
} GpsFix_t;

// ─── Control ──────────────────────────────────────────────────────────────────

typedef struct
{
    float rollError;
    float pitchError;
    float pwmAngle;
    float setPoint;
} ControlData_t;

// ─── SD Card Snapshot (packed for DMA) ───────────────────────────────────────

typedef struct __attribute__((packed))
{
    float accel[3];
    float gyro[3];
    float mag[3];
    float position;
    float velocity;
    float pressure;
    float tiltAngle;
    float rpy[3];
    float rollError;
    float pitchError;
    float pwmAngle;
    FlightState_t flightState;
    uint32_t timestamp;
} SDCardDataFormat_t;

// ─── Top-Level State Container ────────────────────────────────────────────────

typedef struct
{
    FlightState_t flightState;
    RawSensorData_t rawData;
    RocketState_t estimate;
    ControlData_t control;
    SDCardDataFormat_t snapshot;
    GpsFix_t gps;
} Rocket_States_t;