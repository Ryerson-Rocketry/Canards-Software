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

typedef enum
{
    START_P,
    READ_P_START_T,
    READ_T_COMPUTE
} BaroStep_t;

typedef struct NMEA_DATA
{
    double latitude;      // latitude in decimal degrees
    char latSide;         // N or S
    double longitude;     // longitude in decimal degrees
    char lonSide;         // E or W
    float altitude;       // altitude in meters
    float speed;          // Ground speed in m/s (Added for GPRMC)
    float hdop;           // horizontal dilution of precision
    int satelliteCount;   // number of satellites
    int fix;              // 1 = fix, 0 = no fix
    char lastMeasure[11]; // hhmmss.ss + null terminator
} GPS;

// 4-byte aligned for DMA
typedef struct __attribute__((packed))
{
    float accel[3];
    float gyro[3];
    float mag[3];
    float position, velocity, pressure, tiltAngle;
    float rpy[3];
    GPS gps;
    uint32_t timestamp;
} SDCardDataFormat_t;

typedef struct
{
    FlightState_t flightState;
    RawSensorData_t rawData;
    RocketState_t estimate;
    SDCardDataFormat_t snapshot;
    GPS gpsState;

} Rocket_States_t;