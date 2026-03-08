#include "main.h"

typedef enum
{
    STATE_PAD,
    STATE_BOOST,
    STATE_BURNOUT,
    STATE_CANARDS_ACTIVATE,
} FlightState_t;

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
    float pitch;
    float yaw;
    float roll;
    float tilt_angle;
    uint32_t launch_tick;
    uint32_t timestamp;
} RocketState_t;

// 4-byte aligned for DMA
typedef struct __attribute__((aligned(4)))
{
    float accel[3];
    float gyro[3];
    float mag[3];
    float position;
    float velocity;
    float pressure;
    float tiltAngle;
    float pitch;
    float yaw;
    float roll;
    uint32_t timestamp;
} SDCardDataFormat_t;