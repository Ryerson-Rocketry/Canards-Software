#pragma once

// ─── Magnetometer Calibration ─────────────────────────────────────────────────
#define MAG_X_OFFSET 0.077f
#define MAG_Y_OFFSET 0.147f

// ─── Atmospheric Constants ────────────────────────────────────────────────────
#define SEA_LEVEL_PA 101325.0f // Standard atmospheric pressure (Pa)
#define PI 3.14159265358979323846f

// ─── GPS ──────────────────────────────────────────────────────────────────────
#define GPS_BUF_SIZE 128

// ─── Logging ──────────────────────────────────────────────────────────────────
#define FLIGHT_DATA_FILE_NAME "LOG.CSV"
// One CSV row is ~294 B worst case (all 28 fields incl. a GPS fix); 512 gives margin.
// The SD buffer, the radio queue item size, and the radio task buffers MUST all use
// this so the full row isn't truncated or read past the end of the buffer.
#define CSV_BUFFER_SIZE 512

// ─── PID Gains ────────────────────────────────────────────────────────────────
#define Kp 0.45f
#define Ki 0.0f
#define Kd 0.19f

// ─── Servo Configuration ──────────────────────────────────────────────────────
#define MOMENT_ARM 0.07383f    // meters
#define SERVO_MIN_US 1000      // Minimum pulse width (µs)
#define SERVO_MAX_US 2000      // Maximum pulse width (µs)
#define SERVO_CENTER_US 1500   // Neutral pulse width (µs)
#define SERVO_US_PER_DEG 10.0f // Pulse width change per degree

// ─── I2C ──────────────────────────────────────────────────────────────────────
#define ESP32_I2C_ADDRESS 0x30 // 48 decimal