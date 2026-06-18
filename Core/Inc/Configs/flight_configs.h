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
#define Ki 0.001f
#define Kd 0.25f

// ─── Servo Configuration ──────────────────────────────────────────────────────
#define MOMENT_ARM 0.07383f    // meters
#define SERVO_MIN_US 1000      // Minimum pulse width (µs)
#define SERVO_MAX_US 2000      // Maximum pulse width (µs)
#define SERVO_CENTER_US 1500   // Neutral pulse width (µs)
#define SERVO_US_PER_DEG 10.0f // Pulse width change per degree

// ─── Aerodynamic Control Model ────────────────────────────────────────────────
// Converts a desired control force (N) into a deflection angle (deg). See servo.c.
// Coefficient is calibrated to 8.33 deg/N at 200 m/s, scaling as 1/v^2 (aero
// force ~ v^2). Update CONTROL_FORCE_COEFFICIENT/REFERENCE_VELOCITY if the curve
// differs.
#define CONTROL_FORCE_COEFFICIENT 8.33f // degrees per Newton at the reference velocity
#define REFERENCE_VELOCITY 200.0f       // m/s — reference velocity for CONTROL_FORCE_COEFFICIENT
#define MINIMUM_AIRSPEED 10.0f          // m/s — below this, no aero authority -> 0 deflection
#define DEFLECTION_ANGLE_LIMIT 10.0f    // mechanical/stall clamp on deflection angle (deg)

// ─── I2C ──────────────────────────────────────────────────────────────────────
#define ESP32_I2C_ADDRESS 0x30 // 48 decimal