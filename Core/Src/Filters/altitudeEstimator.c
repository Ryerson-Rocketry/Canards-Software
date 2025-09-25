#include "madgwick.h"
#include "math.h"

float calcVerticalLinearAccel(float accel[4], float orien[4])
{
    float orienConjugate[4];
    quatConjugate(orien, orienConjugate);

    float accelEarthFrame[4];
    float temp[4];
    quatProduct(orien, accel, temp);
    quatProduct(temp, orienConjugate, accelEarthFrame);

    float gravity = 9.81f;

    return accelEarthFrame[3] - gravity;
}

float pressureToAltitude(float P, float P0)
{
    return 44330.0f * (1.0f - powf(P / P0, 0.190294957f));
}

float iirFilter(float input, float prev_output, float alpha)
{
    return alpha * input + (1.0f - alpha) * prev_output;
}

// Ts
float calculateSamplingPeriod(float time, float iteration)
{
    return time / iteration;
}

// delta Xk-1
float calcPositionError(float baroZ, float estPos)
{
    return baroZ - estPos;
}

void calcKalmanGain(float vertAccelStdDev, float baroStdDev, float out[2])
{
    out[0] = sqrt(2 * vertAccelStdDev / baroStdDev);
    out[1] = vertAccelStdDev / baroStdDev;
}

// delta Vk-1
float calcVelCorrection(float samplingTime, float verticalAccel, float baroAlt)
{
    return samplingTime * verticalAccel * baroAlt;
}

void complementaryFilter(float samplingPeriod, float prev[2], float kalmanGain[2], float posError, float velCorrection, float out[2])
{
    float A0[2][2] = {{1, samplingPeriod}, {0, 1}};
    float A1[2][2] = {{1, samplingPeriod / 2}, {0, 1}};
    float A2[2] = {samplingPeriod / 2, 1};

    // A0 * prev
    float x0[2];
    x0[0] = A0[0][0] * prev[0] + A0[0][1] * prev[1];
    x0[1] = A0[1][0] * prev[0] + A0[1][1] * prev[1];

    // A1 * Kalman Gain * Sampling Period * Position Error
    float x1[2];
    x1[0] = (A1[0][0] * kalmanGain[0] + A1[0][1] * kalmanGain[1]) * samplingPeriod * posError;
    x1[1] = (A1[1][0] * kalmanGain[0] + A1[1][1] * kalmanGain[1]) * samplingPeriod * posError;

    // A2 * Velocity Correction Term
    float x2[2];
    x2[0] = A2[0] * velCorrection;
    x2[1] = A2[1] * velCorrection;

    // out[0] = height, out[1] = velocity
    for (int i = 0; i < 2; i++)
    {
        out[i] = x0[i] + x1[i] + x2[i];
    }
}
