#ifndef ALTITUDE_ESTIMATOR_H
#define ALTITUDE_ESTIMATOR_H

#ifdef __cplusplus
extern "C"
{
#endif

    float calcVerticalLinearAccel(float accel[4], float orien[4]);
    float iirFilter(float input, float prev_output, float alpha);
    float calculateSamplingPeriod(float time, float iteration);
    void calcKalmanGain(float vertAccelStdDev, float baroStdDev, float out[2]);
    float calcVelCorrection(float samplingTime, float verticalAccel, float baroAlt);
    float calcPositionError(float baroZ, float estPos);
    void complementaryFilter(
        float samplingPeriod,
        float prev[2],
        float kalmanGain[2],
        float posError,
        float velCorrection,
        float out[2]);
    float pressureToAltitude(float P, float P0);

#ifdef __cplusplus
}
#endif

#endif