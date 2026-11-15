#pragma once

void altPredict(float dt, float *pos, float *vel, float acceleration, float processNoise[2][2], float systemCov[2][2], float stateTransition[2][2]);
void altUpdate(float *pos, float *vel, float press, float systemCov[2][2], float baroAltitudeVariance);

/**
 * @brief Store the ground reference pressure (Pa) used as the altitude baseline
 */
void setGroundPressure(float p);

/**
 * @brief Altitude (m) above the stored ground reference pressure
 */
float getRelativeAltitude(float currentPressurePa);
