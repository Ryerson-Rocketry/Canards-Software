#include "math.h"
#include "Kalman/math_utils.h"

/**
 * @brief Predicts the next state using the Kalman filter prediction step.
 *
 * Performs the prediction phase of the Kalman filter, updating position and velocity
 * estimates based on the acceleration and process noise. Updates the system covariance
 * matrix to reflect the increased uncertainty after prediction.
 *
 * @param dt Time step (delta time) in seconds for the prediction interval.
 * @param pos Pointer to the current position estimate. Will be updated with predicted position.
 * @param vel Pointer to the current velocity estimate. Will be updated with predicted velocity.
 * @param acceleration The acceleration value applied during this time step.
 * @param processNoise 2x2 matrix representing the process noise covariance.
 * @param systemCov 2x2 matrix representing the system state covariance. Will be updated with predicted covariance.
 *
 * @return void
 *
 * @note This function modifies the values pointed to by pos, vel, and systemCov in-place.
 */
void kalmanPredict(float dt, float *pos, float *vel, float acceleration, float processNoise[2][2], float systemCov[2][2])
{
    // F = state transition, allows us to model natural process of system
    float stateTransition[2][2] = {
        {1.0f, dt},
        {0.0f, 1.0f}};
    float stateTransitionT[2][2];
    float out[2][2];
    float tmp[2][2];
    *pos += (*vel * dt) + (0.5f * acceleration * dt * dt);
    *vel += acceleration * dt;

    // P = FPF^T + Q
    matMul2x2_2x2(stateTransition, systemCov, tmp);  // FP
    matTranspose(stateTransition, stateTransitionT); // F^T
    matMul2x2_2x2(tmp, stateTransitionT, out);       // FPF^T
    matAdd(out, processNoise, systemCov);            // FPF^T + Q
}

/**
 * @brief Updates the Kalman filter state estimates for position and velocity
 *
 * @param[in,out] pos Pointer to the current position estimate; will be updated with new estimate
 * @param[in,out] vel Pointer to the current velocity estimate; will be updated with new estimate
 * @param[in] press Current pressure measurement used as observation input
 * @param[in,out] systemCov 2x2 system covariance matrix; will be updated after prediction step
 *
 * @return void
 *
 * @details This function performs a measurement update step of the Kalman filter,
 * incorporating new pressure readings to refine position and velocity estimates.
 * The system covariance matrix is modified in place.
 */
void kalmanUpdate(float *pos, float *vel, float press, float systemCov[2][2])
{
    float measurementFunc[2] = {1, 0}; // H
    float kalmanGain[2];               // K
    float residual;                    // y
    float systemUncertainty;           // S
    float tmp[2];
    float out[2];
    float res[2][2];
    float temp[2][2];

    float baroAltitudeVariance = 0.05f;

    // K = PH^TS^-1
    matMul2x2_2x1(systemCov, measurementFunc, tmp); // tmp = PH^T
    matMul1x2_2x2(measurementFunc, systemCov, out); // out = HP
    systemUncertainty = matMul1x2_2x1(out, measurementFunc) + baroAltitudeVariance;

    kalmanGain[0] = tmp[0] * (1.0f / systemUncertainty);
    kalmanGain[1] = tmp[1] * (1.0f / systemUncertainty);

    // y = z - Hx, H = [1 0]^T, so no velocity
    residual = press - (*pos);

    // x = x + Ky
    *pos += kalmanGain[0] * residual;
    *vel += kalmanGain[1] * residual;

    // P = (I-KH)P
    matMul2x1_1x2(kalmanGain, measurementFunc, res); // KH
    matIdentitySub(res, temp);                       // I - KH
    matMul2x2_2x2(temp, systemCov, systemCov);
}
