#pragma once

#include "main.h"
#include <Eigen/Dense>
#include <Eigen/src/Core/Matrix.h>


class AttitudeEstimation
{
private:
    float gyroNoiseSigma;
    Eigen::Vector3f gyro;
    Eigen::Matrix4f I4;
    Eigen::Matrix4f omega;
    Eigen::Matrix4f F;
    Eigen::Matrix4f Q;
    Eigen::Matrix<float, 6,1> Z;
    Eigen::Matrix<float, 6, 6> measurementNoiseCov;

    // World-frame magnetic reference, auto-calibrated from the sensor at startup
    // (rocket vertical on the pad) instead of being hardcoded to (1,0,0). See
    // attitudeCorrection.
    Eigen::Vector3f magReference;
    bool magRefInitialized;

    Eigen::Matrix4f Omega();
    Eigen::Vector4f discretization();
    Eigen::Matrix4f getStateErrorCovariance();


public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    AttitudeEstimation(/* args */);
    ~AttitudeEstimation();
    Eigen::Vector4f attitude;
    Eigen::Matrix4f stateErrorCov;
    float dt;
    Eigen::Vector4f attitudeEstimation(float gyro_array[3]);
    Eigen::Vector4f attitudeCorrection(float accel_array[3], float mag_array[3]);

};