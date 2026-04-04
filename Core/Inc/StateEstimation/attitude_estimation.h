#pragma once

#include "main.h"
#include <Eigen/Dense>
#include <Eigen/src/Core/Matrix.h>


class AttitudeEstimation
{
private:
    float dt;
    Eigen::Matrix<float, 3, 1> gyro;
    Eigen::MatrixXd I4;
    Eigen::Matrix4f Omega;

    Eigen::Vector4f discretization();
    Eigen::Vector4f AttitudeEstimation::linearization();



public:
    AttitudeEstimation(/* args */);
    ~AttitudeEstimation();
    Eigen::Vector4f attitude;
    Eigen::Vector4f attitudeEstimation(float gyro_array[3]);
    Eigen::Vector4f AttitudeEstimation::getPredictionCovariance();
};