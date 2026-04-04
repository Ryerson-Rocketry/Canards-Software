#include <iostream>
#include <Eigen/Dense>
#include "StateEstimation/attitude_estimation.h"
#include "math.h"


AttitudeEstimation::AttitudeEstimation(/* args */)
{
    dt = 0.01f;
    I4 = Eigen::MatrixXd::Identity(4,4);
}

AttitudeEstimation::~AttitudeEstimation()
{
}


Eigen::Vector4f AttitudeEstimation::discretization() {

    float totalAngularSpeed = gyro.norm();
    
    float wx = gyro.x();
    float wy = gyro.y();
    float wz = gyro.z();
    
    Omega << 0,  -wx,  -wy,  -wz,
          wx,   0,   wz,  -wy,
          wy,  -wz,   0,   wx,
          wz,   wy, -wx,   0;

    attitude *= (cosf(totalAngularSpeed * dt /2) * I4 + (2/totalAngularSpeed) * sinf(totalAngularSpeed * dt /2) * Omega);

    return attitude;
}

Eigen::Vector4f AttitudeEstimation::linearization() {
    float wx = gyro.x();
    float wy = gyro.y();
    float wz = gyro.z();

    Omega << 0,  -wx,  -wy,  -wz,
        wx,   0,   wz,  -wy,
        wy,  -wz,   0,   wx,
        wz,   wy, -wx,   0;

    attitude = (I4 + dt * Omega / 2) * attitude;

    return attitude;
}

Eigen::Vector4f AttitudeEstimation::attitudeEstimation(float gyro_array[3] ) {
    
    // turn gyro into 3x1 Eigen matrix
    Eigen::Map<Eigen::MatrixXf> gyro(gyro_array, 3,1);
   
    // get attitude by integrating gyroscopic rates
    attitude += gyro * dt;
    
    /*
    Apply discretization because we are using discrete Kalman Filter and it
    preserves the physics when we step from one time instant to the next
    */

    attitude = discretization();


    /*
    We NEED to linearize our data as it's currently nonlinear and Kalman filters
    only accept linear data so we must take the derivative of it
    */ 

    attitude = linearization();

    return attitude;
}

Eigen::Vector4f AttitudeEstimation::getPredictionCovariance() {

}