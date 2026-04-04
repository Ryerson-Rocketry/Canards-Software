#include <iostream>
#define EIGEN_DEFAULT_SCALAR float  // or use explicit float types throughout
#include <Eigen/Dense>
#include "StateEstimation/attitude_estimation.h"
#include "math.h"


AttitudeEstimation::AttitudeEstimation(/* args */)
{
    this->dt = 0.01f;
    this->I4 = Eigen::Matrix4f::Identity();
    this->attitude = Eigen::Vector4f(1, 0, 0, 0);
    
    this->gyro = Eigen::Vector3f::Zero();
    this->omega = Eigen::Matrix4f::Zero();
    this->F = Eigen::Matrix4f::Zero();
    this->stateErrorCov = Eigen::Matrix4f::Identity();

    this->gyroNoiseSigma = 0.01f; 
    this->measurementNoiseCov = Eigen::Matrix<float, 6, 6>::Identity();
    this->measurementNoiseCov.block<3,3>(0,0) *= 0.1f;  // accel noise
    this->measurementNoiseCov.block<3,3>(3,3) *= 0.5f;  // mag noise
}

AttitudeEstimation::~AttitudeEstimation()
{
}

Eigen::Matrix4f AttitudeEstimation::Omega() {
    float wx = this->gyro.x();
    float wy = this->gyro.y();
    float wz = this->gyro.z();
    
    this->omega << 0,  -wx,  -wy,  -wz,
          wx,   0,   wz,  -wy,
          wy,  -wz,   0,   wx,
          wz,   wy, -wx,   0;
    
    return this->omega;
}

Eigen::Vector4f AttitudeEstimation::discretization() {

    float totalAngularSpeed = this->gyro.norm();
    
    if (totalAngularSpeed < 1e-6f) return this->attitude;

    this->omega = Omega();

    this->attitude = ((cosf(totalAngularSpeed * this->dt /2.0f) * this->I4 + (2.0f/totalAngularSpeed) * sinf(totalAngularSpeed * this->dt /2.0f) * this->omega))*this->attitude;

    return this->attitude;
}

Eigen::Vector4f AttitudeEstimation::linearization() {
    
    this->omega = Omega();

    this->attitude = (I4 + this->dt * this->omega / 2) * this->attitude;

    return this->attitude;
}

Eigen::Matrix4f AttitudeEstimation::getStateErrorCovariance() {
    float wx = this->gyro.x();
    float wy = this->gyro.y();
    float wz = this->gyro.z();

    F << 1, -this->dt * wx / 2.0f , -this->dt * wy / 2.0f, -this->dt * wz / 2,
        this->dt * wx / 2.0f, 1, this->dt * wz / 2.0f, -this->dt * wy /2.0f,
        this->dt * wy /2.0f, -this->dt * wz / 2, 1, this->dt * wx / 2,
        this->dt * wz / 2.0f, this->dt * wy / 2.0f, -this->dt * wx / 2.0f, 1;

    Eigen::Matrix4f F_Transpose = F.transpose(); 

    this->omega = Omega();
    this->Q = gyroNoiseSigma * gyroNoiseSigma * this->omega * this->omega.transpose() * this->dt;
    this->stateErrorCov = F * this->stateErrorCov * F_Transpose + this->Q;

    return this->stateErrorCov;
}


Eigen::Vector4f AttitudeEstimation::attitudeEstimation(float gyro_array[3] ) {
    
    // turn gyro into 3x1 Eigen matrix
    Eigen::Map<Eigen::Vector3f> gyroInput(gyro_array);
    this->gyro = gyroInput; 
   
    
    /*
    Apply discretization because we are using discrete Kalman Filter and it
    preserves the physics when we step from one time instant to the next
    */

    this->attitude = discretization();


    /*
    We NEED to linearize our data as it's currently nonlinear and Kalman filters
    only accept linear data so we must take the derivative of it
    */ 

    this->attitude = linearization();

    // predict Pt, the state error covariance
    this->getStateErrorCovariance();

    return this->attitude;
}

Eigen::Vector4f AttitudeEstimation::attitudeCorrection(float accel_array[3], float mag_array[3]) {
    
    // Pack measurements into Z (6x1)
    this->Z << accel_array[0], accel_array[1], accel_array[2],
               mag_array[0],   mag_array[1],   mag_array[2];

    // Reference vectors
    Eigen::Vector3f g(0.0f, 0.0f, 9.81f);
    Eigen::Vector3f r(1.0f, 0.0f, 0.0f);

    // Extract quaternion parts
    float qw = attitude[0];
    Eigen::Vector3f qv = attitude.tail<3>();

    // Skew-symmetric matrix helper
    auto skew = [](Eigen::Vector3f v) -> Eigen::Matrix3f {
        Eigen::Matrix3f S;
        S <<  0,      -v.z(),  v.y(),
              v.z(),   0,     -v.x(),
             -v.y(),   v.x(),  0;
        return S;
    };

    // Compute u vectors
    Eigen::Vector3f ug = g.cross(qv);
    Eigen::Vector3f ur = r.cross(qv);

    // Build H 
    Eigen::Matrix3f Hv_g = skew(ug + qw * g) + qv.dot(g) * Eigen::Matrix3f::Identity() - g * qv.transpose();
    Eigen::Matrix3f Hv_r = skew(ur + qw * r) + qv.dot(r) * Eigen::Matrix3f::Identity() - r * qv.transpose();

    // Assemble full 6x4 H matrix
    Eigen::Matrix<float, 6, 4> H;
    H.block<3,1>(0, 0) = 2.0f * ug;
    H.block<3,3>(0, 1) = 2.0f * Hv_g;
    H.block<3,1>(3, 0) = 2.0f * ur;
    H.block<3,3>(3, 1) = 2.0f * Hv_r;

    // Kalman gain: K = Pt * H^T * (H * Pt * H^T + Rt)^-1
    Eigen::Matrix<float, 4, 6> K = stateErrorCov * H.transpose()
                                 * (H * stateErrorCov * H.transpose() + this->measurementNoiseCov).inverse();

    // Innovation: difference between real measurement and predicted measurement
    Eigen::Matrix<float, 6, 1> innovation = this->Z - H * this->attitude;

    // State correction
    this->attitude = this->attitude + K * innovation;

    // Normalize quaternion to prevent drift
    this->attitude.normalize();

    // Covariance correction
    this->stateErrorCov = (Eigen::Matrix4f::Identity() - K * H) * this->stateErrorCov;

    return this->attitude;
}