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

    // Fallback until the real field direction is captured on the first correction.
    this->magReference = Eigen::Vector3f(1.0f, 0.0f, 0.0f);
    this->magRefInitialized = false;
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

    this->attitude = ((cosf(totalAngularSpeed * this->dt /2.0f) * this->I4 + (1.0f/totalAngularSpeed) * sinf(totalAngularSpeed * this->dt /2.0f) * this->omega))*this->attitude;

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
    this->gyro = this->gyro * M_PI / 180.0f;
    /*
    Apply discretization because we are using discrete Kalman Filter and it
    preserves the physics when we step from one time instant to the next
    */

    this->attitude = discretization();

    // predict Pt, the state error covariance
    this->getStateErrorCovariance();

    return this->attitude;
}

Eigen::Vector4f AttitudeEstimation::attitudeCorrection(float accel_array[3], float mag_array[3], bool useAccel) {
    
    float mag_n[3] = {mag_array[0], mag_array[1], mag_array[2]};
    float mag_norm = sqrtf(mag_array[0]*mag_array[0] + mag_array[1]*mag_array[1] + mag_array[2]*mag_array[2]);
    if (mag_norm > 0.001f) {
        mag_n[0] = mag_array[0]/mag_norm;
        mag_n[1] = mag_array[1]/mag_norm;
        mag_n[2] = mag_array[2]/mag_norm;

        // Use the first valid reading as the local magnetic field reference.
        if (!magRefInitialized) {
            this->magReference = Eigen::Vector3f(mag_n[0], mag_n[1], mag_n[2]);
            this->magRefInitialized = true;
        }
    }

    float accel_norm = sqrtf(accel_array[0]*accel_array[0] + accel_array[1]*accel_array[1] + accel_array[2]*accel_array[2]);
    float accel_n[3] = {accel_array[0], accel_array[1], accel_array[2]};
    if (accel_norm > 0.001f) {
        accel_n[0] /= accel_norm;
        accel_n[1] /= accel_norm;
        accel_n[2] /= accel_norm;
    }
    this->Z << accel_n[0], accel_n[1], accel_n[2],
            mag_n[0],   mag_n[1],   mag_n[2];

    Eigen::Vector3f g(0.0f, 0.0f, 1.0f);
    Eigen::Vector3f r = this->magReference;

    // Predict how each reference vector appears in the sensor's body frame.
    auto worldToBody = [](const Eigen::Vector3f& v,
                          const Eigen::Vector4f& q) -> Eigen::Vector3f {
        float qw = q[0];
        Eigen::Vector3f qv = q.tail<3>();
        Eigen::Vector3f firstCross = qv.cross(v);
        return v - 2.0f * qw * firstCross + 2.0f * qv.cross(firstCross);
    };

    Eigen::Matrix<float, 6, 1> predicted;
    predicted.head<3>() = worldToBody(g, this->attitude);
    predicted.tail<3>() = worldToBody(r, this->attitude);

    // Estimate how the predicted readings change with each quaternion component.
    Eigen::Matrix<float, 6, 4> H;
    const float eps = 1e-4f;
    for (int column = 0; column < 4; ++column) {
        Eigen::Vector4f qPlus = this->attitude;
        Eigen::Vector4f qMinus = this->attitude;
        qPlus[column] += eps;
        qMinus[column] -= eps;
        qPlus.normalize();
        qMinus.normalize();

        Eigen::Matrix<float, 6, 1> hPlus;
        Eigen::Matrix<float, 6, 1> hMinus;
        hPlus.head<3>() = worldToBody(g, qPlus);
        hPlus.tail<3>() = worldToBody(r, qPlus);
        hMinus.head<3>() = worldToBody(g, qMinus);
        hMinus.tail<3>() = worldToBody(r, qMinus);
        H.col(column) = (hPlus - hMinus) / (2.0f * eps);
    }

    // in flight accel is thrust/drag not gravity, so ignore its rows (zero innovation, huge noise)
    Eigen::Matrix<float, 6, 6> Rt = this->measurementNoiseCov;
    if (!useAccel)
        Rt.block<3, 3>(0, 0) *= 1.0e4f;

    // Kalman gain: K = Pt * H^T * (H * Pt * H^T + Rt)^-1
    Eigen::Matrix<float, 4, 6> K = stateErrorCov * H.transpose()
                                 * (H * stateErrorCov * H.transpose() + Rt).inverse();

    // Compare the sensor readings with the predicted readings.
    Eigen::Matrix<float, 6, 1> innovation = this->Z - predicted;
    if (!useAccel)
        innovation.head<3>().setZero();

    // Use that difference to update the attitude estimate.
    this->attitude = this->attitude + K * innovation;

    // Keep the quaternion at unit length.
    this->attitude.normalize();

    // Update the uncertainty after applying the correction.
    this->stateErrorCov = (Eigen::Matrix4f::Identity() - K * H) * this->stateErrorCov;

    return this->attitude;
}