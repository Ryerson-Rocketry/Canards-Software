#include "StateEstimation/attitude_estimation_wrapper.h"
#include "StateEstimation/attitude_estimation.h"
#include <Eigen/Dense>
#include <math.h>
#include "main.h"

#ifdef __cplusplus
extern "C" {
#endif

AttitudeEstimationHandle AttitudeEstimation_create(void) {
    return static_cast<void*>(new AttitudeEstimation());
}

void AttitudeEstimation_destroy(AttitudeEstimationHandle handle) {
    delete static_cast<AttitudeEstimation*>(handle);
}

void AttitudeEstimation_setDt(AttitudeEstimationHandle handle, float dt) {
    static_cast<AttitudeEstimation*>(handle)->dt = dt;
}

void AttitudeEstimation_predict(AttitudeEstimationHandle handle,
                                float gyro_array[3],
                                float out_quaternion[4]) {
    AttitudeEstimation* est = static_cast<AttitudeEstimation*>(handle);
    Eigen::Vector4f q = est->attitudeEstimation(gyro_array);
    out_quaternion[0] = q[0];
    out_quaternion[1] = q[1];
    out_quaternion[2] = q[2];
    out_quaternion[3] = q[3];
}

void AttitudeEstimation_correct(AttitudeEstimationHandle handle,
                                float accel_array[3],
                                float mag_array[3],
                                float out_quaternion[4]) {
    AttitudeEstimation* est = static_cast<AttitudeEstimation*>(handle);
    Eigen::Vector4f q = est->attitudeCorrection(accel_array, mag_array);
    out_quaternion[0] = q[0];
    out_quaternion[1] = q[1];
    out_quaternion[2] = q[2];
    out_quaternion[3] = q[3];
}

void AttitudeEstimation_getRPY(AttitudeEstimationHandle handle,
                               float out_rpy[3]) {
    AttitudeEstimation* est = static_cast<AttitudeEstimation*>(handle);
    Eigen::Vector4f q = est->attitude;

    float qw = q[0], qx = q[1], qy = q[2], qz = q[3];

    // Roll
    out_rpy[0] = atan2f(2.0f*(qw*qx + qy*qz),
                        1.0f - 2.0f*(qx*qx + qy*qy))
                 * (180.0f / M_PI);

    // Pitch
    float sinp = 2.0f*(qw*qy - qz*qx);
    out_rpy[1] = (fabsf(sinp) >= 1.0f)
                 ? copysignf(90.0f, sinp)
                 : asinf(sinp) * (180.0f / M_PI);

    // Yaw
    out_rpy[2] = atan2f(2.0f*(qw*qz + qx*qy),
                        1.0f - 2.0f*(qy*qy + qz*qz))
                 * (180.0f / M_PI);
}

#ifdef __cplusplus
}
#endif