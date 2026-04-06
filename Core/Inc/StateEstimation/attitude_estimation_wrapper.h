#pragma once

#ifdef __cplusplus
extern "C"
{
#endif

    // Opaque handle — C code never touches the C++ internals
    typedef void *AttitudeEstimationHandle;

    // Lifecycle
    AttitudeEstimationHandle AttitudeEstimation_create(void);
    void AttitudeEstimation_destroy(AttitudeEstimationHandle handle);

    // Update dt dynamically (call before estimation if dt changes)
    void AttitudeEstimation_setDt(AttitudeEstimationHandle handle, float dt);

    // Prediction step — call every gyro sample
    // gyro_array: [wx, wy, wz] in rad/s
    // out_quaternion: [qw, qx, qy, qz]
    void AttitudeEstimation_predict(AttitudeEstimationHandle handle,
                                    float gyro_array[3],
                                    float out_quaternion[4]);

    // Correction step — call when accel/mag data is available
    // accel_array: [ax, ay, az]
    // mag_array:   [mx, my, mz]
    // out_quaternion: [qw, qx, qy, qz]
    void AttitudeEstimation_correct(AttitudeEstimationHandle handle,
                                    float accel_array[3],
                                    float mag_array[3],
                                    float out_quaternion[4]);

    // Get RPY in degrees from current quaternion
    // out_rpy: [roll, pitch, yaw] in degrees
    void AttitudeEstimation_getRPY(AttitudeEstimationHandle handle,
                                   float out_rpy[3]);

#ifdef __cplusplus
}
#endif