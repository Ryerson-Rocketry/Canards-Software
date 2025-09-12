#ifndef MADGWICK_H
#define MADGWICK_H

#include <math.h>

typedef struct
{
    float mag_sensor[4];
    float accel_sensor[4];
    float gyro_sensor[4];
    float compensatedGyroMeasurements[4];
    float beta;
    float integralGain;
    float augmentationOfStepSize;
    float gyroBias[4];
    float dt;
    float estOrientation[4];
    float direcErrorQuat[4];
    float quatEstDeriv[4];
    float gravity[4];
    float euler[3];
} Madgwick;

// update function
void updateMadgwick(Madgwick *madgwick);

// Math helpers
void quatConjugate(float q[4], float out[4]);
void normalizeQuat(float q[4], float out[4]);
void quatProduct(float a[4], float b[4], float out[4]);
void quat2EulerAng(float q[4], float out[3]);
void scalarMulQuat(float q[4], float scalar, float out[4]);
void calcAccelQuatDeriv(float accel[4], float q[4], float out[4]);
void calcAccelQuat(float q[4], float accelQuatDeriv[4], float samplingTime, float out[4]);
void computeReferenceMag2EarthFrame(float q[4], float mag[4], float quatDeriv[4], float out[4]);
void calcMagObjectiveFunc(float earthMagInc[4], float mag[4], float q[4], float out[3]);
void calcMagJacobianMat(float q[4], float b[3], float J[3][4]);
void calcAccelObjFunc(float q[4], float accel[4], float out[3]);
void calcAccelJacobianMat(float q[4], float J[3][4]);
void mat3x4Transpose(float q[3][4], float out[4][3]);
void magAndAccelObjFunc(float fg[3], float fb[3], float out[6]);
void magAndAccelJacobT(float gravJacobian[3][4], float magJacobian[3][4], float out[4][6]);
void gradientObjFunc(float magAndAccelJMatT[4][6], float magAndAccelObj[6], float out[4]);
void calcStepSize(float augmentationOfStepSize, float quatDeriv[4], float samplingSize, float *out);
void calcGradientDescentQuat(float q[4], float stepSize, float gradient[4], float out[4]);
void directionOfErrorQuat(float gradient[4], float out[4]);
void estimatedQuatDeriv(float accelQuatDerivative[4], const float beta, float direcOfErrorQuat[4], float out[4]);
void getGyroError(float q[4], float direcOfErrorQuat[4], float out[4]);
void getGyroBias(float gyroError[4], float samplingSize, float integralGain, float out[4]);
void getCompensatedGyroMeas(float accel[4], float gyroBias[4], float out[4]);

#endif
