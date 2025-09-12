#include "madgwick.h"
#include "math.h"

/*
 *  Perform the Quaternian Conjugate formula
 *  param: float q[4], input quaternian
 *  param: float out[4], array to hold quaternian conjugate
 *  return: array that holds the conjugate of the input
 */
void quatConjugate(float q[4], float out[4])
{
    out[0] = q[0];
    for (int i = 1; i < 4; i++)
    {
        out[i] = -1 * q[i];
    }
}

void normalizeQuat(float q[4], float out[4])
{
    float norm = sqrtf(powf(q[0], 2) + powf(q[1], 2) + powf(q[2], 2) + powf(q[3], 2));
    for (int i = 0; i < 4; i++)
    {
        out[i] = q[i] / norm;
    }
}

/*
 *  Perform Quaternian Product between two Quaternians
 *  param: float a [4], quaternian one
 *  param: float b [4], quaternian two
 *  param: float out [3], output array
 *  return: (1x4 Matrix) quaternian but the matrix is transposed
 */
void quatProduct(float a[4], float b[4], float out[4])
{
    out[0] = a[0] * b[0] - a[1] * b[1] - a[2] * b[2] - a[3] * b[3];
    out[1] = a[0] * b[1] + a[1] * b[0] + a[2] * b[3] - a[3] * b[2];
    out[2] = a[0] * b[2] - a[1] * b[3] + a[2] * b[0] + a[3] * b[1];
    out[3] = a[0] * b[3] + a[1] * b[2] - a[2] * b[1] + a[3] * b[0];
    normalizeQuat(out, out);
}

/*
 *  Convert Quaternian to Euler Angles
 *  param: float q [4], quaternian
 *  param: float out [3], output array
 *  return: [yaw, pitch, roll]
 */
void quat2EulerAng(float q[4], float out[3])
{
    out[0] = atan2f(2 * q[1] * q[2] - 2 * q[0] * q[3], 2 * q[0] * q[0] + 2 * q[1] * q[1] - 1);
    out[1] = -asinf(2 * q[1] * q[3] + 2 * q[0] * q[2]);
    out[2] = atan2f(2 * q[2] * q[3] - 2 * q[0] * q[1], 2 * q[0] * q[0] + 2 * q[3] * q[3] - 1);
}

void scalarMulQuat(float q[4], float scalar, float out[4])
{
    for (int i = 0; i < 4; i++)
    {
        out[i] = scalar * q[i];
    }
}

void calcAccelQuatDeriv(float accel[4], float q[4], float out[4])
{
    float temp[4];
    scalarMulQuat(q, 0.5f, temp);
    quatProduct(temp, accel, out);
}

void calcAccelQuat(float q[4], float accelQuatDeriv[4], float samplingTime, float out[4])
{
    float temp[4];
    scalarMulQuat(accelQuatDeriv, samplingTime, temp);

    for (int i = 0; i < 4; i++)
    {
        out[i] = q[i] + temp[i];
    }
}

void computeReferenceMag2EarthFrame(float q[4], float mag[4], float quatDeriv[4], float out[4])
{
    float temp[4];
    quatProduct(q, mag, temp);
    quatProduct(temp, quatDeriv, out);

    float normalizedVal[4];
    normalizeQuat(out, normalizedVal);

    out[0] = 0;
    out[1] = sqrtf(normalizedVal[1] * normalizedVal[1] + normalizedVal[2] * normalizedVal[2]);
    out[2] = 0;
}

void calcMagObjectiveFunc(float earthMagInc[4], float mag[4], float q[4], float out[3])
{
    out[0] = 2 * earthMagInc[1] * (0.5f - q[2] * q[2] - q[3] * q[3]) + 2 * earthMagInc[3] * (q[1] * q[3] - q[0] * q[2]) - mag[1];
    out[1] = 2 * earthMagInc[1] * (q[1] * q[2] - q[0] * q[3]) + 2 * earthMagInc[3] * (q[0] * q[1] + q[2] * q[3]) - mag[2];
    out[2] = 2 * earthMagInc[1] * (q[0] * q[2] + q[1] * q[3]) + 2 * earthMagInc[3] * (0.5f - q[1] * q[1] - q[2] * q[2]) - mag[3];
}

void calcMagJacobianMat(float q[4], float b[3], float J[3][4])
{
    float bx = b[0];
    float bz = b[2];
    float q1 = q[0];
    float q2 = q[1];
    float q3 = q[2];
    float q4 = q[3];

    J[0][0] = -2 * bz * q3;
    J[0][1] = 2 * bz * q4;
    J[0][2] = -4 * bx * q3 - 2 * bz * q1;
    J[0][3] = -4 * bx * q4 + 2 * bz * q2;

    J[1][0] = -2 * bx * q4 + 2 * bz * q2;
    J[1][1] = 2 * bx * q3 + 2 * bz * q1;
    J[1][2] = 2 * bx * q2 + 2 * bz * q4;
    J[1][3] = -2 * bx * q1 + 2 * bz * q3;

    J[2][0] = 2 * bx * q3;
    J[2][1] = 2 * bx * q4 - 4 * bz * q2;
    J[2][2] = 2 * bx * q1 - 4 * bz * q3;
    J[2][3] = 2 * bx * q2;
}

void calcAccelObjFunc(float q[4], float accel[4], float out[3])
{
    float ax = accel[1];
    float ay = accel[2];
    float az = accel[3];

    out[0] = 2 * (q[1] * q[3] - q[0] * q[2]) - ax;
    out[1] = 2 * (q[0] * q[1] + q[2] * q[3]) - ay;
    out[2] = 2 * (0.5f - q[1] * q[1] - q[2] * q[2]) - az;
}

void calcAccelJacobianMat(float q[4], float J[3][4])
{
    J[0][0] = -2 * q[2];
    J[0][1] = 2 * q[3];
    J[0][2] = -2 * q[0];
    J[0][3] = 2 * q[1];

    J[1][0] = 2 * q[1];
    J[1][1] = 2 * q[0];
    J[1][2] = 2 * q[3];
    J[1][3] = 2 * q[2];

    J[2][0] = 0;
    J[2][1] = -4 * q[1];
    J[2][2] = -4 * q[2];
    J[2][3] = 0;
}

void mat3x4Transpose(float q[3][4], float out[4][3])
{
    for (int i = 0; i < 3; i++)
    {
        for (int j = 0; j < 4; j++)
        {
            out[j][i] = q[i][j];
        }
    }
}

void magAndAccelObjFunc(float fg[3], float fb[3], float out[6])
{
    for (int i = 0; i < 3; i++)
    {
        out[i] = fg[i];
        out[i + 3] = fb[i];
    }
}

void magAndAccelJacobT(float Jg[3][4], float Jb[3][4], float out[4][6])
{
    float stacked[6][4];

    for (int i = 0; i < 3; ++i)
        for (int j = 0; j < 4; ++j)
            stacked[i][j] = Jg[i][j];

    for (int i = 0; i < 3; ++i)
        for (int j = 0; j < 4; ++j)
            stacked[i + 3][j] = Jb[i][j];

    for (int i = 0; i < 4; ++i)
        for (int j = 0; j < 6; ++j)
            out[i][j] = stacked[j][i];
}

void gradientObjFunc(float magAndAccelJMatT[4][6], float magAndAccelObj[6], float out[4])
{
    for (int i = 0; i < 4; i++)
    {
        out[i] = 0;
        for (int j = 0; j < 6; j++)
        {
            out[i] += magAndAccelJMatT[i][j] * magAndAccelObj[j];
        }
    }
}

void calcStepSize(float augmentationOfStepSize, float quatDeriv[4], float samplingSize, float *out)
{
    if (augmentationOfStepSize < 1)
    {
        return;
    }

    float norm = sqrtf(powf(quatDeriv[0], 2) + powf(quatDeriv[1], 2) + powf(quatDeriv[2], 2) + powf(quatDeriv[3], 2));

    *out = augmentationOfStepSize * norm * samplingSize;
}

void calcGradientDescentQuat(float q[4], float stepSize, float gradient[4], float out[4])
{

    float temp[4];
    float normalizedGradient[4];

    normalizeQuat(gradient, normalizedGradient);
    scalarMulQuat(normalizedGradient, stepSize, temp);

    for (int i = 0; i < 4; i++)
    {
        out[i] = q[i] - temp[i];
    }

    normalizeQuat(out, out);
}

void directionOfErrorQuat(float gradient[4], float out[4])
{
    normalizeQuat(gradient, out);
}

void estimatedQuatDeriv(float accelQuatDerivative[4], const float beta, float direcOfErrorQuat[4], float out[4])
{
    float temp[4];
    scalarMulQuat(direcOfErrorQuat, beta, temp);

    for (int i = 0; i < 4; i++)
    {
        out[i] = accelQuatDerivative[i] - temp[i];
    }
    normalizeQuat(out, out);
}

void getGyroError(float q[4], float direcOfErrorQuat[4], float out[4])
{
    float temp[4];
    scalarMulQuat(q, 2, temp);
    float qConjugate[4];
    quatConjugate(temp, qConjugate);
    quatProduct(qConjugate, direcOfErrorQuat, out);
}

void getGyroBias(float gyroError[4], float samplingSize, float integralGain, float out[4])
{
    for (int i = 0; i < 4; ++i)
    {
        out[i] += integralGain * gyroError[i] * samplingSize;
    }
}

void getCompensatedGyroMeas(float accel[4], float gyroBias[4], float out[4])
{
    out[0] = 0;
    for (int i = 1; i < 4; i++)
    {
        out[i] = accel[i] - gyroBias[i];
    }
}

void updateMadgwick(Madgwick *madgwick)
{

    // get Earth Magentic Field Inclination Matrix
    float earthMagneticFieldInc[4];
    computeReferenceMag2EarthFrame(madgwick->estOrientation, madgwick->mag_sensor, madgwick->quatEstDeriv, earthMagneticFieldInc);

    // gradient descent

    // magnetometer
    float Jb[3][4];
    calcMagJacobianMat(madgwick->estOrientation, earthMagneticFieldInc, Jb);

    float fb[3];
    calcMagObjectiveFunc(earthMagneticFieldInc, madgwick->mag_sensor, madgwick->estOrientation, fb);

    // accelerometer

    float accelQuatDeriv[4];
    calcAccelQuatDeriv(madgwick->accel_sensor, madgwick->estOrientation, accelQuatDeriv);

    float fg[3];
    calcAccelObjFunc(madgwick->estOrientation, madgwick->accel_sensor, fg);

    float Jg[3][4];
    calcAccelJacobianMat(madgwick->estOrientation, Jg);

    // sensor fusion combine the accelerometer and magnetometer data
    float fgb[6];
    magAndAccelObjFunc(fg, fb, fgb);

    float JgbT[4][6];
    magAndAccelJacobT(Jg, Jb, JgbT);

    // calculate the objective functiong gradient
    float gradObjFuncMat[4];
    gradientObjFunc(JgbT, fgb, gradObjFuncMat);

    // get the direction of error
    float direcErrorQuat[4];
    directionOfErrorQuat(gradObjFuncMat, direcErrorQuat);

    // get gyro error
    float gyroError[4];
    getGyroError(madgwick->estOrientation, direcErrorQuat, gyroError);

    // get gyro bias
    getGyroBias(gyroError, madgwick->dt, madgwick->integralGain, madgwick->gyroBias);

    // get compesnated gyro measurements
    getCompensatedGyroMeas(madgwick->gyro_sensor, madgwick->gyroBias, madgwick->compensatedGyroMeasurements);

    float gyroQuat[4] = {0,
                         madgwick->compensatedGyroMeasurements[0],
                         madgwick->compensatedGyroMeasurements[1],
                         madgwick->compensatedGyroMeasurements[2]};

    float qDerivGyro[4];
    quatProduct(madgwick->estOrientation, gyroQuat, qDerivGyro);

    for (int i = 0; i < 4; i++)
    {
        qDerivGyro[i] *= 0.5f;
    }

    // calculate the step size
    float stepSize = 0;
    calcStepSize(madgwick->augmentationOfStepSize, qDerivGyro, madgwick->dt, &stepSize);

    float gradCorr[4];
    scalarMulQuat(direcErrorQuat, madgwick->beta, gradCorr);

    for (int i = 0; i < 4; i++)
    {
        madgwick->quatEstDeriv[i] = qDerivGyro[i] - gradCorr[i];
    }

    // integrate and normalize
    for (int i = 0; i < 4; i++)
    {
        madgwick->estOrientation[i] += madgwick->quatEstDeriv[i] * madgwick->dt;
    }
    normalizeQuat(madgwick->estOrientation, madgwick->estOrientation);
    quat2EulerAng(madgwick->estOrientation, madgwick->euler);
}
