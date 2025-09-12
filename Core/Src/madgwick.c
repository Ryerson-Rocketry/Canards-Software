#include "madgwick.h"
#include "math.h"

/*
 *  Perform the Quaternion Conjugate formula
 *  @param: float q[4], input quaternion
 *  @param: float out[4], array to hold quaternion conjugate
 */
void quatConjugate(float q[4], float out[4])
{
    out[0] = q[0];
    for (int i = 1; i < 4; i++)
    {
        out[i] = -1 * q[i];
    }
}

/*
 *  Normalize the quaternion
 *  @param: float q[4], input quaternion
 *  @param: float out[4], normalized quaternion
 */
void normalizeQuat(float q[4], float out[4])
{
    float norm = sqrtf(powf(q[0], 2) + powf(q[1], 2) + powf(q[2], 2) + powf(q[3], 2));
    for (int i = 0; i < 4; i++)
    {
        out[i] = q[i] / norm;
    }
}

/*
 *  Perform Quaternion Product between two Quaternians
 *  @param: float a [4], quaternion one
 *  @param: float b [4], quaternion two
 *  @param: float out [3], output array
 *
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
 *  Convert Quaternion to Euler Angles
 *  @param: float q [4], quaternion
 *  @param: float out [3], output array
 *  returns: [yaw, pitch, roll]
 */
void quat2EulerAng(float q[4], float out[3])
{
    out[0] = atan2f(2 * q[1] * q[2] - 2 * q[0] * q[3], 2 * q[0] * q[0] + 2 * q[1] * q[1] - 1);
    out[1] = -asinf(2 * q[1] * q[3] + 2 * q[0] * q[2]);
    out[2] = atan2f(2 * q[2] * q[3] - 2 * q[0] * q[1], 2 * q[0] * q[0] + 2 * q[3] * q[3] - 1);
}

/*
 *  Scalar multiplication of a quaternion
 *  @param: float q[4], input quaternion
 *  @param: scalar, scalar you want to multiply with
 *  @param: float out[4], array to hold quaternion conjugate
 */
void scalarMulQuat(float q[4], float scalar, float out[4])
{
    for (int i = 0; i < 4; i++)
    {
        out[i] = scalar * q[i];
    }
}

/*
 *  Perform the Quaternion Conjugate formula
 *  @param: float q[4], input quaternion
 *  @param: float out[4], array to hold quaternion conjugate
 *
 */
void calcAccelQuatDeriv(float accel[4], float q[4], float out[4])
{
    float temp[4];
    scalarMulQuat(q, 0.5f, temp);
    quatProduct(temp, accel, out);
}

/*
 * Calcuulate the quaternion dervied from accelerometer
 * @param: float q [4], estimated orientation
 * @param: float  accelQuatDeriv[4], the quaternion derivative of the accel quaternion
 * @param: float samplingTime, time elapsed from prev operation
 * @param: float out[4], output array to hold the acceleration quaternion
 *
 */
void calcAccelQuat(float q[4], float accelQuatDeriv[4], float samplingTime, float out[4])
{
    float temp[4];
    scalarMulQuat(accelQuatDeriv, samplingTime, temp);

    for (int i = 0; i < 4; i++)
    {
        out[i] = q[i] + temp[i];
    }
}

/**
 * Calculaate the normalized vector of Earth's magnetic inclincation
 * @param: float q[4], input quaternion
 * @param: float  mag [4], magnetometer values
 * @param: float quatDeriv[4], the quaternion derivative of the estimated orientation
 * @param: calculates Earth's magnetic field relative to Magnetometer (I think)
 *
 */
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

/**
 *  Calculates the objective function of the magnetometer data
 *  @param: float earthMagInc[4], the Earth's magnetic inclination in form of a quaternion
 *  @param: float  mag [4], magnetometer data
 *  @param: float q[4], estimated orientation
 *  @param: float out[4], output array
 *
 */
void calcMagObjectiveFunc(float earthMagInc[4], float mag[4], float q[4], float out[3])
{
    out[0] = 2 * earthMagInc[1] * (0.5f - q[2] * q[2] - q[3] * q[3]) + 2 * earthMagInc[3] * (q[1] * q[3] - q[0] * q[2]) - mag[1];
    out[1] = 2 * earthMagInc[1] * (q[1] * q[2] - q[0] * q[3]) + 2 * earthMagInc[3] * (q[0] * q[1] + q[2] * q[3]) - mag[2];
    out[2] = 2 * earthMagInc[1] * (q[0] * q[2] + q[1] * q[3]) + 2 * earthMagInc[3] * (0.5f - q[1] * q[1] - q[2] * q[2]) - mag[3];
}

/**
 *  Calculates the Jacobian  matrix of the Magnetometer  data
 *  @param: float q [4], estimated orientation
 *  @param: float b [3], vector where filter ref direc is the same of earth mag field
 *  @param: float J[3][4], Output of Jacboian matrix with size 3x4
 */
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

/**
 * Calculates the accelerometer objective function for gradient descent.
 * This expresses the difference between the expected gravity vector (from quaternion) and measured acceleration.
 * Used as the "f" vector in Madgwick's gradient computation.
 * @param q     [4] Estimated orientation quaternion.
 * @param accel [4] Measured accelerometer data (first element ignored; a_x, a_y, a_z at [1],[2],[3]).
 * @param out   [3] Output: objective function vector.
 *
 * out[0]: Error along X-axis.
 * out[1]: Error along Y-axis.
 * out[2]: Error along Z-axis.
 */
void calcAccelObjFunc(float q[4], float accel[4], float out[3])
{
    float ax = accel[1];
    float ay = accel[2];
    float az = accel[3];

    out[0] = 2 * (q[1] * q[3] - q[0] * q[2]) - ax;
    out[1] = 2 * (q[0] * q[1] + q[2] * q[3]) - ay;
    out[2] = 2 * (0.5f - q[1] * q[1] - q[2] * q[2]) - az;
}

/**
 * Computes the Jacobian matrix of the accelerometer objective function with respect to the quaternion.
 * This is the partial derivative d(f_accel)/d(q) used in gradient descent.
 * @param q [4]     Estimated orientation quaternion.
 * @param J [3][4]  Output: 3x4 Jacobian matrix.
 */
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

/**
 * Transposes a 3x4 matrix into a 4x3 matrix.
 * @param q   [3][4] Input matrix.
 * @param out [4][3] Output: transposed matrix (out[j][i] = q[i][j]).
 */
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

/**
 * Concatenates two 3-element vectors into a single 6-element vector.
 * Used for combining magnetometer and accelerometer objective functions.
 * @param fg  [3]  First vector (e.g., magnetometer error).
 * @param fb  [3]  Second vector (e.g., accelerometer error).
 * @param out [6]  Output: combined vector ([fg, fb]).
 */
void magAndAccelObjFunc(float fg[3], float fb[3], float out[6])
{
    for (int i = 0; i < 3; i++)
    {
        out[i] = fg[i];
        out[i + 3] = fb[i];
    }
}

/**
 * Stacks two 3x4 Jacobian matrices (mag and accel) vertically and transposes the result.
 * Final output is 4x6, used for computing the quaternion gradient in Madgwick's filter.
 * @param Jg   [3][4] Magnetometer Jacobian.
 * @param Jb   [3][4] Accelerometer Jacobian.
 * @param out  [4][6] Output: stacked and transposed matrix.
 */
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

/**
 * Matrix-vector multiplication: multiplies a 4x6 matrix by a 6x1 vector to get a 4x1 vector.
 * Used to compute the gradient vector in Madgwick's filter: gradient = J^T * f.
 * @param magAndAccelJMatT [4][6]  Transposed, stacked Jacobian matrix.
 * @param magAndAccelObj   [6]     Combined objective function vector.
 * @param out              [4]     Output: gradient vector (length 4).
 */
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

/**
 * Calculates the adaptive step size (μ_t) for Madgwick's filter.
 * This determines how large a step to take in gradient descent, based on quaternion derivative norm.
 * μ_t = α * ||q̇_ω|| * Δt, where α is augmentationOfStepSize.
 * @param augmentationOfStepSize Scalar parameter (α, usually > 1).
 * @param quatDeriv [4]   Quaternion derivative (from gyro).
 * @param samplingSize    Sampling period (Δt).
 * @param out             Pointer to step size output.
 */
void calcStepSize(float augmentationOfStepSize, float quatDeriv[4], float samplingSize, float *out)
{
    if (augmentationOfStepSize < 1)
    {
        return;
    }

    float norm = sqrtf(powf(quatDeriv[0], 2) + powf(quatDeriv[1], 2) + powf(quatDeriv[2], 2) + powf(quatDeriv[3], 2));

    *out = augmentationOfStepSize * norm * samplingSize;
}

/**
 * Updates the quaternion estimate using a step of gradient descent.
 * q_new = normalize(q - μ * (gradient / ||gradient||))
 * @param q        [4] Current quaternion estimate.
 * @param stepSize      Step size (μ_t).
 * @param gradient [4]  Gradient vector (from J^T * f).
 * @param out      [4]  Output: updated, normalized quaternion.
 */
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

/**
 * Normalizes a quaternion gradient to get the direction of the error.
 * @param gradient [4] Quaternion gradient vector.
 * @param out      [4] Output: normalized direction of error quaternion.
 */
void directionOfErrorQuat(float gradient[4], float out[4])
{
    normalizeQuat(gradient, out);
}

/**
 * Calculates the estimated quaternion derivative for Madgwick's update step.
 * q̇_est = q̇_ω - β * directionOfErrorQuat
 * @param accelQuatDerivative [4] Derivative from gyro (q̇_ω).
 * @param beta               Filter gain parameter (β, sometimes called μ).
 * @param direcOfErrorQuat   [4] Direction of error quaternion (unit vector).
 * @param out                [4] Output: estimated quaternion derivative.
 */
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

/**
 * Computes the gyro error quaternion (used to correct for gyro bias).
 * error = 2 * q * conj(directionOfErrorQuat)
 * @param q                [4] Current quaternion estimate.
 * @param direcOfErrorQuat [4] Direction of error quaternion (normalized).
 * @param out              [4] Output: gyro error quaternion.
 */
void getGyroError(float q[4], float direcOfErrorQuat[4], float out[4])
{
    float temp[4];
    scalarMulQuat(q, 2, temp);
    float qConjugate[4];
    quatConjugate(temp, qConjugate);
    quatProduct(qConjugate, direcOfErrorQuat, out);
}

/**
 * Integrates gyro error over time to estimate the gyroscope bias (for drift correction).
 * out[i] += integralGain * gyroError[i] * samplingSize
 * @param gyroError    [4] Gyro error quaternion.
 * @param samplingSize Scalar time step.
 * @param integralGain Scalar gain for integral update.
 * @param out [4]      Output: accumulated gyro bias.
 */
void getGyroBias(float gyroError[4], float samplingSize, float integralGain, float out[4])
{
    for (int i = 0; i < 4; ++i)
    {
        out[i] += integralGain * gyroError[i] * samplingSize;
    }
}

/**
 * Computes bias-compensated gyro measurement for Madgwick filter update.
 * Subtracts estimated gyro bias from measured (or predicted) gyro data.
 * @param accel    [4] Gyro measurement (gyro_w), typically [0, gx, gy, gz].
 * @param gyroBias [4] Estimated gyro bias (integral).
 * @param out [4]      Output: bias-compensated gyro measurement.
 * @note Only indices 1..3 are used; out[0] is set to zero.
 */
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
