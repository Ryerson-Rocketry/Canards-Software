#include "Kalman/math_utils.h"

#define SEA_LEVEL_PRESSURE 101325.0f

void matMul2x2_2x2(float a[2][2], float b[2][2], float result[2][2])
{
    for (int i = 0; i < 2; i++)
    {
        for (int j = 0; j < 2; j++)
        {
            result[i][j] = a[i][0] * b[0][j] + a[i][1] * b[1][j];
        }
    }
}

void matTranspose(float a[2][2], float result[2][2])
{
    for (int i = 0; i < 2; i++)
    {
        for (int j = 0; j < 2; j++)
        {
            result[j][i] = a[i][j];
        }
    }
}

void matAdd(float a[2][2], float b[2][2], float result[2][2])
{
    for (int i = 0; i < 2; i++)
    {
        for (int j = 0; j < 2; j++)
        {
            result[i][j] = a[i][j] + b[i][j];
        }
    }
}

// result = v * m
// v is 1x2, m is 2x2, result is 1x2
void matMul1x2_2x2(float v[2], float m[2][2], float result[2])
{
    float tmp[2]; // local buffer for safety
    tmp[0] = v[0] * m[0][0] + v[1] * m[1][0];
    tmp[1] = v[0] * m[0][1] + v[1] * m[1][1];

    result[0] = tmp[0];
    result[1] = tmp[1];
}

// result = m * v
// m is 2x2, v is 2x1, result is 2x1
void matMul2x2_2x1(float m[2][2], float v[2], float result[2])
{
    float tmp[2]; // local buffer for safety
    tmp[0] = m[0][0] * v[0] + m[0][1] * v[1];
    tmp[1] = m[1][0] * v[0] + m[1][1] * v[1];

    result[0] = tmp[0];
    result[1] = tmp[1];
}

// result = a * b
// a is 2x1, b is 1x2, result is 2x2
void matMul2x1_1x2(float a[2], float b[2], float result[2][2])
{
    // Row 1
    result[0][0] = a[0] * b[0];
    result[0][1] = a[0] * b[1];

    // Row 2
    result[1][0] = a[1] * b[0];
    result[1][1] = a[1] * b[1];
}

// result = I - a
// a is 2x2, result is 2x2
void matIdentitySub(float a[2][2], float result[2][2])
{
    float tmp00 = 1.0f - a[0][0];
    float tmp01 = 0.0f - a[0][1];
    float tmp10 = 0.0f - a[1][0];
    float tmp11 = 1.0f - a[1][1];

    result[0][0] = tmp00;
    result[0][1] = tmp01;
    result[1][0] = tmp10;
    result[1][1] = tmp11;
}

float calculateAltitude(float pressurePa)
{
    return 0.0f;
}