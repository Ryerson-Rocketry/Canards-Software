void matMul2x2_2x2(float a[2][2], float b[2][2], float result[2][2]);
void matTranspose(float a[2][2], float result[2][2]);
void matAdd(float a[2][2], float b[2][2], float result[2][2]);
void matMul2x2_2x1(float m[2][2], float v[2], float result[2]);
void matMul1x2_2x2(float v[2], float m[2][2], float result[2]);
void matMul2x1_1x2(float a[2], float b[2], float result[2][2]);
float matMul1x2_2x1(float a[2], float b[2]);
void matIdentitySub(float a[2][2], float result[2][2]);
float calculateAltitude(float pressurePa);
