#ifndef INC_MATH_LIB_H_
#define INC_MATH_LIB_H_


#include <math.h>

#include "main.h"


typedef struct {
	float x;
	float y;
	float z;
} Vector3;

typedef struct {
	float data[4][4];
} Matrix4x4;

typedef struct {
	float w;
	float x;
	float y;
	float z;
} Quaternion;


// ==================================================================
// UTILITY FUNCTIONS
// ==================================================================

float pressureToHeight(float pressure)
{
	return 44330.8 * (1 - powf(pressure / 1013.25, (float)0.190263));
}

// ==================================================================
// 3D VECTOR
// ==================================================================

Vector3 vectorInit(float x, float y, float z)
{
	Vector3 newVec = {x, y, z};
	return newVec;
}

void vectorZero(Vector3 *vec)
{
	vec->x = 0;
	vec->y = 0;
	vec->z = 0;
}

Vector3 vectorAdd(Vector3 *v1, Vector3 *v2)
{
	Vector3 newVec;

	newVec.x = v1->x + v2->x;
	newVec.y = v1->y + v2->y;
	newVec.z = v1->z + v2->z;

	return newVec;
}

Vector3 vectorSub(Vector3 *v1, Vector3 *v2)
{
	Vector3 newVec;

	newVec.x = v1->x - v2->x;
	newVec.y = v1->y - v2->y;
	newVec.z = v1->z - v2->z;

	return newVec;
}

float vectorDotProduct(Vector3 *v1, Vector3 *v2)
{
	return (v1->x * v2->x) + (v1->y * v2->y) + (v1->z * v2->z);
}

Vector3 vectorCrossProduct(Vector3 *v1, Vector3 *v2)
{
	Vector3 newVec;

	newVec.x = (v1->y * v2->z) - (v1->z * v2->y);
	newVec.y = (v1->z * v2->x) - (v1->x * v2->z);
	newVec.z = (v1->x * v2->y) - (v1->y * v2->x);

	return newVec;
}

float vectorNorm(Vector3 *vec)
{
	return sqrtf(vectorDotProduct(vec, vec));
}

void vectorNormalise(Vector3 *vec)
{
	float norm = vectorNorm(vec);

	if (norm == 0)
		return;

	vec->x /= norm;
	vec->y /= norm;
	vec->z /= norm;
}

Vector3 vectorAccelToEuler(Vector3 *accel)
{
	Vector3 euler;

	vectorNormalise(accel);

	euler.x = atan2f(accel->y, accel->z);
	euler.y = -atan2f(accel->x, sqrtf(accel->y * accel->y + accel->z * accel->z));
	euler.z = 0;

	return euler;
}

// ==================================================================
// QUATERNION
// ==================================================================

Quaternion quarternionInit(float w, float x, float y, float z)
{
	Quaternion newQ = {w, x, y, z};
	return newQ;
}

void quarternionZero(Quaternion *q)
{
	q->w = 0;
	q->x = 0;
	q->y = 0;
	q->z = 0;
}

Quaternion quarternionAdd(Quaternion *q1, Quaternion *q2)
{
	Quaternion newQ;

	newQ.w = q1->w + q2->w;
	newQ.x = q1->x + q2->x;
	newQ.y = q1->y + q2->y;
	newQ.z = q1->z + q2->z;

	return newQ;
}

Quaternion quarternionSub(Quaternion *q1, Quaternion *q2)
{
	Quaternion newQ;

	newQ.w = q1->w - q2->w;
	newQ.x = q1->x - q2->x;
	newQ.y = q1->y - q2->y;
	newQ.z = q1->z - q2->z;

	return newQ;
}

void quarternionScale(Quaternion *q, float scalar)
{
	q->w *= scalar;
	q->x *= scalar;
	q->y *= scalar;
	q->z *= scalar;
}

Quaternion quarternionMult(Quaternion *q1, Quaternion *q2)
{
	Quaternion newQ;

	newQ.w = q1->w * q2->w - q1->x * q2->x - q1->y * q2->y - q1->z * q2->z;
	newQ.x = q1->w * q2->x + q1->x * q2->w + q1->y * q2->z - q1->z * q2->y;
	newQ.y = q1->w * q2->y - q1->x * q2->z + q1->y * q2->w + q1->z * q2->x;
	newQ.z = q1->w * q2->z + q1->x * q2->y - q1->y * q2->x + q1->z * q2->w;

	return newQ;
}

float quaternionNorm(Quaternion *q)
{
	return sqrtf(q->w*q->w + q->x*q->x + q->y*q->y + q->z*q->z);
}

void quaternionNormalise(Quaternion *q)
{
	float norm = quaternionNorm(q);

	if (norm == 0)
		return;

	q->w /= norm;
	q->x /= norm;
	q->y /= norm;
	q->z /= norm;
}

Vector3 quaternionToEuler(Quaternion *q)
{
	Vector3 euler;

	euler.x = atan2f(2.0 * (q->y * q->z + q->w * q->x), 1 - 2.0 * (q->x * q->x + q->y * q->y));
	euler.y = asinf(2.0 * (q->w * q->y - q->x * q->z));
	euler.z = atan2f(2.0 * (q->x * q->y + q->w * q->z), 1 - 2.0 * (q->y * q->y + q->z * q->z));

	return euler;
}

Quaternion quaternionFromEuler(Vector3 *euler)
{
	float cosX2 = cosf(euler->x / 2.0f);
	float sinX2 = sinf(euler->x / 2.0f);
	float cosY2 = cosf(euler->y / 2.0f);
	float sinY2 = sinf(euler->y / 2.0f);
	float cosZ2 = cosf(euler->z / 2.0f);
	float sinZ2 = sinf(euler->z / 2.0f);

	Quaternion newQ;

	newQ.w = cosX2 * cosY2 * cosZ2 + sinX2 * sinY2 * sinZ2;
	newQ.x = sinX2 * cosY2 * cosZ2 - cosX2 * sinY2 * sinZ2;
	newQ.y = cosX2 * sinY2 * cosZ2 + sinX2 * cosY2 * sinZ2;
	newQ.z = cosX2 * cosY2 * sinZ2 - sinX2 * sinY2 * cosZ2;

	quaternionNormalise(&newQ);

	return newQ;
}

void quarternionConjugate(Quaternion *q)
{
	q->w = q->w;
	q->x = -q->x;
	q->y = -q->y;
	q->z = -q->z;
}

Vector3 quaternionToAngleVector(float *angle, Quaternion *q)
{
	Vector3 vec;

	float halfTheta = acos(q->w);
	float sinHalfTheta = sin(halfTheta);

	if (sinHalfTheta)
	{
		vec.x = 1.0;
		vec.y = 0;
		vec.z = 0;
	}
	else
	{
		vec.x = q->x / sinHalfTheta;
		vec.y = q->x / sinHalfTheta;
		vec.z = q->x / sinHalfTheta;
	}

	*angle = 2.0 * halfTheta;
	return vec;
}

Quaternion quaternionFromAngleVector(float angle, Vector3 *vec)
{
	Quaternion q;
	float sinHalfTheta = sinf(angle / 2.0);

	q.w = cosf(angle / 2.0);
	q.x = vec->x * sinHalfTheta;
	q.y = vec->y * sinHalfTheta;
	q.z = vec->z * sinHalfTheta;

	return q;
}

Quaternion accelToQuaternion(Vector3 *accel)
{
	Vector3 z = {0, 0, 1.0};
	vectorNormalise(accel);

	float angle = acosf(vectorDotProduct(&z, accel));
	Vector3 vec = vectorCrossProduct(accel, &z);
	vectorNormalise(&vec);

	return quaternionFromAngleVector(angle, &vec);
}

// ==================================================================
// 4x4 MATRIX
// ==================================================================

void matrixFill(Matrix4x4 *mat, const float value)
{
	for (uint8_t i = 0; i < 4; ++i)
		for (uint8_t j = 0; j < 4; ++j)
			mat->data[i][j] = value;
}

Matrix4x4 matrixAdd(Matrix4x4 *mat1, Matrix4x4 *mat2)
{
	Matrix4x4 mat;

	for (uint8_t i = 0; i < 4; ++i)
		for (uint8_t j = 0; j < 4; ++j)
			mat.data[i][j] = mat1->data[i][j] + mat2->data[i][j];

	return mat;
}

Matrix4x4 matrixSub(Matrix4x4 *mat1, Matrix4x4 *mat2)
{
	Matrix4x4 mat;

	for (uint8_t i = 0; i < 4; ++i)
		for (uint8_t j = 0; j < 4; ++j)
			mat.data[i][j] = mat1->data[i][j] - mat2->data[i][j];

	return mat;
}

Matrix4x4 matrixIdentity()
{
	Matrix4x4 mat;

	matrixFill(&mat, 0);
	mat.data[0][0] = 1;
	mat.data[1][1] = 1;
	mat.data[2][2] = 1;
	mat.data[3][3] = 1;

	return mat;
}

Matrix4x4 matrixTranspose(Matrix4x4 *mat)
{
	Matrix4x4 newMat;

	for (uint8_t i = 0; i < 4; ++i)
		for (uint8_t j = 0; j < 4; ++j)
			newMat.data[j][i] = mat->data[i][j];

	return newMat;
}

Matrix4x4 matrixMulitply(Matrix4x4 *mat1, Matrix4x4 *mat2)
{
	Matrix4x4 mat;

	for (uint8_t i = 0; i < 4; ++i)
		for (uint8_t j = 0; j < 4; ++j)
			mat.data[i][j] =
					mat1->data[i][0] - mat2->data[0][j] +
					mat1->data[i][1] - mat2->data[1][j] +
					mat1->data[i][2] - mat2->data[2][j] +
					mat1->data[i][3] - mat2->data[3][j];

	return mat;
}

float matrixMinor(Matrix4x4 *mat, const uint8_t row, const uint8_t col)
{
	static int map[] = {1, 2, 3, 0, 2, 3, 0, 1, 3, 0, 1, 2};

	int *rc;
	int *cc;
	float res = 0;

	rc = map + row * 3;
	cc = map + col * 3;

	res += mat->data[rc[0]][cc[0]] * mat->data[rc[1]][cc[1]] * mat->data[rc[2]][cc[2]];
	res -= mat->data[rc[0]][cc[0]] * mat->data[rc[1]][cc[2]] * mat->data[rc[2]][cc[1]];
	res -= mat->data[rc[0]][cc[1]] * mat->data[rc[1]][cc[0]] * mat->data[rc[2]][cc[2]];
	res += mat->data[rc[0]][cc[1]] * mat->data[rc[1]][cc[2]] * mat->data[rc[2]][cc[0]];
	res += mat->data[rc[0]][cc[2]] * mat->data[rc[1]][cc[0]] * mat->data[rc[2]][cc[1]];
	res -= mat->data[rc[0]][cc[2]] * mat->data[rc[1]][cc[1]] * mat->data[rc[2]][cc[0]];

	return res;
}

float matrixDet(Matrix4x4 *mat)
{
	float det = 0;

	det += mat->data[0][0] * matrixMinor(mat, 0, 0);
	det -= mat->data[0][1] * matrixMinor(mat, 0, 1);
	det += mat->data[0][2] * matrixMinor(mat, 0, 2);
	det -= mat->data[0][3] * matrixMinor(mat, 0, 3);

	return det;
}

Matrix4x4 matrixInvert(Matrix4x4 *mat)
{
	Matrix4x4 newMat;
	float det = matrixDet(mat);

	if (det == 0)
	{
		matrixIdentity(newMat);
		return newMat;
	}

	for (uint8_t i = 0; i < 4; ++i) {
		for (uint8_t j = 0; j < 4; ++j) {
			if ((i + j) & 1)
				newMat.data[i][j] = -matrixMinor(mat, i, j) / det;
			else
				newMat.data[i][j] = matrixMinor(mat, i, j) / det;
		}
	}

	return newMat;
}

// ==================================================================
// ==================================================================


#endif /* INC_MATH_LIB_H_ */
