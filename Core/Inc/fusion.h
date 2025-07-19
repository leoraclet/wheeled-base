#ifndef INC_FUSION_H_
#define INC_FUSION_H_


#include <math.h>


void MadgwickQuaternionUpdate6(float ax, float ay, float az, float gx, float gy, float gz);
void QuartenionToAngles(float *yaw, float *pitch, float *roll);
void MadgwickQuaternionUpdate9(float ax, float ay, float az, float gx, float gy, float gz, float mx, float my, float mz);
void MahonyQuaternionUpdate9(float ax, float ay, float az, float gx, float gy, float gz, float mx, float my, float mz);
void QuaternionToYawPitchRoll(float *data);


#endif /* INC_FUSION_H_ */
