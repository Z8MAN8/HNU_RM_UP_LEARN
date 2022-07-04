/**
  ******************************************************************************
  * @file    QuaternionAHRS.h
  * @author  Wang Hongxi
  * @version V1.2.3
  * @date    2021/6/22
  * @brief
  ******************************************************************************
  * @attention
  *
  ******************************************************************************
  */
#ifndef QUAT_AHRS_H
#define QUAT_AHRS_H

#include "stdint.h"

// algorithm parameter ---------------------------------------------------------------
#define twoKpDef (2.0f * 0.25f) // 2 * proportional gain
//------------------------------------------------------------------------------------

typedef struct
{
    float q[4];

    float Accel[3];

    float Gyro[3];

    float Yaw;
    float Pitch;
    float Roll;

    float YawTotalAngle;
    float PitchTotalAngle;
} AHRS_t;

extern float twoKp;                   // 2 * proportional gain
extern volatile float q0, q1, q2, q3; // quaternion of sensor frame relative to auxiliary frame
extern AHRS_t AHRS;

void Quaternion_AHRS_InitIMU(float ax, float ay, float az, float ref_gNorm);
void Quaternion_AHRS_Update(float gx, float gy, float gz, float ax, float ay, float az, float mx, float my, float mz, float dt);
void Quaternion_AHRS_UpdateIMU(float gx, float gy, float gz, float ax, float ay, float az, float gVecx, float gVecy, float gVecz, float dt);
void Get_EulerAngle(float *q);

#endif
