//
// Created by HLiamso on 2022-03-13.
//

#ifndef Ins
#define Ins

#include "stdint.h"



typedef struct
{
    float q[4];

    float Gyro[3];
    float Accel[3];

    float xn[3];
    float yn[3];
    float zn[3];

    float atanxz;
    float atanyz;

    float Roll;
    float Pitch;
    float Yaw;
    float YawTotalAngle;
} Ins_T;

typedef struct
{
    float q[4];

    float Gyro[3];
    float Accel[3];
} INS_Tx_t;

typedef struct
{
    float q[4];
    float TimeStamp;
} QuaternionFrame_t;

#define Q_FRAME_LEN 50
typedef struct
{
    QuaternionFrame_t qFrame[Q_FRAME_LEN];
    uint16_t LatestNum;
} QuaternionBuf_t;

typedef struct
{
    uint8_t flag;

    float scale[3];

    float Yaw;
    float Pitch;
    float Roll;
} IMU_Param_t;

extern Ins_T ins;
extern float RefTemp;
extern QuaternionBuf_t QuaternionBuffer;

void INS_Init(void);
void IMU_Temperature_Ctrl(void);

void QuaternionUpdate(float *q, float gx, float gy, float gz, float dt);
void QuaternionToEularAngle(float *q, float *Yaw, float *Pitch, float *Roll);
void EularAngleToQuaternion(float Yaw, float Pitch, float Roll, float *q);
void InsertQuaternionFrame(QuaternionBuf_t *qBuf, float *q, float time_stamp);
uint16_t FindTimeMatchFrame(QuaternionBuf_t *qBuf, float match_time_stamp);
void BodyFrameToEarthFrame(const float *vecBF, float *vecEF, float *q);
void EarthFrameToBodyFrame(const float *vecEF, float *vecBF, float *q);

#endif //Ins
