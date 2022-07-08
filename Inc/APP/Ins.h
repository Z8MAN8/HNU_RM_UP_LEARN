//
// Created by HLiamso on 2022-03-13.
//

#ifndef Ins
#define Ins

#include "stdint.h"



typedef struct
{
    float q[4];

    float gyro[3];
    float accel[3];

    float xn[3];
    float yn[3];
    float zn[3];

    float atanxz;
    float atanyz;

    float roll;
    float pitch;
    float yaw;
    float yaw_total_angle;
} InsTypeDef;

typedef struct
{
    float q[4];

    float Gyro[3];
    float Accel[3];
} InsTxTypeDef;

typedef struct
{
    float q[4];
    float TimeStamp;
} QuaternionFrameTypeDef;

#define Q_FRAME_LEN 50
typedef struct
{
    QuaternionFrameTypeDef qFrame[Q_FRAME_LEN];
    uint16_t LatestNum;
} QuaternionBufTypeDef;

typedef struct
{
    uint8_t flag;

    float scale[3];

    float yaw;
    float pitch;
    float roll;
} ImuParamTypeDef;

/**
  * @brief     IMU 数据结构体
  */
typedef struct
{
    float acc_x;   //m/s^2
    float acc_y;   //m/s^2
    float acc_z;   //m/s^2
    float gyro_x;  //degree/s
    float gyro_y;  //degree/s
    float gyro_z;  //degree/s
    float angle_x; //degree
    float angle_y; //degree
    float angle_z; //degree
} ImuTypeDef;


extern InsTypeDef ins;
extern float RefTemp;
extern QuaternionBufTypeDef Quaternion_buffer;

/**
  * @brief 获取IMU相关数据.
  * @param imu_data  存储IMU数据的结构体变量.
  */
void IMU_Get_data(ImuTypeDef *imu_data);

void INS_Init(void);
void IMU_Temperature_Ctrl(void);

void QuaternionUpdate(float *q, float gx, float gy, float gz, float dt);
void QuaternionToEularAngle(float *q, float *Yaw, float *Pitch, float *Roll);
void EularAngleToQuaternion(float Yaw, float Pitch, float Roll, float *q);
void InsertQuaternionFrame(QuaternionBufTypeDef *qBuf, float *q, float time_stamp);
uint16_t FindTimeMatchFrame(QuaternionBufTypeDef *qBuf, float match_time_stamp);
void BodyFrameToEarthFrame(const float *vecBF, float *vecEF, float *q);
void EarthFrameToBodyFrame(const float *vecEF, float *vecBF, float *q);

#endif //Ins
