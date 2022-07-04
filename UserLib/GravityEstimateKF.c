/**
  ******************************************************************************
  * @file    GravityEstimateKF.c
  * @author  Hongxi Wong
  * @version V1.0.1
  * @date    2021/6/19
  * @brief
  ******************************************************************************
  * @attention
  *
  ******************************************************************************
  */
#include "GravityEstimateKF.h"

KalmanFilter_t gEstimateKF;
float gVec[3];

float gEstimateKF_F[9] = {1, 0, 0,
                          0, 1, 0,
                          0, 0, 1};
float gEstimateKF_P[9] = {10000, 0.1, 0.1,
                          0.1, 10000, 0.1,
                          0.1, 0.1, 10000};
static float gEstimateKF_Q[9] = {0.01, 0, 0,
                                 0, 0.01, 0,
                                 0, 0, 0.01};
static float gEstimateKF_R[9] = {1000, 0, 0,
                                 0, 1000, 0,
                                 0, 0, 1000};
float gEstimateKF_K[9];
const float gEstimateKF_H[9] = {1, 0, 0,
                                0, 1, 0,
                                0, 0, 1};

static void gEstimateKF_Tuning(KalmanFilter_t *kf);

void gEstimateKF_Init(float process_noise, float measure_noise)
{
    for (uint8_t i = 0; i < 9; i += 4)
    {
        gEstimateKF_Q[i] = process_noise;
        gEstimateKF_R[i] = measure_noise;
    }

    Kalman_Filter_Init(&gEstimateKF, 3, 0, 3);
    gEstimateKF.User_Func0_f = gEstimateKF_Tuning;
    memcpy(gEstimateKF.F_data, gEstimateKF_F, sizeof(gEstimateKF_F));
    memcpy(gEstimateKF.P_data, gEstimateKF_P, sizeof(gEstimateKF_P));
    memcpy(gEstimateKF.Q_data, gEstimateKF_Q, sizeof(gEstimateKF_Q));
    memcpy(gEstimateKF.R_data, gEstimateKF_R, sizeof(gEstimateKF_R));
    memcpy(gEstimateKF.H_data, gEstimateKF_H, sizeof(gEstimateKF_H));
}

void gEstimateKF_Update(float gx, float gy, float gz, float ax, float ay, float az, float dt)
{
    static float gxdt, gydt, gzdt;

    gxdt = gx * dt;
    gydt = gy * dt;
    gzdt = gz * dt;

    gEstimateKF.F_data[1] = gzdt;
    gEstimateKF.F_data[2] = -gydt;

    gEstimateKF.F_data[3] = -gzdt;
    gEstimateKF.F_data[5] = gxdt;

    gEstimateKF.F_data[6] = gydt;
    gEstimateKF.F_data[7] = -gxdt;

    gEstimateKF.MeasuredVector[0] = ax;
    gEstimateKF.MeasuredVector[1] = ay;
    gEstimateKF.MeasuredVector[2] = az;

    Kalman_Filter_Update(&gEstimateKF);

    for (uint8_t i = 0; i < 3; i++)
    {
        gVec[i] = gEstimateKF.FilteredValue[i];
    }
}

void gEstimateKF_SetQR(float process_noise, float measure_noise)
{
    for (uint8_t i = 0; i < 9; i += 4)
    {
        gEstimateKF_Q[i] = process_noise;
        gEstimateKF_R[i] = measure_noise;
    }
}

static void gEstimateKF_Tuning(KalmanFilter_t *kf)
{
    memcpy(gEstimateKF_F, kf->F_data, sizeof(gEstimateKF_F));
    memcpy(gEstimateKF_P, kf->P_data, sizeof(gEstimateKF_P));
    memcpy(kf->Q_data, gEstimateKF_Q, sizeof(gEstimateKF_Q));
    memcpy(kf->R_data, gEstimateKF_R, sizeof(gEstimateKF_R));
    memcpy(gEstimateKF_K, kf->K_data, sizeof(gEstimateKF_K));
}
