/**
 ******************************************************************************
 * @file    GravityEstimateKF.h
 * @author  Hongxi Wong
 * @version V1.0.1
 * @date    2020/2/16
 * @brief
 ******************************************************************************
 * @attention
 *
 ******************************************************************************
 */
#ifndef _gEstimateKF_H
#define _gEstimateKF_H
#include "kalman_filter.h"

/* boolean type definitions */
#ifndef TRUE
#define TRUE 1 /**< boolean true  */
#endif

#ifndef FALSE
#define FALSE 0 /**< boolean fails */
#endif

extern float gVec[3];
extern float gEstimateKF_P[9];

void gEstimateKF_Init(float process_noise, float measure_noise);
void gEstimateKF_Update(float gx, float gy, float gz, float ax, float ay, float az, float dt);
void gEstimateKF_SetQR(float process_noise, float measure_noise);

#endif
