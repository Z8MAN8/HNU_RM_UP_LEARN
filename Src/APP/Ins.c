//
// Created by HLiamso on 2022-03-13.
//

#include "Ins.h"
#include "cmsis_os.h"
#include "QuaternionAHRS.h"
#include "QuaternionEKF.h"
#include "tim.h"
#include "BMI088driver.h"
#include "controller.h"
#include "usart.h"
#include "bsp_PWM.h"
#include "Gimbal.h"

InsTypeDef ins;
InsTxTypeDef ins_tx;
ImuParamTypeDef IMU_param;
QuaternionBufTypeDef Quaternion_buffer;
PIDTypeDef TempCtrl = {0};
static uint8_t ins_tx_buffer[100];
uint8_t ins_tx_size;
const float xb[3] = {1, 0, 0};
const float yb[3] = {0, 1, 0};
const float zb[3] = {0, 0, 1};

uint32_t INS_DWT_Count = 0;
static float dt = 0, t = 0;
uint8_t ins_debug_mode = 0;
float RefTemp = 40;
float Testdata[6]={0};
char Vofatail[4] = {0x00, 0x00, 0x80, 0x7f};
extern ImuTypeDef       imu;
static void IMU_Param_Correction(ImuParamTypeDef *param, float gyro[3], float accel[3]);

void ins_task(void const * argument)
{
    /* USER CODE BEGIN InsTask */
    BMI088_Read(&BMI088);

    if (fabsf(sqrtf(BMI088.accel[0] * BMI088.accel[0] +
                    BMI088.accel[1] * BMI088.accel[1] +
                    BMI088.accel[2] * BMI088.accel[2]) -
              BMI088.g_norm) < 1)
        Quaternion_AHRS_InitIMU(BMI088.accel[0], BMI088.accel[1], BMI088.accel[2], BMI088.g_norm);
    IMU_param.scale[0] = 1;
    IMU_param.scale[1] = 1;
    IMU_param.scale[2] = 1;
    IMU_param.yaw = 0;
    IMU_param.pitch = 0;
    IMU_param.roll = 0;
    IMU_param.flag = 1;

    IMU_QuaternionEKF_Init(10, 0.001, 1000000 * 10, 0.9996 * 0 + 1, 0);
    // imu heat init
    PID_Init_Plus(&TempCtrl, 2000, 300, 0, 1000, 20, 0, 0, 0, 0, 0, 0, 0);
    HAL_TIM_PWM_Start(&htim10, TIM_CHANNEL_1);
    uint32_t ins_wake_time = osKernelSysTick();
    /* Infinite loop */
    for(;;)
    {
        HAL_GPIO_WritePin(GPIOE, task1_Pin, GPIO_PIN_SET);
        static uint32_t count = 0;
        dt = DWT_GetDeltaT(&INS_DWT_Count);
        t += dt;

        // ins update
        if ((count % 1) == 0)
        {
            BMI088_Read(&BMI088);
            ins.accel[0] = BMI088.accel[0];
            ins.accel[1] = BMI088.accel[1];
            ins.accel[2] = BMI088.accel[2];
            ins.gyro[0] = BMI088.gyro[0];
            ins.gyro[1] = BMI088.gyro[1];
            ins.gyro[2] = BMI088.gyro[2];
            IMU_Param_Correction(&IMU_param, ins.gyro, ins.accel);
            ins.atanxz = -atan2f(ins.accel[0], ins.accel[2]) * 180 / PI;
            ins.atanyz = atan2f(ins.accel[1], ins.accel[2]) * 180 / PI;

            Quaternion_AHRS_UpdateIMU(ins.gyro[0], ins.gyro[1], ins.gyro[2], ins.accel[0], ins.accel[1], ins.accel[2], 0, 0, 0, dt);
            IMU_QuaternionEKF_Update(ins.gyro[0], ins.gyro[1], ins.gyro[2], ins.accel[0], ins.accel[1], ins.accel[2], dt);

            // BodyFrameToEarthFrame(xb, INS.xn, INS.q);
            // BodyFrameToEarthFrame(yb, INS.yn, INS.q);
            // BodyFrameToEarthFrame(zb, INS.zn, INS.q);

            memcpy(ins.gyro, QEKF_INS.Gyro, sizeof(QEKF_INS.Gyro));
            memcpy(ins.accel, QEKF_INS.Accel, sizeof(QEKF_INS.Accel));
            memcpy(ins.q, QEKF_INS.q, sizeof(QEKF_INS.q));
            ins.yaw = QEKF_INS.Yaw;
            ins.pitch = QEKF_INS.Pitch;
            ins.roll = QEKF_INS.Roll;
            ins.yaw_total_angle = QEKF_INS.YawTotalAngle;

            memcpy(ins_tx.Gyro, ins.gyro, sizeof(ins.gyro));
            memcpy(ins_tx.Accel, ins.accel, sizeof(ins.accel));
            memcpy(ins_tx.q, ins.q, sizeof(ins.q));

            memcpy(ins_tx_buffer, &ins_tx, sizeof(ins_tx));
            ins_tx_size = sizeof(ins_tx);
            imu.acc_x = ins.accel[0];
            imu.acc_y = ins.accel[1];
            imu.acc_z = ins.accel[2];
            imu.angle_x = ins.yaw_total_angle;
            imu.angle_y = ins.roll;
            imu.angle_z = ins.pitch;
            imu.gyro_x = ins.gyro[0];
            imu.gyro_y = ins.gyro[1];
            imu.gyro_z = ins.gyro[2];

//            HAL_UART_Transmit(&huart6, ins_tx_buffer, sizeof(ins_tx), 100);

//        if (GlobalDebugMode == INS_DEBUG)
//        {
//            if (ins_debug_mode == 0)
//                Serial_Debug(&huart1, 1, AHRS.yaw, AHRS.pitch, AHRS.roll, INS.yaw, INS.pitch, INS.roll);
//
//        }
        }

        // temperature control
        if ((count % 2) == 0)
        {
            // 500hz
            IMU_Temperature_Ctrl();
//            HAL_UART_Transmit(&huart1,(uint8_t *)&Testdata,sizeof(Testdata),0xFFFFFFFFU);
//            HAL_UART_Transmit(&huart1, (uint8_t *)&Vofatail, sizeof(Vofatail),0xFFFFFFFFU);
//            if (GlobalDebugMode == IMU_HEAT_DEBUG)
//                Serial_Debug(&huart1, 1, RefTemp, BMI088.temperature, TempCtrl.Output / 1000.0f, TempCtrl.Pout / 1000.0f, TempCtrl.Iout / 1000.0f, TempCtrl.Dout / 1000.0f);
//
        }

        if ((count % 100) == 0)
        {
        }

        count++;
        HAL_GPIO_WritePin(GPIOE, task1_Pin, GPIO_PIN_RESET);
        vTaskDelayUntil(&ins_wake_time,1);
    }
    /* USER CODE END InsTask */
}

/**
 * @brief        Update quaternion
 */
void QuaternionUpdate(float *q, float gx, float gy, float gz, float dt)
{
    float qa, qb, qc;

    gx *= 0.5f * dt;
    gy *= 0.5f * dt;
    gz *= 0.5f * dt;
    qa = q[0];
    qb = q[1];
    qc = q[2];
    q[0] += (-qb * gx - qc * gy - q[3] * gz);
    q[1] += (qa * gx + qc * gz - q[3] * gy);
    q[2] += (qa * gy - qb * gz + q[3] * gx);
    q[3] += (qa * gz + qb * gy - qc * gx);
}

/**
 * @brief        Convert quaternion to eular angle
 */
void QuaternionToEularAngle(float *q, float *Yaw, float *Pitch, float *Roll)
{
    *Yaw = atan2f(2.0f * (q[0] * q[3] + q[1] * q[2]), 2.0f * (q[0] * q[0] + q[1] * q[1]) - 1.0f) * 57.295779513f;
    *Pitch = atan2f(2.0f * (q[0] * q[1] + q[2] * q[3]), 2.0f * (q[0] * q[0] + q[3] * q[3]) - 1.0f) * 57.295779513f;
    *Roll = asinf(2.0f * (q[0] * q[2] - q[1] * q[3])) * 57.295779513f;
}

/**
 * @brief        Convert eular angle to quaternion
 */
void EularAngleToQuaternion(float Yaw, float Pitch, float Roll, float *q)
{
    float cosPitch, cosYaw, cosRoll, sinPitch, sinYaw, sinRoll;
    Yaw /= 57.295779513f;
    Pitch /= 57.295779513f;
    Roll /= 57.295779513f;
    cosPitch = arm_cos_f32(Pitch / 2);
    cosYaw = arm_cos_f32(Yaw / 2);
    cosRoll = arm_cos_f32(Roll / 2);
    sinPitch = arm_sin_f32(Pitch / 2);
    sinYaw = arm_sin_f32(Yaw / 2);
    sinRoll = arm_sin_f32(Roll / 2);
    q[0] = cosPitch * cosRoll * cosYaw + sinPitch * sinRoll * sinYaw;
    q[1] = sinPitch * cosRoll * cosYaw - cosPitch * sinRoll * sinYaw;
    q[2] = sinPitch * cosRoll * sinYaw + cosPitch * sinRoll * cosYaw;
    q[3] = cosPitch * cosRoll * sinYaw - sinPitch * sinRoll * cosYaw;
}

void InsertQuaternionFrame(QuaternionBufTypeDef *qBuf, float *q, float time_stamp)
{
    if (qBuf->LatestNum == Q_FRAME_LEN - 1)
        qBuf->LatestNum = 0;
    else
        qBuf->LatestNum++;

    qBuf->qFrame[qBuf->LatestNum].TimeStamp = time_stamp;
    for (uint16_t i = 0; i < 4; i++)
        qBuf->qFrame[qBuf->LatestNum].q[i] = q[i];
}

uint16_t FindTimeMatchFrame(QuaternionBufTypeDef *qBuf, float match_time_stamp)
{
    float min_time_error = fabsf(qBuf->qFrame[0].TimeStamp - match_time_stamp);
    uint16_t num = 0;
    for (uint16_t i = 0; i < Q_FRAME_LEN; i++)
    {
        if (fabsf(qBuf->qFrame[i].TimeStamp - match_time_stamp) < min_time_error)
        {
            min_time_error = fabsf(qBuf->qFrame[i].TimeStamp - match_time_stamp);
            num = i;
        }
    }
    return num;
}

/**
 * @brief          Transform 3dvector from BodyFrame to EarthFrame
 * @param[1]       vector in BodyFrame
 * @param[2]       vector in EarthFrame
 * @param[3]       quaternion
 */
void BodyFrameToEarthFrame(const float *vecBF, float *vecEF, float *q)
{
    vecEF[0] = 2.0f * ((0.5f - q[2] * q[2] - q[3] * q[3]) * vecBF[0] +
                       (q[1] * q[2] - q[0] * q[3]) * vecBF[1] +
                       (q[1] * q[3] + q[0] * q[2]) * vecBF[2]);

    vecEF[1] = 2.0f * ((q[1] * q[2] + q[0] * q[3]) * vecBF[0] +
                       (0.5f - q[1] * q[1] - q[3] * q[3]) * vecBF[1] +
                       (q[2] * q[3] - q[0] * q[1]) * vecBF[2]);

    vecEF[2] = 2.0f * ((q[1] * q[3] - q[0] * q[2]) * vecBF[0] +
                       (q[2] * q[3] + q[0] * q[1]) * vecBF[1] +
                       (0.5f - q[1] * q[1] - q[2] * q[2]) * vecBF[2]);
}

/**
 * @brief          Transform 3dvector from EarthFrame to BodyFrame
 * @param[1]       vector in EarthFrame
 * @param[2]       vector in BodyFrame
 * @param[3]       quaternion
 */
void EarthFrameToBodyFrame(const float *vecEF, float *vecBF, float *q)
{
    vecBF[0] = 2.0f * ((0.5f - q[2] * q[2] - q[3] * q[3]) * vecEF[0] +
                       (q[1] * q[2] + q[0] * q[3]) * vecEF[1] +
                       (q[1] * q[3] - q[0] * q[2]) * vecEF[2]);

    vecBF[1] = 2.0f * ((q[1] * q[2] - q[0] * q[3]) * vecEF[0] +
                       (0.5f - q[1] * q[1] - q[3] * q[3]) * vecEF[1] +
                       (q[2] * q[3] + q[0] * q[1]) * vecEF[2]);

    vecBF[2] = 2.0f * ((q[1] * q[3] + q[0] * q[2]) * vecEF[0] +
                       (q[2] * q[3] - q[0] * q[1]) * vecEF[1] +
                       (0.5f - q[1] * q[1] - q[2] * q[2]) * vecEF[2]);
}

static void IMU_Param_Correction(ImuParamTypeDef *param, float gyro[3], float accel[3])
{
    static float lastYawOffset, lastPitchOffset, lastRollOffset;
    static float c_11, c_12, c_13, c_21, c_22, c_23, c_31, c_32, c_33;
    float cosPitch, cosYaw, cosRoll, sinPitch, sinYaw, sinRoll;

    if (fabsf(param->yaw - lastYawOffset) > 0.001f ||
        fabsf(param->pitch - lastPitchOffset) > 0.001f ||
        fabsf(param->roll - lastRollOffset) > 0.001f || param->flag)
    {
        cosYaw = arm_cos_f32(param->yaw / 57.295779513f);
        cosPitch = arm_cos_f32(param->pitch / 57.295779513f);
        cosRoll = arm_cos_f32(param->roll / 57.295779513f);
        sinYaw = arm_sin_f32(param->yaw / 57.295779513f);
        sinPitch = arm_sin_f32(param->pitch / 57.295779513f);
        sinRoll = arm_sin_f32(param->roll / 57.295779513f);

        // 1.yaw(alpha) 2.pitch(beta) 3.roll(gamma)
        c_11 = cosYaw * cosRoll + sinYaw * sinPitch * sinRoll;
        c_12 = cosPitch * sinYaw;
        c_13 = cosYaw * sinRoll - cosRoll * sinYaw * sinPitch;
        c_21 = cosYaw * sinPitch * sinRoll - cosRoll * sinYaw;
        c_22 = cosYaw * cosPitch;
        c_23 = -sinYaw * sinRoll - cosYaw * cosRoll * sinPitch;
        c_31 = -cosPitch * sinRoll;
        c_32 = sinPitch;
        c_33 = cosPitch * cosRoll;
        param->flag = 0;
    }
    float gyro_temp[3];
    for (uint8_t i = 0; i < 3; i++)
        gyro_temp[i] = gyro[i] * param->scale[i];

    gyro[0] = c_11 * gyro_temp[0] +
              c_12 * gyro_temp[1] +
              c_13 * gyro_temp[2];
    gyro[1] = c_21 * gyro_temp[0] +
              c_22 * gyro_temp[1] +
              c_23 * gyro_temp[2];
    gyro[2] = c_31 * gyro_temp[0] +
              c_32 * gyro_temp[1] +
              c_33 * gyro_temp[2];

    float accel_temp[3];
    for (uint8_t i = 0; i < 3; i++)
        accel_temp[i] = accel[i];

    accel[0] = c_11 * accel_temp[0] +
               c_12 * accel_temp[1] +
               c_13 * accel_temp[2];
    accel[1] = c_21 * accel_temp[0] +
               c_22 * accel_temp[1] +
               c_23 * accel_temp[2];
    accel[2] = c_31 * accel_temp[0] +
               c_32 * accel_temp[1] +
               c_33 * accel_temp[2];

    lastYawOffset = param->yaw;
    lastPitchOffset = param->pitch;
    lastRollOffset = param->roll;
}

void IMU_Temperature_Ctrl(void)
{
    PID_Calculate(&TempCtrl, BMI088.temperature, float_constrain(BMI088.temp_when_cali, 37, 42));

    TIM_Set_PWM(&htim10, TIM_CHANNEL_1, float_constrain(float_rounding(TempCtrl.Output), 0, UINT32_MAX));
}