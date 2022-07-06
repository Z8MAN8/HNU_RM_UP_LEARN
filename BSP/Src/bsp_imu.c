//
// Created by turboDog on 2021/11/27.
//

#include "bsp_imu.h"
#include "Ins.h"


/**
  * @brief 获取IMU相关数据.
  * @param imu_data  存储IMU数据的结构体变量.
  */
void Imu_Get_data(ImuTypeDef *imu_data){

    imu_data->acc_x = ins.accel[0];
    imu_data->acc_y = ins.accel[1];
    imu_data->acc_z = ins.accel[2];
    imu_data->angle_x = ins.yaw_total_angle;
    imu_data->angle_y = ins.roll;
    imu_data->angle_z = ins.pitch;
    imu_data->gyro_x = ins.gyro[0];
    imu_data->gyro_y = ins.gyro[1];
    imu_data->gyro_z = ins.gyro[2];
}