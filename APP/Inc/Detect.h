//
// Created by 14685 on 2022/7/3.
//

#ifndef HNU_RM_UP_DETECT_H
#define HNU_RM_UP_DETECT_H

#endif //HNU_RM_UP_DETECT_H

#include "stdint.h"
typedef enum
{
    DEVICE_NORMAL = 0,
    CHASSIS_M1_OFFLINE,
    CHASSIS_M2_OFFLINE,
    CHASSIS_M3_OFFLINE,
    CHASSIS_M4_OFFLINE,
    REMOTE_CTRL_OFFLINE,
    GIMBAL_YAW_OFFLINE,
    GIMBAL_PIT_OFFLINE,
    AMMO_BOOSTER1_OFFLINE,
    AMMO_BOOSTER2_OFFLINE,
    TRIGGER_MOTO_OFFLINE,
    ERROR_LIST_LENGTH,
    //imu
    BOARD_ACCEL_TOE,
    BOARD_GYRO_TOE,
    BOARD_MAG_TOE
} ErrIDType;
typedef struct
{
    volatile uint32_t last_time;
    volatile uint32_t err_exist : 1;   //1 = err_exist, 0 = everything ok
    volatile uint32_t enable : 1;
    volatile uint32_t warn_pri : 6;    //priority
    volatile uint32_t delta_time : 16; //time interval last
    volatile uint32_t set_timeout : 16;
} __attribute__((__packed__)) OfflineDevTypeDef;
typedef struct
{
    volatile OfflineDevTypeDef *err_now;
    volatile OfflineDevTypeDef  err_list[ERROR_LIST_LENGTH];
    ErrIDType err_id;
} __attribute__((__packed__)) GlbErrTypeDef;
void GlobalErr_Detector_init(void);
void Err_Detector_hook(int err_id);
void detect_task(const void* argu);

void Module_Offline_callback(void);

extern GlbErrTypeDef glb_err;