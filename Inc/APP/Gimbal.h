//
// Created by 14685 on 2022/7/3.
//

#ifndef HNU_RM_UP_GIMBAL_H
#define HNU_RM_UP_GIMBAL_H

#include <user_lib.h>
#include "stm32f4xx_hal.h"
#include "bsp_uart.h"
#include "stdint.h"
#include "stdint.h"
#include "Ins.h"

/* 云台控制周期 (ms) */
#define GIMBAL_PERIOD 1
/* 云台回中初始化时间 (ms) */
#define BACK_CENTER_TIME 6000

#define YAW_MOTOR_MAXOUT 30000
#define PITCH_MOTOR_MAXOUT 30000

#define YAW_V_PID_MAXOUT_M_INIT 15000
#define YAW_V_PID_MAXOUT_M 25000
#define YAW_V_PID_MAXINTEGRAL_M 3000
#define YAW_V_PID_KP_M 500
#define YAW_V_PID_KI_M 20.0
#define YAW_V_PID_KD_M 10.0
#define YAW_V_PID_LPF_M 0.000001
#define YAW_V_PID_D_LPF_M 0.005

#define YAW_A_PID_MAXOUT_M 320
#define YAW_A_PID_MAXINTEGRAL_M 320
#define YAW_A_PID_KP_M 30
#define YAW_A_PID_KI_M 0.5
#define YAW_A_PID_KD_M 0.001

#define YAW_V_PID_MAXOUT_A 25000
#define YAW_V_PID_MAXINTEGRAL_A 3000
#define YAW_V_PID_KP_A 500
#define YAW_V_PID_KI_A 20.0
#define YAW_V_PID_KD_A 10.0
#define YAW_V_PID_LPF_A 0.000001
#define YAW_V_PID_D_LPF_A 0.005

#define YAW_A_PID_MAXOUT_A 320
#define YAW_A_PID_MAXINTEGRAL_A 320
#define YAW_A_PID_KP_A 30
#define YAW_A_PID_KI_A 200
#define YAW_A_PID_KD_A 0.001

#define YAW_V_FFC_MAXOUT_INIT 0
#define YAW_V_FFC_MAXOUT 0
#define YAW_V_FCC_C0 1000
#define YAW_V_FCC_C1 20
#define YAW_V_FCC_C2 0
#define YAW_V_FCC_LPF 0.005

#define YAW_A_FFC_MAXOUT 0
#define YAW_A_FCC_C0 0
#define YAW_A_FCC_C1 0.005
#define YAW_A_FCC_C2 0
#define YAW_A_FCC_LPF 0.0001


#define PITCH_V_PID_MAXOUT_A 16000
#define PITCH_V_PID_MAXINTEGRAL_A 2000
#define PITCH_V_PID_KP_A 16
#define PITCH_V_PID_KI_A 2.0
#define PITCH_V_PID_KD_A 0.005
#define PITCH_V_PID_LPF_A 0.000001
#define PITCH_V_PID_D_LPF_A 0.08

#define PITCH_A_PID_MAXOUT_A 3000
#define PITCH_A_PID_MAXINTEGRAL_A 600
#define PITCH_A_PID_KP_A 90.0
#define PITCH_A_PID_KI_A 180
#define PITCH_A_PID_KD_A 1.0
#define PITCH_A_PID_LPF_A 0.1
#define PITCH_A_PID_D_LPF_A 0.01

#define PITCH_V_FFC_MAXOUT_A 30000
#define PITCH_V_FCC_C0_A 25
#define PITCH_V_FCC_C1_A 100 * 0
#define PITCH_V_FCC_C2_A 0
#define PITCH_V_FCC_LPF_A 0.005

#define PITCH_A_FFC_MAXOUT_A 320
#define PITCH_A_FCC_C0_A 0
#define PITCH_A_FCC_C1_A 1
#define PITCH_A_FCC_C2_A 0
#define PITCH_A_FCC_LPF_A 0.00001

#define PITCH_V_PID_MAXOUT_INIT_M 8000
#define PITCH_V_PID_MAXOUT_M 12000
#define PITCH_V_PID_MAXINTEGRAL_M 3000
#define PITCH_V_PID_KP_M 16
#define PITCH_V_PID_KI_M 0
#define PITCH_V_PID_KD_M 0.015
#define PITCH_V_PID_LPF_M 0.000001
#define PITCH_V_PID_D_LPF_M 0.08

#define PITCH_A_PID_MAXOUT_M 320
#define PITCH_A_PID_MAXINTEGRAL_M 600
#define PITCH_A_PID_KP_M 90.0
#define PITCH_A_PID_KI_M 0
#define PITCH_A_PID_KD_M 3.5
#define PITCH_A_PID_LPF_M 0.1
#define PITCH_A_PID_D_LPF_M 0.01

#define PITCH_V_FFC_MAXOUT_M 30000
#define PITCH_V_FCC_C0_M 25
//#define PITCH_V_FCC_C0_M 0
//#define PITCH_V_FCC_C1_M 100 *_M 0
#define PITCH_V_FCC_C1_M 0
//#define PITCH_V_FCC_C2_M 0
#define PITCH_V_FCC_C2_M 0
#define PITCH_V_FCC_LPF_M 0.005

#define PITCH_A_FFC_MAXOUT_M 320
#define PITCH_A_FCC_C0_M 1
#define PITCH_A_FCC_C1_M 1
//#define PITCH_A_FCC_C1_M 0
#define PITCH_A_FCC_C2_M 0
#define PITCH_A_FCC_LPF_M 0.00001

/**
  * @brief     云台控制模式枚举
  */
typedef enum
{
    GIMBAL_INIT = 0,         //云台初始化
    GIMBAL_RELAX = 1,            //云台断电
    GIMBAL_CLOSE_LOOP_ZGYRO = 2, //云台跟随imu z轴角度
    GIMBAL_AUTO	= 3						 //云台自瞄模式
} GimbalModeType;

/**
  * @brief     云台控制信号输入状态枚举
  */
typedef enum
{
    NO_ACTION = 0,           //无控制信号输入
    IS_ACTION = 1,               //有控制信号输入
} ActionModeType;

/**
  * @brief     云台回中状态枚举
  */
typedef enum
{
    PIT_BACK_STEP,           //云台 pitch 轴回中
    YAW_BACK_STEP,           //云台 yaw 轴回中
    BACK_IS_OK,              //云台回中完毕
} GimbalBackType;

/**
  * @brief     云台控制数据结构体
  */
typedef struct
{
    GimbalModeType ctrl_mode; //云台当前控制模式
    GimbalModeType last_mode; //云台上次控制模式

    ActionModeType ac_mode;   //云台控制信号输入模式

    uint8_t  no_action_flag; //无控制信号标志
    uint32_t no_action_time; //无控制信号时间

    float ecd_offset_angle;  //云台初始编码器值
    float yaw_offset_angle;  //云台初始 yaw 轴角度 （由imu得）
    float pit_offset_angle;  //云台初始 pit 轴角度 （由imu得）
} GimbalYawTypeDef;





/**
  * @brief     云台控制任务函数
  */
void gimbal_task(const void* argu);

/**
  * @brief     云台控制参数初始化
  */
static void Gimbal_Init_param(void);

/**
  * @brief     获取云台传感器及控制信息
  */
static void Gimbal_Get_information(void);

/**
  * @brief     获取云台控制模式，具体的模式判断在云台归中初始化处理函数中
  * @retval    GIMBAL_RELAX or GIMBAL_INIT
  */
static void Gimbal_Get_mode(void);

/**
  * @brief     云台归中初始化模式处理函数
  */
static void Gimbal_Init_handle(void);

/**
  * @brief     云台跟随编码器闭环控制处理函数
  */
static void Gimbal_Loop_handle(void);

/**
  * @brief     云台自瞄控制代码接口
  */
void Gimbal_Auto_control(void);

/**
  * @brief     云台relax模式处理函数
  */
static void Gimbal_Relax_handle(void);

/**
  * @brief     云台 yaw 轴位置闭环控制
  * @retval    yaw refer angle, unit is degree.
  */
void Gimbal_Control_yaw(void);

/**
  * @brief     云台 pitch 轴位置闭环控制
  * @retval    pitch refer angle, unit is degree.
  */
void Gimbal_Control_pitch(void);

/**
  * @brief     云台闭环控制电机，发送电流
  */
void Gimbal_Control_moto(void);

/**
  * @brief     get relative position angle to center
  * @param[in] raw_ecd: gimbal motor encoder raw angle
  * @param[in] center_offset: read gimbal_cali_data from chip flash
  * @retval    relative angle, unit is degree.
  */
static int16_t Gimbal_Get_relative_pos(int16_t raw_ecd, int16_t center_offset);

/**
  * @brief     限制云台初始化归中时的相关参数
  */
static void Gimbal_Back_param(void);

/**
  * @brief     将云台的PID参数切换为手动模式
  */
void PID_Reset_manual(void);

/**
  * @brief     将云台的PID参数切换为自动模式
  */
void PID_Reset_auto(void);



extern GimbalYawTypeDef gim;

/* 云台相对（相对于归中值）角度,unit: degree （由电机编码器得）*/
extern volatile float yaw_relative_angle;
extern float pit_relative_angle;

/* 云台 PID 控制相关数据 */
extern float yaw_angle_ref;
extern float pit_angle_ref;
extern float yaw_angle_fdb;
extern float pit_angle_fdb;
extern ImuTypeDef imu;    //储存IMU传感器相关的数据

extern bool_t recv_flag;    //虚拟串口接收标志位
extern float angle_history[50];    //存放25帧历史姿态数据



#endif //HNU_RM_UP_GIMBAL_H