//
// Created by 14685 on 2022/7/3.
//

#ifndef HNU_RM_UP_SHOOT_H
#define HNU_RM_UP_SHOOT_H

#include "sys.h"
#include "cmsis_os.h"
#include "motor.h"
#include "Gimbal.h"

/*************************发射速度设置*******************************/
#define SHOT_FRIC_WHEEL_SPEED    6000
#define SHOT_SUCCESS_FRIC_WHEEL_SPEED  6800  //发射成功摩擦轮会减速
#define SHOT_ABLE_FRIC_WHEEL_SPEED  6000
#define SHOOT_PERIOD 2
/* 单发拨弹的编码器行程 */
#define DEGREE_60_TO_ENCODER  49146
#define DEGREE_45_TO_ENCODER -36864

#define TRIGGER_MOTOR_REDUCTION_RATIO 36
#define BULLETS_PER_ROUND 8

/*************************发射频率设置*******************************/
#define TRIGGER_MOTOR_SPEED      1500


//遥控器单发
#define RC_SINGLE_TRIG   ((last_sw1 != RC_MI) && (rc.sw1 == RC_MI))           // ((last_wheel_value != 660) && (rc.wheel == 660))
//遥控器连发
#define RC_CONTIN_TRIG   ((rc.sw1 == RC_MI) && (HAL_GetTick() - shoot_continue_time >= 1500))   //((rc.wheel == 660) && (HAL_GetTick() - continue_shoot_time >= 1000))

/**
 * @brief 射击状态枚举
 */
enum ShootState
{
    SINGLE_SHOOT,          /*! 单发射击    */
    TRIBLE_SHOOT,          /*! 三连发射击  */
    CONTINUOUS_SHOOT,      /*! 连续射击   */
    DONT_SHOOT,            /*! 不射击    */
};

/**
 * @brief           控制弹仓盖的闭合
 */
void Cap_Control_open(void);

/**
 * @brief           子弹的单发和连发处理
 */
void Shoot_Custom_control(void);

/**
 * @brief           开关摩擦轮处理
 */
void FrictionWheel_Turn_on_off(void);

/**
  * @brief          根据期望射频计算拨弹电机速度，可实现以任意间隔任意射频发射任意数量子弹
  * @param[1]       射频，单位：发/s
  * @param[2]       单次射击子弹数
  * @param[3]       两次射击间隔时间
  * @retval         拨弹电机期望速度 单位：RPM
  */
float ShootAndDelay(float speedInNumsPerSec, uint32_t numsOfOneShot, uint32_t delayTimeInMs);


extern uint8_t  shooter_output; //裁判系统对SHOOT的供电情况
/*extern PIDTypeDef pid_trigger
extern PIDTypeDef pid_trigger_speed
extern PIDTypeDef pid_shoot_left
extern PIDTypeDef pid_shoot_right*/

#endif //HNU_RM_UP_SHOOT_H
