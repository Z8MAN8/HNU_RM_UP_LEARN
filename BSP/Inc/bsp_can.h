//
// Created by HLiamso on 2021-11-14.
//

#ifndef BSP_CAN_H
#define BSP_CAN_H

#include <motor.h>
#include "Transmission.h"
#include "stm32f4xx_hal_def.h"

#define CAN_RC_DATA_Frame_0 0x131
#define CAN_RC_DATA_Frame_1 0x132
#define COM_CAN hcan2
#define CONTROL_CAN hcan1
#define FILTER_BUF 5

static CAN_TxHeaderTypeDef  tx_message;
static uint8_t              can_send_data[8];
extern MotorTypeDef chassis_motor[4];
//TODO
//extern chassis_mode_e chassis_mode;

#ifdef TEST_ON_ICRA
typedef enum
{
    //接收ID
    CAN_3508_M1_ID       = 0x201,//拨弹电机
    CAN_3508_M2_ID       = 0x202,//左摩擦轮
    CAN_3508_M3_ID       = 0x203,//右摩擦轮
    CAN_3508_M4_ID       = 0x204,
    CAN_PIT_MOTOR_ID     = 0x206,//ID 2 010
    CAN_YAW_MOTOR_ID     = 0x205,
    //CAN_TRIGGER_MOTOR_ID = 0x207,
    CAN_SUPERCAP_RECV    = 0x211,
    //发送ID
    CAN_CHASSIS_ID       = 0x200,
    CAN_SUPER_CAP_ID      = 0X210,
    CAN_GIMBAL_ID = 0x1ff,
    CAN_GIMBAL_ID_PITCH        = 0x1ff,
    CAN_GIMBAL_ID_YAW          = 0x2ff,

} CANMsgIDType;
#else
typedef enum
{
    //接收ID
    CAN_3508_M1_ID       = 0x201,//拨弹电机
    CAN_3508_M2_ID       = 0x202,//左摩擦轮
    CAN_3508_M3_ID       = 0x203,//右摩擦轮
    CAN_3508_M4_ID       = 0x204,
    CAN_YAW_MOTOR_ID     = 0x205,//ID 1 001
    CAN_PIT_MOTOR_ID     = 0x206,//ID 2 010
    CAN_TRIGGER_MOTOR_ID = 0x207,
    CAN_SUPERCAP_RECV    = 0x211,
    //发送ID
    CAN_CHASSIS_ID       = 0x200,
    CAN_SUPER_CAP_ID      = 0X210,
    CAN_GIMBAL_ID        = 0x1ff,

} CANMsgIDType;
#endif
void CAN_Device_Init(void);
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan);

/**
  * @brief     发送 CAN 数据
  * @param     can_id: CAN 设备 ID，只有 CAN1 或者 CAN2
  * @param     send_id: 发送数据 ID
  * @param     send_data: 发送数据指针，大小为 8 位
  */
void Write_CAN(CAN_HandleTypeDef can, uint32_t send_id, uint8_t send_data[]);


/**
  * @brief     电机参数结构体
  */
typedef struct
{
    /* 以下是电机电调直接回传的数据 */

    uint16_t ecd;         //电机的编码器数值
    uint16_t last_ecd;    //上一次电机的编码器数值
    int16_t  speed_rpm;   //电机的转速值

    /* 以下是计算出来的电机相关数据 */
    int32_t  round_cnt;   //电机旋转的总圈数
    int32_t  total_ecd;   //电机旋转的总编码器数值
    int32_t  total_angle; //电机旋转的总角度

    /* 以下电机计算相关数据时的中间变量，可以忽略 */
    uint16_t offset_ecd;
    uint32_t msg_cnt;
    int32_t  ecd_raw_rate;
    int32_t  rate_buf[FILTER_BUF];
    uint8_t  buf_cut;
    int32_t  filter_rate;
} MotoMeasureTypeDef;


extern MotorTypeDef chassis_motor[];
extern MotorTypeDef YawMotor;
extern MotorTypeDef PitMotor;
extern MotoMeasureTypeDef moto_trigger;
extern MotoMeasureTypeDef moto_test;
extern MotoMeasureTypeDef moto_shoot[];//0左，1右；
extern float PowerData[4];
//extern int shoot_status;
//extern int shoot_cnt;
//extern int last_cnt;
/**
  * @brief     CAN1 中断回调函数，在程序初始化时注册
  * @param     recv_id: CAN1 接收到的数据 ID
  * @param     data: 接收到的 CAN1 数据指针
  */
void can1_recv_callback(uint32_t recv_id, uint8_t data[]);
/**
  * @brief     CAN2 中断回调函数，在程序初始化时注册
  * @param     recv_id: CAN2 接收到的数据 ID
  * @param     data: 接收到的 CAN2 数据指针
  */
void can2_recv_callback(uint32_t recv_id, uint8_t data[]);

/**
  * @brief     计算电机的转速rmp 圈数round_cnt
  *            总编码器数值total_ecd 总旋转的角度total_angle
  * @param     ptr: 电机参数 MotoMeasureTypeDef 结构体指针
  * @param     data: 接收到的电机 CAN 数据指针
  */
static void Encoder_Data_handle(MotoMeasureTypeDef *ptr, uint8_t data[]);

/**
  * @brief     获得电机初始偏差
  * @param     ptr: 电机参数 MotoMeasureTypeDef 结构体指针
  * @param     data: 接收到的电机 CAN 数据指针
  */
static void Get_Moto_offset(MotoMeasureTypeDef *ptr, uint8_t data[]);

/**
  * @brief     发送底盘电机电流数据到电调
  */
void send_chassis_moto_current(int16_t current[]);
void send_chassis_moto_zero_current(void);

/**
  * @brief     发送云台电机电流数据到电调
  */
void GimbalMoto_Send_current(int16_t yaw_current, int16_t pit_current);
void GimbalMoto_Send_zero_current(void);

/**
  * @brief     发送云台电机电流数据到电调
  */
void ShootMoto_Send_current(int16_t left_current, int16_t right_current, int16_t pit_current);
void sendSuperCap(void);

/**
  * @brief     发送功率信息到电容管理板
 */
void PowerDataResolve(uint8_t data[]);


#endif //BSP_CAN_H