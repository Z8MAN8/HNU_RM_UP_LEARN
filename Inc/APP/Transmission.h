//
// Created by 14685 on 2022/7/3.
//

#ifndef HNU_RM_UP_TRANSMISSION_H
#define HNU_RM_UP_TRANSMISSION_H


#include "gpio.h"
#include "Gimbal.h"
#include "cmsis_os.h"


#define COM_CAN hcan2
#define CONTROL_CAN hcan1

#define CAN_UP_TX_INFO 0x134
/* 上下板CAN通信的扩展标识符 */
#define CAN_RPY_TX          0x140   /* 底盘跟随云台相对角度 */
#define CAN_ODOM_TX_ONE     0x141   /* odom数据帧第一帧 */
#define CAN_ODOM_TX_TWO     0x142   /* odom数据帧第二帧 */
#define CAN_ODOM_TX_THREE   0x143   /* odom数据帧第二帧 */

#define CAN_GIM_STATE         0x145   /* 底盘状态数据 */



/* BCP通讯协议相关 */
//TODO: 考虑不同帧长的情况
#define FRAME_NUM     10        /* 所有通讯帧的类型总数 */
#define FRAME_MAX_LEN 36        /* 通讯帧的最大长度 */
#define FRAME_RPY_LEN 25        /* 欧拉角rpy方式控制长度 */
#define FRAME_ODOM_LEN 36       /* 里程计控制方式长度 */
#define FRAME_IMU_LEN 24        /* imu控制方式长度 */
#define FRAME_CTRL_LEN 24       /* 角/线速度控制方式长度 */
/* 目标地址表 */
#define BROADCAST   0x00        /* 广播 */
#define MAINFLOD    0x01        /* 上位机 */
#define SENTRY_UP   0x02        /* 哨兵机器人上云台 */
#define SENTRY_DOWN 0x03        /* 哨兵机器人下云台 */
#define INFANTRY    0x04        /* 步兵机器人 */
#define ENGINEER    0x05        /* 工程机器人 */
#define HERO        0x06        /* 英雄机器人 */
#define AIR         0x07        /* 空中机器人 */
#define RADAR       0x08        /* 雷达站 */
#define GATHER      0x09        /* 视觉采集台 */
#define STANDARD    0x10        /* AI机器人/全自动步兵机器人 */
/* 功能码表 */
#define CHASSIS                 0x10        /* 速度方式控制 */
#define CHASSIS_ODOM            0x11        /* 里程计方式控制 */
#define CHASSIS_CTRL            0x12        /* 角/线速度方式控制 */
#define CHASSIS_IMU             0x13        /* 底盘imu数据 */
#define GIMBAL                  0x20        /* 欧拉角rpy方式控制 */
#define GAME_STATUS             0x30        /* 比赛类型数据*/
#define ROBOT_HP                0x31        /* 机器人血量数据 */
#define ICRA_BUFF_DEBUFF_ZONE   0x32        /* 增益区数据 */
#define GAME_MODE               0x33        /* 机器人颜色数据 */
#define ROBOT_COMMAND           0x34        /* 机器人位置信息 */
#define CLIENT_MAP_COMMAND      0x35        /* 雷达发送目标位置信息 */
#define BARREL                  0x40        /* 发射机构数据 */
#define MANIFOLD_CTRL           0x50        /* 控制模式 */
#define MODE                    0x60        /* 模式控制 */
#define DEV_ERROR               0xE0        /* 故障信息 */
#define HEARTBEAT               0xF0        /* 心跳数据 */

/**
  * @brief  自瞄发送结构体
  */
typedef  struct
{
    uint16_t head;				    /*! 帧头 */
    float pitchAngleGet;    	    /*! pitch轴角度 */
    float yawAngleGet;      	    /*! yaw轴角度 */
    uint8_t rotateDirection;        /*! 旋转方向 */
    float timeBais;         	    /*! 预测时间偏置 */
    float compensateBais;   	    /*! 弹道补偿偏置 */
    uint8_t gimbal_mode;	 	    /*! 云台模式 */
    uint32_t index;                 /*! 帧序号 */
}__attribute__((packed)) SendFrameTypeDef;


/**
  * @brief  自瞄接收结构体
  */
typedef  struct
{
    uint16_t head;  				/*! 帧头 */
    float pitchAngleSet;            /*! pitch轴角度设定值*/
    float yawAngleSet;              /*! yaw轴角度设定值 */
    float targetAngle;
    uint8_t shootCommand;
    uint32_t index;                 /*! 帧序号 */
}__attribute__((packed)) RecvFrameTypeDef;


/**
  * @brief  通讯帧结构体 （BCP通讯协议） 此为最大DATA长度的帧，用于接收中转
  */
typedef  struct
{
    uint8_t HEAD;  				    /*! 帧头 */
    uint8_t D_ADDR;                 /*! 目标地址 */
    uint8_t ID;                     /*! 功能码 */
    uint8_t LEN;                    /*! 数据长度 */
    int8_t DATA[FRAME_MAX_LEN];     /*! 数据内容 */
    uint8_t SC;                     /*! 和校验 */
    uint8_t AC;                     /*! 附加校验 */
}__attribute__((packed)) BCPFrameTypeDef;

/**
  * @brief  欧拉角方式控制通讯帧结构体
  */
typedef  struct
{
    uint8_t HEAD;  				    /*! 帧头 */
    uint8_t D_ADDR;                 /*! 目标地址 */
    uint8_t ID;                     /*! 功能码 */
    uint8_t LEN;                    /*! 数据长度 */
    int8_t DATA[FRAME_RPY_LEN];     /*! 数据内容 */
    uint8_t SC;                     /*! 和校验 */
    uint8_t AC;                     /*! 附加校验 */
}__attribute__((packed)) BCPRpyTypeDef;

/**
  * @brief  imu方式控制通讯帧结构体
  */
typedef  struct
{
    uint8_t HEAD;  				    /*! 帧头 */
    uint8_t D_ADDR;                 /*! 目标地址 */
    uint8_t ID;                     /*! 功能码 */
    uint8_t LEN;                    /*! 数据长度 */
    int8_t DATA[FRAME_IMU_LEN];    /*! 数据内容 */
    uint8_t SC;                     /*! 和校验 */
    uint8_t AC;                     /*! 附加校验 */
}__attribute__((packed)) BCPImuTypeDef;

extern BCPFrameTypeDef upper_rx_data;       //接收上位机数据中转帧
extern BCPFrameTypeDef upper_tx_data;       //发送上位机数据中转帧
extern BCPFrameTypeDef upper_tx_all_data[FRAME_NUM];       //合并发送上位机数据帧

extern BCPRpyTypeDef rpy_rx_data;  //接收欧拉角方式控制数据帧
extern BCPRpyTypeDef rpy_tx_data;  //发送欧拉角方式控制数据帧

extern BCPImuTypeDef imu_tx_data;           //发送imu方式控制数据帧

extern RecvFrameTypeDef auto_rx_data;
extern SendFrameTypeDef auto_tx_data;

/**
  * @brief     获取上板给下板发送的数据
  * @param[in] data: 即将存储数据的数组
  * @param[in] send_type: 通讯数据的类型，即CAN扩展类型
  * @retval    装好数据的数组
  */
void Get_Communicate_data(uint8_t* data, uint16_t send_type);

/**
  * @brief     发送上板给下板传输的数据
  * @param[in] hcan: 两板通信的CAN
  * @param[in] data: 上板给下板发送的数据
  * @param[in] send_type: 通讯数据的类型，即CAN扩展类型
  * @retval    装好数据的数组
  */
void Send_Communicate_data(CAN_HandleTypeDef *_hcan, uint8_t *data, uint16_t send_type);

/**
 * @brief BCP和校验算法
 * @param frame
 * @retval 合并为一个数据的校验码 SC:和校验（高8位） AC:附加和校验（低8位）
 */
uint16_t Sumcheck_Cal(BCPFrameTypeDef frame);

/**
 * @brief 将要发送的数据类型填入到对应的数据帧中
 * @param send_mode 功能码表
 * @param data_buf 填装好的DATA
 */
void Add_Frame_To_Upper(uint16_t send_mode, int8_t* data_buf);

#endif //HNU_RM_UP_TRANSMISSION_H