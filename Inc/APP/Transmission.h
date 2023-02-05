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

/* BCP通讯协议相关 */
#define FRAME_MAX_LEN 36        /* 通讯帧的最大长度 */
/* 目标地址表 */
#define broadcast   0x00        /* 广播 */
#define mainfold    0x01        /* 上位机 */
#define sentry_up   0x02        /* 哨兵机器人上云台 */
#define sentry_down 0x03        /* 哨兵机器人下云台 */
#define infantry    0x04        /* 步兵机器人 */
#define engineer    0x05        /* 工程机器人 */
#define hero        0x06        /* 英雄机器人 */
#define air         0x07        /* 空中机器人 */
#define radar       0x08        /* 雷达站 */
#define gather      0x09        /* 视觉采集台 */
#define standard    0x10        /* AI机器人/全自动步兵机器人 */
/* 功能码表 */
#define chassis                 0x10        /* 速度方式控制 */
#define chassis_odom            0x11        /* 里程计方式控制 */
#define chassis_ctrl            0x12        /* 角/线速度方式控制 */
#define gimbal                  0x20        /* 欧拉角rpy方式控制 */
#define game_status             0x30        /* 比赛类型数据*/
#define robot_HP                0x31        /* 机器人血量数据 */
#define ICRA_buff_debuff_zone   0x32        /* 增益区数据 */
#define game_mode               0x33        /* 机器人颜色数据 */
#define robot_command           0x34        /* 机器人位置信息 */
#define client_map_command      0x35        /* 雷达发送目标位置信息 */
#define barrel                  0x40        /* 发射机构数据 */
#define manifold_ctrl           0x50        /* 控制模式 */
#define mode                    0x60        /* 模式控制 */
#define dev_error               0xE0        /* 故障信息 */
#define heartbeat               0xF0        /* 心跳数据 */


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
  * @brief  通讯帧结构体 （BCP通讯协议）
  */
typedef  struct
{
    uint16_t HEAD;  				/*! 帧头 */
    uint16_t D_ADDR;                /*! 目标地址 */
    uint16_t ID;                    /*! 功能码 */
    uint16_t LEN;                   /*! 数据长度 */
    int8_t DATA[FRAME_MAX_LEN];     /*! 数据内容 */
    uint8_t SC;                     /*! 和校验 */
    uint8_t AC;                     /*! 附加校验 */
}__attribute__((packed)) BCPFrameTypeDef;

extern BCPFrameTypeDef upper_rx_data;  //接收上位机数据帧

extern RecvFrameTypeDef auto_rx_data;
extern SendFrameTypeDef auto_tx_data;

/**
  * @brief     获取上板给下板发送的数据
  * @param[in] data: 即将存储数据的数组
  * @retval    装好数据的数组
  */
void Get_Communicate_data(uint8_t* data);

/**
  * @brief     发送上板给下板传输的数据
  * @param[in] hcan: 两板通信的CAN
  * @param[in] data: 上板给下板发送的数据
  * @retval    装好数据的数组
  */
void Send_Communicate_data(CAN_HandleTypeDef *_hcan, uint8_t *data);

/**
 * @brief BCP和校验算法
 * @param frame
 * @retval 合并为一个数据的校验码 SC:和校验（高8位） AC:附加和校验（低8位）
 */
uint16_t Sumcheck_Cal(BCPFrameTypeDef frame);

#endif //HNU_RM_UP_TRANSMISSION_H