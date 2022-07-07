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

#endif //HNU_RM_UP_TRANSMISSION_H