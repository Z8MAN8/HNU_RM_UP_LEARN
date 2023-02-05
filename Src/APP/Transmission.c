//
// Created by 14685 on 2022/7/3.
//

#include "Transmission.h"
#include "QuaternionAHRS.h"
#include "main.h"
#include "usbd_cdc_if.h"
#include "Gimbal.h"
#include "usart.h"
#include "bsp_can.h"

BCPFrameTypeDef upper_rx_data;
float testdata[4]={0};
RecvFrameTypeDef auto_rx_data;
//初始化自瞄发送帧
SendFrameTypeDef auto_tx_data =
        {
                .head=0xbbbb,
                .rotateDirection=1,
                .index=0
//                .index=0,
//                .q0=0.0f,
//                .q1=0.0f,
//                .q2=0.0f,
//                .q3=0.0f,
        };

/*上板给下板发送的数据*/
static uint8_t cm_data[8] = {0};
extern volatile float yaw_angle_ref_v;


void transmission_task(void const * argument)
{
    __HAL_UART_ENABLE_IT(&huart1,UART_IT_RXNE);
    uint32_t transmission_wake_time = osKernelSysTick();

    while (1)
    {
        /*给下板发送数据*/
        Get_Communicate_data(cm_data);
        Send_Communicate_data(&COM_CAN, cm_data);

        auto_tx_data.pitchAngleGet=pit_angle_fdb;
        auto_tx_data.yawAngleGet=yaw_angle_fdb;
        auto_tx_data.gimbal_mode=gim.ctrl_mode;
        testdata[0]=-AHRS.Pitch;
        testdata[1]=-AHRS.Roll;
        testdata[2]=-AHRS.Yaw;

        osDelayUntil(&transmission_wake_time, 1);
    }

}


void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
    //相机硬件同步
    if(GPIO_Pin==CAM_IO_Pin)
    {
        CDC_Transmit_FS((uint8_t*)&auto_tx_data,sizeof(auto_tx_data));
        int i=(auto_tx_data.index%25)*2;
        //记录拍摄这一帧时云台的位姿
        angle_history[i]=yaw_angle_fdb;
        angle_history[i+1]=pit_angle_fdb;
        auto_tx_data.index++;
        HAL_GPIO_TogglePin(GPIOH,GPIO_PIN_12);
    }
}

void Get_Communicate_data(uint8_t* data){
    //c不支持浮点数移位操作，因为浮点数储存和整数储存不同
    //这里将浮点数转化为整型数进行移位(不是强制类型转换，是将这个储存空间的内存的解释规则转化为整形
    uint32_t *yaw_angle = (uint32_t *)&yaw_relative_angle;
    uint32_t *yaw_ref_v = (uint32_t *)&yaw_angle_ref_v;
    //第一位用于判断是否唤醒底盘
    data[3] = *yaw_angle >> 24;
    data[2] = *yaw_angle >> 16;
    data[1] = *yaw_angle >> 8;
    data[0] = *yaw_angle;
    data[7] = *yaw_ref_v >> 24;
    data[6] = *yaw_ref_v >> 16;
    data[5] = *yaw_ref_v >> 8;
    data[4] = *yaw_ref_v;
}

void Send_Communicate_data(CAN_HandleTypeDef *_hcan, uint8_t *data) {
    static CAN_TxHeaderTypeDef TX_MSG;
    static uint8_t CAN_Send_Data[8];
    uint32_t send_mail_box;

    TX_MSG.StdId = CAN_UP_TX_INFO;
    TX_MSG.IDE = CAN_ID_STD;
    TX_MSG.RTR = CAN_RTR_DATA;
    TX_MSG.DLC = 0x08;
    TX_MSG.TransmitGlobalTime = DISABLE;
    CAN_Send_Data[0] = data[0];
    CAN_Send_Data[1] = data[1];
    CAN_Send_Data[2] = data[2];
    CAN_Send_Data[3] = data[3];
    CAN_Send_Data[4] = data[4];
    CAN_Send_Data[5] = data[5];
    CAN_Send_Data[6] = data[6];
    CAN_Send_Data[7] = data[7];
    HAL_CAN_AddTxMessage(_hcan, &TX_MSG, CAN_Send_Data, &send_mail_box);
}

uint16_t Sumcheck_Cal(BCPFrameTypeDef frame){
    uint8_t sumcheck = 0;
    uint8_t addcheck = 0;
    uint16_t allcheck = 0;

    sumcheck += frame.HEAD;
    addcheck += sumcheck;
    sumcheck += frame.D_ADDR;
    addcheck += sumcheck;
    sumcheck += frame.ID;
    addcheck += sumcheck;
    sumcheck += frame.LEN;
    addcheck += sumcheck;

    for(int i = 0; i<FRAME_MAX_LEN; i++){
        sumcheck += frame.DATA[i];
        addcheck += sumcheck;
    }
    allcheck = (uint16_t)(sumcheck << 8 | addcheck);
    return allcheck;
}
