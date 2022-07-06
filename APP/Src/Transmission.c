//
// Created by 14685 on 2022/7/3.
//

#include "Transmission.h"
#include "main.h"
#include "usbd_cdc_if.h"
#include "Gimbal.h"

RecvFrameTypeDef auto_rx_data;
SendFrameTypeDef auto_tx_data;



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