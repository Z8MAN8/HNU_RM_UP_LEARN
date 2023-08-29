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
#include "shoot.h"

BCPFrameTypeDef upper_rx_data;
BCPFrameTypeDef upper_tx_all_data[FRAME_NUM];
BCPFrameTypeDef upper_tx_data;
BCPRpyTypeDef rpy_rx_data;
BCPRpyTypeDef rpy_tx_data;
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
static uint8_t gim_state_buffer[8] = {0};
extern volatile float yaw_angle_ref_v;

extern uint8_t USB_SEND_OK;

// 都发了些什么？
//  - cm_data: 发送给底盘的 角度数据的buffer
//  - auto_tx_data: 发送给上位机 用于自瞄的 （目前只传了pitch、yaw角度以及云台控制模式以及旋转方向
//  - rpy_tx_data: 通过usb发送给 上位机的 欧拉角数据
//  - testdata: 测试数据

// xxx_data 和 xxx_buffer 是什么关系？
//  - xxx_data 是包含了帧头（帧尾）的完整的整个数据帧data，xxx_buffer是单纯的数据部分。在接下来的处理中加上帧头（帧尾）变成 xxx_data

// 这些角度都是什么？
//  - xxx_angle_fdb: 反馈角度（单位：degree） 用imu反馈的值减去offset(归中完成时的imu角度）得到
//  - xxx_relative_angle：相对（归中值）角度（单位：degree）（用电机编码器的值和写死的归中完成的值算出来的）
//  - xxx_angle_ref：期望角度（单位：degree
//  - gim.xxx_offset_angle：归中完成时imu的角度值。一开始使用 xxx_relative_angle 作为现在反馈角度（xxx_angle_fdb)，在得到这个值之后就用这个了（imu更精确）
//  - 同理，后面接了v就是指角速度

void transmission_task(void const * argument)
{
    int8_t rpy_tx_buffer[FRAME_RPY_LEN] = {0} ;
    int32_t rpy_data = 0;
    uint32_t *gimbal_rpy = (uint32_t *)&rpy_data;

    int8_t imu_tx_buffer[FRAME_IMU_LEN] = {0} ;
    int32_t imu_data = 0;
    uint32_t *gimbal_imu = (uint32_t *)&imu_data;

    //gimbal_rpy、gimbal_imu分别指向rpy_data、imu_data数据帧的内存地址

    __HAL_UART_ENABLE_IT(&huart1,UART_IT_RXNE);//打开uart1中断
    uint32_t transmission_wake_time = osKernelSysTick();

    while (1)
    {

        /*给下板发送数据*/
        //发送gimbal任务计算得到的yaw_relative_angle，云台相对于中心位置的角度，
        //问题：yaw_angle_ref_v在全局只出现过一次？？？没有赋值
        //已解决：yaw_angle_ref_v没有用到
        Get_Communicate_data(cm_data, CAN_RPY_TX);//给cm_data赋值
        Send_Communicate_data(&COM_CAN, cm_data, CAN_RPY_TX);//发送cm_data数据

        //传输：云台的模式、是否打开弹仓，摩擦轮转动是否正常
        Get_Communicate_data(gim_state_buffer, CAN_GIM_STATE);
        Send_Communicate_data(&COM_CAN, gim_state_buffer, CAN_GIM_STATE);

        auto_tx_data.pitchAngleGet=pit_angle_fdb;
        auto_tx_data.yawAngleGet=yaw_angle_fdb;
        auto_tx_data.gimbal_mode=gim.ctrl_mode;
        //问题：记录但未使用
        testdata[0]=-AHRS.Pitch;
        testdata[1]=-AHRS.Roll;
        testdata[2]=-AHRS.Yaw;

        /* USB发送角度帧 */
        //gimbal_rpy、gimbal_imu分别指向rpy_data、imu_data数据帧的内存地址
        //发送三个数据，一个数据用四个字节存储
        //问题：在gimbal任务中，一般gim.yaw_offset_angle = imu.angle_x; 所以向上位机发送的是什么数据
        //已解决
        rpy_tx_buffer[0] = 0;
        rpy_data = (imu.angle_x- gim.yaw_offset_angle) * 1000;
        rpy_tx_buffer[1] = *gimbal_rpy;
        rpy_tx_buffer[2] = *gimbal_rpy >> 8;
        rpy_tx_buffer[3] = *gimbal_rpy >> 16;
        rpy_tx_buffer[4] = *gimbal_rpy >> 24;
        rpy_data = (imu.angle_y-gim.pit_offset_angle) * 1000;
        rpy_tx_buffer[5] = *gimbal_rpy;
        rpy_tx_buffer[6] = *gimbal_rpy >> 8;
        rpy_tx_buffer[7] = *gimbal_rpy >> 16;
        rpy_tx_buffer[8] = *gimbal_rpy >> 24;
        rpy_data = imu.angle_z * 1000;
        rpy_tx_buffer[9] = *gimbal_rpy;
        rpy_tx_buffer[10] = *gimbal_rpy >> 8;
        rpy_tx_buffer[11] = *gimbal_rpy >> 16;
        rpy_tx_buffer[12] = *gimbal_rpy >> 24;

        // 这里是发送数据，云台接收数据在 `usbd_cdc_if.c` 中的**回调函数** CDC_Receive_FS() 里进行处理。
        //构建一个通信协议帧
        Add_Frame_To_Upper(GIMBAL, rpy_tx_buffer);
        CDC_Transmit_FS((uint8_t*)&rpy_tx_data, sizeof(rpy_tx_data));

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
        //问题：记录后在哪里使用
        angle_history[i]=yaw_angle_fdb;
        angle_history[i+1]=pit_angle_fdb;
        auto_tx_data.index++;
        HAL_GPIO_TogglePin(GPIOH,CAM_IO_Pin);
    }
}

void Get_Communicate_data(uint8_t* data, uint16_t send_type){
    switch (send_type) {
        case CAN_RPY_TX: {
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
        }break;

        // 传输云台状态数据
        case CAN_GIM_STATE: {
            // 其实可以强制类型转换成 uint8_t 也不会丢数据，但是那就要在参数那里转换，比较难看，现在就传这一个也不挤所以先保留int传过去好了
            int *gim_state = (int *)&gim.ctrl_mode;
            data[3] = 0;
            data[2] = cap_ok;
            data[1] = shoot_ok; //摩擦轮转动正常标志位;
            data[0] = *gim_state;
            // 未用到，留空
            data[7] = 0;
            data[6] = 0;
            data[5] = 0;
            data[4] = 0;
        }
        default: break;
    }
}

void Send_Communicate_data(CAN_HandleTypeDef *_hcan, uint8_t *data, uint16_t send_type) {
    static CAN_TxHeaderTypeDef TX_MSG;
    static uint8_t CAN_Send_Data[8];
    uint32_t send_mail_box;

//问题：这里用扩展帧，下层读取的时候用的是标准帧
    TX_MSG.StdId = CAN_UP_TX_INFO;
    TX_MSG.IDE = CAN_ID_EXT;            /* 上下板通讯使用扩展帧 */
    TX_MSG.ExtId = send_type;
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

void Add_Frame_To_Upper(uint16_t send_mode, int8_t* data_buf){
    switch (send_mode) {
        case GIMBAL:{
            rpy_tx_data.HEAD = 0XFF;
            rpy_tx_data.D_ADDR = MAINFLOD;
            rpy_tx_data.ID = GIMBAL;
            rpy_tx_data.LEN = FRAME_RPY_LEN;
            memcpy(&rpy_tx_data.DATA, data_buf, sizeof(rpy_tx_data.DATA));

            /* 将 odom 帧先转存到中转帧中做数据校验计算 */
            memcpy(&upper_tx_data, &rpy_tx_data, sizeof(rpy_tx_data));
            rpy_tx_data.SC = (uint8_t)Sumcheck_Cal(upper_tx_data) >> 8;
            rpy_tx_data.AC = (uint8_t)Sumcheck_Cal(upper_tx_data);
            memset(&upper_tx_data, 0, sizeof(upper_tx_data));
        }break;
        default:break;
    }
}

//TODO: 校验结果不对
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

    for(int i = 0; i<frame.LEN; i++){
        sumcheck += frame.DATA[i];
        addcheck += sumcheck;
    }
    allcheck = (uint16_t)(sumcheck << 8 | addcheck);
    return allcheck;
}
