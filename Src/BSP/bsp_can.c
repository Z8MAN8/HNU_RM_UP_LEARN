//
// Created by HLiamso on 2021-11-14.
//

#include <can.h>
#include <Detect.h>
#include <Shoot.h>
#include "bsp_can.h"
#include "Transmission.h"
#define CAN_DOWN_TX_INFO 0x133
//TODO:
//chassis_mode_e chassis_mode;
/* 云台电机 */
MotorTypeDef PitMotor_Manual;
MotorTypeDef YawMotor_Manual;
MotorTypeDef PitMotor_Auto;
MotorTypeDef YawMotor_Auto;
/* 拨弹电机 */
MotoMeasureTypeDef moto_trigger;
/* 底盘电机 */
MotorTypeDef chassis_motor[4];
/* 3508摩擦轮电机 */
MotoMeasureTypeDef moto_shoot[2];//0左，1右；

void CAN_Device_Init(void){
    CAN_FilterTypeDef can_filter_st;
    can_filter_st.FilterActivation = ENABLE;
    can_filter_st.FilterMode = CAN_FILTERMODE_IDMASK;
    can_filter_st.FilterScale = CAN_FILTERSCALE_32BIT;
    can_filter_st.FilterIdHigh = 0x0000;
    can_filter_st.FilterIdLow = 0x0000;
    can_filter_st.FilterMaskIdHigh = 0x0000;
    can_filter_st.FilterMaskIdLow = 0x0000;
    can_filter_st.FilterBank = 0;
    can_filter_st.FilterFIFOAssignment = CAN_RX_FIFO0;
    // CAN1 过滤器初始化
    while (HAL_CAN_ConfigFilter(&hcan1, &can_filter_st) != HAL_OK)
    {
    }
    // 启动CAN1
    while (HAL_CAN_Start(&hcan1) != HAL_OK)
    {
    }
    // 启动通知
    while (HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING) != HAL_OK)
    {

    }

    can_filter_st.SlaveStartFilterBank = 14;
    can_filter_st.FilterBank = 14;
    // CAN2 过滤器初始化
    while (HAL_CAN_ConfigFilter(&hcan2, &can_filter_st) != HAL_OK)
    {
    }
    // 启动CAN2
    while (HAL_CAN_Start(&hcan2) != HAL_OK)
    {
    }
    // 启动通知
    while (HAL_CAN_ActivateNotification(&hcan2, CAN_IT_RX_FIFO0_MSG_PENDING) != HAL_OK)
    {
    }
}
void send_chassis_moto_zero_current(void)
{
//    static uint8_t data[8];
//
//    data[0] = 0;
//    data[1] = 0;
//    data[2] = 0;
//    data[3] = 0;
//    data[4] = 0;
//    data[5] = 0;
//    data[6] = 0;
//    data[7] = 0;
//
//    Write_CAN(hcan1, CAN_CHASSIS_ID, data);
//    stop_chassis = 1;
}

void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan){
    CAN_RxHeaderTypeDef rx_header;
    uint8_t rx_data[8];
    HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &rx_header, rx_data);
    if(hcan==&COM_CAN){
        switch (rx_header.StdId){
            case CAN_DOWN_TX_INFO :
                //TODO:
//                chassis_mode = rx_data[0];
                break;
            case 0x134 ://拨弹电机
                shooter_output = rx_data[0];
                shooter_id1_17mm_cooling_limit = (uint16_t)(rx_data[1]<<8 | rx_data[2]);
                shooter_id1_17mm_cooling_heat  = (uint16_t)(rx_data[3]<<8 | rx_data[4]);

                break;
            case CAN_3508_M3_ID:
            {
                moto_trigger.msg_cnt++;
                moto_trigger.msg_cnt <= 10 ? Get_Moto_offset(&moto_trigger, rx_data) : Encoder_Data_handle(
                        &moto_trigger, rx_data);
                Err_Detector_hook(TRIGGER_MOTO_OFFLINE);//错误检测
            }
                break;
            default:
                break;
        }
    }
    if(hcan==&CONTROL_CAN) {
        switch (rx_header.StdId) {
            case CAN_YAW_MOTOR_ID: {
                //不理解为什么要获取50次偏移量？？？
                //已解决：稳定后再运算
                if (YawMotor_Manual.msg_cnt++ <= 50)
                    get_motor_offset(&YawMotor_Manual, rx_data);
                else
                    get_moto_info(&YawMotor_Manual, rx_data);
                Err_Detector_hook(GIMBAL_YAW_OFFLINE);

                if (YawMotor_Auto.msg_cnt++ <= 50)
                    get_motor_offset(&YawMotor_Auto, rx_data);
                else
                    get_moto_info(&YawMotor_Auto, rx_data);
                Err_Detector_hook(GIMBAL_YAW_OFFLINE);
            }
                break;
            case CAN_PIT_MOTOR_ID: {
                if (PitMotor_Manual.msg_cnt++ <= 50)
                    get_motor_offset(&PitMotor_Manual, rx_data);
                else
                    get_moto_info(&PitMotor_Manual, rx_data);
                Err_Detector_hook(GIMBAL_PIT_OFFLINE);

                if (PitMotor_Auto.msg_cnt++ <= 50)
                    get_motor_offset(&PitMotor_Auto, rx_data);
                else
                    get_moto_info(&PitMotor_Auto, rx_data);
                Err_Detector_hook(GIMBAL_PIT_OFFLINE);
            }
                break;
            case CAN_3508_M1_ID:
            {
                moto_shoot[0].msg_cnt++ <= 50 ? Get_Moto_offset(&moto_shoot[0], rx_data) : \
                Encoder_Data_handle(&moto_shoot[0], rx_data);
                Err_Detector_hook(AMMO_BOOSTER1_OFFLINE);
            }
                break;
            case CAN_3508_M2_ID:
            {
                //PE4置位
                //GPIOE->BSRR=0x10;
                moto_shoot[1].msg_cnt++ <= 50 ? Get_Moto_offset(&moto_shoot[1], rx_data) : \
                Encoder_Data_handle(&moto_shoot[1], rx_data);
                Err_Detector_hook(AMMO_BOOSTER2_OFFLINE);


                //PE4复位
                //GPIOE->BSRR=0x100000;
            }
                break;
                //拨弹电机

            default:
            {
            }
                break;
        }

        }
    }
//        case CAN_3508_M1_ID:
//        {
//            chassis_motor[0].msg_cnt++ <= 50 ? get_motor_offset(&chassis_motor[0], rx_data) : \
//            get_moto_info(&chassis_motor[0], rx_data);
//            Err_Detector_hook(CHASSIS_M1_OFFLINE);
//        }
//        break;
//        case CAN_3508_M2_ID:
//        {
//            chassis_motor[1].msg_cnt++ <= 50 ? get_motor_offset(&chassis_motor[1], rx_data) : \
//            get_moto_info(&chassis_motor[1], rx_data);
//            Err_Detector_hook(CHASSIS_M2_OFFLINE);
//        }
//        break;
//
//        case CAN_3508_M3_ID:
//        {
//            chassis_motor[2].msg_cnt++ <= 50 ? get_motor_offset(&chassis_motor[2], rx_data) : \
//            get_moto_info(&chassis_motor[2], rx_data);
//            Err_Detector_hook(CHASSIS_M3_OFFLINE);
//        }
//        break;
//        case CAN_3508_M4_ID:
//        {
//            chassis_motor[3].msg_cnt++ <= 50 ? get_motor_offset(&chassis_motor[3], rx_data) : \
//            get_moto_info(&chassis_motor[3], rx_data);
//            Err_Detector_hook(CHASSIS_M4_OFFLINE);
//        }
//        break;
//
//        case CAN_SUPERCAP_RECV:
//            //PowerDataResolve(rx_data);
//        default:
//        {
//        }
//        break;
//        }
//    }

void Write_CAN(CAN_HandleTypeDef can, uint32_t send_id, uint8_t send_data[]){
    uint32_t send_mail_box;
    tx_message.StdId = send_id;
    tx_message.IDE = CAN_ID_STD;
    tx_message.RTR = CAN_RTR_DATA;
    tx_message.DLC = 0x08;
    //问题：为什么不能直接用send_data发送,感觉不会出问题
    for(int i=0;i<8;i++)
        can_send_data[i] = send_data[i];
    HAL_CAN_AddTxMessage(&can, &tx_message, can_send_data, &send_mail_box);
}


/**
  * @brief     获得电机初始偏差
  * @param     ptr: 电机参数 MotoMeasureTypeDef 结构体指针
  * @param     data: 接收到的电机 CAN 数据指针
  */
static void Get_Moto_offset(MotoMeasureTypeDef *ptr, uint8_t data[])
{
    ptr->ecd        = (uint16_t)(data[0] << 8 | data[1]);
    ptr->offset_ecd = ptr->ecd;
}

/**
  * @brief     计算电机的转速rmp 圈数round_cnt
  *            总编码器数值total_ecd 总旋转的角度total_angle
  * @param     ptr: 电机参数 MotoMeasureTypeDef 结构体指针
  * @param     data: 接收到的电机 CAN 数据指针
  */
static void Encoder_Data_handle(MotoMeasureTypeDef *ptr, uint8_t data[])
{
    int32_t temp_sum = 0;

    ptr->last_ecd      = ptr->ecd;
    //转子机械角度
    ptr->ecd           = (uint16_t)(data[0] << 8 | data[1]);
    //转子转速
    ptr->speed_rpm     = (int16_t)(data[2] << 8 | data[3]);
    //转矩电流没有处理
    if (ptr->ecd - ptr->last_ecd > 4096)
    {
        ptr->round_cnt--;
        ptr->ecd_raw_rate = ptr->ecd - ptr->last_ecd - 8192;
    }
    else if (ptr->ecd - ptr->last_ecd < -4096)
    {
        ptr->round_cnt++;
        ptr->ecd_raw_rate = ptr->ecd - ptr->last_ecd + 8192;
    }
    else
    {
        ptr->ecd_raw_rate = ptr->ecd - ptr->last_ecd;
    }

    ptr->total_ecd = ptr->round_cnt * 8192 + ptr->ecd - ptr->offset_ecd;
    ptr->total_angle = ptr->total_ecd * 360 / 8192;


    ptr->rate_buf[ptr->buf_cut++] = ptr->ecd_raw_rate;
    if (ptr->buf_cut >= FILTER_BUF)
        ptr->buf_cut = 0;
    for (uint8_t i = 0; i < FILTER_BUF; i++)
    {
        temp_sum += ptr->rate_buf[i];
    }
    ptr->filter_rate = (int32_t)(temp_sum/FILTER_BUF);
}


/**
  * @brief     发送云台电机电流数据到电调
  */
extern int16_t trigger_moto_current;

//#ifdef TEST_ON_ICRA
//void GimbalMoto_Send_current(int16_t yaw_current, int16_t pit_current)
//{
//    static uint8_t data[8];
//    static uint8_t data_yaw[8];
//
//    data[0] = 0;
//    data[1] = 0;
//    data[2] = pit_current >> 8;
//    data[3] = pit_current;
//    //data[4] = trigger_current >> 8;
//    //data[5] = trigger_current;
//    data[6] = 0;
//    data[7] = 0;
//
//    data_yaw[0] = 0;
//    data_yaw[1] = 0;
//    data_yaw[2] = 0;
//    data_yaw[3] = 0;
//    data_yaw[4] = yaw_current >> 8;
//    data_yaw[5] = yaw_current;
//    data_yaw[6] = 0;
//    data_yaw[7] = 0;
//
//    Write_CAN(CONTROL_CAN, CAN_GIMBAL_ID_PITCH, data);
//    Write_CAN(CONTROL_CAN, CAN_GIMBAL_ID_YAW, data_yaw);
//}
//void GimbalMoto_Send_zero_current(void)
//{
//    static uint8_t data[8];
//
//    data[0] = 0;
//    data[1] = 0;
//    data[2] = 0;
//    data[3] = 0;
//    data[4] = 0;
//    data[5] = 0;
//    data[6] = 0;
//    data[7] = 0;
//
//    Write_CAN(CONTROL_CAN, CAN_GIMBAL_ID_YAW, data);
//    Write_CAN(CONTROL_CAN, CAN_GIMBAL_ID_PITCH, data);
//}
//
//#else
void GimbalMoto_Send_current(int16_t yaw_current, int16_t pit_current)
{
    static uint8_t data[8];

    data[0] = yaw_current >> 8;
    data[1] = yaw_current;
    data[2] = pit_current >> 8;
    data[3] = pit_current;
    //data[4] = trigger_current >> 8;
    //data[5] = trigger_current;
    data[6] = 0;
    data[7] = 0;

    Write_CAN(CONTROL_CAN, CAN_GIMBAL_ID, data);
}
void GimbalMoto_Send_zero_current(void)
{
    static uint8_t data[8];
    data[0] = 0;
    data[1] = 0;
    data[2] = 0;
    data[3] = 0;
    data[4] = 0;
    data[5] = 0;
    data[6] = 0;
    data[7] = 0;
    Write_CAN(CONTROL_CAN, CAN_GIMBAL_ID, data);
}
//#endif

void ShootMoto_Send_current(int16_t left_current, int16_t right_current, int16_t pit_current)
{
    static uint8_t data[8];
    data[0] = left_current >> 8;
    data[1] = left_current;
    data[2] = right_current >> 8;
    data[3] = right_current;
    data[4] = pit_current >> 8;
    data[5] = pit_current;
    data[6] = 0;
    data[7] = 0;
    //疑问：分别向can1、2发送数据？？？
    Write_CAN(CONTROL_CAN, CAN_CHASSIS_ID, data);
    Write_CAN(hcan2, CAN_CHASSIS_ID, data);
}

void sendSuperCap(void)
{
    uint16_t temPower =9000;//功率设定步进0.01W，范围为3000-13000（30W-130W）
    uint8_t sendbuf[8];//发送的数据内容
    sendbuf[0]=temPower >> 8;
    sendbuf[1]=temPower;
    Write_CAN(COM_CAN, CAN_SUPER_CAP_ID, sendbuf);
}
