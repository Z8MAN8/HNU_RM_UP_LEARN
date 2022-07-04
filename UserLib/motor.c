/**
  ******************************************************************************
  * @file    Motor.c
  * @author  Hongxi Wong
  * @version V1.2.1
  * @date    2021/4/13
  * @brief   
  ******************************************************************************
  * @attention 
  *
  ******************************************************************************
  */
#include "motor.h"
#include "stm32f4xx_hal_can.h"


float Motor_Torque_Calculate(Motor_t *motor, float torque, float target_torque)
{
    // 前馈控制
    Feedforward_Calculate(&motor->FFC_Torque, target_torque);
    // 反馈控制
    PID_Calculate(&motor->PID_Torque, torque, target_torque);

    if (motor->TorqueCtrl_User_Func_f != NULL)
        motor->TorqueCtrl_User_Func_f(motor);

    if (motor->Direction != NEGATIVE)
        motor->Output = motor->FFC_Torque.Output + motor->PID_Torque.Output + motor->Ke * motor->Velocity_RPM;
    else
        motor->Output = motor->FFC_Torque.Output + motor->PID_Torque.Output - motor->Ke * motor->Velocity_RPM;
    // 输出限幅
    motor->Output = float_constrain(motor->Output, -motor->Max_Out, motor->Max_Out);

    return motor->Output;
}

float Motor_Speed_Calculate(Motor_t *motor, float velocity, float target_speed)
{
    // 前馈控制
    Feedforward_Calculate(&motor->FFC_Velocity, target_speed);
    // 反馈控制
    PID_Calculate(&motor->PID_Velocity, velocity, target_speed);
    // 线性扰动观测器
    LDOB_Calculate(&motor->LDOB, velocity, motor->Output);

    if (motor->SpeedCtrl_User_Func_f != NULL)
        motor->SpeedCtrl_User_Func_f(motor);

    // 扰动补偿
    motor->Output = motor->FFC_Velocity.Output + motor->PID_Velocity.Output - motor->LDOB.Output;
    // 输出限幅
    motor->Output = float_constrain(motor->Output, -motor->Max_Out, motor->Max_Out);

    return motor->Output;
}

float Motor_Angle_Calculate(Motor_t *motor, float angle, float velocity, float target_angle)
{
    // 外环前馈控制
    Feedforward_Calculate(&motor->FFC_Angle, target_angle);
    // 外环反馈控制
    PID_Calculate(&motor->PID_Angle, angle, target_angle);

    if (motor->AngleCtrl_User_Func_f != NULL)
        motor->AngleCtrl_User_Func_f(motor);

    // 内环
    Motor_Speed_Calculate(motor, velocity, motor->FFC_Angle.Output + motor->PID_Angle.Output);

    return motor->Output;
}

/**
  * @Func	    void get_moto_info(moto_measure_t *ptr, CAN_HandleTypeDef* hcan)
  * @Brief      process data received from CAN
  * @Param	    Motor_t *ptr  CAN_HandleTypeDef *_hcan
  * @Retval	    None
  * @Date       2019/11/5
 **/
void get_moto_info(Motor_t *ptr, uint8_t *aData)
{
    // 详见C620电调手册
    if (ptr->Direction != NEGATIVE)
    {
        ptr->RawAngle = (uint16_t)(aData[0] << 8 | aData[1]);
        ptr->Velocity_RPM = (int16_t)(aData[2] << 8 | aData[3]);
    }
    else
    {
        ptr->RawAngle = 8191 - (uint16_t)(aData[0] << 8 | aData[1]);
        ptr->Velocity_RPM = -(int16_t)(aData[2] << 8 | aData[3]);
    }

    ptr->Real_Current = (int16_t)(aData[4] << 8 | aData[5]);
    ptr->Temperature = aData[6];

    if (ptr->RawAngle - ptr->last_angle > 4096)
        ptr->round_cnt--;
    else if (ptr->RawAngle - ptr->last_angle < -4096)
        ptr->round_cnt++;

    ptr->Angle = loop_float_constrain(ptr->RawAngle - ptr->zero_offset, -4095.5, 4095.5);

    ptr->AngleInDegree = ptr->Angle * 0.0439507f;

    ptr->total_angle = ptr->round_cnt * 8192 + ptr->RawAngle - ptr->offset_angle;

    ptr->last_angle = ptr->RawAngle; //update last_angle
}

/*this function should be called after system+can init */
void get_motor_offset(Motor_t *ptr, uint8_t *aData)
{
    ptr->RawAngle = (uint16_t)(aData[0] << 8 | aData[1]);
    ptr->offset_angle = ptr->RawAngle;
    ptr->last_angle = ptr->RawAngle; //update last_angle
}

//HAL_StatusTypeDef Send_Motor_Current_1_4(CAN_HandleTypeDef *_hcan,
//                                         int16_t c1, int16_t c2, int16_t c3, int16_t c4)
///**
//  * @Func	    void Send_Motor_Current(CAN_HandleTypeDef* hcan,
//                                    int16_t c1, int16_t c2, int16_t c3, int16_t c4)
//  * @Brief      
//  * @Param	    cx x=1,2,3,4
//  * @Retval	    None
//  * @Date       2019/11/5
// **/
//HAL_StatusTypeDef Send_Motor_Current_1_4(CAN_HandleTypeDef *_hcan,
//                                         int16_t c1, int16_t c2, int16_t c3, int16_t c4)
//{
//    static CAN_TxHeaderTypeDef TX_MSG;
//    static uint8_t CAN_Send_Data[8];
//    uint32_t send_mail_box;

//    // 设置发送数据包的ID和其他属性 除ID为特定内容外均为默认（通用）情况
//    TX_MSG.StdId = CAN_Transmit_1_4_ID;
//    TX_MSG.IDE = CAN_ID_STD;
//    TX_MSG.RTR = CAN_RTR_DATA;
//    TX_MSG.DLC = 0x08;

//    // 根据C620电调协议设置电机电流值
//    CAN_Send_Data[0] = (c1 >> 8);
//    CAN_Send_Data[1] = c1;
//    CAN_Send_Data[2] = (c2 >> 8);
//    CAN_Send_Data[3] = c2;
//    CAN_Send_Data[4] = (c3 >> 8);
//    CAN_Send_Data[5] = c3;
//    CAN_Send_Data[6] = (c4 >> 8);
//    CAN_Send_Data[7] = c4;

//    while (!((_hcan->State == HAL_CAN_STATE_READY) || (_hcan->State == HAL_CAN_STATE_LISTENING)))
//    {
//    }
//    while (HAL_CAN_GetTxMailboxesFreeLevel(_hcan) == 0) //如果三个邮箱都阻塞了就等一会儿，直到其中某个邮箱空闲
//    {
//    }
//    /* Check Tx Mailbox 1 status */
//    if ((_hcan->Instance->TSR & CAN_TSR_TME0) != 0U)
//    {
//        send_mail_box = CAN_TX_MAILBOX0;
//    }
//    /* Check Tx Mailbox 1 status */
//    else if ((_hcan->Instance->TSR & CAN_TSR_TME1) != 0U)
//    {
//        send_mail_box = CAN_TX_MAILBOX1;
//    }

//    /* Check Tx Mailbox 2 status */
//    else if ((_hcan->Instance->TSR & CAN_TSR_TME2) != 0U)
//    {
//        send_mail_box = CAN_TX_MAILBOX2;
//    }
//    return HAL_CAN_AddTxMessage(_hcan, &TX_MSG, CAN_Send_Data, &send_mail_box);
//}

//HAL_StatusTypeDef Send_Motor_Current_5_8(CAN_HandleTypeDef *_hcan,
//                                         int16_t c1, int16_t c2, int16_t c3, int16_t c4)
//{
//    static CAN_TxHeaderTypeDef TX_MSG;
//    static uint8_t CAN_Send_Data[8];
//    uint32_t send_mail_box;

//    TX_MSG.StdId = CAN_Transmit_5_8_ID;
//    TX_MSG.IDE = CAN_ID_STD;
//    TX_MSG.RTR = CAN_RTR_DATA;
//    TX_MSG.DLC = 0x08;
//    CAN_Send_Data[0] = (c1 >> 8);
//    CAN_Send_Data[1] = c1;
//    CAN_Send_Data[2] = (c2 >> 8);
//    CAN_Send_Data[3] = c2;
//    CAN_Send_Data[4] = (c3 >> 8);
//    CAN_Send_Data[5] = c3;
//    CAN_Send_Data[6] = (c4 >> 8);
//    CAN_Send_Data[7] = c4;

//    while (!((_hcan->State == HAL_CAN_STATE_READY) || (_hcan->State == HAL_CAN_STATE_LISTENING)))
//    {
//    }
//    while (HAL_CAN_GetTxMailboxesFreeLevel(_hcan) == 0) //如果三个邮箱都阻塞了就等一会儿，直到其中某个邮箱空闲
//    {
//    }
//    /* Check Tx Mailbox 1 status */
//    if ((_hcan->Instance->TSR & CAN_TSR_TME0) != 0U)
//    {
//        send_mail_box = CAN_TX_MAILBOX0;
//    }
//    /* Check Tx Mailbox 1 status */
//    else if ((_hcan->Instance->TSR & CAN_TSR_TME1) != 0U)
//    {
//        send_mail_box = CAN_TX_MAILBOX1;
//    }

//    /* Check Tx Mailbox 2 status */
//    else if ((_hcan->Instance->TSR & CAN_TSR_TME2) != 0U)
//    {
//        send_mail_box = CAN_TX_MAILBOX2;
//    }
//    return HAL_CAN_AddTxMessage(_hcan, &TX_MSG, CAN_Send_Data, &send_mail_box);
//}
