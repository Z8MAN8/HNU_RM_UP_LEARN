//
// Created by 14685 on 2022/7/3.
//

#include "Gimbal.h"
#include <stdbool.h>
#include "controller.h"
#include "bsp_can.h"
#include "bsp_imu.h"
#include "bsp_uart.h"
#include "Detect.h"
#include "ramp.h"


#include "keyboard.h"

/*云台归中值*/
int32_t   pit_center_offset = 3616;
int32_t   yaw_center_offset = 4600;

/*储存IMU传感器相关的数据*/
ImuTypeDef imu;

GimbalBackType gimbal_back_step;

First_Order_Filter_t mouse_y_lpf,mouse_x_lpf;

/* 云台相对角度,unit: degree*/
float     pit_relative_angle;
volatile float  yaw_relative_angle;

/* gimbal pid parameter */
float yaw_angle_fdb = 0;
float pit_angle_fdb = 0;
float c[3] = {0};
/* 云台电机期望角度(degree) */
float yaw_angle_ref;
float pit_angle_ref;

bool_t recv_flag=false;   //虚拟串口接收标志位
static _Bool auto_pid_flag = 1;
static _Bool manual_pid_flag = 0;





void gimbal_task(void const * argument)
{
    /*初始化云台控制参数*/
    Gimbal_Init_param();
    /*获取PreviousWakeTime*/
    uint32_t Gimbal_Wake_time = osKernelSysTick();

    while (1)
    {
        /*获取云台传感器及控制信息*/
        Gimbal_Get_information();

        switch (gim.ctrl_mode)
        {
            case (GIMBAL_INIT):
            {
                Gimbal_Init_handle();
            }break;

            case (GIMBAL_CLOSE_LOOP_ZGYRO):
            {
                Gimbal_Loop_handle();
            }break;

            case (GIMBAL_AUTO):
            {
                Gimbal_Auto_control();
            }break;

            default:
            {
                Gimbal_Relax_handle();
            }break;

        }





        /*绝对延时，保证Gimbal_TASK固定周期运行*/
        osDelayUntil(&Gimbal_Wake_time, GIMBAL_PERIOD);
    }



}


void Gimbal_Get_information(void)
{
    /*获取IMU数据*/
    Imu_Get_data(&imu);

    /*获取云台相对角度*/
    yaw_relative_angle = Gimbal_Get_relative_pos(YawMotor.RawAngle, yaw_center_offset)/22.75f;
    pit_relative_angle = Gimbal_Get_relative_pos(YawMotor.RawAngle, yaw_center_offset)/22.75f;

    /*处理PC端键鼠控制*/
    PC_Handle_kb();

    /*获取云台当前模式*/
    Gimbal_Get_mode();

}


void Gimbal_Get_mode(void)
{
   /* gim.ac_mode = Remote_Is_action();
    gim.last_mode = gim.ctrl_mode;*/
   if(   glb_err.err_list[REMOTE_CTRL_OFFLINE].err_exist //遥控器离线
      || glb_err.err_list[GIMBAL_YAW_OFFLINE].err_exist  //yaw轴电机离线
      || glb_err.err_list[GIMBAL_PIT_OFFLINE].err_exist) //pitch轴电机离线
   {/*如果有模块离线，则云台为GIMBAL_RELAX模式*/
       gim.ctrl_mode = GIMBAL_RELAX;
       gim.last_mode = gim.ctrl_mode;
   }

   else{     /*无模块离线，进行下一步判断*/
       if(gim.last_mode == GIMBAL_RELAX)
       {   /*如果云台之前是RELAX状态，就进入GIMBAL_INIT*/
           gim.ctrl_mode = GIMBAL_INIT;
           gim.last_mode = gim.ctrl_mode;
       }

       else{   /*无模块离线，且初始化完毕，进行下一步判断*/
           switch (rc.sw2)
           {
               case (RC_MI):    //想要进入GIMBAL_CLOSE_LOOP_ZGYRO模式
               {
                   gim.ctrl_mode = GIMBAL_CLOSE_LOOP_ZGYRO;
               }break;

               case (RC_UP):    //想要进入GIMBAL_AUTO模式
               {
                   gim.ctrl_mode = GIMBAL_AUTO;
               }break;

               default:    //想要进入GIMBAL_RELAX模式
               {
                   gim.ctrl_mode = GIMBAL_RELAX;
               }break;
           }    /*完成遥控器拨杆的初步模式判断*/

           gim.last_mode = gim.ctrl_mode;
       }

       while (rc.mouse.r)
       {    /*长按鼠标右键，进入自瞄模式*/
           gim.ctrl_mode = GIMBAL_AUTO;
           gim.last_mode = gim.ctrl_mode;
       }

   }
}



int16_t Gimbal_Get_relative_pos(int16_t raw_ecd, int16_t center_offset)
{
    int16_t tmp = 0;
    if (center_offset >= 4096)
    {
        if (raw_ecd > center_offset - 4096)
            tmp = raw_ecd - center_offset;
        else
            tmp = raw_ecd + 8192 - center_offset;
    }
    else
    {
        if (raw_ecd > center_offset + 4096)
            tmp = raw_ecd - 8192 - center_offset;
        else
            tmp = raw_ecd - center_offset;
    }
    return tmp;
}

/*static ActionModeType Remote_Is_action(void)
{
    if ((abs(rc.ch1) >= 10)
        || (abs(rc.ch2) >= 10)
        || (abs(rc.ch3) >= 10)
        || (abs(rc.ch4) >= 10)
        || (abs(rc.mouse.x) >= 5)
        || (abs(rc.mouse.y) >= 5))
    {
        return IS_ACTION;
    }
    else
    {
        return NO_ACTION;
    }
}*/


/*云台初始化处理函数*/
void Gimbal_Init_handle(void)
{
    /*限制云台初始化归中时的相关参数*/
    Gimbal_Back_param();

    pit_angle_fdb = pit_relative_angle;
    yaw_angle_fdb = yaw_relative_angle;

    /* gimbal pitch back center */
    pit_angle_ref = pit_relative_angle * (1 - ramp_calc(&pit_ramp));

    switch (gimbal_back_step)
    {
        //在pitch轴没有回中完成之前不会进行yaw轴的回中
        case PIT_BACK_STEP:
        {
            /* keep yaw unmove this time */
            yaw_angle_ref = gim.ecd_offset_angle;

            if(fabs(pit_angle_fdb) <= 2.0f)
                gimbal_back_step = YAW_BACK_STEP;
        }break;

        case YAW_BACK_STEP:
        {
            /* yaw back center after pitch arrive */
            yaw_angle_ref = yaw_relative_angle * ( 1 - ramp_calc(&yaw_ramp));

            if (fabs(yaw_angle_fdb) <= 0.5f)
                gimbal_back_step = BACK_IS_OK;
        }break;

        case BACK_IS_OK:
        {
            /* yaw arrive and switch gimbal state */
            /*if(rc.sw2 == RC_MI)
                gim.ctrl_mode = GIMBAL_CLOSE_LOOP_ZGYRO;
            else if(rc.sw2 == RC_UP)
                gim.ctrl_mode = GIMBAL_AUTO;*/


            /*初始化完毕后，将限制云台归中的相关参数恢复正常*/
            gim.yaw_offset_angle = imu.angle_x;
            gim.pit_offset_angle = imu.angle_y;
            pit_angle_ref = 0;
            yaw_angle_ref = 0;
            YawMotor.PID_Velocity.MaxOut=YAW_V_PID_MAXOUT_M;
            YawMotor.FFC_Velocity.MaxOut=YAW_V_FFC_MAXOUT;
            PitMotor.PID_Velocity.MaxOut=PITCH_V_PID_MAXOUT_M;
            //pid_pit_speed.max_output = 8000;
        }break;
    }
}

/*云台跟随编码器闭环控制处理函数*/
void Gimbal_Loop_handle()
{
    if(manual_pid_flag == 0){
        PID_Reset_manual();
        manual_pid_flag = 1;
        auto_pid_flag = 0;
    }

    pit_angle_fdb = imu.angle_y-gim.pit_offset_angle;
    yaw_angle_fdb = imu.angle_x - gim.yaw_offset_angle;

        Gimbal_Control_yaw();
        Gimbal_Control_pitch();

        //限制pitch轴的活动角度
        if ((pit_relative_angle >= PIT_ANGLE_MIN) && (pit_relative_angle <= PIT_ANGLE_MAX))
        {
            VAL_LIMIT(pit_angle_ref, PIT_ANGLE_MIN, PIT_ANGLE_MAX);
        }
}


void Gimbal_Control_yaw(void)
{
    float fx=First_Order_Filter_Calculate(&mouse_x_lpf,rc.mouse.x);
    //yaw轴的角度累加，单位degree
    yaw_angle_ref += -rc.ch3 * RC_RATIO * GIMBAL_RC_MOVE_RATIO_YAW
                     -fx * KB_RATIO * GIMBAL_PC_MOVE_RATIO_YAW;

}

void Gimbal_Control_pitch(void)
{
    float fy=First_Order_Filter_Calculate(&mouse_y_lpf,rc.mouse.y);
    //pitch轴的角度累加，单位degree
    pit_angle_ref += rc.ch4 * RC_RATIO * GIMBAL_RC_MOVE_RATIO_PIT
                     - fy * KB_RATIO * GIMBAL_PC_MOVE_RATIO_PIT;
}


//TODO:移植自瞄相关，修改各种结构体，枚举等规范
void Gimbal_Auto_control(void)
{
    if(auto_pid_flag == 0){
        PID_Reset_auto();
        auto_pid_flag = 1;
        manual_pid_flag = 0;
    }

    //	float rate=0.3,a,b;
    static float last_p=0.0f,last_y=0.0f;
    float fx=0.0;

    fx=First_Order_Filter_Calculate(&mouse_x_lpf,rc.mouse.x);
    //yaw轴的角度累加，单位degree
    static float manual_offset =0.0;
    manual_offset+=-rc.ch3 * RC_RATIO * GIMBAL_RC_MOVE_RATIO_YAW
                   -fx * KB_RATIO * GIMBAL_PC_MOVE_RATIO_YAW;

//	HAL_GPIO_TogglePin(GPIOG,GPIO_PIN_2);
    if(recv_flag) {
        pit_angle_ref = data_recv.pitchAngleSet * 0.7f + last_p * 0.3f;
        yaw_angle_ref = data_recv.yawAngleSet + manual_offset;
    }
    //遥控器微调
//    gimbal_yaw_control();
    //gimbal_pitch_control();
    //pit_angle_ref=pit_relative_angle+b;

    //限制pit轴的活动角度
    if ((pit_angle_ref >= PIT_ANGLE_MAX) && (pit_angle_ref <= PIT_ANGLE_MIN))
    {
        VAL_LIMIT(pit_angle_ref, PIT_ANGLE_MIN, PIT_ANGLE_MAX);
    }
    if ((yaw_angle_ref >= 170) && (yaw_angle_ref <= -170))
    {
        VAL_LIMIT(yaw_angle_ref, -170, 170);
    }
    last_p=data_recv.pitchAngleSet;
    last_y=data_recv.yawAngleSet;
//    //计算pitch轴相对角度差
    pit_angle_fdb = pit_relative_angle;
//    //计算yaw轴相对角度差
//    yaw_angle_fdb = yaw_relative_angle;
    //尝试在自瞄时也使用IMU
//    pit_angle_fdb = imu.angle_y-gim.pit_offset_angle;
    yaw_angle_fdb = imu.angle_x - gim.yaw_offset_angle;
}


/*云台离线处理，发送零电流*/
void Gimbal_Relax_handle(void)
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

    write_can(CONTROL_CAN, CAN_GIMBAL_ID, data);
}



void Gimbal_Back_param(void)
{
    gimbal_back_step = PIT_BACK_STEP;
    gim.ecd_offset_angle = yaw_relative_angle;
    YawMotor.PID_Velocity.MaxOut=YAW_V_PID_MAXOUT_M_INIT;
    YawMotor.FFC_Velocity.MaxOut=YAW_V_FFC_MAXOUT_INIT;
    PitMotor.PID_Velocity.MaxOut=PITCH_V_PID_MAXOUT_INIT_M;
    ramp_init(&pit_ramp, BACK_CENTER_TIME/GIMBAL_PERIOD);
    ramp_init(&yaw_ramp, BACK_CENTER_TIME/GIMBAL_PERIOD);

    //pid_pit_speed.max_output = 15000;
}


void Gimbal_Init_param(void)
{

    /* 云台pitch轴电机PID参数初始化 */
    PID_Init(&PitMotor.PID_Velocity, PITCH_V_PID_MAXOUT_INIT_M, PITCH_V_PID_MAXINTEGRAL_M, 0,
             PITCH_V_PID_KP_M, PITCH_V_PID_KI_M, PITCH_V_PID_KD_M, 1000, 5000, PITCH_V_PID_LPF_M, PITCH_V_PID_D_LPF_M, 0,
             Integral_Limit | Trapezoid_Intergral | OutputFilter | DerivativeFilter);
    c[0] = PITCH_V_FCC_C0_M;
    c[1] = PITCH_V_FCC_C1_M;
    c[2] = PITCH_V_FCC_C2_M;
    Feedforward_Init(&PitMotor.FFC_Velocity, PITCH_V_FFC_MAXOUT_M, c, PITCH_V_FCC_LPF_M, 4, 4);
//    LDOB_Init(&PitMotor.LDOB, 30000 * 0, 0.1, c, 0.00001, 4, 4);
    PID_Init(&PitMotor.PID_Angle, PITCH_A_PID_MAXOUT_M, PITCH_A_PID_MAXINTEGRAL_M, 0,
             PITCH_A_PID_KP_M, PITCH_A_PID_KI_M, PITCH_A_PID_KD_M, 5, 2, PITCH_A_PID_LPF_M, PITCH_A_PID_D_LPF_M, 0,
             Integral_Limit | Trapezoid_Intergral| DerivativeFilter|Derivative_On_Measurement);
    c[0] = PITCH_A_FCC_C0_M;
    c[1] = PITCH_A_FCC_C1_M;
    c[2] = PITCH_A_FCC_C2_M;
    Feedforward_Init(&PitMotor.FFC_Angle, PITCH_A_FFC_MAXOUT_M, c, PITCH_A_FCC_LPF_M, 3, 3);
    PitMotor.Max_Out = PITCH_MOTOR_MAXOUT * 0.9f;

    /* 云台yaw轴电机PID参数初始化 */
    PID_Init(&YawMotor.PID_Velocity, YAW_V_PID_MAXOUT_M_INIT, YAW_V_PID_MAXINTEGRAL_M, 0,
             YAW_V_PID_KP_M, YAW_V_PID_KI_M, YAW_V_PID_KD_M, 1000, 5000,
             YAW_V_PID_LPF_M, YAW_V_PID_D_LPF_M, 0,
             Integral_Limit | Trapezoid_Intergral | DerivativeFilter);
    c[0] = YAW_V_FCC_C0;
    c[1] = YAW_V_FCC_C1;
    c[2] = YAW_V_FCC_C2;
    Feedforward_Init(&YawMotor.FFC_Velocity, YAW_V_FFC_MAXOUT_INIT, c, YAW_V_FCC_LPF, 4, 4);
//    LDOB_Init(&YawMotor.LDOB, 30000 * 0, 0.1, c, 0.00001, 4, 4);

    c[0] = YAW_A_FCC_C0;
    c[1] = YAW_A_FCC_C1;
    c[2] = YAW_A_FCC_C2;
    Feedforward_Init(&YawMotor.FFC_Angle, YAW_A_FFC_MAXOUT, c, YAW_A_FCC_LPF, 3, 3);
    PID_Init(&YawMotor.PID_Angle, YAW_A_PID_MAXOUT_M, YAW_A_PID_MAXINTEGRAL_M, 0.0,
             YAW_A_PID_KP_M, YAW_A_PID_KI_M, YAW_A_PID_KD_M, 5, 2, 0, 0, 0,
             Integral_Limit | Trapezoid_Intergral);
    YawMotor.Max_Out = YAW_MOTOR_MAXOUT * 0.9f;

    First_Order_Filter_Init(&mouse_x_lpf,0.014,0.1);
    First_Order_Filter_Init(&mouse_y_lpf,0.014,0.1);
    /* 将云台的初始化状态设置为释放 */
    gim.ctrl_mode = GIMBAL_RELAX;
}


void PID_Reset_manual()
{
    PID_Init(&YawMotor.PID_Velocity, YAW_V_PID_MAXOUT_M, YAW_V_PID_MAXINTEGRAL_M, 0,
             YAW_V_PID_KP_M, YAW_V_PID_KI_M, YAW_V_PID_KD_M, 1000, 5000,
             YAW_V_PID_LPF_M, YAW_V_PID_D_LPF_M, 0,
             Integral_Limit | Trapezoid_Intergral);
    PID_Init(&YawMotor.PID_Angle, YAW_A_PID_MAXOUT_M, YAW_A_PID_MAXINTEGRAL_M, 0.0,
             YAW_A_PID_KP_M, YAW_A_PID_KI_M, YAW_A_PID_KD_M, 5, 2, 0, 0, 0,
             Integral_Limit | Trapezoid_Intergral);
    PID_Init(&PitMotor.PID_Velocity, PITCH_V_PID_MAXOUT_M, PITCH_V_PID_MAXINTEGRAL_M, 0,
             PITCH_V_PID_KP_M, PITCH_V_PID_KI_M, PITCH_V_PID_KD_M, 1000, 5000,
             PITCH_V_PID_LPF_M, PITCH_V_PID_D_LPF_M, 0,
             Integral_Limit | Trapezoid_Intergral);
    PID_Init(&PitMotor.PID_Angle, PITCH_A_PID_MAXOUT_M, PITCH_A_PID_MAXINTEGRAL_M, 0.0,
             PITCH_A_PID_KP_M, PITCH_A_PID_KI_M, PITCH_A_PID_KD_M, 5, 2, 0, 0, 0,
             Integral_Limit | Trapezoid_Intergral);
    c[0] = PITCH_V_FCC_C0_M,
            c[1] = PITCH_V_FCC_C1_M,
            c[2] = PITCH_V_FCC_C2_M,
            Feedforward_Init(&PitMotor.FFC_Velocity, PITCH_V_FFC_MAXOUT_M, c, PITCH_V_FCC_LPF_M, 4, 4);
//    LDOB_Init(&PitMotor.LDOB, 30000 * 0, 0.1, c, 0.00001, 4, 4);
    PID_Init(&PitMotor.PID_Angle, PITCH_A_PID_MAXOUT_M, PITCH_A_PID_MAXINTEGRAL_M, 0,
             PITCH_A_PID_KP_M, PITCH_A_PID_KI_M, PITCH_A_PID_KD_M, 5, 2, PITCH_A_PID_LPF_M, PITCH_A_PID_D_LPF_M, 0,
             Integral_Limit | Trapezoid_Intergral| DerivativeFilter|Derivative_On_Measurement);
    c[0] = PITCH_A_FCC_C0_M,
            c[1] = PITCH_A_FCC_C1_M,
            c[2] = PITCH_A_FCC_C2_M,
            Feedforward_Init(&PitMotor.FFC_Angle, PITCH_A_FFC_MAXOUT_M, c, PITCH_A_FCC_LPF_M, 3, 3);
}


void PID_Reset_auto()
{
//	YawMotor.PID_Velocity.MaxOut=YAW_V_PID_MAXOUT_A;
//	YawMotor.PID_Velocity.IntegralLimit=YAW_V_PID_MAXINTEGRAL_A;
//	YawMotor.PID_Velocity.Kp=YAW_V_PID_KP_A;
//	YawMotor.PID_Velocity.Ki=YAW_V_PID_KI_A;
//	YawMotor.PID_Velocity.Kd=YAW_V_PID_KD_A;
//
//	YawMotor.PID_Angle.MaxOut=YAW_A_PID_MAXOUT_A;
//	YawMotor.PID_Angle.IntegralLimit=YAW_A_PID_MAXINTEGRAL_A;
//	YawMotor.PID_Angle.Kp=YAW_A_PID_KP_A;
//	YawMotor.PID_Angle.Ki=YAW_A_PID_KI_A;
//	YawMotor.PID_Angle.Kd=YAW_A_PID_KD_A;
    PID_Init(&YawMotor.PID_Velocity, YAW_V_PID_MAXOUT_A, YAW_V_PID_MAXINTEGRAL_A, 0,
             YAW_V_PID_KP_A, YAW_V_PID_KI_A, YAW_V_PID_KD_A, 1000, 5000,
             YAW_V_PID_LPF_A, YAW_V_PID_D_LPF_A, 0,
             Integral_Limit | Trapezoid_Intergral);
    PID_Init(&YawMotor.PID_Angle, YAW_A_PID_MAXOUT_A, YAW_A_PID_MAXINTEGRAL_A, 0.0,
             YAW_A_PID_KP_A, YAW_A_PID_KI_A, YAW_A_PID_KD_A, 5, 2, 0, 0, 0,
             Integral_Limit | Trapezoid_Intergral);
    PID_Init(&PitMotor.PID_Velocity, PITCH_V_PID_MAXOUT_A, PITCH_V_PID_MAXINTEGRAL_A, 0,
             PITCH_V_PID_KP_A, PITCH_V_PID_KI_A, PITCH_V_PID_KD_A, 1000, 5000,
             PITCH_V_PID_LPF_A, PITCH_V_PID_D_LPF_A, 0,
             Integral_Limit | Trapezoid_Intergral);
    PID_Init(&PitMotor.PID_Angle, PITCH_A_PID_MAXOUT_A, PITCH_A_PID_MAXINTEGRAL_A, 0.0,
             PITCH_A_PID_KP_A, PITCH_A_PID_KI_A, PITCH_A_PID_KD_A, 5, 2, 0, 0, 0,
             Integral_Limit | Trapezoid_Intergral);
    c[0] = PITCH_V_FCC_C0_A;
    c[1] = PITCH_V_FCC_C1_A;
    c[2] = PITCH_V_FCC_C2_A;
    Feedforward_Init(&PitMotor.FFC_Velocity, PITCH_V_FFC_MAXOUT_A, c, PITCH_V_FCC_LPF_A, 4, 4);
//    LDOB_Init(&PitMotor.LDOB, 30000 * 0, 0.1, c, 0.00001, 4, 4);
    PID_Init(&PitMotor.PID_Angle, PITCH_A_PID_MAXOUT_A, PITCH_A_PID_MAXINTEGRAL_A, 0,
             PITCH_A_PID_KP_A, PITCH_A_PID_KI_A, PITCH_A_PID_KD_A, 5, 2, PITCH_A_PID_LPF_A, PITCH_A_PID_D_LPF_A, 0,
             Integral_Limit | Trapezoid_Intergral| DerivativeFilter|Derivative_On_Measurement);
    c[0] = PITCH_A_FCC_C0_A;
    c[1] = PITCH_A_FCC_C1_A;
    c[2] = PITCH_A_FCC_C2_A;
    Feedforward_Init(&PitMotor.FFC_Angle, PITCH_A_FFC_MAXOUT_A, c, PITCH_A_FCC_LPF_A, 3, 3);
}