//
// Created by 14685 on 2022/7/3.
//

#include "Gimbal.h"
#include <stdbool.h>
#include "controller.h"
#include "bsp_can.h"
#include "bsp_uart.h"
#include "Detect.h"
#include "ramp.h"
#include "kalman.h"
#include "keyboard.h"
#include "sys.h"

#include "bsp_dwt.h"

float dt = 0.0;//待初始化
float task_dt = 1.0;//待初始化

//float yaw_a_kp = 30;
//float yaw_a_ki = 200;
//float yaw_a_kd = 0.001;
//float pitch_a_kp = 90;
//float pitch_a_ki = 180;
//float pitch_a_kd = 1;
float yaw_a_kp = 30;//云台偏航角、俯仰角参数设置
float yaw_a_ki = 200;
float yaw_a_kd = 0.001;
float pitch_a_kp = 90;
float pitch_a_ki = 180;
float pitch_a_kd = 1;

float Ballistic_compensation_mannul;  //对自瞄数据进行手动弹道补偿
static float gimbal_yaw = 0;
static float gimbal_pitch = 0;  //解析上位机发送的云台角度
static float yaw_speed = 0;
static float pitch_speed = 0;
static float roll_speed = 0;    //解析上位机发送的目标移动速度
int16_t yaw_moto_current_manual = 0;
int16_t yaw_moto_current_auto = 0;
int16_t pit_moto_current_manual = 0;
int16_t pit_moto_current_auto = 0;
//TODO:考虑将noaction处理函数和飞坡前的对准操作合在一起

ImuTypeDef imu;    //储存IMU传感器相关的数据
GimbalBackType gimbal_back_step;
GimbalYawTypeDef gim;

/*云台归中值*/
int32_t   pit_center_offset = 3616;
#ifdef SIDEWAYS
int32_t   yaw_center_offset = 5426;
#else
int32_t   yaw_center_offset = 4600;
#endif

/*储存鼠标坐标数据*/
First_Order_Filter_t mouse_y_lpf,mouse_x_lpf;

/* 云台相对角度,unit: degree*/
float     pit_relative_angle;
volatile float  yaw_relative_angle;

/* 上板发送给下板的数据之一 */
volatile float yaw_angle_ref_v;

/* gimbal pid parameter */
float yaw_angle_fdb = 0;
float pit_angle_fdb = 0;
float c[3] = {0};

/* 云台电机期望角度(degree) */
float yaw_angle_ref =0;
float pit_angle_ref=0;

/* 云台电机电流 */
int16_t *yaw_moto_current = &yaw_moto_current_manual;
int16_t *pit_moto_current = &pit_moto_current_manual;

bool_t recv_flag=false;   //虚拟串口接收标志位
/*切换手动和自动模式相应PID参数的标志位*/
static _Bool auto_pid_flag = 0;
static _Bool manual_pid_flag = 1;

//存放25帧历史姿态数据
float angle_history[50];





void gimbal_task(void const * argument){
    /*初始化云台控制参数*/
    Gimbal_Init_param();//分别初始化云台pitch、yaw角手动和自动的前馈控制、pid参数
    /*获取PreviousWakeTime*/
    uint32_t Gimbal_Wake_time = osKernelSysTick();//记录任务开始滴答计数值
    DWT_Init(168);//再次初始化DWT外设？？？（main函数中已初始化）

    while (1){
        /*获取云台传感器及控制信息*/
        Gimbal_Get_information();
        //问题：获取IMU数据时是否会产生任务冲突？？？
        //猜想：ins任务中ins.accel数据是写入，本任务中是读取，可能需要一个互斥量或信号量的辅助

        //已解决：不冲突
        switch (gim.ctrl_mode){//任务控制模式的判断与执行

            case (GIMBAL_INIT):{
                Gimbal_Init_handle();
            }break;

            case (GIMBAL_CLOSE_LOOP_ZGYRO):{
                Gimbal_Loop_handle();
            }break;

            case (GIMBAL_AUTO):{
                Gimbal_Auto_control();
            }break;

            case (GIMBAL_RELAX):{
                Gimbal_Relax_handle();
            }break;
        }
        if(gim.ctrl_mode!=GIMBAL_RELAX){
            Gimbal_Control_moto();
        }



        /*绝对延时，保证Gimbal_TASK固定周期运行*/
        osDelayUntil(&Gimbal_Wake_time, GIMBAL_PERIOD);//等待一个时钟节拍的时间，1ms
    }
}


void Gimbal_Get_information(void){
    /*获取IMU数据*/
    IMU_Get_data(&imu);//imu相关数据赋值

    /*获取云台相对角度*/
    //问题：计算公式、Gimbal_Get_relative_pos函数的具体计算
    //yaw_center_offset为固定值？？？

    //已解决！！！
    yaw_relative_angle = Gimbal_Get_relative_pos(YawMotor_Manual.RawAngle, yaw_center_offset) / 22.75f;
    pit_relative_angle = Gimbal_Get_relative_pos(PitMotor_Manual.RawAngle, pit_center_offset) / 22.75f;

    /*处理PC端键鼠控制*/
    PC_Handle_kb();//这个可以放到串口空闲中断吗？ keyboard在UserLib文件夹中

    /*获取云台当前模式，这里只能判断是RELAX还是INIT模式*/
    // 因为其他模式的判断在Gimbal_Init_handle()里面做
    Gimbal_Get_mode();
}

//  云台状态机逻辑：
//  - Gimbal_Get_mode：获取云台模式，先进行一个总体的错误判断。如果云台出现以下错误：1. 遥控器离线 2. yaw轴或pitch轴电机离线 以及云台被手动调整为relax模式时，切换/维持RELAX状态。同时，如果当前状态为GIMBAL_RELAX，
//    则直接切换模式位GIMBAL_INIT（反正问题/控制状态还持续的话下一轮又变成RELAX了
//  - 可以理解成，RELAX和INIT都是云台的准备状态，剩下两个GIMBAL_CLOSE_LOOP_ZGYRO、GIMBAL_AUTO才是云台的正常运行状态

void Gimbal_Get_mode(void){
    /* gim.ac_mode = Remote_Is_action();
     gim.last_mode = gim.ctrl_mode;*/
    if(   glb_err.err_list[REMOTE_CTRL_OFFLINE].err_exist //遥控器离线
          || glb_err.err_list[GIMBAL_YAW_OFFLINE].err_exist  //yaw轴电机离线
          || glb_err.err_list[GIMBAL_PIT_OFFLINE].err_exist//pitch轴电机离线
          || rc.sw2 ==RC_DN){/*如果有模块离线或右侧拨杆值在DN时，则云台为GIMBAL_RELAX模式*/
        gim.ctrl_mode = GIMBAL_RELAX;
    }

    else if(gim.ctrl_mode == GIMBAL_RELAX){
        gim.ctrl_mode = GIMBAL_INIT;
    }

    gim.last_mode = gim.ctrl_mode;//猜想：赋值位置是否有问题，last_mode赋值是否应该在ctrl_mode之前。
}



int16_t Gimbal_Get_relative_pos(int16_t raw_ecd, int16_t center_offset){
    int16_t tmp = 0;
    if (center_offset >= 4096){
        if (raw_ecd > center_offset - 4096)
            tmp = raw_ecd - center_offset;
        else
            tmp = raw_ecd + 8192 - center_offset;
    }
    else{
        if (raw_ecd > center_offset + 4096)
            tmp = raw_ecd - 8192 - center_offset;
        else
            tmp = raw_ecd - center_offset;
    }
    return tmp;
}

//  包含云台回中处理和init后根据控制器信号选择手动或自动控制的逻辑
//  - gimbal_back_step 并不是一旦为 BACK_IS_OK 就不会再变，不会再次进入回中步骤了。Gimbal_Relax_handle() 中会调用 Gimbal_Back_param() 给他重新赋初始值。
//    也就是说每进入relax状态一次，后续都会重新回中
//  - 此时xxx_angle_fdb不是用imu，而是用编码器算出来的（没有imu精确），因为imu的offset要在归中后得到，然后才可以开始用imu

/*云台初始化处理函数*/
void Gimbal_Init_handle(void){

    pit_angle_fdb = pit_relative_angle;
    yaw_angle_fdb = yaw_relative_angle;

    /* gimbal pitch back center */
    //云台俯仰回中
    pit_angle_ref = pit_relative_angle * (1 - ramp_calc(&pit_ramp));//用到俯仰角

    switch (gimbal_back_step){//回中一步一步进行
        //在pitch轴没有回中完成之前不会进行yaw轴的回中
        case PIT_BACK_STEP:{
            /* keep yaw unmove this time */
            yaw_angle_ref = gim.ecd_offset_angle;

            if(fabs(pit_angle_fdb) <= 2.0f)//误差较小时？？？
                gimbal_back_step = YAW_BACK_STEP;
        }break;

        case YAW_BACK_STEP:{
            /* yaw back center after pitch arrive */
            yaw_angle_ref = yaw_relative_angle * ( 1 - ramp_calc(&yaw_ramp));

            if (fabs(yaw_angle_fdb) <= 0.5f)
                gimbal_back_step = BACK_IS_OK;
        }break;

        case BACK_IS_OK:{
            /* yaw arrive and switch gimbal state */
            if(rc.sw2 == RC_UP){//云台归中后通过遥控器，选择云台模式：GIMBAL_CLOSE_LOOP_ZGYRO、GIMBAL_AUTO
                gim.ctrl_mode = GIMBAL_CLOSE_LOOP_ZGYRO;
            }
            else if (rc.sw2 == RC_MI){
                gim.ctrl_mode = GIMBAL_AUTO;
            }

            gim.yaw_offset_angle = imu.angle_x;//偏航和俯仰角偏移量赋值
            gim.pit_offset_angle = imu.angle_y;
            //猜想：归中时，陀螺仪偏航、俯仰角定义为0°
            pit_angle_ref = 0;
            yaw_angle_ref = 0;
            /*云台归中完成，将各参数设回正常值*/
            //自动和手动最大值赋值
            YawMotor_Manual.PID_Velocity.MaxOut=YAW_V_PID_MAXOUT_M;
            YawMotor_Manual.FFC_Velocity.MaxOut=YAW_V_FFC_MAXOUT;
            PitMotor_Manual.PID_Velocity.MaxOut=PITCH_V_PID_MAXOUT_M;

            YawMotor_Auto.PID_Velocity.MaxOut=YAW_V_PID_MAXOUT_M;
            YawMotor_Auto.FFC_Velocity.MaxOut=YAW_V_FFC_MAXOUT;
            PitMotor_Auto.PID_Velocity.MaxOut=PITCH_V_PID_MAXOUT_M;

        }break;
    }
}

/*云台跟随编码器闭环控制处理函数*/
void Gimbal_Loop_handle(){
    if(recv_flag) {  //欧拉角rpy方式控制

        //绝对角度与相对角度控制区别：相对角度加入变量Ballistic_compensation_mannul

        if (!rpy_rx_data.DATA[0]){     //绝对角度控制
            gimbal_yaw = *(int32_t*)&rpy_rx_data.DATA[1] / 1000.0;
            /*(int32_t)(rpy_rx_data.DATA[4] << 24 | rpy_rx_data.DATA[3] << 16
                           | rpy_rx_data.DATA[2] << 8 | rpy_rx_data.DATA[1])/1000;*/
            gimbal_pitch = *(int32_t*)&rpy_rx_data.DATA[5] / 1000.0;
            /*(int32_t)(rpy_rx_data.DATA[8] << 24 | rpy_rx_data.DATA[7] << 16
                             | rpy_rx_data.DATA[6] << 8 | rpy_rx_data.DATA[5])/1000;*/
        }
        else{     //相对角度控制
            gimbal_yaw = (*(int32_t*)&rpy_rx_data.DATA[1] / 1000.0)/* + pit_angle_fdb*/;
            /*((int32_t)(rpy_rx_data.DATA[4] << 24 | rpy_rx_data.DATA[3] << 16
                            | rpy_rx_data.DATA[2] << 8 | rpy_rx_data.DATA[1])/1000) + pit_angle_fdb;*/
            gimbal_pitch = (*(int32_t*)&rpy_rx_data.DATA[5] / 1000.0)
                           - Ballistic_compensation_mannul * KB_RATIO * GIMBAL_PC_MOVE_RATIO_PIT * MANUAL_OFFSET_PIT/* + yaw_angle_fdb*/;
            /*((int32_t)(rpy_rx_data.DATA[8] << 24 | rpy_rx_data.DATA[7] << 16
                              | rpy_rx_data.DATA[6] << 8 | rpy_rx_data.DATA[5])/1000) + yaw_angle_fdb;*/
        }
        yaw_speed   = *(int32_t*)&rpy_rx_data.DATA[13] / 1000.0;
        pitch_speed = *(int32_t*)&rpy_rx_data.DATA[17] / 1000.0;
        roll_speed  = *(int32_t*)&rpy_rx_data.DATA[21] / 1000.0;
    }
    /*普通模式中与自瞄模式的相互切换*/
    if(rc.sw2==RC_MI||rc.mouse.r==1){
        gim.ctrl_mode = GIMBAL_AUTO;
        gim.yaw_offset_angle = imu.angle_x;
        yaw_angle_ref = 0;
        yaw_angle_fdb = 0;
    }
        /*切换完毕，进入普通模式的控制*/
    else{
        if(manual_pid_flag == 0){
//            PID_Reset_manual();
            manual_pid_flag = 1;
            auto_pid_flag = 0;
        }

        pit_angle_fdb = imu.angle_y - gim.pit_offset_angle;
        yaw_angle_fdb = imu.angle_x - gim.yaw_offset_angle;

        Gimbal_Control_yaw();
        Gimbal_Control_pitch();

        //限制pitch轴的活动角度
        if ((pit_relative_angle >= PIT_ANGLE_MIN) && (pit_relative_angle <= PIT_ANGLE_MAX)){
            VAL_LIMIT(pit_angle_ref, PIT_ANGLE_MIN, PIT_ANGLE_MAX);
        }
    }
}


void Gimbal_Control_yaw(void){
    float fx=First_Order_Filter_Calculate(&mouse_x_lpf,rc.mouse.x);
    //yaw轴的角度累加，单位degree
    yaw_angle_ref += -rc.ch3 * RC_RATIO * GIMBAL_RC_MOVE_RATIO_YAW
                     -fx * KB_RATIO * GIMBAL_PC_MOVE_RATIO_YAW;
}

void Gimbal_Control_pitch(void){
    float fy=First_Order_Filter_Calculate(&mouse_y_lpf,rc.mouse.y);
    //pitch轴的角度累加，单位degree
    pit_angle_ref += rc.ch4 * RC_RATIO * GIMBAL_RC_MOVE_RATIO_PIT
                     - fy * KB_RATIO * GIMBAL_PC_MOVE_RATIO_PIT;
}


void Gimbal_Auto_control(void){
//    static float gimbal_yaw = 0;
//    static float gimbal_pitch = 0;  //解析上位机发送的云台角度
    /*static float dt = 0.0;//待初始化
    static float task_dt = 1.0;//待初始化*/
    static uint32_t old_counter=0;
    static uint32_t old_counter_task=0;
    float target_distance = 0; //与识别目标的距离
//    static bool_t com_protect = 1; //为1时一帧数据处理完毕



    /*自瞄模式中与普通模式的相互切换*/
    if(rc.sw2==RC_UP && rc.mouse.r!=1){
        gim.ctrl_mode=GIMBAL_CLOSE_LOOP_ZGYRO;
        gim.yaw_offset_angle = imu.angle_x;
        yaw_angle_ref = 0;
        yaw_angle_fdb = 0;
        Ballistic_compensation_mannul = 0;
    }
    /*切换完毕，进入自瞄模式的控制*/
    else{
        if(auto_pid_flag == 0){
//            PID_Reset_auto();
            auto_pid_flag = 1;
            manual_pid_flag = 0;
        }

        Ballistic_compensation_mannul += First_Order_Filter_Calculate(&mouse_y_lpf,rc.mouse.y)*0.05;

        //	float rate=0.3,a,b;
        static float last_p=0.0f,last_y=0.0f;
        float fx=0.0;

        fx=First_Order_Filter_Calculate(&mouse_x_lpf,rc.mouse.x);
        //yaw轴的角度累加，单位degree
        static float manual_offset =0.0;
        manual_offset+=-rc.ch3 * RC_RATIO * GIMBAL_RC_MOVE_RATIO_YAW
                       -fx * KB_RATIO * GIMBAL_PC_MOVE_RATIO_YAW;

//	HAL_GPIO_TogglePin(GPIOG,GPIO_PIN_2);
//TODO:
        if(recv_flag) {  //欧拉角rpy方式控制
            if (!rpy_rx_data.DATA[0]){     //绝对角度控制
                gimbal_yaw = *(int32_t*)&rpy_rx_data.DATA[1] / 1000.0;
                        /*(int32_t)(rpy_rx_data.DATA[4] << 24 | rpy_rx_data.DATA[3] << 16
                                       | rpy_rx_data.DATA[2] << 8 | rpy_rx_data.DATA[1])/1000;*/
                gimbal_pitch = *(int32_t*)&rpy_rx_data.DATA[5] / 1000.0;
                        /*(int32_t)(rpy_rx_data.DATA[8] << 24 | rpy_rx_data.DATA[7] << 16
                                         | rpy_rx_data.DATA[6] << 8 | rpy_rx_data.DATA[5])/1000;*/
            }
            else {     //相对角度控制(目前自瞄的控制方式）
                gimbal_yaw = (*(int32_t *) &rpy_rx_data.DATA[1] / 1000.0)/* - yaw_angle_fdb*/;
                /*((int32_t)(rpy_rx_data.DATA[4] << 24 | rpy_rx_data.DATA[3] << 16
                                | rpy_rx_data.DATA[2] << 8 | rpy_rx_data.DATA[1])/1000) + pit_angle_fdb;*/
                gimbal_pitch = (*(int32_t *) &rpy_rx_data.DATA[5] / 1000.0)
                        - Ballistic_compensation_mannul * KB_RATIO * GIMBAL_PC_MOVE_RATIO_PIT/* - pit_angle_fdb*/;
                /*((int32_t)(rpy_rx_data.DATA[8] << 24 | rpy_rx_data.DATA[7] << 16
                                  | rpy_rx_data.DATA[6] << 8 | rpy_rx_data.DATA[5])/1000) + yaw_angle_fdb;*/
                /*if((360-abs(gimbal_yaw)) < abs(gimbal_yaw)){
                    if(gimbal_yaw > 0){
                        gimbal_yaw = -(360-abs(gimbal_yaw));
                    }
                    else
                        gimbal_yaw = (360-abs(gimbal_yaw));
                }*/
            }
            pit_angle_ref = gimbal_pitch /** 0.7f + last_p * 0.3f*/;
            yaw_angle_ref = gimbal_yaw /** 0.7f + last_p * 0.3f*/ /*+ manual_offset*/;
            target_distance = *(int32_t*)&rpy_rx_data.DATA[13] / 1000;  //获取目标距离
            recv_flag = 0;
            /*预留task_dt ，DWT测上位机数据接收间隔时间,*/
            task_dt = DWT_GetDeltaT(&old_counter_task);
            old_counter = DWT->CYCCNT;
            old_counter_task = DWT->CYCCNT;
            /*task_dt约为0.01*/

        } else
        {   /*预留dt ，DWT测控制间隔时间*/
            dt = DWT_GetDeltaT(&old_counter);
            old_counter = DWT->CYCCNT;
            /*dt约为0.001*/
        }
        //遥控器微调
//    gimbal_yaw_control();
        //gimbal_pitch_control();
        //pit_angle_ref=pit_relative_angle+b;

        //限制pit轴的活动角度
        if ((pit_angle_ref >= PIT_ANGLE_MAX) || (pit_angle_ref <= PIT_ANGLE_MIN)){
            VAL_LIMIT(pit_angle_ref, PIT_ANGLE_MIN, PIT_ANGLE_MAX);
        }
/*        if ((yaw_angle_ref >= 170) || (yaw_angle_ref <= -170)){
            VAL_LIMIT(yaw_angle_ref, -170, 170);
        }*/
//  自瞄时暂时依然使用编码器数据
//    //计算pitch轴相对角度差
        pit_angle_fdb = pit_relative_angle;
//    //计算yaw轴相对角度差
//    yaw_angle_fdb = yaw_relative_angle;
        //尝试在自瞄时也使用IMU
//    pit_angle_fdb = imu.angle_y-gim.pit_offset_angle;
        yaw_angle_fdb = imu.angle_x - gim.yaw_offset_angle;

        /*gimbal_pitch = 0;
        gimbal_yaw = 0;*/
    }

}

void Gimbal_Control_moto(void)
{
    yaw_moto_current_manual = Motor_Angle_Calculate(&YawMotor_Manual, yaw_angle_fdb, imu.gyro_z, yaw_angle_ref);
    yaw_moto_current_auto = Motor_Angle_Calculate(&YawMotor_Auto, yaw_angle_fdb, imu.gyro_z, yaw_angle_ref);

    /* pitch轴俯仰角度限制 */
    float delta=pit_relative_angle+pit_angle_ref-pit_angle_fdb;
    if(delta-PIT_ANGLE_MAX>0)
        pit_angle_ref=pit_angle_fdb+PIT_ANGLE_MAX-pit_relative_angle;
    else if(delta-PIT_ANGLE_MIN<0)
        pit_angle_ref=pit_angle_fdb-(PIT_ANGLE_MIN-pit_relative_angle);

    VAL_LIMIT(pit_angle_ref, PIT_ANGLE_MIN, PIT_ANGLE_MAX);
//  /* pitch轴预期速度计算，单位degree/s */
//  pit_speed_ref    = PID_Calc(&pid_pit, pit_angle_fdb, pit_angle_ref);    //degree
//  /* pitch轴电机电压计算 */

//	pit_moto_current = PID_Calc(&pid_pit_speed, imu.gyro_y, pit_speed_ref); //degree/s
//	//滤波
//	pit_moto_current = 0.5*last_current + 0.5*pit_moto_current;
//	last_current = pit_moto_current;
//pitch轴速度为gyro_x
    pit_moto_current_manual = Motor_Angle_Calculate(&PitMotor_Manual, pit_angle_fdb, imu.gyro_x, pit_angle_ref);
    pit_moto_current_auto = Motor_Angle_Calculate(&PitMotor_Auto, pit_angle_fdb, imu.gyro_x, pit_angle_ref);

    if(gim.ctrl_mode==GIMBAL_AUTO){
        /*yaw_moto_current = &yaw_moto_current_auto;
        pit_moto_current = &pit_moto_current_auto;*/
        /*yaw_moto_current = 0;
        pit_moto_current = 0;*/
        //问题：auto_pid_flag=1时没有做处理吗
        if(auto_pid_flag == 0){
            if(abs(yaw_moto_current_manual - yaw_moto_current_auto) > 10)
                yaw_moto_current = &yaw_moto_current_manual;
            if(abs(pit_moto_current_manual - pit_moto_current_auto) > 10)
                pit_moto_current = &pit_moto_current_manual;
        }
    }

    if(gim.ctrl_mode==GIMBAL_CLOSE_LOOP_ZGYRO){
        yaw_moto_current = &yaw_moto_current_manual;
        pit_moto_current = &pit_moto_current_manual;
        if(manual_pid_flag == 0){
            if(abs(yaw_moto_current_manual - yaw_moto_current_auto) > 10)
                yaw_moto_current = &yaw_moto_current_auto;
            if(abs(pit_moto_current_manual - pit_moto_current_auto) > 10)
                pit_moto_current = &pit_moto_current_auto;
        }
    }

    if(gim.ctrl_mode==GIMBAL_INIT){
        yaw_moto_current = &yaw_moto_current_manual;
        pit_moto_current = &pit_moto_current_manual;
    }

    //发送电流到云台电机电调
    GimbalMoto_Send_current(*yaw_moto_current, *pit_moto_current);
}


/*云台离线处理，发送零电流，将PID各参设为归中模式*/
void Gimbal_Relax_handle(void){
    static uint8_t data[8];
    Gimbal_Back_param();

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



void Gimbal_Back_param(void){
    gimbal_back_step = PIT_BACK_STEP;
    gim.ecd_offset_angle = yaw_relative_angle;
    YawMotor_Manual.PID_Velocity.MaxOut=YAW_V_PID_MAXOUT_M_INIT;
    YawMotor_Manual.FFC_Velocity.MaxOut=YAW_V_FFC_MAXOUT_INIT;
    PitMotor_Manual.PID_Velocity.MaxOut=PITCH_V_PID_MAXOUT_INIT_M;
    //TODO:考虑加入KI值
//    YawMotor_Manual.PID_Angle.Ki=80000;
    ramp_init(&pit_ramp, BACK_CENTER_TIME/GIMBAL_PERIOD);
    ramp_init(&yaw_ramp, BACK_CENTER_TIME/GIMBAL_PERIOD);
    //pid_pit_speed.max_output = 15000;
}


void Gimbal_Init_param(void){
    /*手动参数初始化*/
    /* 云台pitch轴电机PID参数初始化 */
    PID_Init(&PitMotor_Manual.PID_Velocity, PITCH_V_PID_MAXOUT_INIT_M, PITCH_V_PID_MAXINTEGRAL_M, 0,
             PITCH_V_PID_KP_M, PITCH_V_PID_KI_M, PITCH_V_PID_KD_M, 1000, 5000, PITCH_V_PID_LPF_M,
             PITCH_V_PID_D_LPF_M, 0,
             Integral_Limit | Trapezoid_Intergral | OutputFilter | DerivativeFilter);
    c[0] = PITCH_V_FCC_C0_M;
    c[1] = PITCH_V_FCC_C1_M;
    c[2] = PITCH_V_FCC_C2_M;
    //疑问：前馈控制预测云台俯仰角和偏航角的角度和速率？？？
    //前馈具体内容不懂
    //已解决：没有用到前馈
    Feedforward_Init(&PitMotor_Manual.FFC_Velocity, PITCH_V_FFC_MAXOUT_M, c, PITCH_V_FCC_LPF_M, 4, 4);
//    LDOB_Init(&PitMotor_Manual.LDOB, 30000 * 0, 0.1, c, 0.00001, 4, 4);
    PID_Init(&PitMotor_Manual.PID_Angle, PITCH_A_PID_MAXOUT_M, PITCH_A_PID_MAXINTEGRAL_M, 0,
             PITCH_A_PID_KP_M, PITCH_A_PID_KI_M, PITCH_A_PID_KD_M, 5, 2, PITCH_A_PID_LPF_M, PITCH_A_PID_D_LPF_M, 0,
             Integral_Limit | Trapezoid_Intergral | DerivativeFilter | Derivative_On_Measurement);
    c[0] = PITCH_A_FCC_C0_M;
    c[1] = PITCH_A_FCC_C1_M;
    c[2] = PITCH_A_FCC_C2_M;
    Feedforward_Init(&PitMotor_Manual.FFC_Angle, PITCH_A_FFC_MAXOUT_M, c, PITCH_A_FCC_LPF_M, 3, 3);
    PitMotor_Manual.Max_Out = PITCH_MOTOR_MAXOUT * 0.9f;

    /* 云台yaw轴电机PID参数初始化 */
    PID_Init(&YawMotor_Manual.PID_Velocity, YAW_V_PID_MAXOUT_M_INIT, YAW_V_PID_MAXINTEGRAL_M, 0,
             YAW_V_PID_KP_M, YAW_V_PID_KI_M, YAW_V_PID_KD_M, 1000, 5000,
             YAW_V_PID_LPF_M, YAW_V_PID_D_LPF_M, 0,
             Integral_Limit | Trapezoid_Intergral | DerivativeFilter);
    c[0] = YAW_V_FCC_C0;
    c[1] = YAW_V_FCC_C1;
    c[2] = YAW_V_FCC_C2;
    Feedforward_Init(&YawMotor_Manual.FFC_Velocity, YAW_V_FFC_MAXOUT_INIT, c, YAW_V_FCC_LPF, 4, 4);
//    LDOB_Init(&YawMotor_Manual.LDOB, 30000 * 0, 0.1, c, 0.00001, 4, 4);

    c[0] = YAW_A_FCC_C0;
    c[1] = YAW_A_FCC_C1;
    c[2] = YAW_A_FCC_C2;
    Feedforward_Init(&YawMotor_Manual.FFC_Angle, YAW_A_FFC_MAXOUT, c, YAW_A_FCC_LPF, 3, 3);
    PID_Init(&YawMotor_Manual.PID_Angle, YAW_A_PID_MAXOUT_M, YAW_A_PID_MAXINTEGRAL_M, 0.0,
             YAW_A_PID_KP_M, YAW_A_PID_KI_M, YAW_A_PID_KD_M, 5, 2, 0, 0, 0,
             Integral_Limit | Trapezoid_Intergral);
    YawMotor_Manual.Max_Out = YAW_MOTOR_MAXOUT * 0.9f;


    /*自瞄参数初始化*/
    /* 云台pitch轴电机PID参数初始化 */
    PID_Init(&PitMotor_Auto.PID_Velocity, PITCH_V_PID_MAXOUT_INIT_M, PITCH_V_PID_MAXINTEGRAL_M, 0,
             PITCH_V_PID_KP_M, PITCH_V_PID_KI_M, PITCH_V_PID_KD_M, 1000, 5000, PITCH_V_PID_LPF_M,
             PITCH_V_PID_D_LPF_M, 0,
             Integral_Limit | Trapezoid_Intergral | OutputFilter | DerivativeFilter);
    c[0] = PITCH_V_FCC_C0_M;
    c[1] = PITCH_V_FCC_C1_M;
    c[2] = PITCH_V_FCC_C2_M;
    Feedforward_Init(&PitMotor_Auto.FFC_Velocity, PITCH_V_FFC_MAXOUT_M, c, PITCH_V_FCC_LPF_M, 4, 4);
//    LDOB_Init(&PitMotor_Auto.LDOB, 30000 * 0, 0.1, c, 0.00001, 4, 4);
    PID_Init(&PitMotor_Auto.PID_Angle, PITCH_A_PID_MAXOUT_M, PITCH_A_PID_MAXINTEGRAL_M, 0,
             PITCH_A_PID_KP_M, PITCH_A_PID_KI_M, PITCH_A_PID_KD_M, 5, 2, PITCH_A_PID_LPF_M, PITCH_A_PID_D_LPF_M, 0,
             Integral_Limit | Trapezoid_Intergral | DerivativeFilter | Derivative_On_Measurement);
    c[0] = PITCH_A_FCC_C0_M;
    c[1] = PITCH_A_FCC_C1_M;
    c[2] = PITCH_A_FCC_C2_M;
    Feedforward_Init(&PitMotor_Auto.FFC_Angle, PITCH_A_FFC_MAXOUT_M, c, PITCH_A_FCC_LPF_M, 3, 3);
    PitMotor_Auto.Max_Out = PITCH_MOTOR_MAXOUT * 0.9f;

    /* 云台yaw轴电机PID参数初始化 */
    PID_Init(&YawMotor_Auto.PID_Velocity, YAW_V_PID_MAXOUT_M_INIT, YAW_V_PID_MAXINTEGRAL_M, 0,
             YAW_V_PID_KP_M, YAW_V_PID_KI_M, YAW_V_PID_KD_M, 1000, 5000,
             YAW_V_PID_LPF_M, YAW_V_PID_D_LPF_M, 0,
             Integral_Limit | Trapezoid_Intergral | DerivativeFilter);
    c[0] = YAW_V_FCC_C0;
    c[1] = YAW_V_FCC_C1;
    c[2] = YAW_V_FCC_C2;
    Feedforward_Init(&YawMotor_Auto.FFC_Velocity, YAW_V_FFC_MAXOUT_INIT, c, YAW_V_FCC_LPF, 4, 4);
//    LDOB_Init(&YawMotor_Auto.LDOB, 30000 * 0, 0.1, c, 0.00001, 4, 4);

    c[0] = YAW_A_FCC_C0;
    c[1] = YAW_A_FCC_C1;
    c[2] = YAW_A_FCC_C2;
    Feedforward_Init(&YawMotor_Auto.FFC_Angle, YAW_A_FFC_MAXOUT, c, YAW_A_FCC_LPF, 3, 3);
    PID_Init(&YawMotor_Auto.PID_Angle, YAW_A_PID_MAXOUT_M, YAW_A_PID_MAXINTEGRAL_M, 0.0,
             YAW_A_PID_KP_M, YAW_A_PID_KI_M, YAW_A_PID_KD_M, 5, 2, 0, 0, 0,
             Integral_Limit | Trapezoid_Intergral);
    YawMotor_Auto.Max_Out = YAW_MOTOR_MAXOUT * 0.9f;

    First_Order_Filter_Init(&mouse_x_lpf,0.014,0.1);
    First_Order_Filter_Init(&mouse_y_lpf,0.014,0.1);
    /* 将云台的初始化状态设置为释放 */
    gim.ctrl_mode = GIMBAL_RELAX;
}


void PID_Reset_manual(){
    PID_Init(&YawMotor_Manual.PID_Velocity, YAW_V_PID_MAXOUT_M, YAW_V_PID_MAXINTEGRAL_M, 0,
             YAW_V_PID_KP_M, YAW_V_PID_KI_M, YAW_V_PID_KD_M, 1000, 5000,
             YAW_V_PID_LPF_M, YAW_V_PID_D_LPF_M, 0,
             Integral_Limit | Trapezoid_Intergral);
    PID_Init(&YawMotor_Manual.PID_Angle, YAW_A_PID_MAXOUT_M, YAW_A_PID_MAXINTEGRAL_M, 0.0,
             YAW_A_PID_KP_M, YAW_A_PID_KI_M, YAW_A_PID_KD_M, 5, 2, 0, 0, 0,
             Integral_Limit | Trapezoid_Intergral);
    PID_Init(&PitMotor_Manual.PID_Velocity, PITCH_V_PID_MAXOUT_M, PITCH_V_PID_MAXINTEGRAL_M, 0,
             PITCH_V_PID_KP_M, PITCH_V_PID_KI_M, PITCH_V_PID_KD_M, 1000, 5000,
             PITCH_V_PID_LPF_M, PITCH_V_PID_D_LPF_M, 0,
             Integral_Limit | Trapezoid_Intergral);
    PID_Init(&PitMotor_Manual.PID_Angle, PITCH_A_PID_MAXOUT_M, PITCH_A_PID_MAXINTEGRAL_M, 0.0,
             PITCH_A_PID_KP_M, PITCH_A_PID_KI_M, PITCH_A_PID_KD_M, 5, 2, 0, 0, 0,
             Integral_Limit | Trapezoid_Intergral);
    c[0] = PITCH_V_FCC_C0_M,
            c[1] = PITCH_V_FCC_C1_M,
            c[2] = PITCH_V_FCC_C2_M,
            Feedforward_Init(&PitMotor_Manual.FFC_Velocity, PITCH_V_FFC_MAXOUT_M, c, PITCH_V_FCC_LPF_M, 4, 4);
//    LDOB_Init(&PitMotor_Manual.LDOB, 30000 * 0, 0.1, c, 0.00001, 4, 4);
    PID_Init(&PitMotor_Manual.PID_Angle, PITCH_A_PID_MAXOUT_M, PITCH_A_PID_MAXINTEGRAL_M, 0,
             PITCH_A_PID_KP_M, PITCH_A_PID_KI_M, PITCH_A_PID_KD_M, 5, 2, PITCH_A_PID_LPF_M, PITCH_A_PID_D_LPF_M, 0,
             Integral_Limit | Trapezoid_Intergral | DerivativeFilter | Derivative_On_Measurement);
    c[0] = PITCH_A_FCC_C0_M,
            c[1] = PITCH_A_FCC_C1_M,
            c[2] = PITCH_A_FCC_C2_M,
            Feedforward_Init(&PitMotor_Manual.FFC_Angle, PITCH_A_FFC_MAXOUT_M, c, PITCH_A_FCC_LPF_M, 3, 3);
}


void PID_Reset_auto(){
//	YawMotor_Manual.PID_Velocity.MaxOut=YAW_V_PID_MAXOUT_A;
//	YawMotor_Manual.PID_Velocity.IntegralLimit=YAW_V_PID_MAXINTEGRAL_A;
//	YawMotor_Manual.PID_Velocity.Kp=YAW_V_PID_KP_A;
//	YawMotor_Manual.PID_Velocity.Ki=YAW_V_PID_KI_A;
//	YawMotor_Manual.PID_Velocity.Kd=YAW_V_PID_KD_A;
//
//	YawMotor_Manual.PID_Angle.MaxOut=YAW_A_PID_MAXOUT_A;
//	YawMotor_Manual.PID_Angle.IntegralLimit=YAW_A_PID_MAXINTEGRAL_A;
//	YawMotor_Manual.PID_Angle.Kp=YAW_A_PID_KP_A;
//	YawMotor_Manual.PID_Angle.Ki=YAW_A_PID_KI_A;
//	YawMotor_Manual.PID_Angle.Kd=YAW_A_PID_KD_A;
    PID_Init(&YawMotor_Manual.PID_Velocity, YAW_V_PID_MAXOUT_A, YAW_V_PID_MAXINTEGRAL_A, 0,
             YAW_V_PID_KP_A, YAW_V_PID_KI_A, YAW_V_PID_KD_A, 1000, 5000,
             YAW_V_PID_LPF_A, YAW_V_PID_D_LPF_A, 0,
             Integral_Limit | Trapezoid_Intergral);
    PID_Init(&YawMotor_Manual.PID_Angle, YAW_A_PID_MAXOUT_A, YAW_A_PID_MAXINTEGRAL_A, 0.0,
             yaw_a_kp, yaw_a_ki, yaw_a_kd, 5, 2, 0, 0, 0,
             Integral_Limit | Trapezoid_Intergral);
    PID_Init(&PitMotor_Manual.PID_Velocity, PITCH_V_PID_MAXOUT_A, PITCH_V_PID_MAXINTEGRAL_A, 0,
             PITCH_V_PID_KP_A, PITCH_V_PID_KI_A, PITCH_V_PID_KD_A, 1000, 5000,
             PITCH_V_PID_LPF_A, PITCH_V_PID_D_LPF_A, 0,
             Integral_Limit | Trapezoid_Intergral);
    /*PID_Init(&PitMotor_Manual.PID_Angle, PITCH_A_PID_MAXOUT_A, PITCH_A_PID_MAXINTEGRAL_A, 0.0,
             pitch_a_kp, pitch_a_ki, pitch_a_kd, 5, 2, 0, 0, 0,
             Integral_Limit | Trapezoid_Intergral);*/
    c[0] = PITCH_V_FCC_C0_A;
    c[1] = PITCH_V_FCC_C1_A;
    c[2] = PITCH_V_FCC_C2_A;
    Feedforward_Init(&PitMotor_Manual.FFC_Velocity, PITCH_V_FFC_MAXOUT_A, c, PITCH_V_FCC_LPF_A, 4, 4);
//    LDOB_Init(&PitMotor_Manual.LDOB, 30000 * 0, 0.1, c, 0.00001, 4, 4);
    PID_Init(&PitMotor_Manual.PID_Angle, PITCH_A_PID_MAXOUT_A, PITCH_A_PID_MAXINTEGRAL_A, 0,
             pitch_a_kp, pitch_a_ki, pitch_a_kd, 5, 2, PITCH_A_PID_LPF_A, PITCH_A_PID_D_LPF_A, 0,
             Integral_Limit | Trapezoid_Intergral | DerivativeFilter | Derivative_On_Measurement);
    c[0] = PITCH_A_FCC_C0_A;
    c[1] = PITCH_A_FCC_C1_A;
    c[2] = PITCH_A_FCC_C2_A;
    Feedforward_Init(&PitMotor_Manual.FFC_Angle, PITCH_A_FFC_MAXOUT_A, c, PITCH_A_FCC_LPF_A, 3, 3);
}

