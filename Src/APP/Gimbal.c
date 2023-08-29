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

float dt = 0.0;//����ʼ��
float task_dt = 1.0;//����ʼ��

//float yaw_a_kp = 30;
//float yaw_a_ki = 200;
//float yaw_a_kd = 0.001;
//float pitch_a_kp = 90;
//float pitch_a_ki = 180;
//float pitch_a_kd = 1;
float yaw_a_kp = 30;//��̨ƫ���ǡ������ǲ�������
float yaw_a_ki = 200;
float yaw_a_kd = 0.001;
float pitch_a_kp = 90;
float pitch_a_ki = 180;
float pitch_a_kd = 1;

float Ballistic_compensation_mannul;  //���������ݽ����ֶ���������
static float gimbal_yaw = 0;
static float gimbal_pitch = 0;  //������λ�����͵���̨�Ƕ�
static float yaw_speed = 0;
static float pitch_speed = 0;
static float roll_speed = 0;    //������λ�����͵�Ŀ���ƶ��ٶ�
int16_t yaw_moto_current_manual = 0;
int16_t yaw_moto_current_auto = 0;
int16_t pit_moto_current_manual = 0;
int16_t pit_moto_current_auto = 0;
//TODO:���ǽ�noaction�������ͷ���ǰ�Ķ�׼��������һ��

ImuTypeDef imu;    //����IMU��������ص�����
GimbalBackType gimbal_back_step;
GimbalYawTypeDef gim;

/*��̨����ֵ*/
int32_t   pit_center_offset = 3616;
#ifdef SIDEWAYS
int32_t   yaw_center_offset = 5426;
#else
int32_t   yaw_center_offset = 4600;
#endif

/*���������������*/
First_Order_Filter_t mouse_y_lpf,mouse_x_lpf;

/* ��̨��ԽǶ�,unit: degree*/
float     pit_relative_angle;
volatile float  yaw_relative_angle;

/* �ϰ巢�͸��°������֮һ */
volatile float yaw_angle_ref_v;

/* gimbal pid parameter */
float yaw_angle_fdb = 0;
float pit_angle_fdb = 0;
float c[3] = {0};

/* ��̨��������Ƕ�(degree) */
float yaw_angle_ref =0;
float pit_angle_ref=0;

/* ��̨������� */
int16_t *yaw_moto_current = &yaw_moto_current_manual;
int16_t *pit_moto_current = &pit_moto_current_manual;

bool_t recv_flag=false;   //���⴮�ڽ��ձ�־λ
/*�л��ֶ����Զ�ģʽ��ӦPID�����ı�־λ*/
static _Bool auto_pid_flag = 0;
static _Bool manual_pid_flag = 1;

//���25֡��ʷ��̬����
float angle_history[50];





void gimbal_task(void const * argument){
    /*��ʼ����̨���Ʋ���*/
    Gimbal_Init_param();//�ֱ��ʼ����̨pitch��yaw���ֶ����Զ���ǰ�����ơ�pid����
    /*��ȡPreviousWakeTime*/
    uint32_t Gimbal_Wake_time = osKernelSysTick();//��¼����ʼ�δ����ֵ
    DWT_Init(168);//�ٴγ�ʼ��DWT���裿������main�������ѳ�ʼ����

    while (1){
        /*��ȡ��̨��������������Ϣ*/
        Gimbal_Get_information();
        //���⣺��ȡIMU����ʱ�Ƿ����������ͻ������
        //���룺ins������ins.accel������д�룬���������Ƕ�ȡ��������Ҫһ�����������ź����ĸ���

        //�ѽ��������ͻ
        switch (gim.ctrl_mode){//�������ģʽ���ж���ִ��

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



        /*������ʱ����֤Gimbal_TASK�̶���������*/
        osDelayUntil(&Gimbal_Wake_time, GIMBAL_PERIOD);//�ȴ�һ��ʱ�ӽ��ĵ�ʱ�䣬1ms
    }
}


void Gimbal_Get_information(void){
    /*��ȡIMU����*/
    IMU_Get_data(&imu);//imu������ݸ�ֵ

    /*��ȡ��̨��ԽǶ�*/
    //���⣺���㹫ʽ��Gimbal_Get_relative_pos�����ľ������
    //yaw_center_offsetΪ�̶�ֵ������

    //�ѽ��������
    yaw_relative_angle = Gimbal_Get_relative_pos(YawMotor_Manual.RawAngle, yaw_center_offset) / 22.75f;
    pit_relative_angle = Gimbal_Get_relative_pos(PitMotor_Manual.RawAngle, pit_center_offset) / 22.75f;

    /*����PC�˼������*/
    PC_Handle_kb();//������Էŵ����ڿ����ж��� keyboard��UserLib�ļ�����

    /*��ȡ��̨��ǰģʽ������ֻ���ж���RELAX����INITģʽ*/
    // ��Ϊ����ģʽ���ж���Gimbal_Init_handle()������
    Gimbal_Get_mode();
}

//  ��̨״̬���߼���
//  - Gimbal_Get_mode����ȡ��̨ģʽ���Ƚ���һ������Ĵ����жϡ������̨�������´���1. ң�������� 2. yaw���pitch�������� �Լ���̨���ֶ�����Ϊrelaxģʽʱ���л�/ά��RELAX״̬��ͬʱ�������ǰ״̬ΪGIMBAL_RELAX��
//    ��ֱ���л�ģʽλGIMBAL_INIT����������/����״̬�������Ļ���һ���ֱ��RELAX��
//  - �������ɣ�RELAX��INIT������̨��׼��״̬��ʣ������GIMBAL_CLOSE_LOOP_ZGYRO��GIMBAL_AUTO������̨����������״̬

void Gimbal_Get_mode(void){
    /* gim.ac_mode = Remote_Is_action();
     gim.last_mode = gim.ctrl_mode;*/
    if(   glb_err.err_list[REMOTE_CTRL_OFFLINE].err_exist //ң��������
          || glb_err.err_list[GIMBAL_YAW_OFFLINE].err_exist  //yaw��������
          || glb_err.err_list[GIMBAL_PIT_OFFLINE].err_exist//pitch��������
          || rc.sw2 ==RC_DN){/*�����ģ�����߻��Ҳದ��ֵ��DNʱ������̨ΪGIMBAL_RELAXģʽ*/
        gim.ctrl_mode = GIMBAL_RELAX;
    }

    else if(gim.ctrl_mode == GIMBAL_RELAX){
        gim.ctrl_mode = GIMBAL_INIT;
    }

    gim.last_mode = gim.ctrl_mode;//���룺��ֵλ���Ƿ������⣬last_mode��ֵ�Ƿ�Ӧ����ctrl_mode֮ǰ��
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

//  ������̨���д����init����ݿ������ź�ѡ���ֶ����Զ����Ƶ��߼�
//  - gimbal_back_step ������һ��Ϊ BACK_IS_OK �Ͳ����ٱ䣬�����ٴν�����в����ˡ�Gimbal_Relax_handle() �л���� Gimbal_Back_param() �������¸���ʼֵ��
//    Ҳ����˵ÿ����relax״̬һ�Σ������������»���
//  - ��ʱxxx_angle_fdb������imu�������ñ�����������ģ�û��imu��ȷ������Ϊimu��offsetҪ�ڹ��к�õ���Ȼ��ſ��Կ�ʼ��imu

/*��̨��ʼ��������*/
void Gimbal_Init_handle(void){

    pit_angle_fdb = pit_relative_angle;
    yaw_angle_fdb = yaw_relative_angle;

    /* gimbal pitch back center */
    //��̨��������
    pit_angle_ref = pit_relative_angle * (1 - ramp_calc(&pit_ramp));//�õ�������

    switch (gimbal_back_step){//����һ��һ������
        //��pitch��û�л������֮ǰ�������yaw��Ļ���
        case PIT_BACK_STEP:{
            /* keep yaw unmove this time */
            yaw_angle_ref = gim.ecd_offset_angle;

            if(fabs(pit_angle_fdb) <= 2.0f)//����Сʱ������
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
            if(rc.sw2 == RC_UP){//��̨���к�ͨ��ң������ѡ����̨ģʽ��GIMBAL_CLOSE_LOOP_ZGYRO��GIMBAL_AUTO
                gim.ctrl_mode = GIMBAL_CLOSE_LOOP_ZGYRO;
            }
            else if (rc.sw2 == RC_MI){
                gim.ctrl_mode = GIMBAL_AUTO;
            }

            gim.yaw_offset_angle = imu.angle_x;//ƫ���͸�����ƫ������ֵ
            gim.pit_offset_angle = imu.angle_y;
            //���룺����ʱ��������ƫ���������Ƕ���Ϊ0��
            pit_angle_ref = 0;
            yaw_angle_ref = 0;
            /*��̨������ɣ����������������ֵ*/
            //�Զ����ֶ����ֵ��ֵ
            YawMotor_Manual.PID_Velocity.MaxOut=YAW_V_PID_MAXOUT_M;
            YawMotor_Manual.FFC_Velocity.MaxOut=YAW_V_FFC_MAXOUT;
            PitMotor_Manual.PID_Velocity.MaxOut=PITCH_V_PID_MAXOUT_M;

            YawMotor_Auto.PID_Velocity.MaxOut=YAW_V_PID_MAXOUT_M;
            YawMotor_Auto.FFC_Velocity.MaxOut=YAW_V_FFC_MAXOUT;
            PitMotor_Auto.PID_Velocity.MaxOut=PITCH_V_PID_MAXOUT_M;

        }break;
    }
}

/*��̨����������ջ����ƴ�����*/
void Gimbal_Loop_handle(){
    if(recv_flag) {  //ŷ����rpy��ʽ����

        //���ԽǶ�����ԽǶȿ���������ԽǶȼ������Ballistic_compensation_mannul

        if (!rpy_rx_data.DATA[0]){     //���ԽǶȿ���
            gimbal_yaw = *(int32_t*)&rpy_rx_data.DATA[1] / 1000.0;
            /*(int32_t)(rpy_rx_data.DATA[4] << 24 | rpy_rx_data.DATA[3] << 16
                           | rpy_rx_data.DATA[2] << 8 | rpy_rx_data.DATA[1])/1000;*/
            gimbal_pitch = *(int32_t*)&rpy_rx_data.DATA[5] / 1000.0;
            /*(int32_t)(rpy_rx_data.DATA[8] << 24 | rpy_rx_data.DATA[7] << 16
                             | rpy_rx_data.DATA[6] << 8 | rpy_rx_data.DATA[5])/1000;*/
        }
        else{     //��ԽǶȿ���
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
    /*��ͨģʽ��������ģʽ���໥�л�*/
    if(rc.sw2==RC_MI||rc.mouse.r==1){
        gim.ctrl_mode = GIMBAL_AUTO;
        gim.yaw_offset_angle = imu.angle_x;
        yaw_angle_ref = 0;
        yaw_angle_fdb = 0;
    }
        /*�л���ϣ�������ͨģʽ�Ŀ���*/
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

        //����pitch��Ļ�Ƕ�
        if ((pit_relative_angle >= PIT_ANGLE_MIN) && (pit_relative_angle <= PIT_ANGLE_MAX)){
            VAL_LIMIT(pit_angle_ref, PIT_ANGLE_MIN, PIT_ANGLE_MAX);
        }
    }
}


void Gimbal_Control_yaw(void){
    float fx=First_Order_Filter_Calculate(&mouse_x_lpf,rc.mouse.x);
    //yaw��ĽǶ��ۼӣ���λdegree
    yaw_angle_ref += -rc.ch3 * RC_RATIO * GIMBAL_RC_MOVE_RATIO_YAW
                     -fx * KB_RATIO * GIMBAL_PC_MOVE_RATIO_YAW;
}

void Gimbal_Control_pitch(void){
    float fy=First_Order_Filter_Calculate(&mouse_y_lpf,rc.mouse.y);
    //pitch��ĽǶ��ۼӣ���λdegree
    pit_angle_ref += rc.ch4 * RC_RATIO * GIMBAL_RC_MOVE_RATIO_PIT
                     - fy * KB_RATIO * GIMBAL_PC_MOVE_RATIO_PIT;
}


void Gimbal_Auto_control(void){
//    static float gimbal_yaw = 0;
//    static float gimbal_pitch = 0;  //������λ�����͵���̨�Ƕ�
    /*static float dt = 0.0;//����ʼ��
    static float task_dt = 1.0;//����ʼ��*/
    static uint32_t old_counter=0;
    static uint32_t old_counter_task=0;
    float target_distance = 0; //��ʶ��Ŀ��ľ���
//    static bool_t com_protect = 1; //Ϊ1ʱһ֡���ݴ������



    /*����ģʽ������ͨģʽ���໥�л�*/
    if(rc.sw2==RC_UP && rc.mouse.r!=1){
        gim.ctrl_mode=GIMBAL_CLOSE_LOOP_ZGYRO;
        gim.yaw_offset_angle = imu.angle_x;
        yaw_angle_ref = 0;
        yaw_angle_fdb = 0;
        Ballistic_compensation_mannul = 0;
    }
    /*�л���ϣ���������ģʽ�Ŀ���*/
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
        //yaw��ĽǶ��ۼӣ���λdegree
        static float manual_offset =0.0;
        manual_offset+=-rc.ch3 * RC_RATIO * GIMBAL_RC_MOVE_RATIO_YAW
                       -fx * KB_RATIO * GIMBAL_PC_MOVE_RATIO_YAW;

//	HAL_GPIO_TogglePin(GPIOG,GPIO_PIN_2);
//TODO:
        if(recv_flag) {  //ŷ����rpy��ʽ����
            if (!rpy_rx_data.DATA[0]){     //���ԽǶȿ���
                gimbal_yaw = *(int32_t*)&rpy_rx_data.DATA[1] / 1000.0;
                        /*(int32_t)(rpy_rx_data.DATA[4] << 24 | rpy_rx_data.DATA[3] << 16
                                       | rpy_rx_data.DATA[2] << 8 | rpy_rx_data.DATA[1])/1000;*/
                gimbal_pitch = *(int32_t*)&rpy_rx_data.DATA[5] / 1000.0;
                        /*(int32_t)(rpy_rx_data.DATA[8] << 24 | rpy_rx_data.DATA[7] << 16
                                         | rpy_rx_data.DATA[6] << 8 | rpy_rx_data.DATA[5])/1000;*/
            }
            else {     //��ԽǶȿ���(Ŀǰ����Ŀ��Ʒ�ʽ��
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
            target_distance = *(int32_t*)&rpy_rx_data.DATA[13] / 1000;  //��ȡĿ�����
            recv_flag = 0;
            /*Ԥ��task_dt ��DWT����λ�����ݽ��ռ��ʱ��,*/
            task_dt = DWT_GetDeltaT(&old_counter_task);
            old_counter = DWT->CYCCNT;
            old_counter_task = DWT->CYCCNT;
            /*task_dtԼΪ0.01*/

        } else
        {   /*Ԥ��dt ��DWT����Ƽ��ʱ��*/
            dt = DWT_GetDeltaT(&old_counter);
            old_counter = DWT->CYCCNT;
            /*dtԼΪ0.001*/
        }
        //ң����΢��
//    gimbal_yaw_control();
        //gimbal_pitch_control();
        //pit_angle_ref=pit_relative_angle+b;

        //����pit��Ļ�Ƕ�
        if ((pit_angle_ref >= PIT_ANGLE_MAX) || (pit_angle_ref <= PIT_ANGLE_MIN)){
            VAL_LIMIT(pit_angle_ref, PIT_ANGLE_MIN, PIT_ANGLE_MAX);
        }
/*        if ((yaw_angle_ref >= 170) || (yaw_angle_ref <= -170)){
            VAL_LIMIT(yaw_angle_ref, -170, 170);
        }*/
//  ����ʱ��ʱ��Ȼʹ�ñ���������
//    //����pitch����ԽǶȲ�
        pit_angle_fdb = pit_relative_angle;
//    //����yaw����ԽǶȲ�
//    yaw_angle_fdb = yaw_relative_angle;
        //����������ʱҲʹ��IMU
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

    /* pitch�ḩ���Ƕ����� */
    float delta=pit_relative_angle+pit_angle_ref-pit_angle_fdb;
    if(delta-PIT_ANGLE_MAX>0)
        pit_angle_ref=pit_angle_fdb+PIT_ANGLE_MAX-pit_relative_angle;
    else if(delta-PIT_ANGLE_MIN<0)
        pit_angle_ref=pit_angle_fdb-(PIT_ANGLE_MIN-pit_relative_angle);

    VAL_LIMIT(pit_angle_ref, PIT_ANGLE_MIN, PIT_ANGLE_MAX);
//  /* pitch��Ԥ���ٶȼ��㣬��λdegree/s */
//  pit_speed_ref    = PID_Calc(&pid_pit, pit_angle_fdb, pit_angle_ref);    //degree
//  /* pitch������ѹ���� */

//	pit_moto_current = PID_Calc(&pid_pit_speed, imu.gyro_y, pit_speed_ref); //degree/s
//	//�˲�
//	pit_moto_current = 0.5*last_current + 0.5*pit_moto_current;
//	last_current = pit_moto_current;
//pitch���ٶ�Ϊgyro_x
    pit_moto_current_manual = Motor_Angle_Calculate(&PitMotor_Manual, pit_angle_fdb, imu.gyro_x, pit_angle_ref);
    pit_moto_current_auto = Motor_Angle_Calculate(&PitMotor_Auto, pit_angle_fdb, imu.gyro_x, pit_angle_ref);

    if(gim.ctrl_mode==GIMBAL_AUTO){
        /*yaw_moto_current = &yaw_moto_current_auto;
        pit_moto_current = &pit_moto_current_auto;*/
        /*yaw_moto_current = 0;
        pit_moto_current = 0;*/
        //���⣺auto_pid_flag=1ʱû����������
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

    //���͵�������̨������
    GimbalMoto_Send_current(*yaw_moto_current, *pit_moto_current);
}


/*��̨���ߴ����������������PID������Ϊ����ģʽ*/
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
    //TODO:���Ǽ���KIֵ
//    YawMotor_Manual.PID_Angle.Ki=80000;
    ramp_init(&pit_ramp, BACK_CENTER_TIME/GIMBAL_PERIOD);
    ramp_init(&yaw_ramp, BACK_CENTER_TIME/GIMBAL_PERIOD);
    //pid_pit_speed.max_output = 15000;
}


void Gimbal_Init_param(void){
    /*�ֶ�������ʼ��*/
    /* ��̨pitch����PID������ʼ�� */
    PID_Init(&PitMotor_Manual.PID_Velocity, PITCH_V_PID_MAXOUT_INIT_M, PITCH_V_PID_MAXINTEGRAL_M, 0,
             PITCH_V_PID_KP_M, PITCH_V_PID_KI_M, PITCH_V_PID_KD_M, 1000, 5000, PITCH_V_PID_LPF_M,
             PITCH_V_PID_D_LPF_M, 0,
             Integral_Limit | Trapezoid_Intergral | OutputFilter | DerivativeFilter);
    c[0] = PITCH_V_FCC_C0_M;
    c[1] = PITCH_V_FCC_C1_M;
    c[2] = PITCH_V_FCC_C2_M;
    //���ʣ�ǰ������Ԥ����̨�����Ǻ�ƫ���ǵĽǶȺ����ʣ�����
    //ǰ���������ݲ���
    //�ѽ����û���õ�ǰ��
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

    /* ��̨yaw����PID������ʼ�� */
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


    /*���������ʼ��*/
    /* ��̨pitch����PID������ʼ�� */
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

    /* ��̨yaw����PID������ʼ�� */
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
    /* ����̨�ĳ�ʼ��״̬����Ϊ�ͷ� */
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

