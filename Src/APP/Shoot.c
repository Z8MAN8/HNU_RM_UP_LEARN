//
// Created by 14685 on 2022/7/3.
//

#include "Shoot.h"
#include "bsp_can.h"
#include "bsp_uart.h"
#include "keyboard.h"
#include "Detect.h"
#include "bsp_tim.h"

//������� PID �ṹ�嶨��
PIDTypeDef pid_trigger       = { 0 };
PIDTypeDef pid_trigger_speed = { 0 };
//Ħ���ֵ�� PID �ṹ�嶨��
PIDTypeDef pid_shoot_left = { 0 };
PIDTypeDef pid_shoot_right = { 0 };

float testtt = 8;

/* �ϴε�ң���������� */
uint8_t   last_left_key;
uint8_t   last_right_key;
uint8_t   last_sw1 = 1;
uint8_t   last_sw2;
int16_t   last_wheel_value;

/*���ָǵ���ز���*/
int cap_open_flag = 0;
int cap_ok = 0;

/* �����ز��� */
enum ShootState shoot_state = DONT_SHOOT;
uint8_t   shoot_cmd = 0;
uint32_t  shoot_continue_time;
uint16_t  fric_wheel_speed;
bool_t fric_wheel_run = 0;
uint8_t shooter_output;
uint16_t shooter_id1_17mm_cooling_limit;
uint16_t shooter_id1_17mm_cooling_heat;
uint8_t shoot_ok = 0; //Ħ����ת��������־λ(����ʵ��ת�ٵó���

/* �����������λ��(��λ�Ǳ�������ֵencoder) */
int32_t trigger_moto_position_ref;
/* �����������ת��(rpm) */
int16_t trigger_moto_speed_ref;
/* ����������� */
int16_t trigger_moto_current;
/* Ħ���ֵ������ */
int16_t shoot_moto_current_left;
int16_t shoot_moto_current_right;
/* ����������ز��� */
uint32_t stall_count = 0;
uint32_t stall_inv_count = 0;
uint8_t  stall_f = 0;

float speedInNumsPerSec;
uint32_t numsOfOneShot;
uint32_t delayTimeInMs;


void shoot_task(const void* argu){
    /* �������PID������ʼ�� */
    PID_Init(&pid_trigger, 4500, 2000, 0, 0.15f, 0.005, 0, 0, 0, 0, 0, 0, 0);
    PID_Init(&pid_trigger_speed, 7000, 3000, 0, 7.0, 0.5, 0.1, 0, 0, 0, 0, 0, 0);
    /* Ħ���ֵ��PID������ʼ�� */
    PID_Init(&pid_shoot_left, 7000, 3000, 0, 9.0f, 0.02, 0.00, 0, 0, 0, 0, 0, 0);
    PID_Init(&pid_shoot_right, 7000, 3000, 0, 9.0f, 0.02, 0.00, 0, 0, 0, 0, 0, 0);

    uint32_t shoot_wake_time = osKernelSysTick();

    while (1){
        /* ����Ħ���� */
        //ң��������
        //Ħ���ֺ͵��ָǵ�״̬�ж�
        if (rc.kb.bit.Q && rc.sw2 != RC_DN)//����Q�������Ҳದ�����ݲ�Ϊ2ʱ��Ħ��������
            fric_wheel_run = 1;

        if ((rc.kb.bit.Q && rc.kb.bit.SHIFT) || rc.sw2 == RC_DN)//����Q����shift�����Ҳದ������Ϊ2ʱ��Ħ���ֹر�
            fric_wheel_run = 0;

        if (glb_err.err_list[REMOTE_CTRL_OFFLINE].err_exist)//������ʱ��Ħ���ֹر�
            fric_wheel_run = 0;

        //* ���ص��ָ� */
        if (rc.kb.bit.R||rc.sw1!=RC_DN){//����R������ದ�����ݲ�Ϊ2ʱ���رյ��ָ�
            cap_open_flag = -1;
            cap_ok = 0;
        }
        if ((rc.kb.bit.R &&rc.kb.bit.SHIFT) || rc.sw1==RC_DN){//����R��shift������ದ������Ϊ2ʱ���򿪵��ָ�
            cap_open_flag = 1;
            cap_ok = 0;
        }

//		GPIOE->ODR|=data_recv.shootCommand<<5;//ʹ��PE5�����cmd
        //HAL_GPIO_WritePin(GPIOE,GPIO_PIN_5,(GPIO_PinState)data_recv.shootCommand);

        /* bullet single or continue trigger command control  */
        if (( RC_SINGLE_TRIG ||rc.mouse.l         //ң��������굥��
              /*||auto_rx_data.shootCommand*/)     //��λ������
            && shoot_state == DONT_SHOOT){//��ʼ�������

            auto_rx_data.shootCommand=0;
            shoot_cmd=1;
            shoot_continue_time = HAL_GetTick();
            //����1v1��ʱ�ر�������
            /*if(rc.kb.bit.SHIFT)
                shoot_state=TRIBLE_SHOOT;
            else*/
            shoot_state=SINGLE_SHOOT;//����״̬
        }
        else if ( RC_CONTIN_TRIG     //ң��������
                  || /*(rc.mouse.r && rc.kb.bit.SHIFT) */rc.kb.bit.V
                  /*|| (recv_flag && rc.mouse.r)*/) {  //����ʶ���Ժ������
            shoot_state=CONTINUOUS_SHOOT;
            //��е�ǶȻ���ֵ
            //���ʣ���can���߽��յ����ݸ�ֵ�������������λ�ã�����
            //������
            trigger_moto_position_ref=moto_trigger.total_ecd;//�����������λ��
        }
        else if(HAL_GetTick()-shoot_continue_time>600){
            shoot_state=DONT_SHOOT;//ʱ����������
        }

        if((abs(moto_shoot[0].speed_rpm) > 4000) || (abs(moto_shoot[1].speed_rpm) > 4000)){//���0�͵��1��ת�ٶ�����4000ʱ��������
            shoot_ok = 1;
        }
        else{
            shoot_ok =0;
        }

        if (fric_wheel_run == 0
        || shooter_output == 0   //����ϵͳ��SHOOTû�й���ʱ��������
        || shoot_ok != 1) {  //����ϵͳ�ָ������Ħ���ָֻ�ת��֮ǰ�����������ת
            shoot_state=DONT_SHOOT;
        }

        /* �����������ʵ�ֺ��� */
        Shoot_Custom_control();
        /* ���ָǿ��ƺ��� */
        Cap_Control_open();

        /* ����Ħ���ֲ������� */
        if(rc.sw1&&(last_sw1!=rc.sw1)){
            last_sw1 = rc.sw1;
            if(rc.sw1==RC_UP)
                fric_wheel_run = !fric_wheel_run;
        }
        /* ����Ħ����ʵ�ֺ��� */
        FrictionWheel_Turn_on_off();

        last_sw2 = rc.sw2;//��¼�Ҳ�ҡ��״̬
        last_left_key    = km.lk_sta;//��¼�Ҳ�ҡ��״̬
        last_right_key   = km.rk_sta;
        last_wheel_value = rc.wheel;//��¼ң������ದ������
        //GPIOE->BSRR=0x10000000;
        osDelayUntil(&shoot_wake_time, SHOOT_PERIOD);//�ȴ�һ��ʱ�ӽ��ĵ�ʱ�䣬2ms
    }
}

void Cap_Control_open(){
    if(!cap_ok){
        if(cap_open_flag == 1){
            PWM_Set_param(3, 1000);
        }
        else{
            PWM_Set_param(3, 2350);
        }
        start_pwm_output(3);
        cap_ok = 1;
    }
}

/* �������� */
void Block_Bullet_handle(void){
    if (pid_trigger_speed.Pout <= -5000) {  //��������
        if (stall_f == 0)
            stall_count ++;
    }
    else
        stall_count = 0;

    if (stall_count >= 600) { //����ʱ��3s
        stall_f = 1;
        stall_count = 0;

    }

    if (stall_f == 1){
        stall_inv_count++;

        if (stall_inv_count >= 100) { //��תʱ��0.5s
            stall_f = 0;
            stall_inv_count = 0;
        }
        else
            trigger_moto_speed_ref = 2000;
    }
}

float ShootAndDelay(float speedInNumsPerSec, uint32_t numsOfOneShot, uint32_t delayTimeInMs){
    static uint32_t ticksInMs = 0, lastNumsOfOneShot = 0, lastDelayTimeInMs = 0, count = 0;
    static int32_t lastAngle = 0;
    static float speed = 0;
    if (count == 0 || lastNumsOfOneShot != numsOfOneShot || lastDelayTimeInMs != delayTimeInMs){
        ticksInMs = HAL_GetTick() + delayTimeInMs + 1;
        lastAngle = moto_trigger.total_angle;
    }
    if (lastAngle - moto_trigger.total_angle > 8191 * TRIGGER_MOTOR_REDUCTION_RATIO / BULLETS_PER_ROUND * numsOfOneShot){
        lastAngle = moto_trigger.total_angle;
        speed = 0;
        ticksInMs = HAL_GetTick();
    }

    if (HAL_GetTick() - ticksInMs > delayTimeInMs)
        speed = speedInNumsPerSec / BULLETS_PER_ROUND * TRIGGER_MOTOR_REDUCTION_RATIO * 60;

    count++;
    lastNumsOfOneShot = numsOfOneShot;
    lastDelayTimeInMs = delayTimeInMs;
    return speed;
}

/* �ӵ��ĵ������������� */
void Shoot_Custom_control(void){
    if (fric_wheel_run
        &&(shooter_id1_17mm_cooling_heat < (shooter_id1_17mm_cooling_limit-30))) { //������������

        //HAL_GPIO_WritePin(GPIOE,GPIO_PIN_12,(GPIO_PinState)shoot_cmd);
        switch(shoot_state){
            case SINGLE_SHOOT:
                if(shoot_cmd){
                    /* ����ǵ������������ת45�� */
                    trigger_moto_position_ref = moto_trigger.total_ecd + DEGREE_45_TO_ENCODER;
                    shoot_cmd=0;
                }
                /* �ջ����㲦���������ת�� */
                trigger_moto_speed_ref = PID_Calculate(&pid_trigger, moto_trigger.total_ecd, trigger_moto_position_ref);
                goto emmm;
            case TRIBLE_SHOOT:
                if(shoot_cmd){
                    /* ������������������ת3*45�� */
                    trigger_moto_position_ref = moto_trigger.total_ecd + 3*DEGREE_45_TO_ENCODER;
                    shoot_cmd=0;
                }
                /* �ջ����㲦���������ת�� */
                trigger_moto_speed_ref = PID_Calculate(&pid_trigger, moto_trigger.total_ecd, trigger_moto_position_ref);
                goto emmm;
            case CONTINUOUS_SHOOT:
#ifdef RM1V1
                speedInNumsPerSec=14.0f /*testtt*/;
#elif RM3V3
                speedInNumsPerSec=10.0f;
#endif
                numsOfOneShot=4;
                delayTimeInMs=10;
                break;
            case DONT_SHOOT:
                trigger_moto_speed_ref = 0;
                goto emmm;
        }
        trigger_moto_speed_ref=-ShootAndDelay(speedInNumsPerSec,numsOfOneShot,delayTimeInMs);
        Block_Bullet_handle();                                 //��������
        /* �ջ����㲦��������� */
        emmm:
        trigger_moto_current = PID_Calculate(&pid_trigger_speed, moto_trigger.speed_rpm, trigger_moto_speed_ref);
    }
    else{
        trigger_moto_current = 0;
    }

    /*if(shoot_state == DONT_SHOOT && abs(moto_shoot[0].speed_rpm)<1000){  //��������ϵ磬���ܷ������ı���
        moto_shoot[0].speed_rpm = 0;
        moto_shoot[1].speed_rpm = 0;
    }*/
    /* �ջ�����Ħ���ֵ������ */
    shoot_moto_current_left = PID_Calculate(&pid_shoot_left, moto_shoot[0].speed_rpm, -fric_wheel_speed);
    shoot_moto_current_right = PID_Calculate(&pid_shoot_right, moto_shoot[1].speed_rpm, fric_wheel_speed);

    /* TODO��2023������
     * ��ʱ����취���������Ž⣬
     * ��������Ϊ��shoot_stateΪDONT PID������fric_wheel_speed��Ϊ�㣬Ioutһֱ������*/
    /*if(shoot_state == DONT_SHOOT){
        shoot_moto_current_left = 0;
        shoot_moto_current_right = 0;
    }*/

    /* ���Ͳ��������Ħ���ֵ������ */
    ShootMoto_Send_current(shoot_moto_current_left, shoot_moto_current_right, trigger_moto_current);
}

/* ����Ħ���ִ��� */
void FrictionWheel_Turn_on_off(void){
    if (fric_wheel_run){
        //��Ħ����
        fric_wheel_speed = SHOT_FRIC_WHEEL_SPEED;
        //�򿪼��⡢����װ��
        HAL_GPIO_WritePin(UVLED_GPIO_Port,UVLED_Pin,GPIO_PIN_SET);
        //TODO:���⣬����û�ӣ���ע����
        //write_led_io(LASER_IO, LED_ON);
    }
    else{
        //�ر�Ħ����
        fric_wheel_speed=0;
        //�رռ��⡢����װ��
        HAL_GPIO_WritePin(UVLED_GPIO_Port,UVLED_Pin,GPIO_PIN_RESET);
        //write_led_io(LASER_IO, LED_OFF);
    }
}