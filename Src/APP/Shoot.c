//
// Created by 14685 on 2022/7/3.
//

#include "Shoot.h"
#include "bsp_can.h"
#include "bsp_uart.h"
#include "keyboard.h"
#include "Detect.h"
#include "bsp_tim.h"

//拨弹电机 PID 结构体定义
PIDTypeDef pid_trigger       = { 0 };
PIDTypeDef pid_trigger_speed = { 0 };
//摩擦轮电机 PID 结构体定义
PIDTypeDef pid_shoot_left = { 0 };
PIDTypeDef pid_shoot_right = { 0 };

/* 上次的遥控数据数据 */
uint8_t   last_left_key;
uint8_t   last_right_key;
uint8_t   last_sw1;
uint8_t   last_sw2;
int16_t   last_wheel_value;

/*弹仓盖的相关参数*/
int cap_open_flag = 0;
int cap_ok = 0;

/* 射击相关参数 */
enum ShootState shoot_state;
uint8_t   shoot_cmd = 0;
uint32_t  shoot_continue_time;
uint16_t  fric_wheel_speed = SHOT_FRIC_WHEEL_SPEED;
bool_t fric_wheel_run = 0;
uint8_t shooter_output;

/* 拨弹电机期望位置(单位是编码器数值encoder) */
int32_t trigger_moto_position_ref;
/* 拨弹电机期望转速(rpm) */
int16_t trigger_moto_speed_ref;
/* 拨弹电机电流 */
int16_t trigger_moto_current;
/* 摩擦轮电机电流 */
int16_t shoot_moto_current_left;
int16_t shoot_moto_current_right;
/* 卡弹处理相关参数 */
uint32_t stall_count = 0;
uint32_t stall_inv_count = 0;
uint8_t  stall_f = 0;

float speedInNumsPerSec;
uint32_t numsOfOneShot;
uint32_t delayTimeInMs;


void shoot_task(const void* argu){
    /* 拨弹电机PID参数初始化 */
    PID_Init(&pid_trigger, 4500, 2000, 0, 0.15f, 0.005, 0, 0, 0, 0, 0, 0, 0);
    PID_Init(&pid_trigger_speed, 7000, 3000, 0, 7.0, 0.5, 0.1, 0, 0, 0, 0, 0, 0);
    /* 摩擦轮电机PID参数初始化 */
    PID_Init(&pid_shoot_left, 7000, 3000, 0, 9.0f, 0.02, 0.00, 0, 0, 0, 0, 0, 0);
    PID_Init(&pid_shoot_right, 7000, 3000, 0, 9.0f, 0.02, 0.00, 0, 0, 0, 0, 0, 0);

    uint32_t shoot_wake_time = osKernelSysTick();

    while (1){
        /* 开关摩擦轮 */
        if (rc.kb.bit.Q && rc.sw2 != RC_DN)
            fric_wheel_run = 1;

        if ((rc.kb.bit.Q && rc.kb.bit.SHIFT) || rc.sw2 == RC_DN)
            fric_wheel_run = 0;

        if (glb_err.err_list[REMOTE_CTRL_OFFLINE].err_exist)
            fric_wheel_run = 0;

        //* 开关弹仓盖 */
        if (rc.kb.bit.R||rc.sw1!=RC_DN){
            cap_open_flag = -1;
            cap_ok = 0;
        }
        if ((rc.kb.bit.R &&rc.kb.bit.SHIFT) || rc.sw1==RC_DN){
            cap_open_flag = 1;
            cap_ok = 0;
        }

//		GPIOE->ODR|=data_recv.shootCommand<<5;//使用PE5来检测cmd
        //HAL_GPIO_WritePin(GPIOE,GPIO_PIN_5,(GPIO_PinState)data_recv.shootCommand);

        /* bullet single or continue trigger command control  */
        if (( RC_SINGLE_TRIG ||rc.mouse.l         //遥控器或鼠标单发
              /*||auto_rx_data.shootCommand*/)     //上位机单发
            && shoot_state == DONT_SHOOT){

            auto_rx_data.shootCommand=0;
            shoot_cmd=1;
            shoot_continue_time = HAL_GetTick();
            if(rc.kb.bit.SHIFT)
                shoot_state=TRIBLE_SHOOT;
            else
                shoot_state=SINGLE_SHOOT;
        }
        else if ( RC_CONTIN_TRIG     //遥控器连发
                  || (rc.mouse.r && rc.kb.bit.SHIFT) ) {  //鼠标连发
            shoot_state=CONTINUOUS_SHOOT;
            trigger_moto_position_ref=moto_trigger.total_ecd;
        }
        else if(HAL_GetTick()-shoot_continue_time>600){
            shoot_state=DONT_SHOOT;
        }

        if (fric_wheel_run == 0){
            shoot_state=DONT_SHOOT;
        }

        /* 单发连发射击实现函数 */
        Shoot_Custom_control();
        /* 弹仓盖控制函数 */
        Cap_Control_open();

        /* 开关摩擦轮操作控制 */
        if(rc.sw1&&(last_sw1!=rc.sw1)){
            last_sw1 = rc.sw1;
            if(rc.sw1==RC_UP)
                fric_wheel_run = !fric_wheel_run;
        }
        /* 开关摩擦轮实现函数 */
        FrictionWheel_Turn_on_off();

        last_sw2 = rc.sw2;
        last_left_key    = km.lk_sta;
        last_right_key   = km.rk_sta;
        last_wheel_value = rc.wheel;
        //GPIOE->BSRR=0x10000000;
        osDelayUntil(&shoot_wake_time, SHOOT_PERIOD);
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

/* 卡弹处理 */
void Block_Bullet_handle(void){
    if (pid_trigger_speed.Pout <= -5000) {  //卡弹电流
        if (stall_f == 0)
            stall_count ++;
    }
    else
        stall_count = 0;

    if (stall_count >= 600) { //卡弹时间3s
        stall_f = 1;
        stall_count = 0;

    }

    if (stall_f == 1){
        stall_inv_count++;

        if (stall_inv_count >= 100) { //反转时间0.5s
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

/* 子弹的单发和连发处理 */
void Shoot_Custom_control(void){
    if (fric_wheel_run
       /* &&shooter_output==1*/) {  //裁判系统对SHOOT没有供电时，不拨弹)

        //HAL_GPIO_WritePin(GPIOE,GPIO_PIN_12,(GPIO_PinState)shoot_cmd);
        switch(shoot_state){
            case SINGLE_SHOOT:
                if(shoot_cmd){
                    /* 如果是单发命令，拨轮旋转45度 */
                    trigger_moto_position_ref = moto_trigger.total_ecd + DEGREE_45_TO_ENCODER;
                    shoot_cmd=0;
                }
                /* 闭环计算拨弹电机期望转速 */
                trigger_moto_speed_ref = PID_Calculate(&pid_trigger, moto_trigger.total_ecd, trigger_moto_position_ref);
                goto emmm;
            case TRIBLE_SHOOT:
                if(shoot_cmd){
                    /* 如果是三发命令，拨轮旋转3*45度 */
                    trigger_moto_position_ref = moto_trigger.total_ecd + 3*DEGREE_45_TO_ENCODER;
                    shoot_cmd=0;
                }
                /* 闭环计算拨弹电机期望转速 */
                trigger_moto_speed_ref = PID_Calculate(&pid_trigger, moto_trigger.total_ecd, trigger_moto_position_ref);
                goto emmm;
            case CONTINUOUS_SHOOT:
                speedInNumsPerSec=4.0f;
                numsOfOneShot=4;
                delayTimeInMs=10;
                break;
            case DONT_SHOOT:
                trigger_moto_speed_ref = 0;
                goto emmm;
        }
        trigger_moto_speed_ref=-ShootAndDelay(speedInNumsPerSec,numsOfOneShot,delayTimeInMs);
        Block_Bullet_handle();                                 //卡弹处理
        /* 闭环计算拨弹电机电流 */
        emmm:
        trigger_moto_current = PID_Calculate(&pid_trigger_speed, moto_trigger.speed_rpm, trigger_moto_speed_ref);
    }
    else{
        trigger_moto_current = 0;
    }
    /* 闭环计算摩擦轮电机电流 */
    shoot_moto_current_left = PID_Calculate(&pid_shoot_left, moto_shoot[0].speed_rpm, -fric_wheel_speed);
    shoot_moto_current_right = PID_Calculate(&pid_shoot_right, moto_shoot[1].speed_rpm, fric_wheel_speed);

    /* 发送拨弹电机、摩擦轮电机电流 */
    ShootMoto_Send_current(shoot_moto_current_left, shoot_moto_current_right, trigger_moto_current);
}

/* 开关摩擦轮处理 */
void FrictionWheel_Turn_on_off(void){
    if (fric_wheel_run){
        //打开摩擦轮
        fric_wheel_speed=SHOT_FRIC_WHEEL_SPEED;
        //打开激光、充能装置
        HAL_GPIO_WritePin(UVLED_GPIO_Port,UVLED_Pin,GPIO_PIN_SET);
        //TODO:激光，车上没接，先注释了
        //write_led_io(LASER_IO, LED_ON);
    }
    else{
        //关闭摩擦轮
        fric_wheel_speed=0;
        //关闭激光、充能装置
        HAL_GPIO_WritePin(UVLED_GPIO_Port,UVLED_Pin,GPIO_PIN_RESET);
        //write_led_io(LASER_IO, LED_OFF);
    }
}