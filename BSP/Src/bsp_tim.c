//
// Created by turboDog on 2021/11/21.
//
#include <tim.h>
#include "bsp_tim.h"
void start_pwm_output(uint8_t pwm_id){
    HAL_TIM_Base_Start(&htim8);
    switch (pwm_id) {
        case 1:
        {
            HAL_TIM_PWM_Start(&htim8,TIM_CHANNEL_1);
        }
        case 2:
        {
            HAL_TIM_PWM_Start(&htim8,TIM_CHANNEL_2);
        }
        case 3:
        {
            HAL_TIM_PWM_Start(&htim8,TIM_CHANNEL_3);
        }
        case 4:
        {
            HAL_TIM_PWM_Start(&htim8,TIM_CHANNEL_4);
        }
    }
}

void set_pwm_group_param(uint8_t pwm_group, uint32_t period){
    switch (pwm_group) {
        case 1:{
            /* Set the Autoreload value */
            TIM8->ARR = (uint32_t)period;
        }
    }
}

void PWM_Set_param(uint8_t pwm_id, uint32_t pulse){
    switch (pwm_id) {
        case 1:{
            /* Set the Capture Compare Register value */
            TIM8->CCR1 = pulse;
        }
        case 2:{
            /* Set the Capture Compare Register value */
            TIM8->CCR2 = pulse;
        }
        case 3:{
            /* Set the Capture Compare Register value */
            TIM8->CCR3 = pulse;
        }
        case 4:{
            /* Set the Capture Compare Register value */
            TIM8->CCR4 = pulse;
        }
    }
}