//
// Created by turboDog on 2021/11/21.
//

#ifndef BOARD_C_INFANTRY_BSP_TIM_H
#define BOARD_C_INFANTRY_BSP_TIM_H

//PWM IO 接口定义
//暂时只实现了tim1
#define PWM_GROUP1        1   //第一组 PWM IO
#define PWM_IO1           1
#define PWM_IO2           2
#define PWM_IO3           3
#define PWM_IO4           4

//#define PWM_GROUP2        2   //第二组 PWM IO
//#define PWM_IO5           5
//#define PWM_IO6           6
//#define PWM_IO7           7
//#define PWM_IO8           8
//
//#define PWM_GROUP3        3   //第三组 PWM IO
//#define PWM_IO9           9
//#define PWM_IO10          10
//#define PWM_IO11          11
//#define PWM_IO12          12
//
//#define PWM_GROUP4        4   //第四组 PWM IO
//#define PWM_IO13          13
//#define PWM_IO14          14
//#define PWM_IO15          15
//#define PWM_IO16          16
//PWM IO 相关函数
/**
  * @brief     开启 PWM 输出
  * @param     pwm_id: PWM IO 的 ID
  */
void start_pwm_output(uint8_t pwm_id);
/**
  * @brief     设置 PWM 组对应参数
  * @param     pwm_group: PWM 组别，目前有 4 组PWM
  * @param     period:  每组 PWM 对应周期时间，单位是微秒(us)
  */
void set_pwm_group_param(uint8_t pwm_group, uint32_t period);
/**
  * @brief     设置 PWM IO 参数
  * @param     pwm_id: PWM IO 的 ID
  * @param     pulse: 配置 PWM 的高电平时间，单位是微秒(us)
  */
void PWM_Set_param(uint8_t pwm_id, uint32_t pulse);


#endif //BOARD_C_INFANTRY_BSP_TIM_H
