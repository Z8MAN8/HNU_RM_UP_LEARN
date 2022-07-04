//
// Created by 14685 on 2022/7/4.
//

#ifndef HNU_RM_UP_BSP_RC_H
#define HNU_RM_UP_BSP_RC_H

#include "cmsis_os.h"
/**
  * @brief     解析后的遥控器数据结构体
  */
typedef struct
{
    /* 遥控器的通道数据，数值范围：-660 ~ 660 */
    int16_t ch1;   //右侧左右
    int16_t ch2;   //右侧上下
    int16_t ch3;   //左侧左右
    int16_t ch4;   //左侧上下

    /* 遥控器的拨杆数据，上中下分别为：1、3、2 */
    uint8_t sw1;   //左侧拨杆
    uint8_t sw2;   //右侧拨杆

    /* PC 鼠标数据 */
    struct
    {
        /* 鼠标移动相关 */
        int16_t x;   //鼠标平移
        int16_t y;   //鼠标上下
        /* 鼠标按键相关，1为按下，0为松开 */
        uint8_t l;   //左侧按键
        uint8_t r;   //右侧按键
    }mouse;

    /* PC 键盘按键数据 */
    union
    {
        uint16_t key_code;
        struct
        {
            uint16_t W:1;
            uint16_t S:1;
            uint16_t A:1;
            uint16_t D:1;
            uint16_t SHIFT:1;
            uint16_t CTRL:1;
            uint16_t Q:1;
            uint16_t E:1;
            uint16_t R:1;
            uint16_t F:1;
            uint16_t G:1;
            uint16_t Z:1;
            uint16_t X:1;
            uint16_t C:1;
            uint16_t V:1;
            uint16_t B:1;
        }bit;
    }kb;

    /* 遥控器左侧拨轮数据 */
    int16_t wheel;
}RcTypeDef;
/**
  * @brief     遥控器拨杆数据枚举
  */
enum
{
    RC_UP = 1,
    RC_MI = 3,
    RC_DN = 2,
};

#endif //HNU_RM_UP_BSP_RC_H
