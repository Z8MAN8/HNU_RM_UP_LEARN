//
// Created by turboDog on 2021/11/16.
//

#ifndef BOARD_C_INFANTRY_BSP_UART_H
#define BOARD_C_INFANTRY_BSP_UART_H
#include "sys.h"
#include "Gimbal.h"

#define SBUS_RX_BUF_NUM 36u


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

/**
  * @brief          remote control init
  * @param[in]      none
  * @retval         none
  */
extern void remote_control_init(void);

/**
  * @brief          遥控器初始化
  * @param[in]      none
  * @retval         none
  */
extern void RC_init(uint8_t *rx1_buf, uint8_t *rx2_buf, uint16_t dma_buf_num);


/**
  * @brief     解析遥控器数据
  * @param     rc: 解析后的遥控器数据结构体指针
  * @param     buff: 串口接收到的遥控器原始数据指针
  */
static void remote_data_handle(RcTypeDef *rc, uint8_t *buff);
/**
  * @brief     发送遥控器数据给上板
  * @param     _hcan: 需要发送的can
  * @param     rc_data: 串口接收到的遥控器原始数据指针
  */
void Send_RC_Data(CAN_HandleTypeDef *_hcan, uint8_t *rc_data);

/**
  * @brief     发送 UART 数据
  * @param     uart_id: UART ID
  * @param     send_data: 发送数据指针
  * @param     size: 发送数据的长度
  */
void write_uart(uint8_t uart_id, uint8_t *send_data, uint16_t size);

/* 解析后的遥控器数据 */
extern RcTypeDef  rc;
extern uint8_t sbus_rx_buf[2][SBUS_RX_BUF_NUM];
extern uint8_t   bluetooth_recv[];
extern uint8_t   nuc_recv[];
extern uint8_t	 referee_recv[];
#endif //BOARD_C_INFANTRY_BSP_UART_H
