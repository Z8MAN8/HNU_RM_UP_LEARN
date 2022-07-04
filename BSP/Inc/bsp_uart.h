//
// Created by turboDog on 2021/11/16.
//

#ifndef BOARD_C_INFANTRY_BSP_UART_H
#define BOARD_C_INFANTRY_BSP_UART_H
#include "bsp_rc.h"
#include "sys.h"
#include "Gimbal.h"

#define SBUS_RX_BUF_NUM 36u
/* 解析后的遥控器数据 */
extern RcTypeDef  rc;
extern uint8_t sbus_rx_buf[2][SBUS_RX_BUF_NUM];
extern uint8_t   bluetooth_recv[];
extern uint8_t   nuc_recv[];
extern uint8_t	 referee_recv[];


/**
  * @brief          remote control init
  * @param[in]      none
  * @retval         none
  */
/**
  * @brief          遥控器初始化
  * @param[in]      none
  * @retval         none
  */
extern void remote_control_init(void);
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
#endif //BOARD_C_INFANTRY_BSP_UART_H
