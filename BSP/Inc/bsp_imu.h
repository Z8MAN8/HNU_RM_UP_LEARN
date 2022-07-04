#ifndef BSP_SPI_H
#define BSP_SPI_H
#include "stdint.h"

extern void SPI1_DMA_init(uint32_t tx_buf, uint32_t rx_buf, uint16_t num);
extern void SPI1_DMA_enable(uint32_t tx_buf, uint32_t rx_buf, uint16_t ndtr);

/**
  * @brief     IMU 数据结构体
  */

typedef struct
{
    float acc_x;   //m/s^2
    float acc_y;   //m/s^2
    float acc_z;   //m/s^2
    float gyro_x;  //degree/s
    float gyro_y;  //degree/s
    float gyro_z;  //degree/s
    float angle_x; //degree
    float angle_y; //degree
    float angle_z; //degree
} ImuTypeDef;

void Imu_Get_data(ImuTypeDef *imu_data);
#endif
