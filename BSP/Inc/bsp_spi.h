//
// Created by turboDog on 2021/11/27.
//

#ifndef BOARD_C_INFANTRY_BSP_SPI_H
#define BOARD_C_INFANTRY_BSP_SPI_H
#include "stdint.h"

extern void SPI1_DMA_init(uint32_t tx_buf, uint32_t rx_buf, uint16_t num);
extern void SPI1_DMA_enable(uint32_t tx_buf, uint32_t rx_buf, uint16_t ndtr);

#endif //BOARD_C_INFANTRY_BSP_SPI_H
