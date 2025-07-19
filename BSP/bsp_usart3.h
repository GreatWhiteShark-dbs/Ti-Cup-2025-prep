#ifndef __BSP_USART3_H
#define __BSP_USART3_H

#include "board.h"
#include "fifo.h"

extern __IO bool rxFrameFlag;
extern __IO uint8_t rxCmd[FIFO_SIZE];
extern __IO uint8_t rxCount;

void usart3_SendCmd(__IO uint8_t *cmd, uint8_t len);
void usart3_SendByte(uint16_t data);

#endif