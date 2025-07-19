#ifndef __BSP_USART1_H
#define __BSP_USART1_H

#include <stdio.h>
#include "stm32f10x.h"
#include "fifo.h"

extern char USART1_RxPacket[];
extern uint8_t USART1_RxFlag;

// 新增USART1专用的接收变量（参考USART3）
extern __IO bool usart1_rxFrameFlag;
extern __IO uint8_t usart1_rxCmd[FIFO_SIZE];
extern __IO uint8_t usart1_rxCount;

void USART1_SendByte(uint8_t Byte);
void USART1_SendArray(uint8_t *Array, uint16_t Length);
void USART1_SendString(char *String);
void USART1_SendNumber(uint32_t Number, uint8_t Length);
void USART1_Printf(char *format, ...);

#endif
