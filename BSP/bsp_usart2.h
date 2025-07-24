#ifndef __BSP_USART2_H
#define __BSP_USART2_H

#include "stm32f10x.h"
#include "fifo.h"
#include <stdio.h>

// USART2接收缓冲区大小
#define USART2_RX_BUFFER_SIZE 256

// USART2接收相关变量
extern __IO bool usart2_rxFrameFlag;
extern __IO uint8_t usart2_rxCmd[USART2_RX_BUFFER_SIZE];
extern __IO uint8_t usart2_rxCount;

// USART2发送函数
void BSP_USART2_SendByte(uint8_t data);
void BSP_USART2_SendArray(uint8_t *array, uint16_t length);
void BSP_USART2_SendString(char *string);

// USART2接收函数
bool BSP_USART2_GetFrame(uint8_t *data, uint8_t *length);
void BSP_USART2_ClearFrame(void);

#endif