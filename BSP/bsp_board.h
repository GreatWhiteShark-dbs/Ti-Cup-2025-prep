#ifndef __BSP_BOARD_H
#define __BSP_BOARD_H

#include "stm32f10x.h"

void nvic_init(void);
void clock_init(void);
void usart1_init(void);
void usart2_init(void);
void usart3_init(void);
void board_init(void);

#endif
