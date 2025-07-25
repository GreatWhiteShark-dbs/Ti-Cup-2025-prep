#ifndef __DELAY_H
#define __DELAY_H

#include "stm32f10x.h"

void delay_ms(int32_t i32Cnt);
void delay_cnt(int32_t i32Cnt);
void delay_us(uint32_t nus);  // 微秒延时函数

#endif
