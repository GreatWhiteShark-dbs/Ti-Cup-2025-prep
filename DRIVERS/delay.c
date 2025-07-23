#include "delay.h"

/**
	*	@brief		毫秒级延时
	*	@param		int32_t u32Cnt
	*	@retval		无
	*/
void delay_ms(int32_t i32Cnt)
{
	__IO int32_t i32end = 0;

	SysTick->LOAD = 0xFFFFFF;
	SysTick->VAL  = 0;
	SysTick->CTRL = (SysTick_CTRL_ENABLE_Msk | SysTick_CTRL_CLKSOURCE_Msk);

	while(i32Cnt > 0)
	{
		SysTick->VAL = 0;
		i32end = 0x1000000 - (SystemCoreClock / 1000);
		while(SysTick->VAL > i32end);
		--i32Cnt;
	}

	SysTick->CTRL = (SysTick->CTRL & (~SysTick_CTRL_ENABLE_Msk));
}

/**
	*	@brief		软件延时
	*	@param		int32_t u32Cnt
	*	@retval		无
	*/
void delay_cnt(int32_t i32Cnt)
{
	while(i32Cnt > 0) { i32Cnt--; }
}

/**
	*	@brief		微秒级延时
	*	@param		uint32_t nus
	*	@retval		无
	*/
void delay_us(uint32_t nus)
{
	uint32_t temp;
	uint32_t fac_us = SystemCoreClock / 8000000;  // 计算微秒因子
	
	SysTick->LOAD = nus * fac_us;   // 设置加载值
	SysTick->VAL = 0x00;            // 清空计数器
	SysTick->CTRL &= ~(1<<2);       // 选择8分频
	SysTick->CTRL |= SysTick_CTRL_ENABLE_Msk;  // 使能计数器
	
	do
	{
		temp = SysTick->CTRL;
	}while((temp & 0x01) && !(temp & (1<<16)));   // 等待计数完成
	
	SysTick->CTRL &= ~SysTick_CTRL_ENABLE_Msk;    // 关闭计数器
	SysTick->VAL = 0X00;                          // 清空计数器
}
