#include "bsp_usart1.h"
#include "fifo.h"
#include <stdio.h>
#include <stdarg.h>

char USART1_RxPacket[100];				//"@MSG\r\n"
uint8_t USART1_RxFlag;
__IO bool usart1_rxFrameFlag = false;
__IO uint8_t usart1_rxCmd[FIFO_SIZE] = {0};
__IO uint8_t usart1_rxCount = 0;

void USART1_SendByte(uint8_t Byte)
{
	USART_SendData(USART1, Byte);
	while (USART_GetFlagStatus(USART1, USART_FLAG_TXE) == RESET);
}

void USART1_SendArray(uint8_t *Array, uint16_t Length)
{
	uint16_t i;
	for (i = 0; i < Length; i ++)
	{
		USART1_SendByte(Array[i]);
	}
}

void USART1_SendString(char *String)
{
	uint8_t i;
	for (i = 0; String[i] != '\0'; i ++)
	{
		USART1_SendByte(String[i]);
	}
}

uint32_t USART1_Pow(uint32_t X, uint32_t Y)
{
	uint32_t Result = 1;
	while (Y --)
	{
		Result *= X;
	}
	return Result;
}

void USART1_SendNumber(uint32_t Number, uint8_t Length)
{
	uint8_t i;
	for (i = 0; i < Length; i ++)
	{
		USART1_SendByte(Number / USART1_Pow(10, Length - i - 1) % 10 + '0');
	}
}

int fputc(int ch, FILE *f)
{
	USART1_SendByte(ch);
	return ch;
}

void USART1_Printf(char *format, ...)
{
	char String[100];
	va_list arg;
	va_start(arg, format);
	vsprintf(String, format, arg);
	va_end(arg);
	USART1_SendString(String);
}

/**
	* @brief   USART1中断函数 - 参考USART3实现
	* @param   无
	* @retval  无
	*/
void USART1_IRQHandler(void)
{
	__IO uint16_t i = 0;

/**********************************************************
***	串口接收中断
**********************************************************/
	if(USART_GetITStatus(USART1, USART_IT_RXNE) != RESET)
	{
		// 未完成一帧数据接收，数据进入缓冲队列
		fifo_enQueue((uint8_t)USART1->DR);

		// 清除串口接收中断
		USART_ClearITPendingBit(USART1, USART_IT_RXNE);
	}

/**********************************************************
***	串口空闲中断
**********************************************************/
	else if(USART_GetITStatus(USART1, USART_IT_IDLE) != RESET)
	{
		// 先读SR再读DR，清除IDLE中断
		USART1->SR; USART1->DR;

		// 提取一帧数据命令
		usart1_rxCount = fifo_queueLength(); 
		for(i=0; i < usart1_rxCount; i++) 
		{ 
			usart1_rxCmd[i] = fifo_deQueue(); 
		}

		// 一帧数据接收完成，置位帧标志位
		usart1_rxFrameFlag = true;
		
		// 兼容原有的数据包格式处理
		if(usart1_rxCount > 0)
		{
			// 复制到原有的接收缓冲区
			for(i = 0; i < usart1_rxCount && i < 99; i++)
			{
				USART1_RxPacket[i] = usart1_rxCmd[i];
			}
			USART1_RxPacket[i] = '\0';
			USART1_RxFlag = 1;
		}
	}
}
