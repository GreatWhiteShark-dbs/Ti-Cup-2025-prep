#include "board.h"
#include "delay.h"
#include "bsp_usart3.h"
#include "bsp_usart1.h"
#include "Emm_V5.h"
#include "gimbal.h"

/**
	*	@brief		MAIN函数
	*	@param		无
	*	@retval		无
	*/
int main(void)
{
/**********************************************************
***	初始化板载外设
**********************************************************/
	board_init();

/**********************************************************
***	上电延时2秒等待Emm_V5.0闭环初始化完毕
**********************************************************/	
	delay_ms(2000);

/**********************************************************
***	USART1测试用例
**********************************************************/
	USART1_SendString("USART1 Test Started!\r\n");
	USART1_Printf("System Clock: %d Hz\r\n", SystemCoreClock);
	USART1_SendString("Waiting for commands...\r\n");

/**********************************************************
***	位置模式：方向CW，速度1000RPM，加速度0（不使用加减速直接启动），脉冲数3200（16细分下发送3200个脉冲电机转一圈），相对运动
**********************************************************/	
  //Emm_V5_Pos_Control(1, 0, 50, 1, 320000, 0, 0);
	Gimbal_SetYawAngle(1800);
	delay_ms(2);
	Gimbal_SetPitchAngle(1800);   
	
/**********************************************************
***	等待返回命令，命令数据缓存在数组rxCmd上，长度为rxCount
**********************************************************/	
//	while(rxFrameFlag == false); rxFrameFlag = false;

/**********************************************************
***	WHILE循环
**********************************************************/	
	uint32_t counter = 0;
	while(1)
	{
		// USART1接收测试
		if(USART1_RxFlag)
		{
			USART1_Printf("Received: %s\r\n", USART1_RxPacket);
			USART1_RxFlag = 0;  // 清除接收标志
		}
		
		// 定时发送测试数据
		counter++;
		if(counter >= 1000000)  // 大约每秒发送一次（取决于系统时钟）
		{
			counter = 0;
			USART1_Printf("Heartbeat: %d\r\n", (int)(counter/1000000));
			USART1_SendString("System running...\r\n");
		}
		
		delay_ms(1);  // 短暂延时
	}
}
