#include "bsp_board.h"
#include "delay.h"
#include "bsp_usart3.h"
#include "bsp_usart1.h"
#include "bsp_emm_v5.h"
#include "bsp_gimbal.h"

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
***	USART1测试用例 - 角度累加变量
**********************************************************/
	static float yaw_angle = 0.0f;    // 偏航角度，初始180度
	static float pitch_angle = 0.0f;  // 俯仰角度，初始180度
	
	USART1_SendString("USART1 Motor Control Test Started!\r\n");
	USART1_Printf("System Clock: %d Hz\r\n", SystemCoreClock);
	USART1_SendString("Send any data to make motors rotate 360 degrees...\r\n");
	USART1_Printf("Initial Position - Yaw: %.1f, Pitch: %.1f\r\n", yaw_angle, pitch_angle);

/**********************************************************
***	设置初始位置
**********************************************************/	
	Gimbal_SetYawAngle(yaw_angle);
	delay_ms(2);
	Gimbal_SetPitchAngle(pitch_angle);   
	
/**********************************************************
***	WHILE循环
**********************************************************/	
	while(1)
	{
		// USART1接收测试 - 接收到数据后返回数据并让电机转一圈
		if(USART1_RxFlag)
		{
			// 返回接收到的数据
			USART1_Printf("Received: %s\r\n", USART1_RxPacket);
			
			// 角度累加360度（转一圈）
			yaw_angle += 360.0f;
			pitch_angle += 360.0f;
			
			// 发送电机控制命令
			USART1_Printf("Rotating motors - New Position: Yaw: %.1f, Pitch: %.1f\r\n", yaw_angle, pitch_angle);
			
			// 控制电机转动
			Gimbal_SetYawAngle(yaw_angle);
			delay_ms(10);  // 短暂延时确保命令发送
			Gimbal_SetPitchAngle(pitch_angle);
			
			USART1_SendString("Motors rotation command sent!\r\n");
			
			// 清除接收标志
			USART1_RxFlag = 0;
		}
		
		delay_ms(10);  // 主循环延时
	}
}
