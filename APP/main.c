#include "bsp_board.h"
#include "delay.h"
#include "bsp_usart3.h"
#include "bsp_usart1.h"
#include "bsp_emm_v5.h"
#include "bsp_gimbal.h"
#include "bsp_car.h"
#include "bsp_sw_i2c.h"
#include "bsp_grayscale.h"

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
***	初始化小车和云台系统
**********************************************************/
	Car_Init();
	Gimbal_Init();

/**********************************************************
***	初始化软件IIC和八路寻迹传感器
**********************************************************/
	BSP_SW_I2C_Init();
	
	if(BSP_Grayscale_Init() == 0)
	{
		USART1_SendString("八路寻迹传感器初始化成功!\r\n");
	}
	else
	{
		USART1_SendString("警告: 八路寻迹传感器初始化失败!\r\n");
	}

/**********************************************************
***	蓝牙控制系统启动信息
**********************************************************/
	USART1_SendString("=== 蓝牙控制系统启动 ===\r\n");
	USART1_Printf("系统时钟: %d Hz\r\n", SystemCoreClock);
	USART1_SendString("控制命令说明:\r\n");
	USART1_SendString("小车控制:\r\n");
	USART1_SendString("  a - 前进 (速度模式)\r\n");
	USART1_SendString("  b - 后退 (速度模式)\r\n");
	USART1_SendString("  c - 左转 (速度模式)\r\n");
	USART1_SendString("  d - 右转 (速度模式)\r\n");
	USART1_SendString("  e - 原地左旋 (速度模式)\r\n");
	USART1_SendString("  f - 原地右旋 (速度模式)\r\n");
	USART1_SendString("  g - 前进100mm (位置模式)\r\n");
	USART1_SendString("  h - 后退100mm (位置模式)\r\n");
	USART1_SendString("  i - 左转90度 (位置模式)\r\n");
	USART1_SendString("  j - 右转90度 (位置模式)\r\n");
	USART1_SendString("  s - 停止\r\n");
	USART1_SendString("云台控制:\r\n");
	USART1_SendString("  u - 云台上仰45度\r\n");
	USART1_SendString("  n - 云台下俯45度\r\n");
	USART1_SendString("  l - 云台左转45度\r\n");
	USART1_SendString("  r - 云台右转45度\r\n");
	USART1_SendString("  z - 云台归零位置\r\n");
	USART1_SendString("系统状态:\r\n");
	USART1_SendString("  p - 获取小车位置信息\r\n");
	USART1_SendString("  q - 获取云台状态信息\r\n");
	USART1_SendString("  x - 重置小车位置坐标\r\n");
	USART1_SendString("八路寻迹:\r\n");
	USART1_SendString("  t - 读取八路寻迹传感器数据\r\n");
	USART1_SendString("  m - 扫描IIC设备\r\n");
	USART1_SendString("等待蓝牙命令...\r\n\r\n");

/**********************************************************
***	云台和小车状态变量
**********************************************************/
	static float gimbal_yaw = 0.0f;
	static float gimbal_pitch = 0.0f;
	
/**********************************************************
***	设置初始位置
**********************************************************/	
	Gimbal_SetYawAngle(gimbal_yaw);
	delay_ms(10);
	Gimbal_SetPitchAngle(gimbal_pitch);
	Car_ResetPosition();
	
/**********************************************************
***	WHILE循环 - 蓝牙命令处理
**********************************************************/	
	while(1)
	{
		// USART1蓝牙命令处理
		if(USART1_RxFlag)
		{
			char command = USART1_RxPacket[0];  // 获取第一个字符作为命令
			
			USART1_Printf("收到命令: %c\r\n", command);
			
			switch(command)
			{
				// 小车速度模式控制
				case 'a':  // 前进
					Car_SetSpeed_Forward(100);
					USART1_SendString("小车前进 (100 RPM)\r\n");
					break;
					
				case 'b':  // 后退
					Car_SetSpeed_Backward(100);
					USART1_SendString("小车后退 (100 RPM)\r\n");
					break;
					
				case 'c':  // 左转
					Car_SetSpeed_TurnLeft(100);
					USART1_SendString("小车左转 (100 RPM)\r\n");
					break;
					
				case 'd':  // 右转
					Car_SetSpeed_TurnRight(100);
					USART1_SendString("小车右转 (100 RPM)\r\n");
					break;
					
				case 'e':  // 原地左旋
					Car_SetSpeed_RotateLeft(80);
					USART1_SendString("小车原地左旋 (80 RPM)\r\n");
					break;
					
				case 'f':  // 原地右旋
					Car_SetSpeed_RotateRight(80);
					USART1_SendString("小车原地右旋 (80 RPM)\r\n");
					break;
					
				// 小车位置模式控制
				case 'g':  // 前进100mm
					Car_SetPos_MoveDistance(1000.0f);
					USART1_SendString("小车前进 100mm\r\n");
					break;
					
				case 'h':  // 后退100mm
					Car_SetPos_MoveDistance(-1000.0f);
					USART1_SendString("小车后退 100mm\r\n");
					break;
					
				case 'i':  // 左转90度
					Car_SetPos_RotateAngle(-90.0f);
					USART1_SendString("小车左转 90度\r\n");
					break;
					
				case 'j':  // 右转90度
					Car_SetPos_RotateAngle(90.0f);
					USART1_SendString("小车右转 90度\r\n");
					break;
					
				case 's':  // 停止
					Car_Stop();
					USART1_SendString("小车停止\r\n");
					break;
					
				// 云台控制
				case 'u':  // 上仰45度
					gimbal_pitch += 45.0f;
					Gimbal_SetPitchAngle(gimbal_pitch);
					USART1_Printf("云台上仰45度，当前俯仰角: %.1f度\r\n", gimbal_pitch);
					break;
					
				case 'n':  // 下俯45度
					gimbal_pitch -= 45.0f;
					Gimbal_SetPitchAngle(gimbal_pitch);
					USART1_Printf("云台下俯45度，当前俯仰角: %.1f度\r\n", gimbal_pitch);
					break;
					
				case 'l':  // 左转45度
					gimbal_yaw -= 45.0f;
					Gimbal_SetYawAngle(gimbal_yaw);
					USART1_Printf("云台左转45度，当前偏航角: %.1f度\r\n", gimbal_yaw);
					break;
					
				case 'r':  // 右转45度
					gimbal_yaw += 45.0f;
					Gimbal_SetYawAngle(gimbal_yaw);
					USART1_Printf("云台右转45度，当前偏航角: %.1f度\r\n", gimbal_yaw);
					break;
					
				case 'z':  // 云台归零
					gimbal_yaw = 0.0f;
					gimbal_pitch = 0.0f;
					Gimbal_SetYawAngle(gimbal_yaw);
					delay_ms(10);
					Gimbal_SetPitchAngle(gimbal_pitch);
					USART1_SendString("云台归零位置\r\n");
					break;
					
				// 状态查询
				case 'p':  // 获取小车位置
				{
					Car_State_t car_state = Car_GetState();
					USART1_SendString("=== 小车状态信息 ===\r\n");
					USART1_Printf("位置: X=%.1fmm, Y=%.1fmm\r\n", car_state.x_position, car_state.y_position);
					USART1_Printf("航向角: %.1f度\r\n", car_state.heading);
					USART1_Printf("轮速: FL=%d, FR=%d, RL=%d, RR=%d RPM\r\n", 
						car_state.front_left_speed, car_state.front_right_speed,
						car_state.rear_left_speed, car_state.rear_right_speed);
					USART1_Printf("运动模式: %d\r\n", car_state.current_mode);
					break;
				}
				
				case 'q':  // 获取云台状态
				{
					Gimbal_State_t gimbal_state = Gimbal_GetState();
					USART1_SendString("=== 云台状态信息 ===\r\n");
					USART1_Printf("俯仰角: %.1f度\r\n", gimbal_state.pitch_angle);
					USART1_Printf("偏航角: %.1f度\r\n", gimbal_state.yaw_angle);
					USART1_Printf("俯仰速度: %d RPM\r\n", gimbal_state.pitch_speed);
					USART1_Printf("偏航速度: %d RPM\r\n", gimbal_state.yaw_speed);
					break;
				}
				
				case 'x':  // 重置小车位置
					Car_ResetPosition();
					USART1_SendString("小车位置坐标已重置\r\n");
					break;

				case 't':  // 读取八路寻迹传感器数据
				{
					Grayscale_Data_t grayscale_data;
					if(BSP_Grayscale_ReadAll(&grayscale_data) == 0)
					{
						BSP_Grayscale_PrintData(&grayscale_data);
					}
					else
					{
						USART1_SendString("读取八路寻迹传感器失败!\r\n");
					}
					break;
				}
				
				case 'm':  // 扫描IIC设备
				{
					uint8_t scan_addr[128] = {0};
					uint8_t count = BSP_SW_I2C_Scan(scan_addr);
					
					USART1_SendString("=== IIC设备扫描结果 ===\r\n");
					USART1_Printf("发现 %d 个设备:\r\n", count);
					
					for(uint8_t i = 0; i < count; i++)
					{
						USART1_Printf("设备 %d: 地址 0x%02X\r\n", i+1, scan_addr[i]);
					}
					
					if(count == 0)
					{
						USART1_SendString("未发现任何IIC设备\r\n");
					}
					break;
				}
				
				default:
					USART1_Printf("未知命令: %c\r\n", command);
					USART1_SendString("请发送有效命令 (a-z)\r\n");
					break;
			}
			
			// 清除接收标志
			USART1_RxFlag = 0;
			USART1_SendString("命令执行完成\r\n\r\n");
		}
		
		delay_ms(10);  // 主循环延时
	}
}
