#include "bsp_board.h"
#include "delay.h"
#include "bsp_usart3.h"
#include "bsp_usart1.h"
#include "bsp_emm_v5.h"
#include "bsp_gimbal.h"
#include "bsp_car.h"
#include "bsp_sw_i2c.h"
#include "bsp_grayscale.h"
#include "bsp_imu.h"
#include "bsp_packet.h"

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
***	初始化九轴传感器
**********************************************************/
	BSP_IMU_Init();
	USART1_SendString("九轴传感器初始化完成\r\n");
	
	// 等待九轴传感器稳定
	delay_ms(1000);
	
	// 检查九轴传感器连接状态
	if(BSP_IMU_IsConnected())
	{
		USART1_SendString("九轴传感器连接正常!\r\n");
	}
	else
	{
		USART1_SendString("警告: 九轴传感器连接异常!\r\n");
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
	USART1_SendString("  k - 云台上仰45度\r\n");
	USART1_SendString("  l - 云台下俯45度\r\n");
	USART1_SendString("  n - 云台左转45度\r\n");
	USART1_SendString("  m - 云台右转45度\r\n");
	USART1_SendString("  o - 云台归零位置\r\n");
	USART1_SendString("轨迹同步:\r\n");
	USART1_SendString("  T - 开始轨迹同步\r\n");
	USART1_SendString("  S - 结束轨迹同步\r\n");
	USART1_SendString("系统状态:\r\n");
	USART1_SendString("  A - 获取小车位置信息\r\n");
	USART1_SendString("  B - 获取云台状态信息\r\n");
	USART1_SendString("  C - 重置小车位置坐标\r\n");
	USART1_SendString("八路寻迹传感器:\r\n");
	USART1_SendString("  D - 读取八路寻迹传感器数据\r\n");
	USART1_SendString("  E - 扫描IIC设备\r\n");
	USART1_SendString("九轴传感器:\r\n");
	USART1_SendString("  F - 读取九轴传感器完整数据\r\n");
	USART1_SendString("  G - 读取加速度数据\r\n");
	USART1_SendString("  H - 读取角速度数据\r\n");
	USART1_SendString("  I - 读取欧拉角数据\r\n");
	USART1_SendString("  J - 读取四元数数据\r\n");
	USART1_SendString("  K - 读取GPS位置数据\r\n");
	USART1_SendString("  L - 检查九轴传感器连接状态\r\n");
	USART1_SendString("  N - 九轴传感器实时监控 (开/关)\r\n");
	USART1_SendString("等待蓝牙命令...\r\n\r\n");

/**********************************************************
***	云台和小车状态变量
**********************************************************/
	static float gimbal_yaw = 0.0f;
	static float gimbal_pitch = 0.0f;
	
/**********************************************************
***	九轴传感器相关变量
**********************************************************/
	static bool imu_monitor_enabled = false;  // 九轴传感器实时监控开关
	static uint32_t imu_monitor_counter = 0;  // 监控计数器
	static uint32_t last_imu_print_time = 0;  // 上次打印时间
	
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
		// 处理九轴传感器数据
        BSP_IMU_ProcessData();
		
		// 九轴传感器实时监控
		if(imu_monitor_enabled)
		{
			imu_monitor_counter++;
			// 每500ms打印一次数据 (假设主循环10ms一次)
			if(imu_monitor_counter >= 50)
			{
				IMU_Data_t imu_data;
				if(BSP_IMU_GetData(&imu_data))
				{
					USART1_Printf("[监控] Pitch:%.1f° Roll:%.1f° Yaw:%.1f° | ", 
								 imu_data.pitch, imu_data.roll, imu_data.yaw);
					USART1_Printf("Acc:%.2f,%.2f,%.2f m/s²\r\n", 
								 imu_data.accel_x, imu_data.accel_y, imu_data.accel_z);
				}
				imu_monitor_counter = 0;
			}
		}

		// USART1蓝牙命令处理
		if(USART1_RxFlag)
		{
			// 使用新的数据包处理模块
			Packet_ProcessReceived((uint8_t*)usart1_rxCmd, usart1_rxCount);
			
			// 如果不是数据包，则按原有单字符命令处理
			if(Packet_IdentifyType((uint8_t*)usart1_rxCmd, usart1_rxCount) == PACKET_TYPE_COMMAND)
			{
				// 原有的单字符命令处理逻辑
				char command = USART1_RxPacket[0];
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
					case 'k':  // 上仰45度
						gimbal_pitch += 0.1f;
						Gimbal_SetPitchAngle(gimbal_pitch);
						USART1_Printf("云台上仰45度，当前俯仰角: %.1f度\r\n", gimbal_pitch);
						break;
						
					case 'l':  // 下俯45度
						gimbal_pitch -= 45.0f;
						Gimbal_SetPitchAngle(gimbal_pitch);
						USART1_Printf("云台下俯45度，当前俯仰角: %.1f度\r\n", gimbal_pitch);
						break;
						
					case 'n':  // 左转45度
						gimbal_yaw -= 45.0f;
						Gimbal_SetYawAngle(gimbal_yaw);
						USART1_Printf("云台左转45度，当前偏航角: %.1f度\r\n", gimbal_yaw);
						break;
						
					case 'm':  // 右转45度
						gimbal_yaw += 45.0f;
						Gimbal_SetYawAngle(gimbal_yaw);
						USART1_Printf("云台右转45度，当前偏航角: %.1f度\r\n", gimbal_yaw);
						break;
						
					case 'o':  // 云台归零
						gimbal_yaw = 0.0f;
						gimbal_pitch = 0.0f;
						Gimbal_SetYawAngle(gimbal_yaw);
						delay_ms(10);
						Gimbal_SetPitchAngle(gimbal_pitch);
						USART1_SendString("云台归零位置\r\n");
						break;
						
					// 状态查询
					case 'A':  // 获取小车位置
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
					
					case 'B':  // 获取云台状态
					{
						Gimbal_State_t gimbal_state = Gimbal_GetState();
						USART1_SendString("=== 云台状态信息 ===\r\n");
						USART1_Printf("俯仰角: %.1f度\r\n", gimbal_state.pitch_angle);
						USART1_Printf("偏航角: %.1f度\r\n", gimbal_state.yaw_angle);
						USART1_Printf("俯仰速度: %d RPM\r\n", gimbal_state.pitch_speed);
						USART1_Printf("偏航速度: %d RPM\r\n", gimbal_state.yaw_speed);
						
						// 显示轨迹同步状态
//						Trajectory_Sync_State_t sync_state = Gimbal_GetTrajectoryState();
//						USART1_Printf("轨迹同步状态: %s\r\n", 
//									 sync_state == TRAJECTORY_SYNC_RUNNING ? "运行中" : "已停止");
						break;
					}
					
					case 'C':  // 重置小车位置
						Car_ResetPosition();
						USART1_SendString("小车位置坐标已重置\r\n");
						break;

					case 'D':  // 读取八路寻迹传感器数据
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
					
					case 'E':  // 扫描IIC设备
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
					
					// 九轴传感器命令
					case 'F':  // 读取九轴传感器完整数据
					{
						BSP_IMU_PrintData();
						break;
					}
					
					case 'G':  // 读取加速度数据
					{
						IMU_Data_t imu_data;
						if(BSP_IMU_GetData(&imu_data))
						{
							USART1_SendString("=== 加速度数据 ===\r\n");
							USART1_Printf("X轴: %.3f m/s²\r\n", imu_data.accel_x);
							USART1_Printf("Y轴: %.3f m/s²\r\n", imu_data.accel_y);
							USART1_Printf("Z轴: %.3f m/s²\r\n", imu_data.accel_z);
							USART1_Printf("合成: %.3f m/s²\r\n", 
										 sqrtf(imu_data.accel_x*imu_data.accel_x + 
											   imu_data.accel_y*imu_data.accel_y + 
											   imu_data.accel_z*imu_data.accel_z));
						}
						else
						{
							USART1_SendString("九轴传感器无有效数据\r\n");
						}
						break;
					}
					
					case 'H':  // 读取角速度数据
					{
						IMU_Data_t imu_data;
						if(BSP_IMU_GetData(&imu_data))
						{
							USART1_SendString("=== 角速度数据 ===\r\n");
							USART1_Printf("X轴: %.3f °/s\r\n", imu_data.gyro_x);
							USART1_Printf("Y轴: %.3f °/s\r\n", imu_data.gyro_y);
							USART1_Printf("Z轴: %.3f °/s\r\n", imu_data.gyro_z);
						}
						else
						{
							USART1_SendString("九轴传感器无有效数据\r\n");
						}
						break;
					}
					
					case 'I':  // 读取欧拉角数据
					{
						IMU_Data_t imu_data;
						if(BSP_IMU_GetData(&imu_data))
						{
							USART1_SendString("=== 欧拉角数据 ===\r\n");
							USART1_Printf("俯仰角(Pitch): %.2f°\r\n", imu_data.pitch);
							USART1_Printf("横滚角(Roll):  %.2f°\r\n", imu_data.roll);
							USART1_Printf("偏航角(Yaw):   %.2f°\r\n", imu_data.yaw);
						}
						else
						{
							USART1_SendString("九轴传感器无有效数据\r\n");
						}
						break;
					}
					
					case 'J':  // 读取四元数数据
					{
						IMU_Data_t imu_data;
						if(BSP_IMU_GetData(&imu_data))
						{
							USART1_SendString("=== 四元数数据 ===\r\n");
							USART1_Printf("W: %.4f\r\n", imu_data.quaternion_w);
							USART1_Printf("X: %.4f\r\n", imu_data.quaternion_x);
							USART1_Printf("Y: %.4f\r\n", imu_data.quaternion_y);
							USART1_Printf("Z: %.4f\r\n", imu_data.quaternion_z);
							USART1_Printf("模长: %.4f\r\n", 
										 sqrtf(imu_data.quaternion_w*imu_data.quaternion_w + 
											   imu_data.quaternion_x*imu_data.quaternion_x + 
											   imu_data.quaternion_y*imu_data.quaternion_y + 
											   imu_data.quaternion_z*imu_data.quaternion_z));
						}
						else
						{
							USART1_SendString("九轴传感器无有效数据\r\n");
						}
						break;
					}
					
					case 'K':  // 读取GPS位置数据
					{
						IMU_Data_t imu_data;
						if(BSP_IMU_GetData(&imu_data))
						{
							USART1_SendString("=== GPS位置数据 ===\r\n");
							if(imu_data.latitude != 0.0 || imu_data.longitude != 0.0)
							{
								USART1_Printf("纬度: %.7f°\r\n", imu_data.latitude);
								USART1_Printf("经度: %.7f°\r\n", imu_data.longitude);
								USART1_Printf("海拔: %.2f m\r\n", imu_data.altitude);
								USART1_Printf("北向速度: %.3f m/s\r\n", imu_data.vel_north);
								USART1_Printf("东向速度: %.3f m/s\r\n", imu_data.vel_east);
								USART1_Printf("垂直速度: %.3f m/s\r\n", imu_data.vel_down);
							}
							else
							{
								USART1_SendString("GPS信号无效或未定位\r\n");
							}
						}
						else
						{
							USART1_SendString("九轴传感器无有效数据\r\n");
						}
						break;
					}
					
					case 'L':  // 检查九轴传感器连接状态
					{
						if(BSP_IMU_IsConnected())
						{
							USART1_SendString("九轴传感器: 连接正常 ✓\r\n");
							
							// 显示详细状态信息
							IMU_Data_t imu_data;
							if(BSP_IMU_GetData(&imu_data))
							{
								USART1_Printf("数据时间戳: %u us\r\n", imu_data.data_ready_timestamp);
								USART1_SendString("数据状态: 有效\r\n");
							}
							else
							{
								USART1_SendString("数据状态: 暂无新数据\r\n");
							}
						}
						else
						{
							USART1_SendString("九轴传感器: 连接异常 ✗\r\n");
							USART1_SendString("请检查:\r\n");
							USART1_SendString("1. 硬件连接 (PA2-TX, PA3-RX)\r\n");
							USART1_SendString("2. 电源供电 (3.3V/5V)\r\n");
							USART1_SendString("3. 波特率设置 (460800)\r\n");
						}
						break;
					}
					
					case 'N':  // 九轴传感器实时监控开关
					{
						imu_monitor_enabled = !imu_monitor_enabled;
						imu_monitor_counter = 0;
						
						if(imu_monitor_enabled)
						{
							USART1_SendString("九轴传感器实时监控: 已开启\r\n");
							USART1_SendString("将每500ms显示姿态和加速度数据\r\n");
							USART1_SendString("再次发送 '8' 命令可关闭监控\r\n");
						}
						else
						{
							USART1_SendString("九轴传感器实时监控: 已关闭\r\n");
						}
						break;
					}
					
					default:
						USART1_Printf("未知命令: %c\r\n", command);
						USART1_SendString("请发送有效命令\r\n");
						USART1_SendString("小车: a-j,s | 云台: u,n,l,r,z | 状态: p,q,x\r\n");
						USART1_SendString("传感器: t,m | 九轴: 1-8\r\n");
						break;
				}
				// 清除接收标志
				USART1_RxFlag = 0;
			}
			// 清除接收标志
			USART1_RxFlag = 0;
		}
		
		delay_ms(10);  // 主循环延时
	}
}
