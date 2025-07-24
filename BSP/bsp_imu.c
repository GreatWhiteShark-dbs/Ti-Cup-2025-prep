#include "bsp_imu.h"
#include "bsp_usart2.h"
#include "bsp_usart1.h"
#include <string.h>
#include <stddef.h>

// 内部变量
static IMU_Data_t imu_data;
static bool data_updated = false;
static uint32_t last_data_time = 0;

// 外部变量声明
extern IMU_Protocol_Info_t g_imu_protocol_info;
extern IMU_Payload_Data_t *payload;

/**
 * @brief  IMU初始化函数
 * @param  None
 * @retval None
 */
void BSP_IMU_Init(void)
{
    // 初始化IMU数据结构体
    memset(&imu_data, 0, sizeof(IMU_Data_t));
    data_updated = false;
    last_data_time = 0;
    
    USART1_Printf("IMU initialized\r\n");
}

/**
 * @brief  IMU数据处理函数
 * @param  None
 * @retval None
 */
void BSP_IMU_ProcessData(void)
{
    uint8_t rx_data[USART2_RX_BUFFER_SIZE];
    uint8_t rx_length = 0;
    
    // 检查是否有新的串口数据
    if (BSP_USART2_GetFrame(rx_data, &rx_length))
    {
        // 处理接收到的数据
        int result = BSP_IMU_Protocol_AnalysisData(rx_data, rx_length);
        
        if (result == IMU_ANALYSIS_OK)
        {
            // 数据解析成功，更新IMU数据结构体
            imu_data.accel_x = g_imu_protocol_info.accel_x;
            imu_data.accel_y = g_imu_protocol_info.accel_y;
            imu_data.accel_z = g_imu_protocol_info.accel_z;
            
            imu_data.gyro_x = g_imu_protocol_info.angle_x;
            imu_data.gyro_y = g_imu_protocol_info.angle_y;
            imu_data.gyro_z = g_imu_protocol_info.angle_z;
            
            imu_data.mag_x = g_imu_protocol_info.mag_x;
            imu_data.mag_y = g_imu_protocol_info.mag_y;
            imu_data.mag_z = g_imu_protocol_info.mag_z;
            
            imu_data.pitch = g_imu_protocol_info.pitch;
            imu_data.roll = g_imu_protocol_info.roll;
            imu_data.yaw = g_imu_protocol_info.yaw;
            
            imu_data.quaternion_w = g_imu_protocol_info.quaternion_data0;
            imu_data.quaternion_x = g_imu_protocol_info.quaternion_data1;
            imu_data.quaternion_y = g_imu_protocol_info.quaternion_data2;
            imu_data.quaternion_z = g_imu_protocol_info.quaternion_data3;
            
            imu_data.latitude = g_imu_protocol_info.latitude;
            imu_data.longitude = g_imu_protocol_info.longtidue;
            imu_data.altitude = g_imu_protocol_info.altidue;
            
            imu_data.vel_north = g_imu_protocol_info.vel_n;
            imu_data.vel_east = g_imu_protocol_info.vel_e;
            imu_data.vel_down = g_imu_protocol_info.vel_d;
            
            imu_data.sample_timestamp = g_imu_protocol_info.sample_timestamp;
            imu_data.data_ready_timestamp = g_imu_protocol_info.data_ready_timestamp;
            
            imu_data.data_valid = true;
            data_updated = true;
            last_data_time = imu_data.data_ready_timestamp;
        }
        
        // 清除接收帧标志
        BSP_USART2_ClearFrame();
    }
}

/**
 * @brief  获取IMU数据
 * @param  imu_data: 存储IMU数据的结构体指针
 * @retval true: 有新数据, false: 没有新数据
 */
bool BSP_IMU_GetData(IMU_Data_t *data)
{
    if (data == NULL)
    {
        return false;
    }
    
    if (data_updated)
    {
        memcpy(data, &imu_data, sizeof(IMU_Data_t));
        data_updated = false;
        return true;
    }
    
    return false;
}

/**
 * @brief  打印IMU数据
 * @param  None
 * @retval None
 */
void BSP_IMU_PrintData(void)
{
    if (imu_data.data_valid)
    {
        USART1_Printf("=== IMU Data ===\r\n");
        USART1_Printf("Accel: X=%.3f Y=%.3f Z=%.3f m/s²\r\n", 
                     imu_data.accel_x, imu_data.accel_y, imu_data.accel_z);
        USART1_Printf("Gyro:  X=%.3f Y=%.3f Z=%.3f °/s\r\n", 
                     imu_data.gyro_x, imu_data.gyro_y, imu_data.gyro_z);
        USART1_Printf("Euler: Pitch=%.2f Roll=%.2f Yaw=%.2f °\r\n", 
                     imu_data.pitch, imu_data.roll, imu_data.yaw);
        USART1_Printf("Quat:  W=%.3f X=%.3f Y=%.3f Z=%.3f\r\n", 
                     imu_data.quaternion_w, imu_data.quaternion_x, 
                     imu_data.quaternion_y, imu_data.quaternion_z);
        
        if (imu_data.latitude != 0.0 || imu_data.longitude != 0.0)
        {
            USART1_Printf("GPS:   Lat=%.7f Lon=%.7f Alt=%.2f m\r\n", 
                         imu_data.latitude, imu_data.longitude, imu_data.altitude);
            USART1_Printf("Vel:   N=%.3f E=%.3f D=%.3f m/s\r\n", 
                         imu_data.vel_north, imu_data.vel_east, imu_data.vel_down);
        }
        
        USART1_Printf("Timestamp: %u us\r\n", imu_data.data_ready_timestamp);
        USART1_Printf("================\r\n");
    }
    else
    {
        USART1_Printf("IMU: No valid data\r\n");
    }
}

/**
 * @brief  检查IMU连接状态
 * @param  None
 * @retval true: 连接正常, false: 连接异常
 */
bool BSP_IMU_IsConnected(void)
{
    // 简单的连接检查：如果数据有效且最近有更新，则认为连接正常
    return imu_data.data_valid;
}