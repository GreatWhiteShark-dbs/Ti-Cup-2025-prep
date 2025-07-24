#ifndef __BSP_IMU_H
#define __BSP_IMU_H

#include "stm32f10x.h"
#include "bsp_imu_protocol.h"
#include <stdbool.h>

// IMU数据结构体
typedef struct
{
    // 加速度数据 (m/s²)
    float accel_x;
    float accel_y;
    float accel_z;
    
    // 角速度数据 (°/s)
    float gyro_x;
    float gyro_y;
    float gyro_z;
    
    // 磁力计数据 (归一化值)
    float mag_x;
    float mag_y;
    float mag_z;
    
    // 欧拉角 (°)
    float pitch;
    float roll;
    float yaw;
    
    // 四元数
    float quaternion_w;
    float quaternion_x;
    float quaternion_y;
    float quaternion_z;
    
    // 位置信息
    double latitude;    // 纬度 (°)
    double longitude;   // 经度 (°)
    float altitude;     // 海拔 (m)
    
    // 速度信息 (m/s)
    float vel_north;
    float vel_east;
    float vel_down;
    
    // 时间戳 (us)
    uint32_t sample_timestamp;
    uint32_t data_ready_timestamp;
    
    // 数据有效标志
    bool data_valid;
} IMU_Data_t;

// IMU初始化函数
void BSP_IMU_Init(void);

// IMU数据处理函数
void BSP_IMU_ProcessData(void);

// IMU数据获取函数
bool BSP_IMU_GetData(IMU_Data_t *imu_data);

// IMU数据打印函数
void BSP_IMU_PrintData(void);

// IMU连接状态检查
bool BSP_IMU_IsConnected(void);

#endif