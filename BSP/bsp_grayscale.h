#ifndef __BSP_GRAYSCALE_H
#define __BSP_GRAYSCALE_H

#include "stm32f10x.h"
#include <stdint.h>

// ========== 从 gw_grayscale_sensor.h 整合的内容 ==========

/* 默认地址 */
#define GW_GRAY_ADDR_DEF 0x4C
#define GW_GRAY_PING 0xAA
#define GW_GRAY_PING_OK 0x66
#define GW_GRAY_PING_RSP GW_GRAY_PING_OK

/* 开启开关数据模式 */
#define GW_GRAY_DIGITAL_MODE 0xDD

/* 开启连续读取模拟数据模式 */
#define GW_GRAY_ANALOG_BASE_ 0xB0
#define GW_GRAY_ANALOG_MODE  (GW_GRAY_ANALOG_BASE_ + 0)

/* 传感器归一化寄存器(v3.6及之后的固件) */
#define GW_GRAY_ANALOG_NORMALIZE 0xCF

/* 循环读取单个探头模拟数据 n从1开始到8 */
#define GW_GRAY_ANALOG(n) (GW_GRAY_ANALOG_BASE_ + (n))

/* 黑色滞回比较参数操作 */
#define GW_GRAY_CALIBRATION_BLACK 0xD0
/* 白色滞回比较参数操作 */
#define GW_GRAY_CALIBRATION_WHITE 0xD1

// 设置所需探头的模拟信号(CE: channel enable)
#define GW_GRAY_ANALOG_CHANNEL_ENABLE 0xCE
#define GW_GRAY_ANALOG_CH_EN_1 (0x1 << 0)
#define GW_GRAY_ANALOG_CH_EN_2 (0x1 << 1)
#define GW_GRAY_ANALOG_CH_EN_3 (0x1 << 2)
#define GW_GRAY_ANALOG_CH_EN_4 (0x1 << 3)
#define GW_GRAY_ANALOG_CH_EN_5 (0x1 << 4)
#define GW_GRAY_ANALOG_CH_EN_6 (0x1 << 5)
#define GW_GRAY_ANALOG_CH_EN_7 (0x1 << 6)
#define GW_GRAY_ANALOG_CH_EN_8 (0x1 << 7)
#define GW_GRAY_ANALOG_CH_EN_ALL (0xFF)

/* 读取错误信息 */
#define GW_GRAY_ERROR 0xDE

/* 设备软件重启 */
#define GW_GRAY_REBOOT 0xC0

/* 读取固件版本号 */
#define GW_GRAY_FIRMWARE 0xC1

/**
 * @brief 从I2C得到的8位的数字信号的数据 读取第n位的数据
 * @param sensor_value_8 数字IO的数据
 * @param n 第1位从1开始, n=1 是传感器的第一个探头数据, n=8是最后一个
 * @return
 */
#define GET_NTH_BIT(sensor_value, nth_bit) (((sensor_value) >> ((nth_bit)-1)) & 0x01)

/**
 * @brief 从一个变量分离出所有的bit
 */
#define SEP_ALL_BIT8(sensor_value, val1, val2, val3, val4, val5, val6, val7, val8) \
do {                                                                              \
val1 = GET_NTH_BIT(sensor_value, 1);                                              \
val2 = GET_NTH_BIT(sensor_value, 2);                                              \
val3 = GET_NTH_BIT(sensor_value, 3);                                              \
val4 = GET_NTH_BIT(sensor_value, 4);                                              \
val5 = GET_NTH_BIT(sensor_value, 5);                                              \
val6 = GET_NTH_BIT(sensor_value, 6);                                              \
val7 = GET_NTH_BIT(sensor_value, 7);                                              \
val8 = GET_NTH_BIT(sensor_value, 8);                                              \
} while(0)

/* 设置设备I2C地址 */
#define GW_GRAY_CHANGE_ADDR 0xAD

/* 广播重置地址所需要发的数据 */
#define GW_GRAY_BROADCAST_RESET "\xB8\xD0\xCE\xAA\xBF\xC6\xBC\xBC"

// ========== 原有的BSP接口定义 ==========

// 八路寻迹传感器数据结构
typedef struct {
    uint8_t digital_data;        // 数字量数据 (8位)
    uint8_t analog_data[8];      // 模拟量数据 (8路)
    uint8_t normalized_data[8];  // 归一化数据 (8路)
    uint8_t is_connected;        // 连接状态
} Grayscale_Data_t;

/**
 * @brief 八路寻迹传感器初始化
 * @param 无
 * @retval int 0=成功, -1=失败
 */
int BSP_Grayscale_Init(void);

/**
 * @brief 读取八路寻迹传感器所有数据
 * @param data 数据结构指针
 * @retval int 0=成功, -1=失败
 */
int BSP_Grayscale_ReadAll(Grayscale_Data_t *data);

/**
 * @brief 只读取数字量数据
 * @param 无
 * @retval uint8_t 数字量数据
 */
uint8_t BSP_Grayscale_ReadDigital(void);

/**
 * @brief 只读取模拟量数据
 * @param analog_data 模拟量数据数组(8个元素)
 * @retval int 0=成功, -1=失败
 */
int BSP_Grayscale_ReadAnalog(uint8_t *analog_data);

/**
 * @brief 只读取归一化数据
 * @param normalized_data 归一化数据数组(8个元素)
 * @retval int 0=成功, -1=失败
 */
int BSP_Grayscale_ReadNormalized(uint8_t *normalized_data);

/**
 * @brief 获取指定通道的数字量状态
 * @param channel 通道号 (1-8)
 * @retval uint8_t 0=白色/无线, 1=黑色/有线
 */
uint8_t BSP_Grayscale_GetChannelDigital(uint8_t channel);

/**
 * @brief 检查传感器连接状态
 * @param 无
 * @retval uint8_t 1=已连接, 0=未连接
 */
uint8_t BSP_Grayscale_IsConnected(void);

/**
 * @brief 打印传感器数据到串口
 * @param data 数据结构指针
 * @retval 无
 */
void BSP_Grayscale_PrintData(Grayscale_Data_t *data);

#endif /* __BSP_GRAYSCALE_H */