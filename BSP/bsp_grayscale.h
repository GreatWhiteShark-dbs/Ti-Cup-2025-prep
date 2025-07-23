#ifndef __BSP_GRAYSCALE_H
#define __BSP_GRAYSCALE_H

#include "stm32f10x.h"
#include "gw_grayscale_sensor.h"

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