#ifndef __BSP_SW_I2C_H
#define __BSP_SW_I2C_H

#include "stm32f10x.h"
#include "sw_i2c.h"

// 软件IIC引脚定义
#define SW_I2C_SCL_PORT    GPIOB
#define SW_I2C_SCL_PIN     GPIO_Pin_6
#define SW_I2C_SDA_PORT    GPIOB
#define SW_I2C_SDA_PIN     GPIO_Pin_7

/**
 * @brief 软件IIC初始化
 * @param 无
 * @retval 无
 */
void BSP_SW_I2C_Init(void);

/**
 * @brief 获取软件IIC接口
 * @param 无
 * @retval sw_i2c_interface_t* IIC接口指针
 */
sw_i2c_interface_t* BSP_SW_I2C_GetInterface(void);

/**
 * @brief IIC总线扫描
 * @param scan_addr 扫描到的地址数组
 * @retval uint8_t 扫描到的设备数量
 */
uint8_t BSP_SW_I2C_Scan(uint8_t *scan_addr);

#endif /* __BSP_SW_I2C_H */