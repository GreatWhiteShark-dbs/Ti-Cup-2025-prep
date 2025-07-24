#ifndef __BSP_SW_I2C_H
#define __BSP_SW_I2C_H

#include "stm32f10x.h"
#include <stdint.h>

// 软件IIC引脚定义
#define SW_I2C_SCL_PORT    GPIOB
#define SW_I2C_SCL_PIN     GPIO_Pin_6
#define SW_I2C_SDA_PORT    GPIOB
#define SW_I2C_SDA_PIN     GPIO_Pin_7

// 从 sw_i2c.h 整合的内容
typedef struct {
	void (*sda_out)(uint8_t bit, void *user_data);
	uint8_t (*sda_in)(void *user_data);
	void (*scl_out)(uint8_t bit, void *user_data);
	void *user_data;
} sw_i2c_interface_t;

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

// 从 sw_i2c.h 整合的函数声明
/**
 * @brief 从IIC总线上的设备读取多个字节
 * @param i2c_interface
 * @param dev_addr 从设备地址
 * @param[out] data 读取到的字节数组
 * @param data_length 读取大小(字节)
 * @return 0:成功, 1:错误
 */
int8_t sw_i2c_read(sw_i2c_interface_t *i2c_interface, uint8_t dev_addr, uint8_t *data, uint8_t data_length);
int8_t sw_i2c_write(sw_i2c_interface_t *i2c_interface, uint8_t dev_addr, const uint8_t *data, uint8_t data_length);

/**
 * @brief 从IIC总线上的设备读取一个字节
 * @param i2c_interface
 * @param dev_addr 从设备地址
 * @param[out] data 读取到的字节
 * @return 0:成功, 1:错误
 */
int8_t sw_i2c_read_byte(sw_i2c_interface_t *i2c_interface, uint8_t dev_addr, uint8_t *data);
int8_t sw_i2c_write_byte(sw_i2c_interface_t *i2c_interface, uint8_t dev_addr, const uint8_t *data);

/**
 * @brief 读取IIC总线从设备的寄存器数据. 即先写入寄存器地址,无终止位,再连续读取所需数据
 * @param i2c_interface
 * @param dev_addr 从设备地址
 * @param mem_addr 寄存器地址
 * @param[out] data 读取到的字节数组
 * @param data_length 读取大小(字节),不包括寄存器地址本身
 * @return 0:成功, 1:错误
 */
int8_t sw_i2c_mem_read(sw_i2c_interface_t *i2c_interface, uint8_t dev_addr, uint8_t mem_addr, uint8_t *data, uint8_t data_length);

/**
 * @brief 写入IIC总线从设备的寄存器. 即先写入寄存器地址,再连续写入数组中的数据
 * @param i2c_interface
 * @param dev_addr 从设备地址
 * @param mem_addr 寄存器地址
 * @param[out] data 连续写入的数据
 * @param data_length 所写入的字节大小,不包括寄存器地址本身
 * @return 0:成功, 1:错误
 */
int8_t sw_i2c_mem_write(sw_i2c_interface_t *i2c_interface, uint8_t dev_addr, uint8_t mem_addr, const uint8_t *data, uint8_t data_length);

#endif /* __BSP_SW_I2C_H */