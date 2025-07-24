#include "bsp_sw_i2c.h"
#include "delay.h"
#include <stddef.h>  

// 从 sw_i2c.c 整合的宏定义
#define ACK 0x0 // acknowledge (SDA LOW)
#define NACK 0x1 // not acknowledge (SDA HIGH)

#define LOW 0x0
#define HIGH 0x1

#define I2C_READ 0x1
#define I2C_WRITE 0x0

// 静态变量
static sw_i2c_interface_t sw_i2c_interface;

// 静态函数声明
static void sda_out(uint8_t bit, void *user_data);
static uint8_t sda_in(void *user_data);
static void scl_out(uint8_t bit, void *user_data);

// 从 sw_i2c.c 整合的静态函数声明
static void sw_i2c_hal_start(sw_i2c_interface_t *i2c_interface);
static void sw_i2c_hal_stop(sw_i2c_interface_t *i2c_interface);
static void sw_i2c_hal_write_bit(sw_i2c_interface_t *i2c_interface, uint8_t bit);
static uint8_t sw_i2c_hal_read_bit(sw_i2c_interface_t *i2c_interface);
static uint8_t sw_i2c_hal_write_byte(sw_i2c_interface_t *i2c_interface, uint8_t bit);
static uint8_t sw_i2c_hal_read_byte(sw_i2c_interface_t *i2c_interface, uint8_t ack);

/**
 * @brief 软件IIC初始化
 * @param 无
 * @retval 无
 */
void BSP_SW_I2C_Init(void)
{
    // 使能GPIOB时钟
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);
    
    // 配置GPIO
    GPIO_InitTypeDef GPIO_InitStructure;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_OD;  // 开漏输出
    GPIO_InitStructure.GPIO_Pin = SW_I2C_SCL_PIN | SW_I2C_SDA_PIN;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(SW_I2C_SCL_PORT, &GPIO_InitStructure);
    
    // 设置初始电平为高
    GPIO_SetBits(SW_I2C_SCL_PORT, SW_I2C_SCL_PIN | SW_I2C_SDA_PIN);
    
    // 配置软件IIC接口
    sw_i2c_interface.sda_in = sda_in;
    sw_i2c_interface.scl_out = scl_out;
    sw_i2c_interface.sda_out = sda_out;
    sw_i2c_interface.user_data = NULL;
}

/**
 * @brief 获取软件IIC接口
 * @param 无
 * @retval sw_i2c_interface_t* IIC接口指针
 */
sw_i2c_interface_t* BSP_SW_I2C_GetInterface(void)
{
    return &sw_i2c_interface;
}

/**
 * @brief IIC总线扫描
 * @param scan_addr 扫描到的地址数组
 * @retval uint8_t 扫描到的设备数量
 */
uint8_t BSP_SW_I2C_Scan(uint8_t *scan_addr)
{
    int i;
    uint8_t count = 0;
    uint8_t data;
    int8_t ret;
    
    for (i = 1; i < 127; ++i) {
        ret = sw_i2c_read(&sw_i2c_interface, i << 1, &data, 1);
        if (ret == 0) {
            scan_addr[count] = i;
            ++count;
        }
    }
    
    return count;
}

/**
 * @brief SDA输出函数
 * @param bit 输出电平 (0=低电平, 1=高电平)
 * @param user_data 用户数据(未使用)
 * @retval 无
 */
static void sda_out(uint8_t bit, void *user_data)
{
    GPIO_WriteBit(SW_I2C_SDA_PORT, SW_I2C_SDA_PIN, (BitAction)bit);
    delay_us(10);  // IIC软件延迟
}

/**
 * @brief SDA读取函数
 * @param user_data 用户数据(未使用)
 * @retval uint8_t 读取到的电平值
 */
static uint8_t sda_in(void *user_data)
{
    uint8_t bit;
    bit = (uint8_t)GPIO_ReadInputDataBit(SW_I2C_SDA_PORT, SW_I2C_SDA_PIN);
    delay_us(10);  // IIC软件延迟
    return bit;
}

/**
 * @brief SCL时钟输出函数
 * @param bit 输出电平 (0=低电平, 1=高电平)
 * @param user_data 用户数据(未使用)
 * @retval 无
 */
static void scl_out(uint8_t bit, void *user_data)
{
    GPIO_WriteBit(SW_I2C_SCL_PORT, SW_I2C_SCL_PIN, (BitAction)bit);
    delay_us(10);  // IIC软件延迟
}

// ========== 从 sw_i2c.c 整合的函数实现 ==========

int8_t sw_i2c_read(sw_i2c_interface_t *i2c_interface, uint8_t dev_addr, uint8_t *data, uint8_t data_length)
{
	uint8_t i;
	uint8_t ack_bit;

	/* 起始位 */
	sw_i2c_hal_start(i2c_interface);

	/* 地址+读写位 */
	ack_bit = sw_i2c_hal_write_byte(i2c_interface, dev_addr | I2C_READ);
	if (ack_bit) {
		/* 从设备没有回复ACK,直接退出 */
		sw_i2c_hal_stop(i2c_interface);
		return 1;
	}

	/* 连续读取N-1个数据 给ACK */
	for (i = 0; i < data_length - 1; ++i) {
		data[i] = sw_i2c_hal_read_byte(i2c_interface, ACK);
	}

	/* 最后一个数据给 NACK */
	data[i] = sw_i2c_hal_read_byte(i2c_interface, NACK);

	/* 停止位 */
	sw_i2c_hal_stop(i2c_interface);
	return 0;
}

int8_t sw_i2c_write(sw_i2c_interface_t *i2c_interface, uint8_t dev_addr, const uint8_t *data, uint8_t data_length)
{
	uint8_t i;
	uint8_t ack_bit;

	/* 起始位 */
	sw_i2c_hal_start(i2c_interface);

	/* 地址+读写位 */
	ack_bit = sw_i2c_hal_write_byte(i2c_interface, dev_addr | I2C_WRITE);
	if (ack_bit) {
		/* 从设备没有回复ACK,直接退出 */
		sw_i2c_hal_stop(i2c_interface);
		return 1;
	}

	/* 连续写入N个数据, 每次读取1 bit的 ACK */
	for (i = 0; i < data_length; ++i) {
		 ack_bit = sw_i2c_hal_write_byte(i2c_interface, data[i]);
	}

	/* 停止位 */
	sw_i2c_hal_stop(i2c_interface);
	return 0;
}

int8_t sw_i2c_read_byte(sw_i2c_interface_t *i2c_interface, uint8_t dev_addr, uint8_t *data)
{
	return sw_i2c_read(i2c_interface, dev_addr, data, 1);
}

int8_t sw_i2c_write_byte(sw_i2c_interface_t *i2c_interface, uint8_t dev_addr, const uint8_t *data)
{
	return sw_i2c_write(i2c_interface, dev_addr, data, 1);
}

int8_t sw_i2c_mem_read(sw_i2c_interface_t *i2c_interface,
					   uint8_t dev_addr, uint8_t mem_addr, uint8_t *data, uint8_t data_length)
{
	uint8_t ack_bit;
	sw_i2c_hal_start(i2c_interface);
	ack_bit = sw_i2c_hal_write_byte(i2c_interface, dev_addr | I2C_WRITE);
	if (ack_bit) {
		sw_i2c_hal_stop(i2c_interface);
		return 1;
	}
	ack_bit = sw_i2c_hal_write_byte(i2c_interface, mem_addr);

	return sw_i2c_read(i2c_interface, dev_addr, data, data_length);
}

int8_t sw_i2c_mem_write(sw_i2c_interface_t *i2c_interface, uint8_t dev_addr, uint8_t mem_addr, const uint8_t *data,
                        uint8_t data_length)
{
	uint8_t ack_bit;
	sw_i2c_hal_start(i2c_interface);
	ack_bit = sw_i2c_hal_write_byte(i2c_interface, dev_addr | I2C_WRITE);
	if (ack_bit) {
		sw_i2c_hal_stop(i2c_interface);
		return 1;
	}
	ack_bit = sw_i2c_hal_write_byte(i2c_interface, mem_addr);

	return sw_i2c_write(i2c_interface, dev_addr, data, data_length);
}

/***************************
 * 基础操作抽象层
 **************************/

/**
 * @brief send start bit by driving sda and scl LOW
 * @param i2c_interface
 */
static void sw_i2c_hal_start(sw_i2c_interface_t *i2c_interface)
{
	i2c_interface->sda_out(HIGH, i2c_interface->user_data);
	i2c_interface->scl_out(HIGH, i2c_interface->user_data);
	i2c_interface->sda_out(LOW, i2c_interface->user_data);
	i2c_interface->scl_out(LOW, i2c_interface->user_data);
}

/**
 * @brief send stop bit
 * @param i2c_interface
 */
static void sw_i2c_hal_stop(sw_i2c_interface_t *i2c_interface)
{
	i2c_interface->sda_out(LOW, i2c_interface->user_data);
	i2c_interface->scl_out(HIGH, i2c_interface->user_data);
	i2c_interface->sda_out(HIGH, i2c_interface->user_data);
}

/**
 * @brief 输出 sda 电平,然后 scl 输出一个时钟
 * @param i2c_interface
 * @param bit bit level to send, 0:LOW, 1:HIGH
 */
static void sw_i2c_hal_write_bit(sw_i2c_interface_t *i2c_interface, uint8_t bit)
{
	i2c_interface->sda_out(bit, i2c_interface->user_data);
	i2c_interface->scl_out(HIGH, i2c_interface->user_data);
	i2c_interface->scl_out(LOW, i2c_interface->user_data);
}

/**
 * @brief 读 sda 电平值,然后 scl 输出一个时钟
 * @param i2c_interface
 * @return 返回 SDA 电平值, 0:LOW, 1:HIGH
 */
static uint8_t sw_i2c_hal_read_bit(sw_i2c_interface_t *i2c_interface)
{
	uint8_t bit;
	i2c_interface->sda_out(HIGH, i2c_interface->user_data);
	i2c_interface->scl_out(HIGH, i2c_interface->user_data);
	bit = i2c_interface->sda_in(i2c_interface->user_data);
	i2c_interface->scl_out(LOW, i2c_interface->user_data);
	return bit;
}

/**
 * @brief 向IIC输出一个字节
 * @param i2c_interface
 * @param byte
 * @return 从设备反馈的 ACK 电平值
 */
static uint8_t sw_i2c_hal_write_byte(sw_i2c_interface_t *i2c_interface, uint8_t byte)
{
	uint8_t i;
	uint8_t ack;

	for (i = 0; i < 8; ++i) {
		sw_i2c_hal_write_bit(i2c_interface, byte & (0x80 >> i));
	}

	ack = sw_i2c_hal_read_bit(i2c_interface);
	return ack;
}

/**
 * @brief 从IIC总线上读取一个字节
 * @param i2c_interface
 * @param ack 向从设备反馈 ACK 或者 NACK
 * @return 读取到的字节
 */
static uint8_t sw_i2c_hal_read_byte(sw_i2c_interface_t *i2c_interface, uint8_t ack)
{
	uint8_t byte = 0;
	uint8_t i;

	i2c_interface->sda_out(HIGH, i2c_interface->user_data);
	for (i = 0; i < 8; ++i) {
		i2c_interface->scl_out(HIGH, i2c_interface->user_data);
		byte <<= 1;
		byte |= i2c_interface->sda_in(i2c_interface->user_data);
		i2c_interface->scl_out(LOW, i2c_interface->user_data);
	}

	sw_i2c_hal_write_bit(i2c_interface, ack);
	return byte;
}