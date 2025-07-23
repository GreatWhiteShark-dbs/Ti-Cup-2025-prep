#include "bsp_sw_i2c.h"
#include "delay.h"
#include <stddef.h>  

// 静态变量
static sw_i2c_interface_t sw_i2c_interface;

// 静态函数声明
static void sda_out(uint8_t bit, void *user_data);
static uint8_t sda_in(void *user_data);
static void scl_out(uint8_t bit, void *user_data);

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