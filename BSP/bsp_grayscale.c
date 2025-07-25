#include "bsp_grayscale.h"
#include "bsp_sw_i2c.h"
#include "bsp_usart1.h"
#include "delay.h"
#include <stdio.h>
#include <string.h>

// 静态变量
static sw_i2c_interface_t *i2c_interface = NULL;
static uint8_t sensor_connected = 0;
static uint8_t iic_write_buff[10] = {0};

/**
 * @brief 八路寻迹传感器初始化
 * @param 无
 * @retval int 0=成功, -1=失败
 */
int BSP_Grayscale_Init(void)
{
    uint8_t recv_value = 0;
    int retry_count = 0;
    
    // 获取软件IIC接口
    i2c_interface = BSP_SW_I2C_GetInterface();
    if (i2c_interface == NULL) {
        return -1;
    }
    
    // 检测传感器连接状态
    while(recv_value != GW_GRAY_PING_OK && retry_count < 10)
    {
        sw_i2c_mem_read(i2c_interface, GW_GRAY_ADDR_DEF << 1, GW_GRAY_PING, &recv_value, 1);
        if(recv_value != GW_GRAY_PING_OK)
        {
            delay_ms(100);
            retry_count++;
        }
    }
    
    if(recv_value == GW_GRAY_PING_OK)
    {
        sensor_connected = 1;
        return 0;  // 成功
    }
    else
    {
        sensor_connected = 0;
        return -1; // 失败
    }
}

/**
 * @brief 读取八路寻迹传感器所有数据
 * @param data 数据结构指针
 * @retval int 0=成功, -1=失败
 */
int BSP_Grayscale_ReadAll(Grayscale_Data_t *data)
{
    if (data == NULL || i2c_interface == NULL || !sensor_connected) {
        return -1;
    }
    
    // 读取数字量
    if (sw_i2c_mem_read(i2c_interface, GW_GRAY_ADDR_DEF << 1, GW_GRAY_DIGITAL_MODE, &data->digital_data, 1) != 0) {
        return -1;
    }
    
    // 读取模拟量
    if (sw_i2c_mem_read(i2c_interface, GW_GRAY_ADDR_DEF << 1, GW_GRAY_ANALOG_MODE, data->analog_data, 8) != 0) {
        return -1;
    }
    
    // 读取归一化数据
    iic_write_buff[0] = GW_GRAY_ANALOG_NORMALIZE;
    iic_write_buff[1] = 0xff;  // 全通道开启
    if (sw_i2c_write(i2c_interface, GW_GRAY_ADDR_DEF << 1, iic_write_buff, 2) != 0) {
        return -1;
    }
    
    delay_ms(10);  // 等待传感器处理
    
    if (sw_i2c_mem_read(i2c_interface, GW_GRAY_ADDR_DEF << 1, GW_GRAY_ANALOG_MODE, data->normalized_data, 8) != 0) {
        return -1;
    }
    
    // 关闭归一化
    iic_write_buff[0] = GW_GRAY_ANALOG_NORMALIZE;
    iic_write_buff[1] = 0x00;  // 全通道关闭
    sw_i2c_write(i2c_interface, GW_GRAY_ADDR_DEF << 1, iic_write_buff, 2);
    
    data->is_connected = sensor_connected;
    return 0;
}

/**
 * @brief 只读取数字量数据
 * @param 无
 * @retval uint8_t 数字量数据
 */
uint8_t BSP_Grayscale_ReadDigital(void)
{
    uint8_t digital_data = 0;
    
    if (i2c_interface != NULL && sensor_connected) {
        sw_i2c_mem_read(i2c_interface, GW_GRAY_ADDR_DEF << 1, GW_GRAY_DIGITAL_MODE, &digital_data, 1);
    }
    
    return digital_data;
}

/**
 * @brief 只读取模拟量数据
 * @param analog_data 模拟量数据数组(8个元素)
 * @retval int 0=成功, -1=失败
 */
int BSP_Grayscale_ReadAnalog(uint8_t *analog_data)
{
    if (analog_data == NULL || i2c_interface == NULL || !sensor_connected) {
        return -1;
    }
    
    return sw_i2c_mem_read(i2c_interface, GW_GRAY_ADDR_DEF << 1, GW_GRAY_ANALOG_MODE, analog_data, 8);
}

/**
 * @brief 只读取归一化数据
 * @param normalized_data 归一化数据数组(8个元素)
 * @retval int 0=成功, -1=失败
 */
int BSP_Grayscale_ReadNormalized(uint8_t *normalized_data)
{
    if (normalized_data == NULL || i2c_interface == NULL || !sensor_connected) {
        return -1;
    }
    
    // 开启归一化
    iic_write_buff[0] = GW_GRAY_ANALOG_NORMALIZE;
    iic_write_buff[1] = 0xff;  // 全通道开启
    if (sw_i2c_write(i2c_interface, GW_GRAY_ADDR_DEF << 1, iic_write_buff, 2) != 0) {
        return -1;
    }
    
    delay_ms(10);  // 等待传感器处理
    
    int result = sw_i2c_mem_read(i2c_interface, GW_GRAY_ADDR_DEF << 1, GW_GRAY_ANALOG_MODE, normalized_data, 8);
    
    // 关闭归一化
    iic_write_buff[0] = GW_GRAY_ANALOG_NORMALIZE;
    iic_write_buff[1] = 0x00;  // 全通道关闭
    sw_i2c_write(i2c_interface, GW_GRAY_ADDR_DEF << 1, iic_write_buff, 2);
    
    return result;
}

/**
 * @brief 获取指定通道的数字量状态
 * @param channel 通道号 (1-8)
 * @retval uint8_t 0=白色/无线, 1=黑色/有线
 */
uint8_t BSP_Grayscale_GetChannelDigital(uint8_t channel)
{
    if (channel < 1 || channel > 8) {
        return 0;
    }
    
    uint8_t digital_data = BSP_Grayscale_ReadDigital();
    return GET_NTH_BIT(digital_data, channel);
}

/**
 * @brief 检查传感器连接状态
 * @param 无
 * @retval uint8_t 1=已连接, 0=未连接
 */
uint8_t BSP_Grayscale_IsConnected(void)
{
    return sensor_connected;
}

/**
 * @brief 打印传感器数据到串口
 * @param data 数据结构指针
 * @retval 无
 */
void BSP_Grayscale_PrintData(Grayscale_Data_t *data)
{
    if (data == NULL) {
        USART1_SendString("错误: 数据指针为空\r\n");
        return;
    }
    
    if (!data->is_connected) {
        USART1_SendString("错误: 八路寻迹传感器未连接\r\n");
        return;
    }
    
    USART1_SendString("=== 八路寻迹传感器数据 ===\r\n");
    
    // 打印数字量
    USART1_Printf("数字量: %d-%d-%d-%d-%d-%d-%d-%d\r\n",
        (data->digital_data>>0)&0x01, (data->digital_data>>1)&0x01, 
        (data->digital_data>>2)&0x01, (data->digital_data>>3)&0x01,
        (data->digital_data>>4)&0x01, (data->digital_data>>5)&0x01, 
        (data->digital_data>>6)&0x01, (data->digital_data>>7)&0x01);
    
    // 打印模拟量
    USART1_Printf("模拟量: %d-%d-%d-%d-%d-%d-%d-%d\r\n",
        data->analog_data[0], data->analog_data[1], data->analog_data[2], data->analog_data[3],
        data->analog_data[4], data->analog_data[5], data->analog_data[6], data->analog_data[7]);
    
    // 打印归一化数据
    USART1_Printf("归一化: %d-%d-%d-%d-%d-%d-%d-%d\r\n",
        data->normalized_data[0], data->normalized_data[1], data->normalized_data[2], data->normalized_data[3],
        data->normalized_data[4], data->normalized_data[5], data->normalized_data[6], data->normalized_data[7]);
}