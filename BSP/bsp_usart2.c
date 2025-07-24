#include "bsp_usart2.h"
#include <stddef.h>

// USART2接收相关变量
__IO bool usart2_rxFrameFlag = false;
__IO uint8_t usart2_rxCmd[USART2_RX_BUFFER_SIZE] = {0};
__IO uint8_t usart2_rxCount = 0;

/**
 * @brief  USART2发送一个字节
 * @param  data: 要发送的字节
 * @retval None
 */
void BSP_USART2_SendByte(uint8_t data)
{
    USART_SendData(USART2, data);
    while (USART_GetFlagStatus(USART2, USART_FLAG_TXE) == RESET);
}

/**
 * @brief  USART2发送数组
 * @param  array: 要发送的数组
 * @param  length: 数组长度
 * @retval None
 */
void BSP_USART2_SendArray(uint8_t *array, uint16_t length)
{
    uint16_t i;
    for (i = 0; i < length; i++)
    {
        BSP_USART2_SendByte(array[i]);
    }
}

/**
 * @brief  USART2发送字符串
 * @param  string: 要发送的字符串
 * @retval None
 */
void BSP_USART2_SendString(char *string)
{
    uint8_t i;
    for (i = 0; string[i] != '\0'; i++)
    {
        BSP_USART2_SendByte(string[i]);
    }
}

/**
 * @brief  获取USART2接收到的一帧数据
 * @param  data: 存储接收数据的缓冲区
 * @param  length: 接收到的数据长度
 * @retval true: 有新的一帧数据, false: 没有新数据
 */
bool BSP_USART2_GetFrame(uint8_t *data, uint8_t *length)
{
    if (usart2_rxFrameFlag)
    {
        if (data != NULL && length != NULL)
        {
            *length = usart2_rxCount;
            for (uint8_t i = 0; i < usart2_rxCount; i++)
            {
                data[i] = usart2_rxCmd[i];
            }
        }
        return true;
    }
    return false;
}

/**
 * @brief  清除USART2接收帧标志
 * @param  None
 * @retval None
 */
void BSP_USART2_ClearFrame(void)
{
    usart2_rxFrameFlag = false;
    usart2_rxCount = 0;
}

/**
 * @brief  USART2中断处理函数
 * @param  None
 * @retval None
 */
void USART2_IRQHandler(void)
{
    static uint8_t rx_buffer[USART2_RX_BUFFER_SIZE];
    static uint8_t rx_index = 0;

    // 串口接收中断
    if (USART_GetITStatus(USART2, USART_IT_RXNE) != RESET)
    {
        uint8_t received_data = USART_ReceiveData(USART2);
        
        // 存储接收到的数据
        if (rx_index < USART2_RX_BUFFER_SIZE)
        {
            rx_buffer[rx_index++] = received_data;
        }
        
        // 清除串口接收中断
        USART_ClearITPendingBit(USART2, USART_IT_RXNE);
    }

    // 串口空闲中断
    else if (USART_GetITStatus(USART2, USART_IT_IDLE) != RESET)
    {
        // 先读SR再读DR，清除IDLE中断
        USART2->SR; 
        USART2->DR;

        // 复制接收到的数据到全局缓冲区
        if (rx_index > 0)
        {
            usart2_rxCount = rx_index;
            for (uint8_t i = 0; i < rx_index && i < USART2_RX_BUFFER_SIZE; i++)
            {
                usart2_rxCmd[i] = rx_buffer[i];
            }
            usart2_rxFrameFlag = true;
        }
        
        // 重置接收索引
        rx_index = 0;
    }
}