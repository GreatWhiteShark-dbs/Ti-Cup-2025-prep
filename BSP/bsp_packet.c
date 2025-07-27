#include "bsp_packet.h"
#include "bsp_usart1.h"
#include <string.h>
#include <math.h>

/* 外部变量声明 */
extern bool trajectory_packet_mode;

/**
  * @brief  识别数据包类型
  * @param  data: 数据缓冲区
  * @param  length: 数据长度
  * @retval 数据包类型
  */
Packet_Type_t Packet_IdentifyType(uint8_t *data, uint8_t length)
{
    if (length < 2) return PACKET_TYPE_COMMAND;
    
    uint16_t start_flag = (data[1] << 8) | data[0];  // 小端序
    
    switch (start_flag) {
        case PACKET_START_FLAG_CAR:
            return PACKET_TYPE_CAR_CONTROL;
        case PACKET_START_FLAG_GIMBAL:
            return PACKET_TYPE_GIMBAL_CONTROL;
        default:
            return PACKET_TYPE_COMMAND;
    }
}

/**
  * @brief  计算校验和
  * @param  data: 数据缓冲区
  * @param  length: 数据长度
  * @retval 校验和
  */
uint8_t Packet_CalculateChecksum(uint8_t *data, uint8_t length)
{
    uint8_t checksum = 0;
    for (uint8_t i = 0; i < length; i++) {
        checksum += data[i];
    }
    return checksum;
}

/**
  * @brief  解析小车控制数据包
  * @param  data: 数据缓冲区
  * @param  length: 数据长度
  * @param  packet: 输出的控制包
  * @retval 解析结果
  */
Packet_Parse_Result_t Packet_ParseCarControl(uint8_t *data, uint8_t length, Car_Control_Packet_t *packet)
{
    if (length != sizeof(Car_Control_Packet_t)) {
        return PACKET_PARSE_ERROR_LENGTH;
    }
    
    // 检查起始标志
    uint16_t start_flag = data[0] | (data[1] << 8);
    if (start_flag != PACKET_START_FLAG_CAR) {
        return PACKET_PARSE_ERROR_START_FLAG;
    }
    
    // 解析数据
    packet->start_flag = start_flag;
    packet->command = data[2];
    packet->param1 = data[3] | (data[4] << 8);
    packet->param2 = data[5] | (data[6] << 8);
    packet->checksum = data[7];
    
    // 验证校验和
    uint8_t calculated_checksum = Packet_CalculateChecksum(data + 2, 5);
    if (calculated_checksum != packet->checksum) {
        return PACKET_PARSE_ERROR_CHECKSUM;
    }
    
    return PACKET_PARSE_OK;
}

/**
  * @brief  处理小车控制数据包
  * @param  packet: 小车控制包
  * @retval 无
  */
void Packet_ProcessCarControl(Car_Control_Packet_t packet)
{
    switch (packet.command) {
        case CAR_CMD_STOP:
            Car_Stop();
            USART1_SendString("小车停止\r\n");
            break;
            
        case CAR_CMD_FORWARD:
            Car_SetSpeed_Forward(packet.param1);
            USART1_Printf("小车前进 (%d RPM)\r\n", packet.param1);
            break;
            
        case CAR_CMD_BACKWARD:
            Car_SetSpeed_Backward(packet.param1);
            USART1_Printf("小车后退 (%d RPM)\r\n", packet.param1);
            break;
            
        case CAR_CMD_TURN_LEFT:
            Car_SetSpeed_TurnLeft(packet.param1);
            USART1_Printf("小车左转 (%d RPM)\r\n", packet.param1);
            break;
            
        case CAR_CMD_TURN_RIGHT:
            Car_SetSpeed_TurnRight(packet.param1);
            USART1_Printf("小车右转 (%d RPM)\r\n", packet.param1);
            break;
            
        case CAR_CMD_ROTATE_LEFT:
            Car_SetSpeed_RotateLeft(packet.param1);
            USART1_Printf("小车原地左旋 (%d RPM)\r\n", packet.param1);
            break;
            
        case CAR_CMD_ROTATE_RIGHT:
            Car_SetSpeed_RotateRight(packet.param1);
            USART1_Printf("小车原地右旋 (%d RPM)\r\n", packet.param1);
            break;
            
        case CAR_CMD_MOVE_DISTANCE:
            Car_SetPos_MoveDistance((float)packet.param1);
            USART1_Printf("小车移动 %dmm\r\n", packet.param1);
            break;
            
        case CAR_CMD_ROTATE_ANGLE:
            Car_SetPos_RotateAngle((float)packet.param1);
            USART1_Printf("小车旋转 %d度\r\n", packet.param1);
            break;
            
        default:
            USART1_Printf("未知小车命令: 0x%02X\r\n", packet.command);
            break;
    }
}

/**
  * @brief  解析云台控制数据包
  * @param  data: 数据缓冲区
  * @param  length: 数据长度
  * @param  packet: 输出的控制包
  * @retval 解析结果
  */
Packet_Parse_Result_t Packet_ParseGimbalControl(uint8_t *data, uint8_t length, Gimbal_Control_Packet_t *packet)
{
    if (length != sizeof(Gimbal_Control_Packet_t)) {
        return PACKET_PARSE_ERROR_LENGTH;
    }
    
    // 检查起始标志
    uint16_t start_flag = data[0] | (data[1] << 8);
    if (start_flag != PACKET_START_FLAG_GIMBAL) {
        return PACKET_PARSE_ERROR_START_FLAG;
    }
    
    // 解析数据
    packet->start_flag = start_flag;
    packet->command = data[2];
    memcpy(&packet->pitch_angle, &data[3], sizeof(float));
    memcpy(&packet->yaw_angle, &data[7], sizeof(float));
    packet->checksum = data[11];
    
    // 验证校验和
    uint8_t calculated_checksum = Packet_CalculateChecksum(data + 2, 9);
    if (calculated_checksum != packet->checksum) {
        return PACKET_PARSE_ERROR_CHECKSUM;
    }
    
    return PACKET_PARSE_OK;
}

/**
  * @brief  处理云台控制数据包
  * @param  packet: 云台控制包
  * @retval 无
  */
void Packet_ProcessGimbalControl(Gimbal_Control_Packet_t packet)
{
    switch (packet.command) {
        case GIMBAL_CMD_SET_ANGLE:
            Gimbal_SetPitchAngle(packet.pitch_angle);
            delay_ms(5);
            Gimbal_SetYawAngle(packet.yaw_angle);
            USART1_Printf("云台设置角度: 俯仰%.1f°, 偏航%.1f°\r\n", 
                         packet.pitch_angle, packet.yaw_angle);
            break;
            
        case GIMBAL_CMD_STOP:
            Gimbal_Stop();
            USART1_SendString("云台停止\r\n");
            break;
            
        case GIMBAL_CMD_RESET:
            Gimbal_SetPitchAngle(0.0f);
            delay_ms(5);
            Gimbal_SetYawAngle(0.0f);
            USART1_SendString("云台归零\r\n");
            break;
            
        default:
            USART1_Printf("未知云台命令: 0x%02X\r\n", packet.command);
            break;
    }
}

/**
  * @brief  通用数据包处理入口
  * @param  data: 数据缓冲区
  * @param  length: 数据长度
  * @retval 无
  */
void Packet_ProcessReceived(uint8_t *data, uint8_t length)
{
    Packet_Type_t packet_type = Packet_IdentifyType(data, length);
    
    switch (packet_type) {
        case PACKET_TYPE_CAR_CONTROL:
        {
            Car_Control_Packet_t packet;
            if (Packet_ParseCarControl(data, length, &packet) == PACKET_PARSE_OK) {
                Packet_ProcessCarControl(packet);
            } else {
                USART1_SendString("小车控制数据包解析错误\r\n");
            }
            break;
        }
        
        case PACKET_TYPE_GIMBAL_CONTROL:
        {
            Gimbal_Control_Packet_t packet;
            if (Packet_ParseGimbalControl(data, length, &packet) == PACKET_PARSE_OK) {
                Packet_ProcessGimbalControl(packet);
            } else {
                USART1_SendString("云台控制数据包解析错误\r\n");
            }
            break;
        }
        
        case PACKET_TYPE_COMMAND:
        default:
            // 单字符命令，由main.c中的原有逻辑处理
            break;
    }
}