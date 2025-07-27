#ifndef __BSP_PACKET_H
#define __BSP_PACKET_H

#include "stm32f10x.h"
#include "bsp_car.h"
#include "bsp_gimbal.h"

/* 数据包类型定义 */
typedef enum {
    PACKET_TYPE_COMMAND = 0x01,      // 单字符命令
    PACKET_TYPE_CAR_CONTROL = 0x03,  // 小车控制数据包
    PACKET_TYPE_GIMBAL_CONTROL = 0x04 // 云台控制数据包
} Packet_Type_t;

/* 数据包起始标志定义 */
#define PACKET_START_FLAG_CAR          0xBB66  // 小车控制数据包起始标志
#define PACKET_START_FLAG_GIMBAL       0xCC77  // 云台控制数据包起始标志

/* 小车控制数据包结构 */
typedef struct {
    uint16_t start_flag;    // 起始标志 0xBB66
    uint8_t command;        // 控制命令
    int16_t param1;         // 参数1 (速度/距离/角度)
    int16_t param2;         // 参数2 (预留)
    uint8_t checksum;       // 校验和
} __attribute__((packed)) Car_Control_Packet_t;

/* 云台控制数据包结构 */
typedef struct {
    uint16_t start_flag;    // 起始标志 0xCC77
    uint8_t command;        // 控制命令
    float pitch_angle;      // 俯仰角度
    float yaw_angle;        // 偏航角度
    uint8_t checksum;       // 校验和
} __attribute__((packed)) Gimbal_Control_Packet_t;

/* 小车控制命令定义 */
typedef enum {
    CAR_CMD_STOP = 0x00,           // 停止
    CAR_CMD_FORWARD = 0x01,        // 前进
    CAR_CMD_BACKWARD = 0x02,       // 后退
    CAR_CMD_TURN_LEFT = 0x03,      // 左转
    CAR_CMD_TURN_RIGHT = 0x04,     // 右转
    CAR_CMD_ROTATE_LEFT = 0x05,    // 原地左旋
    CAR_CMD_ROTATE_RIGHT = 0x06,   // 原地右旋
    CAR_CMD_MOVE_DISTANCE = 0x07,  // 移动指定距离
    CAR_CMD_ROTATE_ANGLE = 0x08    // 旋转指定角度
} Car_Command_t;

/* 云台控制命令定义 */
typedef enum {
    GIMBAL_CMD_SET_ANGLE = 0x01,      // 设置角度
    GIMBAL_CMD_STOP = 0x03,           // 停止
    GIMBAL_CMD_RESET = 0x04           // 归零
} Gimbal_Command_t;

/* 数据包解析结果 */
typedef enum {
    PACKET_PARSE_OK = 0,           // 解析成功
    PACKET_PARSE_ERROR_LENGTH,     // 长度错误
    PACKET_PARSE_ERROR_START_FLAG, // 起始标志错误
    PACKET_PARSE_ERROR_CHECKSUM,   // 校验和错误
    PACKET_PARSE_ERROR_UNKNOWN     // 未知错误
} Packet_Parse_Result_t;

/* 函数声明 */
// 数据包识别
Packet_Type_t Packet_IdentifyType(uint8_t *data, uint8_t length);

// 小车控制数据包处理
Packet_Parse_Result_t Packet_ParseCarControl(uint8_t *data, uint8_t length, Car_Control_Packet_t *packet);
void Packet_ProcessCarControl(Car_Control_Packet_t packet);

// 云台控制数据包处理
Packet_Parse_Result_t Packet_ParseGimbalControl(uint8_t *data, uint8_t length, Gimbal_Control_Packet_t *packet);
void Packet_ProcessGimbalControl(Gimbal_Control_Packet_t packet);

// 通用数据包处理入口
void Packet_ProcessReceived(uint8_t *data, uint8_t length);

// 校验和计算
uint8_t Packet_CalculateChecksum(uint8_t *data, uint8_t length);

// 轨迹坐标转换为偏航角度
float Packet_TrajectoryToYawAngle(uint16_t x, uint16_t y);

#endif