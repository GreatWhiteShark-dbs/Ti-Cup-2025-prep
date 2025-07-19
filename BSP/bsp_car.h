#ifndef __BSP_CAR_H
#define __BSP_CAR_H

#include "stm32f10x.h"
#include "bsp_emm_v5.h"
#include <math.h>

/* 小车电机地址定义 */
#define FRONT_LEFT_MOTOR_ADDR     1    // 左前轮电机地址
#define FRONT_RIGHT_MOTOR_ADDR    2    // 右前轮电机地址
#define REAR_LEFT_MOTOR_ADDR      3    // 左后轮电机地址
#define REAR_RIGHT_MOTOR_ADDR     4    // 右后轮电机地址

/* 小车物理参数定义 */
#define WHEEL_DIAMETER           85.0f   // 轮子直径（mm）
#define WHEEL_CIRCUMFERENCE      267.04f // 轮子周长（mm）π*d = 3.14159*85
#define WHEELBASE_LENGTH         115.0f  // 轴距（前后轮中心间距，mm）
#define WHEELBASE_WIDTH          181.6f  // 轮距（左右轮中心间距，mm）
#define PULSES_PER_REVOLUTION    3200    // 每圈脉冲数（16细分）

/* 小车速度和加速度定义 */
#define CAR_POS_SPEED            100     // 位置模式速度（RPM）
#define CAR_POS_ACCEL            1       // 位置模式加速度
#define CAR_VEL_ACCEL            2       // 速度模式加速度

/* 电机正反转方向调整宏（0=正转，1=反转） */
#define FRONT_LEFT_DIR_INVERT    0       // 左前轮方向反转标志
#define FRONT_RIGHT_DIR_INVERT   1       // 右前轮方向反转标志
#define REAR_LEFT_DIR_INVERT     0       // 左后轮方向反转标志
#define REAR_RIGHT_DIR_INVERT    1       // 右后轮方向反转标志

/* 小车运动模式枚举 */
typedef enum {
    CAR_MODE_STOP = 0,        // 停止
    CAR_MODE_FORWARD,         // 前进
    CAR_MODE_BACKWARD,        // 后退
    CAR_MODE_TURN_LEFT,       // 左转
    CAR_MODE_TURN_RIGHT,      // 右转
    CAR_MODE_ROTATE_LEFT,     // 原地左旋
    CAR_MODE_ROTATE_RIGHT,    // 原地右旋
    CAR_MODE_MANUAL           // 手动模式
} Car_Mode_t;

/* 小车状态结构体 */
typedef struct {
    float x_position;         // X坐标位置（mm）
    float y_position;         // Y坐标位置（mm）
    float heading;            // 航向角（度）
    uint16_t front_left_speed;   // 左前轮速度（RPM）
    uint16_t front_right_speed;  // 右前轮速度（RPM）
    uint16_t rear_left_speed;    // 左后轮速度（RPM）
    uint16_t rear_right_speed;   // 右后轮速度（RPM）
    Car_Mode_t current_mode;     // 当前运动模式
} Car_State_t;

/* 函数声明 */
void Car_Init(void);                                    // 小车初始化
void Car_Stop(void);                                    // 小车停止

/* 速度模式控制函数 */
void Car_SetSpeed_Forward(uint16_t speed);              // 前进（速度模式）
void Car_SetSpeed_Backward(uint16_t speed);             // 后退（速度模式）
void Car_SetSpeed_TurnLeft(uint16_t speed);             // 左转（速度模式）
void Car_SetSpeed_TurnRight(uint16_t speed);            // 右转（速度模式）
void Car_SetSpeed_RotateLeft(uint16_t speed);           // 原地左旋（速度模式）
void Car_SetSpeed_RotateRight(uint16_t speed);          // 原地右旋（速度模式）
void Car_SetSpeed_Manual(uint16_t fl, uint16_t fr, uint16_t rl, uint16_t rr); // 手动设置各轮速度

/* 位置模式控制函数 */
void Car_SetPos_MoveDistance(float distance);           // 直线移动指定距离（mm）
void Car_SetPos_TurnAngle(float angle);                 // 转向指定角度（度）
void Car_SetPos_RotateAngle(float angle);               // 原地旋转指定角度（度）
void Car_SetPos_MoveTo(float x, float y);               // 移动到指定坐标
void Car_SetPos_Manual(float fl_dist, float fr_dist, float rl_dist, float rr_dist); // 手动设置各轮距离

/* 状态获取函数 */
Car_State_t Car_GetState(void);                         // 获取小车状态
void Car_ResetPosition(void);                           // 重置位置坐标

/* 辅助函数 */
float Car_DistanceToPulses(float distance_mm);          // 距离转换为脉冲数
float Car_AngleToWheelDistance(float angle_deg);        // 角度转换为轮子移动距离

#endif
