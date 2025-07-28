#ifndef __BSP_GIMBAL_H
#define __BSP_GIMBAL_H

#include "stm32f10x.h"
#include "bsp_emm_v5.h"

/* 云台电机地址定义 */
#define PITCH_MOTOR_ADDR    5    // 俯仰电机地址
#define YAW_MOTOR_ADDR      6    // 偏航角电机地址

/* 云台电机细分设置 */
#define GIMBAL_MICROSTEP           256                    // 细分数（可修改：1, 2, 4, 8, 16, 32等）
#define GIMBAL_PULSES_PER_REV      (200 * GIMBAL_MICROSTEP)  // 每圈脉冲数 = 200 * 细分数
#define GIMBAL_ANGLE_TO_PULSES(angle)  ((uint32_t)((angle) * GIMBAL_PULSES_PER_REV / 360.0f))  // 角度转脉冲数

/* 云台速度和加速度定义 */
#define GIMBAL_POS_SPEED      100   // 位置模式速度（RPM）
#define GIMBAL_POS_ACCEL     200      // 位置模式加速度
#define GIMBAL_VEL_ACCEL     200      // 速度模式加速度

/* 云台状态结构体 */
typedef struct {
    float pitch_angle;    // 俯仰角度
    float yaw_angle;      // 偏航角度
    uint16_t pitch_speed; // 俯仰速度
    uint16_t yaw_speed;   // 偏航速度
} Gimbal_State_t;

/* 轨迹点结构体 */
typedef struct {
    uint16_t x;          // X坐标（0-1000）
    uint16_t y;          // Y坐标（0-1000）
} Trajectory_Point_t;

/* 轨迹同步状态枚举 */
typedef enum {
    TRAJECTORY_SYNC_STOPPED = 0,    // 停止同步
    TRAJECTORY_SYNC_RUNNING = 1     // 正在同步
} Trajectory_Sync_State_t;

/* 函数声明 */
void Gimbal_Init(void);                              // 云台初始化
void Gimbal_SetPitchAngle(float angle);             // 设置俯仰角度
void Gimbal_SetYawAngle(float angle);               // 设置偏航角度
void Gimbal_SetPitchSpeed(uint16_t speed);          // 设置俯仰速度
void Gimbal_SetYawSpeed(uint16_t speed);            // 设置偏航速度
void Gimbal_Stop(void);                             // 云台停止
Gimbal_State_t Gimbal_GetState(void);               // 获取云台状态

#endif
