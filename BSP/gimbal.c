#include "gimbal.h"
#include "delay.h"

/* 云台状态变量 */
static Gimbal_State_t gimbal_state = {0};

/**
  * @brief  云台初始化
  * @param  无
  * @retval 无
  */
void Gimbal_Init(void)
{
    // 使能两个电机
    Emm_V5_En_Control(PITCH_MOTOR_ADDR, true, false);
    Emm_V5_En_Control(YAW_MOTOR_ADDR, true, false);
    
    // 等待电机初始化完成
    delay_ms(100);
    
    // 初始化状态变量
    gimbal_state.pitch_angle = 0;
    gimbal_state.yaw_angle = 0;
    gimbal_state.pitch_speed = 0;
    gimbal_state.yaw_speed = 0;
}

/**
  * @brief  设置俯仰角度
  * @param  angle: 目标角度
  * @retval 无
  */
void Gimbal_SetPitchAngle(float angle)
{
    uint32_t pulses;
    // 角度转脉冲数（假设16细分，减速比1:1，则一圈3200脉冲）
    pulses = (uint32_t)(angle * 3200.0f / 360.0f);
    
    Emm_V5_Pos_Control(PITCH_MOTOR_ADDR, 0, GIMBAL_POS_SPEED, GIMBAL_POS_ACCEL, pulses, true, false);
    gimbal_state.pitch_angle = angle;
}

/**
  * @brief  设置偏航角度
  * @param  angle: 目标角度
  * @retval 无
  */
void Gimbal_SetYawAngle(float angle)
{
    uint32_t pulses;
    pulses = (uint32_t)(angle * 3200.0f / 360.0f);
    
    Emm_V5_Pos_Control(YAW_MOTOR_ADDR, 0, GIMBAL_POS_SPEED, GIMBAL_POS_ACCEL, pulses, true, false);
    gimbal_state.yaw_angle = angle;
}

/**
  * @brief  设置俯仰速度
  * @param  speed: 目标速度（RPM）
  * @retval 无
  */
void Gimbal_SetPitchSpeed(uint16_t speed)
{
    Emm_V5_Vel_Control(PITCH_MOTOR_ADDR, 0, speed, GIMBAL_VEL_ACCEL, false);
    gimbal_state.pitch_speed = speed;
}

/**
  * @brief  设置偏航速度
  * @param  speed: 目标速度（RPM）
  * @retval 无
  */
void Gimbal_SetYawSpeed(uint16_t speed)
{
    Emm_V5_Vel_Control(YAW_MOTOR_ADDR, 0, speed, GIMBAL_VEL_ACCEL, false);
    gimbal_state.yaw_speed = speed;
}

/**
  * @brief  云台停止
  * @param  无
  * @retval 无
  */
void Gimbal_Stop(void)
{
    Emm_V5_Stop_Now(PITCH_MOTOR_ADDR, false);
    Emm_V5_Stop_Now(YAW_MOTOR_ADDR, false);
    
    gimbal_state.pitch_speed = 0;
    gimbal_state.yaw_speed = 0;
}

/**
  * @brief  获取云台状态
  * @param  无
  * @retval 云台状态结构体
  */
Gimbal_State_t Gimbal_GetState(void)
{
    return gimbal_state;
}