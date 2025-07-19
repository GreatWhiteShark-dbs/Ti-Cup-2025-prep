#include "bsp_car.h"
#include "delay.h"
#include <math.h>

// 定义数学常数（如果编译器没有提供）
#ifndef M_PI
#define M_PI 3.14159265358979323846f
#endif

/* 小车状态变量 */
static Car_State_t car_state = {0};

/**
  * @brief  小车初始化
  * @param  无
  * @retval 无
  */
void Car_Init(void)
{
    // 使能四个电机
    Emm_V5_En_Control(FRONT_LEFT_MOTOR_ADDR, true, false);
    Emm_V5_En_Control(FRONT_RIGHT_MOTOR_ADDR, true, false);
    Emm_V5_En_Control(REAR_LEFT_MOTOR_ADDR, true, false);
    Emm_V5_En_Control(REAR_RIGHT_MOTOR_ADDR, true, false);
    
    // 等待电机初始化完成
    delay_ms(200);
    
    // 初始化状态变量
    car_state.x_position = 0.0f;
    car_state.y_position = 0.0f;
    car_state.heading = 0.0f;
    car_state.front_left_speed = 0;
    car_state.front_right_speed = 0;
    car_state.rear_left_speed = 0;
    car_state.rear_right_speed = 0;
    car_state.current_mode = CAR_MODE_STOP;
}

/**
  * @brief  小车停止
  * @param  无
  * @retval 无
  */
void Car_Stop(void)
{
    Emm_V5_Stop_Now(FRONT_LEFT_MOTOR_ADDR, false);
    delay_ms(5);
    Emm_V5_Stop_Now(FRONT_RIGHT_MOTOR_ADDR, false);
    delay_ms(5);
    Emm_V5_Stop_Now(REAR_LEFT_MOTOR_ADDR, false);
    delay_ms(5);
    Emm_V5_Stop_Now(REAR_RIGHT_MOTOR_ADDR, false);
    
    car_state.front_left_speed = 0;
    car_state.front_right_speed = 0;
    car_state.rear_left_speed = 0;
    car_state.rear_right_speed = 0;
    car_state.current_mode = CAR_MODE_STOP;
}

/**
  * @brief  前进（速度模式）
  * @param  speed: 速度（RPM）
  * @retval 无
  */
void Car_SetSpeed_Forward(uint16_t speed)
{
    Emm_V5_Vel_Control(FRONT_LEFT_MOTOR_ADDR, FRONT_LEFT_DIR_INVERT, speed, CAR_VEL_ACCEL, false);
    delay_ms(5);
    Emm_V5_Vel_Control(FRONT_RIGHT_MOTOR_ADDR, FRONT_RIGHT_DIR_INVERT, speed, CAR_VEL_ACCEL, false);
    delay_ms(5);
    Emm_V5_Vel_Control(REAR_LEFT_MOTOR_ADDR, REAR_LEFT_DIR_INVERT, speed, CAR_VEL_ACCEL, false);
    delay_ms(5);
    Emm_V5_Vel_Control(REAR_RIGHT_MOTOR_ADDR, REAR_RIGHT_DIR_INVERT, speed, CAR_VEL_ACCEL, false);
    
    car_state.front_left_speed = speed;
    car_state.front_right_speed = speed;
    car_state.rear_left_speed = speed;
    car_state.rear_right_speed = speed;
    car_state.current_mode = CAR_MODE_FORWARD;
}

/**
  * @brief  后退（速度模式）
  * @param  speed: 速度（RPM）
  * @retval 无
  */
void Car_SetSpeed_Backward(uint16_t speed)
{
    Emm_V5_Vel_Control(FRONT_LEFT_MOTOR_ADDR, !FRONT_LEFT_DIR_INVERT, speed, CAR_VEL_ACCEL, false);
    delay_ms(5);
    Emm_V5_Vel_Control(FRONT_RIGHT_MOTOR_ADDR, !FRONT_RIGHT_DIR_INVERT, speed, CAR_VEL_ACCEL, false);
    delay_ms(5);
    Emm_V5_Vel_Control(REAR_LEFT_MOTOR_ADDR, !REAR_LEFT_DIR_INVERT, speed, CAR_VEL_ACCEL, false);
    delay_ms(5);
    Emm_V5_Vel_Control(REAR_RIGHT_MOTOR_ADDR, !REAR_RIGHT_DIR_INVERT, speed, CAR_VEL_ACCEL, false);
    
    car_state.front_left_speed = speed;
    car_state.front_right_speed = speed;
    car_state.rear_left_speed = speed;
    car_state.rear_right_speed = speed;
    car_state.current_mode = CAR_MODE_BACKWARD;
}

/**
  * @brief  左转（速度模式）
  * @param  speed: 速度（RPM）
  * @retval 无
  */
void Car_SetSpeed_TurnLeft(uint16_t speed)
{
    uint16_t inner_speed = speed / 2;  // 内侧轮速度减半
    
    Emm_V5_Vel_Control(FRONT_LEFT_MOTOR_ADDR, FRONT_LEFT_DIR_INVERT, inner_speed, CAR_VEL_ACCEL, false);
    delay_ms(5);
    Emm_V5_Vel_Control(FRONT_RIGHT_MOTOR_ADDR, FRONT_RIGHT_DIR_INVERT, speed, CAR_VEL_ACCEL, false);
    delay_ms(5);
    Emm_V5_Vel_Control(REAR_LEFT_MOTOR_ADDR, REAR_LEFT_DIR_INVERT, inner_speed, CAR_VEL_ACCEL, false);
    delay_ms(5);
    Emm_V5_Vel_Control(REAR_RIGHT_MOTOR_ADDR, REAR_RIGHT_DIR_INVERT, speed, CAR_VEL_ACCEL, false);
    
    car_state.front_left_speed = inner_speed;
    car_state.front_right_speed = speed;
    car_state.rear_left_speed = inner_speed;
    car_state.rear_right_speed = speed;
    car_state.current_mode = CAR_MODE_TURN_LEFT;
}

/**
  * @brief  右转（速度模式）
  * @param  speed: 速度（RPM）
  * @retval 无
  */
void Car_SetSpeed_TurnRight(uint16_t speed)
{
    uint16_t inner_speed = speed / 2;  // 内侧轮速度减半
    
    Emm_V5_Vel_Control(FRONT_LEFT_MOTOR_ADDR, FRONT_LEFT_DIR_INVERT, speed, CAR_VEL_ACCEL, false);
    delay_ms(5);
    Emm_V5_Vel_Control(FRONT_RIGHT_MOTOR_ADDR, FRONT_RIGHT_DIR_INVERT, inner_speed, CAR_VEL_ACCEL, false);
    delay_ms(5);
    Emm_V5_Vel_Control(REAR_LEFT_MOTOR_ADDR, REAR_LEFT_DIR_INVERT, speed, CAR_VEL_ACCEL, false);
    delay_ms(5);
    Emm_V5_Vel_Control(REAR_RIGHT_MOTOR_ADDR, REAR_RIGHT_DIR_INVERT, inner_speed, CAR_VEL_ACCEL, false);
    
    car_state.front_left_speed = speed;
    car_state.front_right_speed = inner_speed;
    car_state.rear_left_speed = speed;
    car_state.rear_right_speed = inner_speed;
    car_state.current_mode = CAR_MODE_TURN_RIGHT;
}

/**
  * @brief  原地左旋（速度模式）
  * @param  speed: 速度（RPM）
  * @retval 无
  */
void Car_SetSpeed_RotateLeft(uint16_t speed)
{
    Emm_V5_Vel_Control(FRONT_LEFT_MOTOR_ADDR, !FRONT_LEFT_DIR_INVERT, speed, CAR_VEL_ACCEL, false);
    delay_ms(5);
    Emm_V5_Vel_Control(FRONT_RIGHT_MOTOR_ADDR, FRONT_RIGHT_DIR_INVERT, speed, CAR_VEL_ACCEL, false);
    delay_ms(5);
    Emm_V5_Vel_Control(REAR_LEFT_MOTOR_ADDR, !REAR_LEFT_DIR_INVERT, speed, CAR_VEL_ACCEL, false);
    delay_ms(5);
    Emm_V5_Vel_Control(REAR_RIGHT_MOTOR_ADDR, REAR_RIGHT_DIR_INVERT, speed, CAR_VEL_ACCEL, false);
    
    car_state.front_left_speed = speed;
    car_state.front_right_speed = speed;
    car_state.rear_left_speed = speed;
    car_state.rear_right_speed = speed;
    car_state.current_mode = CAR_MODE_ROTATE_LEFT;
}

/**
  * @brief  原地右旋（速度模式）
  * @param  speed: 速度（RPM）
  * @retval 无
  */
void Car_SetSpeed_RotateRight(uint16_t speed)
{
    Emm_V5_Vel_Control(FRONT_LEFT_MOTOR_ADDR, FRONT_LEFT_DIR_INVERT, speed, CAR_VEL_ACCEL, false);
    delay_ms(5);
    Emm_V5_Vel_Control(FRONT_RIGHT_MOTOR_ADDR, !FRONT_RIGHT_DIR_INVERT, speed, CAR_VEL_ACCEL, false);
    delay_ms(5);
    Emm_V5_Vel_Control(REAR_LEFT_MOTOR_ADDR, REAR_LEFT_DIR_INVERT, speed, CAR_VEL_ACCEL, false);
    delay_ms(5);
    Emm_V5_Vel_Control(REAR_RIGHT_MOTOR_ADDR, !REAR_RIGHT_DIR_INVERT, speed, CAR_VEL_ACCEL, false);
    
    car_state.front_left_speed = speed;
    car_state.front_right_speed = speed;
    car_state.rear_left_speed = speed;
    car_state.rear_right_speed = speed;
    car_state.current_mode = CAR_MODE_ROTATE_RIGHT;
}

/**
  * @brief  手动设置各轮速度（速度模式）
  * @param  fl: 左前轮速度（RPM）
  * @param  fr: 右前轮速度（RPM）
  * @param  rl: 左后轮速度（RPM）
  * @param  rr: 右后轮速度（RPM）
  * @retval 无
  */
void Car_SetSpeed_Manual(uint16_t fl, uint16_t fr, uint16_t rl, uint16_t rr)
{
    Emm_V5_Vel_Control(FRONT_LEFT_MOTOR_ADDR, FRONT_LEFT_DIR_INVERT, fl, CAR_VEL_ACCEL, false);
    delay_ms(5);
    Emm_V5_Vel_Control(FRONT_RIGHT_MOTOR_ADDR, FRONT_RIGHT_DIR_INVERT, fr, CAR_VEL_ACCEL, false);
    delay_ms(5);
    Emm_V5_Vel_Control(REAR_LEFT_MOTOR_ADDR, REAR_LEFT_DIR_INVERT, rl, CAR_VEL_ACCEL, false);
    delay_ms(5);
    Emm_V5_Vel_Control(REAR_RIGHT_MOTOR_ADDR, REAR_RIGHT_DIR_INVERT, rr, CAR_VEL_ACCEL, false);
    
    car_state.front_left_speed = fl;
    car_state.front_right_speed = fr;
    car_state.rear_left_speed = rl;
    car_state.rear_right_speed = rr;
    car_state.current_mode = CAR_MODE_MANUAL;
}

/**
  * @brief  直线移动指定距离（位置模式）
  * @param  distance: 移动距离（mm，正数前进，负数后退）
  * @retval 无
  */
void Car_SetPos_MoveDistance(float distance)
{
    uint32_t pulses = (uint32_t)Car_DistanceToPulses(fabs(distance));
    uint8_t direction = (distance >= 0) ? 0 : 1;
    
    Emm_V5_Pos_Control(FRONT_LEFT_MOTOR_ADDR, direction ^ FRONT_LEFT_DIR_INVERT, CAR_POS_SPEED, CAR_POS_ACCEL, pulses, false, false);
    delay_ms(5);
    Emm_V5_Pos_Control(FRONT_RIGHT_MOTOR_ADDR, direction ^ FRONT_RIGHT_DIR_INVERT, CAR_POS_SPEED, CAR_POS_ACCEL, pulses, false, false);
    delay_ms(5);
    Emm_V5_Pos_Control(REAR_LEFT_MOTOR_ADDR, direction ^ REAR_LEFT_DIR_INVERT, CAR_POS_SPEED, CAR_POS_ACCEL, pulses, false, false);
    delay_ms(5);
    Emm_V5_Pos_Control(REAR_RIGHT_MOTOR_ADDR, direction ^ REAR_RIGHT_DIR_INVERT, CAR_POS_SPEED, CAR_POS_ACCEL, pulses, false, false);
    
    // 更新位置估算
    car_state.x_position += distance * cos(car_state.heading * M_PI / 180.0f);
    car_state.y_position += distance * sin(car_state.heading * M_PI / 180.0f);
}

/**
  * @brief  原地旋转指定角度（位置模式）
  * @param  angle: 旋转角度（度，正数右转，负数左转）
  * @retval 无
  */
void Car_SetPos_RotateAngle(float angle)
{
    float wheel_distance = Car_AngleToWheelDistance(fabs(angle));
    uint32_t pulses = (uint32_t)Car_DistanceToPulses(wheel_distance);
    uint8_t left_dir = (angle >= 0) ? 0 : 1;  // 右转时左轮前进
    uint8_t right_dir = (angle >= 0) ? 1 : 0; // 右转时右轮后退
    
    Emm_V5_Pos_Control(FRONT_LEFT_MOTOR_ADDR, left_dir ^ FRONT_LEFT_DIR_INVERT, CAR_POS_SPEED, CAR_POS_ACCEL, pulses, false, false);
    delay_ms(5);
    Emm_V5_Pos_Control(FRONT_RIGHT_MOTOR_ADDR, right_dir ^ FRONT_RIGHT_DIR_INVERT, CAR_POS_SPEED, CAR_POS_ACCEL, pulses, false, false);
    delay_ms(5);
    Emm_V5_Pos_Control(REAR_LEFT_MOTOR_ADDR, left_dir ^ REAR_LEFT_DIR_INVERT, CAR_POS_SPEED, CAR_POS_ACCEL, pulses, false, false);
    delay_ms(5);
    Emm_V5_Pos_Control(REAR_RIGHT_MOTOR_ADDR, right_dir ^ REAR_RIGHT_DIR_INVERT, CAR_POS_SPEED, CAR_POS_ACCEL, pulses, false, false);
    
    // 更新航向角
    car_state.heading += angle;
    if (car_state.heading >= 360.0f) car_state.heading -= 360.0f;
    if (car_state.heading < 0.0f) car_state.heading += 360.0f;
}

/**
  * @brief  手动设置各轮移动距离（位置模式）
  * @param  fl_dist: 左前轮移动距离（mm）
  * @param  fr_dist: 右前轮移动距离（mm）
  * @param  rl_dist: 左后轮移动距离（mm）
  * @param  rr_dist: 右后轮移动距离（mm）
  * @retval 无
  */
void Car_SetPos_Manual(float fl_dist, float fr_dist, float rl_dist, float rr_dist)
{
    uint32_t fl_pulses = (uint32_t)Car_DistanceToPulses(fabs(fl_dist));
    uint32_t fr_pulses = (uint32_t)Car_DistanceToPulses(fabs(fr_dist));
    uint32_t rl_pulses = (uint32_t)Car_DistanceToPulses(fabs(rl_dist));
    uint32_t rr_pulses = (uint32_t)Car_DistanceToPulses(fabs(rr_dist));
    
    uint8_t fl_dir = (fl_dist >= 0) ? 0 : 1;
    uint8_t fr_dir = (fr_dist >= 0) ? 0 : 1;
    uint8_t rl_dir = (rl_dist >= 0) ? 0 : 1;
    uint8_t rr_dir = (rr_dist >= 0) ? 0 : 1;
    
    Emm_V5_Pos_Control(FRONT_LEFT_MOTOR_ADDR, fl_dir ^ FRONT_LEFT_DIR_INVERT, CAR_POS_SPEED, CAR_POS_ACCEL, fl_pulses, false, false);
    delay_ms(5);
    Emm_V5_Pos_Control(FRONT_RIGHT_MOTOR_ADDR, fr_dir ^ FRONT_RIGHT_DIR_INVERT, CAR_POS_SPEED, CAR_POS_ACCEL, fr_pulses, false, false);
    delay_ms(5);
    Emm_V5_Pos_Control(REAR_LEFT_MOTOR_ADDR, rl_dir ^ REAR_LEFT_DIR_INVERT, CAR_POS_SPEED, CAR_POS_ACCEL, rl_pulses, false, false);
    delay_ms(5);
    Emm_V5_Pos_Control(REAR_RIGHT_MOTOR_ADDR, rr_dir ^ REAR_RIGHT_DIR_INVERT, CAR_POS_SPEED, CAR_POS_ACCEL, rr_pulses, false, false);
    
    car_state.current_mode = CAR_MODE_MANUAL;
}

/**
  * @brief  获取小车状态
  * @param  无
  * @retval 小车状态结构体
  */
Car_State_t Car_GetState(void)
{
    return car_state;
}

/**
  * @brief  重置位置坐标
  * @param  无
  * @retval 无
  */
void Car_ResetPosition(void)
{
    car_state.x_position = 0.0f;
    car_state.y_position = 0.0f;
    car_state.heading = 0.0f;
}

/**
  * @brief  距离转换为脉冲数
  * @param  distance_mm: 距离（mm）
  * @retval 脉冲数
  */
float Car_DistanceToPulses(float distance_mm)
{
    return (distance_mm / WHEEL_CIRCUMFERENCE) * PULSES_PER_REVOLUTION;
}

/**
  * @brief  角度转换为轮子移动距离
  * @param  angle_deg: 角度（度）
  * @retval 轮子移动距离（mm）
  */
float Car_AngleToWheelDistance(float angle_deg)
{
    // 原地旋转时，轮子移动的弧长 = (轮距/2) * 角度弧度
    float radius = WHEELBASE_WIDTH / 2.0f;
    float angle_rad = angle_deg * M_PI / 180.0f;
    return radius * angle_rad;
}
