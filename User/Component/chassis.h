#ifndef __CHASSIS_H__
#define __CHASSIS_H__

#include "filter_lpf1.h"
#include "driver_bmi088.h"

typedef struct {
    float Lx; // 轴距的一半（前后轮之间的距离的一半）（单位：米）
    float Ly; // 轮距的一半（左右轮之间的距离的一半）（单位：米）
    float Wheel_R; // 轮半径（单位：米）
} Chassis_Param_t;

typedef struct {
    LPF1_t lpf1_vx; // 低通滤波器实例对象
    LPF1_t lpf1_vy; // 低通滤波器实例对象
    LPF1_t lpf1_vw; // 低通滤波器实例对象
    float vx; // 前后速度
    float vy; // 左右速度
    float vw; // 旋转速度
} Chassis_Velocity_t;

typedef struct {
    float w_lf; // 左前轮
    float w_lr; // 左后轮
    float w_rr; // 右后轮
    float w_rf; // 右前轮
} Motor_Velocity_t;

typedef struct {
    float x; // 横向位置（单位：米）
    float y; // 纵向位置（单位：米）
    float theta; // 旋转角度（单位：弧度）
} Chassis_Position_t;

void Chassis_InverseKinematics(Chassis_Param_t *chassis_param, Motor_Velocity_t *motor_velocity, Chassis_Velocity_t *chassis_velocity);
void Chassis_ForwardKinematics(Chassis_Param_t *chassis_param, Chassis_Velocity_t *chassis_velocity, Motor_Velocity_t *motor_velocity);
void WorldSpeedToChassisSpeed(float rad, float world_vx, float world_vy, float *chassis_vx, float *chassis_vy);

#endif // __CHASSIS_H__
