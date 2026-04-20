#include "chassis.h"
#include <math.h>

/**
 * @brief 计算底盘逆运动学
 * @param chassis_velocity 底盘速度
 * @return 电机速度
 */
void Chassis_InverseKinematics(Chassis_Param_t *chassis_param, Motor_Velocity_t *motor_velocity, Chassis_Velocity_t *chassis_velocity) {
    // 这里需要根据底盘的几何参数进行计算
    float K = chassis_param->Lx + chassis_param->Ly;
    float R = chassis_param->Wheel_R;

    motor_velocity->w_lf = (chassis_velocity->vx - chassis_velocity->vy + (K) * (-chassis_velocity->vw)) / R;
    motor_velocity->w_lr = (chassis_velocity->vx + chassis_velocity->vy + (K) * (-chassis_velocity->vw)) / R;
    motor_velocity->w_rr = (-chassis_velocity->vx + chassis_velocity->vy + (K) * (-chassis_velocity->vw)) / R;
    motor_velocity->w_rf = (-chassis_velocity->vx - chassis_velocity->vy + (K) * (-chassis_velocity->vw)) / R;
}

/**
 * @brief 计算底盘正运动学
 * @param motor_velocity 电机速度
 * @return 底盘速度
 */
void Chassis_ForwardKinematics(Chassis_Param_t *chassis_param, Chassis_Velocity_t *chassis_velocity, Motor_Velocity_t *motor_velocity) {
    // 这里需要根据底盘的几何参数进行计算，以下是一个示例
    float K = chassis_param->Lx + chassis_param->Ly;
    float R = chassis_param->Wheel_R;

    // float vw_copmp_factor = 1080.0f / 877.0f;
    float vw_copmp_factor = 1.0f;
    chassis_velocity->vx = (motor_velocity->w_lf + motor_velocity->w_lr - motor_velocity->w_rr - motor_velocity->w_rf) * R / 4.0f;
    chassis_velocity->vy = (-motor_velocity->w_lf + motor_velocity->w_lr + motor_velocity->w_rr - motor_velocity->w_rf) * R / 4.0f;
    chassis_velocity->vw = (-motor_velocity->w_lf - motor_velocity->w_lr - motor_velocity->w_rr - motor_velocity->w_rf) * R / (4.0f * (K)) * vw_copmp_factor;
}

/**
 * @brief 将世界坐标转换为底盘坐标
 * @param euler_angles 旋转角度（单位：弧度）
 * @param world_vx 世界坐标速度x轴（单位：米/秒）
 * @param world_vy 世界坐标速度y轴（单位：米/秒）
 * @param chassis_vx 底盘坐标速度x轴（单位：米/秒）
 * @param chassis_vy 底盘坐标速度y轴（单位：米/秒）
 */
void WorldSpeedToChassisSpeed(float rad, float world_vx, float world_vy, float *chassis_vx, float *chassis_vy) {
    *chassis_vx = cosf(rad) * world_vx - sinf(rad) * world_vy;
    *chassis_vy = sinf(rad) * world_vx + cosf(rad) * world_vy;
}
