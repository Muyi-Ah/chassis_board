#include "chassis.h"

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
