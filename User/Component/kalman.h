#ifndef __KALMAN_H__
#define __KALMAN_H__

#include <stdint.h>
#include <stdbool.h>
#include "arm_math.h"

// 5态扩展卡尔曼滤波器结构体
struct KalmanFilter
{
    // 状态相关矩阵数据 (n = 5)
    // 状态向量 X = [x, y, theta, v_xb, v_yb]^T
    float X_data[5];
    float P_data[25];
    float F_data[25];
    float Q_data[25];

    // 观测相关矩阵数据 (m = 3)
    // 观测向量 Z = [vx_wheel, vy_wheel, theta_pseudo]^T
    float Z_data[3];
    float H_data[15];
    float R_data[9];

    // 卡尔曼运算中间变量数据
    float Y_data[3];        // 3x1
    float S_data[9];        // 3x3
    float S_inv_data[9];    // 3x3
    float K_data[15];       // 5x3

    // 矩阵连乘和加减的缓存数据
    float F_T_data[25];     // 5x5
    float FP_data[25];      // 5x5
    float FPF_T_data[25];   // 5x5
    
    float HX_data[3];       // 3x1
    float H_T_data[15];     // 5x3
    float HP_data[15];      // 3x5
    float HPH_T_data[9];    // 3x3
    float PH_T_data[15];    // 5x3
    float KY_data[5];       // 5x1
    float K_HP_data[25];    // 5x5

    // CMSIS-DSP 矩阵实例
    arm_matrix_instance_f32 mat_X;
    arm_matrix_instance_f32 mat_P;
    arm_matrix_instance_f32 mat_F;
    arm_matrix_instance_f32 mat_Q;

    arm_matrix_instance_f32 mat_Z;
    arm_matrix_instance_f32 mat_H;
    arm_matrix_instance_f32 mat_R;
    
    arm_matrix_instance_f32 mat_Y;
    arm_matrix_instance_f32 mat_S;
    arm_matrix_instance_f32 mat_S_inv;
    arm_matrix_instance_f32 mat_K;

    arm_matrix_instance_f32 mat_F_T;
    arm_matrix_instance_f32 mat_FP;
    arm_matrix_instance_f32 mat_FPF_T;

    arm_matrix_instance_f32 mat_HX;
    arm_matrix_instance_f32 mat_H_T;
    arm_matrix_instance_f32 mat_HP;
    arm_matrix_instance_f32 mat_HPH_T;
    arm_matrix_instance_f32 mat_PH_T;
    arm_matrix_instance_f32 mat_KY;
    arm_matrix_instance_f32 mat_K_HP;

    // 零速更新 (ZUPT) 逻辑变量
    bool is_stationary;
    float theta_locked;
};

// 函数声明
void kalman_init(struct KalmanFilter *kf);
void kalman_predict(struct KalmanFilter *kf, float u_ax, float u_ay, float u_dtheta, float dt);
void kalman_update(struct KalmanFilter *kf, float vx_wheel, float vy_wheel, float vw_wheel);

#endif // __KALMAN_H__
