#include "kalman.h"
#include "arm_math.h"
#include <string.h>

#ifndef M_PI_F
#define M_PI_F 3.14159265358979323846f
#endif

// 辅助函数：将角度限制在 [-pi, pi] 范围内
static inline float constrain_angle(float angle) {
    while (angle > M_PI_F) {
        angle -= 2.0f * M_PI_F;
    }
    while (angle < -M_PI_F) {
        angle += 2.0f * M_PI_F;
    }
    return angle;
}

void kalman_init(struct KalmanFilter *kf)
{
    // 清零所有数据
    memset(kf, 0, sizeof(struct KalmanFilter));

    // 初始化状态相关矩阵实例
    arm_mat_init_f32(&kf->mat_X, 5, 1, kf->X_data);
    arm_mat_init_f32(&kf->mat_P, 5, 5, kf->P_data);
    arm_mat_init_f32(&kf->mat_F, 5, 5, kf->F_data);
    arm_mat_init_f32(&kf->mat_Q, 5, 5, kf->Q_data);

    // 初始化观测相关矩阵实例
    arm_mat_init_f32(&kf->mat_Z, 3, 1, kf->Z_data);
    arm_mat_init_f32(&kf->mat_H, 3, 5, kf->H_data);
    arm_mat_init_f32(&kf->mat_R, 3, 3, kf->R_data);

    // 初始化中间变量矩阵实例
    arm_mat_init_f32(&kf->mat_Y, 3, 1, kf->Y_data);
    arm_mat_init_f32(&kf->mat_S, 3, 3, kf->S_data);
    arm_mat_init_f32(&kf->mat_S_inv, 3, 3, kf->S_inv_data);
    arm_mat_init_f32(&kf->mat_K, 5, 3, kf->K_data);

    // 初始化计算缓存矩阵实例
    arm_mat_init_f32(&kf->mat_F_T, 5, 5, kf->F_T_data);
    arm_mat_init_f32(&kf->mat_FP, 5, 5, kf->FP_data);
    arm_mat_init_f32(&kf->mat_FPF_T, 5, 5, kf->FPF_T_data);
    
    arm_mat_init_f32(&kf->mat_HX, 3, 1, kf->HX_data);
    arm_mat_init_f32(&kf->mat_H_T, 5, 3, kf->H_T_data);
    arm_mat_init_f32(&kf->mat_HP, 3, 5, kf->HP_data);
    arm_mat_init_f32(&kf->mat_HPH_T, 3, 3, kf->HPH_T_data);
    arm_mat_init_f32(&kf->mat_PH_T, 5, 3, kf->PH_T_data);
    arm_mat_init_f32(&kf->mat_KY, 5, 1, kf->KY_data);
    arm_mat_init_f32(&kf->mat_K_HP, 5, 5, kf->K_HP_data);

    // 初始化观测矩阵 H (常数矩阵)
    // Z = [vx_wheel, vy_wheel, theta]^T
    // X = [x, y, theta, vxb, vyb]^T
    // vx_wheel 对应 vxb -> H[0][3] = 1
    // vy_wheel 对应 vyb -> H[1][4] = 1
    // theta_pseudo 对应 theta -> H[2][2] = 1
    kf->H_data[0*5 + 3] = 1.0f;
    kf->H_data[1*5 + 4] = 1.0f;
    kf->H_data[2*5 + 2] = 1.0f;
    // 预计算 H 的转置 H^T
    arm_mat_trans_f32(&kf->mat_H, &kf->mat_H_T);

    // 初始化过程噪声协方差 Q (可根据实际情况调参)
    kf->Q_data[0*5 + 0] = 1e-4f; // x
    kf->Q_data[1*5 + 1] = 1e-4f; // y
    kf->Q_data[2*5 + 2] = 1e-5f; // theta
    kf->Q_data[3*5 + 3] = 1e-2f; // vxb
    kf->Q_data[4*5 + 4] = 1e-2f; // vyb

    // 初始化状态协方差 P
    kf->P_data[0*5 + 0] = 1.0f;
    kf->P_data[1*5 + 1] = 1.0f;
    kf->P_data[2*5 + 2] = 1.0f;
    kf->P_data[3*5 + 3] = 1.0f;
    kf->P_data[4*5 + 4] = 1.0f;
    
    // ZUPT 初始化
    kf->is_stationary = false;
    kf->theta_locked = 0.0f;
}

void kalman_predict(struct KalmanFilter *kf, float u_ax, float u_ay, float u_dtheta, float dt)
{
    float x_last = kf->X_data[0];
    float y_last = kf->X_data[1];
    float theta_last = kf->X_data[2];
    float vx_last = kf->X_data[3];
    float vy_last = kf->X_data[4];

    float sin_t = sinf(theta_last);
    float cos_t = cosf(theta_last);

    // ==========================================
    // 1. 状态预测 X_pred = f(X_last, U)
    // ==========================================
    kf->X_data[0] = x_last + (vx_last * cos_t - vy_last * sin_t) * dt;
    kf->X_data[1] = y_last + (vx_last * sin_t + vy_last * cos_t) * dt;
    kf->X_data[2] = constrain_angle(theta_last + u_dtheta);
    kf->X_data[3] = vx_last + u_ax * dt;
    kf->X_data[4] = vy_last + u_ay * dt;

    // ==========================================
    // 2. 协方差预测 P_pred = F * P_last * F^T + Q
    // ==========================================
    // 构建状态转移雅可比矩阵 F
    // 默认全为0，需要每次都初始化对角线为1
    memset(kf->F_data, 0, sizeof(kf->F_data));
    kf->F_data[0*5 + 0] = 1.0f;
    kf->F_data[1*5 + 1] = 1.0f;
    kf->F_data[2*5 + 2] = 1.0f;
    kf->F_data[3*5 + 3] = 1.0f;
    kf->F_data[4*5 + 4] = 1.0f;

    // 填入非零偏导数
    kf->F_data[0*5 + 2] = (-vx_last * sin_t - vy_last * cos_t) * dt;
    kf->F_data[0*5 + 3] = cos_t * dt;
    kf->F_data[0*5 + 4] = -sin_t * dt;

    kf->F_data[1*5 + 2] = (vx_last * cos_t - vy_last * sin_t) * dt;
    kf->F_data[1*5 + 3] = sin_t * dt;
    kf->F_data[1*5 + 4] = cos_t * dt;

    // 矩阵运算: P = F * P * F^T + Q
    arm_mat_mult_f32(&kf->mat_F, &kf->mat_P, &kf->mat_FP);            // FP = F * P
    arm_mat_trans_f32(&kf->mat_F, &kf->mat_F_T);                      // F_T = F^T
    arm_mat_mult_f32(&kf->mat_FP, &kf->mat_F_T, &kf->mat_FPF_T);      // FPF_T = FP * F^T
    arm_mat_add_f32(&kf->mat_FPF_T, &kf->mat_Q, &kf->mat_P);          // P = FPF_T + Q
}

void kalman_update(struct KalmanFilter *kf, float vx_wheel, float vy_wheel, float vw_wheel)
{
    // ==========================================
    // 1. ZUPT 零速更新与自适应 R 矩阵配置
    // ==========================================
    // 判定底盘是否处于完全静止状态
    bool current_stationary = (fabsf(vx_wheel) < 0.005f && fabsf(vy_wheel) < 0.005f && fabsf(vw_wheel) < 0.005f);

    if (current_stationary && !kf->is_stationary) {
        // 刚刚停下，锁定当前的偏航角
        kf->theta_locked = kf->X_data[2];
    }
    kf->is_stationary = current_stationary;

    // 填充观测向量 Z (Z[0]=vx_w, Z[1]=vy_w)
    kf->Z_data[0] = vx_wheel;
    kf->Z_data[1] = vy_wheel;

    // 根据运动状态配置自适应观测协方差 R 和观测 Z[2]
    memset(kf->R_data, 0, sizeof(kf->R_data)); // 清零非对角线
    if (kf->is_stationary) {
        // 静止状态：高度信任轮速为0，高度信任角度锁定（消灭静止漂移）
        kf->Z_data[2] = kf->theta_locked;
        
        kf->R_data[0*3 + 0] = 1e-4f; // vx_wheel noise
        kf->R_data[1*3 + 1] = 1e-4f; // vy_wheel noise
        kf->R_data[2*3 + 2] = 1e-5f; // theta noise (钉死航向)
    } else {
        // 运动状态：信任IMU高频预测，降低轮速信任度（防打滑），完全忽略角度伪观测（让Mahony自由发挥）
        kf->Z_data[2] = kf->X_data[2]; // Z设为预测值，制造0残差
        
        kf->R_data[0*3 + 0] = 0.05f;   // vx_wheel noise (稍微大点防打滑)
        kf->R_data[1*3 + 1] = 0.05f;   // vy_wheel noise
        kf->R_data[2*3 + 2] = 1000.0f; // theta noise (极大值，相当于不更新角度)
    }

    // ==========================================
    // 2. 卡尔曼标准更新步骤
    // ==========================================
    // 步骤 1: Y = Z - H * X
    arm_mat_mult_f32(&kf->mat_H, &kf->mat_X, &kf->mat_HX);
    arm_mat_sub_f32(&kf->mat_Z, &kf->mat_HX, &kf->mat_Y);

    // 对于角度残差 Y[2]，需要将其约束在 [-pi, pi] 范围内
    kf->Y_data[2] = constrain_angle(kf->Y_data[2]);

    // 步骤 2: S = H * P * H^T + R
    arm_mat_mult_f32(&kf->mat_H, &kf->mat_P, &kf->mat_HP);             // HP = H * P
    arm_mat_mult_f32(&kf->mat_HP, &kf->mat_H_T, &kf->mat_HPH_T);       // HPH_T = HP * H^T
    arm_mat_add_f32(&kf->mat_HPH_T, &kf->mat_R, &kf->mat_S);           // S = HPH_T + R

    // 步骤 3: K = P * H^T * S^-1
    arm_status status = arm_mat_inverse_f32(&kf->mat_S, &kf->mat_S_inv);
    if (status != ARM_MATH_SUCCESS) {
        // 矩阵奇异，求逆失败。保护机制：直接跳过本次更新。
        return;
    }
    arm_mat_mult_f32(&kf->mat_P, &kf->mat_H_T, &kf->mat_PH_T);         // PH_T = P * H^T
    arm_mat_mult_f32(&kf->mat_PH_T, &kf->mat_S_inv, &kf->mat_K);       // K = PH_T * S_inv

    // 步骤 4: X = X + K * Y
    arm_mat_mult_f32(&kf->mat_K, &kf->mat_Y, &kf->mat_KY);             // KY = K * Y
    arm_mat_add_f32(&kf->mat_X, &kf->mat_KY, &kf->mat_X);              // X = X + KY
    kf->X_data[2] = constrain_angle(kf->X_data[2]);                    // 保证更新后的角度合法

    // 步骤 5: P = P - K * H * P
    // 因为前面已经算过 HP = H * P，这里直接复用
    arm_mat_mult_f32(&kf->mat_K, &kf->mat_HP, &kf->mat_K_HP);          // K_HP = K * (HP)
    arm_mat_sub_f32(&kf->mat_P, &kf->mat_K_HP, &kf->mat_P);            // P = P - K_HP
}
