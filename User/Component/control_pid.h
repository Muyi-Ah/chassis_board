#ifndef __CONTROL_PID_H__
#define __CONTROL_PID_H__

typedef struct PID_Controller_vtable_struct PID_Controller_vtable_t; // 前向声明PID控制器虚函数表结构体

/** 
 * @brief PID控制器结构体 
 */
typedef struct
{
    PID_Controller_vtable_t* vtable; // 虚函数表指针

    float Kp;
    float Ki;
    float Kd;
    float p_out;  // 比例环节输出
    float i_out;  // 积分环节输出
    float d_out;  // 微分环节输出
    
    float setpoint; // 目标值
    float measurement; // 测量值

    float integral; // 积分值
    float max_integral; // 积分限幅，防止积分过大导致系统不稳定

    float error; // 当前误差值
    float previous_error; // 上次的误差值，用于计算微分

    float output;  // 总输出
} PID_Controller_t;

/**
 * @brief PID控制器虚函数表结构体
 */
struct PID_Controller_vtable_struct
{
    void (*compute)(PID_Controller_t* pid, float setpoint, float measurement);
    float (*get_output)(PID_Controller_t* pid);
};

void PID_Compute(PID_Controller_t* pid, float setpoint, float measurement);
float PID_GetOutput(PID_Controller_t* pid);

#endif /* __CONTROL_PID_H__ */