#include "control_pid.h"

void inline PID_Compute(PID_Controller_t* pid, float setpoint, float measurement)
{
    // 更新PID控制器的状态
    pid->setpoint = setpoint;
    pid->measurement = measurement;
    pid->error = pid->setpoint - pid->measurement;
    pid->integral += pid->error;

    // 积分限幅，防止积分过大导致系统不稳定
    if (pid->integral > pid->max_integral)
    {
        pid->integral = pid->max_integral;
    }
    else if (pid->integral < -pid->max_integral)
    {
        pid->integral = -pid->max_integral;
    }

    // 计算PID输出
    pid->p_out = pid->Kp * pid->error;
    pid->i_out = pid->Ki * pid->integral;
    pid->d_out = pid->Kd * (pid->error - pid->previous_error);

    // 更新上次的误差值
    pid->previous_error = pid->error;

    // 计算总输出
    pid->output = pid->p_out + pid->i_out + pid->d_out;
}

float inline PID_GetOutput(PID_Controller_t* pid)
{
    return pid->output;
}