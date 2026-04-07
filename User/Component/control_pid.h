#ifndef __CONTROL_PID_H__
#define __CONTROL_PID_H__

typedef struct PID_Controller_vtable_t PID_Controller_vtable_t;

typedef struct
{
    float Kp;
    float Ki;
    float Kd;
    float p_out;  // 比例环节输出
    float i_out;  // 积分环节输出
    float d_out;  // 微分环节输出
    float output;  // 总输出

    float setpoint;
    float integral;
    float max_integral;
    float last_error;
    float previous_error;

    PID_Controller_vtable_t* vtable;
} PID_Controller_t;

struct PID_Controller_vtable_t
{
    void (*init)(PID_Controller_t* pid, float Kp, float Ki, float Kd, float max_integral);
    float (*compute)(PID_Controller_t* pid, float input);
    float (*get_output)(PID_Controller_t* pid);
};

#endif /* __CONTROL_PID_H__ */