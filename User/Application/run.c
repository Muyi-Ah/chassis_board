#include "run.h"
#include "bsp_led.h"
#include "bsp_time.h"
#include "FreeRTOS.h"
#include "cmsis_gcc.h"
#include "cmsis_os2.h"
#include "task.h"
#include "bsp_uart.h"
#include "bsp_instance.h"
#include "device_instance.h"
#include "router.h"
#include "chassis.h"
#include "clamp.h"
#include "bmi08.h"
#include "driver_bmi088.h"
#include "MahonyAHRS.h"
#include "kalman.h"
#include <math.h>

#define M_PI_F 3.14159265358979323846f

// EKF 底盘定位滤波器实例
struct KalmanFilter chassis_kf;

/** 
 * @brief 将角度限制在 [-pi, pi] 范围内
 * @param a 输入角度
 * @return 限制后的角度
 */
static inline float wrap_to_pi(float a)
{
    while (a >= M_PI_F) a -= 2.0f * M_PI_F;
    while (a <  -M_PI_F) a += 2.0f * M_PI_F;
    return a;
}

/** 
 * @brief 快速整型转字符串函数
 * @param str 字符串缓冲区
 * @param val 要转换的整型值
 * @return 转换后字符串的指针
 */
static inline char* fast_itoa(char *str, int32_t val) {
    if (val == 0) {
        *str++ = '0';
        return str;
    }
    if (val < 0) {
        *str++ = '-';
        val = -val;
    }
    char temp[10]; 
    int i = 0;
    while (val > 0) {
        temp[i++] = (val % 10) + '0';
        val /= 10;
    }
    while (i > 0) {
        *str++ = temp[--i];
    }
    return str;
}

/** 
 * @brief 梯形加减速限制函数 (Slew Rate Limiter)
 * @param current_velocity 当前实际速度 (会被更新)
 * @param target_velocity 目标速度
 * @param max_accel 最大加速度 (单位: 速度单位/秒)
 * @param max_decel 最大减速度 (单位: 速度单位/秒)
 * @param dt 控制周期 (单位: 秒)
 */
static inline void slew_rate_limit(float *current_velocity, float target_velocity, float max_accel, float max_decel, float dt) {
    float error = target_velocity - *current_velocity;
    
    // 加速或减速的判定：
    // 如果目标速度和当前速度同号，且目标绝对值 > 当前绝对值，说明在加速；否则在减速(含刹车和反转)
    float limit_step;
    if (target_velocity * (*current_velocity) >= 0.0f && fabsf(target_velocity) > fabsf(*current_velocity)) {
        limit_step = max_accel * dt;
    } else {
        limit_step = max_decel * dt;
    }
    
    if (error > limit_step) {
        *current_velocity += limit_step;
    } else if (error < -limit_step) {
        *current_velocity -= limit_step;
    } else {
        *current_velocity = target_velocity;
    }
}

/** 
 * @brief 运行任务函数
 */
void runTask(void *argument)
{
    // 等待 IMU 初始化完成事件 (标志位 0x00000001U 为 1)
    // 使用 osFlagsNoClear，让其他任务也能等到这个标志
    extern osEventFlagsId_t imu_init_event;
    osEventFlagsWait(imu_init_event, 0x00000001U, osFlagsWaitAny | osFlagsNoClear, osWaitForever);

    // 用于保存经过斜率限制后的实际控制速度
    float current_vx_cmd = 0.0f;
    float current_vy_cmd = 0.0f;
    float current_vw_cmd = 0.0f;
    
    // 加减速参数配置 (单位: m/s^2 或 rad/s^2)
    // 假设 0.3m/s 的最大速度，用 0.5s 达到，则加速度为 0.6 m/s^2
    const float MAX_ACCEL_LINEAR = 0.8f; // 线性加速度最大值
    const float MAX_DECEL_LINEAR = 1.2f; // 线性减速最大值
    const float MAX_ACCEL_ANGULAR = 1.5f; // 角度加速度最大值
    const float MAX_DECEL_ANGULAR = 3.0f; // 角度减速最大值

    // 初始化 EKF 滤波器
    kalman_init(&chassis_kf);

    uint32_t wake_time = osKernelGetTickCount();

    // M3508 减速比约 19.2032 (3591.0f / 187.0f)
    // 将车轮的目标角速度 (rad/s) 乘以减速比，转换为电机转子的目标角速度 (rad/s)
    float reduction_ratio = 3591.0f / 187.0f;
    float reduction_ratio_inv = 1.0f / reduction_ratio;
    float dr16_normalize_inv = 1.0f / 660.0f;

    while (1)
    {
        /* ----------------------------------------------------
         * 5态 EKF 数据融合定位
         * ---------------------------------------------------- */
        static float last_yaw_rad = 0.0f;
        float current_yaw_rad = euler_angles.yaw_rad;
        float delta_theta = current_yaw_rad - last_yaw_rad;
        
        // 处理偏航角跨越 -pi 和 pi 的跳变
        while (delta_theta > M_PI_F)  delta_theta -= 2.0f * M_PI_F;
        while (delta_theta < -M_PI_F) delta_theta += 2.0f * M_PI_F;
        last_yaw_rad = current_yaw_rad;

        // EKF 预测步 (传入加速度计和角度增量)
        kalman_predict(&chassis_kf, accel_ms2_body.x, accel_ms2_body.y, delta_theta, 0.001f);

        // EKF 更新步 (传入轮速里程计速度)
        kalman_update(&chassis_kf, chassis_observe_velocity.vx, 
                                   chassis_observe_velocity.vy, 
                                   chassis_observe_velocity_filtered.vw);

        // 更新到底盘位置结构体供外部使用
        chassis_position.x = chassis_kf.X_data[0];
        chassis_position.y = chassis_kf.X_data[1];
        chassis_position.theta = chassis_kf.X_data[2];

        /* ----------------------------------------------------
         * 5态 EKF 数据融合定位结束
         * ---------------------------------------------------- */

        /* ----------------------------------------------------
        * 底盘控制
        * ---------------------------------------------------- */
        if (dr16_data.s2 == 3) {
            // chassis_target_position.theta = M_PI_F * 0.25f;
            chassis_target_position.x = 0.0f;
            chassis_target_position.y = 1.8f;
        }
        else if (dr16_data.s2 == 2) {
            // chassis_target_position.theta = M_PI_F * 0.5f;
            chassis_target_position.x = 0.0f;
            chassis_target_position.y = 0.0f;
        }
        else {
            chassis_target_position.theta = 0.0f;
            chassis_target_position.x = 0.0f;
            chassis_target_position.y = 0.0f;
        }

        if (dr16_data.s1 == 3)
        {
            // 遥控器控制底盘速度
            chassis_control_velocity.vx = 0.8f * dr16_data.ch1 * dr16_normalize_inv;
            chassis_control_velocity.vy = -(0.8f * dr16_data.ch0 * dr16_normalize_inv);
            chassis_control_velocity.vw = -(1.5f * dr16_data.ch2 * dr16_normalize_inv);
        }
        else if (dr16_data.s1 == 2) {
            // 世界坐标系下位置控制

            // 设置PID目标值
            pid_position_x_world.setpoint = chassis_target_position.x;
            pid_position_y_world.setpoint = chassis_target_position.y;

            // PID计算x、y轴速度
            pid_position_x_world.vtable->compute(&pid_position_x_world, pid_position_x_world.setpoint, chassis_position.x);
            pid_position_y_world.vtable->compute(&pid_position_y_world, pid_position_y_world.setpoint, chassis_position.y);

            // PID计算偏航角速度
            float theta_error = wrap_to_pi(chassis_target_position.theta - euler_angles.yaw_rad);
            pid_position_theta_world.setpoint = theta_error;
            pid_position_theta_world.vtable->compute(&pid_position_theta_world, pid_position_theta_world.setpoint, 0.0f);

            // 将世界坐标速度转换为底盘坐标速度
            float vx_target_world = pid_position_x_world.output;
            float vy_target_world = pid_position_y_world.output;

            float vx_cmd_chassis = 0.0f;
            float vy_cmd_chassis = 0.0f;

            // 将世界坐标速度转换为底盘坐标速度
            WorldSpeedToChassisSpeed(-chassis_position.theta, 
                                    vx_target_world, 
                                    vy_target_world, 
                                    &vx_cmd_chassis, 
                                    &vy_cmd_chassis);

            // 底盘控制速度设置
            chassis_control_velocity.vx = vx_cmd_chassis;
            chassis_control_velocity.vy = vy_cmd_chassis;
            chassis_control_velocity.vw = pid_position_theta_world.output;
        }
        else {
            chassis_control_velocity.vx = 0.0f;
            chassis_control_velocity.vy = 0.0f;
            chassis_control_velocity.vw = 0.0f;
        }

        // 对目标速度进行限制
        clamp(&chassis_control_velocity.vx, -MAX_ACCEL_LINEAR, MAX_ACCEL_LINEAR);
        clamp(&chassis_control_velocity.vy, -MAX_ACCEL_LINEAR, MAX_ACCEL_LINEAR);
        clamp(&chassis_control_velocity.vw, -MAX_ACCEL_ANGULAR, MAX_ACCEL_ANGULAR);
        
        /*** 底盘斜坡控制 (梯形加减速) ***/
        // 对遥控器产生的瞬态目标速度进行平滑限制
        slew_rate_limit(&current_vx_cmd, chassis_control_velocity.vx, MAX_ACCEL_LINEAR, MAX_DECEL_LINEAR, 0.001f);
        slew_rate_limit(&current_vy_cmd, chassis_control_velocity.vy, MAX_ACCEL_LINEAR, MAX_DECEL_LINEAR, 0.001f);
        slew_rate_limit(&current_vw_cmd, chassis_control_velocity.vw, MAX_ACCEL_ANGULAR, MAX_DECEL_ANGULAR, 0.001f);
        
        // 将平滑后的速度用于后续的逆运动学计算
        chassis_control_velocity.vx = current_vx_cmd;
        chassis_control_velocity.vy = current_vy_cmd;
        chassis_control_velocity.vw = current_vw_cmd;
        /*** 梯形加减速结束 ***/
        
        /*** 底盘逆运动学计算过程 ***/

        Chassis_InverseKinematics(&chassis_param, &motor_control_velocity, &chassis_control_velocity);

        // 设置每个电机的目标角速度
        pid_speed_motor1.setpoint = motor_control_velocity.w_lf * reduction_ratio;
        pid_speed_motor2.setpoint = motor_control_velocity.w_rf * reduction_ratio;
        pid_speed_motor3.setpoint = motor_control_velocity.w_rr * reduction_ratio;
        pid_speed_motor4.setpoint = motor_control_velocity.w_lr * reduction_ratio;

        /*** 底盘逆运动学计算结束 ***/

        // 计算每个电机的PID输出
        pid_speed_motor1.vtable->compute(&pid_speed_motor1, pid_speed_motor1.setpoint, motor1.rad_per_sec);
        pid_speed_motor2.vtable->compute(&pid_speed_motor2, pid_speed_motor2.setpoint, motor2.rad_per_sec);
        pid_speed_motor3.vtable->compute(&pid_speed_motor3, pid_speed_motor3.setpoint, motor3.rad_per_sec);
        pid_speed_motor4.vtable->compute(&pid_speed_motor4, pid_speed_motor4.setpoint, motor4.rad_per_sec);

        // 将PID输出发送到电调
        Motor_Group_Send_Currents(&can_bus_1,
            pid_speed_motor1.output,
            pid_speed_motor2.output,
            pid_speed_motor3.output,
            pid_speed_motor4.output);

        /*** 底盘正运动学计算过程 ***/
        motor_observe_velocity.w_lf = motor1.rad_per_sec * reduction_ratio_inv;
        motor_observe_velocity.w_rf = motor2.rad_per_sec * reduction_ratio_inv;
        motor_observe_velocity.w_rr = motor3.rad_per_sec * reduction_ratio_inv;
        motor_observe_velocity.w_lr = motor4.rad_per_sec * reduction_ratio_inv;

        // 通过正运动学计算底盘的当前速度
        Chassis_ForwardKinematics(&chassis_param, &chassis_observe_velocity, &motor_observe_velocity);
        
        // 对底盘速度进行低通滤波
        chassis_observe_velocity_filtered.vx = chassis_observe_velocity.lpf1_vx.vtable->compute(&chassis_observe_velocity.lpf1_vx, chassis_observe_velocity.vx);
        chassis_observe_velocity_filtered.vy = chassis_observe_velocity.lpf1_vy.vtable->compute(&chassis_observe_velocity.lpf1_vy, chassis_observe_velocity.vy);
        chassis_observe_velocity_filtered.vw = chassis_observe_velocity.lpf1_vw.vtable->compute(&chassis_observe_velocity.lpf1_vw, chassis_observe_velocity.vw);
        
        /*** 底盘正运动学计算结束 ***/

        /* ----------------------------------------------------
         * 底盘控制结束
         * ---------------------------------------------------- */

        /*** 里程计算 ***/
        /* ----------------------------------------------------
         * 纯轮速航迹推算 (保留用于对比测试)
         * ----------------------------------------------------
        const float VELOCITY_DEADZONE = 0.005f;
        const float OMEGA_DEADZONE = 0.01f;

        // 计算旋转角度
        if (!(fabsf(chassis_observe_velocity_filtered.vw) < OMEGA_DEADZONE))
        {
            chassis_position.theta += chassis_observe_velocity_filtered.vw * 0.001f;
        }

        // 确保旋转角度在 [-pi, pi] 范围内
        // 极其高效的 while 折叠法（因为每 1ms 角度变化极小，while 最多只会执行 1 次）
        while (chassis_position.theta > M_PI_F) {
            chassis_position.theta -= 2.0f * M_PI_F;
        }
        while (chassis_position.theta < -M_PI_F) {
            chassis_position.theta += 2.0f * M_PI_F;
        }

        // 计算位移量
        float current_vx = chassis_observe_velocity_filtered.vx;
        float current_vy = chassis_observe_velocity_filtered.vy;

        // 对速度进行死区处理，在三角函数变换之前过滤！
        if (fabsf(current_vx) < VELOCITY_DEADZONE) {
            current_vx = 0.0f;
        }
        if (fabsf(current_vy) < VELOCITY_DEADZONE) {
            current_vy = 0.0f;
        }

        // 计算位移量
        float delta_x = (current_vx * cosf(euler_angles.yaw_rad) - 
                        current_vy * sinf(euler_angles.yaw_rad)) * 0.001f;

        float delta_y = (current_vx * sinf(euler_angles.yaw_rad) + 
                        current_vy * cosf(euler_angles.yaw_rad)) * 0.001f;

        // 更新位置
        chassis_position.x += delta_x;
        chassis_position.y += delta_y;
        */

        /*** 里程计算结束 ***/

        wake_time += pdMS_TO_TICKS(1);
        osDelayUntil(wake_time); // 绝对延时，保证严格的 1ms 周期
    }
}

void blinkTask(void *argument)
{
    // 等待 IMU 初始化完成事件 (标志位 0x00000001U 为 1)
    // 使用 osFlagsNoClear，让其他任务也能等到这个标志
    extern osEventFlagsId_t imu_init_event;
    osEventFlagsWait(imu_init_event, 0x00000001U, osFlagsWaitAny | osFlagsNoClear, osWaitForever);

    while (1)
    {
        led_full_toggle();
        osDelay(pdMS_TO_TICKS(500)); // 延时500毫秒
    }
}

/** 
 * @brief 发送任务函数
 */
void sendTask(void *argument)
{
    // 暂停当前线程
    osThreadSuspend(osThreadGetId());

    extern osEventFlagsId_t imu_init_event;
    osEventFlagsWait(imu_init_event, 0x00000001U, osFlagsWaitAny | osFlagsNoClear, osWaitForever);

    char buffer[128];
    uint32_t wake_time = osKernelGetTickCount();
    while (1)
    {
        char *ptr = buffer;
        
        // ptr = fast_itoa(ptr, motor1.RPM); *ptr++ = ',';
        // ptr = fast_itoa(ptr, motor1.filtered_RPM); *ptr++ = ',';
        
        // ptr = fast_itoa(ptr, motor2.RPM); *ptr++ = ',';
        // ptr = fast_itoa(ptr, motor2.filtered_RPM); *ptr++ = ',';
        
        // ptr = fast_itoa(ptr, motor3.RPM); *ptr++ = ',';
        // ptr = fast_itoa(ptr, motor3.filtered_RPM); *ptr++ = ',';
        
        // ptr = fast_itoa(ptr, motor4.RPM); *ptr++ = ',';
        // ptr = fast_itoa(ptr, motor4.filtered_RPM);
        
        ptr = fast_itoa(ptr, (int32_t)(chassis_position.x * 1000.0f)); *ptr++ = ',';
        ptr = fast_itoa(ptr, (int32_t)(chassis_position.y * 1000.0f)); *ptr++ = ',';
        ptr = fast_itoa(ptr, (int32_t)(euler_angles.yaw_rad * 1000.0f));

        // ptr = fast_itoa(ptr, (int32_t)(chassis_observe_velocity.vx * 10000.0f)); *ptr++ = ',';
        // ptr = fast_itoa(ptr, (int32_t)(chassis_observe_velocity.vy * 10000.0f)); *ptr++ = ',';
        // ptr = fast_itoa(ptr, (int32_t)(chassis_observe_velocity_filtered.vw * 10000.0f));

        *ptr++ = '\r';
        *ptr++ = '\n';
        
        uint16_t len = (uint16_t)(ptr - buffer);
        uart_bus_2.vptr->send(&uart_bus_2, (uint8_t *)buffer, len); // 发送数据到UART2
        
        wake_time += pdMS_TO_TICKS(1);
        osDelayUntil(wake_time); // 每1ms执行一次
    }
}

#include "device_instance.h"
#include "MahonyAHRS.h"

// 引用外部四元数
extern volatile float q0, q1, q2, q3;
extern float gravity_accel;
void INSTask(void *argument)
{
    // === 1. IMU 零偏校准 (前3秒) ===
    // 1000Hz 运行频率，3秒 = 3000 次采样
    const uint32_t CALIBRATION_SAMPLES = 3000;
    uint32_t sample_count = 0;
    
    // 用于累加的变量 (使用双精度防止累加溢出和精度丢失)
    double sum_gyro_x = 0, sum_gyro_y = 0, sum_gyro_z = 0;
    double sum_accel_x = 0, sum_accel_y = 0, sum_accel_z = 0;
    
    // 临时存储转换后的物理单位，但不带零偏
    struct bmi08_sensor_data_f temp_gyro, temp_accel;
    struct imu_offset zero_offset = {0}; // 零偏全0结构体，仅用于临时转换

    uint32_t wake_time = osKernelGetTickCount();

    while(sample_count < CALIBRATION_SAMPLES)
    {
        // 读取传感器数据
        bmi08a_get_data(&accel_data, &bmi08_dev);
        bmi08g_get_data(&gyro_data, &bmi08_dev);

        // 转换 LSB 到物理单位 (不扣除零偏)
        driver_bmi088_gyro_to_rad_s(&gyro_data, &temp_gyro, &zero_offset);
        driver_bmi088_accel_to_ms2(&accel_data, &temp_accel, &zero_offset);

        // 累加
        sum_gyro_x += temp_gyro.x;
        sum_gyro_y += temp_gyro.y;
        sum_gyro_z += temp_gyro.z;
        sum_accel_x += temp_accel.x;
        sum_accel_y += temp_accel.y;
        sum_accel_z += temp_accel.z;

        sample_count++;
        
        wake_time += pdMS_TO_TICKS(1);
        osDelayUntil(wake_time); // 1ms 采样一次
    }

    // === 2. 计算平均零偏并写入全局配置 ===
    bmi088_offset.gyro_x = (float)(sum_gyro_x / CALIBRATION_SAMPLES);
    bmi088_offset.gyro_y = (float)(sum_gyro_y / CALIBRATION_SAMPLES);
    bmi088_offset.gyro_z = (float)(sum_gyro_z / CALIBRATION_SAMPLES);
    
    // 注意：由于在大多数应用场景中寻找“绝对水平面”极为困难，
    // 加速度计如果强行减去重力计算零偏，会导致倾斜放置上电时出现错误的零偏（将倾角算进了零偏里）。
    // 对于 Mahony 算法而言，加速度计的常数零偏对最终姿态（特别是 Pitch 和 Roll）影响很小。
    // 因此，行业标准做法是：每次上电只校准陀螺仪零偏，不对加速度计做上电动态零偏校准。
    // 加速度计如果真需要校准，通常是在工厂用六面椭球拟合法（六面校准）算出一套死参数写在 Flash 里。
    bmi088_offset.accel_x = (float)(sum_accel_x / CALIBRATION_SAMPLES);
    bmi088_offset.accel_y = (float)(sum_accel_y / CALIBRATION_SAMPLES);
    bmi088_offset.accel_z = (float)(sum_accel_z / CALIBRATION_SAMPLES);
    gravity_accel = (float)sqrtf(bmi088_offset.accel_x * bmi088_offset.accel_x + 
                                    bmi088_offset.accel_y * bmi088_offset.accel_y + 
                                    bmi088_offset.accel_z * bmi088_offset.accel_z);
    
    bmi088_offset.is_calibrated = 1; // 标记校准完成

    // 广播 IMU 初始化和零偏校准完成事件
    extern osEventFlagsId_t imu_init_event;
    osEventFlagsSet(imu_init_event, 0x00000001U);

    // === 3. 正常运行 Mahony 解算 ===
    while(1)
    {
        // 1. 获取数据
        bmi08a_get_data(&accel_data, &bmi08_dev);
        bmi08g_get_data(&gyro_data, &bmi08_dev);

        // 2. 转换为物理单位并自动扣除刚计算出的零偏 (bmi088_offset)
        driver_bmi088_gyro_to_rad_s(&gyro_data, &gyro_rad_s, &bmi088_offset);
        driver_bmi088_accel_to_ms2(&accel_data, &accel_ms2, &bmi088_offset);

        // 3. 送入 Mahony 融合
        MahonyAHRSupdateIMU(gyro_rad_s.x, gyro_rad_s.y, gyro_rad_s.z,
                            accel_ms2.x, accel_ms2.y, accel_ms2.z);

        // 4. 提取欧拉角
        driver_bmi088_quaternion_to_euler(q0, q1, q2, q3, &euler_angles);

        // 5. 获取去除重力分量后的加速度计数据
        driver_bmi088_body_gravity(&accel_ms2, &accel_ms2_body, &euler_angles, gravity_accel);

        wake_time += pdMS_TO_TICKS(1);
        osDelayUntil(wake_time); // 每1毫秒执行一次
    }
}
