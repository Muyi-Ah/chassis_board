#include "device_instance.h"
#include "chassis.h"
#include "device_dr16.h"
#include "filter_lpf1.h"
#include "control_pid.h"
#include "bmi08_defs.h"
#include "bmi08x.h"
#include "stm32f4xx_hal.h"
#include "driver_bmi088.h"

// 低通滤波器虚函数表实例
LPF1_vtable_t lpf1_vtable = {
    .compute = LPF1_compute // 低通滤波器计算函数指针
};

// 电机虚函数表实例
Motor_vtable_t motor_vtable = {
    .data_update = Motor_DataUpdate, // 电机数据更新函数指针
    .set_speed_rpm = Motor_SetSpeedRPM, // 设置电机速度函数指针
    .set_speed_rad_per_sec = Motor_SetSpeedRadPerSec, // 设置电机速度函数指针
    .get_speed_rpm = Motor_GetSpeedRPM, // 获取电机每分钟转速函数指针
    .get_speed_rad_per_sec = Motor_GetSpeedRadPerSec, // 获取电机每秒转速函数指针
    .get_encoder_value = Motor_GetEncoderValue, // 获取电机编码器值函数指针
    .get_current = Motor_GetCurrent // 获取电机电流函数指针
};

// 电机1实例
Motor_t motor1 = {
    .lpf1_rpm = {
        .alpha = 0.5f, // 低通滤波器的平滑因子，取值范围为0到1
        .input = 0, // 初始输入值
        .previous_output = 0, // 初始上一次输出值
        .output = 0, // 初始输出值
        .vtable = &lpf1_vtable // 虚函数表指针
    },
    .last_update_timestamp_ms = 0, // 初始上次更新数据的时间戳，单位为毫秒
    .type = M3508, // 电机类型
    .id = 0x201, // 电机ID
    .encoder_value = 0, // 初始编码器值
    .RPM = 0, // 初始速度
    .filtered_RPM = 0, // 初始速度经过滤波后的值
    .current = 0, // 初始电流
    .temperature = 0, // 初始温度
    .vptr = &motor_vtable // 虚函数表指针
};

// 电机2实例
Motor_t motor2 = {
    .lpf1_rpm = {
        .alpha = 0.5f, // 低通滤波器的平滑因子，取值范围为0到1
        .input = 0, // 初始输入值
        .previous_output = 0, // 初始上一次输出值
        .output = 0, // 初始输出值
        .vtable = &lpf1_vtable // 虚函数表指针
    },
    .last_update_timestamp_ms = 0, // 初始上次更新数据的时间戳，单位为毫秒
    .type = M3508, // 电机类型
    .id = 0x202, // 电机ID
    .encoder_value = 0, // 初始编码器值
    .RPM = 0, // 初始速度
    .filtered_RPM = 0, // 初始速度经过滤波后的值
    .current = 0, // 初始电流
    .temperature = 0, // 初始温度
    .vptr = &motor_vtable // 虚函数表指针
};

// 电机3实例
Motor_t motor3 = {
    .lpf1_rpm = {
        .alpha = 0.5f, // 低通滤波器的平滑因子，取值范围为0到1
        .input = 0, // 初始输入值
        .previous_output = 0, // 初始上一次输出值
        .output = 0, // 初始输出值
        .vtable = &lpf1_vtable // 虚函数表指针
    },
    .last_update_timestamp_ms = 0, // 初始上次更新数据的时间戳，单位为毫秒
    .type = M3508, // 电机类型
    .id = 0x203, // 电机ID
    .encoder_value = 0, // 初始编码器值
    .RPM = 0, // 初始速度
    .filtered_RPM = 0, // 初始速度经过滤波后的值
    .current = 0, // 初始电流
    .temperature = 0, // 初始温度
    .vptr = &motor_vtable // 虚函数表指针
};

// 电机4实例
Motor_t motor4 = {
    .lpf1_rpm = {
        .alpha = 0.5f, // 低通滤波器的平滑因子，取值范围为0到1
        .input = 0, // 初始输入值
        .previous_output = 0, // 初始上一次输出值
        .output = 0, // 初始输出值
        .vtable = &lpf1_vtable // 虚函数表指针
    },
    .last_update_timestamp_ms = 0, // 初始上次更新数据的时间戳，单位为毫秒
    .type = M3508, // 电机类型
    .id = 0x204, // 电机ID
    .encoder_value = 0, // 初始编码器值
    .RPM = 0, // 初始速度
    .filtered_RPM = 0, // 初始速度经过滤波后的值
    .current = 0, // 初始电流
    .temperature = 0, // 初始温度
    .vptr = &motor_vtable // 虚函数表指针
};

// DR16遥控器虚函数表实例
DR16_vtable_t dr16_vtable = {
    .data_update = DR16_DataUpdate, // DR16数据更新函数指针
    .get_ch0 = DR16_GetCh0, // 获取通道0数据函数指针
    .get_ch1 = DR16_GetCh1, // 获取通道1数据函数指针
    .get_ch2 = DR16_GetCh2, // 获取通道2数据函数指针
    .get_ch3 = DR16_GetCh3, // 获取通道3数据函数指针
    .get_s1 = DR16_GetS1, // 获取开关1状态函数指针
    .get_s2 = DR16_GetS2 // 获取开关2状态函数指针
};

// DR16遥控器数据实例
DR16_Data_t dr16_data = {
    .ch0 = 0, // 初始通道0数据
    .ch1 = 0, // 初始通道1数据
    .ch2 = 0, // 初始通道2数据
    .ch3 = 0, // 初始通道3数据
    .s1 = 0, // 初始开关1状态
    .s2 = 0, // 初始开关2状态
    .vptr = &dr16_vtable // 虚函数表指针
};

// PID控制器虚函数表实例
PID_Controller_vtable_t pid_vtable = {
    .compute = PID_Compute, // PID计算函数指针
    .get_output = PID_GetOutput // 获取PID输出函数指针
};

// 电机1速度控制器实例
PID_Controller_t pid_speed_motor1 = {
    .Kp = 200.0f, // 比例增益
    .Ki = 0.0f, // 积分增益
    .Kd = 0.0f, // 微分增益
    .integral = 0, // 初始积分值
    .max_integral = 0, // 积分限幅，防止积分过大导致系统不稳定
    .previous_error = 0, // 初始上次误差值
    .output = 0, // 初始输出值
    .vtable = &pid_vtable // 虚函数表指针
};

// 电机2速度控制器实例
PID_Controller_t pid_speed_motor2 = {
    .Kp = 200.0f, // 比例增益
    .Ki = 0.0f, // 积分增益
    .Kd = 0.0f, // 微分增益
    .integral = 0, // 初始积分值
    .max_integral = 0, // 积分限幅，防止积分过大导致系统不稳定
    .previous_error = 0, // 初始上次误差值
    .output = 0, // 初始输出值
    .vtable = &pid_vtable // 虚函数表指针
};

// 电机3速度控制器实例
PID_Controller_t pid_speed_motor3 = {
    .Kp = 200.0f, // 比例增益
    .Ki = 0.0f, // 积分增益
    .Kd = 0.0f, // 微分增益
    .integral = 0, // 初始积分值
    .max_integral = 0, // 积分限幅，防止积分过大导致系统不稳定
    .previous_error = 0, // 初始上次误差值
    .output = 0, // 初始输出值
    .vtable = &pid_vtable // 虚函数表指针
};

// 电机4速度控制器实例
PID_Controller_t pid_speed_motor4 = {
    .Kp = 200.0f, // 比例增益
    .Ki = 0.0f, // 积分增益
    .Kd = 0.0f, // 微分增益
    .integral = 0, // 初始积分值
    .max_integral = 0, // 积分限幅，防止积分过大导致系统不稳定
    .previous_error = 0, // 初始上次误差值
    .output = 0, // 初始输出值
    .vtable = &pid_vtable // 虚函数表指针
};

// 机器人参数参数实例
Chassis_Param_t chassis_param = {
    .Lx = 0.160145f, // 轴距的一半（前后轮之间的距离的一半）（单位：米）(160.145mm)
    .Ly = 0.133735f, // 轮距的一半（左右轮之间的距离的一半）（单位：米）(133.735mm)
    .Wheel_R = 0.0767f // 轮半径（单位：米）（76.7mm）
};

// 机器人控制速度实例
Chassis_Velocity_t chassis_control_velocity = {
    .vx = 0, // 初始前后速度
    .vy = 0, // 初始左右速度
    .vw = 0 // 初始旋转速度
};

// 电机控制速度实例
Motor_Velocity_t motor_control_velocity = {
    .w_lf = 0, // 初始左前轮速度
    .w_lr = 0, // 初始左后轮速度
    .w_rr = 0, // 初始右后轮速度
    .w_rf = 0 // 初始右前轮速度
};

// 机器人观察速度实例
Chassis_Velocity_t chassis_observe_velocity = {
    .lpf1_vx = {
        .alpha = 0.05f, // 低通滤波器系数
        .input = 0, // 初始输入值
        .previous_output = 0, // 初始上一次输出值
        .output = 0, // 初始输出值
        .vtable = &lpf1_vtable // 虚函数表指针
    },
    .lpf1_vy = {
        .alpha = 0.05f, // 低通滤波器系数
        .input = 0, // 初始输入值
        .previous_output = 0, // 初始上一次输出值
        .output = 0, // 初始输出值
        .vtable = &lpf1_vtable // 虚函数表指针
    },
    .lpf1_vw = {
        .alpha = 0.05f, // 低通滤波器系数
        .input = 0, // 初始输入值
        .previous_output = 0, // 初始上一次输出值
        .output = 0, // 初始输出值
        .vtable = &lpf1_vtable // 虚函数表指针
    },
    .vx = 0, // 初始前后速度
    .vy = 0, // 初始左右速度
    .vw = 0 // 初始旋转速度
};

// 机器人观察速度实例（滤波后）
Chassis_Velocity_t chassis_observe_velocity_filtered = {
    .vx = 0, // 初始前后速度
    .vy = 0, // 初始左右速度
    .vw = 0 // 初始旋转速度
};

// 电机观察速度实例
Motor_Velocity_t motor_observe_velocity = {
    .w_lf = 0, // 初始左前轮速度
    .w_lr = 0, // 初始左后轮速度
    .w_rr = 0, // 初始右后轮速度
    .w_rf = 0 // 初始右前轮速度
};

// 机器人位置实例
Chassis_Position_t chassis_position = {
    .x = 0, // 初始横向位置（单位：米）
    .y = 0, // 初始纵向位置（单位：米）
    .theta = 0 // 初始旋转角度（单位：弧度）
};

uint16_t accel_pin = GPIO_PIN_9; // 加速度计片选引脚
uint16_t gyro_pin = GPIO_PIN_11; // 陀螺仪片选引脚

// BMI088设备实例
struct bmi08_dev bmi08_dev = {
    .accel_chip_id = BMI088_ACCEL_CHIP_ID,
    .gyro_chip_id = BMI08_GYRO_CHIP_ID,

    // 加速度计配置
    .accel_cfg.power = BMI08_ACCEL_PM_ACTIVE,
    .accel_cfg.range = BMI088_ACCEL_RANGE_3G,
    .accel_cfg.bw = BMI08_ACCEL_BW_OSR4,
    .accel_cfg.odr = BMI08_ACCEL_ODR_1600_HZ,

    // 陀螺仪配置
    .gyro_cfg.power = BMI08_GYRO_PM_NORMAL,
    .gyro_cfg.range = BMI08_GYRO_RANGE_500_DPS,
    .gyro_cfg.bw = BMI08_GYRO_BW_116_ODR_1000_HZ,
    .gyro_cfg.odr = BMI08_GYRO_BW_116_ODR_1000_HZ,

    .intf_ptr_accel = &accel_pin,
    .intf_ptr_gyro = &gyro_pin,
    .intf = BMI08_SPI_INTF,
    .variant = BMI088_VARIANT,
    .read_write_len = 32,

    // SPI接口函数指针
    .read = bmi088_spi_read,
    .write = bmi088_spi_write,
    .delay_us = bmi088_delay_us,
};

// 陀螺仪数据实例
struct bmi08_sensor_data gyro_data = {
    .x = 0,
    .y = 0,
    .z = 0,
};

// 加速度计数据实例
struct bmi08_sensor_data accel_data = {
    .x = 0,
    .y = 0,
    .z = 0,
};

// 陀螺仪数据实例（弧度/秒）
struct bmi08_sensor_data_f gyro_rad_s = {
    .x = 0,
    .y = 0,
    .z = 0,
};

// 加速度计数据实例（m/s^2）
struct bmi08_sensor_data_f accel_ms2 = {
    .x = 0,
    .y = 0,
    .z = 0,
};

// 欧拉角实例
struct euler_angles euler_angles = {
    .pitch_rad = 0,
    .roll_rad = 0,
    .yaw_rad = 0,
    .pitch_deg = 0,
    .roll_deg = 0,
    .yaw_deg = 0,
};

// BMI088校准数据实例
struct imu_offset bmi088_offset = {
    .gyro_x = 0.0f,
    .gyro_y = 0.0f,
    .gyro_z = 0.0f,
    .accel_x = 0.0f,
    .accel_y = 0.0f,
    .accel_z = 0.0f,
    .is_calibrated = 0, // 初始未校准
};

// 去除重力加速度后的加速度计数据实例（m/s^2）
struct bmi08_sensor_data_f accel_ms2_body = {
    .x = 0,
    .y = 0,
    .z = 0,
};

// 重力加速度实例（m/s^2）
float gravity_accel = 0.0f;

