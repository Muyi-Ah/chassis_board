#include "device_instance.h"
#include "device_dr16.h"

Motor_vtable_t motor_vtable = {
    .data_update = Motor_DataUpdate, // 电机数据更新函数指针
    .set_speed_rpm = Motor_SetSpeedRPM, // 设置电机速度函数指针
    .set_speed_rad_per_sec = Motor_SetSpeedRadPerSec, // 设置电机速度函数指针
    .get_speed_rpm = Motor_GetSpeedRPM, // 获取电机每分钟转速函数指针
    .get_speed_rad_per_sec = Motor_GetSpeedRadPerSec, // 获取电机每秒转速函数指针
    .get_encoder_value = Motor_GetEncoderValue, // 获取电机编码器值函数指针
    .get_current = Motor_GetCurrent // 获取电机电流函数指针
};

Motor_t motor1 = {
    .type = M3508, // 电机类型
    .id = 0x201, // 电机ID
    .encoder_value = 0, // 初始编码器值
    .RPM = 0, // 初始速度
    .current = 0, // 初始电流
    .temperature = 0, // 初始温度
    .vptr = &motor_vtable // 虚函数表指针
};

Motor_t motor2 = {
    .type = M3508, // 电机类型
    .id = 0x202, // 电机ID
    .encoder_value = 0, // 初始编码器值
    .RPM = 0, // 初始速度
    .current = 0, // 初始电流
    .temperature = 0, // 初始温度
    .vptr = &motor_vtable // 虚函数表指针
};

Motor_t motor3 = {
    .type = M3508, // 电机类型
    .id = 0x203, // 电机ID
    .encoder_value = 0, // 初始编码器值
    .RPM = 0, // 初始速度
    .current = 0, // 初始电流
    .temperature = 0, // 初始温度
    .vptr = &motor_vtable // 虚函数表指针
};

Motor_t motor4 = {
    .type = M3508, // 电机类型
    .id = 0x204, // 电机ID
    .encoder_value = 0, // 初始编码器值
    .RPM = 0, // 初始速度
    .current = 0, // 初始电流
    .temperature = 0, // 初始温度
    .vptr = &motor_vtable // 虚函数表指针
};

DR16_vtable_t dr16_vtable = {
    .data_update = DR16_DataUpdate, // DR16数据更新函数指针
    .get_ch0 = DR16_GetCh0, // 获取通道0数据函数指针
    .get_ch1 = DR16_GetCh1, // 获取通道1数据函数指针
    .get_ch2 = DR16_GetCh2, // 获取通道2数据函数指针
    .get_ch3 = DR16_GetCh3, // 获取通道3数据函数指针
    .get_s1 = DR16_GetS1, // 获取开关1状态函数指针
    .get_s2 = DR16_GetS2 // 获取开关2状态函数指针
};

DR16_Data_t dr16_data = {
    .ch0 = 0, // 初始通道0数据
    .ch1 = 0, // 初始通道1数据
    .ch2 = 0, // 初始通道2数据
    .ch3 = 0, // 初始通道3数据
    .s1 = 0, // 初始开关1状态
    .s2 = 0, // 初始开关2状态
    .vptr = &dr16_vtable // 虚函数表指针
};
