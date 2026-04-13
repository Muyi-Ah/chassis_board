#ifndef __DEVICE_MOTOR_H__
#define __DEVICE_MOTOR_H__

#include <stdint.h>
#include "filter_lpf1.h"

// 电机类型枚举
typedef enum
{
    M3508 = 0,
    GM6020
} Motor_Type;

typedef struct Motor_vtable_struct Motor_vtable_t; // 前向声明电机虚函数表结构体

/**
 * @brief 电机对象结构体
 */
typedef struct
{
    Motor_vtable_t *vptr; // 虚函数表指针

    LPF1_t lpf1_rpm; // 电机速度低通滤波器

    Motor_Type type; // 电机类型
    uint32_t id; // 电机ID
    int16_t encoder_value; // 电机编码器值
    int16_t RPM; // 电机速度
    int16_t filtered_RPM; // 电机速度经过滤波后的值
    float rad_per_sec; // 电机速度（弧度/秒）
    int16_t current; // 电机电流
    uint8_t temperature; // 电机温度

    uint32_t last_update_timestamp_ms; // 上次更新数据的时间戳，可以用于判断数据是否过期

} Motor_t;

/**
 * @brief 电机虚函数表结构体
 */
struct Motor_vtable_struct
{
    void (*data_update)(Motor_t *motor, uint8_t *data); // 电机数据更新的函数指针
    void (*set_speed_rpm)(Motor_t *motor, int16_t speed); // 设置电机速度的函数指针
    void (*set_speed_rad_per_sec)(Motor_t *motor, float speed); // 设置电机速度的函数指针
    int16_t (*get_speed_rpm)(Motor_t *motor); // 获取电机每分钟转速的函数指针
    float (*get_speed_rad_per_sec)(Motor_t *motor); // 获取电机每秒转速的函数指针
    int16_t (*get_encoder_value)(Motor_t *motor); // 获取电机编码器值的函数指针
    int16_t (*get_current)(Motor_t *motor); // 获取电机电流的函数指针
    uint8_t (*get_temperature)(Motor_t *motor); // 获取电机温度的函数指针
    int16_t (*get_rpm_value_filtered)(Motor_t *motor); // 获取电机速度经过滤波后的函数指针
};

// 电机相关函数声明
void Motor_DataUpdate(Motor_t *motor, uint8_t *data); // 电机数据更新函数声明
void Motor_SetSpeedRPM(Motor_t *motor, int16_t speed); // 设置电机速度函数声明
void Motor_SetSpeedRadPerSec(Motor_t *motor, float speed); // 设置电机速度函数声明
int16_t Motor_GetSpeedRPM(Motor_t *motor); // 获取电机每分钟转速函数声明
float Motor_GetSpeedRadPerSec(Motor_t *motor); // 获取电机每秒转速函数声明
int16_t Motor_GetEncoderValue(Motor_t *motor); // 获取电机编码器值函数声明
int16_t Motor_GetCurrent(Motor_t *motor); // 获取电机电流函数声明
uint8_t Motor_GetTemperature(Motor_t *motor); // 获取电机温度函数声明
int16_t Motor_GetRPMFiltered(Motor_t *motor); // 获取电机速度经过滤波后的函数声明
#endif /* __DEVICE_MOTOR_H__ */