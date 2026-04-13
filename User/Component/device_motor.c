#include "device_motor.h"
#include "bsp_time.h"

/**
 * @brief 电机数据更新函数
 * @param motor 电机对象指针
 * @param data 数据指针
 * @note 该函数用于更新电机的编码器值、速度、电流等信息
 */
void Motor_DataUpdate(Motor_t *motor, uint8_t *data)
{
    motor->encoder_value = (data[0] << 8) | data[1]; // 解析编码器值
    motor->RPM = (data[2] << 8) | data[3]; // 解析速度
    motor->current = (data[4] << 8) | data[5]; // 解析电流
    motor->temperature = data[6]; // 解析温度

    // 更新上次更新数据的时间戳
    motor->last_update_timestamp_ms = Get_Time_ms(); // 获取当前时间戳，单位为毫秒

    motor->filtered_RPM = motor->lpf1_rpm.vtable->compute(&motor->lpf1_rpm, motor->RPM); // 使用低通滤波器过滤速度
    motor->rad_per_sec = motor->filtered_RPM * 2 * 3.14159f / 60; // 将每分钟转速转换为每秒转速
}

/**
 * @brief 设置电机速度（单位：RPM）
 * @param motor 电机对象指针
 * @param speed 目标速度，单位为每分钟转速（RPM）
 * @note 该函数可以通过发送CAN消息来设置电机的目标速度
 */
void Motor_SetSpeedRPM(Motor_t *motor, int16_t speed)
{
    // 这里可以添加发送CAN消息的代码来设置电机速度
}

/**
 * @brief 设置电机速度（单位：rad/s）
 * @param motor 电机对象指针
 * @param speed 目标速度，单位为每秒转速（rad/s）
 * @note 该函数可以通过发送CAN消息来设置电机的目标速度
 */
void Motor_SetSpeedRadPerSec(Motor_t *motor, float speed)
{
    // 这里可以添加发送CAN消息的代码来设置电机速度
}

/**
 * @brief 获取电机每分钟转速
 * @param motor 电机对象指针
 * @return 电机每分钟转速（RPM）
 */
int16_t Motor_GetSpeedRPM(Motor_t *motor)
{
    return motor->RPM; // 返回电机每分钟转速
}

/**
 * @brief 获取电机每秒转速
 * @param motor 电机对象指针
 * @return 电机每秒转速（rad/s）
 */
float Motor_GetSpeedRadPerSec(Motor_t *motor)
{
    return motor->RPM * 2 * 3.14159f / 60; // 将每分钟转速转换为每秒转速
}

/**
 * @brief 获取电机编码器值
 * @param motor 电机对象指针
 * @return 电机编码器值
 */
int16_t Motor_GetEncoderValue(Motor_t *motor)
{
    return motor->encoder_value; // 返回电机编码器值
}

/**
 * @brief 获取电机电流
 * @param motor 电机对象指针
 * @return 电机电流
 */
int16_t Motor_GetCurrent(Motor_t *motor)
{
    return motor->current; // 返回电机电流
}

/**
 * @brief 获取电机温度
 * @param motor 电机对象指针
 * @return 电机温度
 */
uint8_t Motor_GetTemperature(Motor_t *motor)
{
    return motor->temperature; // 返回电机温度
}
