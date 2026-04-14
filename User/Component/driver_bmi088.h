#ifndef __DRIVER_BMI088_H__
#define __DRIVER_BMI088_H__

#include "bmi08_defs.h"
#include <stdint.h>

struct euler_angles
{
    float pitch_rad;    // 弧度 (rad)
    float roll_rad;     // 弧度 (rad)
    float yaw_rad;      // 弧度 (rad)
    
    float pitch_deg; // 角度 (degree)
    float roll_deg;  // 角度 (degree)
    float yaw_deg;   // 角度 (degree)
};

// 标准重力加速度常量 (m/s^2)
#define STANDARD_GRAVITY_MS2 9.80665f

// IMU 零偏结构体
struct imu_offset
{
    float gyro_x;
    float gyro_y;
    float gyro_z;
    float accel_x;
    float accel_y;
    float accel_z;
    uint8_t is_calibrated; // 是否已经完成校准 (0: 否, 1: 是)
};

void driver_bmi088_init(void);
BMI08_INTF_RET_TYPE bmi088_spi_read(uint8_t reg_addr, uint8_t *reg_data, uint32_t len, void *intf_ptr);
BMI08_INTF_RET_TYPE bmi088_spi_write(uint8_t reg_addr, const uint8_t *reg_data, uint32_t len, void *intf_ptr);
void bmi088_delay_us(uint32_t period, void *intf_ptr);

// 数据转换函数声明
void driver_bmi088_gyro_to_rad_s(struct bmi08_sensor_data *raw_gyro, struct bmi08_sensor_data_f *out_gyro_rad, struct imu_offset *offset);
void driver_bmi088_accel_to_ms2(struct bmi08_sensor_data *raw_accel, struct bmi08_sensor_data_f *out_accel_ms2, struct imu_offset *offset);

// 四元数转欧拉角声明
void driver_bmi088_quaternion_to_euler(float q0, float q1, float q2, float q3, struct euler_angles *euler);

// 去除重力加速度声明
void driver_bmi088_body_gravity(struct bmi08_sensor_data_f *accel_ms2, struct bmi08_sensor_data_f *accel_ms2_body, struct euler_angles *euler, float gravity_accel);

#endif // __DRIVER_BMI088_H__
