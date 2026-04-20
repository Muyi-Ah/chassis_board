#ifndef __DEVICE_INSTANCE_H__
#define __DEVICE_INSTANCE_H__

#include "device_motor.h"
#include "device_dr16.h"
#include "control_pid.h"
#include "chassis.h"

extern Motor_t motor1; // 声明电机1实例对象
extern Motor_t motor2; // 声明电机2实例对象
extern Motor_t motor3; // 声明电机3实例对象
extern Motor_t motor4; // 声明电机4实例对象

extern DR16_Data_t dr16_data; // 声明DR16实例对象

extern PID_Controller_t pid_speed_motor1; // 声明电机1速度PID控制器实例对象
extern PID_Controller_t pid_speed_motor2; // 声明电机2速度PID控制器实例对象
extern PID_Controller_t pid_speed_motor3; // 声明电机3速度PID控制器实例对象
extern PID_Controller_t pid_speed_motor4; // 声明电机4速度PID控制器实例对象

extern Chassis_Param_t chassis_param; // 声明底盘参数实例对象
extern Chassis_Velocity_t chassis_control_velocity; // 声明底盘控制速度实例对象
extern Motor_Velocity_t motor_control_velocity; // 声明电机控制速度实例对象
extern Chassis_Velocity_t chassis_observe_velocity; // 声明底盘观测速度实例对象
extern Motor_Velocity_t motor_observe_velocity; // 声明电机观测速度实例对象
extern Chassis_Velocity_t chassis_observe_velocity_filtered; // 声明底盘观测速度实例对象，滤波后结果
extern Chassis_Position_t chassis_position; // 声明底盘位置实例对象

extern struct bmi08_dev bmi08_dev; // 声明BMI088设备实例对象

extern struct bmi08_sensor_data gyro_data; // 声明陀螺仪数据实例对象
extern struct bmi08_sensor_data accel_data; // 声明加速度数据实例对象

extern struct bmi08_sensor_data_f gyro_rad_s; // 声明陀螺仪数据实例（弧度/秒）
extern struct bmi08_sensor_data_f accel_ms2; // 声明加速度计数据实例（m/s^2）

extern struct euler_angles euler_angles; // 声明欧拉角实例对象

extern struct imu_offset bmi088_offset; // IMU 零偏配置实例

extern struct bmi08_sensor_data_f accel_ms2_body; // 去除重力加速度后的加速度计数据实例（m/s^2）

extern float gravity_accel; // 重力加速度实例（m/s^2）

extern PID_Controller_t pid_position_x_world; // 世界坐标系下x轴位置PID控制器实例对象
extern PID_Controller_t pid_position_y_world; // 世界坐标系下y轴位置PID控制器实例对象
extern PID_Controller_t pid_position_theta_world; // 世界坐标系下theta轴位置PID控制器实例对象

extern Chassis_Position_t chassis_target_position; // 世界坐标系下的目标位置实例对象


#endif /* __DEVICE_INSTANCE_H__ */
