#ifndef __DEVICE_INSTANCE_H__
#define __DEVICE_INSTANCE_H__

#include "device_motor.h"
#include "device_dr16.h"

extern Motor_t motor1; // 声明电机1实例对象
extern Motor_t motor2; // 声明电机2实例对象
extern Motor_t motor3; // 声明电机3实例对象
extern Motor_t motor4; // 声明电机4实例对象

extern DR16_Data_t dr16_data; // 声明DR16实例对象

#endif /* __DEVICE_INSTANCE_H__ */
