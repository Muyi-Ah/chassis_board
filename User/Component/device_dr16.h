#ifndef __DEVICE_DR16_H__
#define __DEVICE_DR16_H__

#include <stdint.h>

typedef struct DR16_vtable_struct DR16_vtable_t; // 前向声明DR16虚函数表结构体

typedef struct
{
    DR16_vtable_t *vptr; // 虚函数表指针

    uint16_t ch0; // 通道0
    uint16_t ch1; // 通道1
    uint16_t ch2; // 通道2
    uint16_t ch3; // 通道3
    uint8_t s1;   // 开关1
    uint8_t s2;   // 开关2
} DR16_Data_t;

struct DR16_vtable_struct
{
    void (*data_update)(DR16_Data_t *dr16, uint8_t *data); // 数据更新函数指针
    uint16_t (*get_ch0)(DR16_Data_t *dr16); // 获取通道0数据函数指针
    uint16_t (*get_ch1)(DR16_Data_t *dr16); // 获取通道1数据函数指针
    uint16_t (*get_ch2)(DR16_Data_t *dr16); // 获取通道2数据函数指针
    uint16_t (*get_ch3)(DR16_Data_t *dr16); // 获取通道3数据函数指针
    uint8_t (*get_s1)(DR16_Data_t *dr16);   // 获取开关1状态函数指针
    uint8_t (*get_s2)(DR16_Data_t *dr16);   // 获取开关2状态函数指针
};

// DR16相关函数声明
void DR16_DataUpdate(DR16_Data_t *dr16, uint8_t *data);
uint16_t DR16_GetCh0(DR16_Data_t *dr16);
uint16_t DR16_GetCh1(DR16_Data_t *dr16);
uint16_t DR16_GetCh2(DR16_Data_t *dr16);
uint16_t DR16_GetCh3(DR16_Data_t *dr16);
uint8_t DR16_GetS1(DR16_Data_t *dr16);
uint8_t DR16_GetS2(DR16_Data_t *dr16);

#endif /* __DEVICE_DR16_H__ */
