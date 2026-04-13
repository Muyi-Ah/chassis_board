#include "device_dr16.h"
#include "fatal_error.h"

/**
 * @brief 更新DR16数据
 * @param dr16 DR16数据结构体指针
 * @param data 从遥控器接收到的数据
 */
void DR16_DataUpdate(DR16_Data_t *dr16, uint8_t *data)
{
    // 地址检查
    CHECK_PTR(dr16);
    CHECK_PTR(data);
    
    // 更新DR16数据结构体成员变量
    dr16->ch0 = (data[0] | (data[1] << 8)) & 0x07FF; // 通道0数据，11位有效
    dr16->ch1 = ((data[1] >> 3) | (data[2] << 5)) & 0x07FF; // 通道1数据，11位有效
    dr16->ch2 = ((data[2] >> 6) | (data[3] << 2) | (data[4] << 10)) & 0x07FF; // 通道2数据，11位有效
    dr16->ch3 = ((data[4] >> 1) | (data[5] << 7)) & 0x07FF; // 通道3数据，11位有效
    dr16->s1 = ((data[5] >> 4) & 0x000C) >> 2; // 开关1状态，2位有效
    dr16->s2 = ((data[5] >> 4) & 0x0003); // 开关2状态，2位有效

    dr16->ch0 -= 1024;
    dr16->ch1 -= 1024;
    dr16->ch2 -= 1024;
    dr16->ch3 -= 1024;
}

/**************** DR16数据获取函数 ****************/
uint16_t DR16_GetCh0(DR16_Data_t *dr16)
{
    CHECK_PTR(dr16);
    return dr16->ch0;
}

uint16_t DR16_GetCh1(DR16_Data_t *dr16)
{
    CHECK_PTR(dr16);
    return dr16->ch1;
}

uint16_t DR16_GetCh2(DR16_Data_t *dr16)
{
    CHECK_PTR(dr16);
    return dr16->ch2;
}

uint16_t DR16_GetCh3(DR16_Data_t *dr16)
{
    CHECK_PTR(dr16);
    return dr16->ch3;
}

uint8_t DR16_GetS1(DR16_Data_t *dr16)
{
    CHECK_PTR(dr16);
    return dr16->s1;
}

uint8_t DR16_GetS2(DR16_Data_t *dr16)
{
    CHECK_PTR(dr16);
    return dr16->s2;
}
