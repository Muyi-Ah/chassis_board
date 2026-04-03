#include "bsp_time.h"

/**
 * @brief 获取系统时间
 * @return 当前系统时间，单位为毫秒
 * @note 该函数可以用于获取系统运行时间或进行时间戳记录等用途
 */
uint32_t Get_Time_ms(void)
{
    return HAL_GetTick(); // 返回毫秒级系统时间
}

/**
 * @brief 延时函数
 * @param ms 延时时间，单位为毫秒
 */
void Delay_ms(uint32_t ms)
{
    HAL_Delay(ms);
}