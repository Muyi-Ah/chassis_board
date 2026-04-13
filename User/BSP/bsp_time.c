#include "bsp_time.h"
#include "core_cm4.h"
#include <stdint.h>

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

/*!
 * @brief 初始化 DWT 外设
 * @note 该函数使能 DWT 外设并清零 CYCCNT 计数器
 */
void DWT_Init(void)
{
    // 使能 DWT 外设（在 DEMCR 寄存器中设置 TRCENA 位）
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    
    // 清零 CYCCNT 计数器
    DWT->CYCCNT = 0;

    // 使能 CYCCNT 计数器
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
}

extern uint32_t SystemCoreClock;

/*!
 * @brief 延时函数
 * @param us 延时时间，单位为微秒
 * @note 该函数使用 DWT 外设进行延时精度
 */
void Delay_us(uint32_t us)
{
    // 计算每个微秒的时钟周期数
    uint32_t ticks_per_us = SystemCoreClock / 1000000;

    // 计算延时的时钟周期数
    uint32_t delay_ticks = us * ticks_per_us;

    // 记录当前时钟周期数
    uint32_t start_ticks = DWT->CYCCNT;

    // 等待延时时间到
    while (DWT->CYCCNT - start_ticks < delay_ticks);
}
