#ifndef __BSP_TIME_H__
#define __BSP_TIME_H__

#include <stdint.h>
#include "stm32f4xx_hal.h"

void DWT_Init(void);

uint32_t Get_Time_ms(void);
void Delay_ms(uint32_t ms);
void Delay_us(uint32_t us);

#endif /* __BSP_TIME_H__ */