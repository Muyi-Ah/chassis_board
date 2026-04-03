#ifndef __BSP_TIME_H__
#define __BSP_TIME_H__

#include <stdint.h>
#include "stm32f4xx_hal.h"

uint32_t Get_Time_ms(void);
void Delay_ms(uint32_t ms);

#endif /* __BSP_TIME_H__ */