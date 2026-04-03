#ifndef __BSP_LED_H__
#define __BSP_LED_H__

#include "stm32f4xx_hal.h"

typedef struct
{
    GPIO_TypeDef* GPIOx;
    uint16_t GPIO_Pin;
} LED_t;

void led_turn_on(uint8_t index);
void led_turn_off(uint8_t index);
void led_toggle(uint8_t index);
void led_full_on(void);
void led_full_off(void);
void led_full_toggle(void);


#endif /* __BSP_LED_H__ */