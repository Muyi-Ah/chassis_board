#include "bsp_led.h"

#define LED_MAX_COUNT 3

// 定义LED数组，包含每个LED的GPIO端口和引脚
LED_t led[LED_MAX_COUNT] = {
    {GPIOE, GPIO_PIN_2},
    {GPIOE, GPIO_PIN_3},
    {GPIOE, GPIO_PIN_4},
};

/**
    * @brief 开启指定LED
    * @param index LED索引
    * @retval None
*/
void led_turn_on(uint8_t index)
{
    HAL_GPIO_WritePin(led[index].GPIOx, led[index].GPIO_Pin, GPIO_PIN_SET);
}

/**
    * @brief 关闭指定LED
    * @param index LED索引
    * @retval None
*/
void led_turn_off(uint8_t index)
{
    HAL_GPIO_WritePin(led[index].GPIOx, led[index].GPIO_Pin, GPIO_PIN_RESET);
}

/**
    * @brief 切换指定LED状态
    * @param index LED索引
    * @retval None
*/
void led_toggle(uint8_t index)
{
    HAL_GPIO_TogglePin(led[index].GPIOx, led[index].GPIO_Pin);
}

/**
    * @brief 开启所有LED
    * @retval None
*/
void led_full_on(void)
{
    for(uint8_t i = 0; i < LED_MAX_COUNT; i++)
    {
        HAL_GPIO_WritePin(led[i].GPIOx, led[i].GPIO_Pin, GPIO_PIN_SET);
    }
}

/**
    * @brief 关闭所有LED
    * @retval None
*/
void led_full_off(void)
{
    for(uint8_t i = 0; i < LED_MAX_COUNT; i++)
    {
        HAL_GPIO_WritePin(led[i].GPIOx, led[i].GPIO_Pin, GPIO_PIN_RESET);
    }   
}

/**
    * @brief 切换所有LED状态
    * @retval None
*/
void led_full_toggle(void)
{
    for(uint8_t i = 0; i < LED_MAX_COUNT; i++)
    {
        HAL_GPIO_TogglePin(led[i].GPIOx, led[i].GPIO_Pin);
    }   
}