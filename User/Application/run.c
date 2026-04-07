#include "run.h"
#include "bsp_led.h"
#include "bsp_time.h"
#include "FreeRTOS.h"
#include "cmsis_gcc.h"
#include "cmsis_os2.h"
#include "task.h"
#include "bsp_uart.h"
#include "bsp_instance.h"
#include "device_instance.h"
// 移除 stdio.h，使用自定义的快速转换函数以降低开销

// 快速整型转字符串辅助函数
static inline char* fast_itoa(char *str, int32_t val) {
    if (val == 0) {
        *str++ = '0';
        return str;
    }
    if (val < 0) {
        *str++ = '-';
        val = -val;
    }
    char temp[10]; 
    int i = 0;
    while (val > 0) {
        temp[i++] = (val % 10) + '0';
        val /= 10;
    }
    while (i > 0) {
        *str++ = temp[--i];
    }
    return str;
}

void runTask(void *argument)
{
    while (1)
    {
        led_full_toggle();
        osDelay(pdMS_TO_TICKS(500)); // 延时500毫秒
    }
}

void sendTask(void *argument)
{
    char buffer[128];
    while (1)
    {
        char *ptr = buffer;
        
        ptr = fast_itoa(ptr, motor1.RPM); *ptr++ = ',';
        ptr = fast_itoa(ptr, motor1.filtered_RPM); *ptr++ = ',';
        
        ptr = fast_itoa(ptr, motor2.RPM); *ptr++ = ',';
        ptr = fast_itoa(ptr, motor2.filtered_RPM); *ptr++ = ',';
        
        ptr = fast_itoa(ptr, motor3.RPM); *ptr++ = ',';
        ptr = fast_itoa(ptr, motor3.filtered_RPM); *ptr++ = ',';
        
        ptr = fast_itoa(ptr, motor4.RPM); *ptr++ = ',';
        ptr = fast_itoa(ptr, motor4.filtered_RPM);
        
        *ptr++ = '\r';
        *ptr++ = '\n';
        
        uint16_t len = (uint16_t)(ptr - buffer);
        uart_bus_2.vptr->send(&uart_bus_2, (uint8_t *)buffer, len); // 发送数据到UART2
        
        TickType_t current_tick = xTaskGetTickCount();
        osDelayUntil(current_tick + pdMS_TO_TICKS(1)); // 每1毫秒执行一次
    }
}
