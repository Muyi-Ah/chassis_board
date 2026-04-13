#include "init.h"
#include "bsp_instance.h"
#include "user_register.h"
#include "bsp_time.h"
#include "driver_bmi088.h"
#include <stddef.h>

void System_Init(void)
{
    // 在这里可以添加系统初始化的代码，例如初始化外设、配置系统时钟等

    // 初始化DWT外设用于us级延时
    DWT_Init();

    // 注册用户回调函数，将其作为CAN接收回调函数
    User_Callback_Register();

    // 初始化BMI088设备
    driver_bmi088_init();

    if(can_bus_1.vptr->start_recv != NULL)
    {
        can_bus_1.vptr->start_recv(&can_bus_1); // 启动CAN总线接收
    }
    if(uart_bus_6.vptr->start_recv != NULL)
    {
        uart_bus_6.vptr->start_recv(&uart_bus_6); // 启动UART总线接收
    }
    if(uart_bus_2.vptr->start_recv != NULL)
    {
        uart_bus_2.vptr->start_recv(&uart_bus_2); // 启动UART总线接收
    }
}