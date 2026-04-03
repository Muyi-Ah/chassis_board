#include "init.h"
#include "bsp_instance.h"

void System_Init(void)
{
    // 在这里可以添加系统初始化的代码，例如初始化外设、配置系统时钟等

    // 注册用户回调函数，将其作为CAN接收回调函数
    User_Callback_Register();

    if(can_bus1.vptr->start_recv != NULL)
    {
        can_bus1.vptr->start_recv(&can_bus1); // 启动CAN总线接收
    }
    if(uart_bus1.vptr->start_recv != NULL)
    {
        uart_bus1.vptr->start_recv(&uart_bus1); // 启动UART总线接收
    }
}