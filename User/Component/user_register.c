#include "user_register.h"
#include "bsp_instance.h"
#include "router.h"
#include "device_dr16.h"

void User_Callback_Register(void)
{
    // 注册电机数据路由函数，将其作为CAN接收回调函数
    BSP_CAN_RegisterRxCallback(&can_bus_1, Motor_Data_Router);
    BSP_UART_RegisterRxCallback(&uart_bus_6, DR16_Data_Router); // 如果需要注册UART接收回调函数，可以在这里进行注册
    //BSP_UART_RegisterRxCallback(&uart_bus_2, DR16_Data_Router); // 如果需要注册UART接收回调函数，可以在这里进行注册
}