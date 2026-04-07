#ifndef __BSP_INSTANCE_H__
#define __BSP_INSTANCE_H__

#include "bsp_uart.h"
#include "bsp_can.h"

extern CANBus_t can_bus_1; // 声明CAN1总线实例对象
extern UARTBus_t uart_bus_6; // 声明UART6总线实例对象
extern UARTBus_t uart_bus_2; // 声明UART2总线实例对象

#endif /* __BSP_INSTANCE_H__ */
