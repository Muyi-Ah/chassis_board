#ifndef __BSP_INSTANCE_H__
#define __BSP_INSTANCE_H__

#include "bsp_can.h"
#include "bsp_uart.h"

extern CANBus_t can_bus1; // 声明CAN1总线实例对象
extern UARTBus_t uart_bus1; // 声明UART1总线实例对象

#endif /* __BSP_INSTANCE_H__ */
