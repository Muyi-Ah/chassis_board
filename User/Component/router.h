#ifndef __ROUTER_H__
#define __ROUTER_H__

#include "bsp_can.h"
#include "bsp_uart.h"
#include <stdint.h>

void Motor_Data_Router(CANBus_t *can_bus, uint32_t std_id, uint8_t *data); // 电机数据路由函数声明
void DR16_Data_Router(UARTBus_t *uart_bus, uint8_t *data, uint16_t length); // DR16数据路由函数声明

void Motor_Group_Send_Currents(CANBus_t *can_bus, int16_t current1, int16_t current2, int16_t current3, int16_t current4); // 电机组电流数据打包发送函数声明

#endif /* __ROUTER_H__ */
