#ifndef __BSP_UART_H__
#define __BSP_UART_H__

#include "stm32f4xx_hal.h"
#include <stdint.h>

#define MAX_UART_BUFF_SIZE 256

typedef struct UARTBus_struct UARTBus_t; // 前向声明UART总线对象结构体
typedef struct UARTBus_vtable_struct UARTBus_vtable_t; // 前向声明UART总线虚函数表结构体

typedef enum
{
    UART_BUS_ID_6 = 6U,
    UART_BUS_ID_2 = 2U
} UARTBus_Id_t;

struct UARTBus_struct
{
    UART_HandleTypeDef *huart;
    UARTBus_vtable_t *vptr; // 虚函数表指针
    uint8_t rx_buffer[MAX_UART_BUFF_SIZE]; // 接收缓冲区
    UARTBus_Id_t id;
    const char *name;
};

struct UARTBus_vtable_struct
{
    void (*start_recv)(UARTBus_t *uart_bus); // 启动接收函数指针
    void (*rx_callback)(UARTBus_t *uart_bus, uint8_t *data, uint16_t length); // 接收回调函数指针
    void (*send)(UARTBus_t *uart_bus, uint8_t *data, uint16_t length); // 发送函数指针
};

void UART_Start(UARTBus_t *self);
void BSP_UART_RegisterRxCallback(UARTBus_t *self, void (*rx_callback)(UARTBus_t *uart_bus, uint8_t *data, uint16_t length));
UARTBus_Id_t BSP_UART_GetId(UARTBus_t *self);
const char *BSP_UART_GetName(UARTBus_t *self);
void uart_send(UARTBus_t *self, uint8_t *data, uint16_t length);

#endif /* __BSP_UART_H__ */
