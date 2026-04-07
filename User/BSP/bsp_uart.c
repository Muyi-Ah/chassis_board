#include "bsp_uart.h"
#include "stm32f4xx_hal_uart.h"
#include "fatal_error.h"

void UART_Start(UARTBus_t *self)
{
    // 地址检查
    CHECK_PTR(self);
    CHECK_PTR(self->huart);
    
    // 启动UART接收中断
    if (HAL_UARTEx_ReceiveToIdle_DMA(self->huart, self->rx_buffer, sizeof(self->rx_buffer)) != HAL_OK)
    {
        // 启动接收中断失败，进行错误处理
        INIT_ERROR();
    }
}

void BSP_UART_RegisterRxCallback(UARTBus_t *self, void (*rx_callback)(UARTBus_t *uart_bus, uint8_t *data, uint16_t length))
{
    // 地址检查
    CHECK_PTR(self);
    
    // 注册接收回调函数
    self->vptr->rx_callback = rx_callback;
}

UARTBus_Id_t BSP_UART_GetId(UARTBus_t *self)
{
    CHECK_PTR(self);
    return self->id;
}

const char *BSP_UART_GetName(UARTBus_t *self)
{
    CHECK_PTR(self);
    return self->name;
}

void uart_send(UARTBus_t *self, uint8_t *data, uint16_t length)
{
    // 地址检查
    CHECK_PTR(self);
    CHECK_PTR(data);
    
    // 发送数据
    if (HAL_UART_Transmit_DMA(self->huart, data, length) != HAL_OK)
    {
        // 发送数据失败，进行错误处理
        INIT_ERROR();
    }
}
