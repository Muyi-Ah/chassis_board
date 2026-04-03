#include "bsp_instance.h"
#include "fatal_error.h"

extern CAN_HandleTypeDef hcan1;
extern UART_HandleTypeDef huart6;

/**
 * @brief CAN1总线实例对象
 */
CANBus_t can_bus1 = {

    .hcan = &hcan1, // 初始化CAN句柄，可以根据需要进行配置
    .id = CAN_BUS_ID_1,
    .name = "can1",

    /**
     * @brief CAN1总线虚函数表
     */
    .vptr = &(CANBus_vtable_t){
        .start_recv = CAN_Start, // 这里可以赋值为实际的启动接收函数
        .rx_callback = NULL // 初始化接收回调函数指针为NULL，用户可以通过BSP_CAN_RegisterRxCallback函数注册实际的回调函数
    },

    /**
     * @brief CAN1总线过滤器配置
     */
    .filter_config_ptr = &(CAN_FilterTypeDef){
        .FilterIdHigh = 0x0000,
        .FilterIdLow = 0x0000,
        .FilterMaskIdHigh = 0x0000,
        .FilterMaskIdLow = 0x0000,
        .FilterFIFOAssignment = CAN_FILTER_FIFO0,
        .FilterBank = 0,
        .FilterMode = CAN_FILTERMODE_IDMASK,
        .FilterScale = CAN_FILTERSCALE_32BIT,
        .FilterActivation = ENABLE
    }
};

static CANBus_t *can_bus_table[] = {
    &can_bus1
};

static CANBus_t *Find_CANBus(CAN_HandleTypeDef *hcan)
{
    uint32_t i;

    for (i = 0; i < (sizeof(can_bus_table) / sizeof(can_bus_table[0])); i++)
    {
        if (can_bus_table[i]->hcan == hcan)
        {
            return can_bus_table[i];
        }
    }

    return NULL;
}

static void CAN_RxCallback(CANBus_t *can_bus)
{
    CAN_RxHeaderTypeDef rx_header;
    uint8_t rx_data[8];

    CHECK_PTR(can_bus);
    CHECK_PTR(can_bus->hcan);

    if (HAL_CAN_GetRxMessage(can_bus->hcan, CAN_RX_FIFO0, &rx_header, rx_data) == HAL_OK)
    {
        CHECK_PTR(can_bus->vptr->rx_callback);
        can_bus->vptr->rx_callback(can_bus, rx_header.StdId, rx_data);
    }
}

void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
    CANBus_t *can_bus = Find_CANBus(hcan);

    if (can_bus != NULL)
    {
        CAN_RxCallback(can_bus);
    }
}

/**
 * @brief UART1总线实例对象
 */
UARTBus_t uart_bus1 = {
    .huart = &huart6, // 初始化UART句柄，可以根据需要进行配置
    .id = UART_BUS_ID_6,
    .name = "uart6",
    .vptr = &(UARTBus_vtable_t){
        .start_recv = UART_Start, // 这里可以赋值为实际的启动接收函数
        .rx_callback = NULL // 初始化接收回调函数指针为NULL，用户可以通过BSP_UART_RegisterRxCallback函数注册实际的回调函数
    }
};

static UARTBus_t *uart_bus_table[] = {
    &uart_bus1
};

static UARTBus_t *Find_UARTBus(UART_HandleTypeDef *huart)
{
    uint32_t i;

    for (i = 0; i < (sizeof(uart_bus_table) / sizeof(uart_bus_table[0])); i++)
    {
        if (uart_bus_table[i]->huart == huart)
        {
            return uart_bus_table[i];
        }
    }

    return NULL;
}

void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
{
    UARTBus_t *uart_bus = Find_UARTBus(huart);

    if (uart_bus != NULL)
    {
        CHECK_PTR(uart_bus->vptr->rx_callback);
        uart_bus->vptr->rx_callback(uart_bus, uart_bus->rx_buffer, Size);

        // 启动UART接收
        UART_Start(uart_bus);
    }
}
