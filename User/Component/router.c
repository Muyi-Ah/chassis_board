#include "router.h"
#include "bsp_instance.h"
#include "device_instance.h"
#include "fatal_error.h"

static uint8_t Is_Motor_CANBus(CANBus_t *can_bus)
{
    return BSP_CAN_GetId(can_bus) == CAN_BUS_ID_1;
}

static uint8_t Is_DR16_UARTBus(UARTBus_t *uart_bus)
{
    return BSP_UART_GetId(uart_bus) == UART_BUS_ID_6;
}

/**
 * @brief 电机数据路由函数
 * @param std_id 标准ID
 * @param data 数据指针
 * @note 该函数根据接收到的CAN消息的标准ID，将数据路由到对应的电机对象进行更新
 */
void Motor_Data_Router(CANBus_t *can_bus, uint32_t std_id, uint8_t *data)
{
    const char *can_bus_name;

    CHECK_PTR(can_bus);
    CHECK_PTR(data);
    CHECK_PTR(BSP_CAN_GetName(can_bus));

    can_bus_name = BSP_CAN_GetName(can_bus);
    (void)can_bus_name;

    if (!Is_Motor_CANBus(can_bus))
    {
        return;
    }

    switch(std_id)
    {
        case 0x201: // 电机1数据
            if (motor1.vptr->data_update != NULL)
            {
                motor1.vptr->data_update(&motor1, data); // 传递接收到的数据给电机对象进行更新
            }
            break;
        case 0x202: // 电机2数据
            if (motor2.vptr->data_update != NULL)
            {
                motor2.vptr->data_update(&motor2, data); // 传递接收到的数据给电机对象进行更新
            }
            break;
        case 0x203: // 电机3数据
            if (motor3.vptr->data_update != NULL)
            {
                motor3.vptr->data_update(&motor3, data); // 传递接收到的数据给电机对象进行更新
            }
            break;
        case 0x204: // 电机4数据
            if (motor4.vptr->data_update != NULL)
            {
                motor4.vptr->data_update(&motor4, data); // 传递接收到的数据给电机对象进行更新
            }
            break;
        default:
            // 处理未知ID的情况，可以选择忽略或进行错误处理
            break;
    }
}

void DR16_Data_Router(UARTBus_t *uart_bus, uint8_t *data, uint16_t length)
{
    const char *uart_bus_name;

    CHECK_PTR(uart_bus);
    CHECK_PTR(data);
    CHECK_PTR(BSP_UART_GetName(uart_bus));

    uart_bus_name = BSP_UART_GetName(uart_bus);
    (void)uart_bus_name;

    if ((length >= 6U) && Is_DR16_UARTBus(uart_bus) && (dr16_data.vptr->data_update != NULL))
    {
        dr16_data.vptr->data_update(&dr16_data, data); // 传递接收到的数据给DR16对象进行更新
    }
}
