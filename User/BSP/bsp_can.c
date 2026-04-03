#include "bsp_can.h"
#include "stm32f4xx_hal_can.h"
#include "fatal_error.h"

/**
 * @brief 启动CAN总线
 * @param self CAN总线对象指针
 */
void CAN_Start(CANBus_t *self)
{
    // 地址检查
    CHECK_PTR(self);
    CHECK_PTR(self->hcan);
    
    // 配置CAN过滤器
    HAL_CAN_ConfigFilter(self->hcan, self->filter_config_ptr);

    if (HAL_CAN_Start(self->hcan) != HAL_OK)
    {
        // 启动CAN失败，进行错误处理
        INIT_ERROR();
    }

    // 启动CAN接收中断
    if (HAL_CAN_ActivateNotification(self->hcan, CAN_IT_RX_FIFO0_MSG_PENDING) != HAL_OK)
    {
        // 启动接收中断失败，进行错误处理
        INIT_ERROR();
    }
}

/**
 * @brief 注册CAN接收回调函数
 * @param self CAN总线对象指针
 * @param rx_callback 接收回调函数指针，参数为接收到的数据和数据长度
 */
void BSP_CAN_RegisterRxCallback(CANBus_t *self, void (*rx_callback)(CANBus_t *can_bus, uint32_t std_id, uint8_t *data))
{
    // 地址检查
    CHECK_PTR(self);
    
    // 注册接收回调函数
    self->vptr->rx_callback = rx_callback;
}

CANBus_Id_t BSP_CAN_GetId(CANBus_t *self)
{
    CHECK_PTR(self);
    return self->id;
}

const char *BSP_CAN_GetName(CANBus_t *self)
{
    CHECK_PTR(self);
    return self->name;
}
