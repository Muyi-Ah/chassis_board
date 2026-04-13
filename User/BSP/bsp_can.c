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

/**
 * @brief 通过CAN总线发送标准帧数据
 * @param self CAN总线对象指针
 * @param std_id 标准帧ID
 * @param data 要发送的数据指针
 * @param len 数据长度
 * @return HAL_StatusTypeDef 发送状态
 */
HAL_StatusTypeDef BSP_CAN_Transmit(CANBus_t *self, uint32_t std_id, uint8_t *data, uint16_t len)
{
    CHECK_PTR(self);
    CHECK_PTR(data);
    
    CAN_TxHeaderTypeDef tx_header;
    uint32_t tx_mailbox;
    
    tx_header.StdId = std_id;
    tx_header.ExtId = 0;
    tx_header.IDE = CAN_ID_STD;
    tx_header.RTR = CAN_RTR_DATA;
    tx_header.DLC = len;
    tx_header.TransmitGlobalTime = DISABLE;
    
    // 查询是否有空闲的发送邮箱
    if (HAL_CAN_GetTxMailboxesFreeLevel(self->hcan) > 0)
    {
        return HAL_CAN_AddTxMessage(self->hcan, &tx_header, data, &tx_mailbox);
    }
    
    return HAL_BUSY;
}
