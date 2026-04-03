#ifndef __BSP_CAN_H__
#define __BSP_CAN_H__

#include "stm32f4xx_hal.h"
#include <stdint.h>

typedef struct CANBus_struct CANBus_t; // 前向声明CAN总线对象结构体

typedef enum
{
    CAN_BUS_ID_1 = 1U
} CANBus_Id_t;

/**
 * @brief CAN总线虚函数表结构体
 */
typedef struct
{
    void (*start_recv)(CANBus_t *can_bus); // 启动接收函数指针
    void (*rx_callback)(CANBus_t *can_bus, uint32_t std_id, uint8_t *data); // 接收回调函数指针
} CANBus_vtable_t;

/**
 * @brief CAN总线对象结构体
 */
struct CANBus_struct
{
    CANBus_vtable_t *vptr; // 虚函数表指针
    CAN_HandleTypeDef *hcan; // CAN句柄
    CAN_FilterTypeDef *filter_config_ptr; // CAN过滤器配置
    CANBus_Id_t id;
    const char *name;

};

// CAN总线相关函数声明
void CAN_Start(CANBus_t *self);
void BSP_CAN_RegisterRxCallback(CANBus_t *self, void (*rx_callback)(CANBus_t *can_bus, uint32_t std_id, uint8_t *data));
CANBus_Id_t BSP_CAN_GetId(CANBus_t *self);
const char *BSP_CAN_GetName(CANBus_t *self);

#endif /* __BSP_CAN_H__ */
