#include "can_driver.h"
#include <string.h>

// CAN接收队列
osMessageQueueId_t CAN_RxQueueHandle;



/**
 * @brief CAN驱动初始化
 */
void CAN_Driver_Init(void)
{
    // 创建CAN接收队列
    CAN_RxQueueHandle = osMessageQueueNew(32, sizeof(CAN_Message_t), NULL);

    // 启动CAN
    HAL_CAN_Start(&hcan);

    // 激活CAN RX中断
    HAL_CAN_ActivateNotification(&hcan, CAN_IT_RX_FIFO0_MSG_PENDING);
    HAL_CAN_ActivateNotification(&hcan, CAN_IT_RX_FIFO1_MSG_PENDING);
    HAL_CAN_ActivateNotification(&hcan, CAN_IT_TX_MAILBOX_EMPTY);
}

/**
 * @brief 发送CAN消息
 */
uint8_t CAN_Driver_Send(uint32_t id, uint8_t* data, uint8_t len)
{
    CAN_TxHeaderTypeDef tx_header;
    uint32_t tx_mailbox;

    // 配置发送头
    tx_header.StdId = id;
    tx_header.ExtId = 0;
    tx_header.IDE = CAN_ID_STD;
    tx_header.RTR = CAN_RTR_DATA;
    tx_header.DLC = len;
    tx_header.TransmitGlobalTime = DISABLE;

    // 发送消息
    if (HAL_CAN_AddTxMessage(&hcan, &tx_header, data, &tx_mailbox) != HAL_OK) {
        return 0;
    }

    return 1;
}

/**
 * @brief 发送电机控制指令
 */
uint8_t CAN_Driver_Send_Motor_Ctrl(uint8_t motor_id, uint8_t func, uint8_t* data, uint8_t len)
{
    uint32_t can_id = CAN_ID_MOTOR_BASE | (motor_id << 4) | func;
    return CAN_Driver_Send(can_id, data, len);
}

/**
 * @brief 发送传感器读取指令
 */
uint8_t CAN_Driver_Send_Sensor_Read(uint8_t sensor_id, uint8_t func)
{
    uint32_t can_id = CAN_ID_SENSOR_BASE | (sensor_id << 4) | func;
    uint8_t dummy_data[1] = {0};
    return CAN_Driver_Send(can_id, dummy_data, 1);
}

/**
 * @brief CAN接收中断回调
 */
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
    CAN_RxHeaderTypeDef rx_header;
    CAN_Message_t msg;

    // 读取消息
    if (HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &rx_header, msg.data) == HAL_OK) {
        msg.id = rx_header.StdId;
        msg.len = rx_header.DLC;
        msg.format = (rx_header.IDE == CAN_ID_STD) ? 0 : 1;
        msg.type = (rx_header.RTR == CAN_RTR_DATA) ? 0 : 1;

        // 放入接收队列
        osMessageQueuePut(CAN_RxQueueHandle, &msg, 0, 0);
    }
}

/**
 * @brief 从队列接收CAN消息
 */
uint8_t CAN_Driver_Receive(CAN_Message_t* msg)
{
    if (osMessageQueueGet(CAN_RxQueueHandle, msg, NULL, 0) == osOK) {
        return 1;
    }
    return 0;
}