#include "can_driver.h"
#include <string.h>
#include "usart.h"

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

	// 配置一个简单的过滤器（接收所有消息）
	CAN_FilterTypeDef filter = {0};
	filter.FilterBank = 0;
	filter.FilterMode = CAN_FILTERMODE_IDMASK;
	filter.FilterScale = CAN_FILTERSCALE_32BIT;
	filter.FilterIdHigh = 0;
	filter.FilterIdLow = 0;
	filter.FilterMaskIdHigh = 0;
	filter.FilterMaskIdLow = 0;
	filter.FilterFIFOAssignment = CAN_RX_FIFO0;
	filter.FilterActivation = ENABLE;
	HAL_CAN_ConfigFilter(&hcan, &filter);
    // 激活CAN RX中断
    HAL_CAN_ActivateNotification(&hcan, CAN_IT_RX_FIFO0_MSG_PENDING);
    HAL_CAN_ActivateNotification(&hcan, CAN_IT_RX_FIFO1_MSG_PENDING);
    HAL_CAN_ActivateNotification(&hcan, CAN_IT_TX_MAILBOX_EMPTY);
}

/**
	* @brief   CAN发送多个字节
	* @param   无
	* @retval  无
	*/

void CAN_SendCmd(CAN_HandleTypeDef *hcan, uint32_t can_id, uint8_t *data, uint8_t len)
{
	if (len == 0) return;
	uint8_t offset = 0;
	CAN_TxHeaderTypeDef TxHeader;
	uint32_t TxMailbox;

	TxHeader.StdId = can_id;
	TxHeader.IDE = CAN_ID_STD;
	TxHeader.RTR = CAN_RTR_DATA;
	TxHeader.TransmitGlobalTime = DISABLE;

	while (len > 0) {
		uint8_t chunk_len = (len > 8) ? 8 : len;
		TxHeader.DLC = chunk_len;
		if (HAL_CAN_AddTxMessage(hcan, &TxHeader, data + offset, &TxMailbox) != HAL_OK) {
			// 发送失败，可选择重试或退出
			break;
		}
		// 增加超时机制，避免永久等待
		uint32_t timeout = 1000; // 等待最大 1000 个循环
		while (HAL_CAN_GetTxMailboxesFreeLevel(hcan) == 0 && timeout--) {
			HAL_Delay(1);  // 让出 CPU，防止任务饿死
		}
		if (timeout == 0) {
			// 超时处理，例如清空邮箱或报错
			break;
		}
		offset += chunk_len;
		len -= chunk_len;
	}
}

uint8_t test[4] = "test";
/**
	* @brief   CAN1_RX0接收中断
	* @param   无
	* @retval  无
	*/
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
	CAN_RxHeaderTypeDef RxHeader;
	uint8_t rx_data[8];
	// 接收一包数据
	if (HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &RxHeader, rx_data) == HAL_OK)
	{
		osMessageQueuePut(CmdDataQueueHandle, &rx_data, 0, 0);
		//Usart_SendString(&huart2, test, 4);
	};
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



