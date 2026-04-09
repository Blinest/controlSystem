#include "can_driver.h"
#include <string.h>
#include "usart.h"
#include "Motor/Motor.h"
#include "Motor/Emm42_command.h"
// CAN接收队列
osMessageQueueId_t CAN_RxQueueHandle;
CAN_TxHeaderTypeDef TxHeader;
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

void CAN_SendCmd(CAN_HandleTypeDef *hcan, uint8_t *cmd, uint8_t len)
{
	if (cmd == NULL || len < 2) return;

	uint8_t addr = cmd[0];          // 设备地址
	uint8_t func = cmd[1];          // 功能码
	uint8_t data_len = len - 2;     // 实际数据长度（不含地址和功能码）
	uint8_t *data = &cmd[2];        // 数据起始指针

	CAN_TxHeaderTypeDef TxHeader;
	TxHeader.IDE = CAN_ID_EXT;
	TxHeader.RTR = CAN_RTR_DATA;
	TxHeader.TransmitGlobalTime = DISABLE;

	uint8_t offset = 0;
	uint8_t pack_idx = 0;
	uint8_t tx_data[8];

	while (offset < data_len)
	{
		uint8_t remain = data_len - offset;
		uint8_t send_data_len = (remain > 7) ? 7 : remain;   // 每帧最多7字节数据
		TxHeader.DLC = 1 + send_data_len;                    // 功能码 + 数据
		TxHeader.ExtId = ((uint32_t)addr << 8) | pack_idx;   // ID = (地址 << 8) | 包序号

		tx_data[0] = func;   // 每帧第一字节固定为功能码
		for (uint8_t i = 0; i < send_data_len; i++) {
			tx_data[1 + i] = data[offset + i];
		}

		uint32_t TxMailbox;
		if (HAL_CAN_AddTxMessage(hcan, &TxHeader, tx_data, &TxMailbox) != HAL_OK) {
			return;   // 发送失败（例如邮箱满）
		}

		offset += send_data_len;
		pack_idx++;
	}
}


/**
	* @brief   CAN1_RX0接收中断
	* @param   无
	* @retval  无
	*/
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
	CAN_RxHeaderTypeDef RxHeader;
	uint8_t rx_data[8];

	if (HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &RxHeader, rx_data) == HAL_OK)
	{
		for (int i = 0; i < MOTOR_NUM; i++)
		{

			if (global_motor[i].id == (RxHeader.ExtId >> 8))
			{
				emm42_parse_can(&global_motor[i], RxHeader.ExtId, rx_data, RxHeader.DLC, true);
				break;
			}
		}
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



