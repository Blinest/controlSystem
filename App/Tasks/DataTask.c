/**
 * @file DataTask.c
 * @brief 数据任务：负责 USART1 RX (外设反馈) 和 USART2 TX (发送至 PC)
 *
 * 任务流程：
 * 1. 监听 CmdDataQueue (USART1 RX)，解析外设反馈并更新全局状态。
 * 2. 将系统状态打包并通过 SensorMessageQueue 缓冲。
 * 3. 轮询 SensorMessageQueue，将反馈字节发送至 USART2 TX (上位机)。
 * 
 * 简化版本：将复杂逻辑移到 DataTaskUtils 库中
 * 
 * @date 2026-03-30
 * @author Psyduck
 */
#include <stdio.h>
#include "cmsis_os.h"
#include "FreeRTOS.h"
#include "task.h"
#include "usart.h"
#include "Motor/Motor.h"
#include "Sensor/Sensor.h"
#include "Motor/Emm42_command.h"
#include "Common/periph_cmd_parser.h"
#include "Common/cmd_parse_unified.h"
#include "Common/cmd_packer.h"
#include "CR/CR.h"



#define RX_BUF_SIZE 256



void StartDataTask(void *argument)
{
    uint8_t rx_byte = 0;
    uint8_t tx_byte = 0;
    
    for(;;)
    {
        // ====================================
        // 1. 数据采集流: 从外设 (CAN 或 Usart RX) 接收并处理
        // ====================================
        if (osMessageQueueGet(CmdDataQueueHandle, &rx_byte, NULL, 0) == osOK)
        {
            // 调用新的 Emm42 协议解析函数
        	emm42_parse_feed_byte(rx_byte);
        	sensor_data_parser_feed_byte(rx_byte);
        }
        
        // ====================================
        // 2. 数据发送流: 打包数据发送给上位机
        // ====================================
        // 检查是否有数据需要发送
        static uint32_t last_send_time = 0;
        uint32_t current_time = osKernelGetTickCount();
        
        // 每100ms发送一次数据到队列 SensorMessageQueue
        if ((current_time - last_send_time) >= 100)
        {
            // 打包系统状态数据 (使用 static 以节省堆栈空间)
            static uint8_t packed_frame[128];
        	const float scale = lqts.operation_space.scale;
        	const uint8_t state = lqts.state;
            
            const uint16_t frame_len = cmd_packer_pack_status_frame(packed_frame, global_motor, global_sensor, &lqts, state);
            
            // 发送到队列
            for (int i = 0; i < frame_len; i++)
            {
                osMessageQueuePut(SensorMessageQueueHandle, &packed_frame[i], 0, 0);
            }
            last_send_time = current_time;
        }

    	// 批量发送数据
    	static uint8_t tx_buffer[256];
    	static uint16_t tx_buffer_len = 0;

    	// 提取并发送
    	tx_buffer_len = 0;
    	while (osMessageQueueGet(SensorMessageQueueHandle, &tx_byte, NULL, 0) == osOK && tx_buffer_len < 256)
    	{
    		tx_buffer[tx_buffer_len++] = tx_byte;
    	}

    	if (tx_buffer_len > 0) {
    		Usart_SendString(&huart2, tx_buffer, tx_buffer_len);
    	}
        osDelay(10); // 增加延时，降低 CPU 占用并给串口发送留出时间
    }
}

