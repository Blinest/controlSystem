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
#include "cmsis_os.h"
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
        // 0. 数据回传流: 发送数据到 PC (USART2 TX)
        // ====================================
        while (osMessageQueueGet(SensorMessageQueueHandle, &tx_byte, NULL, 0) == osOK)
        {
            Usart_SendString(&huart2, &tx_byte, 1);
        }

        // ====================================
        // 1. 数据采集流: 从外设 (USART1 RX) 接收并处理
        // ====================================
        if (osMessageQueueGet(CmdDataQueueHandle, &rx_byte, NULL, 0) == osOK)
        {
            // 调用新的 Emm42 协议解析函数
            // 这个函数会解析字节流，更新 motor 全局结构体，并触发数据上报
        	emm42_parse_feed_byte(rx_byte);
        	sensor_data_parser_feed_byte(rx_byte);
        }
        
        // ====================================
        // 2. 数据发送流: 打包数据发送给上位机
        // ====================================
        // 检查是否有数据需要发送
        static uint32_t last_send_time = 0;
        uint32_t current_time = osKernelGetTickCount();
        
        // 每100ms发送一次数据（可调整）
        if ((current_time - last_send_time) >= 100)
        {
            // 打包系统状态数据
            uint8_t packed_frame[64];
        	const float scale = lqts.operation_space.scale;
        	const uint8_t state = lqts.state;
            const uint16_t frame_len = cmd_pack_status_frame(packed_frame, global_motor, global_sensor, scale, state);
            
            // 发送到队列
            for (int i = 0; i < frame_len; i++)
            {
                osMessageQueuePut(SensorMessageQueueHandle, &packed_frame[i], 0, 0);
            }
            
            last_send_time = current_time;
        }
        
        osDelay(1); // 释放 CPU，防止忙等
    }
}

