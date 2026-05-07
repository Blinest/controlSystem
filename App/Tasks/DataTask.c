/**
 * @file DataTask.c
 * @brief 数据任务：负责 USART1 RX (传感器反馈) 和 USART2 TX (发送至 PC)
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
#include "Common/XV2_cmd_parser.h"
#include "Common/cmd_packer.h"

#include "CR/CR.h"
#include "Sensor/WT_IMU.h"
#include "Sensor/IMU.h"




#define RX_BUF_SIZE 256

void StartDataTask(void *argument)
{
    uint8_t rx_byte = 0;
    uint8_t tx_byte = 0;
	MotorContext *active_motor_ctx = NULL;   // 当前激活的电机上下文

	// 初始化解析器
    for(;;)
    {
        // ====================================
        // 1. 数据采集流: 从队列获取字节并分发到解析器
        // ====================================
       while (osMessageQueueGet(CmdDataQueueHandle, &rx_byte, NULL, 0) == osOK)
        {
       		// 电机指令解析函数
       		if (active_motor_ctx == NULL) {
       			// 当前无活动帧，尝试将字节解释为地址
       			MotorContext *ctx = Motor_GetContextByAddr(rx_byte);
       			if (ctx != NULL) {
       				active_motor_ctx = ctx;
       				X_V2_SerialParser_Reset(&active_motor_ctx->parser); // 新帧开始
       			}
       			// 如果 ctx == NULL，该字节既非传感器 ID 也非电机地址，丢弃
       		}
       		// 如果当前有活动电机帧，则将字节喂入
       		if (active_motor_ctx != NULL) {
       			X_V2_ParseResult res = X_V2_SerialParser_Feed(
					   &active_motor_ctx->parser,
					   rx_byte,
					   &active_motor_ctx->global_motor,
					   true
				   );
       			// 根据解析结果处理
       			if (res == X_V2_PARSE_OK) {
       				// 帧完成，电机数据已更新
       				active_motor_ctx = NULL;   // 帧结束，释放上下文
       			} else if (res != X_V2_PARSE_INCOMPLETE) {
       				// 错误（地址不匹配、校验错、长度溢出等）
       				active_motor_ctx = NULL;
       			}
       		}
        }

        // ====================================
        // 2. 数据发送流: 打包数据发送给上位机
        // ====================================
        // 检查是否有数据需要发送
        static uint32_t last_send_time = 0;
        uint32_t current_time = osKernelGetTickCount();
        
        // 每200ms发送一次数据到队列 SensorMessageQueue
        if ((current_time - last_send_time) >= 200)
        {
            // 打包系统状态数据 (使用 static 以节省堆栈空间)
            static uint8_t packed_frame[256];
        	const uint8_t state = CR.state;
            const uint16_t frame_len = cmd_packer_pack_status_frame(
            	packed_frame,
				motor_ctx,  // 修复：传递整个数组，而不是单个元素
				global_sensor,
				&CR,
				state
			);
            
            // 发送到队列
            for (int i = 0; i < frame_len; i++)
            {
                osMessageQueuePut(SensorMessageQueueHandle, &packed_frame[i], 0, 0);
            }
            last_send_time = current_time;
        }

    	/* ===================================================
		 * 3. 从发送队列取出数据并通过 USART2 发出
		 * =================================================== */
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
