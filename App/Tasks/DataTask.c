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
#include "Common/sensor_cmd_parser.h"

#include "CR/CR.h"
#include "Sensor/WT_IMU.h"
#include "Sensor/IMU.h"




#define RX_BUF_SIZE 256

void StartDataTask(void *argument)
{
    uint8_t rx_byte = 0;
    uint8_t tx_byte = 0;
    MotorContext *active_motor_ctx = NULL;   // 当前激活的电机上下文
    static uint8_t packed_frame[256];
    static uint8_t tx_buffer[256];
    static uint16_t tx_buffer_len = 0;
    static uint32_t last_send_time = 0;

    for (int i = 0; i < SENSOR_NUM; i++) {
        SensorContext_Init(&sensor_context[i]);
    }

    for (;;) {
        // ====================================
        // 1. 数据采集流: 从队列获取字节并分发到解析器
        // ====================================
        while (osMessageQueueGet(CmdDataQueueHandle, &rx_byte, NULL, 0) == osOK) {
            if (active_motor_ctx != NULL) {
                // 电机帧解析中
                X_V2_ParseResult res = X_V2_SerialParser_Feed(
                    &active_motor_ctx->parser,
                    rx_byte,
                    &active_motor_ctx->global_motor,
                    false  // 先禁用校验验证，确保帧能被接受
                );
                // 根据解析结果处理
                if (res == X_V2_PARSE_OK) {
                    // 帧完成，电机数据已更新
                    active_motor_ctx = NULL;   // 帧结束，释放上下文
                } else if (res != X_V2_PARSE_INCOMPLETE) {
                    // 错误（地址不匹配、校验错、长度溢出等）
                    active_motor_ctx = NULL;
                }
                continue;
            }

            // 传感器解析器是否正在进行帧解析
            if (sensor_context[0].parser_mode != SENSOR_PARSER_MODE_NONE) {
                // 继续喂入当前传感器解析器
                if (sensor_context[0].parser_mode == SENSOR_PARSER_MODE_IMU) {
                    SensorParser_IMU_Feed(rx_byte, sensor_context, motor_ctx, &CR);
                } else {
                    CMCU_Parser_Feed(rx_byte, sensor_context, motor_ctx, &CR);
                }
                continue;
            }

            // 传感器空闲时，优先判断是否为电机地址
            MotorContext *ctx = Motor_GetContextByAddr(rx_byte);
            if (ctx != NULL) {
                active_motor_ctx = ctx;
                X_V2_SerialParser_Reset(&active_motor_ctx->parser);
            	
                // 当前字节是电机帧地址，立即作为第一字节喂入解析器
            	X_V2_SerialParser_Feed(&active_motor_ctx->parser,
					   rx_byte,
					   &active_motor_ctx->global_motor,
					   true);
            	continue;
            }

            // 非电机地址时，尝试启动传感器解析
            if (rx_byte == SENSOR_ID) {
                SensorParser_IMU_Feed(rx_byte, sensor_context, motor_ctx, &CR);
            } else if (rx_byte == CMCU_ADDR_DEFAULT) {
                CMCU_Parser_Feed(rx_byte, sensor_context, motor_ctx, &CR);
            }
        }

        // ====================================
        // 2. 数据发送流: 打包数据发送给上位机
        // ====================================
        uint32_t current_time = osKernelGetTickCount();
        if ((current_time - last_send_time) >= 200) {
            const uint8_t state = CR.state;
            const uint16_t frame_len = cmd_packer_pack_status_frame(
                packed_frame,
                motor_ctx,
                sensor_context,
                &CR,
                state
            );

            for (int i = 0; i < frame_len; i++) {
                osMessageQueuePut(SensorMessageQueueHandle, &packed_frame[i], 0, 0);
            }
            last_send_time = current_time;
        }

        /* ===================================================
         * 3. 从发送队列取出数据并通过 USART2 发出
         * =================================================== */
        tx_buffer_len = 0;
        while (osMessageQueueGet(SensorMessageQueueHandle, &tx_byte, NULL, 0) == osOK && tx_buffer_len < sizeof(tx_buffer)) {
            tx_buffer[tx_buffer_len++] = tx_byte;
        }
        if (tx_buffer_len > 0) {
            Usart_SendString(&huart2, tx_buffer, tx_buffer_len);
        }
        osDelay(10); // 增加延时，降低 CPU 占用并给串口发送留出时间
    }
}
