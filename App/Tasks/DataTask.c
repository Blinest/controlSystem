/**
 * @file DataTask.c
 * @brief 数据任务 — 每 200ms 打包电机状态经 USART1 发送给上位机
 */
#include "cmsis_os2.h"
#include "FreeRTOS.h"
#include "task.h"
#include "Motor/Motor.h"
#include "Common/cmd_packer.h"
#include "usart.h"

#define DATA_BUF_SIZE 256

void StartDataTask(void *argument)
{
    static uint32_t last_send_time = 0;

    for (;;)
    {
        uint32_t current_time = osKernelGetTickCount();
        if (current_time - last_send_time >= 200)
        {
            static uint8_t packed_frame[DATA_BUF_SIZE];
            uint8_t state = 1;  /* 正常工作状态 */
            uint16_t frame_len = cmd_packer_pack_status_frame(packed_frame, global_motor, state);

            /* 通过 USART2 发送给上位机 */
            Usart_SendString(&huart2, packed_frame, frame_len);

            last_send_time = current_time;
        }

        osDelay(10);
    }
}
