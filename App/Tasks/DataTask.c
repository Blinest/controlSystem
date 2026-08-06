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
            cmd_packer_send_status_frame();
        	motor_status_check();
            last_send_time = current_time;
        }

        osDelay(10);
    }
}
