/**
 * @file CmdCtrlTask.c
 * @brief 指令控制任务：字节入 pc_cmd_parser + 50Hz 舵机控制
 */

#include "cmsis_os.h"
#include "cmsis_os2.h"
#include "main.h"

#include "Common/pc_cmd_parser.h"
#include "Motor/Motor.h"
#include "SW/SW.h"

void StartCmdCtrlTask(void *argument)
{
    uint8_t byte;

    for (;;)
    {
        if (osMessageQueueGet(CmdCtrlQueueHandle, &byte, NULL, 0) == osOK)
        {
            pc_cmd_parser_feed_byte(byte);
        }

        /* 50Hz 舵机控制节拍 */
        static uint32_t last_tick = 0;
        uint32_t now = osKernelGetTickCount();
        if (now - last_tick >= CONTROL_PERIOD_MS)
        {
            motor_control_step();
            SW_action_group_control();
            last_tick = now;
        }

        osDelay(5);
    }
}
