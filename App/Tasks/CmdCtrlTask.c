/**
 * @file CmdCtrlTask.c
 * @brief 指令控制任务：字节入 pc_cmd_parser + 50Hz 舵机控制
 */

#include "cmsis_os.h"
#include "cmsis_os2.h"
#include "main.h"
#include "LYZ/LYZ.h"
#include "usart.h"

#include "Common/pc_cmd_parser.h"
#include "Motor/Motor.h"

void StartCmdCtrlTask(void *argument)
{
    uint8_t byte;
	static uint8_t tick;
	/* 50Hz 舵机控制节拍 */
	static uint32_t last_tick_servo = 0;
	static uint32_t last_send_time_motor = 0;
    for (;;)
    {
        while (osMessageQueueGet(CmdCtrlQueueHandle, &byte, NULL, 0) == osOK)
        {
            pc_cmd_parser_feed_byte(byte);
        }

        uint32_t now = osKernelGetTickCount();
        if (now - last_tick_servo >= CONTROL_PERIOD_MS)
        {
            motor_control_step();
            last_tick_servo = now;
        }

    	// if (now - last_send_time_motor >= CONTROL_PERIOD_MS)
    	// {
    	// 	if ((LYZ.current_S >= 15 || LYZ.current_S <= 5) && tick < 10)
    	// 	{
    	// 		motor_run_AQ_abs(2,0, 2000, LYZ.current_S);
    	// 		tick++;
    	// 	}
    	// 	else if (LYZ.current_S > 5 && LYZ.current_S < 15)
    	// 	{
    	// 		tick = 0;
    	// 	}
    	// 	last_send_time_motor = now;
    	// }
        osDelay(5);
    }
}
