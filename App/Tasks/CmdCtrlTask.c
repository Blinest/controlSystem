/**
*  @file CmdCtrlTask.c
 * @brief 指令控制任务：负责 USART2 RX (上位机指令) 和 USART1 TX (发送至外设)
 *
 * 任务流程：
 * 1. 监听 CmdCtrlQueue (USART2 RX)，解析上位机指令。
 * 2. 验证校验和后，执行系统状态切换或转发至外设 (USART1 TX)。
 */

#include "cmsis_os.h"
#include "cmsis_os2.h"
#include "main.h"
#include "usart.h"
#include "Common/cmd_parse.h"
#include "string.h"
#define RX_BUF_SIZE 256
uint8_t rx_buffer[RX_BUF_SIZE];
uint16_t rx_len = 0;


float motor_pos[MOTOR_NUM][3] = {0};
float sensor_angle[SENSOR_NUM][3] = {0};
float scale_val = 100.0f;

uint8_t sys_state = 0;       // 0:未启动  1:已启动
bool is_connected = false;   // false:未连接 true:已连接

void StartCmdCtrlTask(void *argument)
{
	uint8_t rx_byte;

	// 初始化所有数组为 0.0
	memset(motor_pos, 0, sizeof(motor_pos));
	memset(sensor_angle, 0, sizeof(sensor_angle));
    
    for (;;)
    {	
        // ====================================
	    // 从队列倾听上位机指令 (USART2 RX)
	    // ====================================
	    while (osMessageQueueGet(CmdCtrlQueueHandle, &rx_byte, NULL, 0) == osOK) {
            // 喂入 PC 控制指令解析状态机，内部会自动处理转发 (USART1 TX) 和状态更新
            cmd_parse_feed_byte(rx_byte);
	    }

	    // 每次循环延时 2 毫秒，释放 CPU 控制权给其他任务
	    osDelay(2);
	  }
}

        // // 取出从串口中断2中得到的数据
        // if (osMessageQueueGet(CmdCtrlQueueHandle, &receive, 0, osWaitForever) == osOK)
        // {
        //     Usart_SendString(&huart1, &receive, 1);
        //     // 进入指令解析函数
        // 	cmd_parse_feed_byte(receive);
        // }
}