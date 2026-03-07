/**
 * @file DataTask.c
 * @brief 电机数据处理模块
 *
 *
 * 任务流程：从CmdDataQueue获取串口1数据 → 处理数据 → 通过USART2发送数据给上位机
 */
#include "cmsis_os.h"
#include "usart.h"
#include "Common/cmd_parse.h"
void StartDataTask(void *argument)
{
    uint8_t receive;
    for(;;)
    {
        // 测试串口用
        // uint8_t test_msg[] = "usart2\r\n";
        // Usart_SendString(&huart2, test_msg, sizeof(test_msg) - 1);


        /* 取出CmdDataQueue队列中的控制指令
         * - 取到字节数据：进行指令解析
         * - 未取到：继续等待
        */
        if (osMessageQueueGet(CmdDataQueueHandle, &receive, 0, osWaitForever) == osOK)
        {
            Usart_SendString(&huart2, &receive, 1);
        	// 进入指令解析状态机，喂入一个字节数据
        	cmd_parse_feed_byte(receive);
        }
    }
}